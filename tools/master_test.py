#!/usr/bin/env python3
"""
ChargerPowerV4 - Master RS485 de teste (CLI interativa)

Corre no lado do Master (ex.: Raspberry Pi ligado por RS485 ao carregador)
para testar o firmware em ambiente real: ver o estado do CP em tempo real,
dar ordem de iniciar/parar sessao, ajustar a corrente, e consultar
falhas/medicoes.

Protocolo (ver Core/Inc/app_manager.h no firmware):
    [0xAA][LEN][DEST_ID][CMD][DATA 0..10][CRC]
    CRC = soma aditiva (mod 256) de STARTBYTE+LEN+DEST+CMD+DATA
    ID deste script (Master) = 0x04 -- e o dest_ID que o carregador usa
    ID do carregador          = 0x01 -- e o dest_ID que enviamos nos pedidos

Hardware (Raspberry Pi / CM4):
    UART:         /dev/serial0 (UART0 / PL011) @ 115200 8N1
                  GPIO14 = UART0_TX, GPIO15 = UART0_RX (footprint KiCad)
    RS-485 DE/RE: GPIO16 (BCM) = RS_CTRL_PIN_UART0, HIGH = transmite, LOW = receciona
                  -- controlado por software a volta de cada envio, porque
                  este adaptador nao faz a direcao automaticamente.

Requisitos: pyserial, lgpio
    pip install pyserial lgpio
    (ou: sudo apt install python3-lgpio)

Uso:
    python3 master_test.py
    python3 master_test.py --port /dev/serial0 --gpio-de 16
"""

import sys
import time
import queue
import argparse
import threading

try:
    import serial
except ImportError:
    print("Falta o modulo pyserial. Instala com: pip install pyserial")
    sys.exit(1)

try:
    import lgpio
except ImportError:
    print("Falta o modulo lgpio. Instala com: pip install lgpio (ou: sudo apt install python3-lgpio)")
    sys.exit(1)


# --------------------------------------------------------------------------
# Protocolo
# --------------------------------------------------------------------------
STARTBYTE = 0xAA
ID_CHARGER = 0x01   # dest_ID a usar quando ENVIAMOS para o carregador
ID_MASTER = 0x04    # dest_ID que o carregador usa quando nos endereca

CMD_AUTH_TRUE = 0x01
CMD_AUTH_FALSE = 0x05

CMD_SESSION_START = 0x11
CMD_SESSION_STOP = 0x12
CMD_SESSION_STATUS = 0x13
CMD_SESSION_INFO = 0x14
CMD_STATE_NOTIFY = 0x15

CMD_RESET = 0x20

CMD_REPORT = 0x30
CMD_GET_VOLTAGE = 0x31
CMD_GET_CURRENT = 0x32
CMD_GET_POWER = 0x33
CMD_GET_ENERGY = 0x34
CMD_GET_TEMP = 0x35
CMD_GET_METER_ALL = 0x36
CMD_GET_FREQ = 0x37

CMD_SET_STATE = 0x40
CMD_GET_CP_STATE = 0x41
CMD_SET_CURRENT = 0x45

CMD_GET_FAULTS = 0x50
CMD_CLEAR_FAULTS = 0x51
CMD_GET_DIAG = 0x52

CMD_RELAY_SET = 0x60
CMD_RELAY_GET = 0x61
CMD_LOCK_SET = 0x62
CMD_LOCK_GET = 0x63
CMD_RELAY_RESET = 0x64

CMD_HEARTBEAT = 0x70
CMD_ACK = 0xF0
CMD_NACK = 0xF1

CP_STATE_NAMES = {
    0: "A - livre / sem veiculo",
    1: "B - veiculo ligado, a aguardar",
    2: "C - a carregar (PWM ativo)",
    3: "D - a carregar c/ ventilacao",
    4: "E - curto CP-PE",
    5: "F - falha na linha CP",
    6: "desconhecido",
}

FAULT_BIT_NAMES = [
    "RCD (diferencial)",        # bit 0
    "SOBRECORRENTE",            # bit 1
    "SOBRETENSAO",              # bit 2
    "SUBTENSAO",                # bit 3
    "RELE",                     # bit 4
    "CONTROL PILOT",            # bit 5
    "COMUNICACAO",              # bit 6
    "CABO (PP)",                # bit 7
]

# Comandos que o proprio script emite e para os quais espera 1 resposta
_ACK_ONLY_CMDS = {
    CMD_AUTH_TRUE, CMD_AUTH_FALSE, CMD_SESSION_START, CMD_SESSION_STOP,
    CMD_SET_CURRENT, CMD_RELAY_SET, CMD_RELAY_RESET, CMD_HEARTBEAT,
    CMD_CLEAR_FAULTS,
}


def calc_crc(payload_bytes):
    """Soma aditiva mod 256 -- igual ao Calculate_CRC() do firmware."""
    return sum(payload_bytes) & 0xFF


def build_frame(dest_id, cmd, data=b""):
    header = bytes([STARTBYTE, len(data), dest_id, cmd]) + bytes(data)
    return header + bytes([calc_crc(header)])


def fmt_faults(active_faults, relay_errors):
    if active_faults == 0:
        faults_str = "nenhuma"
    else:
        faults_str = ", ".join(
            name for i, name in enumerate(FAULT_BIT_NAMES) if active_faults & (1 << i)
        )
    return f"active_faults=0x{active_faults:02X} ({faults_str})  relay_errors=0x{relay_errors:02X}"


# --------------------------------------------------------------------------
# Camada de transporte: thread de leitura + parser de tramas
# --------------------------------------------------------------------------
class RS485Link:
    # Guard time after the last bit physically leaves the wire before
    # dropping DE back to receive -- avoids clipping the stop bit.
    _DE_TURNAROUND_S = 0.002

    def __init__(self, port, baud, gpio_chip, de_pin):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.rx_buf = bytearray()
        self.responses = queue.Queue()
        self.stop_flag = threading.Event()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.on_state_notify = None  # callback(cp_state:int)
        self.verbose_raw = False

        self.de_pin = de_pin
        self.gpio_h = lgpio.gpiochip_open(gpio_chip)
        # Starts LOW (receive) -- matches the firmware's idle bus state.
        lgpio.gpio_claim_output(self.gpio_h, self.de_pin, 0)

    def start(self):
        self.reader_thread.start()

    def stop(self):
        self.stop_flag.set()
        self.reader_thread.join(timeout=1)
        self.ser.close()
        lgpio.gpio_free(self.gpio_h, self.de_pin)
        lgpio.gpiochip_close(self.gpio_h)

    def send(self, cmd, data=b""):
        frame = build_frame(ID_CHARGER, cmd, data)
        if self.verbose_raw:
            print(f"[TX] {frame.hex(' ')}")
        lgpio.gpio_write(self.gpio_h, self.de_pin, 1)  # transmit
        try:
            self.ser.write(frame)
            self.ser.flush()  # tcdrain() -- blocks until physically transmitted
            time.sleep(self._DE_TURNAROUND_S)
        finally:
            lgpio.gpio_write(self.gpio_h, self.de_pin, 0)  # back to receive

    def _reader_loop(self):
        while not self.stop_flag.is_set():
            try:
                chunk = self.ser.read(64)
            except serial.SerialException as exc:
                print(f"\n[ERRO SERIE] {exc}")
                break
            if chunk:
                self.rx_buf.extend(chunk)
                self._parse_buffer()

    def _parse_buffer(self):
        buf = self.rx_buf
        i = 0
        while i < len(buf):
            if buf[i] != STARTBYTE:
                i += 1
                continue
            if len(buf) - i < 4:
                break  # cabecalho incompleto, espera mais bytes
            length = buf[i + 1]
            dest_id = buf[i + 2]
            cmd = buf[i + 3]
            total = 4 + length + 1
            if total > (10 + 5):
                i += 1  # tamanho absurdo -> 0xAA era so dados, avanca 1 byte
                continue
            if len(buf) - i < total:
                break  # trama incompleta, espera mais bytes
            data = bytes(buf[i + 4:i + 4 + length])
            rx_crc = buf[i + total - 1]
            calc = calc_crc(buf[i:i + total - 1])
            if calc == rx_crc:
                if self.verbose_raw:
                    print(f"[RX] {bytes(buf[i:i + total]).hex(' ')}")
                self._handle_frame(dest_id, cmd, data)
                i += total
            else:
                i += 1  # CRC falhou, o 0xAA era lixo, tenta o proximo
        del buf[:i]

    def _handle_frame(self, dest_id, cmd, data):
        if dest_id != ID_MASTER:
            return  # nao e para nos

        if cmd == CMD_STATE_NOTIFY and len(data) >= 1 and data[0] != CMD_ACK:
            cp_state = data[0]
            name = CP_STATE_NAMES.get(cp_state, f"0x{cp_state:02X}")
            print(f"\n>>> MUDANCA DE ESTADO CP: {name}")
            if self.on_state_notify:
                self.on_state_notify(cp_state)
            # Confirma ao firmware para nao ficar a retransmitir/repetir
            self.send(CMD_STATE_NOTIFY, bytes([CMD_ACK]))
            return

        self.responses.put((cmd, data))

    def request(self, cmd, data=b"", timeout=1.0):
        """Envia um comando e espera pela resposta correspondente (best-effort)."""
        # Drena respostas antigas nao consumidas para nao confundir com esta
        while not self.responses.empty():
            try:
                self.responses.get_nowait()
            except queue.Empty:
                break
        self.send(cmd, data)
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                rx_cmd, rx_data = self.responses.get(timeout=deadline - time.time())
            except queue.Empty:
                break
            if rx_cmd == cmd:
                return rx_data
            # resposta de outro comando entretanto -- ignora e continua a espera
        return None


# --------------------------------------------------------------------------
# Comandos de alto nivel
# --------------------------------------------------------------------------
def cmd_status(link):
    data = link.request(CMD_GET_CP_STATE)
    if data is None:
        print("Sem resposta (timeout). Confirma a ligacao RS485/alimentacao.")
        return
    cp_state = data[0]
    print(f"Estado CP: {CP_STATE_NAMES.get(cp_state, cp_state)}")

    data = link.request(CMD_GET_FAULTS)
    if data and len(data) >= 3:
        print(fmt_faults(data[1], data[2]))

    data = link.request(CMD_RELAY_GET)
    if data and len(data) >= 2:
        relay_state = data[1]
        print(f"Reles (K1..K4 fechados=bit1): 0x{relay_state:02X}")


def _decode_3line_u16(data, scale, unit):
    # data = [ACK][MSB1][LSB1][MSB2][LSB2][MSB3][LSB3]
    if not data or len(data) < 7:
        return None
    vals = []
    for i in range(3):
        raw = (data[1 + i * 2] << 8) | data[2 + i * 2]
        vals.append(raw * scale)
    return [f"L{i+1}={v:.1f}{unit}" for i, v in enumerate(vals)]


def cmd_meters(link):
    v = link.request(CMD_GET_VOLTAGE)
    if v:
        parts = _decode_3line_u16(v, 0.1, "V")
        print("Tensao: " + (", ".join(parts) if parts else "sem dados"))

    c = link.request(CMD_GET_CURRENT)
    if c:
        parts = _decode_3line_u16(c, 0.1, "A")
        print("Corrente: " + (", ".join(parts) if parts else "sem dados"))

    p = link.request(CMD_GET_POWER)
    if p:
        parts = _decode_3line_u16(p, 0.1, "kW")
        print("Potencia: " + (", ".join(parts) if parts else "sem dados"))


def cmd_faults(link):
    data = link.request(CMD_GET_FAULTS)
    if data and len(data) >= 3:
        print(fmt_faults(data[1], data[2]))
    else:
        print("Sem resposta.")


def cmd_clear_faults(link):
    data = link.request(CMD_CLEAR_FAULTS)
    if data:
        print("Falhas limpas." if data[0] == CMD_ACK else "NACK.")
    else:
        print("Sem resposta.")


def cmd_charge(link, amps):
    amps = max(6, min(33, amps))
    print(f"-> AUTH_TRUE")
    link.request(CMD_AUTH_TRUE)
    print(f"-> SESSION_START")
    link.request(CMD_SESSION_START)
    print(f"-> SET_CURRENT {amps}A")
    link.request(CMD_SET_CURRENT, bytes([amps]))
    print("Pedido de carregamento enviado. Usa 'status' para acompanhar.")


def cmd_stop(link):
    link.request(CMD_SESSION_STOP)
    link.request(CMD_AUTH_FALSE)
    print("Sessao terminada (SESSION_STOP + AUTH_FALSE enviados).")


def cmd_current(link, amps):
    amps = max(6, min(33, amps))
    data = link.request(CMD_SET_CURRENT, bytes([amps]))
    print(f"Corrente pedida: {amps}A -> {'ACK' if data and data[0]==CMD_ACK else 'sem confirmacao'}")


def cmd_heartbeat(link):
    data = link.request(CMD_HEARTBEAT)
    print("Vivo (ACK)" if data and data[0] == CMD_ACK else "Sem resposta")


HELP_TEXT = """
Comandos disponiveis:
  status              Estado CP + falhas + reles
  meters | m          Tensao / corrente / potencia das 3 linhas
  charge <A> | c <A>  Autoriza + inicia sessao + pede corrente <A> (6-33)
  current <A>         Ajusta so a corrente durante a sessao (6-33)
  stop                Termina a sessao (SESSION_STOP + AUTH_FALSE)
  faults              Mostra falhas ativas
  clear               Limpa falhas (CMD_CLEAR_FAULTS)
  ping                Heartbeat
  raw on|off          Mostra/esconde os bytes TX/RX crus
  help | h            Esta mensagem
  quit | q            Sai (Ctrl+C tambem funciona)

Notificacoes de mudanca de estado do CP aparecem sozinhas, a qualquer
momento, assinaladas com '>>> MUDANCA DE ESTADO CP: ...'.
"""


def main():
    parser = argparse.ArgumentParser(description="Master RS485 de teste para o ChargerPowerV4")
    parser.add_argument("--port", default="/dev/serial0", help="Porta serie (default /dev/serial0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--gpio-chip", type=int, default=0, help="gpiochip a usar (default 0)")
    parser.add_argument("--gpio-de", type=int, default=16, help="Pino BCM do DE/RE do RS485 (default 16, RS_CTRL_PIN_UART0)")
    args = parser.parse_args()

    try:
        link = RS485Link(args.port, args.baud, args.gpio_chip, args.gpio_de)
    except serial.SerialException as exc:
        print(f"Nao consegui abrir {args.port}: {exc}")
        sys.exit(1)
    except lgpio.error as exc:
        print(f"Nao consegui reservar o GPIO{args.gpio_de} (gpiochip{args.gpio_chip}): {exc}")
        print("Confirma que nao ha outro processo a usar o pino, e que corres com permissoes de GPIO.")
        sys.exit(1)

    link.start()
    print(f"Ligado a {args.port} @ {args.baud} 8N1, DE/RE em GPIO{args.gpio_de}. Escreve 'help' para ver os comandos.")
    print(HELP_TEXT)

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            op = parts[0].lower()

            if op in ("quit", "q", "exit"):
                break
            elif op in ("help", "h", "?"):
                print(HELP_TEXT)
            elif op == "status":
                cmd_status(link)
            elif op in ("meters", "m"):
                cmd_meters(link)
            elif op in ("charge", "c"):
                if len(parts) < 2:
                    print("Uso: charge <amps>")
                    continue
                cmd_charge(link, int(parts[1]))
            elif op == "current":
                if len(parts) < 2:
                    print("Uso: current <amps>")
                    continue
                cmd_current(link, int(parts[1]))
            elif op == "stop":
                cmd_stop(link)
            elif op == "faults":
                cmd_faults(link)
            elif op == "clear":
                cmd_clear_faults(link)
            elif op == "ping":
                cmd_heartbeat(link)
            elif op == "raw":
                if len(parts) >= 2 and parts[1].lower() == "on":
                    link.verbose_raw = True
                    print("Trama crua: ON")
                else:
                    link.verbose_raw = False
                    print("Trama crua: OFF")
            else:
                print(f"Comando desconhecido: {op!r}. Escreve 'help'.")
    except KeyboardInterrupt:
        pass
    finally:
        print("\nA fechar...")
        link.stop()


if __name__ == "__main__":
    main()
