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

Log:
    Cada execucao grava logs/master_AAAAMMDD_HHMMSS.log ao lado do script
    (ou em --log-dir), com timestamp ao milissegundo: comandos escritos,
    respostas, erros, mudancas de estado do CP e todas as tramas TX/RX cruas
    (mesmo com 'raw off'). 'note <texto>' marca um momento no log.

Requisitos: pyserial, lgpio
    pip install pyserial lgpio
    (ou: sudo apt install python3-lgpio)
    O readline (historico com as setas, Tab para completar) ja vem com o
    Python em Linux; se faltar, o script funciona na mesma sem isso.

Uso:
    python3 master_test.py
    python3 master_test.py --port /dev/serial0 --gpio-de 16
    python3 master_test.py --log-dir ~/logs --no-color
"""

import os
import sys
import time
import queue
import logging
import argparse
import threading
from collections import namedtuple
from datetime import datetime

try:
    import readline  # historico (setas) e Tab para completar -- opcional
except ImportError:
    readline = None

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
MAX_DATA_LEN = 10   # DATA_RS485 no firmware

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
CMD_GET_RCD_DIAG = 0x53

CMD_RELAY_SET = 0x60
CMD_RELAY_GET = 0x61
CMD_LOCK_SET = 0x62
CMD_LOCK_GET = 0x63
CMD_RELAY_RESET = 0x64
CMD_RELAY_MEAS_SET = 0x65

CMD_HEARTBEAT = 0x70
CMD_ACK = 0xF0
CMD_NACK = 0xF1

# Nome de cada comando, para o log e para o modo 'raw'
CMD_NAMES = {value: name for name, value in list(globals().items())
             if name.startswith("CMD_") and isinstance(value, int)}

CP_STATE_NAMES = {
    0: "A - livre / sem veiculo",
    1: "B - veiculo ligado, a aguardar",
    2: "C - a carregar (PWM ativo)",
    3: "D - a carregar c/ ventilacao",
    4: "E - curto CP-PE",
    5: "F - falha na linha CP",
    6: "desconhecido",
}

# STATE_MACHINE do firmware (app_manager.h) -- usado no CMD_GET_RCD_DIAG
FSM_STATE_NAMES = {
    0: "IDLE",
    1: "READY",
    2: "CHARGING",
    3: "CHARGING_COMPLETE",
    4: "FAULT_RCD",
    5: "FAULT_RELAY_CONTACT",
    6: "FAULT_GRID",
    7: "FAULT_CAR",
    8: "FAULT_CP_SHORT",
    9: "FAULT_CABLE",
}

# CMD_GET_RCD_DIAG / data[3] -- flags (RCD_DIAG_FLAG_* em app_manager.h)
RCD_DIAG_FLAG_ONGOING = 1 << 0   # o impulso do ultimo disparo ainda esta em LOW
RCD_DIAG_FLAG_RECORDED = 1 << 1  # houve pelo menos um disparo desde o arranque

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
FAULT_BIT_RCD = 1 << 0

# CMD_RELAY_GET / data_RX[1] -- estado ao vivo dos contactores (ver
# APP_Dispatch_Command() no firmware). K2/K3 so tem significado com
# SYSTEM_PHASES==3; em build monofasico ficam sempre a 0.
RELAY_NAMES = ["K1", "K2", "K3", "K4"]  # bits 0..3

# CMD_GET_FAULTS / data_RX[2] -- detalhe por contactor (ver
# APP_Verify_Relays_Closed/Open e RELAY_ERR_* em app_manager.h).
# Bits [3:0] = falhou fechar, bits [7:4] = soldado (nao abriu).
RELAY_ERROR_BIT_NAMES = [
    "K1 falhou fechar",        # bit 0
    "K2 falhou fechar",        # bit 1
    "K3 falhou fechar",        # bit 2
    "K4 falhou fechar",        # bit 3
    "K1 soldado (nao abriu)",  # bit 4
    "K2 soldado (nao abriu)",  # bit 5
    "K3 soldado (nao abriu)",  # bit 6
    "K4 soldado (nao abriu)",  # bit 7
]


def fmt_bitmask(value, names):
    """Decodifica um bitmask numa lista de nomes, um por bit definido."""
    return ", ".join(name for i, name in enumerate(names) if value & (1 << i))


def calc_crc(payload_bytes):
    """Soma aditiva mod 256 -- igual ao Calculate_CRC() do firmware."""
    return sum(payload_bytes) & 0xFF


def build_frame(dest_id, cmd, data=b""):
    header = bytes([STARTBYTE, len(data), dest_id, cmd]) + bytes(data)
    return header + bytes([calc_crc(header)])


def cmd_name(cmd):
    return CMD_NAMES.get(cmd, "0x{:02X}".format(cmd))


def cp_name(cp_state):
    return CP_STATE_NAMES.get(cp_state, "0x{:02X}".format(cp_state))


def cp_style(cp_state):
    """Cor de cada estado do CP no ecra."""
    return {1: ("yellow",), 2: ("green", "bold"), 3: ("green", "bold"),
            4: ("red", "bold"), 5: ("red", "bold"), 6: ("yellow",)}.get(cp_state, ())


# --------------------------------------------------------------------------
# Ecra + log
# --------------------------------------------------------------------------
def setup_logging(log_dir):
    """Um ficheiro por execucao. Devolve (logger, caminho); caminho = None se
    nao der para escrever na pasta (o script continua, so sem log)."""
    log = logging.getLogger("master")
    log.setLevel(logging.DEBUG)
    log.propagate = False
    try:
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(log_dir, datetime.now().strftime("master_%Y%m%d_%H%M%S.log"))
        handler = logging.FileHandler(path, encoding="utf-8")
    except OSError as exc:
        print("Aviso: sem log -- nao consegui escrever em {}: {}".format(log_dir, exc))
        log.addHandler(logging.NullHandler())
        return log, None
    handler.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
    log.addHandler(handler)
    return log, path


class UI:
    """Tudo o que aparece no ecra passa por aqui, e fica tambem no log.

    Thread-safe: a thread de leitura do RS485 tambem escreve (mudancas de
    estado do CP, tramas em modo 'raw'). Se chegar uma mensagem enquanto o
    utilizador esta a meio de escrever um comando, a mensagem aparece por
    cima e o prompt, com o que ja estava escrito, e redesenhado por baixo."""

    _ANSI = {"bold": "1", "dim": "2", "red": "31", "green": "32",
             "yellow": "33", "magenta": "35", "cyan": "36"}
    KEY_WIDTH = 12
    COL_WIDTH = 11

    def __init__(self, log, color):
        self.log = log
        self.color = color
        self.lock = threading.RLock()
        self.at_prompt = False
        if (sys.stdout.encoding or "").lower().replace("-", "") == "utf8":
            self.sym = {"ok": "✔", "err": "✖", "warn": "⚠",
                        "info": "•", "event": "»", "rule": "─"}
        else:
            self.sym = {"ok": "+", "err": "x", "warn": "!",
                        "info": "-", "event": ">>", "rule": "-"}
        if color:
            # \001/\002 marcam o que nao ocupa espaco no ecra, para o
            # readline calcular bem a largura do prompt
            self.prompt = "\001\033[1;36m\002charger>\001\033[0m\002 "
        else:
            self.prompt = "charger> "

    def style(self, text, *styles):
        if not self.color or not styles:
            return str(text)
        return "\033[" + ";".join(self._ANSI[s] for s in styles) + "m" + str(text) + "\033[0m"

    def _write(self, text):
        with self.lock:
            if self.at_prompt:
                typed = readline.get_line_buffer() if readline else ""
                clear = "\r\033[K" if self.color else "\n"
                sys.stdout.write(clear + text + "\n" + self.prompt + typed)
            else:
                sys.stdout.write(text + "\n")
            sys.stdout.flush()

    def _emit(self, screen, plain, level=logging.INFO):
        self._write(screen)
        if plain is not None:
            self.log.log(level, plain)

    # --- mensagens -------------------------------------------------------
    def ok(self, msg):
        self._emit("  " + self.style(self.sym["ok"], "green", "bold") + " " + msg, msg)

    def err(self, msg):
        self._emit("  " + self.style(self.sym["err"] + " " + msg, "red", "bold"), msg, logging.ERROR)

    def warn(self, msg):
        self._emit("  " + self.style(self.sym["warn"] + " " + msg, "yellow"), msg, logging.WARNING)

    def info(self, msg):
        self._emit("  " + self.style(self.sym["info"] + " " + msg, "dim"), msg)

    def section(self, title):
        rule = self.sym["rule"]
        bar = rule * max(4, 46 - len(title))
        self._emit("\n" + self.style(rule * 2 + " " + title + " " + bar, "cyan", "bold"),
                   "== " + title + " ==")

    def kv(self, key, value, *styles, plain=None):
        """Linha 'chave  valor', alinhada. plain = texto para o log quando
        o valor ja vem com cores."""
        self._emit("  " + self.style(key.ljust(self.KEY_WIDTH), "dim") + self.style(value, *styles),
                   key + ": " + (plain if plain is not None else str(value)))

    def table_header(self, cols):
        self._emit("  " + " " * self.KEY_WIDTH
                   + "".join(self.style(c.rjust(self.COL_WIDTH), "dim") for c in cols), None)

    def table_row(self, label, cells):
        head = "  " + self.style(label.ljust(self.KEY_WIDTH), "dim")
        if cells is None:
            self._emit(head + self.style("sem resposta", "yellow"),
                       label + ": sem resposta", logging.WARNING)
        else:
            self._emit(head + "".join(c.rjust(self.COL_WIDTH) for c in cells),
                       label + ": " + " | ".join(cells))

    def event(self, msg, *styles):
        stamp = datetime.now().strftime("%H:%M:%S")
        self._emit(self.style(stamp, "dim") + " "
                   + self.style(self.sym["event"] + " " + msg, *(styles or ("magenta", "bold"))),
                   "EVENTO " + msg)

    def raw(self, direction, name, frame):
        # So ecra -- o RS485Link ja grava todas as tramas no log
        self._emit(self.style("  [{}] {:<20} {}".format(direction, name, frame.hex(" ")), "dim"), None)

    def plain(self, text=""):
        self._emit(text, None)

    def read_command(self):
        with self.lock:
            self.at_prompt = True
        try:
            return input(self.prompt).strip()
        finally:
            with self.lock:
                self.at_prompt = False


# --------------------------------------------------------------------------
# Camada de transporte: thread de leitura + parser de tramas
# --------------------------------------------------------------------------
class RS485Link:
    # Guard time after the last bit physically leaves the wire before
    # dropping DE back to receive -- avoids clipping the stop bit.
    _DE_TURNAROUND_S = 0.002

    def __init__(self, port, baud, gpio_chip, de_pin, log):
        self.log = log
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.rx_buf = bytearray()
        self.responses = queue.Queue()
        # send() toggles the DE pin and writes the serial port -- it's called
        # both from the main thread (user commands) and from _reader_loop
        # (auto-ACK on CMD_STATE_NOTIFY). Without this lock, two overlapping
        # sends interleave on the wire and corrupt each other's frame.
        self.send_lock = threading.Lock()
        self.stop_flag = threading.Event()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.on_state_notify = None  # callback(cp_state, cable_amps ou None)
        self.on_raw = None           # callback(direcao, nome, trama) -- modo 'raw'
        self.on_error = None         # callback(mensagem) -- erros na thread de leitura

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

    def _trace(self, direction, cmd, frame):
        name = cmd_name(cmd)
        self.log.debug("%s %-20s %s", direction, name, frame.hex(" "))
        if self.on_raw:
            self.on_raw(direction, name, frame)

    def send(self, cmd, data=b""):
        frame = build_frame(ID_CHARGER, cmd, data)
        self._trace("TX", cmd, frame)
        with self.send_lock:
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
                msg = "Erro na porta serie: {}".format(exc)
                if self.on_error:
                    self.on_error(msg)
                else:
                    self.log.error(msg)
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
            if total > (MAX_DATA_LEN + 5):
                i += 1  # tamanho absurdo -> 0xAA era so dados, avanca 1 byte
                continue
            if len(buf) - i < total:
                break  # trama incompleta, espera mais bytes
            data = bytes(buf[i + 4:i + 4 + length])
            rx_crc = buf[i + total - 1]
            calc = calc_crc(buf[i:i + total - 1])
            if calc == rx_crc:
                self._trace("RX", cmd, bytes(buf[i:i + total]))
                self._handle_frame(dest_id, cmd, data)
                i += total
            else:
                self.log.debug("RX CRC invalido, descartado: %s", bytes(buf[i:i + total]).hex(" "))
                i += 1  # CRC falhou, o 0xAA era lixo, tenta o proximo
        del buf[:i]

    def _handle_frame(self, dest_id, cmd, data):
        if dest_id != ID_MASTER:
            self.log.debug("RX para outro destino (0x%02X), ignorado", dest_id)
            return

        if cmd == CMD_STATE_NOTIFY and len(data) >= 1 and data[0] != CMD_ACK:
            cp_state = data[0]
            # data[1] = corrente maxima do cabo (PP), em Amps -- 0 = desconhecida/sem cabo.
            # So presente em firmware com ENABLE_PP_SENSE; len(data)==1 em firmware antigo.
            cable_amps = data[1] if len(data) >= 2 else None
            if self.on_state_notify:
                self.on_state_notify(cp_state, cable_amps)
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
# Comandos de alto nivel -- cada um recebe (ctx, args)
# --------------------------------------------------------------------------
class Context:
    """O que cada comando precisa: ecra, ligacao, log e o modo 'raw'."""

    def __init__(self, ui, link, log):
        self.ui = ui
        self.link = link
        self.log = log
        self.show_raw = False
        self.last_uptime_s = None  # do ultimo 'diag', para detetar resets do MCU


class UsageError(Exception):
    """Argumentos invalidos -- a mensagem e a linha de uso do comando."""


def _parse_amps(ui, args, usage):
    if len(args) != 1:
        raise UsageError(usage)
    try:
        amps = int(args[0])
    except ValueError:
        raise UsageError(usage)
    clamped = max(6, min(33, amps))
    if clamped != amps:
        ui.warn("{} A fora do intervalo 6-33 A -- a usar {} A".format(amps, clamped))
    return clamped


def _parse_on_off(args, usage):
    if len(args) != 1 or args[0].lower() not in ("on", "off"):
        raise UsageError(usage)
    return args[0].lower() == "on"


def _expect_ack(ctx, label, cmd, data=b"", nack_hint=""):
    """Envia um comando que so responde ACK/NACK e mostra o resultado.
    label = a acao (ex.: 'Fechar contactores')."""
    reply = ctx.link.request(cmd, data)
    if reply is None:
        ctx.ui.err(label + ": sem resposta (timeout)")
        return False
    if reply and reply[0] == CMD_ACK:
        ctx.ui.ok(label + " (ACK)")
        return True
    if reply and reply[0] == CMD_NACK:
        ctx.ui.err(label + ": recusado (NACK)" + nack_hint)
        return False
    ctx.ui.warn(label + ": resposta inesperada " + (reply.hex(" ") or "(vazia)"))
    return False


def _decode_3line_u16(data, scale):
    # data = [ACK][MSB1][LSB1][MSB2][LSB2][MSB3][LSB3]
    if not data or len(data) < 7:
        return None
    return [((data[1 + i * 2] << 8) | data[2 + i * 2]) * scale for i in range(3)]


def show_faults(ui, active_faults, relay_errors):
    if active_faults:
        ui.kv("Falhas", "{} (0x{:02X})".format(fmt_bitmask(active_faults, FAULT_BIT_NAMES), active_faults),
              "red", "bold")
    else:
        ui.kv("Falhas", "nenhuma", "green")
    if relay_errors:
        ui.kv("Erros rele", "{} (0x{:02X})".format(fmt_bitmask(relay_errors, RELAY_ERROR_BIT_NAMES), relay_errors),
              "red")
    else:
        ui.kv("Erros rele", "nenhum", "dim")


def show_relays(ui, relay_state):
    shown, plain = [], []
    for i, name in enumerate(RELAY_NAMES):
        closed = bool(relay_state & (1 << i))
        text = name + (" fechado" if closed else " aberto")
        shown.append(ui.style(text, "bold") if closed else ui.style(text, "dim"))
        plain.append(text)
    ui.kv("Reles", "   ".join(shown), plain=", ".join(plain))


def cmd_status(ctx, args):
    ui, link = ctx.ui, ctx.link
    ui.section("Estado")
    data = link.request(CMD_GET_CP_STATE)
    if not data:
        ui.err("Sem resposta (timeout). Confirma a ligacao RS485/alimentacao.")
        return
    ui.kv("CP", cp_name(data[0]), *cp_style(data[0]))

    rcd_active = False
    data = link.request(CMD_GET_FAULTS)
    if data and len(data) >= 3:
        show_faults(ui, data[1], data[2])
        rcd_active = bool(data[1] & FAULT_BIT_RCD)
    else:
        ui.warn("Falhas: sem resposta")

    data = link.request(CMD_RELAY_GET)
    if data and len(data) >= 2:
        show_relays(ui, data[1])
    else:
        ui.warn("Reles: sem resposta")

    if rcd_active:
        cmd_rcd(ctx, [])


def cmd_meters(ctx, args):
    ui = ctx.ui
    ui.section("Medicoes")
    ui.table_header(["L1", "L2", "L3"])
    for label, cmd, unit in (("Tensao", CMD_GET_VOLTAGE, "V"),
                             ("Corrente", CMD_GET_CURRENT, "A"),
                             ("Potencia", CMD_GET_POWER, "kW")):
        vals = _decode_3line_u16(ctx.link.request(cmd), 0.1)
        ui.table_row(label, None if vals is None else ["{:.1f} {}".format(v, unit) for v in vals])


def cmd_faults(ctx, args):
    ctx.ui.section("Falhas")
    data = ctx.link.request(CMD_GET_FAULTS)
    if data and len(data) >= 3:
        show_faults(ctx.ui, data[1], data[2])
    else:
        ctx.ui.err("Sem resposta (timeout).")


def cmd_rcd(ctx, args):
    """CMD_GET_RCD_DIAG -- historico da linha do RCD (PC13) desde o arranque
    do firmware: largura do ultimo disparo confirmado, impulsos curtos
    rejeitados, e o que o carregador estava a fazer no momento do disparo.
    Os contadores nao sao zerados pelo 'clear'."""
    ui = ctx.ui
    ui.section("RCD")
    data = ctx.link.request(CMD_GET_RCD_DIAG)
    if not data or len(data) < 10:
        ui.err("Sem resposta (firmware sem CMD_GET_RCD_DIAG?)")
        return
    width_ms = (data[1] << 8) | data[2]
    flags = data[3]
    rejected = (data[4] << 8) | data[5]
    rejected_max_ms = data[6]
    current_a = ((data[7] << 8) | data[8]) * 0.1
    state = data[9]

    if flags & RCD_DIAG_FLAG_RECORDED:
        if flags & RCD_DIAG_FLAG_ONGOING:
            ui.kv("Disparo", "saida AINDA em LOW, ha {} ms".format(width_ms), "red", "bold")
        else:
            ui.kv("Disparo", "impulso LOW de {} ms".format(width_ms), "red", "bold")
        ui.kv("No disparo", "{}, corrente L1 = {:.1f} A".format(FSM_STATE_NAMES.get(state, state), current_a))
    else:
        ui.kv("Disparo", "nenhum desde o arranque", "green")

    if rejected:
        max_str = "{} ms".format(rejected_max_ms) if rejected_max_ms < 255 else ">= 255 ms"
        ui.kv("Rejeitados", "{} (o maior com {})".format(rejected, max_str), "yellow")
    else:
        ui.kv("Rejeitados", "0", "dim")

    if flags & RCD_DIAG_FLAG_RECORDED and flags & RCD_DIAG_FLAG_ONGOING:
        ui.info("Pede 'rcd' outra vez: se a largura continuar a crescer, o modulo tranca a saida.")


def cmd_diag(ctx, args):
    """CMD_GET_DIAG -- uptime do carregador (desde o ultimo reset do MCU) e
    contadores de saude do RS485 desde o arranque. Cada erro de UART fazia o
    carregador deixar de ouvir o Master ate um reset; agora a rececao e
    reiniciada sozinha e fica contada aqui."""
    ui = ctx.ui
    ui.section("Diagnostico")
    data = ctx.link.request(CMD_GET_DIAG)
    if not data or len(data) < 10:
        ui.err("Sem resposta (firmware sem CMD_GET_DIAG?)")
        return
    uptime_s = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]
    uart_errors = (data[5] << 8) | data[6]
    rx_restarts = (data[7] << 8) | data[8]
    tx_recoveries = data[9]

    hours, rest = divmod(uptime_s, 3600)
    minutes, seconds = divmod(rest, 60)
    ui.kv("Uptime", "{}h {:02d}m {:02d}s".format(hours, minutes, seconds))
    if ctx.last_uptime_s is not None and uptime_s < ctx.last_uptime_s:
        ui.warn("O carregador reiniciou desde o ultimo 'diag' (uptime voltou atras)")
    ctx.last_uptime_s = uptime_s

    ui.kv("Erros UART", str(uart_errors), "yellow" if uart_errors else "dim")
    ui.kv("RX reposto", str(rx_restarts), "yellow" if rx_restarts else "dim")
    ui.kv("TX reposto", str(tx_recoveries), "yellow" if tx_recoveries else "dim")
    if uart_errors or rx_restarts or tx_recoveries:
        ui.info("Houve problemas no RS485, mas o carregador recuperou sozinho "
                "(antes, cada erro de UART obrigava a um reset).")


def cmd_relays(ctx, args):
    """CMD_RELAY_GET isolado -- util para ver, no momento de um FAULT_BIT_RELAY,
    qual contactor especifico (K1..K4) esta a ler fechado."""
    ctx.ui.section("Reles")
    data = ctx.link.request(CMD_RELAY_GET)
    if data and len(data) >= 2:
        show_relays(ctx.ui, data[1])
    else:
        ctx.ui.err("Sem resposta (timeout).")


def cmd_relay_raw(ctx, args):
    """CMD_RELAY_SET / CMD_RELAY_RESET -- fecha/abre os contactores
    DIRETAMENTE, sem passar pela state machine (sem lock do cabo, sem
    pre-charge weld check, sem verificar se fecharam mesmo). E' o comando de
    teste de bancada do firmware ('activation here only for test purposes'
    em app_manager.c) -- nao usar com um veiculo real ligado."""
    if _parse_on_off(args, "relay on|off"):
        ctx.ui.warn("A fechar os contactores DIRETAMENTE (sem lock do cabo nem verificacoes de "
                    "seguranca). So para teste de bancada, sem carro ligado.")
        _expect_ack(ctx, "Fechar contactores", CMD_RELAY_SET,
                    nack_hint=" -- provavelmente FAULT_BIT_RCD ativo, ve 'faults'")
    else:
        _expect_ack(ctx, "Abrir contactores", CMD_RELAY_RESET)


def cmd_relay_meas(ctx, args):
    """CMD_RELAY_MEAS_SET -- forca os circuitos de medicao do K1/K4 a ficarem
    sempre ativos (em vez de so um pulso breve durante cada verificacao),
    para poder sondar RELAY_STATE_1/4 (PB1/PB0) com multimetro/osciloscopio
    a vontade. So para diagnostico de bancada."""
    force_on = _parse_on_off(args, "meas on|off")
    label = ("Forcar circuito de medicao K1/K4 sempre ativo" if force_on
             else "Libertar circuito de medicao K1/K4 (volta ao normal)")
    if _expect_ack(ctx, label, CMD_RELAY_MEAS_SET, bytes([1 if force_on else 0])) and force_on:
        ctx.ui.info("Lembra-te de 'meas off' no fim, para nao ficar ligado sem necessidade.")


def cmd_clear(ctx, args):
    _expect_ack(ctx, "Limpar falhas", CMD_CLEAR_FAULTS)


def cmd_charge(ctx, args):
    amps = _parse_amps(ctx.ui, args, "charge <A>   (6-33)")
    ctx.ui.section("Carregamento a {} A".format(amps))
    _expect_ack(ctx, "AUTH_TRUE", CMD_AUTH_TRUE)
    _expect_ack(ctx, "SESSION_START", CMD_SESSION_START)
    _expect_ack(ctx, "SET_CURRENT {} A".format(amps), CMD_SET_CURRENT, bytes([amps]))
    ctx.ui.info("Pedido enviado. Usa 'status' para acompanhar.")


def cmd_current(ctx, args):
    amps = _parse_amps(ctx.ui, args, "current <A>   (6-33)")
    _expect_ack(ctx, "SET_CURRENT {} A".format(amps), CMD_SET_CURRENT, bytes([amps]))


def cmd_stop(ctx, args):
    ctx.ui.section("Fim de sessao")
    _expect_ack(ctx, "SESSION_STOP", CMD_SESSION_STOP)
    _expect_ack(ctx, "AUTH_FALSE", CMD_AUTH_FALSE)


def cmd_ping(ctx, args):
    t0 = time.monotonic()
    reply = ctx.link.request(CMD_HEARTBEAT)
    dt_ms = (time.monotonic() - t0) * 1000
    if reply and reply[0] == CMD_ACK:
        ctx.ui.ok("Carregador vivo (ACK em {:.0f} ms)".format(dt_ms))
    else:
        ctx.ui.err("Sem resposta ao heartbeat. Confirma a ligacao RS485/alimentacao.")


def cmd_raw(ctx, args):
    ctx.show_raw = _parse_on_off(args, "raw on|off")
    ctx.ui.ok("Tramas TX/RX no ecra: " + ("ON" if ctx.show_raw else "OFF (o log continua a grava-las)"))


def cmd_note(ctx, args):
    if not args:
        raise UsageError("note <texto>")
    ctx.ui.ok("Nota: " + " ".join(args))


def cmd_help(ctx, args):
    ui = ctx.ui
    for group in GROUP_ORDER:
        ui.section(group)
        for command in COMMANDS:
            if command.group != group:
                continue
            left = command.names[0] + (" " + command.usage if command.usage else "")
            aliases = ", ".join(command.names[1:])
            line = "  " + ui.style(left.ljust(18), "bold") + command.help
            if aliases:
                line += ui.style("  [" + aliases + "]", "dim")
            ui.plain(line)
    ui.plain("")
    ui.plain("  " + ui.style("As mudancas de estado do CP aparecem sozinhas, a qualquer momento ("
                             + ui.sym["event"] + ").", "dim"))
    ui.plain("  " + ui.style("Setas: historico de comandos. Tab: completa o comando.", "dim"))


Command = namedtuple("Command", "names usage help group handler")

GROUP_ORDER = ["Sessao", "Leituras", "Bancada (sem carro ligado)", "Sistema"]

COMMANDS = [
    Command(("charge", "c"), "<A>", "Autoriza + inicia sessao + pede <A> (6-33)", "Sessao", cmd_charge),
    Command(("current",), "<A>", "Ajusta so a corrente durante a sessao (6-33)", "Sessao", cmd_current),
    Command(("stop",), "", "Termina a sessao (SESSION_STOP + AUTH_FALSE)", "Sessao", cmd_stop),
    Command(("status",), "", "Estado do CP + falhas + reles", "Leituras", cmd_status),
    Command(("meters", "m"), "", "Tensao / corrente / potencia das 3 linhas", "Leituras", cmd_meters),
    Command(("faults",), "", "Falhas ativas, com detalhe por rele", "Leituras", cmd_faults),
    Command(("rcd",), "", "Linha RCD: ultimo disparo e impulsos rejeitados", "Leituras", cmd_rcd),
    Command(("relays", "r"), "", "Estado ao vivo dos contactores K1..K4", "Leituras", cmd_relays),
    Command(("diag",), "", "Uptime do carregador e saude do RS485 (erros/recuperacoes)", "Leituras", cmd_diag),
    Command(("relay",), "on|off", "Atua os contactores DIRETAMENTE (sem verificacoes)",
            "Bancada (sem carro ligado)", cmd_relay_raw),
    Command(("meas",), "on|off", "Forca o circuito de medicao K1/K4 sempre ativo",
            "Bancada (sem carro ligado)", cmd_relay_meas),
    Command(("clear",), "", "Limpa as falhas (CMD_CLEAR_FAULTS)", "Sistema", cmd_clear),
    Command(("ping",), "", "Heartbeat, com tempo de resposta", "Sistema", cmd_ping),
    Command(("raw",), "on|off", "Mostra/esconde as tramas TX/RX (o log grava sempre)", "Sistema", cmd_raw),
    Command(("note",), "<texto>", "Marca um momento no log", "Sistema", cmd_note),
    Command(("help", "h", "?"), "", "Esta ajuda", "Sistema", cmd_help),
    Command(("quit", "q", "exit"), "", "Sai (Ctrl+C ou Ctrl+D tambem)", "Sistema", None),
]
LOOKUP = {name: command for command in COMMANDS for name in command.names}


# --------------------------------------------------------------------------
# Prompt: historico + Tab
# --------------------------------------------------------------------------
HISTORY_FILE = os.path.expanduser("~/.charger_master_history")
_ON_OFF_COMMANDS = ("relay", "meas", "raw")


def _complete(text, state):
    line = readline.get_line_buffer()
    words = line.split()
    if not words or (len(words) == 1 and not line.endswith(" ")):
        options = sorted(name for name in LOOKUP if name.startswith(text))
    elif words[0].lower() in _ON_OFF_COMMANDS:
        options = [o for o in ("on", "off") if o.startswith(text)]
    else:
        options = []
    return options[state] + " " if state < len(options) else None


def setup_readline():
    if readline is None:
        return
    try:
        readline.read_history_file(HISTORY_FILE)
    except OSError:
        pass
    readline.set_history_length(500)
    readline.set_completer(_complete)
    readline.set_completer_delims(" ")
    readline.parse_and_bind("tab: complete")


def save_readline_history():
    if readline is None:
        return
    try:
        readline.write_history_file(HISTORY_FILE)
    except OSError:
        pass


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def on_state_notify(ui, cp_state, cable_amps):
    msg = "CP: " + cp_name(cp_state)
    if cable_amps is not None:
        msg += "  (cabo: {})".format("{} A".format(cable_amps) if cable_amps > 0 else "desconhecido")
    ui.event(msg, *cp_style(cp_state))


def print_banner(ui, args, log_path):
    rule = ui.sym["rule"] * 50
    ui.plain(ui.style(rule, "cyan"))
    ui.plain("  " + ui.style("ChargerPowerV4", "bold") + "  Master RS485 de teste")
    ui.plain(ui.style(rule, "cyan"))
    ui.kv("Porta", "{} @ {} 8N1".format(args.port, args.baud))
    ui.kv("DE/RE", "GPIO{} (gpiochip{})".format(args.gpio_de, args.gpio_chip))
    if log_path:
        ui.kv("Log", log_path)
    else:
        ui.kv("Log", "desligado", "yellow")


def main():
    default_log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    parser = argparse.ArgumentParser(description="Master RS485 de teste para o ChargerPowerV4")
    parser.add_argument("--port", default="/dev/serial0", help="Porta serie (default /dev/serial0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--gpio-chip", type=int, default=0, help="gpiochip a usar (default 0)")
    parser.add_argument("--gpio-de", type=int, default=16, help="Pino BCM do DE/RE do RS485 (default 16, RS_CTRL_PIN_UART0)")
    parser.add_argument("--log-dir", default=default_log_dir, help="Pasta dos logs (default: logs/ ao lado do script)")
    parser.add_argument("--no-color", action="store_true", help="Sem cores (ex.: para guardar a saida num ficheiro)")
    args = parser.parse_args()

    log, log_path = setup_logging(os.path.expanduser(args.log_dir))
    color = (sys.stdout.isatty() and not args.no_color
             and "NO_COLOR" not in os.environ and os.environ.get("TERM") != "dumb")
    ui = UI(log, color)
    log.info("Inicio da sessao: porta=%s baud=%d gpiochip=%d DE=GPIO%d",
             args.port, args.baud, args.gpio_chip, args.gpio_de)

    try:
        link = RS485Link(args.port, args.baud, args.gpio_chip, args.gpio_de, log)
    except serial.SerialException as exc:
        ui.err("Nao consegui abrir {}: {}".format(args.port, exc))
        sys.exit(1)
    except lgpio.error as exc:
        ui.err("Nao consegui reservar o GPIO{} (gpiochip{}): {}".format(args.gpio_de, args.gpio_chip, exc))
        ui.info("Confirma que nao ha outro processo a usar o pino, e que corres com permissoes de GPIO.")
        sys.exit(1)

    ctx = Context(ui, link, log)
    link.on_state_notify = lambda cp_state, cable_amps: on_state_notify(ui, cp_state, cable_amps)
    link.on_raw = lambda direction, name, frame: ui.raw(direction, name, frame) if ctx.show_raw else None
    link.on_error = ui.err

    setup_readline()
    print_banner(ui, args, log_path)
    link.start()
    ui.plain("")
    cmd_ping(ctx, [])  # confirma logo se o carregador responde
    ui.plain("\n  Escreve " + ui.style("help", "bold") + " para ver os comandos.")

    try:
        while True:
            try:
                line = ui.read_command()
            except (EOFError, KeyboardInterrupt):
                break
            if not line:
                continue
            log.info("> %s", line)
            words = line.split()
            command = LOOKUP.get(words[0].lower())
            if command is None:
                ui.warn("Comando desconhecido: {!r}. Escreve 'help'.".format(words[0]))
                continue
            if command.handler is None:
                break
            try:
                command.handler(ctx, words[1:])
            except UsageError as exc:
                ui.warn("Uso: " + str(exc))
            except KeyboardInterrupt:
                ui.warn("Interrompido.")
            except Exception as exc:
                log.exception("Erro a executar %r", line)
                ui.err("Erro inesperado: {} (detalhe no log)".format(exc))
    finally:
        ui.plain("")
        log.info("Fim da sessao")
        link.stop()
        save_readline_history()
        if log_path:
            ui.plain("  Log gravado em " + ui.style(log_path, "bold"))


if __name__ == "__main__":
    main()
