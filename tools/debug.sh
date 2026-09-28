#!/usr/bin/env bash
# Arranca o ST-Link GDB server em fundo, liga o GDB, grava o .elf e para em main().
# Uso: tools/debug.sh [Debug|Release]
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_TYPE="${1:-Debug}"
ELF="$PROJECT_DIR/build/$BUILD_TYPE/ChargerPowerV4.elf"

CUBECLT="/opt/st/stm32cubeclt_1.21.0"
GDBSERVER_DIR="$CUBECLT/STLink-gdb-server/bin"
GDBSERVER_BIN="$GDBSERVER_DIR/ST-LINK_gdbserver"
PROGRAMMER_DIR="$CUBECLT/STM32CubeProgrammer/bin"
GDB="$CUBECLT/GNU-tools-for-STM32/bin/arm-none-eabi-gdb"
PORT=61234

if [ ! -f "$ELF" ]; then
    echo "ELF nao encontrado: $ELF"
    echo "Compila primeiro: cmake --build --preset $BUILD_TYPE"
    exit 1
fi

if ss -tln 2>/dev/null | grep -q ":$PORT "; then
    echo "Porta $PORT ja esta em uso -- ha um gdbserver preso de uma sessao anterior?"
    echo "Verifica com: ss -tlnp | grep $PORT"
    exit 1
fi

LOG="$(mktemp)"
(
    cd "$GDBSERVER_DIR"
    exec "$GDBSERVER_BIN" -cp "$PROGRAMMER_DIR" -d -p "$PORT"
) > "$LOG" 2>&1 &
GDBSERVER_PID=$!

cleanup() {
    kill "$GDBSERVER_PID" >/dev/null 2>&1 || true
    wait "$GDBSERVER_PID" 2>/dev/null || true
    rm -f "$LOG"
}
trap cleanup EXIT INT TERM

echo "A arrancar ST-Link GDB server (pid $GDBSERVER_PID)..."
ready=0
for _ in $(seq 1 100); do
    if grep -q "Waiting for debugger connection" "$LOG" 2>/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "$GDBSERVER_PID" 2>/dev/null; then
        echo "gdbserver terminou inesperadamente:"
        cat "$LOG"
        exit 1
    fi
    sleep 0.1
done

if [ "$ready" -ne 1 ]; then
    echo "gdbserver nao ficou pronto a tempo. Log:"
    cat "$LOG"
    exit 1
fi

echo "Servidor pronto. A ligar o GDB e a gravar $ELF..."
"$GDB" -q "$ELF" \
    -ex "target extended-remote localhost:$PORT" \
    -ex "monitor reset" \
    -ex "load" \
    -ex "break main" \
    -ex "continue"
