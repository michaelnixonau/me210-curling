#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 5 ]]; then
  echo "Usage: $0 <avrdude_bin> <avrdude_conf> <mcu> <port> <hex_file>"
  exit 2
fi

AVRDUDE_BIN="$1"
AVRDUDE_CONF="$2"
MCU="$3"
PORT="$4"
HEX_FILE="$5"

RESET_DELAY_SEC="${BLE_RESET_DELAY_SEC:-0.18}"
BAUD_LIST="${BLE_UPLOAD_BAUDS:-115200}"

send_bridge_reset() {
  if [[ "${PORT}" =~ ^net:([^:]+):([0-9]+)$ ]]; then
    local host="${BASH_REMATCH[1]}"
    local tcp_port="${BASH_REMATCH[2]}"
    if python3 - "$host" "$tcp_port" <<'PY'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
try:
    sock = socket.create_connection((host, port), timeout=0.5)
    sock.sendall(b"RESET\n")
    sock.close()
except OSError:
    sys.exit(1)
PY
    then
      sleep "${RESET_DELAY_SEC}"
      return 0
    fi
  fi
  return 1
}

for BAUD in ${BAUD_LIST}; do
  echo "[ble-upload] Trying ${MCU} at ${BAUD} baud over ${PORT}"
  send_bridge_reset || true
  if "${AVRDUDE_BIN}" \
    -C "${AVRDUDE_CONF}" \
    -p "${MCU}" \
    -c stk500v1 \
    -P "${PORT}" \
    -b "${BAUD}" \
    -U "flash:w:${HEX_FILE}:i"; then
    echo "[ble-upload] Upload succeeded at ${BAUD} baud"
    exit 0
  fi
done

echo "[ble-upload] Upload failed at all baud rates"
exit 1
