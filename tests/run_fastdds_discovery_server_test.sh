#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT_FILE="$(mktemp)"
OUTPUT_FILE="$(mktemp)"

cleanup() {
  if [ -n "${LISTENER_PID:-}" ]; then
    kill "${LISTENER_PID}" 2>/dev/null || true
  fi
  rm -f "${PORT_FILE}" "${OUTPUT_FILE}"
}
trap cleanup EXIT

python3 - "${PORT_FILE}" <<'PY' &
import socket
import sys
import time

port_file = sys.argv[1]
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 0))
with open(port_file, "w", encoding="utf-8") as handle:
    handle.write(str(sock.getsockname()[1]))
time.sleep(30)
PY
LISTENER_PID=$!

for _ in $(seq 1 50); do
  [ -s "${PORT_FILE}" ] && break
  sleep 0.1
done

if [ ! -s "${PORT_FILE}" ]; then
  echo "listener did not report a port" >&2
  exit 1
fi

PORT="$(cat "${PORT_FILE}")"

set +e
FASTDDS_DISCOVERY_SERVER_HOST=127.0.0.1 \
FASTDDS_DISCOVERY_SERVER_PORT="${PORT}" \
timeout 10s "${REPO_DIR}/run_fastdds_discovery_server.sh" >"${OUTPUT_FILE}" 2>&1
STATUS=$?
set -e

if [ "${STATUS}" -eq 0 ]; then
  echo "expected launcher to fail when UDP port is already in use" >&2
  cat "${OUTPUT_FILE}" >&2
  exit 1
fi

if ! rg -q "UDP port ${PORT} is already in use" "${OUTPUT_FILE}"; then
  echo "expected clear occupied-port diagnostic" >&2
  cat "${OUTPUT_FILE}" >&2
  exit 1
fi

if ! rg -q "users:" "${OUTPUT_FILE}"; then
  echo "expected diagnostic to include owning process" >&2
  cat "${OUTPUT_FILE}" >&2
  exit 1
fi
