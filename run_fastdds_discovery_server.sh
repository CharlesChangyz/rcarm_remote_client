#!/usr/bin/env bash
set -euo pipefail

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[run_fastdds_discovery_server] missing ROS 2 Humble setup: /opt/ros/humble/setup.bash" >&2
  exit 1
fi

# ROS/ament setup scripts may read optional environment variables before
# defining them, which is incompatible with `set -u`.
set +u
source /opt/ros/humble/setup.bash
set -u

SERVER_ID="${FASTDDS_DISCOVERY_SERVER_ID:-0}"
SERVER_HOST="${FASTDDS_DISCOVERY_SERVER_HOST:-192.168.3.83}"
SERVER_PORT="${FASTDDS_DISCOVERY_SERVER_PORT:-11811}"

if ! command -v fastdds >/dev/null 2>&1; then
  echo "[run_fastdds_discovery_server] fastdds command not found after sourcing ROS 2 Humble" >&2
  exit 1
fi

if command -v ss >/dev/null 2>&1; then
  if UDP_LISTENERS="$(ss -H -lunp "sport = :${SERVER_PORT}" 2>/dev/null)" && [ -n "${UDP_LISTENERS}" ]; then
    echo "[run_fastdds_discovery_server] UDP port ${SERVER_PORT} is already in use; cannot listen on ${SERVER_HOST}:${SERVER_PORT}" >&2
    echo "[run_fastdds_discovery_server] existing listener(s):" >&2
    while IFS= read -r line; do
      echo "[run_fastdds_discovery_server]   ${line}" >&2
    done <<< "${UDP_LISTENERS}"
    echo "[run_fastdds_discovery_server] stop the existing discovery server or choose FASTDDS_DISCOVERY_SERVER_PORT=<free-port>" >&2
    exit 1
  fi
fi

echo "[run_fastdds_discovery_server] server_id=${SERVER_ID}"
echo "[run_fastdds_discovery_server] listen=${SERVER_HOST}:${SERVER_PORT}"
echo "[run_fastdds_discovery_server] starting Fast DDS discovery server"
exec fastdds discovery -i "${SERVER_ID}" -l "${SERVER_HOST}" -p "${SERVER_PORT}"
