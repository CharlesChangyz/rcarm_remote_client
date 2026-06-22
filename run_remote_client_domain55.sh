#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[run_remote_client_domain55] missing ROS 2 Humble setup: /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  echo "[run_remote_client_domain55] missing workspace setup: ${WORKSPACE_DIR}/install/setup.bash" >&2
  echo "[run_remote_client_domain55] build first: cd \"${WORKSPACE_DIR}\" && colcon build --symlink-install" >&2
  exit 1
fi

# ROS/ament setup scripts may read optional environment variables before
# defining them, which is incompatible with `set -u`.
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

DISCOVERY_SERVER_HOST="${DISCOVERY_SERVER_HOST:-192.168.3.83}"
DISCOVERY_SERVER_PORT="${DISCOVERY_SERVER_PORT:-11811}"
export ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER:-${DISCOVERY_SERVER_HOST}:${DISCOVERY_SERVER_PORT}}"

echo "[run_remote_client_domain55] ros_domain_id=${ROS_DOMAIN_ID}"
echo "[run_remote_client_domain55] ros_localhost_only=${ROS_LOCALHOST_ONLY}"
echo "[run_remote_client_domain55] rmw_implementation=${RMW_IMPLEMENTATION}"
echo "[run_remote_client_domain55] ros_discovery_server=${ROS_DISCOVERY_SERVER}"
echo "[run_remote_client_domain55] starting remote GUI"
exec ros2 run rc_arm_remote_client remote_gui "$@"
