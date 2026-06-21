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

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
export ROS_DOMAIN_ID="55"

echo "[run_remote_client_domain55] ros_domain_id=${ROS_DOMAIN_ID}"
echo "[run_remote_client_domain55] starting remote GUI"
exec ros2 run rc_arm_remote_client remote_gui "$@"
