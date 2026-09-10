#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/src/odin_ros_driver/config/control_command_mapping.yaml"

if [ ! -f "${CONFIG_FILE}" ]; then
    echo "[ERROR] Mapping configuration not found: ${CONFIG_FILE}" >&2
    exit 1
fi

exec "${SCRIPT_DIR}/runros.sh" \
    ros2 launch odin_ros_driver odin1_ros2.launch.py \
    "config_file:=${CONFIG_FILE}" \
    "launch_rviz:=true"
