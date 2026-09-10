#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE_CONFIG="${SCRIPT_DIR}/src/odin_ros_driver/config/control_command_relocal.yaml"
RUNTIME_DIR="${SCRIPT_DIR}/src/odin_ros_driver/map/.runtime"

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 /absolute/path/to/map.bin" >&2
    exit 1
fi

MAP_PATH="$(readlink -f "$1")"
if [ ! -f "${MAP_PATH}" ]; then
    echo "[ERROR] Map file not found: ${MAP_PATH}" >&2
    exit 1
fi

mkdir -p "${RUNTIME_DIR}"
RUNTIME_CONFIG="${RUNTIME_DIR}/control_command_relocalization.yaml"

ESCAPED_MAP_PATH="${MAP_PATH//\\/\\\\}"
ESCAPED_MAP_PATH="${ESCAPED_MAP_PATH//&/\\&}"
ESCAPED_MAP_PATH="${ESCAPED_MAP_PATH//|/\\|}"
sed "s|/absolute/path/to/1hao.bin|${ESCAPED_MAP_PATH}|g" \
    "${TEMPLATE_CONFIG}" > "${RUNTIME_CONFIG}"

exec "${SCRIPT_DIR}/runros.sh" \
    ros2 launch odin_ros_driver odin1_ros2.launch.py \
    "config_file:=${RUNTIME_CONFIG}" \
    "launch_rviz:=true"
