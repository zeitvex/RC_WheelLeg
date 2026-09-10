#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

source "${SCRIPT_DIR}/runros.sh"
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
