#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export DISPLAY="${DISPLAY:-:0}"
export PYTHONUNBUFFERED=1

echo "[sim2real-screen] service starting as $(id -un), DISPLAY=$DISPLAY"

for _ in $(seq 1 120); do
    if [ -S "/tmp/.X11-unix/X${DISPLAY#:}" ]; then
        break
    fi
    sleep 1
done

if [ ! -S "/tmp/.X11-unix/X${DISPLAY#:}" ]; then
    echo "[sim2real-screen] X11 socket for DISPLAY=$DISPLAY not found" >&2
    exit 1
fi

exec "$SCRIPT_DIR/run_fullscreen_quit.sh"
