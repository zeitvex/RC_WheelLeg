#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export DISPLAY="${DISPLAY:-:0}"

if [ -z "${XAUTHORITY:-}" ]; then
    UID_VALUE="$(id -u)"

    for candidate in \
        "$HOME/.Xauthority" \
        "/run/user/$UID_VALUE/gdm/Xauthority" \
        "/run/user/$UID_VALUE/Xauthority"
    do
        if [ -f "$candidate" ]; then
            export XAUTHORITY="$candidate"
            break
        fi
    done
fi

python3 fullscreen_quit.py
