#!/usr/bin/env bash
set -euo pipefail

if ! command -v systemctl >/dev/null 2>&1; then
    echo "systemctl not found; this installer is for systemd-based Linux." >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="sim2real-screen.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

if [ "${EUID:-$(id -u)}" -eq 0 ]; then
    RUN_USER="${1:-${SUDO_USER:-rc2}}"
else
    RUN_USER="${1:-$(id -un)}"
fi

if ! id "$RUN_USER" >/dev/null 2>&1; then
    echo "User '$RUN_USER' does not exist. Usage: bash install_boot_service.sh rc2" >&2
    exit 1
fi

RUN_GROUP="$(id -gn "$RUN_USER")"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
AUTOSTART_FILE="$RUN_HOME/.config/autostart/sim2real-screen.desktop"

chmod +x "$SCRIPT_DIR/run_fullscreen_quit.sh" "$SCRIPT_DIR/run_boot_screen_service.sh"
if [ -f "$AUTOSTART_FILE" ]; then
    rm -f "$AUTOSTART_FILE"
    echo "Removed desktop autostart to avoid duplicate screen instances:"
    echo "  $AUTOSTART_FILE"
fi

SERVICE_CONTENT="[Unit]
Description=sim2real touchscreen control panel
Wants=display-manager.service
After=systemd-user-sessions.service display-manager.service
StartLimitIntervalSec=0

[Service]
Type=simple
User=$RUN_USER
Group=$RUN_GROUP
WorkingDirectory=$SCRIPT_DIR
Environment=HOME=$RUN_HOME
Environment=DISPLAY=:0
Environment=PYTHONUNBUFFERED=1
ExecStart=$SCRIPT_DIR/run_boot_screen_service.sh
Restart=always
RestartSec=2
KillSignal=SIGINT
TimeoutStopSec=20

[Install]
WantedBy=graphical.target
"

printf "%s" "$SERVICE_CONTENT" | sudo tee "$SERVICE_PATH" >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"

echo "Installed and enabled:"
echo "  $SERVICE_PATH"
echo
echo "Start now:"
echo "  sudo systemctl restart $SERVICE_NAME"
echo
echo "Check status/logs:"
echo "  systemctl status $SERVICE_NAME --no-pager"
echo "  journalctl -u $SERVICE_NAME -f"
echo
echo "Important: the graphical desktop for user '$RUN_USER' must auto-login, otherwise Tk cannot open DISPLAY=:0."
