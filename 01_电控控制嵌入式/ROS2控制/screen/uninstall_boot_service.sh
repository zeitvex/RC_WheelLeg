#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="sim2real-screen.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

sudo systemctl disable --now "$SERVICE_NAME" 2>/dev/null || true
sudo rm -f "$SERVICE_PATH"
sudo systemctl daemon-reload

echo "Removed $SERVICE_NAME"
