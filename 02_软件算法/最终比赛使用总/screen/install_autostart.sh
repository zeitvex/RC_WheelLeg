#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOSTART_DIR="$HOME/.config/autostart"
DESKTOP_FILE="$AUTOSTART_DIR/sim2real-screen.desktop"

mkdir -p "$AUTOSTART_DIR"
chmod +x "$SCRIPT_DIR/run_fullscreen_quit.sh"

cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=sim2real Screen
Comment=Start the sim2real touch control panel
Exec=$SCRIPT_DIR/run_fullscreen_quit.sh
Path=$SCRIPT_DIR
Terminal=false
X-GNOME-Autostart-enabled=true
StartupNotify=false
EOF

echo "Installed desktop autostart:"
echo "  $DESKTOP_FILE"
echo
echo "It will start after this user logs into the graphical desktop."
echo "For boot-time use, enable automatic login for this user on the Orin desktop."
