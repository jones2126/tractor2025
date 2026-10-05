#!/usr/bin/env bash
set -euo pipefail

REPO=/home/al/tractor2025
SOURCE_UNIT="$REPO/tractor_rpi/setup/tractor-wifi-control.service"
TARGET_UNIT=/etc/systemd/system/tractor-wifi-control.service
SERVER="$REPO/tractor_rpi/testing/webrtc/wifi_primary_control_20261003.py"
PAGE="$REPO/tractor_rpi/testing/webrtc/wifi_primary_control_20261003.html"
TLS_DIR=/home/al/.config/tractor-wifi-control/tls

if [[ $EUID -ne 0 ]]; then
    echo "Run this installer with sudo." >&2
    exit 1
fi

for required in "$SOURCE_UNIT" "$SERVER" "$PAGE" "$TLS_DIR/server.crt" "$TLS_DIR/server.key"; do
    [[ -r "$required" ]] || { echo "Required file is missing or unreadable: $required" >&2; exit 1; }
done

if pgrep -f '[p]ython3.*wifi_primary_control_20261003.py' >/dev/null && \
   ! systemctl is-active --quiet tractor-wifi-control.service; then
    echo "NOTICE: a manually started Wi-Fi control server is running."
    echo "Stop it with Ctrl+C before starting tractor-wifi-control.service."
fi

install -m 0644 "$SOURCE_UNIT" "$TARGET_UNIT"
systemctl daemon-reload
systemd-analyze verify "$TARGET_UNIT"
systemctl enable tractor-wifi-control.service

echo "Installed and enabled tractor-wifi-control.service."
echo "It will start automatically on the next boot."
echo "To start it now: sudo systemctl start tractor-wifi-control.service"
echo "To inspect it:   systemctl status tractor-wifi-control.service --no-pager"
