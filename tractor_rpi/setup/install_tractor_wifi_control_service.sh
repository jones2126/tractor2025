#!/usr/bin/env bash
set -euo pipefail

REPO=/home/al/tractor2025
SOURCE_UNIT="$REPO/tractor_rpi/setup/tractor-wifi-control.service"
TARGET_UNIT=/etc/systemd/system/tractor-wifi-control.service
SERVER="$REPO/tractor_rpi/testing/webrtc/wifi_primary_control_20261003.py"
PAGE="$REPO/tractor_rpi/testing/webrtc/wifi_primary_control_20261003.html"
TLS_DIR=/home/al/.config/tractor-wifi-control/tls
CONFIG_DIR=/home/al/.config/tractor-wifi-control
OPERATOR_KEY_FILE="$CONFIG_DIR/operator.key"

if [[ $EUID -ne 0 ]]; then
    echo "Run this installer with sudo." >&2
    exit 1
fi

for required in "$SOURCE_UNIT" "$SERVER" "$PAGE" "$TLS_DIR/server.crt" "$TLS_DIR/server.key"; do
    [[ -r "$required" ]] || { echo "Required file is missing or unreadable: $required" >&2; exit 1; }
done

# The server creates the URL access key on its first start. Keep its directory
# private and owned by the unprivileged service account.
install -d -m 0700 -o al -g al "$CONFIG_DIR"

# Migrate the key printed by the currently installed service so the URL that
# is already open on the phone remains valid. On a fresh installation the
# server creates a new persistent key instead.
if [[ ! -e "$OPERATOR_KEY_FILE" ]]; then
    current_key="$(
        journalctl -u tractor-wifi-control.service -o cat --no-pager -n 500 2>/dev/null |
            sed -n 's|^ZeroTier: .*?key=\([A-Za-z0-9_-]*\)$|\1|p' |
            tail -n 1 || true
    )"
    if [[ "$current_key" =~ ^[A-Za-z0-9_-]{24,128}$ ]]; then
        printf '%s\n' "$current_key" > "$OPERATOR_KEY_FILE"
        chown al:al "$OPERATOR_KEY_FILE"
        chmod 0600 "$OPERATOR_KEY_FILE"
        echo "Preserved the current phone-control URL access key."
    fi
fi

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
