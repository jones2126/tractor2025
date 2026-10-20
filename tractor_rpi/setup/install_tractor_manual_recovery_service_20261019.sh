#!/usr/bin/env bash
set -euo pipefail

REPO=/home/al/tractor2025
NORMAL_SOURCE="$REPO/tractor_rpi/setup/tractor-wifi-control.service"
RECOVERY_SOURCE="$REPO/tractor_rpi/setup/tractor-wifi-recovery.service"
NORMAL_TARGET=/etc/systemd/system/tractor-wifi-control.service
RECOVERY_TARGET=/etc/systemd/system/tractor-wifi-recovery.service
WORKFLOW="$REPO/tractor_rpi/recovery/manual_recovery_workflow_20261019.py"
TLS_DIR=/home/al/.config/tractor-wifi-control/tls

if [[ $EUID -ne 0 ]]; then
    echo "Run this installer with sudo." >&2
    exit 1
fi

for required in "$NORMAL_SOURCE" "$RECOVERY_SOURCE" "$WORKFLOW" \
    "$TLS_DIR/server.crt" "$TLS_DIR/server.key"; do
    [[ -r "$required" ]] || { echo "Missing or unreadable: $required" >&2; exit 1; }
done

install -m 0644 "$NORMAL_SOURCE" "$NORMAL_TARGET"
install -m 0644 "$RECOVERY_SOURCE" "$RECOVERY_TARGET"
install -m 0755 "$WORKFLOW" /usr/local/sbin/tractor-manual-recovery
systemctl daemon-reload
systemd-analyze verify "$NORMAL_TARGET" "$RECOVERY_TARGET"
systemctl disable tractor-wifi-recovery.service >/dev/null 2>&1 || true
systemctl enable tractor-wifi-control.service

echo "Installed Manual Recovery. It is disabled at boot by design."
echo "Before field use, verify the phone-facing router SSID is TractorField."
echo "Start:  sudo tractor-manual-recovery start"
echo "Status: sudo tractor-manual-recovery status"
echo "Stop:   sudo tractor-manual-recovery stop"
