#!/usr/bin/env bash
set -euo pipefail

REPO=/home/al/tractor2025
NORMAL_SOURCE="$REPO/tractor_rpi/setup/tractor-wifi-control.service"
RECOVERY_SOURCE="$REPO/tractor_rpi/setup/tractor-wifi-recovery.service"
DIRECT_SOURCE="$REPO/tractor_rpi/setup/tractor-wifi-direct-mission.service"

if [[ $EUID -ne 0 ]]; then
    echo "Run this installer with sudo." >&2
    exit 1
fi

for source in "$NORMAL_SOURCE" "$RECOVERY_SOURCE" "$DIRECT_SOURCE"; do
    [[ -r "$source" ]] || { echo "Missing or unreadable: $source" >&2; exit 1; }
done

install -m 0644 "$NORMAL_SOURCE" /etc/systemd/system/tractor-wifi-control.service
install -m 0644 "$RECOVERY_SOURCE" /etc/systemd/system/tractor-wifi-recovery.service
install -m 0644 "$DIRECT_SOURCE" /etc/systemd/system/tractor-wifi-direct-mission.service
systemctl daemon-reload
systemd-analyze verify \
    /etc/systemd/system/tractor-wifi-control.service \
    /etc/systemd/system/tractor-wifi-recovery.service \
    /etc/systemd/system/tractor-wifi-direct-mission.service
systemctl disable tractor-wifi-direct-mission.service >/dev/null 2>&1 || true

echo "Installed the direct-mission unit; it remains disabled and stopped."
echo "Do not start it without a separately authorized, blades-off field package."
