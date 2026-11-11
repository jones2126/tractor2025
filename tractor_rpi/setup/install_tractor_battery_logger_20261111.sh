#!/usr/bin/env bash
# Install only; use --start-now only during a separately approved stationary check.
set -euo pipefail

TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
SOURCE_UNIT="${TRACTOR_REPO}/tractor_rpi/setup/tractor-battery-logger.service"
TARGET_UNIT="/etc/systemd/system/tractor-battery-logger.service"
START_NOW=false

case "${1:-}" in
    "") ;;
    --start-now) START_NOW=true ;;
    *) echo "Usage: sudo $0 [--start-now]" >&2; exit 2 ;;
esac
[[ ${EUID} -eq 0 ]] || { echo "Run this installer with sudo." >&2; exit 1; }
[[ -f "${SOURCE_UNIT}" ]] || { echo "Missing ${SOURCE_UNIT}" >&2; exit 1; }
[[ -f "${TRACTOR_REPO}/tractor_rpi/battery_voltage_logger_20261111.py" ]] || {
    echo "Missing battery logger module." >&2; exit 1;
}

install -m 0644 "${SOURCE_UNIT}" "${TARGET_UNIT}"
systemctl daemon-reload
systemctl enable tractor-battery-logger.service
echo "Installed and enabled tractor-battery-logger.service."
echo "Logs will be written to /var/lib/tractor-battery/."
echo "No service was started in this session."

if [[ "${START_NOW}" == true ]]; then
    systemctl restart tractor-battery-logger.service
    systemctl --no-pager --full status tractor-battery-logger.service
fi
