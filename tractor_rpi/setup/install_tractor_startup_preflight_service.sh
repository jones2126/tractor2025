#!/usr/bin/env bash
# Install the Tractor01 boot preflight. Use --start-now to run it immediately.
set -euo pipefail

TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
SOURCE_UNIT="${TRACTOR_REPO}/tractor_rpi/setup/tractor-startup-preflight.service"
TARGET_UNIT="/etc/systemd/system/tractor-startup-preflight.service"
START_NOW=false

if [[ "${1:-}" == "--start-now" ]]; then
    START_NOW=true
elif [[ $# -ne 0 ]]; then
    echo "Usage: sudo $0 [--start-now]" >&2
    exit 2
fi

if [[ ${EUID} -ne 0 ]]; then
    echo "Run this installer with sudo." >&2
    exit 1
fi

for required in \
    "${SOURCE_UNIT}" \
    "${TRACTOR_REPO}/tractor_rpi/testing/run_startup_mission_preflight_20261026.sh" \
    "${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_20261002.py" \
    "${TRACTOR_REPO}/tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py"; do
    [[ -f "${required}" ]] || { echo "Missing required file: ${required}" >&2; exit 1; }
done

install -m 0644 "${SOURCE_UNIT}" "${TARGET_UNIT}"
systemctl daemon-reload
systemctl enable tractor-startup-preflight.service

echo "Installed and enabled tractor-startup-preflight.service."
echo "The existing rtcm-server ExecStartPre remains the mandatory Heading-F9P RAM configuration step."

if [[ "${START_NOW}" == true ]]; then
    systemctl restart tractor-startup-preflight.service
    systemctl --no-pager --full status tractor-startup-preflight.service
fi
