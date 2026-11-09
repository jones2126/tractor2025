#!/usr/bin/env bash
set -euo pipefail

# Operator approved production boot activation after the motor-off Manual,
# guarded Pause recovery, and local-only notification tests passed.
APPROVED_FOR_PRODUCTION_BOOT_ACTIVATION=true
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

python3 "${HERE}/verify_package_20261104.py"
[[ "${APPROVED_FOR_PRODUCTION_BOOT_ACTIVATION}" == true ]] || {
    echo "NOT PRODUCTION BOOT AUTHORIZATION: APPROVED_FOR_PRODUCTION_BOOT_ACTIVATION=false" >&2
    echo "No service enablement was changed." >&2
    exit 3
}
[[ "${EUID}" -eq 0 ]] || { echo "Run the authorized revision with sudo." >&2; exit 1; }

install -m 0644 "${HERE}/tractor-rpi-arbitrator.service" \
    /etc/systemd/system/tractor-rpi-arbitrator.service
install -m 0644 "${HERE}/tractor-phone-arbitrator.service" \
    /etc/systemd/system/tractor-phone-arbitrator.service
systemctl daemon-reload

systemctl disable tractor-wifi-control.service teensy-bridge.service \
    >/dev/null 2>&1 || true
systemctl enable tractor-rpi-arbitrator.service tractor-phone-arbitrator.service

echo "PASS: V3 arbitrator and local GL-router phone service are selected for boot."
echo "The running services were not restarted by this installer."
echo "Engine start and physical motion remain separately gated."
