#!/usr/bin/env bash
set -euo pipefail

APPROVED_FOR_INSTALL=false
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

python3 "${HERE}/verify_package_20261104.py"
[[ "${APPROVED_FOR_INSTALL}" == true ]] || {
    echo "NOT INSTALL AUTHORIZATION: APPROVED_FOR_INSTALL=false" >&2
    echo "No system file or running service was changed." >&2
    exit 3
}
[[ "${EUID}" -eq 0 ]] || { echo "Run the authorized revision with sudo." >&2; exit 1; }

install -m 0644 "${HERE}/tractor-rpi-arbitrator.service" \
    /etc/systemd/system/tractor-rpi-arbitrator.service
install -m 0644 "${HERE}/tractor-phone-arbitrator.service" \
    /etc/systemd/system/tractor-phone-arbitrator.service
systemctl daemon-reload
systemctl disable tractor-rpi-arbitrator.service tractor-phone-arbitrator.service \
    >/dev/null 2>&1 || true
echo "Candidate units installed but disabled. No service was started."
