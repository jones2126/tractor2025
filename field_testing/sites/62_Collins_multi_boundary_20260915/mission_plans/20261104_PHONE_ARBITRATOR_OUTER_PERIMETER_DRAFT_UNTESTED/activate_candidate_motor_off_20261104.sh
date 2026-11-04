#!/usr/bin/env bash
set -euo pipefail

APPROVED_FOR_MOTOR_OFF_COMMISSIONING=true
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

required=(
  --engine-off-confirmed
  --blades-off-confirmed
  --rear-lever-out-confirmed
  --red-estop-ready-confirmed
  --wheels-chocked-confirmed
)
[[ "$#" -eq "${#required[@]}" ]] || {
  echo "ERROR: all five physical confirmations are required in the printed order" >&2
  exit 2
}
for expected in "${required[@]}"; do
  [[ "${1:-}" == "${expected}" ]] || {
    echo "ERROR: expected ${expected}" >&2
    exit 2
  }
  shift
done

[[ "${APPROVED_FOR_MOTOR_OFF_COMMISSIONING}" == true ]]
[[ "${EUID}" -eq 0 ]] || { echo "Run with sudo." >&2; exit 1; }
python3 "${HERE}/verify_package_20261104.py"

! pgrep -f '[p]ure_pursuit' >/dev/null || {
  echo "ERROR: Pure Pursuit is running" >&2; exit 1;
}

failed=true
cleanup_failure() {
  if [[ "${failed}" == true ]]; then
    systemctl stop tractor-phone-arbitrator.service tractor-rpi-arbitrator.service \
      >/dev/null 2>&1 || true
    echo "Candidate activation failed; candidate services were stopped." >&2
    echo "Leave engine off and rear transmission lever OUT pending rollback." >&2
  fi
}
trap cleanup_failure EXIT

systemctl stop \
  tractor-phone-arbitrator.service \
  tractor-wifi-direct-mission.service \
  tractor-wifi-control.service \
  tractor-wifi-recovery.service
systemctl stop teensy-bridge.service

systemctl start tractor-rpi-arbitrator.service
sleep 5
systemctl is-active --quiet tractor-rpi-arbitrator.service

systemctl start tractor-phone-arbitrator.service
sleep 5
systemctl is-active --quiet tractor-phone-arbitrator.service

failed=false
echo "PASS: candidate arbitrator and phone services are active for motor-off commissioning."
echo "Engine start and physical motion remain prohibited."
