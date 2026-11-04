#!/usr/bin/env bash
set -euo pipefail

APPROVED_FOR_FIELD=false
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
SOURCE="${HERE}/../20260928_outer_perimeter_field_test"
MISSION="${SOURCE}/generated/62_Collins_outer_perimeter_field_test_1mps_20260928.txt"
AUDIT="${SOURCE}/generated/62_Collins_outer_perimeter_field_test_1mps_20260928_audit.csv"
VERIFY="${HERE}/verify_package_20261104.py"
PREFLIGHT="${REPO}/tractor_rpi/testing/mission_preflight_20261002.py"
CONTROLLER="${REPO}/tractor_rpi/pure-pursuit/pure_pursuit_arbitrator_feed_20261031.py"
LOGGER="${REPO}/tractor_rpi/field_test_logger_20260828.py"
EXPECTED_FIRMWARE="teensy_main_20261104_rpi_command_lease_v3"
CONFIRMATION="RUN PHONE ARBITRATOR OUTER PERIMETER BLADES OFF"

dashboard_mode=false
verify_only=false
case "${1:-}" in
    --dashboard) dashboard_mode=true; shift ;;
    --verify-only) verify_only=true; shift ;;
esac
[[ $# -eq 0 ]] || { echo "Usage: $0 [--dashboard|--verify-only]" >&2; exit 2; }

python3 "${VERIFY}"
if [[ "${verify_only}" == true ]]; then
    echo "Verification complete. Nothing was started."
    exit 0
fi
[[ "${APPROVED_FOR_FIELD}" == true ]] || {
    echo "NOT FIELD AUTHORIZATION: APPROVED_FOR_FIELD=false" >&2
    exit 3
}

systemctl is-active --quiet tractor-rpi-arbitrator.service
systemctl is-active --quiet tractor-phone-arbitrator.service
for conflicting in teensy-bridge.service tractor-wifi-control.service tractor-wifi-direct-mission.service tractor-wifi-recovery.service; do
    ! systemctl is-active --quiet "${conflicting}" || {
        echo "ERROR: conflicting service active: ${conflicting}" >&2; exit 1;
    }
done
! pgrep -f '[p]ure_pursuit.*20261031.py' >/dev/null || {
    echo "ERROR: Pure Pursuit is already running" >&2; exit 1;
}

if [[ "${dashboard_mode}" == true ]]; then
    python3 "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}"
else
    sudo python3 "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}"
    read -r -p "Type ${CONFIRMATION} to start: " response
    [[ "${response}" == "${CONFIRMATION}" ]] || { echo "Aborted."; exit 1; }
fi

log_dir="/home/al/field_logs/20261104_phone_arbitrator_outer_perimeter"
mkdir -p "${log_dir}"
field_log="${log_dir}/outer_perimeter_$(date '+%Y%m%d_%H%M%S').csv"
logger_pid=""
cleanup() {
    if [[ -n "${logger_pid}" ]] && kill -0 "${logger_pid}" 2>/dev/null; then
        kill "${logger_pid}"; wait "${logger_pid}" 2>/dev/null || true
    fi
    echo "Mission stopped. Field log: ${field_log}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

python3 -u "${LOGGER}" --output "${field_log}" & logger_pid=$!
sleep 2
kill -0 "${logger_pid}" 2>/dev/null

python3 -u "${CONTROLLER}" "${MISSION}" \
    --arbitrator-safety-port 6015 --arbitrator-proposal-only \
    --mode live --gps-port 6010 --status-port 6003 \
    --min-fix "RTK Fixed" --ip 127.0.0.1 --port 16004 \
    --max-speed 1.00 --tracking-window 6.0 --audit-file "${AUDIT}" \
    --reacquire-max-advance 5.0 --resume-stable-seconds 5.0 \
    --control-port 6011 --telemetry-port 6012
