#!/usr/bin/env bash
set -euo pipefail

# Supervised engineering-test launcher for a mission with validation status REVIEW.
# This is intentionally separate from the PASS-only archived mission launcher.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MISSION="${SCRIPT_DIR}/62_Collins_polygon_1_supervised_test_mission.txt"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260714.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260717.py"
EXPECTED_SHA256="2b477d14479a6f138af2dbd2e8156d0e5db7ad77f76341248b5dea8833db703c"
MAX_SPEED_MPS="0.75"

for required in "${MISSION}" "${CONTROLLER}" "${LOGGER}"; do
    if [[ ! -f "${required}" ]]; then
        echo "ERROR: required file not found: ${required}" >&2
        exit 1
    fi
done

actual_sha256="$(sha256sum "${MISSION}" | awk '{print $1}')"
if [[ "${actual_sha256}" != "${EXPECTED_SHA256}" ]]; then
    echo "ERROR: mission SHA-256 does not match the visually reviewed file." >&2
    echo "Expected: ${EXPECTED_SHA256}" >&2
    echo "Actual  : ${actual_sha256}" >&2
    exit 1
fi

if pgrep -f '[p]ython3.*field_test_logger_20260717.py' >/dev/null; then
    echo "ERROR: a field_test_logger process is already running." >&2
    echo "Stop or inspect it before starting this launcher." >&2
    exit 1
fi
if pgrep -f '[p]ython3.*pure_pursuit_controller_20260714.py' >/dev/null; then
    echo "ERROR: a Pure Pursuit controller process is already running." >&2
    echo "Stop or inspect it before starting this launcher." >&2
    exit 1
fi

echo "============================================================"
echo " SUPERVISED REVIEW MISSION — NOT A STATIC VALIDATION PASS"
echo " Site             : 62_Collins_polygon_1"
echo " Outer perimeter  : 0.50 m/s"
echo " Second perimeter : 0.75 m/s"
echo " Controller cap   : ${MAX_SPEED_MPS} m/s"
echo "============================================================"
echo "Validation warning: sampled minimum radius was 0.77 m."
echo "Keep the tractor in Pause until RTK Fixed, heading, neutral,"
echo "steering direction, route start, and e-stop are confirmed."
echo "For the first test, keep the mower deck off and remain ready"
echo "to return the tractor to Pause or use the e-stop."
echo
read -r -p 'Type RUN REVIEW to continue: ' confirmation
if [[ "${confirmation}" != "RUN REVIEW" ]]; then
    echo "Aborted; nothing was started."
    exit 1
fi

mkdir -p /home/al/field_logs
timestamp="$(date '+%Y%m%d_%H%M%S')"
field_log="/home/al/field_logs/62_Collins_polygon_1_${timestamp}.csv"

logger_pid=""
cleanup() {
    if [[ -n "${logger_pid}" ]] && kill -0 "${logger_pid}" 2>/dev/null; then
        echo
        echo "Stopping field logger (PID ${logger_pid})..."
        kill "${logger_pid}"
        wait "${logger_pid}" 2>/dev/null || true
    fi
    echo "Field log: ${field_log}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
trap 'exit 129' HUP

echo "Starting field logger..."
python3 -u "${LOGGER}" --output "${field_log}" &
logger_pid=$!
sleep 2
if ! kill -0 "${logger_pid}" 2>/dev/null; then
    echo "ERROR: field logger stopped during startup." >&2
    wait "${logger_pid}" || true
    exit 1
fi

echo "Field logger running (PID ${logger_pid})."
echo "Starting controller. Ctrl+C stops the controller and logger."
echo
python3 -u "${CONTROLLER}" \
    "${MISSION}" \
    --mode live \
    --gps-port 6010 \
    --ip 127.0.0.1 \
    --port 6004 \
    --max-speed "${MAX_SPEED_MPS}" \
    "$@"
