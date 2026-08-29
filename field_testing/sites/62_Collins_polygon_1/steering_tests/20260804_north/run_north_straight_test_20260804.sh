#!/usr/bin/env bash
# Run exactly one isolated northern-slope steering test. No turns are included.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260714.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260828.py"
TEST_ID="${1:-}"

case "${TEST_ID^^}" in
    1E) mission="north_line_1_east_20260804.txt"; direction="EAST"; heading="090"; speed="0.50" ;;
    1W) mission="north_line_1_west_20260804.txt"; direction="WEST"; heading="270"; speed="0.50" ;;
    2E) mission="north_line_2_east_20260804.txt"; direction="EAST"; heading="090"; speed="0.50" ;;
    2W) mission="north_line_2_west_20260804.txt"; direction="WEST"; heading="270"; speed="0.50" ;;
    2E85) mission="north_line_2_east_085_20260804.txt"; direction="EAST"; heading="090"; speed="0.85" ;;
    2W85) mission="north_line_2_west_085_20260804.txt"; direction="WEST"; heading="270"; speed="0.85" ;;
    *)
        echo "Usage: $0 {1E|1W|2E|2W|2E85|2W85}" >&2
        echo "Each choice is a separate straight mission with no automatic turn." >&2
        exit 2
        ;;
esac
MAX_SPEED_MPS="${speed}"

MISSION="${SCRIPT_DIR}/${mission}"
for required in "${MISSION}" "${CONTROLLER}" "${LOGGER}"; do
    if [[ ! -f "${required}" ]]; then
        echo "ERROR: required file not found: ${required}" >&2
        exit 1
    fi
done

if pgrep -f '[p]ython3.*field_test_logger_20260828.py' >/dev/null; then
    echo "ERROR: a field logger is already running." >&2
    exit 1
fi
if pgrep -f '[p]ython3.*pure_pursuit_controller_20260714.py' >/dev/null; then
    echo "ERROR: a Pure Pursuit controller is already running." >&2
    exit 1
fi

read -r start_lat start_lon _ < "${MISSION}"
read -r end_lat end_lon _ < <(tail -n 1 "${MISSION}")

echo "============================================================"
echo " NORTHERN-SLOPE STRAIGHT STEERING TEST ${TEST_ID^^}"
echo " Direction       : ${direction} (compass heading ${heading} degrees)"
echo " Mission speed   : ${MAX_SPEED_MPS} m/s"
echo " Start lat/lon   : ${start_lat}, ${start_lon}"
echo " End lat/lon     : ${end_lat}, ${end_lon}"
echo " Autonomous turns: NONE"
echo "============================================================"
echo "Keep the radio switch UP/PAUSE and mower deck disengaged."
echo "Manually position near the stated start, pointing ${direction}."
echo "The start and end are approximately 3 m inside the newly driven path."
echo "After the controller is ready, select Auto only when the area is clear."
echo "Select Pause immediately at completion or whenever behavior is unexpected."
echo
read -r -p "Type RUN ${TEST_ID^^} to start the logger and controller: " confirmation
if [[ "${confirmation}" != "RUN ${TEST_ID^^}" ]]; then
    echo "Aborted; nothing was started."
    exit 1
fi

mkdir -p /home/al/field_logs
timestamp="$(date '+%Y%m%d_%H%M%S')"
field_log="/home/al/field_logs/north_steering_${TEST_ID^^}_${timestamp}.csv"

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

python3 -u "${LOGGER}" --output "${field_log}" &
logger_pid=$!
sleep 2
if ! kill -0 "${logger_pid}" 2>/dev/null; then
    echo "ERROR: field logger stopped during startup." >&2
    wait "${logger_pid}" || true
    exit 1
fi

echo "Logger running (PID ${logger_pid})."
echo "Starting Pure Pursuit; remain in Pause until its status is reviewed."
echo
python3 -u "${CONTROLLER}" \
    "${MISSION}" \
    --mode live \
    --gps-port 6010 \
    --min-fix "RTK Fixed" \
    --ip 127.0.0.1 \
    --port 6004 \
    --max-speed "${MAX_SPEED_MPS}"
