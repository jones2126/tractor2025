#!/usr/bin/env bash
# Supervised launcher for the 15-inch-outset Collins mission.
# Starts field_test_logger_20260728.py in the background and Pure Pursuit in
# the foreground. Ctrl+C stops both cleanly.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
MISSION="${SCRIPT_DIR}/62_Collins_polygon_1_mission.txt"
VALIDATION="${SCRIPT_DIR}/62_Collins_polygon_1_mission_validation.json"
SETTINGS="${SCRIPT_DIR}/02_plan_settings.json"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260714.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260728.py"
EXPECTED_SHA256="0C5FF51604D67350A43E6AB225A2CA30CD2E147BBAE12CA0DA00A67CB0FDB187"
MAX_SPEED_MPS="0.85"

for required in \
    "${MISSION}" "${VALIDATION}" "${SETTINGS}" "${CONTROLLER}" "${LOGGER}"; do
    if [[ ! -f "${required}" ]]; then
        echo "ERROR: required file not found: ${required}" >&2
        exit 1
    fi
done

actual_sha256="$(tr -d '\r' < "${MISSION}" | sha256sum | awk '{print toupper($1)}')"
if [[ "${actual_sha256}" != "${EXPECTED_SHA256}" ]]; then
    echo "ERROR: mission SHA-256 does not match the reviewed mission." >&2
    echo "Expected: ${EXPECTED_SHA256}" >&2
    echo "Actual  : ${actual_sha256}" >&2
    exit 1
fi

python3 - "${VALIDATION}" "${SETTINGS}" "${MISSION}" <<'PY'
import json
import math
import pathlib
import sys

validation_path, settings_path, mission_path = map(pathlib.Path, sys.argv[1:])
validation = json.loads(validation_path.read_text(encoding="utf-8"))
settings = json.loads(settings_path.read_text(encoding="utf-8"))

if validation.get("status") != "REVIEW":
    raise SystemExit(
        f"ERROR: expected REVIEW validation status, got {validation.get('status')!r}"
    )
if not math.isclose(float(settings.get("boundary_outset_m", -1)), 0.381, abs_tol=1e-9):
    raise SystemExit("ERROR: settings do not contain the reviewed 0.381 m outset")
if not math.isclose(float(settings.get("boundary_clearance_m", -1)), 0.0, abs_tol=1e-9):
    raise SystemExit("ERROR: settings do not contain the reviewed 0 m inset")

speeds = []
for line_number, line in enumerate(mission_path.read_text(encoding="utf-8").splitlines(), 1):
    if not line.strip():
        continue
    fields = line.split()
    if len(fields) != 5:
        raise SystemExit(f"ERROR: mission line {line_number} does not have five fields")
    speeds.append(float(fields[4]))
if not speeds or any(not math.isclose(speed, 0.85, abs_tol=1e-9) for speed in speeds):
    raise SystemExit("ERROR: every reviewed mission speed must be 0.85 m/s")
PY

if pgrep -f '[p]ython3.*field_test_logger_20260728.py' >/dev/null; then
    echo "ERROR: a field_test_logger_20260728.py process is already running." >&2
    exit 1
fi
if pgrep -f '[p]ython3.*pure_pursuit_controller_20260714.py' >/dev/null; then
    echo "ERROR: a Pure Pursuit controller is already running." >&2
    exit 1
fi

echo "============================================================"
echo " SUPERVISED FIELD TEST — STATIC VALIDATION STATUS: REVIEW"
echo " Site              : 62_Collins_polygon_1"
echo " Mission speed     : 0.85 m/s"
echo " Controller cap    : ${MAX_SPEED_MPS} m/s"
echo " Containment outset: 0.381 m (15 inches)"
echo " Presumed uncut    : 83.0 square meters (4.7%)"
echo "============================================================"
echo "Known warning: sampled perimeter corner radius is 0.46 m."
echo "The tractor center may travel 15 inches outside the logged"
echo "Pass 1 centerline; the deck may extend 36 inches outside it."
echo
echo "BEFORE CONTINUING:"
echo "  - Physically verify the full exterior area is clear."
echo "  - Keep the mower deck OFF for the first supervised run."
echo "  - Keep the tractor in Pause until RTK Fixed, heading,"
echo "    steering direction, neutral, route start, and e-stop"
echo "    operation are confirmed."
echo "  - Remain beside the e-stop and be ready to select Pause."
echo
read -r -p 'Type RUN 15 INCH OUTSET to start logger and controller: ' confirmation
if [[ "${confirmation}" != "RUN 15 INCH OUTSET" ]]; then
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
echo "Starting Pure Pursuit. Ctrl+C stops the controller and logger."
echo
python3 -u "${CONTROLLER}" \
    "${MISSION}" \
    --mode live \
    --gps-port 6010 \
    --min-fix "RTK Fixed" \
    --ip 127.0.0.1 \
    --port 6004 \
    --max-speed "${MAX_SPEED_MPS}" \
    "$@"
