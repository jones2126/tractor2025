#!/usr/bin/env bash
# Supervised, blades-off launcher for the 2026-08-30 combined REVIEW_TEST mission.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
MISSION="${SCRIPT_DIR}/62_Collins_combined_14ring_21stripe_REVIEW_TEST_20260830.txt"
REPORT="${SCRIPT_DIR}/combined_review_test_report_20260830.json"
VALIDATION="${SCRIPT_DIR}/combined_validation_20260830.json"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260714.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260828.py"
EXPECTED_SHA256="eae4849dfb988082eb91e5f7b9aae602bbb14938cc450490bd333305d35b5ea7"
MAX_SPEED_MPS="1.25"

for required in "${MISSION}" "${REPORT}" "${VALIDATION}" "${CONTROLLER}" "${LOGGER}"; do
    if [[ ! -f "${required}" ]]; then
        echo "ERROR: required file not found: ${required}" >&2
        exit 1
    fi
done

actual_sha256="$(sha256sum "${MISSION}" | awk '{print $1}')"
if [[ "${actual_sha256}" != "${EXPECTED_SHA256}" ]]; then
    echo "ERROR: mission SHA-256 does not match the reviewed test mission." >&2
    echo "Expected: ${EXPECTED_SHA256}" >&2
    echo "Actual  : ${actual_sha256}" >&2
    exit 1
fi

python3 - "${REPORT}" "${VALIDATION}" <<'PY'
import json
import math
import pathlib
import sys

report = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
validation = json.loads(pathlib.Path(sys.argv[2]).read_text(encoding="utf-8"))
checks = {
    "review-test label": report.get("status") == "REVIEW_TEST",
    "validation": validation.get("status") == "PASS",
    "waypoints": report.get("waypoints") == 4124,
    "stripes": report.get("included_stripes") == 21,
    "transition containment": report.get("transition", {}).get("contained_in_site") is True,
    "transition outside core": math.isclose(
        float(report.get("transition", {}).get("length_inside_core_m", -1.0)),
        0.0,
        abs_tol=1e-9,
    ),
    "route containment": report.get("route_contained_in_site") is True,
    "short gaps": report.get("sub_minimum_consecutive_gaps") == 0,
    "validator duplicates": validation.get("duplicate_consecutive_points") == 0,
    "spiral speed": math.isclose(float(report.get("spiral_speed_mps", -1.0)), 1.25, abs_tol=1e-9),
    "turn speed": math.isclose(float(report.get("turn_speed_mps", -1.0)), 1.25, abs_tol=1e-9),
    "stripe speed": math.isclose(float(report.get("stripe_speed_mps", -1.0)), 1.25, abs_tol=1e-9),
    "validation minimum speed": math.isclose(float(validation.get("minimum_speed_mps", -1.0)), 1.25, abs_tol=1e-9),
    "validation maximum speed": math.isclose(float(validation.get("maximum_speed_mps", -1.0)), 1.25, abs_tol=1e-9),
}
failed = [name for name, passed in checks.items() if not passed]
if failed:
    raise SystemExit("ERROR: REVIEW_TEST gate failed: " + ", ".join(failed))
PY

if pgrep -f '[p]ython3.*field_test_logger_20260828.py' >/dev/null; then
    echo "ERROR: a field logger is already running." >&2
    exit 1
fi
if pgrep -f '[p]ython3.*pure_pursuit_controller_20260714.py' >/dev/null; then
    echo "ERROR: a Pure Pursuit controller is already running." >&2
    exit 1
fi

echo "============================================================"
echo " COMBINED 14-RING + 21-STRIPE REVIEW_TEST"
echo " Static validation : PASS"
echo " Mission waypoints : 4124"
echo " Route length      : 2028.5 m"
echo " Controller cap    : ${MAX_SPEED_MPS} m/s"
echo " AUTO command      : 1.25 m/s at every waypoint"
echo " Expected actual   : approximately 1.0 m/s from Manual comparison"
echo " Expected duration : approximately 34 minutes at measured speed"
echo " Transition        : 12.30 m contained LRL"
echo " Minimum radius    : 1.88 m sampled"
echo "============================================================"
echo "This is a supervised test mission, not a production release."
echo "Keep the mower deck disengaged for the entire first run."
echo "Remain beside the e-stop and be ready to select Pause."
echo

if [[ -f /home/al/mission_preflight_20260804.py ]]; then
    echo "Running mission preflight; keep the tractor in Pause."
    sudo python3 /home/al/mission_preflight_20260804.py
else
    echo "ERROR: /home/al/mission_preflight_20260804.py is missing." >&2
    exit 1
fi

echo
echo "Checking position and heading against the mission start..."
python3 - <<'PY'
import json
import math
import socket
import time

START_LAT = 40.485616704
START_LON = -80.332356671
START_HEADING = 163.81
MAX_DISTANCE_M = 1.5
MAX_HEADING_ERROR_DEG = 20.0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", 6009))
sock.settimeout(5.0)
latest = None
deadline = time.time() + 5.0
while time.time() < deadline:
    try:
        data, _ = sock.recvfrom(65535)
        latest = json.loads(data)
    except socket.timeout:
        break
sock.close()
if latest is None:
    raise SystemExit("ERROR: no GPS packet received on UDP 6009")
lat = float(latest["lat"])
lon = float(latest["lon"])
heading = float(latest["heading_deg"])
east = (lon - START_LON) * 111320.0 * math.cos(math.radians(START_LAT))
north = (lat - START_LAT) * 110540.0
distance = math.hypot(east, north)
heading_error = abs((heading - START_HEADING + 180.0) % 360.0 - 180.0)
print(f"Current: lat={lat:.9f}, lon={lon:.9f}, heading={heading:.1f} deg")
print(f"Start error: distance={distance:.2f} m, heading={heading_error:.1f} deg")
if latest.get("fix_quality") != "RTK Fixed" or not latest.get("headValid"):
    raise SystemExit("ERROR: start gate requires RTK Fixed and headValid")
if latest.get("carrier") != "fixed":
    raise SystemExit(f"ERROR: start gate requires fixed heading carrier, got {latest.get('carrier')!r}")
if distance > MAX_DISTANCE_M:
    raise SystemExit(f"ERROR: {distance:.2f} m from start; maximum is {MAX_DISTANCE_M:.2f} m")
if heading_error > MAX_HEADING_ERROR_DEG:
    raise SystemExit(
        f"ERROR: heading error {heading_error:.1f} deg; maximum is {MAX_HEADING_ERROR_DEG:.1f} deg"
    )
print("PASS: position, RTK fix, carrier, and heading are suitable for the test start.")
PY

echo
echo "Review plot before continuing:"
echo "  ${SCRIPT_DIR}/combined_review_test_preview_20260830.png"
read -r -p 'Type RUN REVIEW TEST BLADES OFF to start: ' confirmation
if [[ "${confirmation}" != "RUN REVIEW TEST BLADES OFF" ]]; then
    echo "Aborted; nothing was started."
    exit 1
fi

mkdir -p /home/al/field_logs/20260830_combined_review_test
timestamp="$(date '+%Y%m%d_%H%M%S')"
field_log="/home/al/field_logs/20260830_combined_review_test/combined_14ring_21stripe_${timestamp}.csv"
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
echo "Starting Pure Pursuit; remain in Pause until controller output is reviewed."
echo "Ctrl+C stops the controller and field logger."
python3 -u "${CONTROLLER}" \
    "${MISSION}" \
    --mode live \
    --gps-port 6010 \
    --min-fix "RTK Fixed" \
    --ip 127.0.0.1 \
    --port 6004 \
    --max-speed "${MAX_SPEED_MPS}"
