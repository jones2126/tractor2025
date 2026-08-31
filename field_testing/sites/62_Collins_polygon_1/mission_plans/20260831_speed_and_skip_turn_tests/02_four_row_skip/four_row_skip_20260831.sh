#!/usr/bin/env bash
# Supervised, blades-off launcher for the shortened 2026-08-31 four-row-skip test.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
MISSION="${SCRIPT_DIR}/62_Collins_combined_14ring_four_row_skip_REVIEW_TEST_20260831.txt"
REPORT="${SCRIPT_DIR}/four_row_skip_report_20260831.json"
VALIDATION="${SCRIPT_DIR}/four_row_skip_validation_20260831.json"
PREVIEW="${SCRIPT_DIR}/four_row_skip_preview_20260831.png"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260714.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260828.py"
EXPECTED_SHA256="5a3346b8887449876883b244d134a324819127dd8b29ee9bdfac269614a13dc1"
MAX_SPEED_MPS="1.25"

for required in "${MISSION}" "${REPORT}" "${VALIDATION}" "${PREVIEW}" "${CONTROLLER}" "${LOGGER}"; do
    if [[ ! -f "${required}" ]]; then
        echo "ERROR: required file not found: ${required}" >&2
        exit 1
    fi
done

actual_sha256="$(sha256sum "${MISSION}" | awk '{print $1}')"
if [[ "${actual_sha256}" != "${EXPECTED_SHA256}" ]]; then
    echo "ERROR: mission SHA-256 does not match the reviewed shortened mission." >&2
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
transit = report.get("start_transit") or {}
checks = {
    "review-test label": report.get("status") == "REVIEW_TEST",
    "shortened spiral mode": report.get("spiral_mode") == "last-ring-from-original-start",
    "source spiral size": report.get("source_spiral_waypoints") == 2976,
    "ring 14 source index": report.get("last_ring_source_start_index") == 2859,
    "ring 14 waypoint count": report.get("included_spiral_waypoints") == 117,
    "start transit mode": transit.get("mode") == "RSL",
    "start transit containment": transit.get("contained_in_site") is True,
    "stripes": report.get("stripe_count") == 21,
    "two-arc stripe turns": report.get("all_stripe_connectors_are_two_arc_csc") is True,
    "route containment": report.get("route_contained_in_site") is True,
    "waypoints": report.get("waypoints") == 1208,
    "validation": validation.get("status") == "PASS",
    "validator waypoints": validation.get("waypoints") == 1208,
    "validator duplicates": validation.get("duplicate_consecutive_points") == 0,
    "minimum radius": float(validation.get("minimum_sampled_radius_m", 0.0)) >= 1.88,
    "spiral speed": math.isclose(float(report.get("spiral_speed_mps", -1.0)), 1.25, abs_tol=1e-9),
    "turn speed": math.isclose(float(report.get("turn_speed_mps", -1.0)), 1.25, abs_tol=1e-9),
    "stripe speed": math.isclose(float(report.get("stripe_speed_mps", -1.0)), 1.25, abs_tol=1e-9),
    "validation minimum speed": math.isclose(float(validation.get("minimum_speed_mps", -1.0)), 1.25, abs_tol=1e-9),
    "validation maximum speed": math.isclose(float(validation.get("maximum_speed_mps", -1.0)), 1.25, abs_tol=1e-9),
}
failed = [name for name, passed in checks.items() if not passed]
if failed:
    raise SystemExit("ERROR: FOUR_ROW_SKIP REVIEW_TEST gate failed: " + ", ".join(failed))
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
echo " RING 14 + FOUR-ROW-SKIP REVIEW_TEST"
echo " Static validation : PASS"
echo " Mission waypoints : 1208"
echo " Route length      : 553.6 m"
echo " Controller cap    : ${MAX_SPEED_MPS} m/s"
echo " AUTO command      : 1.25 m/s at every waypoint"
echo " Expected actual   : approximately 1.0 m/s from 2026-08-30"
echo " Expected duration : approximately 9-10 minutes"
echo " Start transit     : 16.45 m contained RSL to ring 14"
echo " Spiral coverage   : ring 14 only; rings 1-13 are omitted"
echo " Stripe pattern    : 21 stripes; four-row advancement"
echo " Stripe turns      : 20 two-arc LSL/RSR; no keyholes"
echo " Turn lookahead    : 1.5 m"
echo " Minimum radius    : 1.89 m sampled"
echo "============================================================"
echo "This is a supervised test mission, not a production release."
echo "Keep the mower deck disengaged for the entire run."
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
echo "Checking position and heading against the original spiral start..."
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
echo "  ${PREVIEW}"
read -r -p 'Type RUN FOUR ROW SKIP TEST BLADES OFF to start: ' confirmation
if [[ "${confirmation}" != "RUN FOUR ROW SKIP TEST BLADES OFF" ]]; then
    echo "Aborted; nothing was started."
    exit 1
fi

mkdir -p /home/al/field_logs/20260831_four_row_skip
timestamp="$(date '+%Y%m%d_%H%M%S')"
field_log="/home/al/field_logs/20260831_four_row_skip/ring14_four_row_skip_${timestamp}.csv"
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
