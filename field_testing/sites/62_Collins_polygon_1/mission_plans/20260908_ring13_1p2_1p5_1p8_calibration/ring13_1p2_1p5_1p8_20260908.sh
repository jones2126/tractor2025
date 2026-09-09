#!/usr/bin/env bash
# Supervised, blades-off launcher for the Ring 13 1.2/1.5/1.8 m/s test.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
BUILDER="${SCRIPT_DIR}/build_ring13_1p2_1p5_1p8_mission_20260908.py"
MISSION="${SCRIPT_DIR}/62_Collins_ring13_1p2_1p5_1p8_settings_20260908.txt"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260714.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260828.py"
PREFLIGHT="${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_20260804.py"
MAX_SPEED_MPS="1.80"

for required in "${BUILDER}" "${CONTROLLER}" "${LOGGER}" "${PREFLIGHT}"; do
    if [[ ! -f "${required}" ]]; then
        echo "ERROR: required file not found: ${required}" >&2
        exit 1
    fi
done

# Rebuild from the reviewed historical geometry every run. The builder verifies
# the historical source SHA before writing the new mission.
python3 "${BUILDER}"

python3 - "${MISSION}" <<'PY'
from collections import Counter
from pathlib import Path
import sys

mission = Path(sys.argv[1])
lines = mission.read_text(encoding="utf-8").splitlines()
if len(lines) != 506:
    raise SystemExit(f"ERROR: expected 506 mission rows, got {len(lines)}")

rows = [line.split() for line in lines]
if any(len(row) != 5 for row in rows):
    raise SystemExit("ERROR: mission contains a row that does not have 5 columns")

checks = [
    (1, 66, "0.75", "transit"),
    (67, 212, "1.20", "lap 1"),
    (213, 359, "1.50", "lap 2"),
    (360, 506, "1.80", "lap 3"),
]
for start, end, expected, label in checks:
    values = {rows[i - 1][4] for i in range(start, end + 1)}
    if values != {expected}:
        raise SystemExit(f"ERROR: {label} speed mismatch: {sorted(values)}")

counts = Counter(row[4] for row in rows)
expected_counts = Counter({"0.75": 66, "1.20": 146, "1.50": 147, "1.80": 147})
if counts != expected_counts:
    raise SystemExit(f"ERROR: unexpected speed counts: {dict(counts)}")

if any(row[3] != "1.50" for row in rows[:66]):
    raise SystemExit("ERROR: transit lookahead is not 1.50 m")
if any(row[3] != "2.00" for row in rows[66:]):
    raise SystemExit("ERROR: Ring 13 lookahead is not 2.00 m")

print("PASS: generated mission has 506 rows and the 1.2/1.5/1.8 schedule.")
PY

if [[ "${1:-}" == "--build-only" ]]; then
    echo "Build-only requested; mission generated and validated."
    exit 0
fi

if [[ $# -gt 0 ]]; then
    echo "Usage: $0 [--build-only]" >&2
    exit 2
fi

if pgrep -f '[p]ython3.*field_test_logger_20260828.py' >/dev/null; then
    echo "ERROR: a field logger is already running." >&2
    exit 1
fi
if pgrep -f '[p]ython3.*pure_pursuit_controller_20260714.py' >/dev/null; then
    echo "ERROR: a Pure Pursuit controller is already running." >&2
    exit 1
fi

echo "============================================================"
echo " RING 13 1.2 / 1.5 / 1.8 M/S CALIBRATION - 2026-09-08"
echo " Geometry source    : validated 2026-08-31 Ring 13 mission"
echo " Mission waypoints  : 506"
echo " Route length       : approximately 236.0 m"
echo " Start transit      : 0.75 m/s command -> Teensy JRK 2368"
echo " Lap 1              : 1.20 m/s command -> Teensy JRK 2233"
echo " Lap 2              : 1.50 m/s command -> Teensy JRK 2178"
echo " Lap 3              : 1.80 m/s command -> Teensy JRK 2135"
echo " Soft limit margin  : JRK 2135 is 255 counts above soft limit 1880"
echo " Ring lookahead     : 2.0 m"
echo "============================================================"
echo "This is a supervised calibration mission, not a production release."
echo "Keep the mower deck disengaged for the entire run."
echo "Remain beside the e-stop and be ready to select Pause."
echo "The 1.80 m/s lap is a first moving test at that command."
echo "Abort for abnormal current, motion, heading loss, tracking, or stopping margin."
echo

echo "Running mission preflight; keep the tractor in Pause."
sudo python3 "${PREFLIGHT}" --expected-firmware teensy_main_20260908_1p8_test

echo
echo "Checking position and heading against the known mission start..."
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
    raise SystemExit(
        f"ERROR: start gate requires fixed heading carrier, got {latest.get('carrier')!r}"
    )
if distance > MAX_DISTANCE_M:
    raise SystemExit(f"ERROR: {distance:.2f} m from start; maximum is {MAX_DISTANCE_M:.2f} m")
if heading_error > MAX_HEADING_ERROR_DEG:
    raise SystemExit(
        f"ERROR: heading error {heading_error:.1f} deg; maximum is {MAX_HEADING_ERROR_DEG:.1f} deg"
    )

print("PASS: position, RTK fix, carrier, and heading are suitable for the known start.")
PY

echo
read -r -p 'Type RUN RING13 1P8 SPEED TEST BLADES OFF to start: ' confirmation
if [[ "${confirmation}" != "RUN RING13 1P8 SPEED TEST BLADES OFF" ]]; then
    echo "Aborted; nothing was started."
    exit 1
fi

mkdir -p /home/al/field_logs/20260908_1p8_speed_calibration
timestamp="$(date '+%Y%m%d_%H%M%S')"
field_log="/home/al/field_logs/20260908_1p8_speed_calibration/ring13_1p2_1p5_1p8_${timestamp}.csv"
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
