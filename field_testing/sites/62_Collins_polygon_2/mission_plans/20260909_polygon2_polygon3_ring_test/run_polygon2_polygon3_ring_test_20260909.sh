#!/usr/bin/env bash
# Supervised, blades-off Polygon 2 -> Polygon 3 ring test at 1.00 m/s.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
BUILDER="${SCRIPT_DIR}/build_polygon2_polygon3_ring_mission_20260909.py"
MISSION="${SCRIPT_DIR}/62_Collins_polygon2_polygon3_rings_1mps_20260909.txt"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260714.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260828.py"
PREFLIGHT="${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_20260804.py"

for required in "${BUILDER}" "${CONTROLLER}" "${LOGGER}" "${PREFLIGHT}"; do
    [[ -f "${required}" ]] || { echo "ERROR: required file not found: ${required}" >&2; exit 1; }
done

python3 "${BUILDER}"
python3 - "${MISSION}" <<'PY'
from pathlib import Path
import math
import sys

rows = [line.split() for line in Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()]
if len(rows) != 844 or any(len(row) != 5 for row in rows):
    raise SystemExit(f"ERROR: expected 844 five-column mission rows, got {len(rows)}")
if {row[4] for row in rows} != {"1.00"}:
    raise SystemExit("ERROR: every mission speed must be 1.00 m/s")
if {row[3] for row in rows} != {"2.00"}:
    raise SystemExit("ERROR: every mission lookahead must be 2.00 m")
if any(not all(math.isfinite(float(value)) for value in row) for row in rows):
    raise SystemExit("ERROR: mission contains a non-finite value")
print("PASS: 844 mission rows, all at 1.00 m/s with 2.00 m lookahead.")
PY

if [[ "${1:-}" == "--build-only" ]]; then
    echo "Build-only requested; mission generated and validated."
    exit 0
fi
[[ $# -eq 0 ]] || { echo "Usage: $0 [--build-only]" >&2; exit 2; }

pgrep -f '[p]ython3.*field_test_logger_20260828.py' >/dev/null && { echo "ERROR: a field logger is already running." >&2; exit 1; }
pgrep -f '[p]ython3.*pure_pursuit_controller_20260714.py' >/dev/null && { echo "ERROR: Pure Pursuit is already running." >&2; exit 1; }

echo "============================================================"
echo " POLYGON 2 -> POLYGON 3 RING TEST - 1.00 M/S"
echo " Polygon 2: outer boundary plus 1 inner ring"
echo " Transfer : recorded RTK-Fixed path from 2026-09-08"
echo " Polygon 3: outer boundary plus 2 inner rings"
echo " Mower deck must remain disengaged"
echo "============================================================"
echo "Running preflight; keep the tractor in Pause."
sudo python3 "${PREFLIGHT}" --expected-firmware teensy_main_20260908_1p8_test

echo "Checking position and heading against the generated mission start..."
python3 - "${MISSION}" <<'PY'
import json, math, socket, sys, time

first = open(sys.argv[1], encoding="utf-8").readline().split()
start_lat, start_lon, start_yaw = map(float, first[:3])
start_heading = math.degrees(start_yaw) % 360.0
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", 6009)); sock.settimeout(5.0)
latest = None; deadline = time.time() + 5.0
while time.time() < deadline:
    try: latest = json.loads(sock.recvfrom(65535)[0])
    except socket.timeout: break
sock.close()
if latest is None: raise SystemExit("ERROR: no GPS packet received on UDP 6009")
lat, lon, heading = float(latest["lat"]), float(latest["lon"]), float(latest["heading_deg"])
east = (lon - start_lon) * 111320.0 * math.cos(math.radians(start_lat))
north = (lat - start_lat) * 110540.0
position_error = math.hypot(east, north)
heading_error = abs((heading - start_heading + 180.0) % 360.0 - 180.0)
print(f"Start target: lat={start_lat:.9f}, lon={start_lon:.9f}, heading={start_heading:.1f} deg")
print(f"Start error: distance={position_error:.2f} m, heading={heading_error:.1f} deg")
if latest.get("fix_quality") != "RTK Fixed" or not latest.get("headValid") or latest.get("carrier") != "fixed":
    raise SystemExit("ERROR: start gate requires RTK Fixed, headValid, and fixed heading carrier")
if position_error > 1.5: raise SystemExit("ERROR: move within 1.50 m of the mission start")
if heading_error > 20.0: raise SystemExit("ERROR: align within 20 degrees of the mission start heading")
print("PASS: position and heading are suitable for mission start.")
PY

read -r -p 'Type RUN POLYGON 2 AND 3 RINGS BLADES OFF to start: ' confirmation
[[ "${confirmation}" == "RUN POLYGON 2 AND 3 RINGS BLADES OFF" ]] || { echo "Aborted; nothing was started."; exit 1; }

log_dir="/home/al/field_logs/20260909_polygon2_polygon3_ring_test"
mkdir -p "${log_dir}"
field_log="${log_dir}/polygon2_polygon3_rings_$(date '+%Y%m%d_%H%M%S').csv"
logger_pid=""
cleanup() {
    if [[ -n "${logger_pid}" ]] && kill -0 "${logger_pid}" 2>/dev/null; then
        echo; echo "Stopping field logger (PID ${logger_pid})..."
        kill "${logger_pid}"; wait "${logger_pid}" 2>/dev/null || true
    fi
    echo "Field log: ${field_log}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

python3 -u "${LOGGER}" --output "${field_log}" &
logger_pid=$!
sleep 2
kill -0 "${logger_pid}" 2>/dev/null || { echo "ERROR: field logger stopped during startup." >&2; exit 1; }
echo "Logger running. Starting Pure Pursuit; remain ready to select Pause."
python3 -u "${CONTROLLER}" "${MISSION}" --mode live --gps-port 6010 --min-fix "RTK Fixed" --ip 127.0.0.1 --port 6004 --max-speed 1.00
