#!/usr/bin/env bash
# Supervised, blades-off complete backyard mission at 1.00 m/s.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
BUILDER="${SCRIPT_DIR}/build_complete_back_yard_mission_20260909.py"
MISSION="${SCRIPT_DIR}/62_Collins_complete_back_yard_1mps_20260909.txt"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260714.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260828.py"
PREFLIGHT="${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_20260804.py"

dashboard_mode=false
build_only=false
if [[ "${1:-}" == "--build-only" ]]; then
    build_only=true
    shift
elif [[ "${1:-}" == "--dashboard" ]]; then
    dashboard_mode=true
    shift
fi
[[ $# -eq 0 ]] || { echo "Usage: $0 [--build-only|--dashboard]" >&2; exit 2; }

for required in "${BUILDER}" "${MISSION}" "${CONTROLLER}" "${LOGGER}" "${PREFLIGHT}"; do
    [[ -f "${required}" ]] || { echo "ERROR: required file not found: ${required}" >&2; exit 1; }
done

if [[ "${build_only}" == true ]]; then
    python3 "${BUILDER}"
else
    echo "Using the reviewed committed mission; verifying its exact checksum and contents."
fi
python3 - "${MISSION}" <<'PY'
import hashlib
from pathlib import Path
import math
import sys

mission_path = Path(sys.argv[1])
mission_bytes = mission_path.read_bytes().replace(b"\r\n", b"\n")
mission_sha256 = hashlib.sha256(mission_bytes).hexdigest()
expected_sha256 = "e089ce42dedd821b4705281bfeba5c9d7d5782877e1b835234df484b68d39cd6"
if mission_sha256 != expected_sha256:
    raise SystemExit(
        f"ERROR: mission checksum is {mission_sha256}; expected reviewed mission {expected_sha256}"
    )
rows = [line.split() for line in mission_bytes.decode("utf-8").splitlines()]
if len(rows) != 5553 or any(len(row) != 5 for row in rows):
    raise SystemExit(f"ERROR: expected 5553 five-column mission rows, got {len(rows)}")
if {row[4] for row in rows} != {"1.00"}:
    raise SystemExit("ERROR: every mission speed must be 1.00 m/s")
if any(not all(math.isfinite(float(value)) for value in row) for row in rows):
    raise SystemExit("ERROR: mission contains a non-finite value")
print("PASS: complete backyard mission has 5553 valid rows at 1.00 m/s.")
PY

if [[ "${build_only}" == true ]]; then
    echo "Build-only requested; mission generated and validated."
    exit 0
fi

pgrep -f '[p]ython3.*field_test_logger_20260828.py' >/dev/null && { echo "ERROR: a field logger is already running." >&2; exit 1; }
pgrep -f '[p]ython3.*pure_pursuit_controller_20260714.py' >/dev/null && { echo "ERROR: Pure Pursuit is already running." >&2; exit 1; }

echo "============================================================"
echo " COMPLETE 62 COLLINS BACK YARD TEST - 1.00 M/S"
echo " Polygon 1: 14 clockwise rings and 21 stripes"
echo " Polygon 2: outer boundary and 1 inner ring"
echo " Polygon 3: outer boundary and 2 inner rings"
echo " Route      : approximately 2.37 km / 39.5 min moving"
echo " Mower deck must remain disengaged"
echo "============================================================"
echo "Running preflight; keep the tractor in Pause."
if [[ "${dashboard_mode}" == true ]]; then
    # The dashboard cannot answer an interactive sudo prompt. Preflight only
    # performs read-only checks and fails closed if any required data is unavailable.
    python3 "${PREFLIGHT}" --expected-firmware teensy_main_20260908_1p8_test
else
    sudo python3 "${PREFLIGHT}" --expected-firmware teensy_main_20260908_1p8_test
fi

echo "Checking position and heading against the mission start..."
python3 - "${MISSION}" <<'PY'
import json, math, socket, sys, time

first = open(sys.argv[1], encoding="utf-8").readline().split()
start_lat, start_lon, start_yaw = map(float, first[:3])
start_heading = (90.0 - math.degrees(start_yaw)) % 360.0
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

if [[ "${dashboard_mode}" == true ]]; then
    echo "Dashboard supplied the blades-off start confirmation."
else
    read -r -p 'Type RUN COMPLETE BACK YARD BLADES OFF to start: ' confirmation
    [[ "${confirmation}" == "RUN COMPLETE BACK YARD BLADES OFF" ]] || { echo "Aborted; nothing was started."; exit 1; }
fi

log_dir="/home/al/field_logs/20260909_complete_back_yard"
mkdir -p "${log_dir}"
field_log="${log_dir}/complete_back_yard_$(date '+%Y%m%d_%H%M%S').csv"
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
python3 -u "${CONTROLLER}" "${MISSION}" --mode live --gps-port 6010 --min-fix "RTK Fixed" --ip 127.0.0.1 --port 6004 --max-speed 1.00 --control-port 6011 --telemetry-port 6012
