#!/usr/bin/env bash
# Supervised launcher for the 2026-08-04 clockwise inward spiral.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
MISSION="${SCRIPT_DIR}/polygon_1_clockwise_inward_38in_20260804.txt"
REPORT="${SCRIPT_DIR}/polygon_1_clockwise_inward_38in_report.json"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260714.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260828.py"
EXPECTED_SHA256="5fc96fa208c7d797e622be14aed37e117f9e3462502b0a3c67064219392b25a8"
MAX_SPEED_MPS="0.85"

for required in "${MISSION}" "${REPORT}" "${CONTROLLER}" "${LOGGER}"; do
    if [[ ! -f "${required}" ]]; then
        echo "ERROR: required file not found: ${required}" >&2
        exit 1
    fi
done

actual_sha256="$(sha256sum "${MISSION}" | awk '{print $1}')"
if [[ "${actual_sha256}" != "${EXPECTED_SHA256}" ]]; then
    echo "ERROR: mission SHA-256 does not match the reviewed mission." >&2
    echo "Expected: ${EXPECTED_SHA256}" >&2
    echo "Actual  : ${actual_sha256}" >&2
    exit 1
fi

python3 - "${REPORT}" <<'PY'
import json, math, pathlib, sys
report = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
checks = {
    "speed": math.isclose(report["speed_mps"], 0.85, abs_tol=1e-9),
    "spacing": math.isclose(report["lane_spacing_m"], 0.9652, abs_tol=1e-9),
    "containment": math.isclose(
        report["outside_boundary_length_m_with_2cm_tolerance"], 0.0,
        abs_tol=1e-9,
    ),
    "curvature": report["minimum_sampled_turn_radius_m"] >= 2.10,
    "waypoints": report["waypoints"] == 2846,
}
failed = [name for name, passed in checks.items() if not passed]
if failed:
    raise SystemExit("ERROR: reviewed report check failed: " + ", ".join(failed))
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
echo " POLYGON 1 CLOCKWISE CONTINUOUS INWARD SPIRAL"
echo " Route length     : 1422.2 m"
echo " Estimated runtime: 27.9 minutes at 0.85 m/s"
echo " Path spacing     : 0.9652 m (38 inches)"
echo " Waypoints        : 2846"
echo " Minimum radius   : 2.13 m sampled"
echo " Start            : 40.485562833, -80.332340333"
echo " Start heading    : 162 degrees compass (south-southeast)"
echo " Autonomous route : continuous clockwise inward spiral"
echo "============================================================"
echo "WARNING: the source lap includes RTK Float/DGPS gaps."
echo "Keep the mower deck disengaged for the first supervised run."
echo "Remain beside the e-stop and be ready to select Pause."
echo

if [[ -f /home/al/mission_preflight_20260804.py ]]; then
    echo "Running mission preflight; keep the switch UP/Pause."
    sudo python3 /home/al/mission_preflight_20260804.py
else
    echo "ERROR: /home/al/mission_preflight_20260804.py is missing." >&2
    exit 1
fi

echo
echo "Checking current position and heading against the mission start..."
python3 - <<'PY'
import json, math, socket, time

START_LAT = 40.485562833
START_LON = -80.332340333
START_HEADING = 162.0
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
if distance > MAX_DISTANCE_M:
    raise SystemExit(f"ERROR: {distance:.2f} m from start; maximum is {MAX_DISTANCE_M:.2f} m")
if heading_error > MAX_HEADING_ERROR_DEG:
    raise SystemExit(
        f"ERROR: heading error {heading_error:.1f} deg; maximum is {MAX_HEADING_ERROR_DEG:.1f} deg"
    )
print("PASS: position and heading are suitable for supervised start.")
PY

read -r -p 'Type RUN CLOCKWISE SPIRAL to start logger and controller: ' confirmation
if [[ "${confirmation}" != "RUN CLOCKWISE SPIRAL" ]]; then
    echo "Aborted; nothing was started."
    exit 1
fi

mkdir -p /home/al/field_logs
timestamp="$(date '+%Y%m%d_%H%M%S')"
field_log="/home/al/field_logs/polygon_1_clockwise_spiral_${timestamp}.csv"
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
echo "Starting Pure Pursuit; remain UP/Pause until controller output is reviewed."
python3 -u "${CONTROLLER}" \
    "${MISSION}" \
    --mode live \
    --gps-port 6010 \
    --min-fix "RTK Fixed" \
    --ip 127.0.0.1 \
    --port 6004 \
    --max-speed "${MAX_SPEED_MPS}"
