#!/usr/bin/env bash
# Supervised straight-line diagnostic for headValid behavior. No turns.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
MISSION="${SCRIPT_DIR}/headvalid_straight_8m_20260805.txt"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260714.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260804.py"
PREFLIGHT="/home/al/mission_preflight_20260804.py"
EXPECTED_SHA256="e46a7af2735650be832dac621210f7dbf9c7a6526bd6ff1c1d16eea49ff3ecc3"

for required in "${MISSION}" "${CONTROLLER}" "${LOGGER}" "${PREFLIGHT}"; do
    if [[ ! -f "${required}" ]]; then
        echo "ERROR: required file not found: ${required}" >&2
        exit 1
    fi
done

actual_sha256="$(sha256sum "${MISSION}" | awk '{print $1}')"
if [[ "${actual_sha256}" != "${EXPECTED_SHA256}" ]]; then
    echo "ERROR: diagnostic mission SHA-256 mismatch." >&2
    echo "Expected: ${EXPECTED_SHA256}" >&2
    echo "Actual  : ${actual_sha256}" >&2
    exit 1
fi

if pgrep -f '[p]ython3.*field_test_logger_20260804.py' >/dev/null; then
    echo "ERROR: a field logger is already running." >&2
    exit 1
fi
if pgrep -f '[p]ython3.*pure_pursuit_controller_20260714.py' >/dev/null; then
    echo "ERROR: a Pure Pursuit controller is already running." >&2
    exit 1
fi

echo "============================================================"
echo " HEADVALID STRAIGHT DIAGNOSTIC"
echo " Distance         : 15.0 m, straight, no turn"
echo " Heading          : 170 degrees compass"
echo " Speed            : 0.50 m/s"
echo " Lookahead        : 2.00 m"
echo " Expected runtime : about 30 seconds"
echo " Start            : 40.485562833, -80.332340333"
echo "============================================================"
echo "Keep the mower deck disengaged and radio UP/Pause."
echo "This test still exercises the unresolved JRK feedback system."
echo "Use Pause immediately for any jerk, heading dropout, or unexpected motion."
echo

python3 "${PREFLIGHT}"

python3 - <<'PY'
import json, math, socket, time

START_LAT = 40.485562833
START_LON = -80.332340333
START_HEADING = 170.0
MAX_DISTANCE_M = 3.0
MAX_HEADING_ERROR_DEG = 10.0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
if hasattr(socket, "SO_REUSEPORT"):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
sock.bind(("", 6009))
sock.settimeout(5.0)
latest = None
latest_drivable = None
deadline = time.time() + 5.0
while time.time() < deadline:
    try:
        payload, _ = sock.recvfrom(65535)
        latest = json.loads(payload)
        if latest.get("fix_quality") == "RTK Fixed" and latest.get("headValid"):
            latest_drivable = latest
    except socket.timeout:
        break
sock.close()
if latest is None:
    raise SystemExit("ERROR: no GPS packet received on UDP 6009")
if latest_drivable is None:
    raise SystemExit("ERROR: no RTK Fixed + headValid packet received during the 5-second start gate")
latest = latest_drivable
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
    raise SystemExit(f"ERROR: heading error {heading_error:.1f} deg; maximum is {MAX_HEADING_ERROR_DEG:.1f} deg")
print("PASS: position and heading are suitable for the diagnostic.")
PY

read -r -p 'Type RUN HEADVALID STRAIGHT to start logger and controller: ' confirmation
if [[ "${confirmation}" != "RUN HEADVALID STRAIGHT" ]]; then
    echo "Aborted; nothing was started."
    exit 1
fi

mkdir -p /home/al/field_logs
timestamp="$(date '+%Y%m%d_%H%M%S')"
field_log="/home/al/field_logs/headvalid_straight_15m_${timestamp}.csv"
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
echo "Starting controller; remain UP/Pause until output is reviewed."
python3 -u "${CONTROLLER}" \
    "${MISSION}" \
    --mode live \
    --gps-port 6010 \
    --min-fix "RTK Fixed" \
    --ip 127.0.0.1 \
    --port 6004 \
    --max-speed 0.50
