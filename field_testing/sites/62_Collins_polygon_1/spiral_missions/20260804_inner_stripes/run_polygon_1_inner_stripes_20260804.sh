#!/usr/bin/env bash
# Supervised launcher for the inner-core east/west stripe completion mission.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
MISSION="${SCRIPT_DIR}/polygon_1_inner_stripes_38in_20260804.txt"
REPORT="${SCRIPT_DIR}/polygon_1_inner_stripes_38in_report.json"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260714.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260804.py"
EXPECTED_SHA256="b9c9ff5b18be1cba7f725b63ba17f561c72769409ee59e2e72fd144b0181c028"

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
    "spacing": math.isclose(report["lane_spacing_m"], 0.9652, abs_tol=1e-9),
    "stripes": report["included_stripes"] == 20,
    "connectors": len(report["connectors"]) == 20,
    "connector location": report["all_connectors_outside_core"] is True,
    "containment": report["route_contained_in_site"] is True,
    "waypoints": report["waypoints"] == 1010,
    "straight speed": math.isclose(report["straight_speed_mps"], 0.85, abs_tol=1e-9),
    "turn speed": math.isclose(report["turn_speed_mps"], 0.50, abs_tol=1e-9),
    "lookahead": math.isclose(report["lookahead_m"], 1.00, abs_tol=1e-9),
}
failed = [name for name, passed in checks.items() if not passed]
if failed:
    raise SystemExit("ERROR: reviewed report check failed: " + ", ".join(failed))
PY

if pgrep -f '[p]ython3.*field_test_logger_20260804.py' >/dev/null; then
    echo "ERROR: a field logger is already running." >&2
    exit 1
fi
if pgrep -f '[p]ython3.*pure_pursuit_controller_20260714.py' >/dev/null; then
    echo "ERROR: a Pure Pursuit controller is already running." >&2
    exit 1
fi

echo "============================================================"
echo " POLYGON 1 INNER-CORE STRIPE COMPLETION"
echo " Stripes          : 20 east/west at 38-inch spacing"
echo " Straight speed   : 0.85 m/s"
echo " Keyhole speed    : 0.50 m/s"
echo " Keyhole radius   : 1.90 m"
echo " Lookahead        : 1.00 m"
echo " Route length     : 472.2 m"
echo " Estimated runtime: 12.8 minutes"
echo " Start            : 40.485562833, -80.332340333"
echo " Start heading    : 162 degrees compass (south-southeast)"
echo "============================================================"
echo "Every strip reaches the core edge before its keyhole begins."
echo "All keyholes remain outside the core and inside the site boundary."
echo "Keep the mower deck disengaged for the first supervised run."
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

read -r -p 'Type RUN INNER STRIPES to start logger and controller: ' confirmation
if [[ "${confirmation}" != "RUN INNER STRIPES" ]]; then
    echo "Aborted; nothing was started."
    exit 1
fi

mkdir -p /home/al/field_logs
timestamp="$(date '+%Y%m%d_%H%M%S')"
field_log="/home/al/field_logs/polygon_1_inner_stripes_${timestamp}.csv"
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
    --max-speed 0.85
