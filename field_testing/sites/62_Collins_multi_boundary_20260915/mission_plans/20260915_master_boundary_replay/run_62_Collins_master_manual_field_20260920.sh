#!/usr/bin/env bash
# Supervised blades-off field launcher for the reviewed, resampled master route.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
GENERATED="${SCRIPT_DIR}/generated_master_manual_field_20260920"
MISSION="${GENERATED}/62_Collins_master_manual_resampled_1mps_20260920.txt"
AUDIT="${GENERATED}/62_Collins_master_manual_resampled_1mps_20260920_audit.csv"
REPORT="${GENERATED}/62_Collins_master_manual_resampled_1mps_20260920_report.json"
VERIFY="${SCRIPT_DIR}/verify_master_manual_field_20260920.py"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260915.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260828.py"
PREFLIGHT="${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_20260804.py"

verify_only=false
dashboard_mode=false
case "${1:-}" in
    --verify-only) verify_only=true; shift ;;
    --dashboard) dashboard_mode=true; shift ;;
esac
[[ $# -eq 0 ]] || { echo "Usage: $0 [--verify-only|--dashboard]" >&2; exit 2; }

for required in "${MISSION}" "${AUDIT}" "${REPORT}" "${VERIFY}" "${CONTROLLER}" "${LOGGER}" "${PREFLIGHT}"; do
    [[ -f "${required}" ]] || { echo "ERROR: required file not found: ${required}" >&2; exit 1; }
done
python3 "${VERIFY}"
if [[ "${verify_only}" == true ]]; then
    echo "Verification-only requested; no services or controller were started."
    exit 0
fi

pgrep -f '[p]ython3.*field_test_logger_20260828.py' >/dev/null && {
    echo "ERROR: a field logger is already running." >&2; exit 1;
}
pgrep -f '[p]ython3.*pure_pursuit_controller_20260915.py' >/dev/null && {
    echo "ERROR: a Pure Pursuit controller is already running." >&2; exit 1;
}

echo "============================================================"
echo " 62 COLLINS REVIEWED MASTER + RECORDED MANUAL STRIPES"
echo " Start       : original W1 (the archived resume mission start)"
echo " Included    : all reviewed route segments and six circle/detour cuts"
echo " Manual      : M18-M2860, distance-resampled to about 0.15 m"
echo " Speed       : 1.00 m/s at every waypoint; 2.00 m lookahead"
echo " Recovery    : handheld mode cycle required after safety loss"
echo " Mower deck must remain DISENGAGED throughout this test"
echo " Keep the handheld available for Pause/Manual at all times"
echo "============================================================"
echo "Running preflight; keep the tractor in Pause."
if [[ "${dashboard_mode}" == true ]]; then
    python3 "${PREFLIGHT}" --expected-firmware teensy_main_20260914
else
    sudo python3 "${PREFLIGHT}" --expected-firmware teensy_main_20260914
fi

echo "Checking position and heading against original W1 on UDP 6010..."
python3 - "${MISSION}" <<'PY'
import json
import math
import socket
import sys
import time

first = open(sys.argv[1], encoding="utf-8").readline().split()
start_lat, start_lon, start_yaw = map(float, first[:3])
start_heading = (90.0 - math.degrees(start_yaw)) % 360.0
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", 6010))
sock.settimeout(5.0)
latest = None
deadline = time.monotonic() + 5.0
while time.monotonic() < deadline:
    try:
        latest = json.loads(sock.recvfrom(65535)[0])
    except socket.timeout:
        break
sock.close()
if latest is None:
    raise SystemExit("ERROR: no navigation GPS packet received on UDP 6010")

lat = float(latest["lat"]); lon = float(latest["lon"])
heading = float(latest["heading_deg"])
east = (lon - start_lon) * 111320.0 * math.cos(math.radians(start_lat))
north = (lat - start_lat) * 110540.0
position_error = math.hypot(east, north)
heading_error = abs((heading - start_heading + 180.0) % 360.0 - 180.0)
print(f"Start target: lat={start_lat:.9f}, lon={start_lon:.9f}, heading={start_heading:.1f} deg")
print(f"Start error: distance={position_error:.2f} m, heading={heading_error:.1f} deg")
if latest.get("fix_quality") != "RTK Fixed":
    raise SystemExit("ERROR: mission start requires RTK Fixed")
if not latest.get("headValid") or str(latest.get("carrier", "")).lower() != "fixed":
    raise SystemExit("ERROR: mission start requires valid fixed-carrier heading")
baseline = latest.get("relpos_length_m")
accuracy = latest.get("relpos_heading_accuracy_deg")
if baseline is None or not 0.80 <= float(baseline) <= 1.30:
    raise SystemExit(f"ERROR: heading baseline {baseline!r} m is outside 0.80-1.30 m")
if accuracy is None or float(accuracy) > 1.0:
    raise SystemExit(f"ERROR: heading accuracy {accuracy!r} deg exceeds 1.0 deg")
if position_error > 1.5:
    raise SystemExit("ERROR: move within 1.50 m of W1 before starting")
if heading_error > 20.0:
    raise SystemExit("ERROR: align within 20 degrees of the W1 heading")
print("PASS: position and heading are suitable for mission start.")
PY

if [[ "${dashboard_mode}" == true ]]; then
    echo "Dashboard supplied the blades-off confirmation."
else
    read -r -p 'Type RUN MASTER MANUAL FIELD BLADES OFF to start: ' confirmation
    [[ "${confirmation}" == "RUN MASTER MANUAL FIELD BLADES OFF" ]] || {
        echo "Aborted; nothing was started."; exit 1;
    }
fi

log_dir="/home/al/field_logs/20260920_master_manual_field_1mps"
mkdir -p "${log_dir}"
field_log="${log_dir}/master_manual_field_$(date '+%Y%m%d_%H%M%S').csv"
logger_pid=""
cleanup() {
    if [[ -n "${logger_pid}" ]] && kill -0 "${logger_pid}" 2>/dev/null; then
        echo ""
        echo "Stopping field logger (PID ${logger_pid})..."
        kill "${logger_pid}"
        wait "${logger_pid}" 2>/dev/null || true
    fi
    echo "Field log: ${field_log}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

python3 -u "${LOGGER}" --output "${field_log}" &
logger_pid=$!
sleep 2
kill -0 "${logger_pid}" 2>/dev/null || {
    echo "ERROR: field logger stopped during startup." >&2; exit 1;
}

echo "Logger running. Starting Pure Pursuit; retain handheld Pause until controller status is reviewed."
controller_args=(
    python3 -u "${CONTROLLER}" "${MISSION}"
    --mode live
    --gps-port 6010
    --status-port 6003
    --min-fix "RTK Fixed"
    --ip 127.0.0.1
    --port 6004
    --max-speed 1.00
    --tracking-window 6.0
    --audit-file "${AUDIT}"
    --reacquire-max-advance 5.0
    --resume-stable-seconds 0.0
    --basic-runtime-heading-gate
    --no-operator-cycle-after-safety-loss
)
if [[ "${dashboard_mode}" == true ]]; then
    controller_args+=(--control-port 6011 --telemetry-port 6012)
fi
"${controller_args[@]}"
