#!/usr/bin/env bash
# Approved launcher for the initial supervised blades-off field test.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
MISSION="${SCRIPT_DIR}/generated/62_Collins_consolidated_perimeter_1mps_20260929.txt"
AUDIT="${SCRIPT_DIR}/generated/62_Collins_consolidated_perimeter_1mps_20260929_audit.csv"
VERIFY="${SCRIPT_DIR}/verify_consolidated_perimeter_20260929.py"
PREFLIGHT="${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_20261002.py"
HEADING_CONFIG="${TRACTOR_REPO}/tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260915.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260828.py"
WIFI_SERVER="${TRACTOR_REPO}/tractor_rpi/testing/webrtc/wifi_primary_control_20261003.py"
EXPECTED_FIRMWARE="teensy_main_20261003_wifi_v3"
APPROVED_FOR_FIELD=true

verify_only=false
dashboard_mode=false
configure_heading=false
case "${1:-}" in
    --verify-only) verify_only=true; shift ;;
    --dashboard) dashboard_mode=true; shift ;;
    --configure-heading) configure_heading=true; shift ;;
esac
[[ $# -eq 0 ]] || { echo "Usage: $0 [--verify-only|--dashboard|--configure-heading]" >&2; exit 2; }

python3 "${VERIFY}"
if [[ "${verify_only}" == true ]]; then
    echo "Verification only; nothing was started."
    exit 0
fi

if [[ "${APPROVED_FOR_FIELD}" != true ]]; then
    echo "BLOCKED: package approval flag is not enabled." >&2
    echo "Do not bypass this gate without Al's replay/preview approval." >&2
    echo "No GPS configuration, logger, controller, or motion command was started." >&2
    exit 3
fi

if [[ "${configure_heading}" == true ]]; then
    echo "Stop rtcm-server before continuing. Applying the verified RAM-only Heading-F9P startup profile."
    sudo python3 -u "${HEADING_CONFIG}" --heading-startup --device-wait-seconds 30
    echo "Heading profile verified without an interactive confirmation."
    echo "Restart rtcm-server, then allow correction and heading solutions to settle before launch."
    exit 0
fi

for required in "${MISSION}" "${AUDIT}" "${PREFLIGHT}" "${CONTROLLER}" "${LOGGER}" "${WIFI_SERVER}"; do
    [[ -f "${required}" ]] || { echo "ERROR: required file not found: ${required}" >&2; exit 1; }
done
pgrep -f '[p]ython3.*wifi_primary_control_20261003.py' >/dev/null || { echo "ERROR: Wi-Fi phone control server is not running" >&2; exit 1; }
pgrep -f '[p]ython3.*field_test_logger_20260828.py' >/dev/null && { echo "ERROR: field logger already running" >&2; exit 1; }
pgrep -f '[p]ython3.*pure_pursuit_controller_20260915.py' >/dev/null && { echo "ERROR: Pure Pursuit controller already running" >&2; exit 1; }

echo "============================================================"
echo " 62 COLLINS CONSOLIDATED PERIMETER — BLADES OFF / SUPERVISED"
echo " Speed 1.00 m/s; left deck edge follows the outer perimeter"
echo " Keep the phone control page open and physical e-stop immediately available"
echo "============================================================"
echo "Keep the phone in Pause. Checking Wi-Fi heartbeat, Teensy bridge, RTK corrections, and heading..."
sleep 15
if [[ "${dashboard_mode}" == true ]]; then
    python3 "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}"
else
    sudo python3 "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}"
fi

python3 - <<'PY'
import json, socket, time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", 6003))
sock.settimeout(1.0)
latest = None
deadline = time.monotonic() + 5.0
while time.monotonic() < deadline:
    try:
        latest = json.loads(sock.recvfrom(65535)[0])
    except socket.timeout:
        continue
sock.close()
if latest is None:
    raise SystemExit("ERROR: no Teensy status on UDP 6003")
wifi = latest.get("wifi_control", {})
if wifi.get("mode") != 0:
    raise SystemExit(f"ERROR: phone must be in Pause; Wi-Fi mode={wifi.get('mode')!r}")
if wifi.get("heartbeat_fresh") != 1:
    raise SystemExit("ERROR: phone heartbeat is not fresh; keep the control page open")
if wifi.get("estop_latched") != 0:
    raise SystemExit("ERROR: Wi-Fi E-stop relay is latched")
try:
    command_age_ms = int(wifi.get("command_age_ms"))
except (TypeError, ValueError):
    raise SystemExit("ERROR: Wi-Fi command age is missing")
if command_age_ms > 500:
    raise SystemExit(f"ERROR: Wi-Fi command is stale ({command_age_ms} ms)")
print(f"PASS: phone control is live in Pause; command age={command_age_ms} ms; E-stop released")
PY

python3 - "${MISSION}" <<'PY'
import json, math, socket, sys, time
first=open(sys.argv[1],encoding="utf-8").readline().split()
start_lat,start_lon,start_yaw=map(float,first[:3])
target_heading=(90.0-math.degrees(start_yaw))%360.0
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);sock.bind(("",6010));sock.settimeout(5.0)
latest=None;deadline=time.monotonic()+5.0
while time.monotonic()<deadline:
    try: latest=json.loads(sock.recvfrom(65535)[0])
    except socket.timeout: break
sock.close()
if latest is None: raise SystemExit("ERROR: no navigation GPS packet on UDP 6010")
lat=float(latest["lat"]);lon=float(latest["lon"]);heading=float(latest["heading_deg"])
east=(lon-start_lon)*111320.0*math.cos(math.radians(start_lat));north=(lat-start_lat)*110540.0
position_error=math.hypot(east,north);heading_error=abs((heading-target_heading+180)%360-180)
print(f"PASS: start alignment distance={position_error:.2f} m; heading error={heading_error:.1f} deg")
if latest.get("fix_quality")!="RTK Fixed": raise SystemExit("ERROR: RTK Fixed required")
if not latest.get("headValid") or str(latest.get("carrier","")).lower()!="fixed": raise SystemExit("ERROR: valid fixed-carrier heading required")
baseline=latest.get("relpos_length_m");accuracy=latest.get("relpos_heading_accuracy_deg")
if baseline is None or not .80<=float(baseline)<=1.30: raise SystemExit(f"ERROR: heading baseline {baseline!r} outside 0.80-1.30 m")
if accuracy is None or float(accuracy)>1.0: raise SystemExit(f"ERROR: heading accuracy {accuracy!r} exceeds 1.0 degree")
if position_error>1.5: raise SystemExit("ERROR: use dashboard voice guidance to move within 1.50 m of start")
if heading_error>20: raise SystemExit("ERROR: use dashboard voice guidance to align within 20 degrees")
PY

CONFIRMATION="RUN CONSOLIDATED PERIMETER BLADES OFF"
if [[ "${dashboard_mode}" != true ]]; then
    read -r -p "Type ${CONFIRMATION} to start: " response
    [[ "${response}" == "${CONFIRMATION}" ]] || { echo "Aborted; nothing started."; exit 1; }
fi

log_dir="/home/al/field_logs/20260929_consolidated_perimeter_1mps"
mkdir -p "${log_dir}"
field_log="${log_dir}/consolidated_perimeter_$(date '+%Y%m%d_%H%M%S').csv"
logger_pid=""
cleanup() {
    if [[ -n "${logger_pid}" ]] && kill -0 "${logger_pid}" 2>/dev/null; then kill "${logger_pid}"; wait "${logger_pid}" 2>/dev/null || true; fi
    echo "Controller shutdown preserves neutral safety behavior. Field log: ${field_log}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
python3 -u "${LOGGER}" --output "${field_log}" & logger_pid=$!
sleep 2
kill -0 "${logger_pid}" 2>/dev/null || { echo "ERROR: logger stopped during startup" >&2; exit 1; }
controller_args=(python3 -u "${CONTROLLER}" "${MISSION}" --mode live --gps-port 6010 --status-port 6003 --min-fix "RTK Fixed" --ip 127.0.0.1 --port 6004 --max-speed 1.00 --tracking-window 6.0 --audit-file "${AUDIT}" --reacquire-max-advance 5.0 --resume-stable-seconds 0.0 --basic-runtime-heading-gate --no-operator-cycle-after-safety-loss)
if [[ "${dashboard_mode}" == true ]]; then controller_args+=(--control-port 6011 --telemetry-port 6012); fi
"${controller_args[@]}"
