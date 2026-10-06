#!/usr/bin/env bash
# Supervised, blades-off rerun of the September 23 mission with phone control.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
MASTER_PACKAGE="${SCRIPT_DIR}/../20260915_master_boundary_replay"
GENERATED="${MASTER_PACKAGE}/generated_master_manual_field_20260920"
MISSION="${GENERATED}/62_Collins_master_manual_resampled_1mps_20260920.txt"
AUDIT="${GENERATED}/62_Collins_master_manual_resampled_1mps_20260920_audit.csv"
VERIFY="${SCRIPT_DIR}/verify_master_manual_phone_20261006.py"
PREFLIGHT="${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_20261002.py"
HEADING_CONFIG="${TRACTOR_REPO}/tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260915.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260828.py"
WIFI_SERVER="${TRACTOR_REPO}/tractor_rpi/testing/webrtc/wifi_primary_control_20261003.py"
EXPECTED_FIRMWARE="teensy_main_20261003_wifi_v3"
EXPECTED_CONFIRMATION="RUN SEPTEMBER 23 MASTER PHONE BLADES OFF"

verify_only=false
dashboard_mode=false
configure_heading=false
case "${1:-}" in
    --verify-only) verify_only=true; shift ;;
    --dashboard) dashboard_mode=true; shift ;;
    --configure-heading) configure_heading=true; shift ;;
esac
[[ $# -eq 0 ]] || {
    echo "Usage: $0 [--verify-only|--dashboard|--configure-heading]" >&2
    exit 2
}

python3 "${VERIFY}"
if [[ "${verify_only}" == true ]]; then
    echo "Verification only; nothing was started."
    exit 0
fi

configure_heading_profile() {
    local config_rc start_rc active_state
    [[ -f "${HEADING_CONFIG}" ]] || {
        echo "ERROR: Heading-F9P configuration tool not found: ${HEADING_CONFIG}" >&2
        return 1
    }
    echo "Applying the verified RAM-only Heading-F9P 5 Hz startup profile..."
    sudo systemctl stop rtcm-server.service || {
        echo "FAIL: could not stop rtcm-server — DO NOT SELECT AUTO" >&2
        return 1
    }
    set +e
    sudo python3 -u "${HEADING_CONFIG}" --heading-startup --device-wait-seconds 30
    config_rc=$?
    sudo systemctl start rtcm-server.service
    start_rc=$?
    set -e
    active_state="$(systemctl is-active rtcm-server.service 2>/dev/null || true)"
    echo "rtcm-server state: ${active_state}"
    if [[ "${config_rc}" -ne 0 || "${start_rc}" -ne 0 || "${active_state}" != active ]]; then
        echo "FAIL: Heading configuration or rtcm-server restart failed — DO NOT SELECT AUTO" >&2
        return 1
    fi
    echo "PASS: Heading profile verified and rtcm-server is active"
}

if [[ "${configure_heading}" == true ]]; then
    configure_heading_profile
    exit 0
fi

for required in "${MISSION}" "${AUDIT}" "${PREFLIGHT}" "${CONTROLLER}" "${LOGGER}" "${WIFI_SERVER}"; do
    [[ -f "${required}" ]] || {
        echo "ERROR: required file not found: ${required}" >&2
        exit 1
    }
done
pgrep -f '[p]ython3.*wifi_primary_control_20261003.py' >/dev/null || {
    echo "ERROR: Wi-Fi phone control server is not running" >&2
    exit 1
}
pgrep -f '[p]ython3.*field_test_logger_20260828.py' >/dev/null && {
    echo "ERROR: field logger already running" >&2
    exit 1
}
pgrep -f '[p]ython3.*pure_pursuit_controller_20260915.py' >/dev/null && {
    echo "ERROR: Pure Pursuit controller already running" >&2
    exit 1
}

echo "============================================================"
echo " SEPTEMBER 23 MASTER MISSION RERUN — PHONE CONTROL"
echo " Exact route : 19,825 waypoints; SHA-256 0276fa22...8436b4"
echo " Speed       : 1.00 m/s; lookahead 2.00 m"
echo " Safety      : blades off, direct supervision, physical e-stop ready"
echo " Phone       : local tractor Wi-Fi only; keep the page open in Pause"
echo "============================================================"
if [[ "${TRACTOR_HEADING_PROFILE_READY:-0}" == 1 ]]; then
    [[ "$(systemctl is-active rtcm-server.service 2>/dev/null || true)" == active ]] || {
        echo "ERROR: dashboard configured heading, but rtcm-server is no longer active" >&2
        exit 1
    }
    echo "PASS: dashboard startup already configured Heading F9P; rtcm-server is active"
else
    configure_heading_profile
fi
echo "Checking phone heartbeat, Teensy bridge, RTK corrections, and heading..."
sleep 15
if [[ "${dashboard_mode}" == true ]]; then
    python3 "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}"
else
    sudo python3 "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}"
fi

python3 - <<'PY'
import json
import socket
import time

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
    raise SystemExit("ERROR: phone heartbeat is not fresh; keep the local control page open")
if wifi.get("estop_latched") != 0:
    raise SystemExit("ERROR: Wi-Fi E-stop relay is latched")
try:
    command_age_ms = int(wifi.get("command_age_ms"))
except (TypeError, ValueError):
    raise SystemExit("ERROR: Wi-Fi command age is missing")
if command_age_ms > 500:
    raise SystemExit(f"ERROR: Wi-Fi command is stale ({command_age_ms} ms)")
print(f"PASS: phone control live in Pause; command age={command_age_ms} ms; E-stop released")
PY

python3 - "${MISSION}" <<'PY'
import json
import math
import socket
import sys
import time

first = open(sys.argv[1], encoding="utf-8").readline().split()
start_lat, start_lon, start_yaw = map(float, first[:3])
target_heading = (90.0 - math.degrees(start_yaw)) % 360.0
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", 6010))
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
    raise SystemExit("ERROR: no navigation GPS packet on UDP 6010")
lat = float(latest["lat"])
lon = float(latest["lon"])
heading = float(latest["heading_deg"])
east = (lon - start_lon) * 111320.0 * math.cos(math.radians(start_lat))
north = (lat - start_lat) * 110540.0
position_error = math.hypot(east, north)
heading_error = abs((heading - target_heading + 180.0) % 360.0 - 180.0)
print(f"Start alignment: distance={position_error:.2f} m; heading error={heading_error:.1f} deg")
if latest.get("fix_quality") != "RTK Fixed":
    raise SystemExit("ERROR: RTK Fixed required")
if not latest.get("headValid") or str(latest.get("carrier", "")).lower() != "fixed":
    raise SystemExit("ERROR: valid fixed-carrier heading required")
baseline = latest.get("relpos_length_m")
accuracy = latest.get("relpos_heading_accuracy_deg")
if baseline is None or not 0.80 <= float(baseline) <= 1.30:
    raise SystemExit(f"ERROR: heading baseline {baseline!r} outside 0.80-1.30 m")
if accuracy is None or float(accuracy) > 1.0:
    raise SystemExit(f"ERROR: heading accuracy {accuracy!r} exceeds 1.0 degree")
if position_error > 1.5:
    raise SystemExit("ERROR: use phone Manual to move within 1.50 m of mission start")
if heading_error > 20.0:
    raise SystemExit("ERROR: use phone Manual to align within 20 degrees of start heading")
print("PASS: position and heading are suitable for mission start.")
PY

if [[ "${dashboard_mode}" != true ]]; then
    read -r -p "Type ${EXPECTED_CONFIRMATION} to start: " response
    [[ "${response}" == "${EXPECTED_CONFIRMATION}" ]] || {
        echo "Aborted; nothing started."
        exit 1
    }
fi

log_dir="/home/al/field_logs/20261006_master_manual_phone_rerun_1mps"
mkdir -p "${log_dir}"
field_log="${log_dir}/master_manual_phone_$(date '+%Y%m%d_%H%M%S').csv"
logger_pid=""
cleanup() {
    if [[ -n "${logger_pid}" ]] && kill -0 "${logger_pid}" 2>/dev/null; then
        kill "${logger_pid}"
        wait "${logger_pid}" 2>/dev/null || true
    fi
    echo "Controller shutdown preserves neutral safety behavior. Field log: ${field_log}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
python3 -u "${LOGGER}" --output "${field_log}" &
logger_pid=$!
sleep 2
kill -0 "${logger_pid}" 2>/dev/null || {
    echo "ERROR: logger stopped during startup" >&2
    exit 1
}

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
