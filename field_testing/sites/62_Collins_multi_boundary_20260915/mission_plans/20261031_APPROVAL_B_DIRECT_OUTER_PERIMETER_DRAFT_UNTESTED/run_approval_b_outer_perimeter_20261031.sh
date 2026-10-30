#!/usr/bin/env bash
# Complete but fail-closed Approval B launcher candidate.
set -euo pipefail

APPROVED_FOR_FIELD=false
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
SOURCE="${HERE}/../20260928_outer_perimeter_field_test"
MISSION="${SOURCE}/generated/62_Collins_outer_perimeter_field_test_1mps_20260928.txt"
AUDIT="${SOURCE}/generated/62_Collins_outer_perimeter_field_test_1mps_20260928_audit.csv"
VERIFY="${HERE}/verify_approval_b_package_20261030.py"
PREFLIGHT="${REPO}/tractor_rpi/testing/mission_preflight_20261002.py"
CONTROLLER="${REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20261022.py"
LOGGER="${REPO}/tractor_rpi/field_test_logger_20260828.py"
EXPECTED_FIRMWARE="teensy_main_20261003_wifi_v3"
CONFIRMATION="RUN APPROVAL B OUTER PERIMETER BLADES OFF"

verify_only=false
dashboard_mode=false
case "${1:-}" in
    --verify-only) verify_only=true; shift ;;
    --dashboard) dashboard_mode=true; shift ;;
esac
[[ $# -eq 0 ]] || { echo "Usage: $0 [--verify-only|--dashboard]" >&2; exit 2; }

python3 "${VERIFY}"
if [[ "${verify_only}" == true ]]; then
    echo "Verification complete. No service, logger, controller, or motion command was started."
    exit 0
fi

[[ "${APPROVED_FOR_FIELD}" == true ]] || {
    echo "DRAFT - UNTESTED IN THE FIELD" >&2
    echo "APPROVED_FOR_FIELD=${APPROVED_FOR_FIELD}" >&2
    echo "Approval B live start is disabled pending a separate test-day authorization." >&2
    exit 3
}

for required in "${MISSION}" "${AUDIT}" "${PREFLIGHT}" "${CONTROLLER}" "${LOGGER}"; do
    [[ -f "${required}" ]] || { echo "ERROR: missing ${required}" >&2; exit 1; }
done
systemctl is-active --quiet tractor-wifi-direct-mission.service || {
    echo "ERROR: direct mission phone-control service is not active" >&2; exit 1;
}
systemctl is-active --quiet tractor-wifi-control.service && {
    echo "ERROR: ZeroTier-primary phone-control service is also active" >&2; exit 1;
}
systemctl is-active --quiet tractor-wifi-recovery.service && {
    echo "ERROR: Manual-recovery phone-control service is also active" >&2; exit 1;
}
pgrep -f '[p]ython3.*field_test_logger_20260828.py' >/dev/null && {
    echo "ERROR: a field logger is already running" >&2; exit 1;
}
pgrep -f '[p]ython3.*pure_pursuit_controller_20261022.py' >/dev/null && {
    echo "ERROR: a Pure Pursuit controller is already running" >&2; exit 1;
}

echo "============================================================"
echo " APPROVAL B - OUTER PERIMETER - BLADES OFF"
echo " 3,462 waypoints; 689.949 m; 1.00 m/s; 2.00 m lookahead"
echo " Phone authority: onboard-router-mission via 192.168.8.20"
echo " Keep phone in Pause until READY FOR GUARDED AUTO"
echo "============================================================"

if [[ "${dashboard_mode}" == true ]]; then
    python3 "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}"
else
    sudo python3 "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}"
fi

python3 - <<'PY'
import json, socket, time
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
sock.bind(("",6003));sock.settimeout(1.0)
latest=None;deadline=time.monotonic()+5.0
while time.monotonic()<deadline:
    try: latest=json.loads(sock.recvfrom(65535)[0])
    except socket.timeout: continue
sock.close()
if latest is None: raise SystemExit("ERROR: no fresh Teensy bridge status")
wifi=latest.get("wifi_control",{})
if wifi.get("mode")!=0: raise SystemExit("ERROR: phone must be in Pause")
if wifi.get("heartbeat_fresh")!=1: raise SystemExit("ERROR: phone heartbeat is not fresh")
if wifi.get("estop_latched")!=0: raise SystemExit("ERROR: Wi-Fi e-stop is latched")
age=int(wifi.get("command_age_ms",999999))
if age>500: raise SystemExit(f"ERROR: phone command is stale ({age} ms)")
print(f"PASS: phone control is fresh in Pause; command age={age} ms")
PY

if [[ "${dashboard_mode}" == true ]]; then
    echo "Dashboard supplied the exact blades-off confirmation."
else
    read -r -p "Type ${CONFIRMATION} to start: " response
    [[ "${response}" == "${CONFIRMATION}" ]] || { echo "Aborted; nothing started."; exit 1; }
fi

log_dir="/home/al/field_logs/20261031_approval_b_outer_perimeter"
mkdir -p "${log_dir}"
field_log="${log_dir}/approval_b_outer_perimeter_$(date '+%Y%m%d_%H%M%S').csv"
logger_pid=""
cleanup() {
    if [[ -n "${logger_pid}" ]] && kill -0 "${logger_pid}" 2>/dev/null; then
        kill "${logger_pid}"; wait "${logger_pid}" 2>/dev/null || true
    fi
    echo "Controller stopped. Field log: ${field_log}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
python3 -u "${LOGGER}" --output "${field_log}" & logger_pid=$!
sleep 2
kill -0 "${logger_pid}" 2>/dev/null || { echo "ERROR: field logger stopped" >&2; exit 1; }

echo "Logger running. Keep phone in Pause until dashboard says READY FOR GUARDED AUTO."
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
    --resume-stable-seconds 5.0
)
if [[ "${dashboard_mode}" == true ]]; then
    controller_args+=(--control-port 6011 --telemetry-port 6012)
fi
"${controller_args[@]}"
