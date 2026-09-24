#!/usr/bin/env bash
# Supervised, blades-off launcher for the post-main-backyard continuation.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
GENERATED="${SCRIPT_DIR}/generated_continuation_20260918"
MISSION="${GENERATED}/62_Collins_continuation_after_main_backyard_1mps_20260918.txt"
AUDIT="${GENERATED}/62_Collins_continuation_after_main_backyard_1mps_audit_20260918.csv"
REPORT="${GENERATED}/62_Collins_continuation_after_main_backyard_1mps_report_20260918.json"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260915.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260828.py"
PREFLIGHT="${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_20260804.py"
EXPECTED_NORMALIZED_SHA256="a5253e3424d1456db13ea21c734eaa77bc4e6f417d1296a56cdbfaf82ef66dae"

verify_only=false
dashboard_mode=false
case "${1:-}" in
    --verify-only) verify_only=true; shift ;;
    --dashboard) dashboard_mode=true; shift ;;
esac
[[ $# -eq 0 ]] || { echo "Usage: $0 [--verify-only|--dashboard]" >&2; exit 2; }

for required in "${MISSION}" "${AUDIT}" "${REPORT}" "${CONTROLLER}" "${LOGGER}" "${PREFLIGHT}"; do
    [[ -f "${required}" ]] || { echo "ERROR: required file not found: ${required}" >&2; exit 1; }
done

python3 - "${MISSION}" "${AUDIT}" "${REPORT}" "${EXPECTED_NORMALIZED_SHA256}" <<'PY'
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

mission_path, audit_path, report_path = map(Path, sys.argv[1:4])
expected_sha256 = sys.argv[4]
mission_bytes = mission_path.read_bytes().replace(b"\r\n", b"\n")
actual_sha256 = hashlib.sha256(mission_bytes).hexdigest()
if actual_sha256 != expected_sha256:
    raise SystemExit(
        f"ERROR: continuation checksum is {actual_sha256}; expected {expected_sha256}"
    )

rows = [line.split() for line in mission_bytes.decode("utf-8").splitlines()]
if len(rows) != 6468 or any(len(row) != 5 for row in rows):
    raise SystemExit(f"ERROR: expected 6468 five-column rows, got {len(rows)}")
if any(not all(math.isfinite(float(value)) for value in row) for row in rows):
    raise SystemExit("ERROR: continuation contains a non-finite value")
if {row[3] for row in rows} != {"2.00"}:
    raise SystemExit("ERROR: every continuation lookahead must be 2.00 m")
if {row[4] for row in rows} != {"1.00"}:
    raise SystemExit("ERROR: every continuation speed must be 1.00 m/s")

with audit_path.open(newline="", encoding="utf-8-sig") as handle:
    audit = list(csv.DictReader(handle))
if len(audit) != len(rows):
    raise SystemExit("ERROR: continuation audit is not aligned with the mission")
if audit[0].get("phase") != "transition_05":
    raise SystemExit("ERROR: continuation no longer begins with transition_05")
if audit[0].get("source_waypoint") != "12873":
    raise SystemExit("ERROR: continuation source waypoint is not 12873")
if any(row.get("phase", "").startswith("main_backyard_") for row in audit):
    raise SystemExit("ERROR: a completed main-backyard phase remains")
if {row.get("speed_mps") for row in audit} != {"1.00"}:
    raise SystemExit("ERROR: audit speed is not uniformly 1.00 m/s")

report = json.loads(report_path.read_text(encoding="utf-8"))
if report.get("mission_sha256") != expected_sha256 or report.get("waypoints") != 6468:
    raise SystemExit("ERROR: continuation report does not match the mission")
if report.get("first_phase") != "transition_05":
    raise SystemExit("ERROR: continuation report first phase changed")
if report.get("distance_from_wp0091_start_m", 999.0) > 0.25:
    raise SystemExit("ERROR: continuation start moved away from the reviewed start")
if report.get("heading_difference_from_wp0091_start_deg", 999.0) > 20.0:
    raise SystemExit("ERROR: continuation start heading changed too far")
if report.get("speed_counts") != {"1.00": 6468}:
    raise SystemExit("ERROR: continuation report speed distribution changed")
if report.get("run_evidence", {}).get("maximum_controller_waypoint_idx", 0) < 12668:
    raise SystemExit("ERROR: run evidence no longer shows main-backyard progress")

print("PASS: exact post-main-backyard continuation verified.")
print("      Begins with recorded transition_05, 0.19 m from the waypoint-91 start.")
print("      6,468 waypoints, 2.00 m lookahead, every speed command 1.00 m/s.")
print("      All main-backyard phases are omitted; remaining reviewed geometry is unchanged.")
PY

if [[ "${verify_only}" == true ]]; then
    echo "Verification-only requested; no services or controller were started."
    exit 0
fi

pgrep -f '[p]ython3.*field_test_logger_20260828.py' >/dev/null && {
    echo "ERROR: a field logger is already running." >&2
    exit 1
}
pgrep -f '[p]ython3.*pure_pursuit_controller_20260915.py' >/dev/null && {
    echo "ERROR: the Pure Pursuit controller is already running." >&2
    exit 1
}

echo "============================================================"
echo " 62 COLLINS CONTINUATION AFTER MAIN BACKYARD - ALL 1.0 M/S"
echo " Start       : reviewed waypoint-91 location"
echo " First phase : recorded transition_05 toward garden right"
echo " Included    : garden right, garden left, front yard, over-road boundary"
echo " Excluded    : every main-backyard phase; stripes remain disabled"
echo " Speed       : 1.00 m/s at every waypoint"
echo " Moving time : approximately 17.9 minutes"
echo " Mower deck must remain disengaged"
echo " Keep the handheld available for Pause/Manual at all times"
echo "============================================================"
echo "Running preflight; keep the tractor in Pause."
if [[ "${dashboard_mode}" == true ]]; then
    python3 "${PREFLIGHT}" --expected-firmware teensy_main_20260914
else
    sudo python3 "${PREFLIGHT}" --expected-firmware teensy_main_20260914
fi

echo "Checking position and heading against the continuation start on UDP 6010..."
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
deadline = time.time() + 5.0
while time.time() < deadline:
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
    raise SystemExit("ERROR: continuation start requires RTK Fixed")
if not latest.get("headValid") or latest.get("carrier") != "fixed":
    raise SystemExit("ERROR: continuation start requires valid fixed-carrier heading")
baseline = latest.get("relpos_length_m")
accuracy = latest.get("relpos_heading_accuracy_deg")
if baseline is None or not 0.80 <= float(baseline) <= 1.30:
    raise SystemExit(f"ERROR: heading baseline {baseline!r} m is outside 0.80-1.30 m")
if accuracy is None or float(accuracy) > 1.0:
    raise SystemExit(f"ERROR: heading accuracy {accuracy!r} deg exceeds 1.0 deg")
if position_error > 1.5:
    raise SystemExit("ERROR: move within 1.50 m of the continuation start")
if heading_error > 20.0:
    raise SystemExit("ERROR: align within 20 degrees of the continuation start heading")
print("PASS: position and heading are suitable for continuation start.")
PY

if [[ "${dashboard_mode}" == true ]]; then
    echo "Dashboard supplied the blades-off continuation confirmation."
else
    read -r -p 'Type RUN CONTINUATION ALL 1 MPS BLADES OFF to start: ' confirmation
    [[ "${confirmation}" == "RUN CONTINUATION ALL 1 MPS BLADES OFF" ]] || {
        echo "Aborted; nothing was started."
        exit 1
    }
fi

log_dir="/home/al/field_logs/20260918_continuation_all_1mps"
mkdir -p "${log_dir}"
field_log="${log_dir}/continuation_all_1mps_$(date '+%Y%m%d_%H%M%S').csv"
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
    echo "ERROR: field logger stopped during startup." >&2
    exit 1
}

echo "Logger running. Starting Pure Pursuit; remain in Pause until status is reviewed."
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
    --reacquire-max-advance 30.0
    --resume-stable-seconds 0.0
    --basic-runtime-heading-gate
    --no-operator-cycle-after-safety-loss
)
if [[ "${dashboard_mode}" == true ]]; then
    controller_args+=(--control-port 6011 --telemetry-port 6012)
fi
"${controller_args[@]}"
