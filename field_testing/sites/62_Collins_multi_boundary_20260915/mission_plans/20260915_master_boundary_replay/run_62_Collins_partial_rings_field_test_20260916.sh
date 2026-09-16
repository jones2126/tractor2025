#!/usr/bin/env bash
# Supervised, blades-off launcher for the reviewed partial rings-only mission.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
MISSION="${SCRIPT_DIR}/generated_rings_only/62_Collins_rings_only_resume_wp0091_20260916.txt"
AUDIT="${SCRIPT_DIR}/generated_rings_only/62_Collins_rings_only_resume_wp0091_audit_20260916.csv"
REPORT="${SCRIPT_DIR}/generated_rings_only/62_Collins_rings_only_resume_wp0091_report_20260916.json"
SOURCE_REPORT="${SCRIPT_DIR}/generated_rings_only/62_Collins_rings_only_master_1mps_report_20260915.json"
CONTROLLER="${TRACTOR_REPO}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260915.py"
LOGGER="${TRACTOR_REPO}/tractor_rpi/field_test_logger_20260828.py"
PREFLIGHT="${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_20260804.py"
EXPECTED_NORMALIZED_SHA256="8a324391b762a82eb28ebe542f8f5bcd9da4c4a1897d921c41a2afcea1338dee"

verify_only=false
dashboard_mode=false
case "${1:-}" in
    --verify-only) verify_only=true; shift ;;
    --dashboard) dashboard_mode=true; shift ;;
esac
[[ $# -eq 0 ]] || { echo "Usage: $0 [--verify-only|--dashboard]" >&2; exit 2; }

for required in "${MISSION}" "${AUDIT}" "${REPORT}" "${SOURCE_REPORT}" "${CONTROLLER}" "${LOGGER}" "${PREFLIGHT}"; do
    [[ -f "${required}" ]] || { echo "ERROR: required file not found: ${required}" >&2; exit 1; }
done

python3 - "${MISSION}" "${AUDIT}" "${REPORT}" "${SOURCE_REPORT}" "${EXPECTED_NORMALIZED_SHA256}" <<'PY'
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

mission_path = Path(sys.argv[1])
audit_path = Path(sys.argv[2])
report_path = Path(sys.argv[3])
source_report_path = Path(sys.argv[4])
expected_sha256 = sys.argv[5]
mission_bytes = mission_path.read_bytes().replace(b"\r\n", b"\n")
actual_sha256 = hashlib.sha256(mission_bytes).hexdigest()
if actual_sha256 != expected_sha256:
    raise SystemExit(
        f"ERROR: mission checksum is {actual_sha256}; expected {expected_sha256}"
    )

rows = [line.split() for line in mission_bytes.decode("utf-8").splitlines()]
if len(rows) != 19250 or any(len(row) != 5 for row in rows):
    raise SystemExit(f"ERROR: expected 19250 five-column rows, got {len(rows)}")
if any(not all(math.isfinite(float(value)) for value in row) for row in rows):
    raise SystemExit("ERROR: mission contains a non-finite value")
if {row[3] for row in rows} != {"2.00"}:
    raise SystemExit("ERROR: every mission lookahead must be 2.00 m")
speed_counts = {speed: sum(row[4] == speed for row in rows) for speed in {row[4] for row in rows}}
if speed_counts != {"0.50": 2945, "1.00": 16305}:
    raise SystemExit(f"ERROR: unexpected mission speed distribution: {speed_counts}")

report = json.loads(report_path.read_text(encoding="utf-8"))
if report.get("source_start_waypoint") != 91 or report.get("trimmed_source_waypoints") != 90:
    raise SystemExit("ERROR: resume report no longer starts at reviewed source waypoint 91")
if report.get("waypoints") != 19250 or report.get("mission_sha256") != expected_sha256:
    raise SystemExit("ERROR: resume report does not match the reviewed mission")
guards = report.get("safety_guards", {})
if guards.get("reacquire_max_advance_m") != 30.0 or not guards.get("phase_locked_recovery"):
    raise SystemExit("ERROR: bounded phase-locked recovery is not enabled")
if guards.get("required_heading_carrier") != "fixed":
    raise SystemExit("ERROR: fixed-carrier heading is not required")

with audit_path.open(newline="", encoding="utf-8-sig") as handle:
    audit = list(csv.DictReader(handle))
if len(audit) != len(rows) or audit[0].get("source_waypoint") != "91":
    raise SystemExit("ERROR: resume audit is not aligned with the mission")
if any(not row.get("phase") for row in audit):
    raise SystemExit("ERROR: resume audit contains an empty phase")

source_report = json.loads(source_report_path.read_text(encoding="utf-8"))
if source_report.get("coverage_blocked_fields") != ["over_the_road"]:
    raise SystemExit("ERROR: report no longer has the reviewed over-road-only limitation")
over_road = source_report.get("fields", {}).get("over_the_road", {})
if not over_road.get("manual_boundary_pass_included"):
    raise SystemExit("ERROR: over-road boundary pass is not marked as included")
if over_road.get("geometrically_available_inner_ring_count") != 1:
    raise SystemExit("ERROR: expected exactly one omitted over-road inner ring")
if over_road.get("ring_count") != 0:
    raise SystemExit("ERROR: reviewed field test must omit the over-road inner ring")
if source_report.get("stripes_enabled") is not False:
    raise SystemExit("ERROR: stripes must remain disabled")
if source_report.get("over_road_route_enters_expanded_pole_exclusion_interior"):
    raise SystemExit("ERROR: over-road route enters the pole exclusion")
if source_report.get("minimum_over_road_route_distance_to_recorded_pole_loop_m", 0.0) < 0.6096:
    raise SystemExit("ERROR: over-road route does not preserve 24-inch pole clearance")
if source_report.get("maximum_waypoint_gap_m", 999.0) > 0.500001:
    raise SystemExit("ERROR: waypoint spacing exceeds 0.50 m")

print("PASS: exact reviewed clear-sky resume mission verified.")
print("      Starts at source waypoint 91; 19,250 rows remain.")
print("      1.00 m/s cruise, 0.50 m/s tight turns, 2.00 m lookahead.")
print("      Recovery is phase-locked and limited to 30 m forward progress.")
print("      Near-360-degree planned connectors are prohibited; stripes disabled.")
print("      Over-road boundary included; its one inner ring remains omitted.")
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
    echo "ERROR: the 2026-09-15 Pure Pursuit controller is already running." >&2
    exit 1
}

echo "============================================================"
echo " 62 COLLINS CLEAR-SKY RESUME - SOURCE WAYPOINT 91"
echo " Inner rings : main backyard, both gardens, and front yard"
echo " Over-road   : reviewed boundary only; inner ring omitted"
echo " Pole        : 24-inch additional exclusion preserved"
echo " Stripes     : disabled"
echo " Moving time : approximately 59.2 minutes"
echo " Mower deck must remain disengaged"
echo " Keep the handheld available for Pause/Manual at all times"
echo " Recovery    : same phase only; maximum 30 m forward"
echo "============================================================"
echo "Running preflight; keep the tractor in Pause."
if [[ "${dashboard_mode}" == true ]]; then
    # The web dashboard cannot answer a sudo password prompt. Preflight uses
    # read-only device/service checks and fails closed on missing access.
    python3 "${PREFLIGHT}" --expected-firmware teensy_main_20260914
else
    sudo python3 "${PREFLIGHT}" --expected-firmware teensy_main_20260914
fi

echo "Checking position and heading against the mission start on UDP 6010..."
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

lat = float(latest["lat"])
lon = float(latest["lon"])
heading = float(latest["heading_deg"])
east = (lon - start_lon) * 111320.0 * math.cos(math.radians(start_lat))
north = (lat - start_lat) * 110540.0
position_error = math.hypot(east, north)
heading_error = abs((heading - start_heading + 180.0) % 360.0 - 180.0)
print(f"Start target: lat={start_lat:.9f}, lon={start_lon:.9f}, heading={start_heading:.1f} deg")
print(f"Start error: distance={position_error:.2f} m, heading={heading_error:.1f} deg")
if latest.get("fix_quality") != "RTK Fixed":
    raise SystemExit("ERROR: mission start requires RTK Fixed")
if not latest.get("headValid") or latest.get("carrier") != "fixed":
    raise SystemExit("ERROR: mission start requires valid fixed-carrier heading")
baseline = latest.get("relpos_length_m")
accuracy = latest.get("relpos_heading_accuracy_deg")
if baseline is None or not 0.80 <= float(baseline) <= 1.30:
    raise SystemExit(f"ERROR: heading baseline {baseline!r} m is outside 0.80-1.30 m")
if accuracy is None or float(accuracy) > 1.0:
    raise SystemExit(f"ERROR: heading accuracy {accuracy!r} deg exceeds 1.0 deg")
if position_error > 1.5:
    raise SystemExit("ERROR: move within 1.50 m of the mission start")
if heading_error > 20.0:
    raise SystemExit("ERROR: align within 20 degrees of the mission start heading")
print("PASS: position and heading are suitable for mission start.")
PY

echo ""
echo "This is the first field run of the guarded recovery controller and this master path."
echo "Start in Pause. Select AUTO only after the controller reports that it is waiting."
echo "If RTK Fixed is lost, select Manual, drive safely, then return to AUTO only"
echo "after LED4 is green and the tractor is near and aligned with the intended path."
if [[ "${dashboard_mode}" == true ]]; then
    echo "Dashboard supplied the blades-off start confirmation."
else
    read -r -p 'Type RUN PARTIAL RINGS BLADES OFF to start: ' confirmation
    [[ "${confirmation}" == "RUN PARTIAL RINGS BLADES OFF" ]] || {
        echo "Aborted; nothing was started."
        exit 1
    }
fi

log_dir="/home/al/field_logs/20260916_partial_rings_master"
mkdir -p "${log_dir}"
field_log="${log_dir}/partial_rings_master_$(date '+%Y%m%d_%H%M%S').csv"
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
    --resume-stable-seconds 5.0
)
if [[ "${dashboard_mode}" == true ]]; then
    controller_args+=(--control-port 6011 --telemetry-port 6012)
fi
"${controller_args[@]}"
