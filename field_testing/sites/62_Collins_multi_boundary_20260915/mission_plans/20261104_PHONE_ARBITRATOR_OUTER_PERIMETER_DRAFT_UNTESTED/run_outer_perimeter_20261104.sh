#!/usr/bin/env bash
set -euo pipefail

APPROVED_FOR_FIELD=true
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="${TRACTOR_REPO:-$(cd "${HERE}/../../../../.." && pwd)}"
SOURCE="${HERE}/../20260928_outer_perimeter_field_test"
MISSION="${SOURCE}/generated/62_Collins_outer_perimeter_field_test_1mps_20260928.txt"
AUDIT="${SOURCE}/generated/62_Collins_outer_perimeter_field_test_1mps_20260928_audit.csv"
VERIFY="${HERE}/verify_package_20261104.py"
PREFLIGHT="${REPO}/tractor_rpi/testing/mission_preflight_phone_control_20261109.py"
CONTROLLER="${REPO}/tractor_rpi/pure-pursuit/pure_pursuit_arbitrator_feed_20261031.py"
LOGGER="${REPO}/tractor_rpi/field_test_logger_20260828.py"
COLLECTOR="${REPO}/field_testing/tools/collect_tractor01_run_evidence_20261114.py"
EXPECTED_FIRMWARE="teensy_main_20261110_rpi_command_lease_v5"
CONFIRMATION="RUN PHONE OUTER PERIMETER BLADES OFF"

dashboard_mode=false
verify_only=false
case "${1:-}" in
    --dashboard) dashboard_mode=true; shift ;;
    --verify-only) verify_only=true; shift ;;
esac
[[ $# -eq 0 ]] || { echo "Usage: $0 [--dashboard|--verify-only]" >&2; exit 2; }

python3 "${VERIFY}"
if [[ "${verify_only}" == true ]]; then
    echo "Verification complete. Nothing was started."
    exit 0
fi
[[ "${APPROVED_FOR_FIELD}" == true ]] || {
    echo "NOT FIELD AUTHORIZATION: APPROVED_FOR_FIELD=false" >&2
    exit 3
}

systemctl is-active --quiet tractor-rpi-arbitrator.service
systemctl is-active --quiet tractor-phone-arbitrator.service
for conflicting in teensy-bridge.service tractor-wifi-control.service tractor-wifi-direct-mission.service tractor-wifi-recovery.service; do
    ! systemctl is-active --quiet "${conflicting}" || {
        echo "ERROR: conflicting service active: ${conflicting}" >&2; exit 1;
    }
done
! pgrep -f '[p]ure_pursuit.*20261031.py' >/dev/null || {
    echo "ERROR: Pure Pursuit is already running" >&2; exit 1;
}

if [[ "${dashboard_mode}" == true ]]; then
    python3 "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}"
else
    sudo python3 "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}"
    read -r -p "Type ${CONFIRMATION} to start: " response
    [[ "${response}" == "${CONFIRMATION}" ]] || { echo "Aborted."; exit 1; }
fi

run_stamp="$(date '+%Y%m%d_%H%M%S')"
run_id="phone_outer_perimeter_${run_stamp}"
started_at_local="$(date '+%Y-%m-%d %H:%M:%S')"
started_at_epoch="$(date '+%s')"
log_dir="/home/al/field_logs/20261109_phone_outer_perimeter"
mkdir -p "${log_dir}"
field_log="${log_dir}/outer_perimeter_${run_stamp}.csv"
pursuit_log="/home/al/repos/field-testing-data/pursuit_log_${run_stamp}.csv"
dashboard_log="${log_dir}/production-mission-dashboard_${run_stamp}.txt"
run_manifest="${log_dir}/run_manifest_${run_stamp}.json"
printf 'Run: %s\nStarted: %s\n' "${run_id}" "${started_at_local}" > "${dashboard_log}"
RUN_ID="${run_id}" MISSION_LABEL="phone_outer_perimeter" \
STARTED_AT_LOCAL="${started_at_local}" STARTED_AT_EPOCH="${started_at_epoch}" \
REPOSITORY="${REPO}" FIELD_LOG="${field_log}" PURSUIT_LOG="${pursuit_log}" \
DASHBOARD_LOG="${dashboard_log}" MISSION_FILE="${MISSION}" AUDIT_FILE="${AUDIT}" \
EXPECTED_FIRMWARE_VALUE="${EXPECTED_FIRMWARE}" RUN_MANIFEST="${run_manifest}" \
python3 -c 'import json,os; p=os.environ; data={"schema":1,"run_id":p["RUN_ID"],"mission_label":p["MISSION_LABEL"],"started_at_local":p["STARTED_AT_LOCAL"],"started_at_epoch":int(p["STARTED_AT_EPOCH"]),"repository":p["REPOSITORY"],"field_log":p["FIELD_LOG"],"pursuit_log":p["PURSUIT_LOG"],"dashboard_log":p["DASHBOARD_LOG"],"mission_file":p["MISSION_FILE"],"audit_file":p["AUDIT_FILE"],"expected_firmware":p["EXPECTED_FIRMWARE_VALUE"],"archive_prefix":p["RUN_ID"]}; open(p["RUN_MANIFEST"],"w",encoding="utf-8").write(json.dumps(data,indent=2,sort_keys=True)+"\n")'
echo "RUN_TRANSCRIPT: ${dashboard_log}"
echo "Run manifest: ${run_manifest}"
echo "Field log: ${field_log}"
echo "Pursuit log: ${pursuit_log}"
logger_pid=""
mission_started=true
cleanup() {
    original_status=$?
    trap - EXIT INT TERM
    set +e
    if [[ -n "${logger_pid}" ]] && kill -0 "${logger_pid}" 2>/dev/null; then
        kill "${logger_pid}"; wait "${logger_pid}" 2>/dev/null || true
    fi
    echo
    echo "MISSION PROCESSES STOPPED"
    echo "Field log:   ${field_log}"
    echo "Pursuit log: ${pursuit_log}"
    echo "Run manifest: ${run_manifest}"
    collection_status=0
    if [[ "${mission_started}" == true ]]; then
        python3 "${COLLECTOR}" --manifest "${run_manifest}" || collection_status=$?
    fi
    if [[ "${collection_status}" -ne 0 && "${original_status}" -eq 0 ]]; then
        original_status="${collection_status}"
    fi
    exit "${original_status}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

python3 -u "${LOGGER}" --output "${field_log}" & logger_pid=$!
sleep 2
kill -0 "${logger_pid}" 2>/dev/null

python3 -u "${CONTROLLER}" "${MISSION}" \
    --arbitrator-safety-port 6015 --arbitrator-proposal-only \
    --mode live --gps-port 6010 --status-port 6003 \
    --min-fix "RTK Fixed" --ip 127.0.0.1 --port 16004 \
    --max-speed 1.00 --tracking-window 6.0 --audit-file "${AUDIT}" \
    --reacquire-max-advance 5.0 --resume-stable-seconds 5.0 \
    --control-port 6011 --telemetry-port 6012 \
    --pursuit-log "${pursuit_log}"
