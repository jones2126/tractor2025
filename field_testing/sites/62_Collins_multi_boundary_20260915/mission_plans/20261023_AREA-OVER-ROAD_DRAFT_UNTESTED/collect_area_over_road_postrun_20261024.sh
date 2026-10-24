#!/usr/bin/env bash
# Copies relevant telemetry into a timestamped archive; source logs are retained.
set -euo pipefail

TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
FIELD_DATA_DIR="/home/al/repos/field-testing-data"
MISSION_LOG_DIR="/home/al/field_logs/20261024_area_over_road_field"
PACKAGE_REL="field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED"
PACKAGE="${TRACTOR_REPO}/${PACKAGE_REL}"
SINCE=""
OUTPUT=""

usage() {
    echo "Usage: $0 [--since 'YYYY-MM-DD HH:MM:SS'] [--output /home/al/name.tgz]"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --since) SINCE="${2:-}"; shift 2 ;;
        --output) OUTPUT="${2:-}"; shift 2 ;;
        --help|-h) usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "${SINCE}" && -f "${MISSION_LOG_DIR}/dashboard_session_start.txt" ]]; then
    SINCE="$(cat "${MISSION_LOG_DIR}/dashboard_session_start.txt")"
fi
SINCE="${SINCE:-$(date '+%Y-%m-%d 00:00:00')}"
STAMP="$(date '+%Y%m%d_%H%M%S')"
ARCHIVE="${OUTPUT:-/home/al/tractor01_area_over_road_${STAMP}.tgz}"
LATEST_LINK="/home/al/tractor01_area_over_road_latest.tgz"
LATEST_INFO="/home/al/tractor01_area_over_road_latest.txt"

[[ "${ARCHIVE}" == /home/al/*.tgz ]] || { echo "ERROR: archive must be /home/al/*.tgz" >&2; exit 2; }
[[ ! -e "${ARCHIVE}" ]] || { echo "ERROR: refusing to overwrite ${ARCHIVE}" >&2; exit 1; }

STAGING="$(mktemp -d '/tmp/tractor01-area-over-road-XXXXXX')"
cleanup() {
    [[ "${STAGING}" == /tmp/tractor01-area-over-road-* ]] || return 1
    rm -rf -- "${STAGING}"
}
trap cleanup EXIT
mkdir -p "${STAGING}/mission_logs" "${STAGING}/wifi_logs" \
    "${STAGING}/recovery_logs" "${STAGING}/pursuit_logs" \
    "${STAGING}/diagnostics" "${STAGING}/mission_package"

copy_find() {
    local destination="$1"
    shift
    local source
    while IFS= read -r -d '' source; do cp -p -- "${source}" "${destination}/"; done < <(find "$@" -print0)
}

if [[ -d "${MISSION_LOG_DIR}" ]]; then
    copy_find "${STAGING}/mission_logs" "${MISSION_LOG_DIR}" -maxdepth 1 -type f -newermt "${SINCE}"
fi
if [[ -d /home/al/field_logs ]]; then
    copy_find "${STAGING}/wifi_logs" /home/al/field_logs -maxdepth 1 -type f \
        \( -name 'wifi_primary_control_*.jsonl' -o -name 'phone_flight_recorder_*.jsonl' \) -newermt "${SINCE}"
fi
if [[ -d /home/al/field_logs/manual_recovery ]]; then
    copy_find "${STAGING}/recovery_logs" /home/al/field_logs/manual_recovery -maxdepth 1 -type f -newermt "${SINCE}"
fi
if [[ -d "${FIELD_DATA_DIR}" ]]; then
    copy_find "${STAGING}/pursuit_logs" "${FIELD_DATA_DIR}" -maxdepth 1 -type f -name 'pursuit_log_*.csv' -newermt "${SINCE}"
fi

cp -p -- "${PACKAGE}/62_collins_area_over_road_mission_project.json" "${STAGING}/mission_package/"
cp -p -- "${PACKAGE}/checkpoint_6_controller_validation_rev003.json" "${STAGING}/mission_package/"
cp -p -- "${PACKAGE}/checkpoint_6_controller_route_rev003.geojson" "${STAGING}/mission_package/"
cp -p -- "${PACKAGE}/checkpoint_6_manifest_rev001.json" "${STAGING}/mission_package/"
cp -p -- "${PACKAGE}/checkpoint_7_pure_pursuit_responses_rev001.txt" "${STAGING}/mission_package/"
cp -p -- "${PACKAGE}/generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024.txt" "${STAGING}/mission_package/"
cp -p -- "${PACKAGE}/generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024_audit.csv" "${STAGING}/mission_package/"

DIAGNOSTICS="${STAGING}/diagnostics"
{
    echo "Status: DRAFT - UNTESTED IN THE FIELD"
    echo "Collected UTC: $(date -u --iso-8601=seconds)"
    echo "Collected local: $(date --iso-8601=seconds)"
    echo "Since: ${SINCE}"
    echo "Hostname: $(hostname)"
    echo "Kernel: $(uname -a)"
} > "${DIAGNOSTICS}/collection_info.txt"
{
    systemctl is-active rtcm-server.service teensy-bridge.service led-controller.service \
        tractor-wifi-control.service tractor-wifi-recovery.service || true
    echo
    systemctl status rtcm-server.service teensy-bridge.service led-controller.service \
        tractor-wifi-control.service tractor-wifi-recovery.service --no-pager -l || true
} > "${DIAGNOSTICS}/service_status.txt" 2>&1
if sudo -n true 2>/dev/null; then JOURNAL=(sudo -n journalctl); else JOURNAL=(journalctl); fi
"${JOURNAL[@]}" -u tractor-wifi-control.service -u tractor-wifi-recovery.service \
    -u teensy-bridge.service -u rtcm-server.service -u led-controller.service \
    --since "${SINCE}" --no-pager > "${DIAGNOSTICS}/service_journal.log" 2>&1 || true
{
    git -C "${TRACTOR_REPO}" rev-parse HEAD
    git -C "${TRACTOR_REPO}" status --short
} > "${DIAGNOSTICS}/repository_state.txt" 2>&1 || true
{
    ip -4 address show || true
    echo
    ip -4 route show || true
    echo
    zerotier-cli listnetworks 2>/dev/null || true
    echo
    iw dev wlan0 link 2>/dev/null || true
    echo
    nmcli -t -f GENERAL,IP4,WIFI-PROPERTIES device show 2>/dev/null || true
    echo
    ss -lunp 2>/dev/null || true
} > "${DIAGNOSTICS}/network_state.txt" 2>&1
pgrep -af 'mission_dashboard_|pure_pursuit_controller_|field_test_logger_|wifi_primary_control_' \
    > "${DIAGNOSTICS}/relevant_processes.txt" 2>&1 || true
{
    echo "MANUAL EVIDENCE STILL REQUIRED"
    echo "Download the system log from the Archer AXE75 at the RTK base station."
    echo "That router provides the TractorField network."
    echo "Record the router clock/time zone and the download time, then preserve the original file unchanged."
} > "${DIAGNOSTICS}/archer_axe75_router_log_reminder.txt"

(
    cd "${STAGING}"
    find . -type f ! -name collected_file_hashes.sha256 -print0 | sort -z | xargs -0 sha256sum
) > "${DIAGNOSTICS}/collected_file_hashes.sha256"

tar -C "${STAGING}" -czf "${ARCHIVE}" .
sync
ARCHIVE_HASH="$(sha256sum "${ARCHIVE}")"
ln -sfn -- "$(basename "${ARCHIVE}")" "${LATEST_LINK}"
{
    echo "${ARCHIVE}"
    echo "${ARCHIVE_HASH}"
} > "${LATEST_INFO}"

echo "ARCHIVE READY"
ls -lh "${ARCHIVE}"
echo "${ARCHIVE_HASH}"
echo "Stable copy path: ${LATEST_LINK}"
echo "Archive details: ${LATEST_INFO}"
echo
echo "REQUIRED MANUAL FOLLOW-UP: download the Archer AXE75 system log for TractorField at the RTK base station."
