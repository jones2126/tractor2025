#!/usr/bin/env bash
# Snapshot one field-day run without stopping services or deleting source logs.
set -euo pipefail

TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
FIELD_DATA_DIR="/home/al/repos/field-testing-data"
SINCE="$(date '+%Y-%m-%d 00:00:00')"
RUN=""
OUTPUT=""

usage() {
    echo "Usage: $0 --run master|perimeter [--since 'YYYY-MM-DD HH:MM:SS'] [--output /home/al/name.tgz]"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run) RUN="${2:-}"; shift 2 ;;
        --since) SINCE="${2:-}"; shift 2 ;;
        --output) OUTPUT="${2:-}"; shift 2 ;;
        --help|-h) usage; exit 0 ;;
        *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

case "${RUN}" in
    master)
        LABEL="master_manual_endurance"
        MISSION_LOG_DIR="/home/al/field_logs/20261006_master_manual_phone_rerun_1mps"
        ;;
    perimeter)
        LABEL="consolidated_perimeter"
        MISSION_LOG_DIR="/home/al/field_logs/20260929_consolidated_perimeter_1mps"
        ;;
    *) echo "ERROR: --run must be master or perimeter" >&2; usage >&2; exit 2 ;;
esac

STAMP="$(date '+%Y%m%d_%H%M%S')"
ARCHIVE="${OUTPUT:-/home/al/tractor01_${LABEL}_${STAMP}.tgz}"
LATEST_LINK="/home/al/tractor01_${LABEL}_latest.tgz"
LATEST_INFO="/home/al/tractor01_${LABEL}_latest.txt"
[[ "${ARCHIVE}" == /home/al/*.tgz ]] || { echo "ERROR: archive must be /home/al/*.tgz" >&2; exit 2; }
[[ ! -e "${ARCHIVE}" ]] || { echo "ERROR: refusing to overwrite ${ARCHIVE}" >&2; exit 1; }

STAGING="$(mktemp -d "/tmp/tractor01-${LABEL}-XXXXXX")"
cleanup() { rm -rf -- "${STAGING}"; }
trap cleanup EXIT
mkdir -p "${STAGING}/mission_logs" "${STAGING}/wifi_logs" \
    "${STAGING}/recovery_logs" "${STAGING}/pursuit_logs" "${STAGING}/diagnostics"

copy_find() {
    local destination="$1"
    shift
    local source
    while IFS= read -r -d '' source; do cp -p -- "${source}" "${destination}/"; done < <(find "$@" -print0)
}

if [[ -d "${MISSION_LOG_DIR}" ]]; then
    copy_find "${STAGING}/mission_logs" "${MISSION_LOG_DIR}" -maxdepth 1 -type f -newermt "${SINCE}"
else
    echo "WARNING: mission log directory not found: ${MISSION_LOG_DIR}" >&2
fi
if [[ -d /home/al/field_logs ]]; then
    copy_find "${STAGING}/wifi_logs" /home/al/field_logs -maxdepth 1 -type f -name 'wifi_primary_control_*.jsonl' -newermt "${SINCE}"
    copy_find "${STAGING}/wifi_logs" /home/al/field_logs -maxdepth 1 -type f -name 'phone_flight_recorder_*.jsonl' -newermt "${SINCE}"
fi
if [[ -d /home/al/field_logs/manual_recovery ]]; then
    copy_find "${STAGING}/recovery_logs" /home/al/field_logs/manual_recovery -maxdepth 1 -type f -newermt "${SINCE}"
fi
if [[ -d "${FIELD_DATA_DIR}" ]]; then
    copy_find "${STAGING}/pursuit_logs" "${FIELD_DATA_DIR}" -maxdepth 1 -type f -name 'pursuit_log_*.csv' -newermt "${SINCE}"
fi

DIAGNOSTICS="${STAGING}/diagnostics"
{
    echo "Run: ${RUN}"
    echo "Mission log directory: ${MISSION_LOG_DIR}"
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
} > "${DIAGNOSTICS}/network_state.txt" 2>&1
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
