#!/usr/bin/env bash
# Collect the mission, phone-control, pursuit, and service evidence after a run.
set -euo pipefail

TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
MISSION_LOG_DIR="/home/al/field_logs/20261006_master_manual_phone_rerun_1mps"
FIELD_DATA_DIR="/home/al/repos/field-testing-data"
SINCE="$(date '+%Y-%m-%d 00:00:00')"
STAMP="$(date '+%Y%m%d_%H%M%S')"
ARCHIVE="/home/al/tractor01_master_manual_phone_rerun_${STAMP}.tgz"
LATEST_LINK="/home/al/tractor01_master_manual_phone_rerun_latest.tgz"
LATEST_INFO="/home/al/tractor01_master_manual_phone_rerun_latest.txt"

usage() {
    echo "Usage: $0 [--since 'YYYY-MM-DD HH:MM:SS'] [--output /home/al/name.tgz]"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --since)
            [[ $# -ge 2 ]] || { echo "ERROR: --since requires a value" >&2; exit 2; }
            SINCE="$2"
            shift 2
            ;;
        --output)
            [[ $# -ge 2 ]] || { echo "ERROR: --output requires a value" >&2; exit 2; }
            ARCHIVE="$2"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

[[ "${ARCHIVE}" == /home/al/*.tgz ]] || {
    echo "ERROR: archive must be an absolute /home/al/*.tgz path" >&2
    exit 2
}
[[ ! -e "${ARCHIVE}" ]] || {
    echo "ERROR: refusing to overwrite existing archive: ${ARCHIVE}" >&2
    exit 1
}

STAGING="$(mktemp -d /tmp/tractor01-master-manual-phone-XXXXXX)"
cleanup() {
    rm -rf -- "${STAGING}"
}
trap cleanup EXIT
mkdir -p "${STAGING}/mission_logs" "${STAGING}/wifi_logs" \
    "${STAGING}/pursuit_logs" "${STAGING}/diagnostics"

copy_matches() {
    local destination="$1"
    shift
    local source
    while IFS= read -r -d '' source; do
        cp -p -- "${source}" "${destination}/"
    done < <(find "$@" -print0)
}

if [[ -d "${MISSION_LOG_DIR}" ]]; then
    copy_matches "${STAGING}/mission_logs" "${MISSION_LOG_DIR}" -maxdepth 1 -type f
else
    echo "WARNING: mission log directory not found: ${MISSION_LOG_DIR}" >&2
fi

if [[ -d /home/al/field_logs ]]; then
    copy_matches "${STAGING}/wifi_logs" /home/al/field_logs -maxdepth 1 -type f \
        -name 'wifi_primary_control_*.jsonl' -newermt "${SINCE}"
fi

if [[ -d "${FIELD_DATA_DIR}" ]]; then
    copy_matches "${STAGING}/pursuit_logs" "${FIELD_DATA_DIR}" -maxdepth 1 -type f \
        -name 'pursuit_log_*.csv' -newermt "${SINCE}"
else
    echo "WARNING: pursuit-log directory not found: ${FIELD_DATA_DIR}" >&2
fi

DIAGNOSTICS="${STAGING}/diagnostics"
{
    echo "Collected UTC: $(date -u --iso-8601=seconds)"
    echo "Collected local: $(date --iso-8601=seconds)"
    echo "Since: ${SINCE}"
    echo "Hostname: $(hostname)"
    echo "Kernel: $(uname -a)"
} > "${DIAGNOSTICS}/collection_info.txt"

{
    systemctl is-active rtcm-server.service teensy-bridge.service \
        led-controller.service tractor-wifi-control.service || true
    echo
    systemctl status rtcm-server.service teensy-bridge.service \
        led-controller.service tractor-wifi-control.service --no-pager -l || true
} > "${DIAGNOSTICS}/service_status.txt" 2>&1

if sudo -n true 2>/dev/null; then
    sudo -n journalctl -u tractor-wifi-control.service \
        -u teensy-bridge.service -u rtcm-server.service \
        -u led-controller.service --since "${SINCE}" --no-pager \
        > "${DIAGNOSTICS}/service_journal.log" 2>&1 || true
else
    journalctl -u tractor-wifi-control.service -u teensy-bridge.service \
        -u rtcm-server.service -u led-controller.service \
        --since "${SINCE}" --no-pager \
        > "${DIAGNOSTICS}/service_journal.log" 2>&1 || true
fi

{
    git -C "${TRACTOR_REPO}" rev-parse HEAD
    git -C "${TRACTOR_REPO}" status --short
} > "${DIAGNOSTICS}/repository_state.txt" 2>&1 || true

{
    ip -4 address show || true
    echo
    ip -4 route show || true
    echo
    iw dev wlan0 link 2>/dev/null || true
} > "${DIAGNOSTICS}/network_state.txt" 2>&1

(
    cd "${STAGING}"
    find . -type f ! -name 'collected_file_hashes.sha256' -print0 \
        | sort -z | xargs -0 sha256sum
) > "${DIAGNOSTICS}/collected_file_hashes.sha256"

mission_count="$(find "${STAGING}/mission_logs" -type f | wc -l)"
wifi_count="$(find "${STAGING}/wifi_logs" -type f | wc -l)"
pursuit_count="$(find "${STAGING}/pursuit_logs" -type f | wc -l)"
echo "Mission files: ${mission_count}; Wi-Fi logs: ${wifi_count}; pursuit logs: ${pursuit_count}"

tar -C "${STAGING}" -czf "${ARCHIVE}" .
sync
archive_hash="$(sha256sum "${ARCHIVE}")"
ln -sfn -- "$(basename "${ARCHIVE}")" "${LATEST_LINK}"
{
    echo "${ARCHIVE}"
    echo "${archive_hash}"
} > "${LATEST_INFO}"

echo "ARCHIVE READY"
ls -lh "${ARCHIVE}"
echo "${archive_hash}"
echo "Stable copy path: ${LATEST_LINK}"
echo "Archive details:  ${LATEST_INFO}"
