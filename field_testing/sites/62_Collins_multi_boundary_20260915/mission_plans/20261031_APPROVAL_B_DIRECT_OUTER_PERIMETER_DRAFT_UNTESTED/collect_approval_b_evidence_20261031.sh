#!/usr/bin/env bash
# Copy (never delete) Approval B evidence into a timestamped, hashed archive.
set -euo pipefail

REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
PACKAGE_REL="field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261031_APPROVAL_B_DIRECT_OUTER_PERIMETER_DRAFT_UNTESTED"
PACKAGE="${REPO}/${PACKAGE_REL}"
ROUTE_REL="field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260928_outer_perimeter_field_test"
ROUTE="${REPO}/${ROUTE_REL}"
SINCE=""
OUTPUT=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --since) SINCE="${2:-}"; shift 2 ;;
        --output) OUTPUT="${2:-}"; shift 2 ;;
        *) echo "Usage: $0 --since 'YYYY-MM-DD HH:MM:SS' [--output /home/al/name.tgz]" >&2; exit 2 ;;
    esac
done

[[ -n "${SINCE}" ]] || { echo "ERROR: --since is required" >&2; exit 2; }
STAMP="$(date '+%Y%m%d_%H%M%S')"
ARCHIVE="${OUTPUT:-/home/al/tractor01_approval_b_${STAMP}.tgz}"
[[ "${ARCHIVE}" == /home/al/*.tgz ]] || { echo "ERROR: output must be /home/al/*.tgz" >&2; exit 2; }
[[ ! -e "${ARCHIVE}" ]] || { echo "ERROR: refusing to overwrite ${ARCHIVE}" >&2; exit 1; }

STAGING="$(mktemp -d '/tmp/tractor01-approval-b-XXXXXX')"
cleanup() {
    [[ "${STAGING}" == /tmp/tractor01-approval-b-* ]] || return 1
    rm -rf -- "${STAGING}"
}
trap cleanup EXIT
mkdir -p "${STAGING}"/{phone_control,pursuit,mission,diagnostics,package}

copy_recent() {
    local destination="$1"; shift
    local source
    while IFS= read -r -d '' source; do cp -p -- "${source}" "${destination}/"; done \
        < <(find "$@" -print0)
}

if [[ -d /home/al/field_logs/approval_b_direct ]]; then
    copy_recent "${STAGING}/phone_control" /home/al/field_logs/approval_b_direct \
        -maxdepth 1 -type f -newermt "${SINCE}"
fi
if [[ -d /home/al/field_logs ]]; then
    copy_recent "${STAGING}/mission" /home/al/field_logs -maxdepth 2 -type f \
        \( -name '*.csv' -o -name '*.jsonl' -o -name '*diagnostics*.json' -o -name '*notes*.txt' \) \
        -newermt "${SINCE}"
fi
if [[ -d /home/al/repos/field-testing-data ]]; then
    copy_recent "${STAGING}/pursuit" /home/al/repos/field-testing-data -maxdepth 1 \
        -type f -name 'pursuit_log_*.csv' -newermt "${SINCE}"
fi

cp -p -- "${PACKAGE}/README.md" "${PACKAGE}/approval_b_project.json" \
    "${PACKAGE}/checkpoint_ledger.jsonl" "${PACKAGE}/FIELD_CHECKLIST_20261031.txt" \
    "${STAGING}/package/"
cp -p -- "${ROUTE}/generated/62_Collins_outer_perimeter_field_test_1mps_20260928.txt" \
    "${ROUTE}/generated/62_Collins_outer_perimeter_field_test_1mps_20260928_audit.csv" \
    "${ROUTE}/generated/62_Collins_outer_perimeter_field_test_1mps_20260928_report.json" \
    "${STAGING}/package/"

{
    echo "Status: DRAFT - UNTESTED IN THE FIELD"
    echo "Collected: $(date --iso-8601=seconds)"
    echo "Since: ${SINCE}"
    hostname
    uname -a
    git -C "${REPO}" rev-parse HEAD
    git -C "${REPO}" status --short --branch
} > "${STAGING}/diagnostics/collection_metadata.txt" 2>&1
{
    systemctl is-active tractor-wifi-direct-mission.service tractor-wifi-control.service \
        tractor-wifi-recovery.service teensy-bridge.service rtcm-server.service || true
    systemctl status tractor-wifi-direct-mission.service tractor-wifi-control.service \
        tractor-wifi-recovery.service teensy-bridge.service rtcm-server.service \
        --no-pager -l || true
} > "${STAGING}/diagnostics/service_status.txt" 2>&1
journalctl -u tractor-wifi-direct-mission.service -u teensy-bridge.service \
    -u rtcm-server.service --since "${SINCE}" --no-pager \
    > "${STAGING}/diagnostics/service_journal.log" 2>&1 || true
{
    ip -4 address show
    ip -4 route show
    zerotier-cli peers 2>/dev/null || true
    ss -lntup 2>/dev/null || true
} > "${STAGING}/diagnostics/network_state.txt" 2>&1
cp -p /run/tractor-phone-control/diagnostics.json \
    "${STAGING}/diagnostics/" 2>/dev/null || true

cat > "${STAGING}/diagnostics/manual_followup.txt" <<'EOF'
Export the GL-SFT1200 system log and per-phone client statistics while powered.
Record its clock/time zone and download time. Preserve the original unchanged.
EOF
(
    cd "${STAGING}"
    find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum
) > "${STAGING}/SHA256SUMS"
tar -C "${STAGING}" -czf "${ARCHIVE}" .
sync
sha256sum "${ARCHIVE}" | tee "${ARCHIVE}.sha256"
echo "ARCHIVE READY: ${ARCHIVE}"
