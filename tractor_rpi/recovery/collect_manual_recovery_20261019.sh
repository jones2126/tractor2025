#!/usr/bin/env bash
# Collect recovery command, phone, checkpoint, network, and service evidence.
set -euo pipefail

SINCE="${1:-$(date '+%Y-%m-%d 00:00:00')}"
STAMP="$(date '+%Y%m%d_%H%M%S')"
OUTPUT="/home/al/tractor01_manual_recovery_${STAMP}.tgz"
STAGING="$(mktemp -d /tmp/tractor01-manual-recovery-XXXXXX)"
trap 'rm -rf -- "$STAGING"' EXIT
mkdir -p "$STAGING/recovery_logs" "$STAGING/diagnostics"

if [[ -d /home/al/field_logs/manual_recovery ]]; then
    find /home/al/field_logs/manual_recovery -maxdepth 1 -type f \
        -newermt "$SINCE" -exec cp -p -- {} "$STAGING/recovery_logs/" \;
fi

systemctl status tractor-wifi-recovery.service tractor-wifi-control.service \
    teensy-bridge.service --no-pager -l \
    > "$STAGING/diagnostics/service_status.txt" 2>&1 || true
journalctl -u tractor-wifi-recovery.service -u tractor-wifi-control.service \
    -u teensy-bridge.service --since "$SINCE" --no-pager \
    > "$STAGING/diagnostics/service_journal.log" 2>&1 || true
ip -4 address show > "$STAGING/diagnostics/network_state.txt" 2>&1 || true
pgrep -af 'pure_pursuit_controller_|mission_dashboard_' \
    > "$STAGING/diagnostics/mission_processes.txt" 2>&1 || true
git -C /home/al/tractor2025 status --short \
    > "$STAGING/diagnostics/repository_state.txt" 2>&1 || true

(
    cd "$STAGING"
    find . -type f -print0 | sort -z | xargs -0 sha256sum
) > "$STAGING/diagnostics/collected_file_hashes.sha256"
tar -C "$STAGING" -czf "$OUTPUT" .
sha256sum "$OUTPUT"
echo "Recovery archive ready: $OUTPUT"
