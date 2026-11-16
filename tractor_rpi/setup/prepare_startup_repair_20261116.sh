#!/usr/bin/env bash
# DRAFT: --plan is read-only. --apply requires separate installation approval.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd -- "${HERE}/../.." && pwd)"
case "${1:---plan}" in
    --plan)
        cat <<'PLAN'
PROPOSED STARTUP REPAIR — NO CHANGES MADE
1. Require root, engine-off/stationary confirmation, exact commit, and a clean checkout.
2. Require both production arbitrators to be inactive before installation.
3. Back up installed startup-preflight and legacy bridge units/drop-ins and enablement states.
4. Stop/disable the legacy startup-preflight and four retired control services.
5. Move the old bridge unit into the backup and persistently mask teensy-bridge.service.
6. Install the migrated startup-preflight unit; preserve its other installed drop-ins (none expected).
7. Reload systemd, enable both existing production arbitrators and migrated startup-preflight.
8. Verify the effective dependencies contain no retired bridge or Wi-Fi owner.
No production control code, firmware, GPS service, route or credentials are changed.
No production arbitrator is started. Operator reboot/activation and validation are separate.
PLAN
        exit 0 ;;
    --apply) ;;
    *) echo "Usage: $0 [--plan | --apply EXPECTED_COMMIT ENGINE_OFF_STATIONARY_BLADES_OFF]" >&2; exit 2 ;;
esac
[[ $# -eq 3 && "$3" == ENGINE_OFF_STATIONARY_BLADES_OFF ]] || { echo "Exact commit and stationary confirmation required" >&2; exit 2; }
[[ "${EUID}" -eq 0 ]] || { echo "Installation requires sudo after approval" >&2; exit 2; }
[[ "${REPO}" == /home/al/tractor2025 && "$(uname -s)" == Linux ]] || { echo "Expected the Tractor01 production checkout on Linux" >&2; exit 2; }
command -v fuser >/dev/null || { echo "fuser is required to verify serial ownership" >&2; exit 2; }
[[ "$(git -C "${REPO}" rev-parse HEAD)" == "$2" ]] || { echo "Unexpected commit" >&2; exit 2; }
[[ "$(git -C "${REPO}" branch --show-current)" == main ]] || { echo "Expected main branch" >&2; exit 2; }
[[ -z "$(git -C "${REPO}" status --porcelain)" ]] || { echo "Repository must be clean" >&2; exit 2; }
for service in tractor-rpi-arbitrator.service tractor-phone-arbitrator.service; do
    state="$(systemctl show "${service}" -p ActiveState --value)"
    [[ "${state}" == inactive || "${state}" == failed ]] || { echo "Refusing to alter an active production stack: ${service}=${state}" >&2; exit 2; }
    # This repair does not install replacement production units: first prove
    # the installed owners are the reviewed ones, without unknown drop-ins.
    package="${REPO}/field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED"
    cmp "${package}/${service}" "/etc/systemd/system/${service}"
    [[ -z "$(systemctl show "${service}" -p DropInPaths --value)" ]] || { echo "Unexpected production drop-in" >&2; exit 2; }
done
[[ -z "$(systemctl show tractor-startup-preflight.service -p DropInPaths --value)" ]] || { echo "Review unexpected startup-preflight drop-ins first" >&2; exit 2; }
backup="/var/backups/tractor-startup-repair-$(date -u '+%Y%m%dT%H%M%SZ')"
mkdir -m 0700 -- "${backup}"
git -C "${REPO}" rev-parse HEAD > "${backup}/installed_commit.txt"
for service in tractor-startup-preflight.service teensy-bridge.service tractor-wifi-control.service tractor-wifi-direct-mission.service tractor-wifi-recovery.service tractor-rpi-arbitrator.service tractor-phone-arbitrator.service; do
    systemctl is-enabled "${service}" > "${backup}/${service}.enablement" 2>&1 || true
    systemctl cat "${service}" > "${backup}/${service}.effective.txt" 2>&1 || true
    for item in "/etc/systemd/system/${service}" "/etc/systemd/system/${service}.d"; do
        if [[ -e "${item}" || -L "${item}" ]]; then cp -a -- "${item}" "${backup}/"; fi
    done
done
(cd -- "${backup}" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS)
echo "Backup: ${backup}"
# Stop the caller first, so it cannot pull the old owner back in.
systemctl stop tractor-startup-preflight.service
systemctl disable tractor-startup-preflight.service
for service in tractor-wifi-control.service tractor-wifi-direct-mission.service tractor-wifi-recovery.service teensy-bridge.service; do
    systemctl stop "${service}"
    # Static units have no boot symlink to remove; disable is harmless for those.
    systemctl disable "${service}"
done
if fuser /dev/teensy >/dev/null 2>&1; then
    echo "Unexpected Teensy owner remains after retiring services; inspect before continuing." >&2
    exit 1
fi
if [[ ! -L /etc/systemd/system/teensy-bridge.service || "$(readlink /etc/systemd/system/teensy-bridge.service)" != /dev/null ]]; then
    if [[ -e /etc/systemd/system/teensy-bridge.service || -L /etc/systemd/system/teensy-bridge.service ]]; then
        mv -- /etc/systemd/system/teensy-bridge.service "${backup}/retired-teensy-bridge.service"
        (cd -- "${backup}" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS)
    fi
    systemctl mask teensy-bridge.service
fi
install -m 0644 "${HERE}/tractor-startup-preflight.service" /etc/systemd/system/tractor-startup-preflight.service
systemctl daemon-reload
systemctl enable tractor-rpi-arbitrator.service tractor-phone-arbitrator.service tractor-startup-preflight.service
[[ "$(systemctl is-enabled teensy-bridge.service)" == masked ]] || { echo "Legacy bridge mask missing" >&2; exit 1; }
dependencies="$(systemctl show tractor-startup-preflight.service -p Requires -p Wants -p After --value)"
if grep -Eq 'teensy-bridge|tractor-wifi-control|tractor-wifi-direct-mission|tractor-wifi-recovery' <<< "${dependencies}"; then
    echo "FAIL: stale startup dependency remains. Do not reboot or drive; inspect backup and units." >&2
    exit 1
fi
for service in rtcm-server.service tractor-rpi-arbitrator.service tractor-phone-arbitrator.service; do
    grep -Fq "${service}" <<< "${dependencies}" || { echo "Missing dependency ${service}" >&2; exit 1; }
done
echo "Installation prepared for next boot; no production services started."
echo "Backup retained: ${backup}. Reboot and stationary preflight validation require operator action."
