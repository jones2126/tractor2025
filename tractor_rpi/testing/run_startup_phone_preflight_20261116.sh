#!/usr/bin/env bash
# Non-motion production boot preflight. Notifications never grant authority.
set -uo pipefail

TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
PREFLIGHT="${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_phone_control_20261109.py"
EXPECTED_FIRMWARE="teensy_main_20261110_rpi_command_lease_v5"
NTFY_TOPIC="${TRACTOR_NTFY_TOPIC:-rpi-tractor01-jones2126}"
NETWORK_WAIT_SECONDS="${TRACTOR_NETWORK_WAIT_SECONDS:-300}"
NETWORK_RETRY_SECONDS="${TRACTOR_NETWORK_RETRY_SECONDS:-3}"
for value in "${NETWORK_WAIT_SECONDS}" "${NETWORK_RETRY_SECONDS}"; do
    [[ "${value}" =~ ^[1-9][0-9]*$ ]] || { echo "Invalid network wait/retry interval" >&2; exit 2; }
done

notify() {
    curl --silent --show-error --fail --connect-timeout 3 --max-time 10 \
        -H "Title: $1" -H "Priority: $2" -H "Tags: tractor" \
        --data-binary "$3" "https://ntfy.sh/${NTFY_TOPIC}" >/dev/null || \
        echo "WARNING: best-effort ntfy publish failed; check local preflight output." >&2
}

network_ready() {
    ip -4 -o address show dev eth0 2>/dev/null | grep -Fq 'inet 192.168.8.20/24' || return 1
    ip -4 route get 1.1.1.1 2>/dev/null |
        grep -Eq 'via 192\.168\.8\.1 .*dev eth0 .*src 192\.168\.8\.20' || return 1
    ping -I eth0 -c 1 -W 2 192.168.8.1 >/dev/null 2>&1 || return 1
    ip -4 route get 192.168.193.88 2>/dev/null |
        grep -Eq 'dev zt[^ ]+ .*src 192\.168\.193\.76' || return 1
    for service in rtcm-server.service tractor-rpi-arbitrator.service tractor-phone-arbitrator.service; do
        systemctl is-active --quiet "${service}" || return 1
    done
}

echo "Waiting for the production control services and qualified network path."
deadline=$((SECONDS + NETWORK_WAIT_SECONDS))
while ! network_ready; do
    if (( SECONDS >= deadline )); then
        echo "NETWORK/SERVICES NOT READY: no preflight PASS. Retry later; no motion authorized." >&2
        exit 75
    fi
    sleep "${NETWORK_RETRY_SECONDS}"
done

# ntfy delivery is optional. Failure must never suppress the local preflight.
notify "Tractor01 startup preflight" default \
    "Production startup checks beginning. No mission or motion requested; keep engine off and blades disengaged."
output="$(mktemp)" || exit 2
trap 'rm -f -- "${output}"' EXIT
python3 -u "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}" | tee "${output}"
statuses=("${PIPESTATUS[@]}")
if [[ "${statuses[0]}" -eq 0 && "${statuses[1]}" -eq 0 && "$(tail -n 1 "${output}")" == 'MISSION PREFLIGHT PASS' ]]; then
    notify "Tractor01 startup preflight PASS" default \
        "Canonical production preflight passed. This grants no motion authority; a fresh mission preflight is still required."
    exit 0
fi
notify "Tractor01 startup preflight FAILED" high \
    "No verified production preflight PASS. Keep engine off; inspect journalctl -b -u tractor-startup-preflight.service."
# A completed failure is retained for inspection rather than retried forever.
exit 1
