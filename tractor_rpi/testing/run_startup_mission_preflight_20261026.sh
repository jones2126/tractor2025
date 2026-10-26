#!/usr/bin/env bash
# Non-motion Tractor01 boot preflight with best-effort ntfy notices.
set -uo pipefail

TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
PREFLIGHT="${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_20261002.py"
EXPECTED_FIRMWARE="${TRACTOR_EXPECTED_FIRMWARE:-teensy_main_20261003_wifi_v3}"
NTFY_TOPIC="${TRACTOR_NTFY_TOPIC:-rpi-tractor01-jones2126}"
NETWORK_WAIT_SECONDS="${TRACTOR_NETWORK_WAIT_SECONDS:-300}"
NETWORK_RETRY_SECONDS="${TRACTOR_NETWORK_RETRY_SECONDS:-3}"
NETWORK_PROBE_URL="${TRACTOR_NETWORK_PROBE_URL:-https://ntfy.sh/}"

for value_name in NETWORK_WAIT_SECONDS NETWORK_RETRY_SECONDS; do
    value="${!value_name}"
    [[ "${value}" =~ ^[1-9][0-9]*$ ]] || {
        echo "ERROR: ${value_name} must be a positive integer; got ${value@Q}" >&2
        exit 2
    }
done

notify() {
    local title="$1"
    local priority="$2"
    local message="$3"
    curl --silent --show-error --fail --max-time 10 \
        -H "Title: ${title}" \
        -H "Priority: ${priority}" \
        -H "Tags: tractor" \
        --data-binary "${message}" \
        "https://ntfy.sh/${NTFY_TOPIC}" >/dev/null || \
        echo "WARNING: ntfy notice could not be delivered" >&2
}

network_ready() {
    ip -4 -o address show dev eth0 2>/dev/null |
        grep -Fq 'inet 192.168.8.20/24' || return 1
    ip -4 route get 1.1.1.1 2>/dev/null |
        grep -Eq 'via 192\.168\.8\.1 .*dev eth0 .*src 192\.168\.8\.20' || return 1
    ping -I eth0 -c 1 -W 2 192.168.8.1 >/dev/null 2>&1 || return 1
    ip -4 route get 192.168.193.88 2>/dev/null |
        grep -Eq 'dev zt[^ ]+ .*src 192\.168\.193\.76' || return 1
    systemctl is-active --quiet tractor-wifi-control.service || return 1
    curl --silent --show-error --fail --connect-timeout 3 --max-time 8 \
        "${NETWORK_PROBE_URL}" >/dev/null || return 1
}

wait_for_network() {
    local deadline=$((SECONDS + NETWORK_WAIT_SECONDS))
    echo "Waiting for the qualified Tractor01 network path and ntfy reachability."
    while (( SECONDS < deadline )); do
        if network_ready; then
            echo "Qualified network path is ready; starting automatic preflight."
            return 0
        fi
        sleep "${NETWORK_RETRY_SECONDS}"
    done

    echo "NETWORK NOT READY after ${NETWORK_WAIT_SECONDS}s." >&2
    echo "Automatic preflight and ntfy notices were not attempted." >&2
    echo "Expected: eth0 192.168.8.20 via 192.168.8.1, GL gateway reachable," >&2
    echo "ZeroTier route to 192.168.193.88 from 192.168.193.76, phone control active," >&2
    echo "and ntfy.sh reachable." >&2
    return 1
}

# Exit 75 tells systemd this was a temporary network condition. The service
# retries later without treating an untested preflight as a PASS or attempting
# an ntfy publish over a path that has not yet been established.
wait_for_network || exit 75

echo "Automatic Tractor01 startup preflight kicked off."
echo "This does not start a mission or grant motion authority."
notify \
    "Tractor01 startup preflight" \
    "default" \
    "Kicked off automatically after boot. Tractor remains in Pause; no mission or motion was started."

if python3 -u "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}"; then
    notify \
        "Tractor01 startup preflight PASS" \
        "default" \
        "Automatic boot preflight passed. A fresh preflight is still required by the mission launcher before Auto."
    exit 0
else
    status=$?
fi

notify \
    "Tractor01 startup preflight FAILED" \
    "high" \
    "Automatic boot preflight failed. Keep the tractor in Pause and do not select Auto. View the full log: journalctl -u tractor-startup-preflight.service -b --no-pager -o cat"
# Normalize a completed preflight failure to status 1. The systemd unit does
# not retry this result; it waits for manual inspection and an explicit rerun.
exit 1
