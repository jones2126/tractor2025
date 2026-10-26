#!/usr/bin/env bash
# Non-motion Tractor01 boot preflight with best-effort ntfy notices.
set -uo pipefail

TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
PREFLIGHT="${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_20261002.py"
EXPECTED_FIRMWARE="${TRACTOR_EXPECTED_FIRMWARE:-teensy_main_20261003_wifi_v3}"
NTFY_TOPIC="${TRACTOR_NTFY_TOPIC:-rpi-tractor01-jones2126}"

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
    "Automatic boot preflight failed. Keep the tractor in Pause and do not select Auto; inspect the service journal."
exit "${status}"
