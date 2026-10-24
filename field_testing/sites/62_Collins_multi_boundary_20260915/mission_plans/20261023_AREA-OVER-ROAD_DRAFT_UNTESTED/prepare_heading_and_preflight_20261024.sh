#!/usr/bin/env bash
# Mandatory non-motion Heading-F9P preparation and live preflight for AREA-OVER-ROAD.
set -u

TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
CONFIGURATOR="${TRACTOR_REPO}/tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py"
PREFLIGHT="${TRACTOR_REPO}/tractor_rpi/testing/mission_preflight_20261002.py"
EXPECTED_FIRMWARE="teensy_main_20261003_wifi_v3"

echo "Keep the tractor stationary in PAUSE with blades disengaged."
echo "This writes the established 5 Hz Heading-F9P profile to volatile RAM only."

cd "${TRACTOR_REPO}" || exit 1
sudo systemctl stop rtcm-server.service || {
    echo "FAIL: could not stop rtcm-server; DO NOT SELECT AUTO" >&2
    exit 1
}

sudo python3 -u "${CONFIGURATOR}" --heading-startup --device-wait-seconds 30
config_rc=$?

sudo systemctl start rtcm-server.service
start_rc=$?
service_state="$(systemctl is-active rtcm-server.service 2>/dev/null || true)"
echo "rtcm-server.service: ${service_state}"

if [[ ${config_rc} -ne 0 || ${start_rc} -ne 0 || "${service_state}" != "active" ]]; then
    echo "FAIL: Heading configuration or rtcm-server restart failed — DO NOT SELECT AUTO" >&2
    exit 1
fi

echo "PASS: Heading profile independently verified and rtcm-server is active."
echo "Waiting 60 seconds for RTK position and fixed-carrier heading reacquisition..."
sleep 60

sudo python3 "${PREFLIGHT}" --expected-firmware "${EXPECTED_FIRMWARE}"
