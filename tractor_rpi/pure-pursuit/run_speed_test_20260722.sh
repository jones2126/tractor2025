#!/usr/bin/env bash
# run_mission_20260717.sh
# Starts the field data logger AND the pure pursuit controller together.
# The logger runs in the background; the controller runs in the foreground.
# Both are stopped cleanly when the controller exits (normally or Ctrl+C).
# Run from any directory -- script resolves its own location.
#
# Usage:
#   ./run_mission_20260717.sh                        # default: RTK Fixed required
#   ./run_mission_20260717.sh --min-fix "RTK Float"  # relaxed gate for testing
#
# Prerequisites (must already be running as services):
#   rtcm-server   (broadcasts GPS on UDP 6002)
#   teensy-bridge (forwards cmd_vel from UDP 6004 to Teensy over USB)
#
# Startup sequence:
#   1. SSH into tractor01
#   2. Run this script -- logger starts immediately; controller prints [WAIT]
#      until RTK Fixed + headValid
#   3. Drive to starting position, mode switch in MANUAL or PAUSE
#   4. Confirm console shows:  idx=0/85 ... fix=RTK Fixed
#   5. Flip mode switch DOWN (Auto / mode 0)
#   6. Watch idx advance -- Ctrl+C or flip out of Auto to abort
#      Logger CSV is written to ~/repos/field-testing-data/ regardless of
#      how the run ends.
#
# Repository location: tractor_rpi/pure-pursuit/run_mission_20260717.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRACTOR_RPI_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONTROLLER="pure_pursuit_controller_20260714.py"
MISSION="mission_speed_test_100ft_right_20260722.txt"
LOGGER="field_test_logger_20260728.py"

# ---- file existence checks ------------------------------------------------
if [[ ! -f "${SCRIPT_DIR}/${CONTROLLER}" ]]; then
    echo "ERROR: ${CONTROLLER} not found in ${SCRIPT_DIR}"
    exit 1
fi
if [[ ! -f "${SCRIPT_DIR}/${MISSION}" ]]; then
    echo "ERROR: ${MISSION} not found in ${SCRIPT_DIR}"
    exit 1
fi
if [[ ! -f "${TRACTOR_RPI_DIR}/${LOGGER}" ]]; then
    echo "ERROR: ${LOGGER} not found in ${TRACTOR_RPI_DIR}"
    exit 1
fi

# ---- trap: stop logger when controller exits for any reason ---------------
LOGGER_PID=""
cleanup() {
    if [[ -n "${LOGGER_PID}" ]] && kill -0 "${LOGGER_PID}" 2>/dev/null; then
        echo ""
        echo "Stopping logger (PID ${LOGGER_PID})..."
        kill "${LOGGER_PID}"
        wait "${LOGGER_PID}" 2>/dev/null || true
        echo "Logger stopped."
    fi
}
trap cleanup EXIT

# ---- banner ---------------------------------------------------------------
echo "========================================"
echo "  tractor2025 -- pure pursuit mission"
echo "  $(date '+%Y-%m-%d %H:%M:%S')"
echo "  Mission    : ${MISSION}"
echo "  Controller : ${CONTROLLER}"
echo "  Logger     : ${LOGGER}"
echo "========================================"

# ---- start logger in background -------------------------------------------
echo "Starting field data logger..."
python3 "${TRACTOR_RPI_DIR}/${LOGGER}" &
LOGGER_PID=$!
echo "Logger running (PID ${LOGGER_PID})"
echo ""
echo "Waiting for RTK Fixed + headValid on UDP 6002..."
echo "Flip mode switch DOWN (Auto) once fix is confirmed."
echo "Ctrl+C to abort at any time."
echo ""

# ---- start controller in foreground (blocks until done or Ctrl+C) ---------
cd "${SCRIPT_DIR}"
python3 "${CONTROLLER}" \
    "${MISSION}" \
    --mode live \
    --ip 127.0.0.1 \
    --port 6004 \
    --max-speed 1.5 \
    "$@"
