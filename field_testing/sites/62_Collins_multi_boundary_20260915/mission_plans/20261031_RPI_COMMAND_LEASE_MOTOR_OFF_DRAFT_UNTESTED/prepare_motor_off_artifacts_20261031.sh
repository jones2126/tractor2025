#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${TRACTOR_REPO:-/home/al/tractor2025-rpi-lease}"
EVIDENCE_DIR="${TRACTOR_MOTOR_OFF_EVIDENCE_DIR:-/home/al/field_logs/20261031_rpi_command_lease_motor_off}"
PACKAGE_REL="field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261031_RPI_COMMAND_LEASE_MOTOR_OFF_DRAFT_UNTESTED"

cd "$REPO_ROOT"
mkdir -p "$EVIDENCE_DIR"

git rev-parse HEAD | tee "$EVIDENCE_DIR/git-head.txt"
git status --short --branch | tee "$EVIDENCE_DIR/git-status-before.txt"

python3 "$PACKAGE_REL/verify_motor_off_package_20261031.py" \
  2>&1 | tee "$EVIDENCE_DIR/package-verifier.txt"

python3 -m unittest \
  tractor_rpi.testing.test_rpi_command_lease_change_boundary_20261031 \
  tractor_rpi.testing.test_rpi_command_lease_protocol_20261031 \
  tractor_rpi.testing.test_teensy_rpi_command_lease_source_20261031 \
  tractor_rpi.testing.test_rpi_command_lease_adapter_and_faults_20261031 \
  tractor_rpi.testing.test_rpi_safety_supervisor_20261031 \
  tractor_rpi.testing.test_rpi_command_lease_runtime_20261031 \
  tractor_rpi.testing.test_rpi_command_lease_pause_commission_20261031 \
  2>&1 | tee "$EVIDENCE_DIR/motor-off-software-tests.txt"

cd "$REPO_ROOT/tractor_teensy"

pio run --project-conf platformio.wifi-primary.ini \
  2>&1 | tee "$EVIDENCE_DIR/rollback-build.txt"
cp -p .pio/build/teensy41/firmware.hex \
  "$EVIDENCE_DIR/teensy_main_20261003_wifi_v3_ROLLBACK.hex"

pio run --project-conf platformio.rpi-command-lease.ini \
  2>&1 | tee "$EVIDENCE_DIR/candidate-build.txt"
cp -p .pio/build/teensy41/firmware.hex \
  "$EVIDENCE_DIR/teensy_main_20261031_rpi_command_lease_v1_CANDIDATE.hex"

cd "$EVIDENCE_DIR"
sha256sum \
  teensy_main_20261003_wifi_v3_ROLLBACK.hex \
  teensy_main_20261031_rpi_command_lease_v1_CANDIDATE.hex \
  > FIRMWARE_SHA256SUMS
sha256sum -c FIRMWARE_SHA256SUMS

echo "PASS: build-only artifacts prepared; no firmware was uploaded."
echo "STOP: provide verifier, test, build, and SHA256 output for separate flash approval."
