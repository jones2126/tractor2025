#!/usr/bin/env bash
set -euo pipefail

# DRAFT - UNTESTED IN THE FIELD
# Deterministic, motion-free software fault suite for Approval A continuation.
# This does not use tc/netem, change interfaces, send UDP 6004, enable
# COMMUNICATIONS DEGRADED, or authorize tractor movement.

cd /home/al/tractor2025

log_dir=${1:-/home/al/field_logs/20261030_stationary_direct_retest}
mkdir -p "$log_dir"

python3 -m unittest -v \
  tractor_rpi.testing.test_wifi_primary_control_20261022 \
  tractor_rpi.testing.test_communications_degraded_guard_20261029 \
  tractor_rpi.testing.test_mission_dashboard_stale_state_20261019 \
  tractor_rpi.testing.test_mission_dashboard_phone_diagnostics_20261029 \
  tractor_rpi.testing.test_republish_wifi_control_url_20261029 \
  2>&1 | tee "$log_dir/software_fault_tests.txt"

printf '%s\n' \
  'PASS: motion-free software fault suite completed.' \
  'This result does not authorize movement or enable COMMUNICATIONS DEGRADED.'
