#!/usr/bin/env bash
set -euo pipefail

python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261031_RPI_COMMAND_LEASE_PHONE_MOTOR_OFF_DRAFT_UNTESTED/verify_phone_motor_off_package_20261031.py

python3 -m unittest \
  tractor_rpi.testing.test_rpi_command_lease_change_boundary_20261031 \
  tractor_rpi.testing.test_rpi_command_lease_protocol_20261031 \
  tractor_rpi.testing.test_teensy_rpi_command_lease_source_20261031 \
  tractor_rpi.testing.test_rpi_command_lease_adapter_and_faults_20261031 \
  tractor_rpi.testing.test_rpi_safety_supervisor_20261031 \
  tractor_rpi.testing.test_rpi_command_lease_runtime_20261031 \
  tractor_rpi.testing.test_rpi_command_lease_pause_commission_20261031 \
  tractor_rpi.testing.test_rpi_command_lease_pause_bridge_20261031
