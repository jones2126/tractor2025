#!/usr/bin/env bash
set -euo pipefail

python3 -m unittest \
  tractor_rpi.testing.test_rpi_command_lease_protocol_20261031 \
  tractor_rpi.testing.test_rpi_command_lease_adapter_and_faults_20261031 \
  tractor_rpi.testing.test_rpi_command_lease_runtime_20261031 \
  tractor_rpi.testing.test_communications_degraded_guard_20261029 \
  tractor_rpi.testing.test_rpi_safety_supervisor_20261031 \
  tractor_rpi.testing.test_rpi_arbitrator_inputs_20261031 \
  tractor_rpi.testing.test_rpi_arbitrator_wire_20261031 \
  tractor_rpi.testing.test_rpi_arbitrator_producers_20261031 \
  tractor_rpi.testing.test_rpi_arbitrator_motor_off_consumer_20261031 \
  tractor_rpi.testing.test_rpi_arbitrator_production_bridge_20261104 \
  tractor_rpi.testing.test_wifi_primary_control_20261022 \
  tractor_rpi.testing.test_wifi_rpi_arbitrator_control_20261104 \
  tractor_rpi.testing.test_teensy_rpi_command_lease_v3_source_20261104 \
  tractor_rpi.testing.test_rpi_full_arbitrator_fault_simulation_20261031

python3 -m tractor_rpi.testing.rpi_full_arbitrator_fault_simulation_20261031
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED/verify_package_20261104.py
