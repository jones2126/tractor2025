#!/usr/bin/env bash
set -euo pipefail

python3 -m unittest \
  tractor_rpi.testing.test_communications_degraded_guard_20261029 \
  tractor_rpi.testing.test_rpi_safety_supervisor_20261031 \
  tractor_rpi.testing.test_rpi_command_lease_runtime_20261031 \
  tractor_rpi.testing.test_rpi_arbitrator_inputs_20261031 \
  tractor_rpi.testing.test_rpi_arbitrator_wire_20261031 \
  tractor_rpi.testing.test_rpi_arbitrator_producers_20261031 \
  tractor_rpi.testing.test_rpi_arbitrator_motor_off_consumer_20261031 \
  tractor_rpi.testing.test_rpi_full_arbitrator_fault_simulation_20261031

python3 -m tractor_rpi.testing.rpi_full_arbitrator_fault_simulation_20261031

echo "PASS: disabled full-arbitrator motor-off software package verified."
echo "NOT FLASH AUTHORIZATION. NOT MOTION AUTHORIZATION."
