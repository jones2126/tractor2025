#!/usr/bin/env bash
set -euo pipefail

cd "${TRACTOR_REPO:-/home/al/tractor2025}"
python3 -m unittest \
  tractor_rpi.testing.test_wifi_primary_control_20261022 \
  tractor_rpi.testing.test_manual_recovery_20261019 \
  tractor_rpi.testing.test_communications_degraded_guard_20261029 \
  tractor_rpi.testing.test_mission_dashboard_stale_state_20261019 \
  tractor_rpi.testing.test_mission_dashboard_phone_diagnostics_20261029 \
  tractor_rpi.testing.test_republish_wifi_control_url_20261029
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261031_APPROVAL_B_DIRECT_OUTER_PERIMETER_DRAFT_UNTESTED/verify_approval_b_package_20261030.py
