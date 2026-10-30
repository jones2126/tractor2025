#!/usr/bin/env bash
# Starts route/telemetry review only. Cannot start the logger or Pure Pursuit.
set -euo pipefail

REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
PACKAGE="${REPO}/field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261031_APPROVAL_B_DIRECT_OUTER_PERIMETER_DRAFT_UNTESTED"
SESSION_DIR="/home/al/field_logs/20261031_approval_b_review"
STAMP="$(date '+%Y%m%d_%H%M%S')"

mkdir -p "${SESSION_DIR}"
date --iso-8601=seconds > "${SESSION_DIR}/dashboard_session_start.txt"
python3 "${PACKAGE}/verify_approval_b_package_20261030.py"
echo "Review dashboard only: Start, logger, and Pure Pursuit are disabled."
set +e
python3 -u "${PACKAGE}/mission_dashboard_approval_b_review_20261031.py" \
    2>&1 | tee "${SESSION_DIR}/dashboard_${STAMP}.log"
status="${PIPESTATUS[0]}"
set -e
exit "${status}"
