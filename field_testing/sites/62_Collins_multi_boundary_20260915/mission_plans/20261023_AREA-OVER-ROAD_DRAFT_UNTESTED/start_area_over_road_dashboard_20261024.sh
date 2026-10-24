#!/usr/bin/env bash
# Starts the review-only web dashboard. Does not start tractor motion.
set -euo pipefail

TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
PACKAGE="${TRACTOR_REPO}/field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED"
SESSION_DIR="/home/al/field_logs/20261024_area_over_road_review"
STAMP="$(date '+%Y%m%d_%H%M%S')"

mkdir -p "${SESSION_DIR}"
date --iso-8601=seconds > "${SESSION_DIR}/dashboard_session_start.txt"
python3 "${PACKAGE}/verify_area_over_road_review_package_20261024.py"

echo "============================================================"
echo " AREA-OVER-ROAD WEB DASHBOARD — REVIEW ONLY"
echo " DRAFT - UNTESTED IN THE FIELD"
echo " Start is disabled; this script cannot command tractor motion."
echo "============================================================"

set +e
python3 -u "${PACKAGE}/mission_dashboard_area_over_road_review_20261024.py" \
    2>&1 | tee "${SESSION_DIR}/dashboard_${STAMP}.log"
status="${PIPESTATUS[0]}"
set -e
exit "${status}"
