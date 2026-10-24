#!/usr/bin/env bash
# DRAFT - UNTESTED IN THE FIELD
# Fail-closed placeholder used by the dashboard review package.
set -euo pipefail

TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
PACKAGE="${TRACTOR_REPO}/field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED"
APPROVED_FOR_FIELD=false

if [[ "${1:-}" == "--verify-only" ]]; then
    python3 "${PACKAGE}/verify_area_over_road_review_package_20261024.py"
    exit 0
fi

echo "LIVE START BLOCKED — DRAFT - UNTESTED IN THE FIELD" >&2
echo "Controller-format route revision 003 is available for dashboard review." >&2
echo "Review and explicitly approve the field package before enabling field motion." >&2
echo "APPROVED_FOR_FIELD=${APPROVED_FOR_FIELD}" >&2
exit 1
