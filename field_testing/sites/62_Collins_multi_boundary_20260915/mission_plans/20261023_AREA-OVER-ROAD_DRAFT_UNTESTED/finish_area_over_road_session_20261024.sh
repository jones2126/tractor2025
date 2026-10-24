#!/usr/bin/env bash
# End-of-session evidence capture and shutdown checklist. Does not power off automatically.
set -euo pipefail

TRACTOR_REPO="${TRACTOR_REPO:-/home/al/tractor2025}"
PACKAGE="${TRACTOR_REPO}/field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED"

if pgrep -f '[p]ython3.*pure_pursuit_controller_' >/dev/null; then
    echo "ERROR: a mission controller is still running." >&2
    echo "Keep the tractor in Pause, stop the mission from the dashboard, then rerun this script." >&2
    exit 1
fi

bash "${PACKAGE}/collect_area_over_road_postrun_20261024.sh" "$@"

echo
echo "================ SHUTDOWN CHECKLIST ================"
echo "1. Confirm phone control is in Pause and drive command is zero."
echo "2. Park safely, stop the engine, and close the dashboard with Ctrl+C."
echo "3. Copy the telemetry archive and its SHA-256 record off Tractor01."
echo "4. Verify the copied archive hash before powering Tractor01 down."
echo "5. DOWNLOAD THE SYSTEM LOG FROM THE ARCHER AXE75 AT THE RTK BASE."
echo "   It provides the TractorField network. Preserve the original log unchanged."
echo "6. Record the AXE75 clock/time zone and download time with the field notes."
echo "====================================================="
echo "This script intentionally does not issue a Linux shutdown command."
