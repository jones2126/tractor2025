#!/usr/bin/env python3
"""Review-only dashboard adapter for AREA-OVER-ROAD.

The map and telemetry can be reviewed, but Start is deliberately disabled.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[4] / "tractor_rpi/pure-pursuit/mission_dashboard_20260910.py"
spec = importlib.util.spec_from_file_location("mission_dashboard_base", SOURCE)
dashboard = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(dashboard)

dashboard.MISSION = HERE / "generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024.txt"
dashboard.AUDIT = HERE / "generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024_audit.csv"
dashboard.LAUNCHER = HERE / "run_area_over_road_review_only_20261024.sh"
dashboard.EXPECTED_CONFIRMATION = "FIELD START DISABLED"


def review_only_safe_to_start(_state):
    return False, "Controller route review pending; field authorization is disabled"


dashboard.safe_to_start = review_only_safe_to_start

replacements = {
    "Tractor01 — 62 Collins clear-sky resume": "Tractor01 — AREA-OVER-ROAD review only",
    "Resumes at source waypoint 91. Recovery stays in the current phase and may advance at most 30 m.":
        "DRAFT - UNTESTED IN THE FIELD. Controller-format route revision 003 is loaded; Start remains disabled pending review and explicit field authorization.",
    "AUTO NOT READY — START MISSION": "FIELD START DISABLED — REVIEW ONLY",
    "START MISSION": "START DISABLED",
}
for old, new in replacements.items():
    dashboard.HTML = dashboard.HTML.replace(old, new)

if __name__ == "__main__":
    print("DRAFT - UNTESTED IN THE FIELD")
    print("Review-only dashboard: live mission start is disabled.")
    dashboard.main()
