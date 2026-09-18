#!/usr/bin/env python3
"""Dashboard wrapper for the 2026-09-18 all-1.0-m/s continuation."""

from __future__ import annotations

import importlib.util
from pathlib import Path


HERE = Path(__file__).resolve().parent
SOURCE = HERE / "mission_dashboard_20260910.py"
spec = importlib.util.spec_from_file_location("mission_dashboard_base", SOURCE)
dashboard = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(dashboard)

package = (
    dashboard.REPO / "field_testing" / "sites"
    / "62_Collins_multi_boundary_20260915" / "mission_plans"
    / "20260915_master_boundary_replay"
)
generated = package / "generated_continuation_20260918"
dashboard.MISSION = generated / "62_Collins_continuation_after_main_backyard_1mps_20260918.txt"
dashboard.AUDIT = generated / "62_Collins_continuation_after_main_backyard_1mps_audit_20260918.csv"
dashboard.LAUNCHER = package / "run_62_Collins_continuation_all_1mps_20260918.sh"
dashboard.EXPECTED_CONFIRMATION = "RUN CONTINUATION ALL 1 MPS BLADES OFF"

dashboard.HTML = dashboard.HTML.replace(
    "Tractor01 — 62 Collins clear-sky resume",
    "Tractor01 — continuation after main backyard",
).replace(
    "Resumes at source waypoint 91. Recovery stays in the current phase and may advance at most 30 m.",
    "Starts at the reviewed waypoint-91 location, then follows transition 05. Main backyard is omitted; every speed command is 1.00 m/s.",
).replace(
    "Start the reviewed clear-sky resume mission at source waypoint 91 with blades off?",
    "Start the post-main-backyard continuation with every waypoint at 1.00 m/s and blades off?",
).replace(
    "RUN PARTIAL RINGS BLADES OFF",
    dashboard.EXPECTED_CONFIRMATION,
)


if __name__ == "__main__":
    dashboard.main()
