#!/usr/bin/env python3
"""Dashboard for the reviewed, resampled 62 Collins blades-off field test."""

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
generated = package / "generated_master_manual_field_20260920"
dashboard.MISSION = generated / "62_Collins_master_manual_resampled_1mps_20260920.txt"
dashboard.AUDIT = generated / "62_Collins_master_manual_resampled_1mps_20260920_audit.csv"
dashboard.LAUNCHER = package / "run_62_Collins_master_manual_field_20260920.sh"
dashboard.EXPECTED_CONFIRMATION = "RUN MASTER MANUAL FIELD BLADES OFF"

replacements = {
    "Tractor01 — 62 Collins clear-sky resume": "Tractor01 — reviewed master + manual field test",
    "Resumes at source waypoint 91. Recovery stays in the current phase and may advance at most 30 m.":
        "Starts at original W1. Blades off. Recovery requires a handheld mode cycle and may advance at most 5 m.",
    "Start the reviewed clear-sky resume mission at source waypoint 91 with blades off?":
        "Start the reviewed master/manual field test at original W1 with blades off?",
    "RUN PARTIAL RINGS BLADES OFF": dashboard.EXPECTED_CONFIRMATION,
}
for original, updated in replacements.items():
    if dashboard.HTML.count(original) != 1:
        raise RuntimeError(f"Dashboard text changed; expected one occurrence of: {original}")
    dashboard.HTML = dashboard.HTML.replace(original, updated)


if __name__ == "__main__":
    dashboard.main()
