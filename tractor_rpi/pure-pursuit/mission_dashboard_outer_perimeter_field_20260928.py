#!/usr/bin/env python3
"""Dashboard and voice guidance for the 62 Collins outer-perimeter field test."""

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
    / "20260928_outer_perimeter_field_test"
)
generated = package / "generated"
dashboard.MISSION = generated / "62_Collins_outer_perimeter_field_test_1mps_20260928.txt"
dashboard.AUDIT = generated / "62_Collins_outer_perimeter_field_test_1mps_20260928_audit.csv"
dashboard.LAUNCHER = package / "run_62_Collins_outer_perimeter_field_20260928.sh"
dashboard.EXPECTED_CONFIRMATION = "RUN OUTER PERIMETER BLADES OFF"

replacements = {
    "Tractor01 — 62 Collins clear-sky resume": "Tractor01 — outer-perimeter field test",
    "Resumes at source waypoint 91. Recovery stays in the current phase and may advance at most 30 m.":
        "Starts at the recorded base point. Blades off. Follow voice guidance to the start; automatic recovery may advance at most 5 m.",
    "Start the reviewed clear-sky resume mission at source waypoint 91 with blades off?":
        "Start the clockwise outer-perimeter field test from the recorded base point with blades off?",
    "RUN PARTIAL RINGS BLADES OFF": dashboard.EXPECTED_CONFIRMATION,
    "Safety stop. RTK position lost. Select handheld Pause. Waiting for RTK Fixed.":
        "Safety stop. RTK position lost. Waiting for RTK Fixed. Automatic recovery is enabled.",
    "Safety stop. Heading solution lost. Select handheld Pause. Waiting for fixed heading.":
        "Safety stop. Heading solution lost. Waiting for valid heading. Automatic recovery is enabled.",
}
for original, updated in replacements.items():
    if dashboard.HTML.count(original) != 1:
        raise RuntimeError(f"Dashboard text changed; expected one occurrence of: {original}")
    dashboard.HTML = dashboard.HTML.replace(original, updated)


if __name__ == "__main__":
    dashboard.main()
