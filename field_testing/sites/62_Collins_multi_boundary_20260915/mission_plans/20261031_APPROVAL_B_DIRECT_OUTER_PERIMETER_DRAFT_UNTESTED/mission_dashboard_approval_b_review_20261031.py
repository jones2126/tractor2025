#!/usr/bin/env python3
"""Review-only Approval B route dashboard; mission Start is disabled."""

from __future__ import annotations

import importlib.util
from pathlib import Path


HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[4] / "tractor_rpi/pure-pursuit/mission_dashboard_outer_perimeter_field_20260928.py"
spec = importlib.util.spec_from_file_location("outer_perimeter_dashboard", SOURCE)
outer = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(outer)
dashboard = outer.dashboard

dashboard.LAUNCHER = HERE / "run_approval_b_outer_perimeter_20261031.sh"
dashboard.EXPECTED_CONFIRMATION = "FIELD START DISABLED"


def review_only_safe_to_start(_state):
    return False, "Approval B field authorization is pending; Pure Pursuit start is disabled"


dashboard.safe_to_start = review_only_safe_to_start
dashboard.HTML = dashboard.HTML.replace(
    "Tractor01 — outer-perimeter field test",
    "Tractor01 — Approval B outer-perimeter review only",
).replace(
    "START MISSION",
    "START DISABLED",
).replace(
    "AUTO NOT READY — START DISABLED",
    "FIELD START DISABLED — REVIEW ONLY",
)

if __name__ == "__main__":
    print("DRAFT - UNTESTED IN THE FIELD")
    print("Review-only dashboard: logger and Pure Pursuit start are disabled.")
    dashboard.main()
