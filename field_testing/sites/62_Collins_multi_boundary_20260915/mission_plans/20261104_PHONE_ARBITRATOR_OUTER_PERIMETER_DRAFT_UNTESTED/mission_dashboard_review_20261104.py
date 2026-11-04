#!/usr/bin/env python3
"""Review dashboard for the disabled phone-arbitrator field package."""

from __future__ import annotations

import importlib.util
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[4] / "tractor_rpi/pure-pursuit/mission_dashboard_outer_perimeter_field_20260928.py"
spec = importlib.util.spec_from_file_location("outer_dashboard", SOURCE)
outer = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(outer)
dashboard = outer.dashboard
dashboard.LAUNCHER = HERE / "run_outer_perimeter_20261104.sh"
dashboard.EXPECTED_CONFIRMATION = "FIELD START DISABLED"

def disabled(_state):
    return False, "Package creation is approved; field motion is not yet approved"

dashboard.safe_to_start = disabled
dashboard.HTML = dashboard.HTML.replace(
    "Tractor01 — outer-perimeter field test",
    "Tractor01 — phone-arbitrator production package review",
).replace("START MISSION", "START DISABLED")

if __name__ == "__main__":
    print("REVIEW ONLY: installation, flashing, and motion remain disabled.")
    dashboard.main()
