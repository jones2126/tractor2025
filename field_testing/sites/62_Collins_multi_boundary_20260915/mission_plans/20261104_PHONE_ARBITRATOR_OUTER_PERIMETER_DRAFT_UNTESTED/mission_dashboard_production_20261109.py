#!/usr/bin/env python3
"""Enabled production dashboard for the approved blades-off perimeter run."""

from __future__ import annotations

import importlib.util
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
SOURCE = REPO / "tractor_rpi/pure-pursuit/mission_dashboard_outer_perimeter_field_20260928.py"
spec = importlib.util.spec_from_file_location("outer_dashboard", SOURCE)
outer = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(outer)
dashboard = outer.dashboard
dashboard.REPO = REPO
dashboard.LAUNCHER = HERE / "run_outer_perimeter_20261104.sh"
dashboard.EXPECTED_CONFIRMATION = "RUN PHONE OUTER PERIMETER BLADES OFF"

legacy_safe_to_start = dashboard.safe_to_start


def production_safe_to_start(state):
    ok, reason = legacy_safe_to_start(state)
    if not ok:
        return ok, reason
    snap = state.snapshot()
    arbitrator = snap["bridge"].get("rpi_arbitrator", {})
    if arbitrator.get("requested_mode") != "PAUSE":
        return False, "Phone request is not Pause"
    if arbitrator.get("actual_serial_output") != "PAUSE":
        return False, "RPi safety controller actual output is not Pause"
    if arbitrator.get("phone_link_fresh") is not True:
        return False, "Direct phone control link is not fresh"
    if arbitrator.get("scheduler_fault_latched") is True:
        return False, "RPi-to-Teensy command scheduler has a latched fault"
    if arbitrator.get("additional_local_faults"):
        return False, "Local actuator health fault remains active"
    if arbitrator.get("communications_degraded_preauthorized") is not True:
        return False, "Bounded communications-degraded policy is not active"
    if arbitrator.get("distance_envelope_source") != "ROUTE_PROGRESS_ONLY":
        return False, "Degraded distance source is not route progress"
    return True, ""


dashboard.safe_to_start = production_safe_to_start
dashboard.HTML = dashboard.HTML.replace(
    "Tractor01 — outer-perimeter field test",
    "Tractor01 — production phone-controlled outer perimeter",
).replace(
    "RUN OUTER PERIMETER BLADES OFF",
    dashboard.EXPECTED_CONFIRMATION,
)


if __name__ == "__main__":
    print("PRODUCTION FIELD PACKAGE: exact reviewed route; blades must remain off.")
    dashboard.main()
