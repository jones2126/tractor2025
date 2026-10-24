#!/usr/bin/env python3
"""Phone-safe dashboard and voice notes for AREA-OVER-ROAD field testing."""
from __future__ import annotations

import csv
import importlib.util
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "20260929_consolidated_perimeter_field_test" / "mission_dashboard_consolidated_perimeter_20260929.py"
spec = importlib.util.spec_from_file_location("phone_dashboard_adapter", SOURCE)
phone_dashboard = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(phone_dashboard)

dashboard = phone_dashboard.dashboard
dashboard.MISSION = HERE / "generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024.txt"
dashboard.AUDIT = HERE / "generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024_audit.csv"
dashboard.LAUNCHER = HERE / "run_area_over_road_field_20261024.sh"
dashboard.EXPECTED_CONFIRMATION = "RUN AREA OVER ROAD BLADES OFF"

replacements = {
    "Tractor01 — consolidated perimeter field test":
        "Tractor01 — AREA-OVER-ROAD blades-off field test",
    "Approved blades-off field test. Voice notes listen for ‘Tractor note…’ while guidance is active.":
        "Authorized AREA-OVER-ROAD test. Start launches Pure Pursuit in Pause; select Auto only after READY FOR GUARDED AUTO.",
    "Start the approved consolidated perimeter mission with blades off and direct supervision?":
        "Launch the AREA-OVER-ROAD Pure Pursuit controller for one directly supervised blades-off run? Keep the phone in Pause until READY FOR GUARDED AUTO.",
    "RUN CONSOLIDATED PERIMETER BLADES OFF": dashboard.EXPECTED_CONFIRMATION,
}
for original, updated in replacements.items():
    if dashboard.HTML.count(original) != 1:
        raise RuntimeError(f"Dashboard text changed; expected one occurrence of: {original}")
    dashboard.HTML = dashboard.HTML.replace(original, updated)

phone_dashboard.NOTE_DIR = Path("/home/al/field_logs/20261024_area_over_road_field")
phone_dashboard.NOTE_SESSION = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
phone_dashboard.NOTE_CSV = phone_dashboard.NOTE_DIR / f"voice_notes_{phone_dashboard.NOTE_SESSION}.csv"
phone_dashboard.NOTE_JSONL = phone_dashboard.NOTE_DIR / f"voice_notes_{phone_dashboard.NOTE_SESSION}.jsonl"
with dashboard.AUDIT.open(newline="", encoding="utf-8-sig") as handle:
    phone_dashboard.AUDIT_ROWS = list(csv.DictReader(handle))


if __name__ == "__main__":
    print("AREA-OVER-ROAD: authorized for one supervised blades-off run")
    print(f"Hands-free voice-note CSV:   {phone_dashboard.NOTE_CSV}")
    print(f"Hands-free voice-note JSONL: {phone_dashboard.NOTE_JSONL}")
    dashboard.main()
