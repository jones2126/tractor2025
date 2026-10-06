#!/usr/bin/env python3
"""Phone-safe dashboard adapter for rerunning the September 23 master mission."""

from __future__ import annotations

import csv
import importlib.util
from datetime import datetime, timezone
from pathlib import Path


HERE = Path(__file__).resolve().parent
MASTER_PACKAGE = HERE.parent / "20260915_master_boundary_replay"
PHONE_PACKAGE = HERE.parent / "20260929_consolidated_perimeter_field_test"
SOURCE = PHONE_PACKAGE / "mission_dashboard_consolidated_perimeter_20260929.py"

spec = importlib.util.spec_from_file_location("phone_dashboard_adapter", SOURCE)
phone_dashboard = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(phone_dashboard)

dashboard = phone_dashboard.dashboard
dashboard.MISSION = (
    MASTER_PACKAGE
    / "generated_master_manual_field_20260920"
    / "62_Collins_master_manual_resampled_1mps_20260920.txt"
)
dashboard.AUDIT = (
    MASTER_PACKAGE
    / "generated_master_manual_field_20260920"
    / "62_Collins_master_manual_resampled_1mps_20260920_audit.csv"
)
dashboard.LAUNCHER = HERE / "run_62_Collins_master_manual_phone_20261006.sh"
dashboard.EXPECTED_CONFIRMATION = "RUN SEPTEMBER 23 MASTER PHONE BLADES OFF"

replacements = {
    "Tractor01 — consolidated perimeter field test":
        "Tractor01 — September 23 master mission rerun",
    "Approved blades-off field test. Voice notes listen for ‘Tractor note…’ while guidance is active.":
        "Exact September 23 route, now using the Wi-Fi phone handheld. Blades off and direct supervision required.",
    "Start the approved consolidated perimeter mission with blades off and direct supervision?":
        "Rerun the exact September 23 master mission with phone control, blades off, and direct supervision?",
    "RUN CONSOLIDATED PERIMETER BLADES OFF": dashboard.EXPECTED_CONFIRMATION,
}
for original, updated in replacements.items():
    if dashboard.HTML.count(original) != 1:
        raise RuntimeError(f"Dashboard text changed; expected one occurrence of: {original}")
    dashboard.HTML = dashboard.HTML.replace(original, updated)

phone_dashboard.NOTE_DIR = Path(
    "/home/al/field_logs/20261006_master_manual_phone_rerun_1mps"
)
phone_dashboard.NOTE_SESSION = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
phone_dashboard.NOTE_CSV = phone_dashboard.NOTE_DIR / f"voice_notes_{phone_dashboard.NOTE_SESSION}.csv"
phone_dashboard.NOTE_JSONL = phone_dashboard.NOTE_DIR / f"voice_notes_{phone_dashboard.NOTE_SESSION}.jsonl"
with dashboard.AUDIT.open(newline="", encoding="utf-8-sig") as handle:
    phone_dashboard.AUDIT_ROWS = list(csv.DictReader(handle))


if __name__ == "__main__":
    print(f"Hands-free voice-note CSV:   {phone_dashboard.NOTE_CSV}")
    print(f"Hands-free voice-note JSONL: {phone_dashboard.NOTE_JSONL}")
    dashboard.main()
