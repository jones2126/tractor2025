#!/usr/bin/env python3
"""Voice-guidance dashboard adapter for the review-gated consolidated perimeter."""
from __future__ import annotations
import importlib.util
from pathlib import Path
HERE=Path(__file__).resolve().parent
SOURCE=HERE.parents[4]/"tractor_rpi/pure-pursuit/mission_dashboard_20260910.py"
spec=importlib.util.spec_from_file_location("mission_dashboard_base",SOURCE)
dashboard=importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(dashboard)
dashboard.MISSION=HERE/"generated/62_Collins_consolidated_perimeter_1mps_20260929_REVIEW_ONLY.txt"
dashboard.AUDIT=HERE/"generated/62_Collins_consolidated_perimeter_1mps_20260929_REVIEW_ONLY_audit.csv"
dashboard.LAUNCHER=HERE/"run_62_Collins_consolidated_perimeter_20260929.sh"
dashboard.EXPECTED_CONFIRMATION="RUN CONSOLIDATED PERIMETER BLADES OFF"
replacements={
"Tractor01 — 62 Collins clear-sky resume":"Tractor01 — consolidated perimeter REVIEW ONLY",
"Resumes at source waypoint 91. Recovery stays in the current phase and may advance at most 30 m.":"Review-gated candidate. Blades off. Follow voice guidance to start; recovery may advance at most 5 m.",
"Start the reviewed clear-sky resume mission at source waypoint 91 with blades off?":"Request the consolidated perimeter blades-off supervised mission? The launcher remains blocked until owner approval.",
"RUN PARTIAL RINGS BLADES OFF":dashboard.EXPECTED_CONFIRMATION,
}
for original,updated in replacements.items():
    if dashboard.HTML.count(original)!=1: raise RuntimeError(f"Dashboard text changed; expected one occurrence of: {original}")
    dashboard.HTML=dashboard.HTML.replace(original,updated)
if __name__=="__main__": dashboard.main()
