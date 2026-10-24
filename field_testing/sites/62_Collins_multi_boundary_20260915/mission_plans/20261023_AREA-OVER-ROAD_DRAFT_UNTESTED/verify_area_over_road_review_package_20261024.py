#!/usr/bin/env python3
"""Fail-closed verifier for the AREA-OVER-ROAD controller-route review package."""
from __future__ import annotations

import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT = HERE / "62_collins_area_over_road_mission_project.json"
VALIDATION = HERE / "checkpoint_6_controller_validation_rev003.json"
MISSION = HERE / "generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024.txt"
AUDIT = HERE / "generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024_audit.csv"
LAUNCHER = HERE / "run_area_over_road_review_only_20261024.sh"
DASHBOARD_STARTER = HERE / "start_area_over_road_dashboard_20261024.sh"


def main() -> None:
    project = json.loads(PROJECT.read_text(encoding="utf-8"))
    validation = json.loads(VALIDATION.read_text(encoding="utf-8"))
    launcher = LAUNCHER.read_text(encoding="utf-8")
    assert project["status_text"] == "DRAFT - UNTESTED IN THE FIELD"
    assert validation["controller_file_created"] is True
    assert validation["field_authorized"] is False
    assert validation["route"]["forward_only"] is True
    assert validation["joins"]["maximum_gap_m"] <= 1e-6
    assert max(item["heading_change_deg"] for item in validation["joins"]["heading_changes"]) < 5.0
    assert min(item["sampled_minimum_radius_m"] for item in validation["turns"]) >= 1.399
    assert validation["containment_and_clearance"]["physical_deck_intersection_with_pole_m2"] == 0.0
    assert validation["containment_and_clearance"]["conservative_deck_plus_chute_intersection_with_pole_m2"] == 0.0
    assert "APPROVED_FOR_FIELD=false" in launcher
    assert "LIVE START BLOCKED" in launcher
    assert DASHBOARD_STARTER.is_file()
    mission_rows = [line.split() for line in MISSION.read_text(encoding="utf-8").splitlines() if line.strip()]
    with AUDIT.open(newline="", encoding="utf-8") as handle:
        audit_rows = list(csv.DictReader(handle))
    assert mission_rows and len(mission_rows) == len(audit_rows)
    assert all(len(row) == 5 for row in mission_rows)
    assert all(row["geometry_status"] == "CONTROLLER_FORMAT_DRAFT_UNTESTED" for row in audit_rows)
    assert all(float(row["speed_mps"]) == 1.0 for row in audit_rows)
    assert all(float(row["lookahead_m"]) in (1.5, 2.0) for row in audit_rows)
    print(f"PASS: controller file contains {len(mission_rows)} valid five-column waypoints")
    print("PASS: route is continuous, forward-only, and uses validated 1.4 m planning turns")
    print("PASS: modeled deck and conservative chute envelope do not enter the pole exclusion")
    print("PASS: live start is disabled")
    print("REVIEW REQUIRED: controller route revision 003 is not field authorized")


if __name__ == "__main__":
    main()
