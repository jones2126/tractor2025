#!/usr/bin/env python3
"""Fail-closed verifier for the authorized AREA-OVER-ROAD blades-off package."""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT = HERE / "62_collins_area_over_road_mission_project.json"
VALIDATION = HERE / "checkpoint_6_controller_validation_rev003.json"
MISSION = HERE / "generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024.txt"
AUDIT = HERE / "generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024_audit.csv"
LAUNCHER = HERE / "run_area_over_road_field_20261024.sh"

EXPECTED_MISSION_SHA256 = "150e70a2be00eed9cf0e5aedc0bc26ba3c8a122a2b37a7c666aab7e3a7859979"
EXPECTED_AUDIT_SHA256 = "d3c41cb43080470b1366a4a1af580a1d22c01038196778b4475b41604849bbae"
EXPECTED_VALIDATION_SHA256 = "0ffc97449bdcb86476b5ab8dd272b0d464175744703b7f951b18d33215c0038e"
EXPECTED_ROWS = 3463


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sha256_lf_text(path: Path) -> str:
    """Hash text canonically so Windows CRLF and Tractor01 LF checkouts agree."""
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def main() -> None:
    for path in (PROJECT, VALIDATION, MISSION, AUDIT, LAUNCHER):
        if not path.is_file():
            raise SystemExit(f"ERROR: required file missing: {path}")
    if sha256(MISSION) != EXPECTED_MISSION_SHA256:
        raise SystemExit("ERROR: mission checksum differs from the approved controller route")
    if sha256_lf_text(AUDIT) != EXPECTED_AUDIT_SHA256:
        raise SystemExit("ERROR: waypoint-audit checksum differs from the approved package")
    if sha256_lf_text(VALIDATION) != EXPECTED_VALIDATION_SHA256:
        raise SystemExit("ERROR: controller validation checksum differs from the reviewed package")

    project = json.loads(PROJECT.read_text(encoding="utf-8"))
    validation = json.loads(VALIDATION.read_text(encoding="utf-8"))
    launcher = LAUNCHER.read_text(encoding="utf-8")
    if project["status_text"] != "DRAFT - UNTESTED IN THE FIELD":
        raise SystemExit("ERROR: project status changed unexpectedly")
    field = project["checkpoint_7_current"]
    if field.get("approved_for_field") is not True:
        raise SystemExit("ERROR: project does not authorize the supervised blades-off run")
    if field.get("blade_state") != "off":
        raise SystemExit("ERROR: first field run must remain blades off")
    if "APPROVED_FOR_FIELD=true" not in launcher:
        raise SystemExit("ERROR: field launcher approval flag is not enabled")

    rows = [line.split() for line in MISSION.read_text(encoding="utf-8").splitlines() if line.strip()]
    with AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        audit = list(csv.DictReader(handle))
    if len(rows) != EXPECTED_ROWS or len(audit) != EXPECTED_ROWS:
        raise SystemExit(f"ERROR: expected {EXPECTED_ROWS} mission/audit rows")
    if any(len(row) != 5 for row in rows):
        raise SystemExit("ERROR: mission contains a row that is not five columns")
    values = [[float(value) for value in row] for row in rows]
    if any(not all(math.isfinite(value) for value in row) for row in values):
        raise SystemExit("ERROR: mission contains a non-finite value")
    if {row[4] for row in rows} != {"1.00"}:
        raise SystemExit("ERROR: every mission speed must be 1.00 m/s")
    if {row[3] for row in rows} != {"1.50", "2.00"}:
        raise SystemExit("ERROR: mission lookaheads must be exactly 1.50 m and 2.00 m")
    if any(row["geometry_status"] != "CONTROLLER_FORMAT_DRAFT_UNTESTED" for row in audit):
        raise SystemExit("ERROR: waypoint audit geometry status changed")
    if validation["field_authorized"] is not False:
        raise SystemExit("ERROR: immutable geometry validation must remain pre-field evidence")
    if validation["joins"]["maximum_gap_m"] > 1e-6:
        raise SystemExit("ERROR: route contains a segment gap")
    if max(item["heading_change_deg"] for item in validation["joins"]["heading_changes"]) >= 5.0:
        raise SystemExit("ERROR: route join heading change exceeds reviewed tolerance")
    if min(item["sampled_minimum_radius_m"] for item in validation["turns"]) < 1.399:
        raise SystemExit("ERROR: constructed turn radius is below reviewed 1.4 m tolerance")
    clearance = validation["containment_and_clearance"]
    if clearance["physical_deck_intersection_with_pole_m2"] != 0.0:
        raise SystemExit("ERROR: physical deck intersects the modeled pole exclusion")
    if clearance["conservative_deck_plus_chute_intersection_with_pole_m2"] != 0.0:
        raise SystemExit("ERROR: conservative deck/chute envelope intersects the pole exclusion")

    print("PASS: exact AREA-OVER-ROAD supervised blades-off package verified")
    print(f"PASS: {EXPECTED_ROWS} waypoints; speed 1.00 m/s; lookaheads 1.50/2.00 m")
    print("PASS: continuous 1.4 m planning geometry and modeled pole clearance verified")
    print("AUTHORIZED: one directly supervised blades-off field run")


if __name__ == "__main__":
    main()
