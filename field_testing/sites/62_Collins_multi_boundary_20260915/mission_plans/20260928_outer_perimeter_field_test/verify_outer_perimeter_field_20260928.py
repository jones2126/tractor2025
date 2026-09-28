#!/usr/bin/env python3
"""Read-only verification of the 62 Collins outer-perimeter field package."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent
GENERATED = HERE / "generated"
STEM = "62_Collins_outer_perimeter_field_test_1mps_20260928"
MISSION = GENERATED / f"{STEM}.txt"
AUDIT = GENERATED / f"{STEM}_audit.csv"
REPORT = GENERATED / f"{STEM}_report.json"
EXPECTED_MISSION_SHA256 = "b47660b4c74d5e26ca5d9448f069fd7193c068cf62e3c0b62d5c083d9aed4d64"
EXPECTED_AUDIT_SHA256 = "54072d526370447f4efd22a84052798543135fc45298e6bda80e983178f5cf25"
EXPECTED_ROWS = 3_462
EXPECTED_PHASES = {
    "base_to_backyard",
    "backyard_outer_perimeter",
    "backyard_to_front",
    "front_outer_perimeter",
    "front_to_overroad",
    "overroad_outer_perimeter",
    "overroad_to_front",
    "front_to_base",
}


def normalized_bytes(path: Path) -> bytes:
    return path.read_bytes().replace(b"\r\n", b"\n")


def verify() -> None:
    for path in (MISSION, AUDIT, REPORT):
        if not path.is_file():
            raise ValueError(f"Required field-package file missing: {path}")

    mission_bytes = normalized_bytes(MISSION)
    mission_sha = hashlib.sha256(mission_bytes).hexdigest()
    if mission_sha != EXPECTED_MISSION_SHA256:
        raise ValueError(f"Mission checksum {mission_sha} differs from the reviewed package")

    audit_bytes = normalized_bytes(AUDIT)
    audit_sha = hashlib.sha256(audit_bytes).hexdigest()
    if audit_sha != EXPECTED_AUDIT_SHA256:
        raise ValueError(f"Audit checksum {audit_sha} differs from the reviewed package")

    report = json.loads(REPORT.read_text(encoding="utf-8"))
    if report.get("status") != "SUPERVISED_BLADES_OFF_FIELD_TEST":
        raise ValueError("Package is not marked as a supervised blades-off field test")
    if report.get("mission_sha256") != mission_sha or report.get("audit_sha256") != audit_sha:
        raise ValueError("Report checksums disagree with the mission or audit")
    if report.get("waypoints") != EXPECTED_ROWS:
        raise ValueError("Report waypoint count changed")
    if report.get("route_direction") != "clockwise":
        raise ValueError("Perimeter direction changed")
    if report.get("deck_edge_rule") != "left deck edge follows outer perimeter":
        raise ValueError("Left-deck-edge perimeter rule changed")
    if float(report.get("turn_softening_radius_m", 0)) != 2.0:
        raise ValueError("Turn-softening radius changed")
    if float(report.get("maximum_waypoint_gap_m", 99)) > 0.21:
        raise ValueError("Report waypoint gap exceeds 0.21 m")

    rows = [line.split() for line in mission_bytes.decode("ascii").splitlines()]
    if len(rows) != EXPECTED_ROWS or any(len(row) != 5 for row in rows):
        raise ValueError("Mission row count or column count changed")
    if any(not all(math.isfinite(float(value)) for value in row) for row in rows):
        raise ValueError("Mission contains a non-finite value")
    if any(row[4] != "1.00" for row in rows):
        raise ValueError("Every speed command must remain 1.00 m/s")
    lookaheads = {row[3] for row in rows}
    if lookaheads != {"2.00"}:
        raise ValueError(f"Unexpected lookahead policy: {sorted(lookaheads)}")

    with AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        audit = list(csv.DictReader(handle))
    if len(audit) != EXPECTED_ROWS:
        raise ValueError("Audit row count changed")
    source_ids = [row.get("source_id", "") for row in audit]
    if len(set(source_ids)) != EXPECTED_ROWS or any(not value for value in source_ids):
        raise ValueError("Audit source identifiers are missing or duplicated")
    phases = {row.get("phase", "") for row in audit}
    if not EXPECTED_PHASES.issubset(phases):
        raise ValueError("One or more required mission phases are missing")
    for index, (mission, lineage) in enumerate(zip(rows, audit), 1):
        if lineage.get("waypoint") != str(index) or not lineage.get("phase"):
            raise ValueError("Audit waypoint numbering or phase is invalid")
        for key, column in (("lat", 0), ("lon", 1), ("yaw_rad", 2), ("lookahead_m", 3), ("speed_mps", 4)):
            if abs(float(lineage[key]) - float(mission[column])) > 1e-9:
                raise ValueError(f"Audit differs from mission at waypoint {index}: {key}")

    lat0 = float(rows[0][0])
    east_scale = 111_320.0 * math.cos(math.radians(lat0))
    max_gap = max(
        math.hypot(
            (float(after[1]) - float(before[1])) * east_scale,
            (float(after[0]) - float(before[0])) * 110_540.0,
        )
        for before, after in zip(rows, rows[1:])
    )
    if max_gap > 0.21:
        raise ValueError(f"Mission waypoint gap {max_gap:.3f} m exceeds 0.21 m")

    print("PASS: exact reviewed outer-perimeter field package verified.")
    print(f"      {EXPECTED_ROWS:,} waypoints; {report['path_length_m']:.3f} m; nominal {report['nominal_motion_time_minutes_at_1mps']:.2f} min.")
    print("      Clockwise route; left deck edge at perimeter; every speed command 1.00 m/s.")
    print(f"      Geometry softened to a 2.00 m radius; lookahead 2.00 m; maximum gap {max_gap:.3f} m.")
    print("      FIRST RUN: blades disengaged, direct supervision, handheld ready for Pause.")


if __name__ == "__main__":
    try:
        verify()
    except (ValueError, KeyError, IndexError, OSError) as exc:
        raise SystemExit(f"FIELD PACKAGE VERIFICATION FAIL: {exc}")
