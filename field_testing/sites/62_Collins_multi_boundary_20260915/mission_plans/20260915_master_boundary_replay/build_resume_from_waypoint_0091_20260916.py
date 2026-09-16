#!/usr/bin/env python3
"""Build the supervised clear-sky resume mission from source waypoint 91."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent
SOURCE_DIR = HERE / "generated_rings_only"
SOURCE_MISSION = SOURCE_DIR / "62_Collins_rings_only_master_1mps_PARTIAL_REVIEW_ONLY_20260915.txt"
SOURCE_AUDIT = SOURCE_DIR / "62_Collins_rings_only_master_1mps_audit_20260915.csv"
OUTPUT_MISSION = SOURCE_DIR / "62_Collins_rings_only_resume_wp0091_20260916.txt"
OUTPUT_AUDIT = SOURCE_DIR / "62_Collins_rings_only_resume_wp0091_audit_20260916.csv"
OUTPUT_REPORT = SOURCE_DIR / "62_Collins_rings_only_resume_wp0091_report_20260916.json"

SOURCE_START_WAYPOINT = 91
FIELD_POSITION = (40.485618833333334, -80.33235599999999)


def ground_distance_m(first, second):
    lat_scale = 110_540.0
    lon_scale = 111_320.0 * math.cos(math.radians(first[0]))
    return math.hypot(
        (second[1] - first[1]) * lon_scale,
        (second[0] - first[0]) * lat_scale,
    )


def main():
    mission_rows = [line for line in SOURCE_MISSION.read_text(encoding="utf-8").splitlines() if line.strip()]
    with SOURCE_AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        audit_rows = list(csv.DictReader(handle))
    if len(mission_rows) != len(audit_rows):
        raise ValueError("source mission and audit row counts differ")

    start_index = SOURCE_START_WAYPOINT - 1
    resume_mission = mission_rows[start_index:]
    resume_audit = []
    for new_waypoint, source in enumerate(audit_rows[start_index:], 1):
        row = dict(source)
        row["source_waypoint"] = source["waypoint"]
        row["waypoint"] = str(new_waypoint)
        resume_audit.append(row)

    OUTPUT_MISSION.write_text("\n".join(resume_mission) + "\n", encoding="utf-8", newline="\n")
    fieldnames = list(audit_rows[0].keys())
    fieldnames.insert(fieldnames.index("waypoint") + 1, "source_waypoint")
    with OUTPUT_AUDIT.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(resume_audit)

    first = resume_mission[0].split()
    start_lat, start_lon, start_yaw = map(float, first[:3])
    speeds = [line.split()[4] for line in resume_mission]
    report = {
        "status": "SUPERVISED_BLADES_OFF_RESUME",
        "source_start_waypoint": SOURCE_START_WAYPOINT,
        "trimmed_source_waypoints": start_index,
        "waypoints": len(resume_mission),
        "start_lat": start_lat,
        "start_lon": start_lon,
        "start_heading_compass_deg": (90.0 - math.degrees(start_yaw)) % 360.0,
        "field_position_lat": FIELD_POSITION[0],
        "field_position_lon": FIELD_POSITION[1],
        "field_position_distance_to_start_m": ground_distance_m(
            FIELD_POSITION, (start_lat, start_lon)),
        "speed_counts": {speed: speeds.count(speed) for speed in sorted(set(speeds))},
        "mission_sha256": hashlib.sha256(OUTPUT_MISSION.read_bytes()).hexdigest(),
        "safety_guards": {
            "reacquire_max_advance_m": 30.0,
            "phase_locked_recovery": True,
            "resume_stable_seconds": 5.0,
            "required_heading_carrier": "fixed",
            "baseline_range_m": [0.80, 1.30],
            "maximum_heading_accuracy_deg": 1.0,
        },
    }
    OUTPUT_REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
