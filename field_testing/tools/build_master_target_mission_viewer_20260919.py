#!/usr/bin/env python3
"""Build an offline, target-only viewer for the archived 2026-09-18 master mission."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path


SITE = Path(__file__).resolve().parents[1] / "sites" / "62_Collins_multi_boundary_20260915"
RUN = SITE / "runs" / "20260918_121342"
PACKAGE = RUN / "20260915_master_boundary_replay" / "generated_rings_only"
MISSION = PACKAGE / "62_Collins_rings_only_resume_wp0091_20260916.txt"
AUDIT = PACKAGE / "62_Collins_rings_only_resume_wp0091_audit_20260916.csv"
SUMMARY = RUN / "collection_summary.json"
STRIPES = SITE / "analysis" / "manual_center_stripes_20260918_recorded_source.csv"
TEMPLATE = Path(__file__).with_name("master-target-mission-viewer-template.html")
OUTPUT = SITE / "analysis" / "master_target_mission_20260918.html"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def load():
    for path in (MISSION, AUDIT, SUMMARY, STRIPES, TEMPLATE):
        if not path.is_file():
            raise FileNotFoundError(path)
    summary = json.loads(SUMMARY.read_text(encoding="utf-8-sig"))
    if summary.get("mission_package_verified_against_tractor") is not True:
        raise ValueError("Archive does not record tractor mission-package verification")

    with AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        audit = list(csv.DictReader(handle))
    mission = [line.split() for line in MISSION.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(mission) != len(audit) or len(mission) != 19_250:
        raise ValueError("Unexpected master mission/audit waypoint count")
    if any(len(values) != 5 for values in mission):
        raise ValueError("Mission does not have five columns per waypoint")

    with STRIPES.open(newline="", encoding="utf-8-sig") as handle:
        stripes = list(csv.DictReader(handle))
    if not stripes or not stripes[-1]["time"].startswith("2026-09-18T17:01:30"):
        raise ValueError("Approved stripe endpoint source is missing or unexpected")
    endpoint = stripes[-1]
    lat0 = float(mission[0][0])
    lon0 = float(mission[0][1])
    x_scale = 111_320 * math.cos(math.radians(lat0))
    y_scale = 111_132

    phase_names = list(dict.fromkeys(row["phase"] for row in audit))
    phase_ids = {name: index for index, name in enumerate(phase_names)}
    points = []
    for index, (values, row) in enumerate(zip(mission, audit), 1):
        lat, lon, yaw, lookahead, speed = map(float, values)
        if row["waypoint"] != str(index):
            raise ValueError(f"Audit waypoint mismatch at {index}")
        if abs(lat - float(row["lat"])) > 1e-8 or abs(lon - float(row["lon"])) > 1e-8:
            raise ValueError(f"Mission/audit coordinate mismatch at {index}")
        points.append([
            round((lon - lon0) * x_scale, 3),
            round((lat - lat0) * y_scale, 3),
            lat, lon, phase_ids[row["phase"]], int(row["source_waypoint"]),
            round((90 - math.degrees(yaw)) % 360, 2), speed,
        ])
    endpoint_data = {
        "x": round((float(endpoint["lon"]) - lon0) * x_scale, 3),
        "y": round((float(endpoint["lat"]) - lat0) * y_scale, 3),
        "lat": float(endpoint["lat"]),
        "lon": float(endpoint["lon"]),
        "heading": round(float(endpoint["heading_deg"]), 2),
        "time": endpoint["time"],
    }
    return {
        "title": "Master mission · target waypoints only",
        "run": "20260918_121342",
        "points": points,
        "phases": phase_names,
        "endpoint": endpoint_data,
        "source": {
            "mission": MISSION.name,
            "missionSha256": digest(MISSION),
            "audit": AUDIT.name,
            "auditSha256": digest(AUDIT),
            "stripeEndpoint": STRIPES.name,
            "stripeSourceSha256": digest(STRIPES),
        },
    }


def main():
    payload = load()
    encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    template = TEMPLATE.read_text(encoding="utf-8")
    if template.count("__DATA__") != 1:
        raise ValueError("Viewer template has no unique data insertion point")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(template.replace("__DATA__", encoded), encoding="utf-8", newline="\n")
    print(f"{OUTPUT} | {len(payload['points']):,} planned waypoints | {OUTPUT.stat().st_size:,} bytes")
    print("Archived mission SHA-256:", payload["source"]["missionSha256"])


if __name__ == "__main__":
    main()
