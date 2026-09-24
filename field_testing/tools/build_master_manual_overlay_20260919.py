#!/usr/bin/env python3
"""Build an offline review viewer with the recorded manual path over the master target."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

from build_master_target_mission_viewer_20260919 import (
    RUN, SITE, TEMPLATE, digest, load,
)


FIELD_LOG = RUN / "partial_rings_master_20260918_121342.csv"
OUTPUT = SITE / "analysis" / "master_with_manual_overlay_20260919.html"
MANUAL_START = "2026-09-18T16:56:07"
MANUAL_END = "2026-09-18T17:04:02"
APPROVED_START = "2026-09-18T16:56:19.024+00:00"
APPROVED_END = "2026-09-18T17:01:30.938+00:00"
JOIN_WAYPOINT = 12845


def period(stamp: str) -> int:
    if stamp < APPROVED_START:
        return 0  # Manual approach before the approved stripe window.
    if stamp <= APPROVED_END:
        return 1  # Approved trace.
    return 2  # Later manual driving, shown for context only.


def build():
    data = load()
    collection = json.loads((RUN / "collection_summary.json").read_text(encoding="utf-8-sig"))
    if digest(FIELD_LOG) != collection["field_sha256"].upper():
        raise ValueError("Archived field log no longer matches collection checksum")
    mission_start_lat = data["points"][0][2]
    mission_start_lon = data["points"][0][3]
    x_scale = 111_320 * math.cos(math.radians(mission_start_lat))
    y_scale = 111_132
    manual = []
    previous_logged = None
    with FIELD_LOG.open(newline="", encoding="utf-8-sig") as handle:
        for field_row, row in enumerate(csv.DictReader(handle), 1):
            stamp = row.get("time", "")
            if not MANUAL_START <= stamp < MANUAL_END or row.get("steer_mode") != "1":
                continue
            lat, lon = float(row["lat"]), float(row["lon"])
            if not (math.isfinite(lat) and math.isfinite(lon)):
                continue
            group = period(stamp)
            repeated = previous_logged is not None and (lat, lon) == previous_logged[:2]
            if repeated and group == previous_logged[2] and stamp != APPROVED_END:
                continue
            previous_logged = (lat, lon, group)
            manual.append([
                round((lon - mission_start_lon) * x_scale, 3),
                round((lat - mission_start_lat) * y_scale, 3),
                lat, lon, stamp, row["fix_quality"],
                round(float(row["heading_deg"]), 2), field_row, group,
            ])
    if not manual or manual[0][4] != "2026-09-18T16:56:07.487+00:00":
        raise ValueError("Manual start not found in archived logger")
    if sum(p[8] == 1 for p in manual) < 1000:
        raise ValueError("Approved stripe interval is unexpectedly short")
    if not any(p[4] == APPROVED_START for p in manual):
        raise ValueError("Approved stripe start is absent")
    if not any(p[4] == APPROVED_END for p in manual):
        raise ValueError("Approved stripe endpoint is absent")
    approved_end = next(p for p in manual if p[4] == APPROVED_END)
    if abs(approved_end[2] - data["endpoint"]["lat"]) > 1e-10 or abs(approved_end[3] - data["endpoint"]["lon"]) > 1e-10:
        raise ValueError("Manual overlay and approved stripe endpoint disagree")
    if data["points"][JOIN_WAYPOINT - 1][4] != data["phases"].index("transition_05"):
        raise ValueError("W12845 is not in the expected transition")
    data["title"] = "Master mission + recorded manual path"
    data["manual"] = manual
    data["joinWaypoint"] = JOIN_WAYPOINT
    data["source"]["manualLog"] = FIELD_LOG.name
    data["source"]["manualLogSha256"] = digest(FIELD_LOG)

    encoded = json.dumps(data, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    template = TEMPLATE.read_text(encoding="utf-8")
    if template.count("__DATA__") != 1:
        raise ValueError("Viewer template has no unique data insertion point")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(template.replace("__DATA__", encoded), encoding="utf-8", newline="\n")
    print(f"{OUTPUT} | {len(data['points']):,} planned | {len(manual):,} unique-position manual points | {OUTPUT.stat().st_size:,} bytes")
    print("Manual first:", manual[0][4], "last:", manual[-1][4])


if __name__ == "__main__":
    build()
