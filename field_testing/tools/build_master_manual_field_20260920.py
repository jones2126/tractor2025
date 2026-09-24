#!/usr/bin/env python3
"""Build the reviewed 62 Collins route with the manual trace resampled for field use.

This development-side builder reads the v2 review artifact; only generated output
files are needed on tractor01. It never modifies the archived 19,250-point mission.
"""

from __future__ import annotations

import bisect
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

from shapely.geometry import LineString

from build_master_target_mission_viewer_20260919 import SITE


REVIEW = SITE / "analysis" / "master_manual_revision_v2_REVIEW_ONLY_20260920"
PACKAGE = SITE / "mission_plans" / "20260915_master_boundary_replay"
OUT = PACKAGE / "generated_master_manual_field_20260920"
STEM = "62_Collins_master_manual_resampled_1mps_20260920"
SOURCE_MISSION = REVIEW / "62_Collins_master_manual_revision_DRAFT_1mps_20260920.txt"
SOURCE_AUDIT = REVIEW / "62_Collins_master_manual_revision_audit_20260920.csv"
SOURCE_REPORT = REVIEW / "62_Collins_master_manual_revision_report_20260920.json"
TARGET_SPACING_M = 0.15
MAX_TRACE_DEVIATION_M = 0.05
EXPECTED_REVIEW_ROWS = 21_792
EXPECTED_FIELD_ROWS = 19_825


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    return ((lon - lon0) * 111_320 * math.cos(math.radians(lat0)),
            (lat - lat0) * 111_132)


def ll(x: float, y: float, lat0: float, lon0: float) -> tuple[float, float]:
    return lat0 + y / 111_132, lon0 + x / (111_320 * math.cos(math.radians(lat0)))


def angle_between(a: float, b: float, t: float) -> float:
    return (a + (((b-a+180) % 360)-180)*t) % 360


def main() -> None:
    source_report = json.loads(SOURCE_REPORT.read_text(encoding="utf-8"))
    with SOURCE_AUDIT.open(newline="", encoding="utf-8") as handle:
        source = list(csv.DictReader(handle))
    mission_lines = SOURCE_MISSION.read_text(encoding="utf-8").splitlines()
    if len(source) != len(mission_lines) or len(source) != EXPECTED_REVIEW_ROWS:
        raise ValueError("Unexpected v2 review row count")
    if source_report["revised_waypoints"] != EXPECTED_REVIEW_ROWS:
        raise ValueError("Unexpected v2 review report")
    if source[0]["source_id"] != "W1" or source[-1]["source_id"] != "W19250":
        raise ValueError("Unexpected master mission endpoints")
    source_ids = [row["source_id"] for row in source]
    source_lines = dict(zip(source_ids, mission_lines))
    if len(source_lines) != len(source_ids):
        raise ValueError("Duplicate review source ID")
    first, last = source_ids.index("M18"), source_ids.index("M2860")
    raw = source[first:last+1]
    if len(raw) != 2843 or any(row["source_id"] != f"M{i+18}" for i, row in enumerate(raw)):
        raise ValueError("Manual sample lineage changed")
    if any(not math.isfinite(float(value)) for row in source for value in
           (row["lat"], row["lon"], row["heading_deg"], row["speed_mps"])):
        raise ValueError("Non-finite source route value")
    lat0, lon0 = float(source[0]["lat"]), float(source[0]["lon"])
    points = [xy(float(row["lat"]), float(row["lon"]), lat0, lon0) for row in raw]
    cumulative = [0.0]
    for a,b in zip(points,points[1:]):
        step = math.dist(a,b)
        if step < 0.0001 or step > 0.20:
            raise ValueError(f"Manual point gap {step:.3f} m is outside expected range")
        cumulative.append(cumulative[-1]+step)
    length = cumulative[-1]
    intervals = math.ceil(length/TARGET_SPACING_M)
    if intervals != 875:
        raise ValueError(f"Manual resample count changed: {intervals} intervals")
    output_manual = [dict(raw[0])]
    output_manual[0]["phase"] = "recorded_manual_resampled"
    output_manual[0]["source_interval"] = "M18 exact"
    for i in range(1, intervals):
        distance = i*length/intervals
        segment = bisect.bisect_right(cumulative,distance)-1
        fraction = (distance-cumulative[segment])/(cumulative[segment+1]-cumulative[segment])
        ax,ay = points[segment]
        bx,by = points[segment+1]
        lat,lon = ll(ax+fraction*(bx-ax),ay+fraction*(by-ay),lat0,lon0)
        a,b = raw[segment],raw[segment+1]
        heading = angle_between(float(a["heading_deg"]),float(b["heading_deg"]),fraction)
        output_manual.append(dict(a,source_id=f"S{i}",phase="recorded_manual_resampled",
                                  lat=f"{lat:.9f}",lon=f"{lon:.9f}",
                                  heading_deg=f"{heading:.3f}",gps_time="",gps_fix="",
                                  field_log_row="",
                                  source_interval=f'{a["source_id"]} to {b["source_id"]} at {fraction:.4f}'))
    output_manual.append(dict(raw[-1],phase="recorded_manual_resampled",
                              source_interval="M2860 exact"))
    if len(output_manual) != 876:
        raise AssertionError("Unexpected resampled manual count")
    revised = source[:first]+output_manual+source[last+1:]
    if len(revised) != EXPECTED_FIELD_ROWS:
        raise AssertionError(f"Unexpected field route count: {len(revised)}")
    line_original = LineString(points)
    line_resampled = LineString([xy(float(row["lat"]),float(row["lon"]),lat0,lon0)
                                for row in output_manual])
    deviation = line_original.hausdorff_distance(line_resampled)
    if deviation > MAX_TRACE_DEVIATION_M:
        raise ValueError(f"Manual trace deviation {deviation:.3f} m exceeds limit")

    ids = [row["source_id"] for row in revised]
    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate source ID")
    for pair in (("W12652","M18"),("M2860","W12845"),("W13642","W13704"),
                 ("W13907","W13941"),("W14517","W14556"),
                 ("W15749","W15807"),("W15867","W15903")):
        if not all(not item.startswith("W") for item in ids[ids.index(pair[0])+1:ids.index(pair[1])]):
            raise ValueError(f"Original W point remains between {pair}")
    all_xy = [xy(float(row["lat"]),float(row["lon"]),lat0,lon0) for row in revised]
    gaps = [math.dist(a,b) for a,b in zip(all_xy,all_xy[1:])]
    if max(gaps) > 0.55:
        raise ValueError(f"Waypoint gap {max(gaps):.3f} m exceeds limit")
    if {float(row["speed_mps"]) for row in revised} != {1.0}:
        raise ValueError("Field route must be uniformly 1.0 m/s")

    OUT.mkdir(parents=True,exist_ok=True)
    mission = OUT / f"{STEM}.txt"
    audit = OUT / f"{STEM}_audit.csv"
    report = OUT / f"{STEM}_report.json"
    output_lines = []
    for row in revised:
        if row["source_id"].startswith("S") and row["source_id"][1:].isdigit():
            yaw = math.radians(90-float(row["heading_deg"]))
            output_lines.append(f'{float(row["lat"]):.9f} {float(row["lon"]):.9f} {yaw:.6f} 2.00 1.00')
        else:
            output_lines.append(source_lines[row["source_id"]])
    mission.write_text("\n".join(output_lines)+"\n",encoding="utf-8",newline="\n")
    fields = ["waypoint","source_id","phase","lat","lon","yaw_rad","lookahead_m","speed_mps",
              "gps_time","gps_fix","field_log_row","source_interval"]
    with audit.open("w",encoding="utf-8",newline="") as handle:
        writer = csv.DictWriter(handle,fieldnames=fields,lineterminator="\n")
        writer.writeheader()
        for i,(row,mission_line) in enumerate(zip(revised,output_lines),1):
            lat,lon,yaw,lookahead,speed = mission_line.split()
            writer.writerow(dict(waypoint=i,source_id=row["source_id"],phase=row["phase"],
                                 lat=lat,lon=lon,yaw_rad=yaw,
                                 lookahead_m=lookahead,speed_mps=speed,gps_time=row["gps_time"],
                                 gps_fix=row["gps_fix"],field_log_row=row["field_log_row"],
                                 source_interval=row.get("source_interval","")))
    summary = {
        "status":"SUPERVISED_BLADES_OFF_FIELD_TEST",
        "waypoints":len(revised),"mission_sha256":digest(mission),"audit_sha256":digest(audit),
        "review_mission_sha256":digest(SOURCE_MISSION),"review_audit_sha256":digest(SOURCE_AUDIT),
        "manual_original_points":len(raw),"manual_resampled_points":len(output_manual),
        "manual_recorded_length_m":length,"manual_resampled_length_m":line_resampled.length,
        "manual_spacing_target_m":TARGET_SPACING_M,
        "manual_spacing_actual_m":length/intervals,
        "manual_hausdorff_deviation_m":deviation,
        "max_consecutive_waypoint_gap_m":max(gaps),
        "speed_counts":dict(Counter(row["speed_mps"] for row in revised)),
        "first_source_id":ids[0],"last_source_id":ids[-1],
        "preserved_review_cuts":source_report["removed_original_ranges"],
        "limitations":["Blades-off supervised field test only; no autonomous mowing approval",
                       "RTK-fixed position and heading and handheld Pause required before start",
                       "New return and shortcut tracking remain untested on tractor01",
                       "Recorded manual source includes 22 DGPS samples; resampling does not improve source accuracy",
                       "Garden-right center stripe coverage is not included"],
    }
    report.write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({"mission":str(mission),"waypoints":len(revised),
                      "manual_points":len(output_manual),"manual_deviation_m":round(deviation,4),
                      "max_gap_m":round(max(gaps),4),"sha256":summary["mission_sha256"]},indent=2))


if __name__ == "__main__":
    main()
