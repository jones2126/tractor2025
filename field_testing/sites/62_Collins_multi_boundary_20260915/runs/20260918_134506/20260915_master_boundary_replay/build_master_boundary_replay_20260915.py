#!/usr/bin/env python3
"""Build review-only exact-drive replay paths from the 2026-09-15 survey."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
REVIEW = HERE.parents[1] / "runs" / "20260915_133923" / "boundary_review"
OUT = HERE / "generated"
LOOKAHEAD_M = 2.0
SPEED_MPS = 0.5
SELECTED_SEGMENTS = tuple(range(2, 18))
POLE_LOOP_START = 13  # zero-based within Segment 14; closest long-loop closure
POLE_LOOP_END = 63    # inclusive; closure gap is approximately 0.07 m

COMPOSITES = {
    "main_backyard": (3, 4),
    "garden_right": (6,),
    "garden_left": (8,),
    "front_yard": (10,),
    # Segment 14 is retained because it is the exact driven connection around
    # the pole between the two outer-boundary pieces.
    "over_the_road_recorded_with_pole_survey": (13, 14, 15),
}


def load_labels():
    with (REVIEW / "segment_labeling_worksheet.csv").open(
        newline="", encoding="utf-8-sig"
    ) as handle:
        return {int(row["segment"]): row for row in csv.DictReader(handle)}


def load_segment(number):
    path = REVIEW / f"segment_{number:02d}_candidate_path.csv"
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        row["segment"] = number
        row["lat"] = float(row["lat"])
        row["lon"] = float(row["lon"])
        row["heading_deg"] = float(row["heading_deg"])
    return rows


def distance_m(a, b):
    latitude = math.radians((a["lat"] + b["lat"]) / 2.0)
    east = (b["lon"] - a["lon"]) * 111_320.0 * math.cos(latitude)
    north = (b["lat"] - a["lat"]) * 110_540.0
    return math.hypot(east, north)


def path_length(rows):
    return sum(distance_m(a, b) for a, b in zip(rows, rows[1:]))


def yaw_rad(row):
    return math.radians(90.0 - row["heading_deg"])


def write_mission(path, rows):
    text = "".join(
        f"{row['lat']:.9f} {row['lon']:.9f} {yaw_rad(row):.6f} "
        f"{LOOKAHEAD_M:.2f} {SPEED_MPS:.2f}\n"
        for row in rows
    )
    path.write_text(text, encoding="ascii", newline="\n")


def feature(name, kind, rows, segments, closed=False):
    coordinates = [[row["lon"], row["lat"]] for row in rows]
    if closed and coordinates and coordinates[0] != coordinates[-1]:
        coordinates.append(coordinates[0])
    return {
        "type": "Feature",
        "properties": {
            "name": name,
            "kind": kind,
            "source_segments": list(segments),
            "review_only": True,
        },
        "geometry": {
            "type": "Polygon" if closed else "LineString",
            "coordinates": [coordinates] if closed else coordinates,
        },
    }


def local_xy(rows, origin):
    latitude = math.radians(origin["lat"])
    return [
        (
            (row["lon"] - origin["lon"]) * 111_320.0 * math.cos(latitude),
            (row["lat"] - origin["lat"]) * 110_540.0,
        )
        for row in rows
    ]


def write_preview(segments, labels, pole_loop):
    origin = segments[2][0]
    fig, axis = plt.subplots(figsize=(11, 9), dpi=170)
    for number in SELECTED_SEGMENTS:
        xy = local_xy(segments[number], origin)
        x, y = zip(*xy)
        role = labels[number]["role"].strip()
        if number == 14:
            color, width, style = "#c62828", 2.3, "-"
        elif "transition" in role:
            color, width, style = "#6f7782", 1.4, "--"
        else:
            color, width, style = "#1565c0", 1.8, "-"
        axis.plot(x, y, color=color, linewidth=width, linestyle=style, alpha=0.9)
        middle = len(x) // 2
        axis.text(x[middle], y[middle], str(number), fontsize=8, weight="bold")
    obstacle_xy = local_xy(pole_loop, origin)
    ox, oy = zip(*obstacle_xy)
    axis.fill(ox, oy, color="#ef5350", alpha=0.28, label="Extracted pole keep-out loop")
    axis.plot([], [], color="#1565c0", linewidth=1.8, label="Boundary survey")
    axis.plot([], [], color="#6f7782", linewidth=1.4, linestyle="--", label="Driven transition")
    axis.plot([], [], color="#c62828", linewidth=2.3, label="Full Segment 14 pole survey")
    axis.set_title("62 Collins master boundary replay — exact recorded phase order")
    axis.set_xlabel("East of Segment 2 start (m)")
    axis.set_ylabel("North of Segment 2 start (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, linewidth=0.5, alpha=0.25)
    axis.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "62_Collins_master_boundary_replay_REVIEW_20260915.png")
    plt.close(fig)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    labels = load_labels()
    segments = {number: load_segment(number) for number in SELECTED_SEGMENTS}

    master = []
    manifest_phases = []
    audit_rows = []
    inter_segment_gaps = []
    for number in SELECTED_SEGMENTS:
        rows = segments[number]
        if master:
            inter_segment_gaps.append({
                "from_segment": master[-1]["segment"],
                "to_segment": number,
                "gap_m": round(distance_m(master[-1], rows[0]), 3),
            })
        start = len(master) + 1
        master.extend(rows)
        end = len(master)
        label = labels[number]["label"].strip()
        role = labels[number]["role"].strip()
        manifest_phases.append({
            "segment": number,
            "label": label,
            "role": role,
            "waypoint_start": start,
            "waypoint_end": end,
            "points": len(rows),
            "path_length_m": round(path_length(rows), 3),
        })
        for waypoint, row in enumerate(rows, start):
            audit_rows.append({
                "waypoint": waypoint,
                "segment": number,
                "phase": label.replace(" ", "_"),
                "role": role,
                "lat": f"{row['lat']:.9f}",
                "lon": f"{row['lon']:.9f}",
                "yaw_rad": f"{yaw_rad(row):.6f}",
                "lookahead_m": f"{LOOKAHEAD_M:.2f}",
                "speed_mps": f"{SPEED_MPS:.2f}",
            })

    master_path = OUT / "62_Collins_master_boundary_replay_REVIEW_ONLY_20260915.txt"
    write_mission(master_path, master)
    with (OUT / "62_Collins_master_boundary_replay_audit_20260915.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=audit_rows[0].keys())
        writer.writeheader()
        writer.writerows(audit_rows)

    individual_dir = OUT / "individual_paths"
    individual_dir.mkdir(exist_ok=True)
    for number, rows in segments.items():
        write_mission(individual_dir / f"segment_{number:02d}_REVIEW_ONLY.txt", rows)
    for name, numbers in COMPOSITES.items():
        rows = [row for number in numbers for row in segments[number]]
        write_mission(individual_dir / f"{name}_REVIEW_ONLY.txt", rows)

    pole_loop = segments[14][POLE_LOOP_START:POLE_LOOP_END + 1]
    geojson = {
        "type": "FeatureCollection",
        "features": [
            feature("main backyard boundary", "boundary", segments[3] + segments[4], (3, 4)),
            feature("garden right boundary", "boundary", segments[6], (6,)),
            feature("garden left boundary", "boundary", segments[8], (8,)),
            feature("front yard boundary", "boundary", segments[10], (10,)),
            feature("over-road outer boundary pieces", "boundary-pieces", segments[13] + segments[15], (13, 15)),
            feature("telephone pole 1 driven keep-out loop", "obstacle", pole_loop, (14,), closed=True),
            feature("telephone pole 1 full recorded survey", "obstacle-survey-path", segments[14], (14,)),
        ],
    }
    (OUT / "62_Collins_boundaries_and_obstacle_REVIEW_20260915.geojson").write_text(
        json.dumps(geojson, indent=2), encoding="utf-8", newline="\n"
    )
    write_preview(segments, labels, pole_loop)

    report = {
        "status": "REVIEW_ONLY_NOT_FIELD_READY",
        "source_directory": str(REVIEW),
        "selected_segments": list(SELECTED_SEGMENTS),
        "omitted_segment": 1,
        "master_waypoints": len(master),
        "master_path_length_m": round(path_length(master), 3),
        "lookahead_m": LOOKAHEAD_M,
        "speed_mps": SPEED_MPS,
        "phases": manifest_phases,
        "inter_segment_gaps": inter_segment_gaps,
        "maximum_inter_segment_gap_m": max(item["gap_m"] for item in inter_segment_gaps),
        "telephone_pole": {
            "source_segment": 14,
            "loop_source_indices_zero_based": [POLE_LOOP_START, POLE_LOOP_END],
            "loop_points": len(pole_loop),
            "loop_length_m": round(path_length(pole_loop), 3),
            "closure_gap_m": round(distance_m(pole_loop[0], pole_loop[-1]), 3),
            "note": "Segment 14 remains in master replay; no straight 13-to-15 shortcut was created.",
        },
        "limitations": [
            "This is an exact-drive boundary/transition replay, not a mowing coverage mission.",
            "No launcher is supplied; recovery controller still requires recorded-data and tractor validation.",
            "The over-road outer boundary is represented by Segments 13 and 15; Segment 14 is separate obstacle geometry and remains in replay order as the safe driven connection.",
        ],
    }
    (OUT / "62_Collins_master_boundary_replay_report_20260915.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8", newline="\n"
    )
    print(json.dumps({
        "mission": str(master_path),
        "waypoints": len(master),
        "length_m": report["master_path_length_m"],
        "max_gap_m": report["maximum_inter_segment_gap_m"],
        "pole_loop_closure_m": report["telephone_pole"]["closure_gap_m"],
    }, indent=2))


if __name__ == "__main__":
    main()
