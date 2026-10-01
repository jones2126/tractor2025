#!/usr/bin/env python3
"""Diagnose the 2026-09-26 manual survey for perimeter and obstacle updates."""

from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from datetime import datetime, timedelta
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from shapely import concave_hull
from shapely.geometry import LineString, MultiPoint, Point, shape
from shapely.ops import transform, unary_union

from build_20260923_combined_coverage_review import REVIEW, SITE, Frame


REPO = Path(__file__).resolve().parents[2]
RUN = SITE / "runs/20260926_phone_manual_perimeter_extension"
SOURCE = RUN / "field_test_20260926_121311.csv"
INVENTORY = SITE / "site_inventory/62_Collins_site_inventory.geojson"
OUT = SITE / "analysis/20260926_perimeter_inventory_update_REVIEW_ONLY"


def parse_time(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_frame() -> Frame:
    with (REVIEW / "segment_02_candidate_path.csv").open(
        newline="", encoding="utf-8-sig"
    ) as handle:
        row = next(csv.DictReader(handle))
    return Frame(float(row["lat"]), float(row["lon"]))


def to_xy(frame: Frame, geometry):
    def convert(lon, lat, z=None):
        return frame.xy(lat, lon)

    return transform(convert, geometry)


def load_inventory(frame: Frame):
    collection = json.loads(INVENTORY.read_text(encoding="utf-8"))
    features = []
    for item in collection["features"]:
        features.append((item["properties"], to_xy(frame, shape(item["geometry"]))))
    return collection, features


def load_points(frame: Frame):
    points = []
    last_by_mode = {}
    with SOURCE.open(newline="", encoding="utf-8-sig") as handle:
        for source_row, row in enumerate(csv.DictReader(handle), 2):
            mode = row.get("trans_mode")
            if mode not in {"0", "1"}:
                continue
            lat, lon = number(row.get("lat")), number(row.get("lon"))
            speed = number(row.get("speed_mps")) or 0.0
            if lat is None or lon is None or speed < 0.12:
                continue
            timestamp = parse_time(row["time"])
            previous = last_by_mode.get(mode)
            if previous and (timestamp - previous).total_seconds() < 0.20:
                continue
            x, y = frame.xy(lat, lon)
            points.append({
                "row": source_row,
                "time": timestamp,
                "time_text": row["time"],
                "mode": mode,
                "source": "phone" if mode == "0" else "legacy_handheld",
                "speed": speed,
                "fix": row.get("fix_quality"),
                "x": x,
                "y": y,
                "lat": lat,
                "lon": lon,
            })
            last_by_mode[mode] = timestamp
    return points


def continuous_parts(points, maximum_gap_s=3.0, maximum_jump_m=4.0):
    parts = []
    current = []
    for point in points:
        split = False
        if current:
            delta = (point["time"] - current[-1]["time"]).total_seconds()
            jump = math.hypot(point["x"] - current[-1]["x"], point["y"] - current[-1]["y"])
            split = point["mode"] != current[-1]["mode"] or delta > maximum_gap_s or jump > maximum_jump_m
        if split:
            if len(current) >= 2:
                parts.append(current)
            current = []
        current.append(point)
    if len(current) >= 2:
        parts.append(current)
    return parts


def contiguous_true_runs(points, predicate, maximum_gap_s=1.0):
    runs = []
    current = []
    for point in points:
        match = predicate(point)
        if current and ((point["time"] - current[-1]["time"]).total_seconds() > maximum_gap_s or not match):
            runs.append(current)
            current = []
        if match:
            current.append(point)
    if current:
        runs.append(current)
    return runs


def line_length(points):
    return sum(
        math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        for a, b in zip(points, points[1:])
    )


def draw_geometry(axis, geometry, edge, face="none", width=1.5, alpha=1.0):
    if geometry.geom_type == "Polygon":
        parts = [geometry]
    elif geometry.geom_type == "MultiPolygon":
        parts = list(geometry.geoms)
    else:
        parts = []
    for part in parts:
        x, y = part.exterior.xy
        axis.fill(x, y, facecolor=face, edgecolor=edge, linewidth=width, alpha=alpha)
        for ring in part.interiors:
            x, y = ring.xy
            axis.fill(x, y, facecolor="white", edgecolor=edge, linewidth=width, alpha=1.0)


def draw_context(axis, areas, obstacles, transitions):
    colors = {
        "AREA-BACKYARD-GARDENS": "#1565c0",
        "AREA-FRONT-YARD": "#ef6c00",
        "AREA-OVER-ROAD": "#795548",
    }
    for asset_id, geometry in areas.items():
        color = colors.get(asset_id, "#607d8b")
        draw_geometry(axis, geometry, color, color, alpha=0.10)
    for _props, geometry in obstacles:
        draw_geometry(axis, geometry, "#c62828", "#ef5350", alpha=0.35)
    for route in transitions:
        x, y = route.xy
        axis.plot(x, y, color="#00897b", linewidth=0.8, linestyle="--", alpha=0.65)


def draw_focused_panels(points, areas, obstacles, transitions, output):
    windows = [
        ("Phone control", "2026-09-26T16:13:00+00:00", "2026-09-26T16:37:20+00:00"),
        ("Legacy handheld — backyard south/east", "2026-09-26T16:37:20+00:00", "2026-09-26T16:53:15+00:00"),
        ("Legacy handheld — front-yard work", "2026-09-26T16:53:15+00:00", "2026-09-26T17:05:00+00:00"),
        ("Legacy handheld — over-road work", "2026-09-26T17:05:00+00:00", "2026-09-26T17:12:30+00:00"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(15, 13), dpi=170)
    for axis, (title, start_text, end_text) in zip(axes.flat, windows):
        start, end = parse_time(start_text), parse_time(end_text)
        selected = [p for p in points if start <= p["time"] <= end]
        draw_context(axis, areas, obstacles, transitions)
        if selected:
            for part in continuous_parts(selected):
                axis.plot([p["x"] for p in part], [p["y"] for p in part], color="#263238", linewidth=1.2)
                stride = max(1, len(part) // 25)
                for first, second in zip(part[::stride], part[1::stride]):
                    axis.annotate("", xy=(second["x"], second["y"]), xytext=(first["x"], first["y"]),
                                  arrowprops={"arrowstyle": "->", "color": "#00897b", "lw": 0.7})
            next_label = selected[0]["time"]
            for point in selected:
                if point["time"] < next_label:
                    continue
                axis.scatter(point["x"], point["y"], s=11, color="#d32f2f", zorder=6)
                axis.annotate(point["time"].astimezone().strftime("%H:%M:%S"), (point["x"], point["y"]),
                              xytext=(3, 3), textcoords="offset points", fontsize=5.5, color="#7f0000")
                next_label = point["time"] + timedelta(seconds=30)
            margin = 5.0
            xs = [p["x"] for p in selected]
            ys = [p["y"] for p in selected]
            axis.set_xlim(min(xs) - margin, max(xs) + margin)
            axis.set_ylim(min(ys) - margin, max(ys) + margin)
        axis.set_title(title)
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, linewidth=0.3, alpha=0.35)
        axis.set_xlabel("Local east (m)")
        axis.set_ylabel("Local north (m)")
    fig.suptitle("2026-09-26 route sequence by operating interval", fontsize=16)
    fig.tight_layout()
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def draw_backyard_hull_options(points, areas, obstacles, transitions, output):
    start = parse_time("2026-09-26T16:43:47+00:00")
    end = parse_time("2026-09-26T16:51:33+00:00")
    selected = [p for p in points if start <= p["time"] <= end]
    cloud = MultiPoint([(p["x"], p["y"]) for p in selected])
    ratios = (0.50, 0.60, 0.70, 1.00)
    fig, axes = plt.subplots(2, 2, figsize=(13, 11), dpi=160)
    for axis, ratio in zip(axes.flat, ratios):
        draw_context(axis, areas, obstacles, transitions)
        hull = concave_hull(cloud, ratio=ratio, allow_holes=False)
        candidate = hull.union(areas["AREA-BACKYARD-GARDENS"]).buffer(0)
        draw_geometry(axis, candidate, "#8e24aa", "#ab47bc", width=2.0, alpha=0.20)
        axis.plot([p["x"] for p in selected], [p["y"] for p in selected], color="#263238", linewidth=0.8)
        added = candidate.area - areas["AREA-BACKYARD-GARDENS"].area
        axis.set_title(f"Hull ratio {ratio:.2f} — adds {added:.1f} m² to current area")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, linewidth=0.3, alpha=0.35)
    fig.suptitle("Backyard south-extension geometry options — review only", fontsize=15)
    fig.tight_layout()
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    frame = load_frame()
    _collection, inventory = load_inventory(frame)
    points = load_points(frame)

    areas = {
        props["asset_id"]: geometry
        for props, geometry in inventory
        if props.get("category") == "mowable_area"
    }
    transitions = [
        geometry for props, geometry in inventory
        if props.get("category") == "transition_route"
    ]
    obstacles = [
        (props, geometry) for props, geometry in inventory
        if props.get("category") == "obstacle" and props.get("geometry_role") == "mowing_exclusion"
    ]
    area_union = unary_union(list(areas.values()))
    transition_corridor = unary_union(transitions).buffer(1.5) if transitions else Point().buffer(0)

    for point in points:
        location = Point(point["x"], point["y"])
        nearest_id, nearest_area = min(areas.items(), key=lambda item: location.distance(item[1]))
        point["nearest_area"] = nearest_id
        point["outside_m"] = location.distance(nearest_area) if not nearest_area.covers(location) else 0.0
        point["in_transition_corridor"] = transition_corridor.covers(location)

    outside_runs = contiguous_true_runs(
        points,
        lambda p: p["outside_m"] >= 0.75 and not p["in_transition_corridor"],
    )
    outside_rows = []
    for index, run in enumerate(outside_runs, 1):
        duration = (run[-1]["time"] - run[0]["time"]).total_seconds()
        length = line_length(run)
        if duration < 1.0 or length < 0.75:
            continue
        outside_rows.append({
            "candidate_id": f"OUT-{index:03d}",
            "source": run[0]["source"],
            "start": run[0]["time_text"],
            "end": run[-1]["time_text"],
            "duration_s": round(duration, 2),
            "path_length_m": round(length, 2),
            "maximum_outside_m": round(max(p["outside_m"] for p in run), 2),
            "nearest_area": max(
                {p["nearest_area"] for p in run},
                key=lambda name: sum(p["nearest_area"] == name for p in run),
            ),
            "start_lat": run[0]["lat"],
            "start_lon": run[0]["lon"],
            "end_lat": run[-1]["lat"],
            "end_lon": run[-1]["lon"],
            "centroid_x_m": round(sum(p["x"] for p in run) / len(run), 2),
            "centroid_y_m": round(sum(p["y"] for p in run) / len(run), 2),
            "points": len(run),
        })

    with (OUT / "outside_boundary_runs.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        columns = list(outside_rows[0]) if outside_rows else ["candidate_id"]
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(outside_rows)

    timeline = []
    next_sample = points[0]["time"]
    for point in points:
        if point["time"] < next_sample:
            continue
        timeline.append({
            "local_time": point["time"].astimezone().strftime("%H:%M:%S"),
            "utc_time": point["time_text"],
            "source": point["source"],
            "x_m": round(point["x"], 2),
            "y_m": round(point["y"], 2),
            "speed_mps": point["speed"],
            "nearest_area": point["nearest_area"],
            "outside_m": round(point["outside_m"], 2),
        })
        next_sample = point["time"] + timedelta(seconds=5)
    with (OUT / "route_timeline_5s.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(timeline[0]))
        writer.writeheader()
        writer.writerows(timeline)

    fig, axis = plt.subplots(figsize=(13, 10), dpi=160)
    draw_context(axis, areas, obstacles, transitions)

    start_time = points[0]["time"]
    lines = []
    values = []
    for part in continuous_parts(points):
        for a, b in zip(part, part[1:]):
            lines.append([(a["x"], a["y"]), (b["x"], b["y"])])
            values.append((a["time"] - start_time).total_seconds() / 60.0)
    collection = LineCollection(lines, cmap="viridis", linewidths=1.4, alpha=0.9)
    collection.set_array(values)
    axis.add_collection(collection)
    figure_colorbar = fig.colorbar(collection, ax=axis, shrink=0.75)
    figure_colorbar.set_label("Minutes after field logger start")

    next_label = start_time
    for point in points:
        if point["time"] < next_label:
            continue
        axis.scatter(point["x"], point["y"], s=18, color="black", zorder=5)
        axis.annotate(point["time"].astimezone().strftime("%H:%M"), (point["x"], point["y"]),
                      xytext=(4, 4), textcoords="offset points", fontsize=6)
        next_label = point["time"] + timedelta(minutes=2)

    for item, run in zip(outside_rows, [r for r in outside_runs if (r[-1]["time"] - r[0]["time"]).total_seconds() >= 1.0 and line_length(r) >= 0.75]):
        center_x = sum(p["x"] for p in run) / len(run)
        center_y = sum(p["y"] for p in run) / len(run)
        axis.annotate(item["candidate_id"], (center_x, center_y), color="#b71c1c", fontsize=7, fontweight="bold")

    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, linewidth=0.3, alpha=0.35)
    axis.set_title("62 Collins — 2026-09-26 manual survey diagnostic\nTime-colored route; red labels are sustained travel outside current mowing areas")
    axis.set_xlabel("Local east (m)")
    axis.set_ylabel("Local north (m)")
    fig.tight_layout()
    fig.savefig(OUT / "20260926_time_route_diagnostic.png", bbox_inches="tight")
    plt.close(fig)
    draw_focused_panels(
        points,
        areas,
        obstacles,
        transitions,
        OUT / "20260926_route_sequence_panels.png",
    )
    draw_backyard_hull_options(
        points,
        areas,
        obstacles,
        transitions,
        OUT / "20260926_backyard_hull_options.png",
    )

    report = {
        "status": "DIAGNOSTIC_ONLY",
        "source": str(SOURCE.relative_to(REPO)).replace("\\", "/"),
        "sampled_moving_points": len(points),
        "continuous_parts": len(continuous_parts(points)),
        "outside_candidate_runs": outside_rows,
        "method": {
            "moving_speed_minimum_mps": 0.12,
            "outside_distance_minimum_m": 0.75,
            "transition_corridor_half_width_m": 1.5,
            "minimum_run_duration_s": 1.0,
            "minimum_run_length_m": 0.75,
        },
    }
    (OUT / "diagnostic_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(OUT / "20260926_time_route_diagnostic.png")
    print(OUT / "outside_boundary_runs.csv")
    print(f"sampled points={len(points)} outside runs={len(outside_rows)}")


if __name__ == "__main__":
    main()
