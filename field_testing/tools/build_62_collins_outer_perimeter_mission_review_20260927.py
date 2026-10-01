#!/usr/bin/env python3
"""Build review-only, per-area outer perimeter missions from rev-3 inventory."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, Polygon, mapping, shape
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

from build_62_collins_site_inventory_rev003_20260926 import (
    DECK_RADIUS_M,
    REPO,
    SITE,
    largest_polygon,
    load_frame,
    to_xy,
)


SOURCE = (
    SITE
    / "site_inventory/revisions/rev_003_20260926_REVIEW_ONLY"
    / "62_Collins_site_inventory_rev003_REVIEW_ONLY.geojson"
)
OUT = SITE / "mission_plans/20260927_outer_perimeter_1mps_REVIEW_ONLY"
MISSIONS = OUT / "individual_area_missions"
GEOJSON_OUT = OUT / "62_Collins_outer_perimeter_1mps_REVIEW_ONLY.geojson"
AUDIT_OUT = OUT / "62_Collins_outer_perimeter_1mps_audit.csv"
REPORT_OUT = OUT / "62_Collins_outer_perimeter_1mps_report.json"
PNG_OUT = OUT / "62_Collins_outer_perimeter_1mps_REVIEW.png"
README_OUT = OUT / "README.md"

SPEED_MPS = 1.0
LOOKAHEAD_M = 2.0
WAYPOINT_SPACING_M = 0.20
OBSTACLE_REVIEW_MARGIN_M = 0.3048
OBSTACLE_CENTER_CLEARANCE_M = DECK_RADIUS_M + OBSTACLE_REVIEW_MARGIN_M
NUMERICAL_CLEARANCE_MARGIN_M = 0.01
SHARP_TURN_REVIEW_DEG = 45.0
TURN_SOFTEN_RADIUS_M = 2.0

AREA_ORDER = (
    "AREA-BACKYARD-GARDENS",
    "AREA-FRONT-YARD",
    "AREA-OVER-ROAD",
)
START_ACCESS = {
    "AREA-BACKYARD-GARDENS": "ACCESS-BACKYARD-EAST",
    "AREA-FRONT-YARD": "ACCESS-FRONT-SOUTHWEST",
    "AREA-OVER-ROAD": "ACCESS-OVERROAD-NORTH-OUTBOUND",
}
INCOMING_ROUTE = {
    "AREA-BACKYARD-GARDENS": "ROUTE-BASE-TO-BACKYARD",
    "AREA-FRONT-YARD": "ROUTE-BACKYARD-TO-FRONT",
    "AREA-OVER-ROAD": "ROUTE-FRONT-TO-OVERROAD",
}


def wrap_angle(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def tangent_heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def resample_closed(ring, spacing):
    line = LineString(ring.coords)
    count = max(8, math.ceil(line.length / spacing))
    points = [line.interpolate(index * line.length / count).coords[0] for index in range(count)]
    return points


def rotate_clockwise(points, start_point):
    start_index = min(range(len(points)), key=lambda index: Point(points[index]).distance(start_point))
    selected = points[start_index:] + points[:start_index]
    return selected + [selected[0]]


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def soften_local_detour(points, navigable, center, influence_radius, turn_radius):
    """Replace one boundary notch with the matching turn-radius-safe opened arc."""
    ring = points[:-1]
    pivot = min(range(len(ring)), key=lambda index: math.dist(ring[index], center))
    first = pivot
    while first > 0 and math.dist(ring[first - 1], center) < influence_radius:
        first -= 1
    last = pivot
    while last + 1 < len(ring) and math.dist(ring[last + 1], center) < influence_radius:
        last += 1
    if first == 0 or last == len(ring) - 1:
        raise RuntimeError("Local turn-softening interval wrapped around the mission start")

    opened = navigable.buffer(-turn_radius, join_style="round")
    if opened.is_empty:
        raise RuntimeError("Navigable area cannot support the local turn-softening radius")
    opened = largest_polygon(
        opened.buffer(turn_radius, join_style="round")
        .intersection(navigable)
        .buffer(0)
    )
    opened_points = resample_closed(opened.exterior, WAYPOINT_SPACING_M)[:-1]
    start_index = min(range(len(opened_points)), key=lambda index: math.dist(opened_points[index], ring[first]))
    end_index = min(range(len(opened_points)), key=lambda index: math.dist(opened_points[index], ring[last]))

    def forward_arc(start, end):
        if end >= start:
            return opened_points[start:end + 1]
        return opened_points[start:] + opened_points[:end + 1]

    candidates = [
        forward_arc(start_index, end_index),
        list(reversed(forward_arc(end_index, start_index))),
    ]
    valid = []
    for arc in candidates:
        candidate = [ring[first], *arc, ring[last]]
        line = LineString(candidate)
        if navigable.buffer(0.002).covers(line):
            valid.append(candidate)
    if not valid:
        raise RuntimeError("No safe local turn-softening arc was found")
    replacement = min(valid, key=path_length)
    combined = ring[:first] + replacement + ring[last + 1:]
    return resample_closed(LineString(combined + [combined[0]]), WAYPOINT_SPACING_M)


def write_mission(path, points, frame):
    rows = []
    for index, point in enumerate(points):
        target = points[index + 1] if index + 1 < len(points) else points[index]
        yaw = tangent_heading(point, target) if target != point else tangent_heading(points[index - 1], point)
        lat, lon = frame.ll(*point)
        rows.append((lat, lon, yaw))
    text = "".join(
        f"{lat:.9f} {lon:.9f} {yaw:.6f} {LOOKAHEAD_M:.2f} {SPEED_MPS:.2f}\n"
        for lat, lon, yaw in rows
    )
    path.write_text(text, encoding="ascii", newline="\n")
    return rows


def draw_polygon(axis, geometry, color, alpha=0.10, width=1.0):
    parts = [geometry] if geometry.geom_type == "Polygon" else list(geometry.geoms)
    for part in parts:
        x, y = part.exterior.xy
        axis.fill(x, y, facecolor=color, edgecolor=color, alpha=alpha, linewidth=width)
        for interior in part.interiors:
            x, y = interior.xy
            axis.fill(x, y, facecolor="white", edgecolor=color, alpha=1.0, linewidth=width)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    MISSIONS.mkdir(parents=True, exist_ok=True)
    frame = load_frame()
    source = json.loads(SOURCE.read_text(encoding="utf-8"))

    areas = {}
    obstacles = {}
    accesses = {}
    routes = {}
    for item in source["features"]:
        props = item["properties"]
        role = props.get("geometry_role")
        asset_id = props.get("asset_id")
        geometry = to_xy(frame, shape(item["geometry"]))
        if role == "candidate_mowable_boundary":
            areas[asset_id] = largest_polygon(geometry)
        elif role in {"candidate_mowing_exclusion", "mowing_exclusion"}:
            obstacles[asset_id] = largest_polygon(geometry)
        elif role == "area_entry_exit_point":
            accesses[asset_id] = geometry
        elif role == "recorded_centerline_between_areas":
            routes[asset_id] = geometry

    expanded_obstacles = unary_union([
        geometry.buffer(OBSTACLE_CENTER_CLEARANCE_M + NUMERICAL_CLEARANCE_MARGIN_M)
        for geometry in obstacles.values()
    ])

    records = []
    audit_rows = []
    geo_features = []
    plot_paths = {}
    plot_sharp_points = {}
    for area_id in AREA_ORDER:
        area = areas[area_id]
        shell = Polygon(area.exterior)
        # The inventory exterior is the surveyed deck-edge perimeter. With a
        # clockwise route, the left deck edge is outward, so the GPS centerline
        # belongs one deck half-width inside that perimeter.
        center_drive_area = shell.buffer(-DECK_RADIUS_M, join_style="round")
        navigable = largest_polygon(center_drive_area.difference(expanded_obstacles).buffer(0))
        softened = navigable.buffer(-TURN_SOFTEN_RADIUS_M, join_style="round")
        if softened.is_empty:
            raise RuntimeError(f"{area_id} cannot support the requested turn-softening radius")
        navigable = largest_polygon(
            softened.buffer(TURN_SOFTEN_RADIUS_M, join_style="round")
            .intersection(navigable)
            .buffer(0)
        )
        navigable = orient(navigable, sign=-1.0)
        raw_ring = navigable.exterior
        access = accesses[START_ACCESS[area_id]]
        incoming = routes[INCOMING_ROUTE[area_id]]
        incoming_coordinates = list(incoming.coords)
        incoming_heading = tangent_heading(incoming_coordinates[-2], incoming_coordinates[-1])
        sampled = resample_closed(raw_ring, WAYPOINT_SPACING_M)
        points = rotate_clockwise(sampled, access)
        entry_turn = abs(wrap_angle(tangent_heading(points[0], points[1]) - incoming_heading))
        line = LineString(points)
        start_gap = Point(points[0]).distance(access)
        minimum_obstacle_clearance = min(line.distance(obstacle) for obstacle in obstacles.values())
        clearance_shortfall = max(0.0, OBSTACLE_CENTER_CLEARANCE_M - minimum_obstacle_clearance)
        boundary_offsets = [Point(point).distance(shell.boundary) for point in points[:-1]]
        heading_steps = [0.0] * len(points)
        for index, (a, b, c) in enumerate(zip(points, points[1:], points[2:]), 1):
            heading_steps[index] = abs(wrap_angle(tangent_heading(b, c) - tangent_heading(a, b)))
        max_heading_step = max(heading_steps)
        sharp_indices = [
            index for index, value in enumerate(heading_steps)
            if math.degrees(value) >= SHARP_TURN_REVIEW_DEG
        ]

        slug = area_id.lower().replace("area-", "").replace("-", "_")
        mission_name = f"62_Collins_{slug}_outer_perimeter_1mps_REVIEW_ONLY.txt"
        mission_path = MISSIONS / mission_name
        mission_rows = write_mission(mission_path, points, frame)
        mission_sha256 = hashlib.sha256(mission_path.read_bytes()).hexdigest()
        record = {
            "area_id": area_id,
            "mission_file": str(mission_path.relative_to(REPO)).replace("\\", "/"),
            "mission_sha256": mission_sha256,
            "waypoints": len(points),
            "path_length_m": round(path_length(points), 3),
            "speed_mps": SPEED_MPS,
            "lookahead_m": LOOKAHEAD_M,
            "waypoint_spacing_m": WAYPOINT_SPACING_M,
            "start_access": START_ACCESS[area_id],
            "start_gap_m": round(start_gap, 3),
            "direction": "clockwise",
            "left_deck_edge_tracks_outer_perimeter": True,
            "entry_heading_change_deg": round(math.degrees(entry_turn), 2),
            "minimum_gps_center_to_outer_perimeter_m": round(min(boundary_offsets), 3),
            "median_gps_center_to_outer_perimeter_m": round(float(sorted(boundary_offsets)[len(boundary_offsets) // 2]), 3),
            "minimum_obstacle_exclusion_clearance_m": round(minimum_obstacle_clearance, 3),
            "required_obstacle_exclusion_clearance_m": OBSTACLE_CENTER_CLEARANCE_M,
            "clearance_shortfall_m": round(clearance_shortfall, 3),
            "maximum_sample_heading_step_deg": round(math.degrees(max_heading_step), 2),
            "sharp_turn_review_threshold_deg": SHARP_TURN_REVIEW_DEG,
            "sharp_turn_review_count": len(sharp_indices),
            "review_status": (
                "clearance_failure" if clearance_shortfall > 0.01
                else "steering_geometry_review_required" if sharp_indices
                else "geometry_clear_start_connector_unresolved"
            ),
        }
        records.append(record)
        plot_paths[area_id] = line
        plot_sharp_points[area_id] = [points[index] for index in sharp_indices]
        geo_features.append({
            "type": "Feature",
            "id": f"{area_id}-OUTER-PERIMETER-REVIEW",
            "properties": {**record, "status": "REVIEW_ONLY_NOT_FIELD_AUTHORIZED"},
            "geometry": mapping(LineString([tuple(reversed(frame.ll(x, y))) for x, y in line.coords])),
        })
        for waypoint, ((lat, lon, yaw), point) in enumerate(zip(mission_rows, points), 1):
            audit_rows.append({
                "area_id": area_id,
                "waypoint": waypoint,
                "east_m": f"{point[0]:.3f}",
                "north_m": f"{point[1]:.3f}",
                "lat": f"{lat:.9f}",
                "lon": f"{lon:.9f}",
                "yaw_rad": f"{yaw:.6f}",
                "lookahead_m": f"{LOOKAHEAD_M:.2f}",
                "speed_mps": f"{SPEED_MPS:.2f}",
                "heading_step_deg": f"{math.degrees(heading_steps[waypoint - 1]):.2f}",
                "sharp_turn_review": "yes" if waypoint - 1 in sharp_indices else "no",
            })

    collection = {
        "type": "FeatureCollection",
        "name": "62 Collins outer perimeter 1 mps review missions",
        "properties": {
            "status": "REVIEW_ONLY_NOT_FIELD_AUTHORIZED",
            "source": str(SOURCE.relative_to(REPO)).replace("\\", "/"),
            "launcher_generated": False,
            "master_between_area_mission_generated": False,
        },
        "features": geo_features,
    }
    GEOJSON_OUT.write_text(json.dumps(collection, indent=2) + "\n", encoding="utf-8")

    with AUDIT_OUT.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=audit_rows[0].keys())
        writer.writeheader()
        writer.writerows(audit_rows)

    report = {
        "status": "REVIEW_ONLY_NOT_FIELD_AUTHORIZED",
        "source": str(SOURCE.relative_to(REPO)).replace("\\", "/"),
        "speed_mps": SPEED_MPS,
        "lookahead_m": LOOKAHEAD_M,
        "waypoint_spacing_m": WAYPOINT_SPACING_M,
        "deck_radius_m": DECK_RADIUS_M,
        "obstacle_review_margin_m": OBSTACLE_REVIEW_MARGIN_M,
        "obstacle_center_clearance_m": OBSTACLE_CENTER_CLEARANCE_M,
        "numerical_clearance_margin_m": NUMERICAL_CLEARANCE_MARGIN_M,
        "turn_softening_radius_m": TURN_SOFTEN_RADIUS_M,
        "route_direction": "clockwise",
        "deck_edge_rule": "left deck edge follows the surveyed outer perimeter; GPS centerline is inset by half the 42-inch deck width",
        "areas": records,
        "owner_review_decisions": {
            "broad_smooth_obstacle_detours": [
                "OBS-NEW-003",
                "OBS-NEW-007",
                "OBS-NEW-011",
                "OBS-NEW-012",
            ],
            "note": "Owner confirms adequate surrounding space; do not follow hard polygon-cut corners. Replace with gradual tangent-connected approaches and departures.",
        },
        "limitations": [
            "The three area missions are separate review paths, not one continuous field mission.",
            "No launcher, dashboard target, checksum gate, or field authorization was generated.",
            "Each path starts near a named access point, but the short access-to-perimeter connector remains unresolved.",
            "Recorded between-area routes are retained in the inventory but are not embedded in these individual missions.",
            "A 12-inch first-run margin is applied around obstacle exclusions that touch an outer perimeter.",
            "Heading-step values are a screening metric; steering feasibility still requires visual review and controller simulation.",
            f"Red X markers identify sampled heading changes of at least {SHARP_TURN_REVIEW_DEG:.0f} degrees that require path smoothing or replacement.",
        ],
    }
    REPORT_OUT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    fig, axis = plt.subplots(figsize=(14, 10), dpi=170)
    colors = {
        "AREA-BACKYARD-GARDENS": "#1565c0",
        "AREA-FRONT-YARD": "#ef6c00",
        "AREA-OVER-ROAD": "#795548",
    }
    for area_id, area in areas.items():
        draw_polygon(axis, area, colors[area_id], alpha=0.07, width=1.0)
    for obstacle_id, obstacle in obstacles.items():
        draw_polygon(axis, obstacle, "#dc2626", alpha=0.24, width=0.8)
        center = obstacle.centroid
        axis.annotate(obstacle_id, (center.x, center.y), fontsize=5.5)
    for area_id, line in plot_paths.items():
        x, y = line.xy
        axis.plot(x, y, color=colors[area_id], linewidth=2.0, label=area_id)
        start = Point(line.coords[0])
        axis.scatter([start.x], [start.y], s=48, color=colors[area_id], edgecolor="white", zorder=5)
        if plot_sharp_points[area_id]:
            sx, sy = zip(*plot_sharp_points[area_id])
            axis.scatter(sx, sy, marker="x", s=55, linewidths=1.8, color="#dc2626", zorder=6)
    axis.scatter([], [], marker="x", s=55, linewidths=1.8, color="#dc2626", label=f"≥{SHARP_TURN_REVIEW_DEG:.0f}° heading-step review")
    axis.set_title("62 Collins separate outer-perimeter missions at 1.0 m/s — REVIEW ONLY")
    axis.set_xlabel("Local east (m)")
    axis.set_ylabel("Local north (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, linewidth=0.3, alpha=0.3)
    axis.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(PNG_OUT, bbox_inches="tight")
    plt.close(fig)

    README_OUT.write_text(
        "# 62 Collins outer-perimeter mission review\n\n"
        "This package contains three separate controller-format outer-perimeter paths at 1.0 m/s. It is for review, not field execution.\n\n"
        "- The revision-3 outer boundary is treated as the desired left deck-edge perimeter. The clockwise GPS centerline is inset by the 0.5334 m deck half-width.\n"
        "- Where an obstacle touches an outer boundary, the path detours around the obstacle exclusion plus the 0.5334 m deck half-width and a 12-inch first-run margin.\n"
        "- Every area runs clockwise so the tractor's left deck edge faces the outer perimeter. Each path starts near its named recorded access point.\n"
        "- Owner review confirms adequate room around OBS-NEW-003, 007, 011, and 012. Their final paths must use broad tangent-connected detours, not the current hard polygon-cut joins.\n"
        f"- Red X markers identify adjacent waypoint heading changes of at least {SHARP_TURN_REVIEW_DEG:.0f} degrees; those corners must be smoothed or replaced before field authorization.\n"
        "- The three files are intentionally separate. Access connectors, between-area sequencing, start-pose checks, and recovery phase gates must be resolved before creating a master mission.\n"
        "- No launcher or dashboard target has been generated.\n\n"
        "Review the PNG and report first. The obstacle-loop mission will be built separately at 0.75 m/s around obstacles and 1.0 m/s on approved connectors.\n",
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps({"output": str(OUT), "areas": records}, indent=2))


if __name__ == "__main__":
    main()
