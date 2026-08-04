#!/usr/bin/env python3
"""Fill the core left by the clockwise spiral with east/west stripes."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("boundary_log", type=Path)
    parser.add_argument("spiral_mission", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--start-row", type=int, default=3734)
    parser.add_argument("--end-row", type=int, default=8686)
    parser.add_argument("--lane-spacing-m", type=float, default=0.9652)
    parser.add_argument("--core-inset-passes", type=int, default=14)
    parser.add_argument("--turn-radius-m", type=float, default=1.90)
    parser.add_argument("--waypoint-spacing-m", type=float, default=0.50)
    parser.add_argument("--straight-speed-mps", type=float, default=0.85)
    parser.add_argument("--turn-speed-mps", type=float, default=0.50)
    parser.add_argument("--lookahead-m", type=float, default=2.0)
    parser.add_argument("--minimum-stripe-m", type=float, default=3.0)
    parser.add_argument("--deck-width-m", type=float, default=1.0668)
    return parser.parse_args()


def frame(rows):
    lat0 = float(rows[0]["lat"])
    lon0 = float(rows[0]["lon"])
    east_scale = 111_320.0 * math.cos(math.radians(lat0))

    def xy(lat, lon):
        return (lon - lon0) * east_scale, (lat - lat0) * 110_540.0

    def ll(x, y):
        return lat0 + y / 110_540.0, lon0 + x / east_scale

    return lat0, lon0, xy, ll


def sample_line(start, end, spacing):
    length = math.dist(start, end)
    count = max(2, math.ceil(length / spacing) + 1)
    return [
        (
            start[0] + (end[0] - start[0]) * index / (count - 1),
            start[1] + (end[1] - start[1]) * index / (count - 1),
        )
        for index in range(count)
    ]


def append_segment(route, kinds, points, kind):
    xy_points = [(float(point[0]), float(point[1])) for point in points]
    if route and math.dist(route[-1], xy_points[0]) < 1e-6:
        xy_points = xy_points[1:]
    route.extend(xy_points)
    kinds.extend([kind] * len(xy_points))


def main():
    args = arguments()
    planner_dir = Path(__file__).resolve().parents[2] / "tractor_rpi" / "pure-pursuit" / "mission_planning"
    sys.path.insert(0, str(planner_dir))
    import site_coverage_planner_20260724 as planner

    with args.boundary_log.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))[args.start_row - 1 : args.end_row]
    rows = [row for row in rows if row.get("lat") and row.get("lon")]
    lat0, lon0, to_xy, to_ll = frame(rows)
    boundary = Polygon([
        to_xy(float(row["lat"]), float(row["lon"])) for row in rows
    ]).simplify(0.10, preserve_topology=True)
    core = boundary.buffer(
        -args.core_inset_passes * args.lane_spacing_m,
        join_style="round",
    )
    if core.is_empty or core.geom_type != "Polygon":
        raise ValueError("Core geometry is empty or disconnected")

    candidates = planner.make_stripes(
        core,
        0.0,
        args.lane_spacing_m,
        0.0,
        "high",
        "reverse",
    )
    stripes = [
        stripe for stripe in candidates
        if float(stripe["length_m"]) >= args.minimum_stripe_m
    ]
    excluded = [
        stripe for stripe in candidates
        if float(stripe["length_m"]) < args.minimum_stripe_m
    ]
    if not stripes:
        raise ValueError("No included stripes")

    spiral_rows = [
        list(map(float, line.split()))
        for line in args.spiral_mission.read_text(encoding="ascii").splitlines()
        if line.strip()
    ]
    spiral_start = [
        to_xy(row[0], row[1]) for row in spiral_rows[:2]
    ]
    start_pose = planner.path_start_pose(spiral_start)
    first_stripe_pose = planner.path_start_pose(
        [stripes[0]["start"], stripes[0]["end"]]
    )
    initial_candidates = []
    for candidate in planner.dubins_candidates(
        start_pose,
        first_stripe_pose,
        args.turn_radius_m,
        args.waypoint_spacing_m / 2.0,
    ):
        line = LineString([(point[0], point[1]) for point in candidate["points"]])
        if not boundary.buffer(0.03).covers(line):
            continue
        if line.intersection(core.buffer(-0.03)).length > 1e-6:
            continue
        initial_candidates.append(candidate)
    if not initial_candidates:
        raise ValueError("No contained outer-start transit outside the core")
    initial = min(initial_candidates, key=lambda candidate: candidate["length_m"])
    initial = {**initial, "mode": f"{initial['mode']}-outer-start-transit"}

    connectors = [initial]
    for outgoing, incoming in zip(stripes, stripes[1:]):
        connector = planner.keyhole_connector(
            planner.path_end_pose([outgoing["start"], outgoing["end"]]),
            planner.path_start_pose([incoming["start"], incoming["end"]]),
            boundary,
            args.turn_radius_m,
            args.waypoint_spacing_m,
        )
        if connector is None or not str(connector["mode"]).endswith("-keyhole"):
            raise ValueError("A preferred contained keyhole connector was not found")
        connectors.append(connector)

    route = []
    kinds = []
    append_segment(route, kinds, connectors[0]["points"], "turn")
    for index, stripe in enumerate(stripes):
        append_segment(
            route,
            kinds,
            sample_line(stripe["start"], stripe["end"], args.waypoint_spacing_m),
            "straight",
        )
        if index + 1 < len(stripes):
            append_segment(route, kinds, connectors[index + 1]["points"], "turn")

    mission = args.output_dir / "polygon_1_inner_stripes_38in_20260804.txt"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with mission.open("w", encoding="ascii", newline="\n") as handle:
        for index, point in enumerate(route):
            neighbor = route[index + 1] if index + 1 < len(route) else route[index - 1]
            if index == 0:
                yaw = start_pose[2]
            elif index + 1 < len(route):
                yaw = math.atan2(neighbor[1] - point[1], neighbor[0] - point[0])
            else:
                yaw = math.atan2(point[1] - neighbor[1], point[0] - neighbor[0])
            lat, lon = to_ll(*point)
            speed = args.turn_speed_mps if kinds[index] == "turn" else args.straight_speed_mps
            handle.write(
                f"{lat:.9f} {lon:.9f} {yaw:.6f} {args.lookahead_m:.2f} {speed:.2f}\n"
            )

    route_line = LineString(route)
    connector_rows = []
    all_connectors_outside_core = True
    for index, connector in enumerate(connectors):
        line = LineString([(point[0], point[1]) for point in connector["points"]])
        inside_core_m = line.intersection(core.buffer(-0.03)).length
        contained = boundary.buffer(0.03).covers(line)
        all_connectors_outside_core &= inside_core_m <= 1e-6
        connector_rows.append({
            "connector": "outer_start_to_stripe_1" if index == 0 else f"stripe_{index}_to_{index + 1}",
            "mode": connector["mode"],
            "length_m": float(connector["length_m"]),
            "contained_in_site": contained,
            "length_inside_core_m": inside_core_m,
        })
    outside_m = route_line.difference(boundary.buffer(0.03)).length
    deck_swath = route_line.buffer(args.deck_width_m / 2.0, cap_style="round", join_style="round")
    uncut = core.difference(deck_swath)
    speeds = [args.turn_speed_mps if kind == "turn" else args.straight_speed_mps for kind in kinds]
    estimated_seconds = sum(
        math.dist(route[index - 1], route[index]) / max(speeds[index], 0.01)
        for index in range(1, len(route))
    )
    start_lat, start_lon = to_ll(*route[0])
    start_heading = (90.0 - math.degrees(start_pose[2])) % 360.0
    end_lat, end_lon = to_ll(*route[-1])
    report = {
        "source_boundary_log": str(args.boundary_log.resolve()),
        "source_spiral_mission": str(args.spiral_mission.resolve()),
        "core_inset_passes": args.core_inset_passes,
        "core_inset_m": args.core_inset_passes * args.lane_spacing_m,
        "stripe_angle_math_degrees": 0.0,
        "lane_spacing_m": args.lane_spacing_m,
        "included_stripes": len(stripes),
        "excluded_short_stripes": len(excluded),
        "excluded_lengths_m": [float(stripe["length_m"]) for stripe in excluded],
        "connectors": connector_rows,
        "all_connectors_outside_core": all_connectors_outside_core,
        "route_contained_in_site": outside_m <= 1e-6,
        "outside_site_length_m": outside_m,
        "turn_radius_m": args.turn_radius_m,
        "straight_speed_mps": args.straight_speed_mps,
        "turn_speed_mps": args.turn_speed_mps,
        "lookahead_m": args.lookahead_m,
        "waypoints": len(route),
        "route_length_m": route_line.length,
        "estimated_runtime_minutes": estimated_seconds / 60.0,
        "core_area_m2": core.area,
        "presumed_core_uncut_area_m2": uncut.area,
        "presumed_core_uncut_percent": 100.0 * uncut.area / core.area,
        "start_lat": start_lat,
        "start_lon": start_lon,
        "start_heading_compass_deg": start_heading,
        "end_lat": end_lat,
        "end_lon": end_lon,
        "mission_sha256": hashlib.sha256(mission.read_bytes()).hexdigest(),
    }
    report_path = args.output_dir / "polygon_1_inner_stripes_38in_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    fig, ax = plt.subplots(figsize=(11, 10))
    bx, by = boundary.exterior.xy
    cx, cy = core.exterior.xy
    ax.fill(bx, by, color="#e8f5e9", alpha=0.55, label="Previously cut spiral area")
    ax.fill(cx, cy, color="#fffde7", edgecolor="#424242", alpha=0.9, label="Inner core edge")
    for index, stripe in enumerate(stripes):
        sx = [stripe["start"][0], stripe["end"][0]]
        sy = [stripe["start"][1], stripe["end"][1]]
        ax.plot(sx, sy, color="#1565c0", linewidth=1.8,
                label="East/west strips" if index == 0 else None)
    for index, connector in enumerate(connectors):
        xy = [(point[0], point[1]) for point in connector["points"]]
        ax.plot([point[0] for point in xy], [point[1] for point in xy],
                color="#ef6c00", linewidth=1.4,
                label="Keyholes in cut area" if index == 0 else None)
    ax.scatter([route[0][0]], [route[0][1]], color="green", marker="*", s=90, label="Start")
    ax.scatter([route[-1][0]], [route[-1][1]], color="red", marker="x", s=55, label="End")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("East of original boundary start (m)")
    ax.set_ylabel("North of original boundary start (m)")
    ax.set_title("Inner-core strip completion\nTurns begin at the core edge and remain in cut space")
    ax.legend(loc="best")
    fig.tight_layout()
    preview = args.output_dir / "polygon_1_inner_stripes_38in_preview.png"
    fig.savefig(preview, dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.fill(cx, cy, color="#fafafa", edgecolor="#212121", label="Inner core")
    clipped = deck_swath.intersection(core)
    parts = [clipped] if clipped.geom_type == "Polygon" else list(clipped.geoms)
    for index, part in enumerate(parts):
        px, py = part.exterior.xy
        ax.fill(px, py, color="#64b5f6", alpha=0.65,
                label="42 in deck swath" if index == 0 else None)
    if not uncut.is_empty:
        parts = [uncut] if uncut.geom_type == "Polygon" else list(uncut.geoms)
        for index, part in enumerate(parts):
            ux, uy = part.exterior.xy
            ax.fill(ux, uy, color="#ef5350", alpha=0.8,
                    label="Presumed uncut" if index == 0 else None)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(f"Inner-core presumed coverage - remaining {uncut.area:.1f} m2")
    ax.legend(loc="best")
    fig.tight_layout()
    coverage = args.output_dir / "polygon_1_inner_stripes_38in_deck_coverage.png"
    fig.savefig(coverage, dpi=180)
    plt.close(fig)

    print(f"Mission : {mission}")
    print(f"Preview : {preview}")
    print(f"Coverage: {coverage}")
    print(f"Report  : {report_path}")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
