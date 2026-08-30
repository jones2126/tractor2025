#!/usr/bin/env python3
"""Join a reviewed inward spiral to core stripes with a contained Dubins path."""

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


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build one REVIEW_TEST mission containing an existing inward spiral, "
            "a contained forward-only transition, and keyhole-connected core stripes."
        )
    )
    parser.add_argument("boundary_csv", type=Path)
    parser.add_argument("spiral_mission", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--mission-name", default="62_Collins_combined_review_test_20260830.txt")
    parser.add_argument("--lane-spacing-m", type=float, default=0.9652)
    parser.add_argument("--core-inset-passes", type=int, default=14)
    parser.add_argument("--stripe-angle-deg", type=float, default=0.0)
    parser.add_argument("--turn-radius-m", type=float, default=1.90)
    parser.add_argument("--waypoint-spacing-m", type=float, default=0.50)
    parser.add_argument("--stripe-speed-mps", type=float, default=0.85)
    parser.add_argument("--turn-speed-mps", type=float, default=0.50)
    parser.add_argument("--stripe-lookahead-m", type=float, default=1.0)
    parser.add_argument("--minimum-stripe-m", type=float, default=3.0)
    parser.add_argument("--deck-width-m", type=float, default=1.0668)
    parser.add_argument("--join-min-gap-m", type=float, default=0.01)
    return parser.parse_args()


def local_frame(lat0: float, lon0: float):
    east_scale = 111_320.0 * math.cos(math.radians(lat0))

    def to_xy(lat: float, lon: float) -> tuple[float, float]:
        return (lon - lon0) * east_scale, (lat - lat0) * 110_540.0

    def to_ll(x: float, y: float) -> tuple[float, float]:
        return lat0 + y / 110_540.0, lon0 + x / east_scale

    return to_xy, to_ll


def load_boundary(path: Path):
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) < 3:
        raise ValueError("Reviewed boundary must contain at least three points")
    rows.sort(key=lambda row: float(row["sequence"]))
    lat0 = float(rows[0]["lat"])
    lon0 = float(rows[0]["lon"])
    to_xy, to_ll = local_frame(lat0, lon0)
    boundary = Polygon(
        [to_xy(float(row["lat"]), float(row["lon"])) for row in rows]
    )
    if not boundary.is_valid:
        raise ValueError("Reviewed boundary polygon is invalid")
    return rows, boundary, to_xy, to_ll


def load_mission(path: Path, to_xy):
    rows = []
    for line_number, line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        if not line.strip():
            continue
        values = list(map(float, line.split()))
        if len(values) != 5:
            raise ValueError(f"Mission line {line_number} does not have five columns")
        lat, lon, yaw, lookahead, speed = values
        rows.append(
            {
                "xy": to_xy(lat, lon),
                "source_yaw": yaw,
                "lookahead": lookahead,
                "speed": speed,
            }
        )
    if len(rows) < 2:
        raise ValueError("Spiral mission must contain at least two waypoints")
    return rows


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


def append_segment(route, points, kind, lookahead, speed, minimum_gap):
    added = 0
    for point in points:
        xy = (float(point[0]), float(point[1]))
        if route and math.dist(route[-1]["xy"], xy) < minimum_gap:
            continue
        route.append(
            {
                "xy": xy,
                "kind": kind,
                "lookahead": float(lookahead),
                "speed": float(speed),
            }
        )
        added += 1
    return added


def mission_yaw(route, index):
    if index + 1 < len(route):
        first = route[index]["xy"]
        second = route[index + 1]["xy"]
    else:
        first = route[index - 1]["xy"]
        second = route[index]["xy"]
    return math.atan2(second[1] - first[1], second[0] - first[0])


def main() -> None:
    args = arguments()
    if Path(args.mission_name).name != args.mission_name:
        raise ValueError("--mission-name must be one filename")
    if not args.mission_name.endswith(".txt"):
        raise ValueError("--mission-name must end in .txt")

    planner_dir = (
        Path(__file__).resolve().parents[2]
        / "tractor_rpi"
        / "pure-pursuit"
        / "mission_planning"
    )
    sys.path.insert(0, str(planner_dir))
    import site_coverage_planner_20260724 as planner

    _boundary_rows, boundary, to_xy, to_ll = load_boundary(args.boundary_csv)
    spiral_rows = load_mission(args.spiral_mission, to_xy)
    core = boundary.buffer(
        -args.core_inset_passes * args.lane_spacing_m,
        join_style="round",
    )
    if core.is_empty or core.geom_type != "Polygon":
        raise ValueError("Core geometry is empty or disconnected")

    candidates = planner.make_stripes(
        core,
        args.stripe_angle_deg,
        args.lane_spacing_m,
        0.0,
        "high",
        "reverse",
    )
    stripes = [
        stripe
        for stripe in candidates
        if float(stripe["length_m"]) >= args.minimum_stripe_m
    ]
    excluded = [
        stripe
        for stripe in candidates
        if float(stripe["length_m"]) < args.minimum_stripe_m
    ]
    if not stripes:
        raise ValueError("No included core stripes")

    spiral_points = [row["xy"] for row in spiral_rows]
    spiral_end_pose = planner.path_end_pose(spiral_points)
    first_stripe_pose = planner.path_start_pose(
        [stripes[0]["start"], stripes[0]["end"]]
    )
    transition_candidates = []
    for candidate in planner.dubins_candidates(
        spiral_end_pose,
        first_stripe_pose,
        args.turn_radius_m,
        args.waypoint_spacing_m / 2.0,
    ):
        points = [(float(point[0]), float(point[1])) for point in candidate["points"]]
        line = LineString(points)
        if not boundary.buffer(0.03).covers(line):
            continue
        inside_core_m = line.intersection(core.buffer(-0.03)).length
        transition_candidates.append(
            {
                **candidate,
                "points": points,
                "inside_core_m": float(inside_core_m),
            }
        )
    if not transition_candidates:
        raise ValueError("No forward-only spiral-end transition is contained in the boundary")
    transition = min(
        transition_candidates,
        key=lambda candidate: (candidate["inside_core_m"], candidate["length_m"]),
    )

    connectors = []
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
    for row in spiral_rows:
        append_segment(
            route,
            [row["xy"]],
            "spiral",
            row["lookahead"],
            row["speed"],
            args.join_min_gap_m,
        )
    spiral_waypoints = len(route)
    append_segment(
        route,
        transition["points"],
        "spiral_to_stripe_transition",
        args.stripe_lookahead_m,
        args.turn_speed_mps,
        args.join_min_gap_m,
    )
    transition_end_index = len(route)

    for index, stripe in enumerate(stripes):
        append_segment(
            route,
            sample_line(stripe["start"], stripe["end"], args.waypoint_spacing_m),
            "stripe",
            args.stripe_lookahead_m,
            args.stripe_speed_mps,
            args.join_min_gap_m,
        )
        if index < len(connectors):
            append_segment(
                route,
                connectors[index]["points"],
                "keyhole",
                args.stripe_lookahead_m,
                args.turn_speed_mps,
                args.join_min_gap_m,
            )

    if len(route) < 2:
        raise ValueError("Combined route has fewer than two points")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    mission_path = args.output_dir / args.mission_name
    with mission_path.open("w", encoding="ascii", newline="\n") as handle:
        for index, row in enumerate(route):
            lat, lon = to_ll(*row["xy"])
            yaw = mission_yaw(route, index)
            handle.write(
                f"{lat:.9f} {lon:.9f} {yaw:.6f} "
                f"{row['lookahead']:.2f} {row['speed']:.2f}\n"
            )

    route_line = LineString([row["xy"] for row in route])
    transition_line = LineString(transition["points"])
    outside_m = float(route_line.difference(boundary.buffer(0.03)).length)
    gaps = [
        math.dist(route[index - 1]["xy"], route[index]["xy"])
        for index in range(1, len(route))
    ]
    duplicate_count = sum(gap < args.join_min_gap_m for gap in gaps)
    estimated_seconds = sum(
        gaps[index - 1] / max(float(route[index]["speed"]), 0.01)
        for index in range(1, len(route))
    )
    connector_rows = []
    for index, connector in enumerate(connectors, 1):
        line = LineString([(point[0], point[1]) for point in connector["points"]])
        connector_rows.append(
            {
                "connector": f"stripe_{index}_to_{index + 1}",
                "mode": connector["mode"],
                "length_m": float(connector["length_m"]),
                "contained_in_site": bool(boundary.buffer(0.03).covers(line)),
                "length_inside_core_m": float(line.intersection(core.buffer(-0.03)).length),
            }
        )

    report = {
        "status": "REVIEW_TEST",
        "source_boundary_csv": str(args.boundary_csv.resolve()),
        "source_spiral_mission": str(args.spiral_mission.resolve()),
        "mission_file": str(mission_path.resolve()),
        "core_inset_passes": args.core_inset_passes,
        "core_inset_m": args.core_inset_passes * args.lane_spacing_m,
        "stripe_angle_math_degrees": args.stripe_angle_deg,
        "included_stripes": len(stripes),
        "excluded_short_stripes": len(excluded),
        "excluded_lengths_m": [float(stripe["length_m"]) for stripe in excluded],
        "spiral_waypoints": spiral_waypoints,
        "transition": {
            "mode": transition["mode"],
            "length_m": float(transition["length_m"]),
            "contained_in_site": bool(boundary.buffer(0.03).covers(transition_line)),
            "length_inside_core_m": float(transition["inside_core_m"]),
            "candidate_count": len(transition_candidates),
            "end_route_index_exclusive": transition_end_index,
        },
        "keyhole_connectors": connector_rows,
        "route_contained_in_site": outside_m <= 1e-6,
        "outside_site_length_m": outside_m,
        "join_min_gap_m": args.join_min_gap_m,
        "sub_minimum_consecutive_gaps": duplicate_count,
        "minimum_consecutive_gap_m": min(gaps),
        "maximum_consecutive_gap_m": max(gaps),
        "turn_radius_m": args.turn_radius_m,
        "waypoints": len(route),
        "route_length_m": float(route_line.length),
        "estimated_runtime_minutes": estimated_seconds / 60.0,
        "mission_sha256": hashlib.sha256(mission_path.read_bytes()).hexdigest(),
    }
    report_path = args.output_dir / "combined_review_test_report_20260830.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    bx, by = boundary.exterior.xy
    cx, cy = core.exterior.xy
    fig, ax = plt.subplots(figsize=(11, 10))
    ax.fill(bx, by, color="#e8f5e9", alpha=0.55, label="Reviewed boundary")
    ax.fill(cx, cy, color="#fffde7", edgecolor="#424242", alpha=0.8, label="Core")
    kinds = {
        "spiral": ("#1565c0", 1.3, "14-ring spiral"),
        "spiral_to_stripe_transition": ("#d81b60", 2.2, "Spiral-to-stripe transition"),
        "stripe": ("#2e7d32", 1.5, "Core stripes"),
        "keyhole": ("#ef6c00", 1.2, "Keyhole turns"),
    }
    plotted = set()
    for first, second in zip(route, route[1:]):
        kind = second["kind"]
        color, width, label = kinds[kind]
        ax.plot(
            [first["xy"][0], second["xy"][0]],
            [first["xy"][1], second["xy"][1]],
            color=color,
            linewidth=width,
            label=label if kind not in plotted else None,
        )
        plotted.add(kind)
    ax.scatter(*route[0]["xy"], color="green", marker="*", s=100, label="Mission start")
    ax.scatter(*route[-1]["xy"], color="red", marker="x", s=65, label="Mission end")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("REVIEW_TEST combined 14-ring spiral and 21-stripe mission")
    ax.legend(loc="best")
    fig.tight_layout()
    preview_path = args.output_dir / "combined_review_test_preview_20260830.png"
    fig.savefig(preview_path, dpi=180)
    plt.close(fig)

    print(f"Mission : {mission_path}")
    print(f"Preview : {preview_path}")
    print(f"Report  : {report_path}")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
