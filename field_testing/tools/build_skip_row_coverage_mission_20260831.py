#!/usr/bin/env python3
"""Build a combined spiral plus gentle four-row-skip stripe mission."""

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


CSC_MODES = {"LSL", "RSR", "LSR", "RSL"}


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build a 14-ring mission whose core stripes advance four rows at a "
            "time, allowing simple two-arc turns instead of three-arc keyholes."
        )
    )
    parser.add_argument("boundary_csv", type=Path)
    parser.add_argument("spiral_mission", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--mission-name", default="62_Collins_combined_14ring_four_row_skip_REVIEW_TEST_20260831.txt")
    parser.add_argument("--lane-spacing-m", type=float, default=0.9652)
    parser.add_argument("--core-inset-passes", type=int, default=14)
    parser.add_argument("--stripe-step", type=int, default=4)
    parser.add_argument("--turn-radius-m", type=float, default=1.90)
    parser.add_argument("--waypoint-spacing-m", type=float, default=0.50)
    parser.add_argument("--spiral-speed-mps", type=float, default=1.25)
    parser.add_argument("--stripe-speed-mps", type=float, default=1.25)
    parser.add_argument("--turn-speed-mps", type=float, default=1.25)
    parser.add_argument("--spiral-lookahead-m", type=float, default=None)
    parser.add_argument("--stripe-lookahead-m", type=float, default=2.0)
    parser.add_argument("--turn-lookahead-m", type=float, default=1.5)
    parser.add_argument("--minimum-stripe-m", type=float, default=3.0)
    parser.add_argument("--join-min-gap-m", type=float, default=0.01)
    parser.add_argument(
        "--spiral-mode",
        choices=("full", "last-ring-from-original-start"),
        default="full",
        help=(
            "full keeps all 14 revolutions; last-ring-from-original-start adds "
            "a contained transit from the original start pose and keeps only "
            "the final revolution"
        ),
    )
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
    rows.sort(key=lambda row: float(row["sequence"]))
    lat0, lon0 = float(rows[0]["lat"]), float(rows[0]["lon"])
    to_xy, to_ll = local_frame(lat0, lon0)
    boundary = Polygon([to_xy(float(row["lat"]), float(row["lon"])) for row in rows])
    if not boundary.is_valid:
        boundary = boundary.buffer(0)
    if boundary.geom_type != "Polygon":
        raise ValueError("Reviewed boundary is not one polygon")
    return boundary, to_xy, to_ll


def load_mission(path: Path, to_xy):
    rows = []
    for line_number, line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        if not line.strip():
            continue
        values = list(map(float, line.split()))
        if len(values) != 5:
            raise ValueError(f"Mission line {line_number} does not have five columns")
        lat, lon, yaw, lookahead, speed = values
        rows.append({"xy": to_xy(lat, lon), "lookahead": lookahead, "speed": speed})
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


def append_segment(route, points, kind, label, lookahead, speed, minimum_gap):
    start_index = len(route)
    for point in points:
        xy = float(point[0]), float(point[1])
        if route and math.dist(route[-1]["xy"], xy) < minimum_gap:
            continue
        route.append({
            "xy": xy,
            "kind": kind,
            "label": label,
            "lookahead": float(lookahead),
            "speed": float(speed),
        })
    return start_index, len(route)


def mission_yaw(route, index):
    if index + 1 < len(route):
        first, second = route[index]["xy"], route[index + 1]["xy"]
    else:
        first, second = route[index - 1]["xy"], route[index]["xy"]
    return math.atan2(second[1] - first[1], second[0] - first[0])


def modular_order(count: int, step: int):
    if math.gcd(count, step) != 1:
        raise ValueError(f"stripe step {step} does not visit all {count} stripes")
    result = []
    current = 0
    for _ in range(count):
        result.append(current)
        current = (current + step) % count
    return result


def last_revolution_start_index(points):
    """Locate the start of revolution 14 by unwrapping angle around the route centroid."""
    center_x = sum(point[0] for point in points) / len(points)
    center_y = sum(point[1] for point in points) / len(points)
    angles = [math.atan2(point[1] - center_y, point[0] - center_x) for point in points]
    unwrapped = [angles[0]]
    for previous, current in zip(angles, angles[1:]):
        change = (current - previous + math.pi) % (2.0 * math.pi) - math.pi
        unwrapped.append(unwrapped[-1] + change)
    clockwise_turns = (unwrapped[-1] - unwrapped[0]) / (2.0 * math.pi)
    if not (-14.5 < clockwise_turns < -13.5):
        raise ValueError(
            f"Expected approximately 14 clockwise revolutions, found {clockwise_turns:.3f}"
        )
    target_angle = unwrapped[0] - 13.0 * 2.0 * math.pi
    return min(range(len(points)), key=lambda index: abs(unwrapped[index] - target_angle))


def choose_csc_connector(planner, start_pose, end_pose, boundary, core, radius, spacing):
    candidates = []
    for candidate in planner.dubins_candidates(start_pose, end_pose, radius, spacing):
        if candidate["mode"] not in CSC_MODES:
            continue
        points = [(float(point[0]), float(point[1])) for point in candidate["points"]]
        line = LineString(points)
        if boundary.buffer(0.03).covers(line):
            candidates.append({
                **candidate,
                "points": points,
                "length_inside_core_m": float(line.intersection(core.buffer(-0.03)).length),
            })
    if not candidates:
        return None
    # Prefer the already-cut spiral/headland area.  This is especially important
    # for the three modulo-wrap relocations, where the shortest Dubins path would
    # otherwise draw a diagonal across the core.
    return min(
        candidates,
        key=lambda candidate: (candidate["length_inside_core_m"], candidate["length_m"]),
    )


def main() -> None:
    args = arguments()
    planner_dir = Path(__file__).resolve().parents[2] / "tractor_rpi" / "pure-pursuit" / "mission_planning"
    sys.path.insert(0, str(planner_dir))
    import site_coverage_planner_20260724 as planner

    boundary, to_xy, to_ll = load_boundary(args.boundary_csv)
    full_spiral_rows = load_mission(args.spiral_mission, to_xy)
    spiral_rows = full_spiral_rows
    last_ring_start_index = 0
    start_transit = None
    if args.spiral_mode == "last-ring-from-original-start":
        full_spiral_points = [row["xy"] for row in full_spiral_rows]
        last_ring_start_index = last_revolution_start_index(full_spiral_points)
        if last_ring_start_index + 1 >= len(full_spiral_rows):
            raise ValueError("Final revolution start is too close to the mission endpoint")
        spiral_rows = full_spiral_rows[last_ring_start_index:]
        original_start_pose = planner.path_start_pose(full_spiral_points[:2])
        last_ring_start_pose = planner.path_start_pose([row["xy"] for row in spiral_rows[:2]])
        start_candidates = []
        for candidate in planner.dubins_candidates(
            original_start_pose,
            last_ring_start_pose,
            args.turn_radius_m,
            args.waypoint_spacing_m / 2.0,
        ):
            points = [(float(point[0]), float(point[1])) for point in candidate["points"]]
            line = LineString(points)
            if boundary.buffer(0.03).covers(line):
                start_candidates.append({**candidate, "points": points})
        if not start_candidates:
            raise ValueError("No contained transit from the original start to ring 14")
        start_transit = min(start_candidates, key=lambda candidate: candidate["length_m"])
    core = boundary.buffer(-args.core_inset_passes * args.lane_spacing_m, join_style="round")
    if core.is_empty or core.geom_type != "Polygon":
        raise ValueError("Core geometry is empty or disconnected")

    generated = planner.make_stripes(core, 0.0, args.lane_spacing_m, 0.0, "high", "reverse")
    generated = [stripe for stripe in generated if float(stripe["length_m"]) >= args.minimum_stripe_m]
    order = modular_order(len(generated), args.stripe_step)

    # Reorient each visited stripe so consecutive rows alternate travel direction.
    stripes = []
    for route_position, source_index in enumerate(order):
        source = generated[source_index]
        west = min((source["start"], source["end"]), key=lambda point: point[0])
        east = max((source["start"], source["end"]), key=lambda point: point[0])
        start, end = (east, west) if route_position % 2 == 0 else (west, east)
        stripes.append({
            "source_index": source_index,
            "row_number": source_index + 1,
            "start": start,
            "end": end,
            "length_m": math.dist(start, end),
        })

    transition_candidates = []
    spiral_pose = planner.path_end_pose([row["xy"] for row in spiral_rows])
    first_pose = planner.path_start_pose([stripes[0]["start"], stripes[0]["end"]])
    for candidate in planner.dubins_candidates(spiral_pose, first_pose, args.turn_radius_m, args.waypoint_spacing_m / 2.0):
        points = [(float(point[0]), float(point[1])) for point in candidate["points"]]
        line = LineString(points)
        if boundary.buffer(0.03).covers(line):
            transition_candidates.append({**candidate, "points": points})
    if not transition_candidates:
        raise ValueError("No contained spiral-to-first-stripe transition")
    transition = min(transition_candidates, key=lambda candidate: candidate["length_m"])

    connectors = []
    for outgoing, incoming in zip(stripes, stripes[1:]):
        connector = choose_csc_connector(
            planner,
            planner.path_end_pose([outgoing["start"], outgoing["end"]]),
            planner.path_start_pose([incoming["start"], incoming["end"]]),
            boundary,
            core,
            args.turn_radius_m,
            args.waypoint_spacing_m,
        )
        if connector is None:
            raise ValueError(
                f"No contained two-arc connector from row {outgoing['row_number']} "
                f"to row {incoming['row_number']}"
            )
        connectors.append(connector)

    route = []
    audit = []
    if start_transit is not None:
        start, end = append_segment(
            route,
            start_transit["points"],
            "start_transit",
            "original_start_to_ring_14",
            args.turn_lookahead_m,
            args.turn_speed_mps,
            args.join_min_gap_m,
        )
        audit.append({
            "kind": "start_transit",
            "label": "original_start_to_ring_14",
            "mode": start_transit["mode"],
            "waypoint_start": start,
            "waypoint_end": end - 1,
        })
    spiral_start = len(route)
    for row in spiral_rows:
        append_segment(
            route,
            [row["xy"]],
            "last_ring" if start_transit is not None else "spiral",
            "ring_14" if start_transit is not None else "14-ring spiral",
            args.spiral_lookahead_m if args.spiral_lookahead_m is not None else row["lookahead"],
            args.spiral_speed_mps,
            args.join_min_gap_m,
        )
    audit.append({
        "kind": "last_ring" if start_transit is not None else "spiral",
        "label": "ring_14" if start_transit is not None else "14-ring spiral",
        "waypoint_start": spiral_start,
        "waypoint_end": len(route) - 1,
    })
    start, end = append_segment(
        route,
        transition["points"],
        "transition",
        "spiral_to_row_1",
        args.turn_lookahead_m,
        args.turn_speed_mps,
        args.join_min_gap_m,
    )
    audit.append({"kind": "transition", "label": "spiral_to_row_1", "mode": transition["mode"], "waypoint_start": start, "waypoint_end": end - 1})

    for index, stripe in enumerate(stripes):
        start, end = append_segment(
            route,
            sample_line(stripe["start"], stripe["end"], args.waypoint_spacing_m),
            "stripe",
            f"row_{stripe['row_number']}",
            args.stripe_lookahead_m,
            args.stripe_speed_mps,
            args.join_min_gap_m,
        )
        audit.append({"kind": "stripe", "label": f"row_{stripe['row_number']}", "waypoint_start": start, "waypoint_end": end - 1})
        if index < len(connectors):
            incoming = stripes[index + 1]
            connector = connectors[index]
            label = f"row_{stripe['row_number']}_to_{incoming['row_number']}"
            start, end = append_segment(
                route,
                connector["points"],
                "gentle_turn",
                label,
                args.turn_lookahead_m,
                args.turn_speed_mps,
                args.join_min_gap_m,
            )
            audit.append({"kind": "gentle_turn", "label": label, "mode": connector["mode"], "waypoint_start": start, "waypoint_end": end - 1})

    args.output_dir.mkdir(parents=True, exist_ok=True)
    mission_path = args.output_dir / args.mission_name
    with mission_path.open("w", encoding="ascii", newline="\n") as handle:
        for index, row in enumerate(route):
            lat, lon = to_ll(*row["xy"])
            handle.write(
                f"{lat:.9f} {lon:.9f} {mission_yaw(route, index):.6f} "
                f"{row['lookahead']:.2f} {row['speed']:.2f}\n"
            )

    route_line = LineString([row["xy"] for row in route])
    gaps = [math.dist(route[index - 1]["xy"], route[index]["xy"]) for index in range(1, len(route))]
    connector_rows = []
    for index, connector in enumerate(connectors):
        outgoing, incoming = stripes[index], stripes[index + 1]
        line = LineString(connector["points"])
        connector_rows.append({
            "from_row": outgoing["row_number"],
            "to_row": incoming["row_number"],
            "rows_advanced_modulo": (incoming["source_index"] - outgoing["source_index"]) % len(stripes),
            "mode": connector["mode"],
            "length_m": float(connector["length_m"]),
            "length_inside_core_m": float(connector["length_inside_core_m"]),
            "contained_in_site": bool(boundary.buffer(0.03).covers(line)),
        })
    report = {
        "status": "REVIEW_TEST",
        "purpose": "Replace adjacent-row keyholes with contained two-arc skip-row turns",
        "source_boundary_csv": str(args.boundary_csv.resolve()),
        "source_spiral_mission": str(args.spiral_mission.resolve()),
        "mission_file": str(mission_path.resolve()),
        "spiral_mode": args.spiral_mode,
        "source_spiral_waypoints": len(full_spiral_rows),
        "included_spiral_waypoints": len(spiral_rows),
        "last_ring_source_start_index": last_ring_start_index,
        "start_transit": (
            {
                "mode": start_transit["mode"],
                "length_m": float(start_transit["length_m"]),
                "contained_in_site": bool(
                    boundary.buffer(0.03).covers(LineString(start_transit["points"]))
                ),
            }
            if start_transit is not None
            else None
        ),
        "stripe_count": len(stripes),
        "stripe_visit_order": [stripe["row_number"] for stripe in stripes],
        "stripe_step": args.stripe_step,
        "lane_spacing_m": args.lane_spacing_m,
        "nominal_row_jump_m": args.stripe_step * args.lane_spacing_m,
        "turn_radius_m": args.turn_radius_m,
        "turn_lookahead_m": args.turn_lookahead_m,
        "stripe_lookahead_m": args.stripe_lookahead_m,
        "all_stripe_connectors_are_two_arc_csc": all(row["mode"] in CSC_MODES for row in connector_rows),
        "connectors": connector_rows,
        "route_contained_in_site": route_line.difference(boundary.buffer(0.03)).length <= 1e-6,
        "outside_site_length_m": float(route_line.difference(boundary.buffer(0.03)).length),
        "minimum_consecutive_gap_m": min(gaps),
        "maximum_consecutive_gap_m": max(gaps),
        "waypoints": len(route),
        "route_length_m": float(route_line.length),
        "spiral_speed_mps": args.spiral_speed_mps,
        "stripe_speed_mps": args.stripe_speed_mps,
        "turn_speed_mps": args.turn_speed_mps,
        "mission_sha256": hashlib.sha256(mission_path.read_bytes()).hexdigest(),
    }
    report_path = args.output_dir / "four_row_skip_report_20260831.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    audit_path = args.output_dir / (
        "last_ring_four_row_skip_waypoint_audit_20260831.csv"
        if start_transit is not None
        else "four_row_skip_waypoint_audit_20260831.csv"
    )
    with audit_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["kind", "label", "mode", "waypoint_start", "waypoint_end"])
        writer.writeheader()
        for row in audit:
            writer.writerow(row)

    bx, by = boundary.exterior.xy
    cx, cy = core.exterior.xy
    fig, ax = plt.subplots(figsize=(11, 10))
    ax.fill(bx, by, color="#e8f5e9", alpha=0.55, label="Reviewed boundary")
    ax.fill(cx, cy, color="#fffde7", edgecolor="#424242", alpha=0.8, label="Inner core")
    styles = {
        "spiral": ("#90a4ae", 0.8, "14-ring spiral"),
        "start_transit": ("#6a1b9a", 2.0, "Original start to ring 14"),
        "last_ring": ("#546e7a", 1.8, "Ring 14 only"),
        "transition": ("#d81b60", 2.0, "Spiral-to-stripe transition"),
        "stripe": ("#2e7d32", 1.7, "Stripes in visit order"),
        "gentle_turn": ("#1565c0", 1.6, "Two-arc skip-row turns"),
    }
    plotted = set()
    for first, second in zip(route, route[1:]):
        kind = second["kind"]
        color, width, label = styles[kind]
        ax.plot(
            [first["xy"][0], second["xy"][0]],
            [first["xy"][1], second["xy"][1]],
            color=color,
            linewidth=width,
            label=label if kind not in plotted else None,
        )
        plotted.add(kind)
    for position, stripe in enumerate(stripes, 1):
        midpoint = ((stripe["start"][0] + stripe["end"][0]) / 2.0, (stripe["start"][1] + stripe["end"][1]) / 2.0)
        ax.text(midpoint[0], midpoint[1], str(position), fontsize=7, ha="center", va="center")
    ax.scatter(*route[0]["xy"], color="green", marker="*", s=100, label="Mission start")
    ax.scatter(*route[-1]["xy"], color="red", marker="x", s=60, label="Mission end")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(
        (
            "Original start + ring 14 + four-row-skip core coverage"
            if start_transit is not None
            else "14 rings + four-row-skip core coverage (numbers show visit order)"
        )
    )
    ax.legend(loc="best")
    fig.tight_layout()
    preview_path = args.output_dir / "four_row_skip_preview_20260831.png"
    fig.savefig(preview_path, dpi=180)
    plt.close(fig)

    print(f"Mission : {mission_path}")
    print(f"Preview : {preview_path}")
    print(f"Report  : {report_path}")
    print(f"Audit   : {audit_path}")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
