#!/usr/bin/env python3
"""Independent pre-field audit for a five-column Pure Pursuit mission.

The validator can run with only a mission file (format, spacing, speed, yaw and
sampled-curvature checks).  Supplying ``--settings`` also reconstructs the
boundary/obstacle clearances and verifies that the complete route is contained
in the same safe drive polygon used by the planner.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from site_planner_common_20260724 import (
    LocalFrame,
    circumcircle_radius,
    distance,
    load_boundary,
    load_obstacles,
    normalize_angle,
    polyline_length,
    read_json,
    read_mission,
    require_matplotlib,
    require_shapely,
    segment_yaw,
    signed_angle_delta,
    write_json,
)


def plot_setup():
    require_matplotlib()
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    return plt


def polygon_parts(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == "Polygon":
        return [geometry]
    if geometry.geom_type == "MultiPolygon":
        return list(geometry.geoms)
    if geometry.geom_type == "GeometryCollection":
        parts = []
        for child in geometry.geoms:
            parts.extend(polygon_parts(child))
        return parts
    return []


def draw_polygon(ax, geometry, facecolor, edgecolor, alpha, label=None):
    first = True
    for polygon in polygon_parts(geometry):
        x, y = polygon.exterior.xy
        ax.fill(
            x,
            y,
            facecolor=facecolor,
            edgecolor=edgecolor,
            alpha=alpha,
            label=label if first else None,
        )
        first = False
        for interior in polygon.interiors:
            hx, hy = interior.xy
            ax.fill(hx, hy, facecolor="white", edgecolor=edgecolor, alpha=1.0)


def safe_area_from_settings(settings: dict[str, object]):
    require_shapely()
    from shapely.geometry import Point, Polygon

    frame = LocalFrame.from_dict(settings["frame"])
    frame, boundary_xy, _rows = load_boundary(
        str(settings["boundary_file"]), frame=frame
    )
    boundary = Polygon(boundary_xy)
    safe_area = boundary.buffer(
        -float(settings["boundary_clearance_m"]),
        join_style="round",
    )
    outer_headland_follows_boundary = bool(
        settings.get("outer_headland_follows_boundary", False)
    )
    containment_area = boundary if outer_headland_follows_boundary else safe_area
    obstacle_shapes = []
    obstacle_file = settings.get("obstacles_file")
    for obstacle in load_obstacles(str(obstacle_file) if obstacle_file else None):
        center = frame.to_xy(obstacle.lat, obstacle.lon)
        shape = Point(*center).buffer(obstacle.exclusion_radius_m, resolution=32)
        obstacle_shapes.append((obstacle, shape))
        safe_area = safe_area.difference(shape)
        containment_area = containment_area.difference(shape)
    return frame, boundary, containment_area, obstacle_shapes


def run_validation(args: argparse.Namespace) -> int:
    mission = read_mission(args.mission)
    errors: list[str] = []
    warnings: list[str] = []

    settings = read_json(args.settings) if args.settings else None
    boundary = safe_area = obstacle_shapes = None
    if settings:
        frame, boundary, safe_area, obstacle_shapes = safe_area_from_settings(settings)
        configured_turn_radius = float(settings["turn_radius_m"])
        configured_spacing = float(settings["waypoint_spacing_m"])
    else:
        frame = LocalFrame(mission[0].lat, mission[0].lon)
        configured_turn_radius = args.turn_radius_m
        configured_spacing = args.expected_spacing_m

    xy = [frame.to_xy(point.lat, point.lon) for point in mission]
    gaps = [distance(first, second) for first, second in zip(xy, xy[1:])]
    route_length = polyline_length(xy)
    maximum_gap = max(gaps, default=0.0)
    duplicate_count = sum(gap < 0.01 for gap in gaps)

    if len(mission) < 2:
        errors.append("Mission needs at least two waypoints")
    if duplicate_count:
        warnings.append(f"{duplicate_count} consecutive waypoint gaps are under 1 cm")
    gap_limit = args.max_gap_m
    if gap_limit is None:
        gap_limit = configured_spacing * 1.60
    if maximum_gap > gap_limit:
        errors.append(
            f"Maximum waypoint gap {maximum_gap:.2f} m exceeds {gap_limit:.2f} m"
        )

    maximum_speed = max(point.speed_mps for point in mission)
    minimum_speed = min(point.speed_mps for point in mission)
    minimum_lookahead = min(point.lookahead_m for point in mission)
    if maximum_speed > args.max_speed_mps:
        errors.append(
            f"Mission speed {maximum_speed:.2f} m/s exceeds validator cap "
            f"{args.max_speed_mps:.2f} m/s"
        )
    if minimum_speed < 0:
        errors.append("Mission contains a negative speed; controller clamps it to zero")
    if minimum_lookahead <= 0:
        errors.append("Every mission lookahead must be positive")

    yaw_errors = []
    for index in range(len(mission) - 1):
        actual_yaw = segment_yaw(xy[index], xy[index + 1])
        yaw_errors.append(
            abs(signed_angle_delta(mission[index].yaw_rad, actual_yaw))
        )
    maximum_yaw_error_deg = math.degrees(max(yaw_errors, default=0.0))
    if maximum_yaw_error_deg > args.yaw_warning_degrees:
        warnings.append(
            f"Maximum stored-yaw vs next-segment difference is "
            f"{maximum_yaw_error_deg:.1f}° (intermediate yaw is currently "
            "ignored by the controller, but this can reveal a bad export)"
        )

    radii = [math.inf] * len(xy)
    for index in range(1, len(xy) - 1):
        radii[index] = circumcircle_radius(xy[index - 1], xy[index], xy[index + 1])
    finite_radii = [radius for radius in radii if math.isfinite(radius)]
    minimum_sampled_radius = min(finite_radii, default=math.inf)
    if minimum_sampled_radius < configured_turn_radius * args.radius_warning_fraction:
        message = (
            f"Sampled path radius {minimum_sampled_radius:.2f} m is below "
            f"{args.radius_warning_fraction:.0%} of configured "
            f"{configured_turn_radius:.2f} m. Inspect the marked corner."
        )
        if args.strict_curvature:
            errors.append(message)
        else:
            warnings.append(message)

    minimum_safe_edge_distance = None
    if safe_area is not None:
        from shapely.geometry import LineString

        route_line = LineString(xy)
        if not safe_area.buffer(args.geometry_tolerance_m).covers(route_line):
            errors.append(
                "Mission route is not fully contained in the reconstructed safe "
                "drive area"
            )
        minimum_safe_edge_distance = route_line.distance(safe_area.boundary)

    report_path = Path(args.report) if args.report else Path(args.mission).with_name(
        f"{Path(args.mission).stem}_validation.json"
    )
    plot_path = Path(args.plot) if args.plot else Path(args.mission).with_name(
        f"{Path(args.mission).stem}_validation.png"
    )
    write_json(
        report_path,
        {
            "mission_file": str(Path(args.mission).resolve()),
            "settings_file": (
                str(Path(args.settings).resolve()) if args.settings else None
            ),
            "status": "FAIL" if errors else ("REVIEW" if warnings else "PASS"),
            "errors": errors,
            "warnings": warnings,
            "waypoints": len(mission),
            "route_length_m": route_length,
            "maximum_gap_m": maximum_gap,
            "gap_limit_m": gap_limit,
            "duplicate_consecutive_points": duplicate_count,
            "minimum_speed_mps": minimum_speed,
            "maximum_speed_mps": maximum_speed,
            "minimum_lookahead_m": minimum_lookahead,
            "maximum_yaw_error_degrees": maximum_yaw_error_deg,
            "minimum_sampled_radius_m": (
                None if math.isinf(minimum_sampled_radius)
                else minimum_sampled_radius
            ),
            "configured_turn_radius_m": configured_turn_radius,
            "minimum_distance_to_safe_area_edge_m": minimum_safe_edge_distance,
        },
    )

    plt = plot_setup()
    fig, ax = plt.subplots(figsize=(12, 10))
    if boundary is not None:
        draw_polygon(
            ax, boundary, facecolor="#eeeeee", edgecolor="#424242",
            alpha=0.45, label="Logged boundary"
        )
    if safe_area is not None:
        draw_polygon(
            ax, safe_area, facecolor="#c8e6c9", edgecolor="#2e7d32",
            alpha=0.45, label="Reconstructed safe drive area"
        )
    ax.plot(
        [point[0] for point in xy],
        [point[1] for point in xy],
        color="#1565c0",
        linewidth=1.5,
        label="Mission",
    )
    ax.scatter(*xy[0], s=100, color="#00c853", edgecolor="black", label="Start")
    ax.scatter(*xy[-1], s=80, marker="x", color="#c62828", label="End")

    if finite_radii and not math.isinf(minimum_sampled_radius):
        tight_index = min(
            (index for index, radius in enumerate(radii) if math.isfinite(radius)),
            key=lambda index: radii[index],
        )
        ax.scatter(
            *xy[tight_index],
            marker="D",
            s=70,
            color="#ff6f00",
            edgecolor="black",
            label=f"Tightest sampled radius ({minimum_sampled_radius:.2f} m)",
            zorder=6,
        )

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    status = "FAIL" if errors else ("REVIEW" if warnings else "PASS")
    ax.set_title(
        f"Mission validation: {status} — {len(mission)} points, "
        f"{route_length:.1f} m"
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=170)
    plt.close(fig)

    print(f"Validation status : {status}")
    print(f"Waypoints         : {len(mission)}")
    print(f"Route length      : {route_length:.1f} m")
    print(f"Maximum gap       : {maximum_gap:.2f} m (limit {gap_limit:.2f} m)")
    print(f"Speed range       : {minimum_speed:.2f}–{maximum_speed:.2f} m/s")
    print(
        "Minimum radius    : "
        + (
            "straight/undefined"
            if math.isinf(minimum_sampled_radius)
            else f"{minimum_sampled_radius:.2f} m"
        )
    )
    for message in errors:
        print(f"ERROR   : {message}")
    for message in warnings:
        print(f"WARNING : {message}")
    print(f"Report            : {report_path}")
    print(f"Validation plot   : {plot_path}")
    if errors:
        print("\nDo not load this mission into the live controller.")
        return 2
    if warnings:
        print(
            "\nResolve/review warnings and use a supervised, low-speed first run. "
            "A REVIEW result is not automatic field approval."
        )
        return 1
    print(
        "\nStatic checks passed. Continue with simulation/dry-run and a supervised "
        "low-speed field test before normal operation."
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Audit a Pure Pursuit mission file")
    parser.add_argument("mission")
    parser.add_argument(
        "--settings",
        help="02_plan_settings.json; enables boundary/obstacle containment checks",
    )
    parser.add_argument(
        "--turn-radius-m",
        type=float,
        default=3.0,
        help="used only without --settings (default 3.0 m)",
    )
    parser.add_argument(
        "--expected-spacing-m",
        type=float,
        default=0.50,
        help="used only without --settings (default 0.50 m)",
    )
    parser.add_argument(
        "--max-gap-m",
        type=float,
        help="default is 1.6 × configured/expected waypoint spacing",
    )
    parser.add_argument("--max-speed-mps", type=float, default=1.50)
    parser.add_argument("--yaw-warning-degrees", type=float, default=20.0)
    parser.add_argument("--radius-warning-fraction", type=float, default=0.90)
    parser.add_argument("--geometry-tolerance-m", type=float, default=0.03)
    parser.add_argument("--strict-curvature", action="store_true")
    parser.add_argument("--report")
    parser.add_argument("--plot")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    for name in (
        "turn_radius_m", "expected_spacing_m", "max_speed_mps",
        "yaw_warning_degrees", "radius_warning_fraction",
        "geometry_tolerance_m",
    ):
        if getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.max_gap_m is not None and args.max_gap_m <= 0:
        parser.error("--max-gap-m must be positive")
    try:
        return run_validation(args)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
