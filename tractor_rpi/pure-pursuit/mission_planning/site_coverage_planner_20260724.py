#!/usr/bin/env python3
"""Interactive lat/lon coverage planner for the standalone Pure Pursuit stack.

Subcommands form deliberate operator checkpoints:

``compare-angles``
    Compare Boustrophedon stripe angles before committing to one.
``preview``
    Create safe-area/headland/stripe plots plus an Excel-editable segments CSV.
``build``
    Read the reviewed segments CSV, connect path pieces with contained Dubins
    paths, and export ``lat lon yaw lookahead speed`` mission rows.

The planner never silently substitutes a straight connector.  If no
turn-radius-constrained Dubins connector remains inside the safe drive polygon,
``build`` stops without producing an executable mission.
"""

from __future__ import annotations

import argparse
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Sequence

from site_planner_common_20260724 import (
    LocalFrame,
    MissionPoint,
    circumcircle_radius,
    deduplicate_points,
    densify_polyline,
    distance,
    dubins_candidates,
    is_included,
    load_boundary,
    load_obstacles,
    normalize_angle,
    polyline_length,
    read_csv,
    read_json,
    require_matplotlib,
    require_shapely,
    rotate_closed_ring,
    segment_yaw,
    write_csv,
    write_json,
    write_mission,
)


SEGMENT_COLUMNS = [
    "sequence",
    "include",
    "reverse",
    "scanline",
    "start_lat",
    "start_lon",
    "end_lat",
    "end_lon",
    "start_east_m",
    "start_north_m",
    "end_east_m",
    "end_north_m",
    "length_m",
    "notes",
]

ANGLE_COLUMNS = [
    "angle_degrees",
    "included_stripes",
    "clipped_segments",
    "short_segments",
    "stripe_length_m",
    "air_connectors_m",
    "estimated_route_m",
]

AUDIT_COLUMNS = [
    "waypoint",
    "kind",
    "east_m",
    "north_m",
    "lat",
    "lon",
    "yaw_rad",
    "lookahead_m",
    "speed_mps",
    "local_radius_m",
]


def plot_setup():
    require_matplotlib()
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    return plt


def polygon_parts(geometry) -> list:
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


def line_parts(geometry) -> list:
    if geometry.is_empty:
        return []
    if geometry.geom_type == "LineString":
        return [geometry]
    if geometry.geom_type in {"MultiLineString", "GeometryCollection"}:
        parts = []
        for child in geometry.geoms:
            parts.extend(line_parts(child))
        return parts
    return []


def validate_positive(name: str, value: float) -> None:
    if value <= 0:
        raise ValueError(f"{name} must be positive")


def make_geometry(
    boundary_file: str,
    obstacles_file: str | None,
    boundary_clearance_m: float,
    headland_passes: int,
    lane_spacing_m: float,
    frame: LocalFrame | None = None,
):
    require_shapely()
    from shapely.geometry import Point, Polygon
    from shapely.validation import explain_validity

    frame, boundary_xy, _rows = load_boundary(boundary_file, frame=frame)
    boundary = Polygon(boundary_xy)
    if not boundary.is_valid:
        raise ValueError(
            f"Boundary polygon is invalid: {explain_validity(boundary)}"
        )
    if boundary.area <= 0:
        raise ValueError("Boundary polygon has zero area")

    drive_area = boundary.buffer(-boundary_clearance_m, join_style="round")
    if drive_area.is_empty:
        raise ValueError(
            "Boundary clearance removes the entire site; reduce "
            "--boundary-clearance-m or check the boundary."
        )

    obstacles = load_obstacles(obstacles_file)
    obstacle_shapes = []
    for obstacle in obstacles:
        center = frame.to_xy(obstacle.lat, obstacle.lon)
        shape = Point(*center).buffer(obstacle.exclusion_radius_m, resolution=32)
        obstacle_shapes.append((obstacle, center, shape))
        drive_area = drive_area.difference(shape)

    components = polygon_parts(drive_area)
    if len(components) != 1:
        areas = ", ".join(f"{part.area:.1f}" for part in components)
        raise ValueError(
            "Safe drive area is disconnected after clearances/obstacles "
            f"({len(components)} components; areas {areas} m²). Plan each "
            "component as a separate mission."
        )
    drive_area = components[0]

    stripe_area = drive_area.buffer(
        -(headland_passes * lane_spacing_m),
        join_style="round",
    )
    if stripe_area.is_empty:
        raise ValueError(
            "Headland passes leave no interior stripe area. Reduce "
            "--headland-passes/--lane-spacing-m or review the site geometry."
        )
    return frame, boundary, drive_area, stripe_area, obstacle_shapes


def make_headland_paths(
    drive_area,
    passes: int,
    spacing_m: float,
    turn_radius_m: float,
):
    paths: list[dict[str, object]] = []
    for pass_index in range(passes):
        inset = (pass_index + 0.5) * spacing_m
        geometry = drive_area.buffer(-inset, join_style="round")
        # An inward offset of a rectangular polygon still has mathematically
        # sharp convex corners. Morphological opening rounds those corners
        # inward with the configured tractor radius and remains inside the
        # unsmoothed center-path polygon.
        eroded = geometry.buffer(-turn_radius_m, join_style="round")
        if eroded.is_empty:
            raise ValueError(
                f"Headland pass {pass_index + 1} is too narrow to round with "
                f"the configured {turn_radius_m:.2f} m turn radius."
            )
        geometry = eroded.buffer(turn_radius_m, join_style="round")
        for component_index, polygon in enumerate(polygon_parts(geometry), 1):
            paths.append(
                {
                    "kind": "headland",
                    "label": f"headland_{pass_index + 1}_outer_{component_index}",
                    "pass": pass_index + 1,
                    "points": list(polygon.exterior.coords),
                }
            )
            for hole_index, interior in enumerate(polygon.interiors, 1):
                paths.append(
                    {
                        "kind": "headland",
                        "label": (
                            f"headland_{pass_index + 1}_obstacle_"
                            f"{component_index}_{hole_index}"
                        ),
                        "pass": pass_index + 1,
                        "points": list(interior.coords),
                    }
                )
    return paths


def make_stripes(
    stripe_area,
    angle_degrees: float,
    spacing_m: float,
    end_trim_m: float,
    scan_from: str,
    first_direction: str,
):
    from shapely import affinity
    from shapely.geometry import LineString

    angle = angle_degrees % 180.0
    origin = stripe_area.centroid.coords[0]
    rotated = affinity.rotate(
        stripe_area, -angle, origin=origin, use_radians=False
    )
    min_x, min_y, max_x, max_y = rotated.bounds
    padding = math.hypot(max_x - min_x, max_y - min_y) + spacing_m

    scan_values = []
    y_value = min_y + spacing_m / 2.0
    while y_value <= max_y - spacing_m / 2.0 + 1e-9:
        scan_values.append(y_value)
        y_value += spacing_m
    if scan_from == "high":
        scan_values.reverse()

    rows: list[list] = []
    for y_value in scan_values:
        cutter = LineString(
            [(min_x - padding, y_value), (max_x + padding, y_value)]
        )
        parts = line_parts(rotated.intersection(cutter))
        parts.sort(key=lambda line: line.centroid.x)
        if parts:
            rows.append(parts)

    result: list[dict[str, object]] = []
    forward_first = first_direction == "forward"
    for scanline, parts in enumerate(rows, 1):
        forward = forward_first if scanline % 2 else not forward_first
        ordered = parts if forward else list(reversed(parts))
        for part in ordered:
            coords = list(part.coords)
            if not forward:
                coords.reverse()
            original_start = coords[0]
            original_end = coords[-1]
            original_length = distance(original_start, original_end)
            applied_trim = min(
                end_trim_m,
                max(0.0, (original_length - 0.01) / 2.0),
            )
            if applied_trim > 0:
                unit_x = (original_end[0] - original_start[0]) / original_length
                unit_y = (original_end[1] - original_start[1]) / original_length
                coords = [
                    (
                        original_start[0] + unit_x * applied_trim,
                        original_start[1] + unit_y * applied_trim,
                    ),
                    (
                        original_end[0] - unit_x * applied_trim,
                        original_end[1] - unit_y * applied_trim,
                    ),
                ]
            world_line = affinity.rotate(
                LineString(coords), angle, origin=origin, use_radians=False
            )
            start, end = world_line.coords[0], world_line.coords[-1]
            result.append(
                {
                    "scanline": scanline,
                    "start": (float(start[0]), float(start[1])),
                    "end": (float(end[0]), float(end[1])),
                    "length_m": float(world_line.length),
                    "original_length_m": original_length,
                    "end_trim_m": applied_trim,
                }
            )
    return result


def air_connector_length(stripes: Sequence[dict[str, object]]) -> float:
    total = 0.0
    for first, second in zip(stripes, stripes[1:]):
        total += distance(first["end"], second["start"])  # type: ignore[arg-type]
    return total


def parse_angle_spec(specification: str) -> list[float]:
    specification = specification.strip()
    if ":" in specification:
        pieces = specification.split(":")
        if len(pieces) != 3:
            raise ValueError("Angle range must be START:STOP:STEP")
        start, stop, step = map(float, pieces)
        if step <= 0 or stop < start:
            raise ValueError("Angle range needs STEP > 0 and STOP >= START")
        values = []
        current = start
        while current <= stop + 1e-9:
            values.append(current)
            current += step
    else:
        values = [float(piece) for piece in specification.split(",") if piece.strip()]
    if not values:
        raise ValueError("No angles supplied")
    return sorted({round(value % 180.0, 6) for value in values})


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


def run_compare_angles(args: argparse.Namespace) -> int:
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    frame, boundary, drive_area, stripe_area, obstacle_shapes = make_geometry(
        args.boundary,
        args.obstacles,
        args.boundary_clearance_m,
        args.headland_passes,
        args.lane_spacing_m,
    )
    angles = parse_angle_spec(args.angles)
    rows = []
    for angle in angles:
        stripes = make_stripes(
            stripe_area,
            angle,
            args.lane_spacing_m,
            args.stripe_end_trim_m,
            args.scan_from,
            args.first_stripe_direction,
        )
        included = [
            stripe for stripe in stripes
            if float(stripe["length_m"]) >= args.minimum_segment_m
        ]
        stripe_length = sum(float(stripe["length_m"]) for stripe in included)
        connectors = air_connector_length(included)
        rows.append(
            {
                "angle_degrees": f"{angle:.1f}",
                "included_stripes": len(included),
                "clipped_segments": len(stripes),
                "short_segments": len(stripes) - len(included),
                "stripe_length_m": f"{stripe_length:.1f}",
                "air_connectors_m": f"{connectors:.1f}",
                "estimated_route_m": f"{stripe_length + connectors:.1f}",
            }
        )

    csv_path = output_dir / "01_angle_comparison.csv"
    plot_path = output_dir / "01_angle_comparison.png"
    write_csv(csv_path, ANGLE_COLUMNS, rows)

    plt = plot_setup()
    fig, left_axis = plt.subplots(figsize=(11, 7))
    x_values = [float(row["angle_degrees"]) for row in rows]
    route_values = [float(row["estimated_route_m"]) for row in rows]
    connector_values = [float(row["air_connectors_m"]) for row in rows]
    line1 = left_axis.plot(
        x_values,
        route_values,
        "o-",
        color="#1565c0",
        label="Stripe + straight-line connector estimate",
    )
    line2 = left_axis.plot(
        x_values,
        connector_values,
        "o--",
        color="#ef6c00",
        label="Straight-line connector estimate",
    )
    left_axis.set_xlabel("Stripe angle (degrees CCW from east)")
    left_axis.set_ylabel("Estimated distance (m)")
    left_axis.grid(True, alpha=0.3)

    right_axis = left_axis.twinx()
    count_values = [int(row["included_stripes"]) for row in rows]
    line3 = right_axis.plot(
        x_values,
        count_values,
        "s-",
        color="#2e7d32",
        label="Included stripe segments",
    )
    right_axis.set_ylabel("Stripe segment count")
    all_lines = line1 + line2 + line3
    left_axis.legend(all_lines, [line.get_label() for line in all_lines])
    left_axis.set_title(
        "Angle comparison (screening estimate only—turn feasibility comes later)"
    )
    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

    best = min(rows, key=lambda row: float(row["estimated_route_m"]))
    print(f"Angles evaluated : {len(rows)}")
    print(
        "Shortest screening estimate: "
        f"{best['angle_degrees']}° ({best['estimated_route_m']} m)"
    )
    print(f"Comparison CSV   : {csv_path}")
    print(f"Comparison plot  : {plot_path}")
    print(
        "\nThe shortest estimate is not automatically the best field choice. "
        "Review slope, wet areas, discharge direction and turn space, then run "
        "preview with the selected --angle-degrees (19 is valid)."
    )
    return 0


def settings_from_args(
    args: argparse.Namespace,
    frame: LocalFrame,
    output_dir: Path,
) -> dict[str, object]:
    start_lat = args.start_lat if args.start_lat is not None else frame.ref_lat
    start_lon = args.start_lon if args.start_lon is not None else frame.ref_lon
    return {
        "format_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "boundary_file": str(Path(args.boundary).resolve()),
        "obstacles_file": (
            str(Path(args.obstacles).resolve()) if args.obstacles else None
        ),
        "frame": frame.to_dict(),
        "angle_degrees": args.angle_degrees % 180.0,
        "lane_spacing_m": args.lane_spacing_m,
        "boundary_clearance_m": args.boundary_clearance_m,
        "headland_passes": args.headland_passes,
        "minimum_segment_m": args.minimum_segment_m,
        "stripe_end_trim_m": args.stripe_end_trim_m,
        "scan_from": args.scan_from,
        "first_stripe_direction": args.first_stripe_direction,
        "ring_direction": args.ring_direction,
        "start_anchor": {"lat": start_lat, "lon": start_lon},
        "turn_radius_m": args.turn_radius_m,
        "waypoint_spacing_m": args.waypoint_spacing_m,
        "straight_lookahead_m": args.straight_lookahead_m,
        "straight_speed_mps": args.straight_speed_mps,
        "turn_lookahead_m": args.turn_lookahead_m,
        "turn_speed_mps": args.turn_speed_mps,
        "headland_lookahead_m": args.headland_lookahead_m,
        "headland_speed_mps": args.headland_speed_mps,
        "outputs": {
            "segments_csv": str((output_dir / "02_coverage_segments.csv").resolve()),
            "preview_plot": str((output_dir / "02_coverage_preview.png").resolve()),
        },
    }


def run_preview(args: argparse.Namespace) -> int:
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    frame, boundary, drive_area, stripe_area, obstacle_shapes = make_geometry(
        args.boundary,
        args.obstacles,
        args.boundary_clearance_m,
        args.headland_passes,
        args.lane_spacing_m,
    )
    headlands = make_headland_paths(
        drive_area,
        args.headland_passes,
        args.lane_spacing_m,
        args.turn_radius_m,
    )
    stripes = make_stripes(
        stripe_area,
        args.angle_degrees,
        args.lane_spacing_m,
        args.stripe_end_trim_m,
        args.scan_from,
        args.first_stripe_direction,
    )

    segment_rows = []
    for sequence, stripe in enumerate(stripes, 1):
        start = stripe["start"]
        end = stripe["end"]
        start_lat, start_lon = frame.to_latlon(*start)
        end_lat, end_lon = frame.to_latlon(*end)
        short = float(stripe["length_m"]) < args.minimum_segment_m
        segment_rows.append(
            {
                "sequence": sequence,
                "include": "n" if short else "y",
                "reverse": "n",
                "scanline": stripe["scanline"],
                "start_lat": f"{start_lat:.9f}",
                "start_lon": f"{start_lon:.9f}",
                "end_lat": f"{end_lat:.9f}",
                "end_lon": f"{end_lon:.9f}",
                "start_east_m": f"{start[0]:.3f}",
                "start_north_m": f"{start[1]:.3f}",
                "end_east_m": f"{end[0]:.3f}",
                "end_north_m": f"{end[1]:.3f}",
                "length_m": f"{float(stripe['length_m']):.3f}",
                "notes": (
                    (
                        f"auto-excluded after {float(stripe['end_trim_m']):.2f} m "
                        f"trim at each end: shorter than "
                        f"{args.minimum_segment_m:.2f} m"
                    )
                    if short else (
                        f"trimmed {float(stripe['end_trim_m']):.2f} m at each end"
                    )
                ),
            }
        )

    settings = settings_from_args(args, frame, output_dir)
    settings_path = output_dir / "02_plan_settings.json"
    segments_path = output_dir / "02_coverage_segments.csv"
    plot_path = output_dir / "02_coverage_preview.png"
    write_json(settings_path, settings)
    write_csv(segments_path, SEGMENT_COLUMNS, segment_rows)

    plt = plot_setup()
    fig, ax = plt.subplots(figsize=(12, 10))
    draw_polygon(
        ax, boundary, facecolor="#eeeeee", edgecolor="#424242",
        alpha=0.45, label="Logged boundary"
    )
    draw_polygon(
        ax, drive_area, facecolor="#c8e6c9", edgecolor="#2e7d32",
        alpha=0.45, label="Safe tractor-center area"
    )
    draw_polygon(
        ax, stripe_area, facecolor="#fff9c4", edgecolor="#f9a825",
        alpha=0.35, label="Interior stripe area"
    )
    for path_index, path in enumerate(headlands):
        points = path["points"]
        ax.plot(
            [point[0] for point in points],
            [point[1] for point in points],
            color="#6a1b9a",
            linewidth=1.5,
            label="Headland center paths" if path_index == 0 else None,
        )
    for sequence, stripe in enumerate(stripes, 1):
        start, end = stripe["start"], stripe["end"]
        included = float(stripe["length_m"]) >= args.minimum_segment_m
        color = "#1565c0" if included else "#c62828"
        ax.annotate(
            "",
            xy=end,
            xytext=start,
            arrowprops={"arrowstyle": "->", "color": color, "lw": 1.3},
        )
        midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
        ax.text(
            *midpoint,
            str(sequence),
            fontsize=7,
            color="white",
            ha="center",
            va="center",
            bbox={"boxstyle": "circle,pad=0.14", "fc": color, "ec": "none"},
        )
    for obstacle, center, shape in obstacle_shapes:
        x, y = shape.exterior.xy
        ax.fill(x, y, color="#ef5350", alpha=0.35)
        ax.text(center[0], center[1], obstacle.name, ha="center", va="center")

    anchor = settings["start_anchor"]
    anchor_xy = frame.to_xy(float(anchor["lat"]), float(anchor["lon"]))
    ax.scatter(
        *anchor_xy,
        color="#00c853",
        edgecolor="black",
        marker="*",
        s=180,
        label="Preferred headland splice anchor",
        zorder=5,
    )
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(
        f"Coverage preview: {args.angle_degrees % 180.0:.1f}°, "
        f"{args.lane_spacing_m:.2f} m lanes"
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(plot_path, dpi=170)
    plt.close(fig)

    included_count = sum(is_included(row["include"]) for row in segment_rows)
    print(f"Safe drive area      : {drive_area.area:.1f} m²")
    print(f"Interior stripe area : {stripe_area.area:.1f} m²")
    print(f"Headland path pieces : {len(headlands)}")
    print(f"Stripe segments      : {len(stripes)} ({included_count} auto-included)")
    print(f"Settings             : {settings_path}")
    print(f"Editable segments    : {segments_path}")
    print(f"Checkpoint plot      : {plot_path}")
    print(
        "\nNext: inspect the plot and edit include/reverse/sequence/notes in the "
        "segments CSV. Then run build. No mission exists yet."
    )
    return 0


def settings_geometry(settings: dict[str, object]):
    frame = LocalFrame.from_dict(settings["frame"])
    return make_geometry(
        str(settings["boundary_file"]),
        (
            str(settings["obstacles_file"])
            if settings.get("obstacles_file") else None
        ),
        float(settings["boundary_clearance_m"]),
        int(settings["headland_passes"]),
        float(settings["lane_spacing_m"]),
        frame=frame,
    )


def read_reviewed_stripes(
    path: str,
    frame: LocalFrame,
) -> list[dict[str, object]]:
    rows = [row for row in read_csv(path) if is_included(row.get("include", "y"))]
    if not rows:
        raise ValueError("Reviewed segments CSV has no included stripes")
    try:
        rows.sort(key=lambda row: float(row["sequence"]))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Included segment sequence values must be numeric") from exc

    stripes = []
    for row_number, row in enumerate(rows, 2):
        try:
            # Lat/lon are authoritative after Excel editing; local columns are
            # conveniences for inspection only.
            start = frame.to_xy(float(row["start_lat"]), float(row["start_lon"]))
            end = frame.to_xy(float(row["end_lat"]), float(row["end_lon"]))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"Invalid start/end lat/lon in included segment row {row_number}"
            ) from exc
        if is_included(row.get("reverse", "n")):
            start, end = end, start
        if distance(start, end) < 0.05:
            raise ValueError(f"Included segment row {row_number} is nearly zero length")
        stripes.append(
            {
                "kind": "stripe",
                "label": f"stripe_{row.get('sequence', row_number - 1)}",
                "points": [start, end],
            }
        )
    return stripes


def path_start_pose(points: Sequence[tuple[float, float]]):
    return points[0][0], points[0][1], segment_yaw(points[0], points[1])


def path_end_pose(points: Sequence[tuple[float, float]]):
    return points[-1][0], points[-1][1], segment_yaw(points[-2], points[-1])


def valid_connector(
    start_pose,
    end_pose,
    drive_area,
    radius_m: float,
    spacing_m: float,
):
    from shapely.geometry import LineString

    safe_numeric_area = drive_area.buffer(0.03)
    for candidate in dubins_candidates(
        start_pose, end_pose, radius_m, spacing_m
    ):
        points = candidate["points"]
        line = LineString([(point[0], point[1]) for point in points])
        if safe_numeric_area.covers(line):
            return candidate
    return None


def prepare_ring(
    raw_points: Sequence[tuple[float, float]],
    clockwise: bool,
    anchor: tuple[float, float],
):
    return rotate_closed_ring(raw_points, anchor, clockwise)


def best_next_ring(
    raw_points: Sequence[tuple[float, float]],
    clockwise: bool,
    previous_pose,
    drive_area,
    turn_radius_m: float,
    waypoint_spacing_m: float,
):
    # Orient once, then try a bounded set of splice points.  Shapely rings can
    # contain hundreds of arc-resolution points.
    oriented = rotate_closed_ring(
        raw_points, (previous_pose[0], previous_pose[1]), clockwise
    )[:-1]
    stride = max(1, len(oriented) // 60)
    best = None
    for index in range(0, len(oriented), stride):
        ring = oriented[index:] + oriented[:index]
        ring.append(ring[0])
        candidate = valid_connector(
            previous_pose,
            path_start_pose(ring),
            drive_area,
            turn_radius_m,
            waypoint_spacing_m,
        )
        if candidate is None:
            continue
        score = float(candidate["length_m"])
        if best is None or score < best[0]:
            best = (score, ring, candidate)
    return best


def ring_variants(
    raw_points: Sequence[tuple[float, float]],
    clockwise: bool,
    maximum_variants: int = 72,
):
    """Create possible start/end splice locations around one oriented ring."""

    # Offset rectangles may have only four Shapely vertices. Densifying before
    # choosing splice candidates allows a safe merge from the middle of a
    # straight edge instead of limiting the search to outward-facing corners.
    dense = densify_polyline(raw_points, 1.0)
    oriented = rotate_closed_ring(dense, dense[0], clockwise)[:-1]
    stride = max(1, math.ceil(len(oriented) / maximum_variants))
    variants = []
    for index in range(0, len(oriented), stride):
        ring = oriented[index:] + oriented[:index]
        ring.append(ring[0])
        variants.append(
            {
                "ring": ring,
                "start_pose": path_start_pose(ring),
                "end_pose": path_end_pose(ring),
            }
        )
    return variants


def plan_headland_chain(
    headlands: Sequence[dict[str, object]],
    anchor: tuple[float, float],
    next_pose,
    clockwise: bool,
    drive_area,
    turn_radius_m: float,
    waypoint_spacing_m: float,
):
    """Select ring splice points with feasible connectors between all pieces.

    The first-ring anchor is a preference, not a hard location. Dynamic
    programming minimizes anchor offset plus connector length while requiring
    every transition—including the transition to the first stripe—to remain
    inside the safe drive polygon.
    """

    if not headlands:
        return [], None

    states = []
    for variant in ring_variants(headlands[0]["points"], clockwise):
        states.append(
            {
                **variant,
                "label": headlands[0]["label"],
                "cost": distance(variant["ring"][0], anchor),
                "previous": None,
                "incoming": None,
            }
        )

    for ring_info in headlands[1:]:
        new_states = []
        for variant in ring_variants(ring_info["points"], clockwise):
            best_state = None
            for previous in states:
                connector = valid_connector(
                    previous["end_pose"],
                    variant["start_pose"],
                    drive_area,
                    turn_radius_m,
                    waypoint_spacing_m,
                )
                if connector is None:
                    continue
                cost = previous["cost"] + float(connector["length_m"])
                if best_state is None or cost < best_state["cost"]:
                    best_state = {
                        **variant,
                        "label": ring_info["label"],
                        "cost": cost,
                        "previous": previous,
                        "incoming": connector,
                    }
            if best_state is not None:
                new_states.append(best_state)
        if not new_states:
            raise ValueError(
                f"No {turn_radius_m:.2f} m-radius contained connector can "
                f"sequence the headland path {ring_info['label']}."
            )
        states = new_states

    best_final = None
    for state in states:
        outgoing = valid_connector(
            state["end_pose"],
            next_pose,
            drive_area,
            turn_radius_m,
            waypoint_spacing_m,
        )
        if outgoing is None:
            continue
        cost = state["cost"] + float(outgoing["length_m"])
        if best_final is None or cost < best_final[0]:
            best_final = (cost, state, outgoing)

    if best_final is None:
        raise ValueError(
            f"No {turn_radius_m:.2f} m-radius contained connector can leave "
            "the final headland and reach the first reviewed stripe. Try a "
            "different ring direction, stripe orientation/order, or more turn room."
        )

    _cost, state, outgoing = best_final
    selected = []
    while state is not None:
        selected.append(state)
        state = state["previous"]
    selected.reverse()
    return selected, outgoing


def append_piece(
    route: list[dict[str, object]],
    points: Sequence[tuple[float, float]],
    kind: str,
    spacing_m: float,
) -> None:
    dense = densify_polyline(points, spacing_m)
    for point in dense:
        if route and distance(route[-1]["xy"], point) < 1e-5:  # type: ignore[arg-type]
            continue
        route.append({"xy": point, "kind": kind})


def append_connector(
    route: list[dict[str, object]],
    candidate: dict[str, object],
) -> None:
    for x, y, _yaw in candidate["points"][1:]:
        if route and distance(route[-1]["xy"], (x, y)) < 1e-5:  # type: ignore[arg-type]
            continue
        route.append({"xy": (x, y), "kind": "connector"})


def mission_parameters(settings: dict[str, object], kind: str):
    if kind == "stripe":
        return (
            float(settings["straight_lookahead_m"]),
            float(settings["straight_speed_mps"]),
        )
    if kind == "headland":
        return (
            float(settings["headland_lookahead_m"]),
            float(settings["headland_speed_mps"]),
        )
    return (
        float(settings["turn_lookahead_m"]),
        float(settings["turn_speed_mps"]),
    )


def run_build(args: argparse.Namespace) -> int:
    settings = read_json(args.settings)
    required = {
        "boundary_file", "frame", "lane_spacing_m", "headland_passes",
        "turn_radius_m", "waypoint_spacing_m", "start_anchor",
    }
    missing = required.difference(settings)
    if missing:
        raise ValueError(f"Settings file is missing: {', '.join(sorted(missing))}")

    frame, boundary, drive_area, stripe_area, obstacle_shapes = settings_geometry(
        settings
    )
    segments_path = args.segments
    if not segments_path:
        segments_path = str(settings["outputs"]["segments_csv"])
    stripes = read_reviewed_stripes(segments_path, frame)
    headlands = make_headland_paths(
        drive_area,
        int(settings["headland_passes"]),
        float(settings["lane_spacing_m"]),
        float(settings["turn_radius_m"]),
    )

    turn_radius_m = float(settings["turn_radius_m"])
    spacing_m = float(settings["waypoint_spacing_m"])
    clockwise = str(settings.get("ring_direction", "clockwise")) == "clockwise"
    anchor_data = settings["start_anchor"]
    anchor = frame.to_xy(float(anchor_data["lat"]), float(anchor_data["lon"]))

    route: list[dict[str, object]] = []
    connector_records = []
    previous_pose = None
    first_stripe_preconnected = False

    if headlands:
        selected_rings, outgoing = plan_headland_chain(
            headlands,
            anchor,
            path_start_pose(stripes[0]["points"]),
            clockwise,
            drive_area,
            turn_radius_m,
            spacing_m,
        )
        for ring_index, selected in enumerate(selected_rings):
            incoming = selected["incoming"]
            if incoming is not None:
                append_connector(route, incoming)
                connector_records.append(
                    {
                        "from": selected_rings[ring_index - 1]["label"],
                        "to": selected["label"],
                        "mode": incoming["mode"],
                        "length_m": float(incoming["length_m"]),
                    }
                )
            append_piece(route, selected["ring"], "headland", spacing_m)
        append_connector(route, outgoing)
        connector_records.append(
            {
                "from": selected_rings[-1]["label"],
                "to": stripes[0]["label"],
                "mode": outgoing["mode"],
                "length_m": float(outgoing["length_m"]),
            }
        )
        previous_pose = path_start_pose(stripes[0]["points"])
        first_stripe_preconnected = True

    for stripe_index, stripe in enumerate(stripes):
        points = stripe["points"]
        start_pose = path_start_pose(points)
        if previous_pose is not None and not (
            stripe_index == 0 and first_stripe_preconnected
        ):
            connector = valid_connector(
                previous_pose,
                start_pose,
                drive_area,
                turn_radius_m,
                spacing_m,
            )
            if connector is None:
                raise ValueError(
                    f"No {turn_radius_m:.2f} m-radius contained Dubins connector "
                    f"from the previous path to {stripe['label']}. Inspect the "
                    "preview, try reversing that CSV row, trim the segment, "
                    "change the stripe order, or increase available turn space."
                )
            append_connector(route, connector)
            connector_records.append(
                {
                    "from": "previous path",
                    "to": stripe["label"],
                    "mode": connector["mode"],
                    "length_m": float(connector["length_m"]),
                }
            )
        append_piece(route, points, "stripe", spacing_m)
        previous_pose = path_end_pose(points)

    if len(route) < 2:
        raise ValueError("Planner produced fewer than two route points")
    route_xy = deduplicate_points([item["xy"] for item in route], tolerance_m=1e-5)
    if len(route_xy) != len(route):
        # Consecutive duplicates should already have been removed. Keep the
        # metadata aligned if floating-point snapping found any extras.
        compact = [route[0]]
        for item in route[1:]:
            if distance(compact[-1]["xy"], item["xy"]) >= 1e-5:  # type: ignore[arg-type]
                compact.append(item)
        route = compact
        route_xy = [item["xy"] for item in route]

    # The connector check is piecewise. This final whole-route check catches a
    # bad manual stripe edit or any unexpected geometry regression.
    from shapely.geometry import LineString

    route_line = LineString(route_xy)
    if not drive_area.buffer(0.03).covers(route_line):
        raise ValueError(
            "Final route leaves the safe drive area. No mission was written; "
            "check manual segment edits and clearances."
        )

    local_radii = [math.inf] * len(route)
    for index in range(1, len(route) - 1):
        local_radii[index] = circumcircle_radius(
            route_xy[index - 1], route_xy[index], route_xy[index + 1]
        )

    mission = []
    audit_rows = []
    for index, item in enumerate(route):
        if index < len(route) - 1:
            yaw = segment_yaw(route_xy[index], route_xy[index + 1])
        else:
            yaw = segment_yaw(route_xy[index - 1], route_xy[index])
        kind = str(item["kind"])
        lookahead, speed = mission_parameters(settings, kind)
        lat, lon = frame.to_latlon(*route_xy[index])
        mission_point = MissionPoint(
            lat=lat,
            lon=lon,
            yaw_rad=normalize_angle(yaw),
            lookahead_m=lookahead,
            speed_mps=speed,
        )
        mission.append(mission_point)
        audit_rows.append(
            {
                "waypoint": index + 1,
                "kind": kind,
                "east_m": f"{route_xy[index][0]:.3f}",
                "north_m": f"{route_xy[index][1]:.3f}",
                "lat": f"{lat:.9f}",
                "lon": f"{lon:.9f}",
                "yaw_rad": f"{mission_point.yaw_rad:.6f}",
                "lookahead_m": f"{lookahead:.2f}",
                "speed_mps": f"{speed:.2f}",
                "local_radius_m": (
                    "" if math.isinf(local_radii[index])
                    else f"{local_radii[index]:.3f}"
                ),
            }
        )

    finite_radii = [radius for radius in local_radii if math.isfinite(radius)]
    minimum_sampled_radius = min(finite_radii, default=math.inf)
    curvature_warning = minimum_sampled_radius < turn_radius_m * 0.90
    if curvature_warning and args.strict_curvature:
        raise ValueError(
            f"Sampled path contains an apparent {minimum_sampled_radius:.2f} m "
            f"radius, below 90% of configured {turn_radius_m:.2f} m. "
            "No mission written because --strict-curvature was set."
        )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    audit_path = Path(args.audit) if args.audit else output.with_name(
        f"{output.stem}_audit.csv"
    )
    report_path = Path(args.report) if args.report else output.with_name(
        f"{output.stem}_build_report.json"
    )
    plot_path = Path(args.plot) if args.plot else output.with_name(
        f"{output.stem}_preview.png"
    )
    write_mission(output, mission)
    write_csv(audit_path, AUDIT_COLUMNS, audit_rows)
    write_json(
        report_path,
        {
            "mission_file": str(output.resolve()),
            "settings_file": str(Path(args.settings).resolve()),
            "segments_file": str(Path(segments_path).resolve()),
            "waypoints": len(mission),
            "route_length_m": polyline_length(route_xy),
            "connector_count": len(connector_records),
            "connectors": connector_records,
            "minimum_sampled_radius_m": (
                None if math.isinf(minimum_sampled_radius)
                else minimum_sampled_radius
            ),
            "configured_turn_radius_m": turn_radius_m,
            "curvature_warning": curvature_warning,
            "contained_in_safe_drive_area": True,
            "controller_format": "lat lon yaw_rad lookahead_m speed_mps",
        },
    )

    plt = plot_setup()
    fig, ax = plt.subplots(figsize=(12, 10))
    draw_polygon(
        ax, boundary, facecolor="#eeeeee", edgecolor="#424242",
        alpha=0.4, label="Logged boundary"
    )
    draw_polygon(
        ax, drive_area, facecolor="#c8e6c9", edgecolor="#2e7d32",
        alpha=0.35, label="Safe drive area"
    )
    colors = {"headland": "#6a1b9a", "connector": "#ef6c00", "stripe": "#1565c0"}
    labels_used = set()
    for first, second in zip(route, route[1:]):
        kind = str(second["kind"])
        label = kind if kind not in labels_used else None
        labels_used.add(kind)
        ax.plot(
            [first["xy"][0], second["xy"][0]],
            [first["xy"][1], second["xy"][1]],
            color=colors[kind],
            linewidth=1.5,
            label=label,
        )
    ax.scatter(
        *route_xy[0], s=100, color="#00c853", edgecolor="black",
        label="Mission start", zorder=5
    )
    ax.scatter(
        *route_xy[-1], s=80, color="#c62828", marker="x",
        label="Mission end", zorder=5
    )
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(
        f"Executable mission preview: {len(mission)} waypoints, "
        f"{polyline_length(route_xy):.1f} m"
    )
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=170)
    plt.close(fig)

    print(f"Mission waypoints      : {len(mission)}")
    print(f"Route length           : {polyline_length(route_xy):.1f} m")
    print(f"Contained connectors   : {len(connector_records)}")
    print(
        "Minimum sampled radius: "
        + (
            "straight/undefined"
            if math.isinf(minimum_sampled_radius)
            else f"{minimum_sampled_radius:.2f} m"
        )
    )
    if curvature_warning:
        print(
            "WARNING: the sampled-radius check found a tight corner. This is "
            "usually an offset-polygon corner, not a Dubins connector. Review "
            "the plot/audit and run the independent validator before field use."
        )
    print(f"Mission file           : {output}")
    print(f"Waypoint audit         : {audit_path}")
    print(f"Build report           : {report_path}")
    print(f"Mission preview        : {plot_path}")
    print(
        "\nThis is a candidate executable file, not field approval. Run the "
        "independent mission validator and complete a low-speed supervised test."
    )
    return 0


def add_geometry_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("boundary", help="final boundary CSV from the log tool")
    parser.add_argument("--obstacles", help="optional circular obstacles CSV")
    parser.add_argument(
        "--lane-spacing-m",
        type=float,
        default=0.90,
        help="coverage lane center spacing (default 0.90 m)",
    )
    parser.add_argument(
        "--boundary-clearance-m",
        type=float,
        default=0.75,
        help="tractor reference-point clearance inside logged boundary (default 0.75 m)",
    )
    parser.add_argument(
        "--headland-passes",
        type=int,
        default=2,
        help="number of perimeter coverage passes before stripes (default 2)",
    )
    parser.add_argument(
        "--minimum-segment-m",
        type=float,
        default=2.0,
        help="shorter stripe pieces are auto-excluded but remain in CSV (default 2 m)",
    )
    parser.add_argument(
        "--stripe-end-trim-m",
        type=float,
        default=3.0,
        help=(
            "remove this distance from both ends of every stripe to create "
            "turn room (default 3.0 m)"
        ),
    )
    parser.add_argument(
        "--scan-from",
        choices=["low", "high"],
        default="low",
        help="start on low/high side of axis perpendicular to stripe angle",
    )
    parser.add_argument(
        "--first-stripe-direction",
        choices=["forward", "reverse"],
        default="forward",
        help="forward follows angle; reverse follows angle+180 degrees",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Interactive Boustrophedon coverage mission planner"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    compare = subparsers.add_parser(
        "compare-angles",
        help="screen candidate stripe angles and save a comparison plot/CSV",
    )
    add_geometry_arguments(compare)
    compare.add_argument(
        "--angles",
        default="0:175:5",
        help="comma list or START:STOP:STEP (default 0:175:5)",
    )
    compare.add_argument("--output-dir", required=True)
    compare.set_defaults(handler=run_compare_angles)

    preview = subparsers.add_parser(
        "preview",
        help="create editable stripe segments and the full geometry checkpoint plot",
    )
    add_geometry_arguments(preview)
    preview.add_argument(
        "--angle-degrees",
        type=float,
        default=19.0,
        help="stripe heading, CCW from east, modulo 180 (default 19)",
    )
    preview.add_argument("--output-dir", required=True)
    preview.add_argument(
        "--ring-direction",
        choices=["clockwise", "counterclockwise"],
        default="clockwise",
    )
    preview.add_argument("--start-lat", type=float)
    preview.add_argument("--start-lon", type=float)
    preview.add_argument("--turn-radius-m", type=float, default=3.0)
    preview.add_argument("--waypoint-spacing-m", type=float, default=0.50)
    preview.add_argument("--straight-lookahead-m", type=float, default=3.0)
    preview.add_argument("--straight-speed-mps", type=float, default=0.50)
    preview.add_argument("--turn-lookahead-m", type=float, default=1.5)
    preview.add_argument("--turn-speed-mps", type=float, default=0.30)
    preview.add_argument("--headland-lookahead-m", type=float, default=2.0)
    preview.add_argument("--headland-speed-mps", type=float, default=0.40)
    preview.set_defaults(handler=run_preview)

    build = subparsers.add_parser(
        "build",
        help="build a candidate executable mission from reviewed segments",
    )
    build.add_argument("--settings", required=True)
    build.add_argument("--segments", help="defaults to the CSV recorded in settings")
    build.add_argument("--output", required=True)
    build.add_argument("--audit")
    build.add_argument("--report")
    build.add_argument("--plot")
    build.add_argument(
        "--strict-curvature",
        action="store_true",
        help="refuse output when sampled corner radius is below 90% of turn radius",
    )
    build.set_defaults(handler=run_build)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    for name in (
        "lane_spacing_m", "boundary_clearance_m", "minimum_segment_m",
        "turn_radius_m", "waypoint_spacing_m", "straight_lookahead_m",
        "turn_lookahead_m", "headland_lookahead_m",
    ):
        if hasattr(args, name):
            try:
                validate_positive(f"--{name.replace('_', '-')}", float(getattr(args, name)))
            except ValueError as exc:
                parser.error(str(exc))
    if hasattr(args, "stripe_end_trim_m") and args.stripe_end_trim_m < 0:
        parser.error("--stripe-end-trim-m cannot be negative")
    if hasattr(args, "headland_passes") and args.headland_passes < 0:
        parser.error("--headland-passes cannot be negative")
    if hasattr(args, "start_lat") and ((args.start_lat is None) != (args.start_lon is None)):
        parser.error("--start-lat and --start-lon must be supplied together")
    for name in ("straight_speed_mps", "turn_speed_mps", "headland_speed_mps"):
        if hasattr(args, name) and getattr(args, name) < 0:
            parser.error(f"--{name.replace('_', '-')} cannot be negative")
    try:
        return int(args.handler(args))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
