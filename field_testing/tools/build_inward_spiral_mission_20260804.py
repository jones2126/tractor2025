#!/usr/bin/env python3
"""Build a continuous clockwise inward-spiral mission from a driven lap."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.polygon import orient


EAST_M_PER_DEG = 111_320.0
NORTH_M_PER_DEG = 110_540.0


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("boundary_log", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--start-row", type=int, default=3734)
    parser.add_argument("--end-row", type=int, default=8686)
    parser.add_argument("--lane-spacing-m", type=float, default=0.9652)
    parser.add_argument("--revolutions", type=int, default=14)
    parser.add_argument("--waypoint-spacing-m", type=float, default=0.50)
    parser.add_argument("--lookahead-m", type=float, default=2.0)
    parser.add_argument("--speed-mps", type=float, default=0.85)
    parser.add_argument("--simplify-m", type=float, default=0.10)
    parser.add_argument("--deck-width-m", type=float, default=1.0668)
    parser.add_argument("--smoothing-iterations", type=int, default=50)
    return parser.parse_args()


def load_lap(path: Path, start_row: int, end_row: int):
    with path.open(newline="", encoding="utf-8-sig") as handle:
        all_rows = list(csv.DictReader(handle))
    rows = [
        row for row in all_rows[start_row - 1 : end_row]
        if row.get("lat") and row.get("lon")
    ]
    if len(rows) < 3:
        raise ValueError("Selected lap does not contain enough GPS positions")
    return rows


def frame(rows):
    ref_lat = float(rows[0]["lat"])
    ref_lon = float(rows[0]["lon"])
    east_scale = EAST_M_PER_DEG * math.cos(math.radians(ref_lat))

    def xy(lat: float, lon: float):
        return (lon - ref_lon) * east_scale, (lat - ref_lat) * NORTH_M_PER_DEG

    def ll(x: float, y: float):
        return ref_lat + y / NORTH_M_PER_DEG, ref_lon + x / east_scale

    return ref_lat, ref_lon, xy, ll


def largest_polygon(geometry):
    if geometry.is_empty:
        raise ValueError("Inward offset became empty")
    if geometry.geom_type == "MultiPolygon":
        return max(geometry.geoms, key=lambda item: item.area)
    if geometry.geom_type != "Polygon":
        raise ValueError(f"Expected Polygon, got {geometry.geom_type}")
    return geometry


def clockwise_ring(polygon: Polygon, anchor: Point) -> tuple[LineString, float]:
    polygon = orient(polygon, sign=-1.0)
    ring = LineString(polygon.exterior.coords)
    return ring, ring.project(anchor)


def ring_point(ring: LineString, start_distance: float, phase: float):
    if phase >= 1.0:
        distance = start_distance
    else:
        distance = (start_distance + phase * ring.length) % ring.length
    point = ring.interpolate(distance)
    return point.x, point.y


def build_spiral(boundary: Polygon, anchor: Point, spacing: float, revolutions: int):
    rings = []
    for index in range(revolutions + 1):
        polygon = largest_polygon(
            boundary.buffer(-index * spacing, join_style="round")
        )
        ring, start_distance = clockwise_ring(polygon, anchor)
        rings.append((polygon, ring, start_distance))

    raw = []
    for index in range(revolutions):
        outer_ring, outer_start = rings[index][1:]
        inner_ring, inner_start = rings[index + 1][1:]
        samples = max(200, math.ceil(outer_ring.length / 0.10))
        for sample in range(samples):
            phase = sample / samples
            outer = ring_point(outer_ring, outer_start, phase)
            inner = ring_point(inner_ring, inner_start, phase)
            raw.append((
                outer[0] + phase * (inner[0] - outer[0]),
                outer[1] + phase * (inner[1] - outer[1]),
            ))
    final_ring, final_start = rings[-1][1:]
    raw.append(ring_point(final_ring, final_start, 0.0))
    return rings, LineString(raw)


def resample(line: LineString, spacing: float):
    count = max(2, math.ceil(line.length / spacing) + 1)
    return [
        line.interpolate(line.length * index / (count - 1)).coords[0]
        for index in range(count)
    ]


def smooth_points(points, iterations: int):
    """Round sampled corners while preserving the exact start and endpoint."""
    smoothed = [tuple(point) for point in points]
    for _ in range(iterations):
        smoothed = [smoothed[0]] + [
            (
                0.25 * smoothed[index - 1][0]
                + 0.50 * smoothed[index][0]
                + 0.25 * smoothed[index + 1][0],
                0.25 * smoothed[index - 1][1]
                + 0.50 * smoothed[index][1]
                + 0.25 * smoothed[index + 1][1],
            )
            for index in range(1, len(smoothed) - 1)
        ] + [smoothed[-1]]
    return smoothed


def turn_radius(a, b, c):
    ab = math.dist(a, b)
    bc = math.dist(b, c)
    ca = math.dist(c, a)
    twice_area = abs(
        (b[0] - a[0]) * (c[1] - a[1])
        - (b[1] - a[1]) * (c[0] - a[0])
    )
    if twice_area < 1e-9:
        return math.inf
    return ab * bc * ca / (2.0 * twice_area)


def write_mission(path: Path, points, to_ll, lookahead: float, speed: float):
    with path.open("w", encoding="ascii", newline="\n") as handle:
        for index, point in enumerate(points):
            other = points[index + 1] if index + 1 < len(points) else points[index - 1]
            if index + 1 < len(points):
                yaw = math.atan2(other[1] - point[1], other[0] - point[0])
            else:
                yaw = math.atan2(point[1] - other[1], point[0] - other[0])
            lat, lon = to_ll(*point)
            handle.write(
                f"{lat:.9f} {lon:.9f} {yaw:.6f} {lookahead:.2f} {speed:.2f}\n"
            )


def main() -> None:
    args = arguments()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = load_lap(args.boundary_log, args.start_row, args.end_row)
    ref_lat, ref_lon, to_xy, to_ll = frame(rows)
    source_points = [to_xy(float(row["lat"]), float(row["lon"])) for row in rows]
    source_boundary = Polygon(source_points)
    if not source_boundary.is_valid:
        source_boundary = source_boundary.buffer(0)
    boundary = largest_polygon(
        source_boundary.simplify(args.simplify_m, preserve_topology=True)
    )
    boundary = orient(boundary, sign=-1.0)
    anchor = Point(source_points[0])

    rings, dense_spiral = build_spiral(
        boundary, anchor, args.lane_spacing_m, args.revolutions
    )
    points = resample(dense_spiral, args.waypoint_spacing_m)
    points = smooth_points(points, args.smoothing_iterations)
    points = resample(LineString(points), args.waypoint_spacing_m)
    mission = args.output_dir / "polygon_1_clockwise_inward_38in_20260804.txt"
    write_mission(mission, points, to_ll, args.lookahead_m, args.speed_mps)

    route = LineString(points)
    outside_length = route.difference(boundary.buffer(0.02)).length
    radii = [turn_radius(points[i - 1], points[i], points[i + 1]) for i in range(1, len(points) - 1)]
    finite_radii = sorted(value for value in radii if math.isfinite(value))
    deck_swath = route.buffer(args.deck_width_m / 2.0, cap_style="round", join_style="round")
    uncut = boundary.difference(deck_swath)
    runtime_minutes = route.length / args.speed_mps / 60.0
    first_yaw = math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0])
    first_compass = (90.0 - math.degrees(first_yaw)) % 360.0
    start_lat, start_lon = to_ll(*points[0])
    end_lat, end_lon = to_ll(*points[-1])

    report = {
        "source_boundary_log": str(args.boundary_log.resolve()),
        "source_data_rows": [args.start_row, args.end_row],
        "source_fix_qualities": dict(sorted(
            __import__("collections").Counter(row["fix_quality"] for row in rows).items()
        )),
        "geometry_warning": (
            "Complete driven lap includes RTK Float, DGPS, and two Invalid rows. "
            "Mission requires supervised field review."
        ),
        "lane_spacing_m": args.lane_spacing_m,
        "lane_spacing_in": args.lane_spacing_m / 0.0254,
        "revolutions": args.revolutions,
        "final_inset_m": args.revolutions * args.lane_spacing_m,
        "waypoint_spacing_m": args.waypoint_spacing_m,
        "smoothing_iterations": args.smoothing_iterations,
        "waypoints": len(points),
        "lookahead_m": args.lookahead_m,
        "speed_mps": args.speed_mps,
        "route_length_m": route.length,
        "estimated_runtime_minutes": runtime_minutes,
        "outside_boundary_length_m_with_2cm_tolerance": outside_length,
        "minimum_sampled_turn_radius_m": finite_radii[0],
        "sampled_turn_radius_p01_m": finite_radii[max(0, int(0.01 * (len(finite_radii) - 1)))],
        "boundary_area_m2": boundary.area,
        "presumed_uncut_area_m2": uncut.area,
        "presumed_uncut_percent": 100.0 * uncut.area / boundary.area,
        "start_lat": start_lat,
        "start_lon": start_lon,
        "start_heading_compass_deg": first_compass,
        "end_lat": end_lat,
        "end_lon": end_lon,
        "mission_sha256": hashlib.sha256(mission.read_bytes()).hexdigest(),
    }
    report_path = args.output_dir / "polygon_1_clockwise_inward_38in_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    fig, ax = plt.subplots(figsize=(11, 10))
    bx, by = boundary.exterior.xy
    ax.plot(bx, by, "--", color="#212121", linewidth=1.6, label="Today’s driven boundary")
    rx, ry = route.xy
    ax.plot(rx, ry, color="#1565c0", linewidth=1.1, label="Clockwise inward spiral")
    step = max(1, len(points) // 35)
    for index in range(0, len(points) - 1, step):
        x, y = points[index]
        nx, ny = points[min(index + 3, len(points) - 1)]
        ax.annotate("", xy=(nx, ny), xytext=(x, y),
                    arrowprops=dict(arrowstyle="->", color="#1565c0", lw=1.1))
    ax.scatter([points[0][0]], [points[0][1]], color="green", s=80, marker="*", label="Start")
    ax.scatter([points[-1][0]], [points[-1][1]], color="red", s=45, marker="x", label="End")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("East of starting point (m)")
    ax.set_ylabel("North of starting point (m)")
    ax.set_title(
        f"Clockwise continuous inward spiral - {args.lane_spacing_m / 0.0254:.0f} in spacing\n"
        f"{route.length:.0f} m, estimated {runtime_minutes:.1f} min at {args.speed_mps:.2f} m/s"
    )
    ax.legend(loc="best")
    fig.tight_layout()
    preview = args.output_dir / "polygon_1_clockwise_inward_38in_preview.png"
    fig.savefig(preview, dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 10))
    ax.fill(bx, by, color="#fafafa", edgecolor="#212121", linewidth=1.5, label="Boundary interior")
    if not uncut.is_empty:
        shapes = [uncut] if uncut.geom_type == "Polygon" else list(uncut.geoms)
        for index, shape in enumerate(shapes):
            ux, uy = shape.exterior.xy
            ax.fill(ux, uy, color="#ef5350", alpha=0.75,
                    label="Presumed uncut" if index == 0 else None)
    clipped_swath = deck_swath.intersection(boundary)
    shapes = [clipped_swath] if clipped_swath.geom_type == "Polygon" else list(clipped_swath.geoms)
    for index, shape in enumerate(shapes):
        sx, sy = shape.exterior.xy
        ax.fill(sx, sy, color="#64b5f6", alpha=0.45,
                label="42 in deck swath" if index == 0 else None)
    ax.plot(rx, ry, color="#0d47a1", linewidth=0.7, label="Tractor centerline")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("East of starting point (m)")
    ax.set_ylabel("North of starting point (m)")
    ax.set_title(
        f"Presumed deck coverage - uncut {uncut.area:.1f} m2 "
        f"({100.0 * uncut.area / boundary.area:.1f}%)"
    )
    ax.legend(loc="best")
    fig.tight_layout()
    coverage = args.output_dir / "polygon_1_clockwise_inward_38in_deck_coverage.png"
    fig.savefig(coverage, dpi=180)
    plt.close(fig)

    print(f"Mission : {mission}")
    print(f"Preview : {preview}")
    print(f"Coverage: {coverage}")
    print(f"Report  : {report_path}")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
