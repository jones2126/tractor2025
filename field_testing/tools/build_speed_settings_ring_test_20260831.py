#!/usr/bin/env python3
"""Build a four-lap speed-calibration mission on the 13th spiral ring."""

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


DEFAULT_COMMANDS_MPS = (0.75, 0.94, 1.08, 1.25)
CURRENT_SPEED_CAL = (
    (0.00, 2836),
    (0.40, 2452),
    (0.87, 2421),
    (0.94, 2404),
    (1.08, 2370),
    (1.25, 2288),
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build four clockwise laps on the ring immediately outside ring 14."
    )
    parser.add_argument("boundary_csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--mission-name", default="62_Collins_ring13_four_speed_settings_REVIEW_TEST_20260831.txt")
    parser.add_argument("--lane-spacing-m", type=float, default=0.9652)
    parser.add_argument("--ring-number", type=int, default=13)
    parser.add_argument("--waypoint-spacing-m", type=float, default=0.50)
    parser.add_argument("--smoothing-iterations", type=int, default=50)
    parser.add_argument("--lookahead-m", type=float, default=2.0)
    parser.add_argument("--commands-mps", type=float, nargs=4, default=DEFAULT_COMMANDS_MPS)
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
    if len(rows) < 3:
        raise ValueError("Reviewed boundary must contain at least three points")
    lat0, lon0 = float(rows[0]["lat"]), float(rows[0]["lon"])
    to_xy, to_ll = local_frame(lat0, lon0)
    polygon = Polygon([to_xy(float(row["lat"]), float(row["lon"])) for row in rows])
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
    if polygon.geom_type != "Polygon":
        raise ValueError("Reviewed boundary is not one polygon")
    return orient(polygon, sign=-1.0), to_ll


def rotate_ring_at_anchor(ring: LineString, anchor: Point) -> LineString:
    start_distance = ring.project(anchor)
    samples = max(400, math.ceil(ring.length / 0.10))
    points = [
        ring.interpolate((start_distance + ring.length * index / samples) % ring.length).coords[0]
        for index in range(samples)
    ]
    points.append(points[0])
    return LineString(points)


def resample_closed(line: LineString, spacing: float):
    count = max(3, math.ceil(line.length / spacing))
    return [line.interpolate(line.length * index / count).coords[0] for index in range(count)]


def smooth_closed(points, iterations: int):
    smoothed = [tuple(point) for point in points]
    for _ in range(iterations):
        smoothed = [
            (
                0.25 * smoothed[index - 1][0]
                + 0.50 * smoothed[index][0]
                + 0.25 * smoothed[(index + 1) % len(smoothed)][0],
                0.25 * smoothed[index - 1][1]
                + 0.50 * smoothed[index][1]
                + 0.25 * smoothed[(index + 1) % len(smoothed)][1],
            )
            for index in range(len(smoothed))
        ]
    return smoothed


def jrk_target(command_mps: float) -> int:
    if command_mps <= CURRENT_SPEED_CAL[0][0]:
        return CURRENT_SPEED_CAL[0][1]
    if command_mps >= CURRENT_SPEED_CAL[-1][0]:
        return CURRENT_SPEED_CAL[-1][1]
    for (v0, t0), (v1, t1) in zip(CURRENT_SPEED_CAL, CURRENT_SPEED_CAL[1:]):
        if command_mps <= v1:
            fraction = (command_mps - v0) / (v1 - v0)
            return int(t0 + fraction * (t1 - t0) + 0.5)
    raise AssertionError("unreachable")


def main() -> None:
    args = arguments()
    if args.ring_number < 2:
        raise ValueError("--ring-number must be at least 2")
    boundary, to_ll = load_boundary(args.boundary_csv)
    inset_m = (args.ring_number - 1) * args.lane_spacing_m
    ring_polygon = boundary.buffer(-inset_m, join_style="round")
    if ring_polygon.is_empty or ring_polygon.geom_type != "Polygon":
        raise ValueError("Requested ring inset is empty or disconnected")
    ring_polygon = orient(ring_polygon, sign=-1.0)
    ring = rotate_ring_at_anchor(LineString(ring_polygon.exterior.coords), Point(boundary.exterior.coords[0]))
    lap_points = resample_closed(ring, args.waypoint_spacing_m)
    lap_points = smooth_closed(lap_points, args.smoothing_iterations)
    smoothed_ring = LineString(lap_points + [lap_points[0]])
    lap_points = resample_closed(smoothed_ring, args.waypoint_spacing_m)

    route = []
    lap_rows = []
    for lap_number, command_mps in enumerate(args.commands_mps, 1):
        start_index = len(route)
        for point in lap_points:
            route.append({"xy": point, "lap": lap_number, "speed": float(command_mps)})
        lap_rows.append({
            "lap": lap_number,
            "waypoint_start_zero_based": start_index,
            "waypoint_end_zero_based": len(route) - 1,
            "command_mps": float(command_mps),
            "expected_current_firmware_jrk_target": jrk_target(float(command_mps)),
        })

    args.output_dir.mkdir(parents=True, exist_ok=True)
    mission_path = args.output_dir / args.mission_name
    with mission_path.open("w", encoding="ascii", newline="\n") as handle:
        for index, row in enumerate(route):
            current = row["xy"]
            following = route[(index + 1) % len(route)]["xy"]
            yaw = math.atan2(following[1] - current[1], following[0] - current[0])
            lat, lon = to_ll(*current)
            handle.write(
                f"{lat:.9f} {lon:.9f} {yaw:.6f} {args.lookahead_m:.2f} {row['speed']:.2f}\n"
            )

    route_line = LineString([row["xy"] for row in route] + [route[0]["xy"]])
    start_lat, start_lon = to_ll(*route[0]["xy"])
    first_yaw = math.atan2(
        route[1]["xy"][1] - route[0]["xy"][1],
        route[1]["xy"][0] - route[0]["xy"][0],
    )
    report = {
        "status": "REVIEW_TEST",
        "purpose": "Measure actual ground speed at four JRK targets on the same 13th-ring path",
        "source_boundary_csv": str(args.boundary_csv.resolve()),
        "mission_file": str(mission_path.resolve()),
        "ring_number": args.ring_number,
        "inset_m": inset_m,
        "lap_length_m": smoothed_ring.length,
        "laps": lap_rows,
        "lookahead_m": args.lookahead_m,
        "waypoint_spacing_m": args.waypoint_spacing_m,
        "smoothing_iterations": args.smoothing_iterations,
        "waypoints": len(route),
        "route_length_m": route_line.length,
        "start_lat": start_lat,
        "start_lon": start_lon,
        "start_heading_compass_deg": (90.0 - math.degrees(first_yaw)) % 360.0,
        "important_limit": "JRK target 2288 is the confirmed full-forward endpoint; this mission does not command beyond it.",
        "mission_sha256": hashlib.sha256(mission_path.read_bytes()).hexdigest(),
    }
    report_path = args.output_dir / "speed_settings_ring13_report_20260831.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    bx, by = boundary.exterior.xy
    fig, ax = plt.subplots(figsize=(10, 9))
    ax.fill(bx, by, color="#eeeeee", alpha=0.55, label="Reviewed boundary")
    colors = ("#6a1b9a", "#1565c0", "#2e7d32", "#ef6c00")
    for lap in lap_rows:
        points = [row["xy"] for row in route if row["lap"] == lap["lap"]]
        points.append(points[0])
        ax.plot(
            [point[0] for point in points],
            [point[1] for point in points],
            color=colors[lap["lap"] - 1],
            linewidth=1.0 + 0.45 * lap["lap"],
            alpha=0.82,
            label=(f"Lap {lap['lap']}: command {lap['command_mps']:.2f} m/s "
                   f"-> JRK {lap['expected_current_firmware_jrk_target']}"),
        )
    ax.scatter(*route[0]["xy"], color="green", marker="*", s=100, label="Start / lap boundary")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("Ring 13 speed-settings test - four clockwise laps")
    ax.legend(loc="best")
    fig.tight_layout()
    preview_path = args.output_dir / "speed_settings_ring13_preview_20260831.png"
    fig.savefig(preview_path, dpi=180)
    plt.close(fig)

    print(f"Mission : {mission_path}")
    print(f"Preview : {preview_path}")
    print(f"Report  : {report_path}")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
