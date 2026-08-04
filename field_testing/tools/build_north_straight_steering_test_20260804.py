#!/usr/bin/env python3
"""Build isolated east/west steering-test missions from a driven boundary log."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon


EARTH_EAST_M_PER_DEG = 111_320.0
EARTH_NORTH_M_PER_DEG = 110_540.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("boundary_log", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--start-row", type=int, default=3734)
    parser.add_argument("--end-row", type=int, default=8686)
    parser.add_argument("--safety-inset-m", type=float, default=3.0)
    parser.add_argument("--north-offsets-m", type=float, nargs="+", default=[4.0, 6.0])
    parser.add_argument("--waypoint-spacing-m", type=float, default=0.5)
    parser.add_argument("--lookahead-m", type=float, default=2.0)
    parser.add_argument("--speed-mps", type=float, default=0.50)
    return parser.parse_args()


def read_lap(path: Path, start_row: int, end_row: int) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    # User-facing data rows are one-based and exclude the CSV header.
    lap = rows[start_row - 1 : end_row]
    points = [r for r in lap if r.get("lat") and r.get("lon")]
    if len(points) < 3:
        raise ValueError("The selected lap contains fewer than three GPS positions")
    return points


def local_frame(rows: list[dict[str, str]]):
    ref_lat = float(rows[0]["lat"])
    ref_lon = float(rows[0]["lon"])
    east_scale = EARTH_EAST_M_PER_DEG * math.cos(math.radians(ref_lat))

    def to_xy(lat: float, lon: float) -> tuple[float, float]:
        return (lon - ref_lon) * east_scale, (lat - ref_lat) * EARTH_NORTH_M_PER_DEG

    def to_ll(x: float, y: float) -> tuple[float, float]:
        return ref_lat + y / EARTH_NORTH_M_PER_DEG, ref_lon + x / east_scale

    return ref_lat, ref_lon, to_xy, to_ll


def longest_line(geometry) -> LineString:
    lines = [geometry] if geometry.geom_type == "LineString" else list(geometry.geoms)
    lines = [line for line in lines if line.length > 0]
    if not lines:
        raise ValueError("No usable east/west intersection at the selected north offset")
    return max(lines, key=lambda line: line.length)


def spaced_points(start: tuple[float, float], end: tuple[float, float], spacing: float):
    length = math.dist(start, end)
    count = max(2, math.ceil(length / spacing) + 1)
    for index in range(count):
        fraction = index / (count - 1)
        yield (
            start[0] + fraction * (end[0] - start[0]),
            start[1] + fraction * (end[1] - start[1]),
        )


def write_mission(
    path: Path,
    start: tuple[float, float],
    end: tuple[float, float],
    to_ll,
    spacing: float,
    lookahead: float,
    speed: float,
) -> int:
    yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    points = list(spaced_points(start, end, spacing))
    with path.open("w", newline="\n", encoding="ascii") as handle:
        for x, y in points:
            lat, lon = to_ll(x, y)
            handle.write(f"{lat:.9f} {lon:.9f} {yaw:.6f} {lookahead:.2f} {speed:.2f}\n")
    return len(points)


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = read_lap(args.boundary_log, args.start_row, args.end_row)
    ref_lat, ref_lon, to_xy, to_ll = local_frame(rows)
    boundary_points = [to_xy(float(r["lat"]), float(r["lon"])) for r in rows]
    boundary = Polygon(boundary_points)
    if not boundary.is_valid:
        boundary = boundary.buffer(0)
    safe_area = boundary.buffer(-args.safety_inset_m, join_style="round")
    if safe_area.is_empty:
        raise ValueError("Safety inset removes the entire polygon")

    min_x, min_y, max_x, max_y = boundary.bounds
    tests = []
    colors = ["#1565c0", "#00897b", "#8e24aa", "#ef6c00"]
    fig, ax = plt.subplots(figsize=(12, 9))
    bx, by = boundary.exterior.xy
    ax.plot(bx, by, "--", color="#212121", linewidth=1.8, label="New driven path (all fixes)")
    sx, sy = safe_area.exterior.xy
    ax.fill(sx, sy, color="#e8f5e9", alpha=0.65, label=f"Area after {args.safety_inset_m:.1f} m safety inset")

    color_index = 0
    for line_index, north_offset in enumerate(args.north_offsets_m, start=1):
        y = max_y - north_offset
        probe = LineString([(min_x - 10.0, y), (max_x + 10.0, y)])
        segment = longest_line(safe_area.intersection(probe))
        west, east = sorted(segment.coords, key=lambda p: p[0])
        if math.dist(west, east) < 8.0:
            raise ValueError(f"Test line {line_index} is shorter than 8 m")

        for direction, start, end in (("east", west, east), ("west", east, west)):
            stem = f"north_line_{line_index}_{direction}_20260804"
            mission_path = args.output_dir / f"{stem}.txt"
            waypoint_count = write_mission(
                mission_path, start, end, to_ll,
                args.waypoint_spacing_m, args.lookahead_m, args.speed_mps,
            )
            start_lat, start_lon = to_ll(*start)
            end_lat, end_lon = to_ll(*end)
            tests.append({
                "id": f"{line_index}{direction[0].upper()}",
                "line": line_index,
                "direction": direction,
                "mission_file": mission_path.name,
                "length_m": math.dist(start, end),
                "waypoints": waypoint_count,
                "start_lat": start_lat,
                "start_lon": start_lon,
                "end_lat": end_lat,
                "end_lon": end_lon,
                "yaw_math_rad": math.atan2(end[1] - start[1], end[0] - start[0]),
                "speed_mps": args.speed_mps,
                "lookahead_m": args.lookahead_m,
            })
            color = colors[color_index % len(colors)]
            color_index += 1
            ax.annotate(
                "",
                xy=end,
                xytext=start,
                arrowprops=dict(arrowstyle="->", color=color, linewidth=2.5),
            )
            midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
            ax.text(*midpoint, f" {line_index}{direction[0].upper()}", color=color,
                    fontsize=10, fontweight="bold")

    manifest = {
        "source_boundary_log": str(args.boundary_log.resolve()),
        "source_data_rows": [args.start_row, args.end_row],
        "boundary_fix_quality_note": (
            "Geometry uses the complete driven lap, including RTK Float/DGPS gaps. "
            "Test endpoints are conservatively inset and require field review."
        ),
        "ref_lat": ref_lat,
        "ref_lon": ref_lon,
        "safety_inset_m": args.safety_inset_m,
        "north_offsets_m": args.north_offsets_m,
        "tests": tests,
    }
    manifest_path = args.output_dir / "north_straight_tests_20260804.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("East of first lap point (m)")
    ax.set_ylabel("North of first lap point (m)")
    ax.set_title(
        "Northern-slope east/west steering tests\n"
        "Each arrow is a separate mission; Pause and reposition manually between runs"
    )
    ax.legend(loc="lower left")
    fig.tight_layout()
    preview_path = args.output_dir / "north_straight_tests_20260804.png"
    fig.savefig(preview_path, dpi=180)
    plt.close(fig)

    print(f"Boundary log : {args.boundary_log}")
    print(f"Manifest     : {manifest_path}")
    print(f"Preview      : {preview_path}")
    for test in tests:
        print(
            f"{test['id']}: {test['direction']:4s} {test['length_m']:.1f} m, "
            f"{test['waypoints']} waypoints -> {test['mission_file']}"
        )


if __name__ == "__main__":
    main()
