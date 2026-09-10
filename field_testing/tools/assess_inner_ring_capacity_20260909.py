#!/usr/bin/env python3
"""Assess complete inward-ring capacity using the production planner rules."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.geometry import Polygon


SCRIPT_DIR = Path(__file__).resolve().parent
PLANNER_DIR = SCRIPT_DIR.parents[1] / "tractor_rpi" / "pure-pursuit" / "mission_planning"
sys.path.insert(0, str(PLANNER_DIR))

from site_planner_common_20260724 import load_boundary  # noqa: E402


def polygon_parts(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == "Polygon":
        return [geometry]
    if geometry.geom_type == "MultiPolygon":
        return list(geometry.geoms)
    return []


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def assess(
    boundary: Polygon,
    *,
    spacing_m: float,
    turn_radius_m: float,
    boundary_outset_m: float,
    maximum_passes: int,
):
    drive_area = boundary.buffer(boundary_outset_m, join_style="round")
    passing = []
    first_failure = None

    for pass_index in range(maximum_passes):
        inset_m = pass_index * spacing_m
        geometry = (
            boundary
            if pass_index == 0
            else boundary.buffer(-inset_m, join_style="round").intersection(drive_area)
        )
        if geometry.is_empty:
            first_failure = {
                "pass": pass_index + 1,
                "inset_m": inset_m,
                "reason": "inward offset is empty",
                "raw_geometry": geometry,
            }
            break

        raw_parts = polygon_parts(geometry)
        if len(raw_parts) != 1:
            first_failure = {
                "pass": pass_index + 1,
                "inset_m": inset_m,
                "reason": f"inward offset has {len(raw_parts)} components",
                "raw_geometry": geometry,
            }
            break

        eroded = geometry.buffer(-turn_radius_m, join_style="round")
        if eroded.is_empty:
            first_failure = {
                "pass": pass_index + 1,
                "inset_m": inset_m,
                "reason": (
                    f"cannot be rounded inward with the {turn_radius_m:.2f} m "
                    "turn radius"
                ),
                "raw_geometry": geometry,
            }
            break

        rounded = eroded.buffer(turn_radius_m, join_style="round")
        rounded_parts = polygon_parts(rounded)
        if len(rounded_parts) != 1:
            first_failure = {
                "pass": pass_index + 1,
                "inset_m": inset_m,
                "reason": f"turn-radius rounding creates {len(rounded_parts)} components",
                "raw_geometry": geometry,
            }
            break

        ring = rounded_parts[0].exterior
        passing.append(
            {
                "pass": pass_index + 1,
                "inset_m": inset_m,
                "centerline_length_m": ring.length,
                "rounded_area_m2": rounded_parts[0].area,
                "geometry": rounded_parts[0],
            }
        )

    return drive_area, passing, first_failure


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("boundary_csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--site-label", required=True)
    parser.add_argument("--lane-spacing-m", type=float, default=0.9652)
    parser.add_argument("--turn-radius-m", type=float, default=1.90)
    parser.add_argument("--boundary-outset-m", type=float, default=0.381)
    parser.add_argument("--maximum-passes", type=int, default=12)
    args = parser.parse_args()

    _frame, boundary_xy, _rows = load_boundary(args.boundary_csv)
    boundary = Polygon(boundary_xy)
    if not boundary.is_valid or boundary.is_empty:
        raise SystemExit("ERROR: boundary is not a valid polygon")

    drive_area, passing, failure = assess(
        boundary,
        spacing_m=args.lane_spacing_m,
        turn_radius_m=args.turn_radius_m,
        boundary_outset_m=args.boundary_outset_m,
        maximum_passes=args.maximum_passes,
    )
    if not passing:
        raise SystemExit("ERROR: even the manually driven outer pass is not feasible")
    if failure is None:
        raise SystemExit("ERROR: assessment did not reach a failing inward pass")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "site": args.site_label,
        "boundary_csv": str(args.boundary_csv.resolve()),
        "boundary_sha256": sha256(args.boundary_csv),
        "boundary_area_m2": boundary.area,
        "boundary_perimeter_m": boundary.length,
        "lane_spacing_m": args.lane_spacing_m,
        "lane_spacing_in": args.lane_spacing_m / 0.0254,
        "turn_radius_m": args.turn_radius_m,
        "boundary_outset_m": args.boundary_outset_m,
        "outer_boundary_is_pass_1": True,
        "passing_total_passes": len(passing),
        "passing_inner_rings": max(0, len(passing) - 1),
        "passing": [
            {key: value for key, value in item.items() if key != "geometry"}
            for item in passing
        ],
        "first_failure": {
            key: value for key, value in failure.items() if key != "raw_geometry"
        },
    }
    report_path = args.output_dir / "inner_ring_capacity_38in_20260909.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    figure, axis = plt.subplots(figsize=(9, 8))
    boundary_x, boundary_y = boundary.exterior.xy
    axis.fill(boundary_x, boundary_y, color="#cfd8dc", alpha=0.28, label="Final boundary")
    axis.plot(boundary_x, boundary_y, color="#607d8b", linewidth=1.1)

    colors = ["#1565c0", "#2e7d32", "#6a1b9a", "#00838f", "#ad5c00"]
    for index, item in enumerate(passing):
        x, y = item["geometry"].exterior.xy
        label = "Pass 1 — boundary-following" if item["pass"] == 1 else (
            f"Pass {item['pass']} — inner ring {item['pass'] - 1} "
            f"({item['inset_m']:.2f} m inset)"
        )
        axis.plot(x, y, color=colors[index % len(colors)], linewidth=2.0, label=label)

    failed_geometry = failure["raw_geometry"]
    for part_index, part in enumerate(polygon_parts(failed_geometry)):
        x, y = part.exterior.xy
        axis.plot(
            x,
            y,
            color="#c62828",
            linewidth=1.7,
            linestyle="--",
            label=(
                f"Pass {failure['pass']} raw offset — rejected"
                if part_index == 0
                else None
            ),
        )

    inner_count = max(0, len(passing) - 1)
    ring_word = "ring" if inner_count == 1 else "rings"
    axis.set_title(
        f"{args.site_label}: 38-inch inward-ring capacity\n"
        f"{len(passing)} total passes = {inner_count} inner {ring_word}; "
        f"pass {failure['pass']} fails {args.turn_radius_m:.2f} m turn-radius rule"
    )
    axis.set_xlabel("East of boundary anchor (m)")
    axis.set_ylabel("North of boundary anchor (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best", fontsize=9)
    figure.tight_layout()
    plot_path = args.output_dir / "inner_ring_capacity_38in_20260909.png"
    figure.savefig(plot_path, dpi=190, bbox_inches="tight")
    plt.close(figure)

    print(f"Site                 : {args.site_label}")
    print(f"Passing total passes : {len(passing)}")
    print(f"Passing inner rings  : {max(0, len(passing) - 1)}")
    for item in passing:
        print(
            f"  pass {item['pass']}: inset={item['inset_m']:.4f} m; "
            f"centerline={item['centerline_length_m']:.1f} m"
        )
    print(
        f"First failure        : pass {failure['pass']} at "
        f"{failure['inset_m']:.4f} m — {failure['reason']}"
    )
    print(f"Report               : {report_path}")
    print(f"Plot                 : {plot_path}")


if __name__ == "__main__":
    main()
