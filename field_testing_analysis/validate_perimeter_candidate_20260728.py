#!/usr/bin/env python3
"""Compare candidate perimeter geometry with the original mission and boundary."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from shapely.geometry import LineString, Point, Polygon


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--boundary", required=True, type=Path)
    parser.add_argument("--original-audit", required=True, type=Path)
    parser.add_argument("--candidate-audit", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--plot", type=Path)
    return parser.parse_args()


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def xy(records: list[dict[str, str]]) -> list[tuple[float, float]]:
    return [(float(row["east_m"]), float(row["north_m"])) for row in records]


def headland(records: list[dict[str, str]], pass_number: int):
    return [
        row
        for row in records
        if row["kind"] == "headland"
        and int(float(row["headland_pass"])) == pass_number
    ]


def point_distance_summary(
    points: list[tuple[float, float]], reference
) -> dict[str, float]:
    distances = [Point(point).distance(reference) for point in points]
    return {
        "minimum_m": min(distances),
        "mean_m": sum(distances) / len(distances),
        "maximum_m": max(distances),
        "within_0.05m_pct": 100.0
        * sum(value <= 0.05 for value in distances)
        / len(distances),
        "within_0.10m_pct": 100.0
        * sum(value <= 0.10 for value in distances)
        / len(distances),
    }


def finite_min_radius(records: list[dict[str, str]]) -> float | None:
    values = [
        float(row["local_radius_m"])
        for row in records
        if row.get("local_radius_m", "").strip()
        and math.isfinite(float(row["local_radius_m"]))
    ]
    return min(values) if values else None


def main() -> int:
    args = arguments()
    boundary_rows = rows(args.boundary)
    boundary_xy = xy(boundary_rows)
    boundary = Polygon(boundary_xy)
    original = rows(args.original_audit)
    candidate = rows(args.candidate_audit)
    original_outer = xy(headland(original, 1))
    original_second = xy(headland(original, 2))
    candidate_outer_rows = headland(candidate, 1)
    candidate_second_rows = headland(candidate, 2)
    candidate_outer = xy(candidate_outer_rows)
    candidate_second = xy(candidate_second_rows)
    route_line = LineString(xy(candidate))
    boundary_line = boundary.boundary

    pass_speeds: dict[str, list[float]] = {}
    for pass_number in (1, 2):
        values = sorted(
            {
                float(row["speed_mps"])
                for row in headland(candidate, pass_number)
            }
        )
        pass_speeds[str(pass_number)] = values

    report = {
        "candidate_route_contained_in_finalized_boundary": bool(
            boundary.buffer(0.03).covers(route_line)
        ),
        "candidate_route_outside_boundary_length_m": float(
            route_line.difference(boundary.buffer(0.03)).length
        ),
        "outer_pass_to_finalized_boundary": point_distance_summary(
            candidate_outer, boundary_line
        ),
        "original_outer_pass_to_finalized_boundary": point_distance_summary(
            original_outer, boundary_line
        ),
        "second_pass_to_finalized_boundary": point_distance_summary(
            candidate_second, boundary_line
        ),
        "candidate_vs_original_second_pass_hausdorff_m": float(
            LineString(candidate_second).hausdorff_distance(
                LineString(original_second)
            )
        ),
        "candidate_outer_vs_boundary_hausdorff_m": float(
            LineString(candidate_outer).hausdorff_distance(boundary_line)
        ),
        "headland_speeds_mps": pass_speeds,
        "minimum_local_radius_by_headland_pass_m": {
            "1": finite_min_radius(candidate_outer_rows),
            "2": finite_min_radius(candidate_second_rows),
        },
        "waypoints": len(candidate),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        from matplotlib import pyplot as plt

        fig, axis = plt.subplots(figsize=(10, 9))
        bx, by = boundary.exterior.xy
        axis.plot(bx, by, color="#212121", linewidth=2.0, label="Finalized boundary")
        ox, oy = zip(*original_outer)
        axis.plot(
            ox,
            oy,
            color="#9e9e9e",
            linewidth=1.6,
            linestyle="--",
            label="Original outer perimeter (~1.25 m inset)",
        )
        cx, cy = zip(*candidate_outer)
        axis.plot(
            cx,
            cy,
            color="#6a1b9a",
            linewidth=2.0,
            label="Candidate outer perimeter (boundary-following)",
        )
        sx, sy = zip(*candidate_second)
        axis.plot(
            sx,
            sy,
            color="#1565c0",
            linewidth=1.8,
            label="Second perimeter (unchanged)",
        )
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("East (m)")
        axis.set_ylabel("North (m)")
        axis.set_title("Perimeter planning comparison")
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best")
        fig.tight_layout()
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.plot, dpi=180)
        plt.close(fig)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
