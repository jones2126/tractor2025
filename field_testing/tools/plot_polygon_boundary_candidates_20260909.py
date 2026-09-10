#!/usr/bin/env python3
"""Plot the two closed-loop boundary candidates from the 2026-09-08 log."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D


@dataclass(frozen=True)
class Segment:
    label: str
    rows: list[dict[str, str]]


def number(row: dict[str, str], field: str) -> float:
    value = float(row[field])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {field}")
    return value


def distance_m(left: dict[str, str], right: dict[str, str]) -> float:
    lat1 = number(left, "lat")
    lon1 = number(left, "lon")
    lat2 = number(right, "lat")
    lon2 = number(right, "lon")
    north = (lat2 - lat1) * 110_540.0
    east = (lon2 - lon1) * 111_320.0 * math.cos(math.radians((lat1 + lat2) / 2.0))
    return math.hypot(east, north)


def pause_runs(rows: list[dict[str, str]]) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    start: int | None = None
    for index, row in enumerate(rows):
        paused = row.get("steer_mode") == "2" and row.get("steer_state") == "PAUSE"
        if paused and start is None:
            start = index
        elif not paused and start is not None:
            runs.append((start, index - 1))
            start = None
    if start is not None:
        runs.append((start, len(rows) - 1))
    return runs


def unique_positions(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    result: list[dict[str, str]] = []
    previous: tuple[float, float] | None = None
    for row in rows:
        try:
            position = (number(row, "lat"), number(row, "lon"))
        except (KeyError, TypeError, ValueError):
            continue
        if position != previous:
            result.append(row)
            previous = position
    return result


def local_xy(rows: list[dict[str, str]]) -> tuple[list[float], list[float]]:
    origin_lat = number(rows[0], "lat")
    origin_lon = number(rows[0], "lon")
    east = []
    north = []
    for row in rows:
        lat = number(row, "lat")
        lon = number(row, "lon")
        east.append((lon - origin_lon) * 111_320.0 * math.cos(math.radians(origin_lat)))
        north.append((lat - origin_lat) * 110_540.0)
    return east, north


def segment_metrics(segment: Segment) -> tuple[float, float, float]:
    path_length = sum(
        distance_m(left, right) for left, right in zip(segment.rows, segment.rows[1:])
    )
    closure = distance_m(segment.rows[0], segment.rows[-1])
    duration = number(segment.rows[-1], "elapsed_sec") - number(
        segment.rows[0], "elapsed_sec"
    )
    return path_length, closure, duration


def trim_start_for_best_closure(segment: Segment) -> tuple[Segment, int]:
    """Remove only the early approach/settling leg that best meets the finish."""
    search_end = max(2, len(segment.rows) // 3)
    finish = segment.rows[-1]
    start_index = min(
        range(search_end), key=lambda index: distance_m(segment.rows[index], finish)
    )
    return Segment(segment.label, segment.rows[start_index:]), start_index


def fix_color(fix_quality: str) -> str:
    if fix_quality == "RTK Fixed":
        return "#1769aa"
    if fix_quality == "RTK Float":
        return "#ed8b00"
    return "#c73e1d"


def draw_adjusted_rtk(axis: plt.Axes, segment: Segment) -> None:
    east, north = local_xy(segment.rows)
    path_length, closure, duration = segment_metrics(segment)
    points = list(zip(east, north))
    line_segments = [list(pair) for pair in zip(points, points[1:])]
    segment_colors = [
        fix_color(right.get("fix_quality", "")) for right in segment.rows[1:]
    ]
    axis.add_collection(
        LineCollection(
            line_segments,
            colors=segment_colors,
            linewidths=2.4,
            capstyle="round",
            joinstyle="round",
        )
    )
    axis.plot(
        [east[-1], east[0]],
        [north[-1], north[0]],
        color="#555555",
        linewidth=1.5,
        linestyle="--",
        label=f"Added closure ({closure:.2f} m)",
    )
    axis.scatter(
        east[0], north[0], marker="^", s=110, color="#20854e", zorder=4
    )
    axis.scatter(
        east[-1], north[-1], marker="X", s=100, color="#c73e1d", zorder=4
    )
    axis.set_title(
        f"{segment.label}\n"
        f"Adjusted path {path_length:.1f} m, {duration:.1f} s; "
        f"added closure {closure:.2f} m"
    )
    axis.set_xlabel("East from adjusted boundary start (m)")
    axis.set_ylabel("North from adjusted boundary start (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.autoscale()
    axis.margins(0.08)
    axis.grid(True, alpha=0.25)

    statuses = {row.get("fix_quality", "") for row in segment.rows}
    legend_items = []
    if "RTK Fixed" in statuses:
        legend_items.append(
            Line2D([0], [0], color=fix_color("RTK Fixed"), linewidth=2.4, label="RTK Fixed")
        )
    if "RTK Float" in statuses:
        legend_items.append(
            Line2D([0], [0], color=fix_color("RTK Float"), linewidth=2.4, label="RTK Float")
        )
    other_statuses = statuses - {"RTK Fixed", "RTK Float"}
    if other_statuses:
        legend_items.append(
            Line2D([0], [0], color=fix_color("Other"), linewidth=2.4, label="Other fix")
        )
    legend_items.extend(
        [
            Line2D([0], [0], color="#555555", linewidth=1.5, linestyle="--", label="Added closure"),
            Line2D([0], [0], marker="^", color="none", markerfacecolor="#20854e", markeredgecolor="#20854e", markersize=9, label="Adjusted start"),
            Line2D([0], [0], marker="X", color="none", markerfacecolor="#c73e1d", markeredgecolor="#c73e1d", markersize=9, label="Recorded finish"),
        ]
    )
    axis.legend(handles=legend_items, loc="best", fontsize=9)


def draw_segment(axis: plt.Axes, segment: Segment) -> None:
    east, north = local_xy(segment.rows)
    path_length, closure, duration = segment_metrics(segment)

    axis.plot(east, north, color="#1769aa", linewidth=2.0, label="Recorded boundary")
    axis.plot(
        [east[-1], east[0]],
        [north[-1], north[0]],
        color="#555555",
        linewidth=1.2,
        linestyle="--",
        label=f"Closure gap ({closure:.1f} m)",
    )
    axis.scatter(east[0], north[0], marker="^", s=90, color="#20854e", zorder=3, label="Start")
    axis.scatter(east[-1], north[-1], marker="X", s=90, color="#c73e1d", zorder=3, label="Finish")
    axis.set_title(
        f"{segment.label}\n{path_length:.1f} m path, {duration:.1f} s, {closure:.1f} m closure gap"
    )
    axis.set_xlabel("East from boundary start (m)")
    axis.set_ylabel("North from boundary start (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best", fontsize=8)


def save_single(segment: Segment, output: Path) -> None:
    figure, axis = plt.subplots(figsize=(8, 7))
    draw_segment(axis, segment)
    figure.tight_layout()
    figure.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("field_log", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    with args.field_log.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))

    pauses = pause_runs(rows)
    if len(pauses) != 6:
        raise SystemExit(f"ERROR: expected 6 Pause sections, found {len(pauses)}")

    driving_gaps = [
        rows[pauses[index][1] + 1 : pauses[index + 1][0]]
        for index in range(len(pauses) - 1)
    ]
    if len(driving_gaps) != 5:
        raise SystemExit(f"ERROR: expected 5 driving sections, found {len(driving_gaps)}")

    segments = [
        Segment("Polygon 2 candidate boundary", unique_positions(driving_gaps[1])),
        Segment("Polygon 3 candidate boundary", unique_positions(driving_gaps[3])),
    ]
    for segment in segments:
        if len(segment.rows) < 2:
            raise SystemExit(f"ERROR: {segment.label} has insufficient valid positions")
        if segment_metrics(segment)[1] > 3.0:
            raise SystemExit(f"ERROR: {segment.label} does not form the expected closed loop")

    adjusted_segments = []
    for segment in segments:
        adjusted, removed_points = trim_start_for_best_closure(segment)
        removed_seconds = number(adjusted.rows[0], "elapsed_sec") - number(
            segment.rows[0], "elapsed_sec"
        )
        print(
            f"{segment.label}: trimmed {removed_points} unique positions / "
            f"{removed_seconds:.2f} s; closure now {segment_metrics(adjusted)[1]:.3f} m"
        )
        adjusted_segments.append(adjusted)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    individual_paths = [
        args.output_dir / "polygon_2_candidate_boundary_20260908.png",
        args.output_dir / "polygon_3_candidate_boundary_20260908.png",
    ]
    for segment, output in zip(segments, individual_paths):
        save_single(segment, output)

    figure, axes = plt.subplots(1, 2, figsize=(14, 6.5))
    for axis, segment in zip(axes, segments):
        draw_segment(axis, segment)
    figure.suptitle("62 Collins polygon boundary candidates — 2026-09-08", fontsize=15)
    figure.tight_layout()
    combined = args.output_dir / "polygon_2_and_3_candidate_boundaries_20260908.png"
    figure.savefig(combined, dpi=180, bbox_inches="tight")
    plt.close(figure)

    for output in [*individual_paths, combined]:
        print(f"Created: {output}")

    adjusted_paths = [
        args.output_dir / "polygon_2_adjusted_boundary_rtk_20260908.png",
        args.output_dir / "polygon_3_adjusted_boundary_rtk_20260908.png",
    ]
    for segment, output in zip(adjusted_segments, adjusted_paths):
        figure, axis = plt.subplots(figsize=(9, 8))
        draw_adjusted_rtk(axis, segment)
        figure.tight_layout()
        figure.savefig(output, dpi=200, bbox_inches="tight")
        plt.close(figure)
        print(f"Created: {output}")


if __name__ == "__main__":
    main()
