#!/usr/bin/env python3
"""Compare Ring 13 tracking error and heading validity by intended speed."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


STAGES = (
    {"speed": 0.75, "first_wp": 0, "last_wp": 66, "route": "approach"},
    {"speed": 1.00, "first_wp": 67, "last_wp": 212, "route": "ring lap 1"},
    {"speed": 1.20, "first_wp": 213, "last_wp": 359, "route": "ring lap 2"},
    {"speed": 1.50, "first_wp": 360, "last_wp": 506, "route": "ring lap 3"},
)
PROGRESS_BINS = 20


def as_float(value: str | None) -> float | None:
    try:
        result = float(value or "")
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def as_int(value: str | None) -> int | None:
    result = as_float(value)
    return int(result) if result is not None else None


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def describe(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "median": None, "p90": None,
                "p95": None, "max": None}
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def pearson(x: list[float], y: list[float]) -> float | None:
    if len(x) != len(y) or len(x) < 2:
        return None
    x_mean = statistics.fmean(x)
    y_mean = statistics.fmean(y)
    numerator = sum((a - x_mean) * (b - y_mean) for a, b in zip(x, y))
    denominator = math.sqrt(
        sum((a - x_mean) ** 2 for a in x)
        * sum((b - y_mean) ** 2 for b in y)
    )
    return numerator / denominator if denominator else None


def invalid_runs(rows: list[dict[str, str]], cycle_seconds: float) -> list[float]:
    runs: list[int] = []
    current = 0
    previous_cycle: int | None = None
    for row in rows:
        cycle = as_int(row.get("cycle"))
        invalid = row.get("head_valid") != "True"
        contiguous = previous_cycle is not None and cycle == previous_cycle + 1
        if invalid:
            if current and not contiguous:
                runs.append(current)
                current = 0
            current += 1
        elif current:
            runs.append(current)
            current = 0
        previous_cycle = cycle
    if current:
        runs.append(current)
    return [length * cycle_seconds for length in runs]


def load_pursuit(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    return [row for row in rows if as_float(row.get("elapsed_s")) is not None]


def load_field(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def stage_rows(
    rows: list[dict[str, str]], first_wp: int, last_wp: int
) -> list[dict[str, str]]:
    selected = []
    for row in rows:
        waypoint = as_int(row.get("waypoint_idx"))
        if waypoint is not None and first_wp <= waypoint <= last_wp:
            selected.append(row)
    return selected


def field_stage_windows(rows: list[dict[str, str]]) -> dict[float, tuple[float, float]]:
    windows = {}
    for stage in STAGES:
        speed = stage["speed"]
        times = [
            as_float(row.get("elapsed_sec"))
            for row in rows
            if as_float(row.get("trans_cmd_vel_mps")) is not None
            and math.isclose(as_float(row.get("trans_cmd_vel_mps")) or -1, speed,
                             abs_tol=0.001)
        ]
        times = [value for value in times if value is not None]
        if times:
            windows[speed] = (min(times), max(times))
    return windows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("pursuit_log", type=Path)
    parser.add_argument("field_log", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    pursuit = load_pursuit(args.pursuit_log)
    field = load_field(args.field_log)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    cycle_deltas = []
    for left, right in zip(pursuit, pursuit[1:]):
        a = as_float(left.get("elapsed_s"))
        b = as_float(right.get("elapsed_s"))
        if a is not None and b is not None and 0 < b - a < 0.2:
            cycle_deltas.append(b - a)
    cycle_seconds = statistics.median(cycle_deltas)

    windows = field_stage_windows(field)
    summaries = []
    progress_medians: dict[float, dict[int, float]] = {}
    cte_by_speed: dict[float, list[float]] = {}

    for stage in STAGES:
        speed = float(stage["speed"])
        rows = stage_rows(pursuit, int(stage["first_wp"]), int(stage["last_wp"]))
        driving = [row for row in rows if row.get("driving") == "True"]
        cte = [as_float(row.get("cross_track_err_m")) for row in driving]
        cte = [value for value in cte if value is not None]
        signed = [as_float(row.get("yt_m")) for row in driving]
        signed = [value for value in signed if value is not None]
        steer = [as_float(row.get("steer_normalized")) for row in driving]
        steer = [value for value in steer if value is not None]
        cte_by_speed[speed] = cte

        heading_valid = [row.get("head_valid") == "True" for row in rows]
        outages = invalid_runs(rows, cycle_seconds)
        rtk_fixed = [row.get("fix_quality") == "RTK Fixed" for row in rows]

        invalid_progress_bins: dict[int, int] = {}
        span = max(1, int(stage["last_wp"]) - int(stage["first_wp"]))
        for row in rows:
            if row.get("head_valid") == "True":
                continue
            waypoint = as_int(row.get("waypoint_idx"))
            if waypoint is None:
                continue
            progress = (waypoint - int(stage["first_wp"])) / span
            bin_index = min(PROGRESS_BINS - 1, max(0, int(progress * PROGRESS_BINS)))
            invalid_progress_bins[bin_index] = invalid_progress_bins.get(bin_index, 0) + 1

        bin_values: dict[int, list[float]] = {}
        for row in driving:
            waypoint = as_int(row.get("waypoint_idx"))
            value = as_float(row.get("cross_track_err_m"))
            if waypoint is None or value is None:
                continue
            progress = (waypoint - int(stage["first_wp"])) / span
            bin_index = min(PROGRESS_BINS - 1, max(0, int(progress * PROGRESS_BINS)))
            bin_values.setdefault(bin_index, []).append(value)
        progress_medians[speed] = {
            index: statistics.median(values) for index, values in bin_values.items()
        }

        field_signal = {}
        if speed in windows:
            first_s, last_s = windows[speed]
            signal_rows = [
                row for row in field
                if as_float(row.get("elapsed_sec")) is not None
                and first_s <= (as_float(row.get("elapsed_sec")) or -1) <= last_s
            ]
            for column in ("heading_numSV_used", "heading_numSV_visible",
                           "heading_cno_mean_dbhz", "base_numSV_used",
                           "base_cno_mean_dbhz", "diff_age"):
                values = [as_float(row.get(column)) for row in signal_rows]
                values = [value for value in values if value is not None]
                field_signal[column] = {
                    "median": statistics.median(values) if values else None,
                    "min": min(values) if values else None,
                    "max": max(values) if values else None,
                }

        summaries.append(
            {
                "intended_speed_mps": speed,
                "route": stage["route"],
                "cycle_count": len(rows),
                "stage_duration_s": len(rows) * cycle_seconds,
                "driving_cycle_percent": 100.0 * len(driving) / len(rows),
                "cross_track_error_m": describe(cte),
                "signed_lateral_error_m": describe(signed),
                "cte_at_or_below_0p25m_percent": 100.0 * sum(v <= 0.25 for v in cte) / len(cte),
                "cte_above_0p50m_percent": 100.0 * sum(v > 0.50 for v in cte) / len(cte),
                "steering_saturated_percent": 100.0 * sum(abs(v) >= 0.999 for v in steer) / len(steer),
                "heading_valid_percent": 100.0 * sum(heading_valid) / len(rows),
                "heading_invalid_total_s": sum(outages),
                "heading_invalid_event_count": len(outages),
                "heading_invalid_longest_s": max(outages) if outages else 0.0,
                "heading_invalid_progress_bin_counts": invalid_progress_bins,
                "rtk_fixed_percent": 100.0 * sum(rtk_fixed) / len(rows),
                "fix_quality_wait_s": sum(
                    row.get("wait_reason", "").startswith("fix_quality=") for row in rows
                ) * cycle_seconds,
                "heading_signal": field_signal,
                "progress_bin_median_cte_m": progress_medians[speed],
            }
        )

    ring_summaries = [row for row in summaries if row["route"].startswith("ring")]
    ring_speeds = [row["intended_speed_mps"] for row in ring_summaries]
    mean_cte = [row["cross_track_error_m"]["mean"] for row in ring_summaries]
    median_cte = [row["cross_track_error_m"]["median"] for row in ring_summaries]
    p95_cte = [row["cross_track_error_m"]["p95"] for row in ring_summaries]
    invalid_percent = [100.0 - row["heading_valid_percent"] for row in ring_summaries]

    common_bins = sorted(set.intersection(*(
        set(progress_medians[speed]) for speed in ring_speeds
    )))
    differences_15_vs_10 = [
        progress_medians[1.50][index] - progress_medians[1.00][index]
        for index in common_bins
    ]
    comparison = {
        "fair_comparison_speeds_mps": ring_speeds,
        "approach_0p75_excluded_from_speed_correlation": True,
        "aggregate_pearson_speed_vs_mean_cte": pearson(ring_speeds, mean_cte),
        "aggregate_pearson_speed_vs_median_cte": pearson(ring_speeds, median_cte),
        "aggregate_pearson_speed_vs_p95_cte": pearson(ring_speeds, p95_cte),
        "aggregate_pearson_speed_vs_heading_invalid_percent": pearson(
            ring_speeds, invalid_percent
        ),
        "aggregate_point_count_warning": 3,
        "matched_progress_bin_count": len(common_bins),
        "matched_progress_median_cte_difference_1p5_minus_1p0_m": (
            statistics.median(differences_15_vs_10) if differences_15_vs_10 else None
        ),
        "matched_progress_bins_worse_at_1p5_count": sum(
            difference > 0 for difference in differences_15_vs_10
        ),
    }

    json_path = args.output_dir / "ring13_tracking_heading_analysis_20260908.json"
    json_path.write_text(
        json.dumps({"stages": summaries, "comparison": comparison}, indent=2) + "\n",
        encoding="utf-8",
    )

    ring_labels = [f"{speed:.2f}" for speed in ring_speeds]
    figure, (distribution, trend) = plt.subplots(1, 2, figsize=(12, 5))
    distribution.boxplot(
        [cte_by_speed[speed] for speed in ring_speeds],
        tick_labels=ring_labels,
        showfliers=False,
    )
    distribution.set(
        title="Cross-track error distribution on repeated ring laps",
        xlabel="Intended speed (m/s)",
        ylabel="Absolute cross-track error (m)",
    )
    distribution.grid(True, axis="y", alpha=0.25)

    trend.plot(ring_speeds, mean_cte, "o-", label="Mean")
    trend.plot(ring_speeds, median_cte, "o-", label="Median")
    trend.plot(ring_speeds, p95_cte, "o-", label="95th percentile")
    trend.set(
        title="Cross-track error versus speed",
        xlabel="Intended speed (m/s)",
        ylabel="Absolute cross-track error (m)",
        xticks=ring_speeds,
    )
    trend.grid(True, alpha=0.25)
    trend.legend()
    figure.tight_layout()
    tracking_plot = args.output_dir / "ring13_cross_track_vs_speed_20260908.png"
    figure.savefig(tracking_plot, dpi=180)
    plt.close(figure)

    figure, (validity, signal, location) = plt.subplots(1, 3, figsize=(15, 5))
    validity.plot(
        [row["intended_speed_mps"] for row in summaries],
        [row["heading_valid_percent"] for row in summaries],
        "o-", color="#1b75bc", label="Heading valid",
    )
    validity.plot(
        [row["intended_speed_mps"] for row in summaries],
        [row["rtk_fixed_percent"] for row in summaries],
        "o-", color="#2a9d55", label="RTK Fixed",
    )
    validity.set(
        title="Navigation validity by intended speed",
        xlabel="Intended speed (m/s)",
        ylabel="Valid cycles (%)",
        ylim=(90, 100.3),
        xticks=[row["intended_speed_mps"] for row in summaries],
    )
    validity.grid(True, alpha=0.25)
    validity.legend()

    signal.plot(
        [row["intended_speed_mps"] for row in summaries],
        [row["heading_signal"]["heading_cno_mean_dbhz"]["median"] for row in summaries],
        "o-", color="#8055a5", label="Heading F9P mean C/N0",
    )
    signal.set(
        title="Heading receiver signal strength",
        xlabel="Intended speed (m/s)",
        ylabel="Median mean C/N0 (dB-Hz)",
        xticks=[row["intended_speed_mps"] for row in summaries],
    )
    signal.grid(True, alpha=0.25)
    signal.legend()

    for row in ring_summaries:
        for bin_index, count in row["heading_invalid_progress_bin_counts"].items():
            location.scatter(
                (int(bin_index) + 0.5) * 100.0 / PROGRESS_BINS,
                row["intended_speed_mps"],
                s=18 + 4 * count,
                alpha=0.65,
                color="#d95f02",
                edgecolors="none",
            )
    location.set(
        title="Heading-invalid cycles around the ring",
        xlabel="Progress around ring lap (%)",
        ylabel="Intended speed (m/s)",
        yticks=ring_speeds,
        xlim=(0, 100),
    )
    location.grid(True, alpha=0.25)
    location.text(
        0.02, 0.02, "Bubble area increases with outage cycles",
        transform=location.transAxes, fontsize=8,
    )
    figure.tight_layout()
    heading_plot = args.output_dir / "ring13_heading_validity_vs_speed_20260908.png"
    figure.savefig(heading_plot, dpi=180)
    plt.close(figure)

    print(f"Pursuit cycles: {len(pursuit)}; cycle interval: {cycle_seconds:.3f}s")
    for row in summaries:
        cte = row["cross_track_error_m"]
        print(
            f"{row['intended_speed_mps']:.2f} m/s {row['route']}: "
            f"CTE median={cte['median']:.3f} mean={cte['mean']:.3f} "
            f"p95={cte['p95']:.3f}; heading valid={row['heading_valid_percent']:.2f}% "
            f"invalid={row['heading_invalid_total_s']:.2f}s"
        )
    print(f"Wrote {json_path}")
    print(f"Wrote {tracking_plot}")
    print(f"Wrote {heading_plot}")


if __name__ == "__main__":
    main()
