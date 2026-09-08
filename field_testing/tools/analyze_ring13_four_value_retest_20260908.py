#!/usr/bin/env python3
"""Analyze the approved 2026-09-08 Ring 13 four-value retest field log."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


EXPECTED_TARGETS = {0.75: 2350, 1.00: 2300, 1.20: 2246, 1.50: 2160}
TRIM_SECONDS = 5.0


def number(row: dict[str, str], name: str) -> float | None:
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


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


def stats(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "stdev": statistics.stdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "p05": percentile(values, 0.05),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def mode_int(rows: list[dict[str, str]], name: str) -> int | None:
    values = []
    for row in rows:
        value = number(row, name)
        if value is not None:
            values.append(int(value))
    return Counter(values).most_common(1)[0][0] if values else None


def contiguous_runs(
    indexed_rows: list[tuple[int, dict[str, str]]],
) -> list[list[tuple[int, dict[str, str]]]]:
    runs: list[list[tuple[int, dict[str, str]]]] = []
    current: list[tuple[int, dict[str, str]]] = []
    previous_index: int | None = None
    for index, row in indexed_rows:
        if previous_index is None or index == previous_index + 1:
            current.append((index, row))
        else:
            if current:
                runs.append(current)
            current = [(index, row)]
        previous_index = index
    if current:
        runs.append(current)
    return runs


def elapsed(row: dict[str, str]) -> float:
    value = number(row, "elapsed_sec")
    if value is None:
        raise ValueError("field log contains an invalid elapsed_sec")
    return value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("field_log", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    with args.field_log.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summaries = []

    for command, expected_target in EXPECTED_TARGETS.items():
        command_rows = [
            (index, row)
            for index, row in enumerate(rows)
            if number(row, "trans_cmd_vel_mps") is not None
            and math.isclose(number(row, "trans_cmd_vel_mps") or -1, command, abs_tol=0.001)
        ]

        eligible = []
        for index, row in command_rows:
            requested = number(row, "jrk_target")
            actual = number(row, "jrk_actual_target")
            valid = number(row, "jrk_valid")
            speed = number(row, "speed_mps")
            mode = number(row, "trans_mode")
            if (
                requested == expected_target
                and actual == expected_target
                and valid == 1
                and speed is not None
                and mode == 0
            ):
                eligible.append((index, row))

        runs = contiguous_runs(eligible)
        steady_rows: list[dict[str, str]] = []
        retained_runs = []
        for run in runs:
            start = elapsed(run[0][1])
            end = elapsed(run[-1][1])
            retained = [
                row
                for _, row in run
                if elapsed(row) >= start + TRIM_SECONDS
                and elapsed(row) <= end - TRIM_SECONDS
            ]
            if retained:
                steady_rows.extend(retained)
                retained_runs.append(
                    {
                        "eligible_start_s": start,
                        "eligible_end_s": end,
                        "eligible_duration_s": end - start,
                        "steady_start_s": elapsed(retained[0]),
                        "steady_end_s": elapsed(retained[-1]),
                        "steady_samples": len(retained),
                    }
                )

        speeds = [number(row, "speed_mps") for row in steady_rows]
        speeds = [value for value in speeds if value is not None]
        feedback = [number(row, "jrk_scaled_feedback") for row in steady_rows]
        feedback = [value for value in feedback if value is not None]
        duty_target = [number(row, "jrk_duty_cycle_target") for row in steady_rows]
        duty_target = [value for value in duty_target if value is not None]
        duty_applied = [number(row, "jrk_duty_cycle") for row in steady_rows]
        duty_applied = [value for value in duty_applied if value is not None]

        all_stage_rows = [row for _, row in command_rows]
        all_duty_target = [number(row, "jrk_duty_cycle_target") for row in all_stage_rows]
        all_duty_target = [value for value in all_duty_target if value is not None]
        all_duty_applied = [number(row, "jrk_duty_cycle") for row in all_stage_rows]
        all_duty_applied = [value for value in all_duty_applied if value is not None]

        mode9_rows = [
            row for row in all_stage_rows if number(row, "trans_mode") == 9
        ]
        errors = [
            int(number(row, "jrk_errors_halting") or 0) for row in all_stage_rows
        ]
        timeouts = [int(number(row, "jrk_timeouts") or 0) for row in all_stage_rows]
        speed_summary = stats(speeds)
        median_speed = speed_summary.get("median", math.nan)
        mean_speed = speed_summary.get("mean", math.nan)

        summaries.append(
            {
                "command_mps": command,
                "expected_jrk_target": expected_target,
                "observed_requested_target_mode": mode_int(steady_rows, "jrk_target"),
                "observed_actual_target_mode": mode_int(steady_rows, "jrk_actual_target"),
                "command_first_s": elapsed(all_stage_rows[0]) if all_stage_rows else None,
                "command_last_s": elapsed(all_stage_rows[-1]) if all_stage_rows else None,
                "command_samples": len(all_stage_rows),
                "steady_runs": retained_runs,
                "steady_duration_s": sum(
                    run["steady_end_s"] - run["steady_start_s"] for run in retained_runs
                ),
                "speed_mps": speed_summary,
                "median_minus_command_mps": median_speed - command,
                "median_percent_of_command": median_speed / command * 100.0,
                "mean_minus_command_mps": mean_speed - command,
                "mean_percent_of_command": mean_speed / command * 100.0,
                "scaled_feedback": stats(feedback),
                "feedback_minus_target_median": (
                    statistics.median(feedback) - expected_target if feedback else None
                ),
                "steady_duty_cycle_target": stats(duty_target),
                "steady_duty_cycle_applied": stats(duty_applied),
                "steady_nonzero_duty_target_percent": (
                    100.0 * sum(value != 0 for value in duty_target) / len(duty_target)
                    if duty_target else None
                ),
                "steady_nonzero_duty_applied_percent": (
                    100.0 * sum(value != 0 for value in duty_applied) / len(duty_applied)
                    if duty_applied else None
                ),
                "whole_stage_peak_abs_duty_target": (
                    max(map(abs, all_duty_target)) if all_duty_target else None
                ),
                "whole_stage_peak_abs_duty_applied": (
                    max(map(abs, all_duty_applied)) if all_duty_applied else None
                ),
                "radio_loss_mode9_samples": len(mode9_rows),
                "radio_loss_mode9_approx_s": len(mode9_rows) / 20.0,
                "maximum_jrk_halting_error_bits": max(errors) if errors else None,
                "jrk_timeout_counter_increase": (
                    max(timeouts) - min(timeouts) if timeouts else None
                ),
            }
        )

    json_path = args.output_dir / "ring13_four_value_retest_analysis_20260908.json"
    json_path.write_text(json.dumps(summaries, indent=2) + "\n", encoding="utf-8")

    csv_path = args.output_dir / "ring13_four_value_retest_summary_20260908.csv"
    columns = [
        "command_mps", "expected_jrk_target", "observed_actual_target_mode",
        "steady_duration_s", "speed_mean_mps", "speed_median_mps",
        "speed_stdev_mps", "speed_p05_mps", "speed_p95_mps",
        "median_percent_of_command", "feedback_median",
        "feedback_minus_target_median", "steady_duty_target_median",
        "steady_duty_applied_median", "whole_stage_peak_abs_duty_target",
        "whole_stage_peak_abs_duty_applied", "radio_loss_mode9_approx_s",
        "maximum_jrk_halting_error_bits", "jrk_timeout_counter_increase",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for summary in summaries:
            writer.writerow(
                {
                    "command_mps": summary["command_mps"],
                    "expected_jrk_target": summary["expected_jrk_target"],
                    "observed_actual_target_mode": summary["observed_actual_target_mode"],
                    "steady_duration_s": summary["steady_duration_s"],
                    "speed_mean_mps": summary["speed_mps"].get("mean"),
                    "speed_median_mps": summary["speed_mps"].get("median"),
                    "speed_stdev_mps": summary["speed_mps"].get("stdev"),
                    "speed_p05_mps": summary["speed_mps"].get("p05"),
                    "speed_p95_mps": summary["speed_mps"].get("p95"),
                    "median_percent_of_command": summary["median_percent_of_command"],
                    "feedback_median": summary["scaled_feedback"].get("median"),
                    "feedback_minus_target_median": summary["feedback_minus_target_median"],
                    "steady_duty_target_median": summary["steady_duty_cycle_target"].get("median"),
                    "steady_duty_applied_median": summary["steady_duty_cycle_applied"].get("median"),
                    "whole_stage_peak_abs_duty_target": summary["whole_stage_peak_abs_duty_target"],
                    "whole_stage_peak_abs_duty_applied": summary["whole_stage_peak_abs_duty_applied"],
                    "radio_loss_mode9_approx_s": summary["radio_loss_mode9_approx_s"],
                    "maximum_jrk_halting_error_bits": summary["maximum_jrk_halting_error_bits"],
                    "jrk_timeout_counter_increase": summary["jrk_timeout_counter_increase"],
                }
            )

    times = [number(row, "elapsed_sec") for row in rows]
    actual_speeds = [number(row, "speed_mps") for row in rows]
    commands = [number(row, "trans_cmd_vel_mps") for row in rows]
    mode9 = [number(row, "trans_mode") == 9 for row in rows]
    heading_invalid = [row.get("head_valid", "").lower() != "true" for row in rows]

    figure, (timeline, calibration) = plt.subplots(
        2, 1, figsize=(12, 8), gridspec_kw={"height_ratios": [2.1, 1]}
    )
    timeline.plot(times, commands, color="#d95f02", linewidth=1.4, label="Commanded speed")
    timeline.plot(times, actual_speeds, color="#1b75bc", linewidth=1.0, label="GPS actual speed")
    timeline.fill_between(
        times, 0, 1.7, where=mode9, color="#d62728", alpha=0.18,
        label="Teensy radio-loss safety mode",
    )
    timeline.fill_between(
        times, 0, 1.7, where=heading_invalid, color="#ffbf00", alpha=0.30,
        label="Heading invalid",
    )
    timeline.set(
        title="Ring 13 four-value calibration retest — 2026-09-08",
        xlabel="Field logger elapsed time (s)",
        ylabel="Speed (m/s)",
        ylim=(0, 1.7),
    )
    timeline.grid(True, alpha=0.25)
    timeline.legend(loc="upper left", ncol=2, fontsize=9)

    x_values = [summary["command_mps"] for summary in summaries]
    medians = [summary["speed_mps"]["median"] for summary in summaries]
    lower = [
        summary["speed_mps"]["median"] - summary["speed_mps"]["p05"]
        for summary in summaries
    ]
    upper = [
        summary["speed_mps"]["p95"] - summary["speed_mps"]["median"]
        for summary in summaries
    ]
    calibration.plot([0, 1.6], [0, 1.6], "--", color="#777777", label="Command = actual")
    calibration.errorbar(
        x_values, medians, yerr=[lower, upper], fmt="o-", capsize=5,
        color="#1b75bc", label="Steady median (5th–95th percentile)",
    )
    for summary in summaries:
        calibration.annotate(
            f"JRK {summary['expected_jrk_target']}",
            (summary["command_mps"], summary["speed_mps"]["median"]),
            xytext=(6, -13), textcoords="offset points", fontsize=9,
        )
    calibration.set(
        xlabel="Commanded speed (m/s)",
        ylabel="GPS actual speed (m/s)",
        xlim=(0.65, 1.58),
        ylim=(0.2, 1.6),
    )
    calibration.grid(True, alpha=0.25)
    calibration.legend(loc="upper left", fontsize=9)
    figure.tight_layout()
    plot_path = args.output_dir / "ring13_four_value_retest_speed_vs_command_20260908.png"
    figure.savefig(plot_path, dpi=170)
    plt.close(figure)

    print(f"Read {len(rows)} field samples")
    print(f"Wrote {json_path}")
    print(f"Wrote {csv_path}")
    print(f"Wrote {plot_path}")
    for summary in summaries:
        speed = summary["speed_mps"]
        print(
            f"{summary['command_mps']:.2f} m/s -> JRK {summary['expected_jrk_target']}: "
            f"median={speed.get('median', math.nan):.3f}, "
            f"mean={speed.get('mean', math.nan):.3f}, "
            f"p05-p95={speed.get('p05', math.nan):.3f}-{speed.get('p95', math.nan):.3f}, "
            f"steady={summary['steady_duration_s']:.1f}s, "
            f"mode9={summary['radio_loss_mode9_approx_s']:.1f}s"
        )


if __name__ == "__main__":
    main()
