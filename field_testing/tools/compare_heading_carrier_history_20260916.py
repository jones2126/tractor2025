#!/usr/bin/env python3
"""Compare heading-carrier reliability across field telemetry CSV files.

The logger republishes receiver state faster than the F9P produces a new
solution, so row counts are not treated as independent observations.  Each
row is weighted by the elapsed time until the next row.  Large logging gaps
are excluded rather than being charged to the last observed carrier state.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path


DERIVED_NAMES = {"heading_comparison_samples_20260828.csv"}
KNOWN_STATES = {"fixed", "float", "none"}


def as_float(value: str | None) -> float | None:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def display_name(path: Path) -> str:
    return path.stem


def run_date(path: Path) -> str:
    matches = re.findall(r"20\d{6}", str(path))
    return matches[-1] if matches else "unknown"


def era(path: Path) -> str:
    date = run_date(path)
    if date < "20260829":
        return "early dual-F9P / coax investigation"
    if date == "20260829":
        return "original heading antenna"
    if date < "20260916":
        return "AT340 heading antenna"
    return "2026-09-16 field session"


@dataclass
class Interval:
    dt: float
    carrier: str
    moving: bool
    auto: bool


@dataclass
class Metrics:
    observed_sec: float
    fixed_sec: float
    float_sec: float
    none_sec: float
    unknown_sec: float
    fixed_pct_known: float | None
    nonfixed_sec: float
    episode_count: int
    longest_episode_sec: float
    episodes_ge_1s: int
    episodes_ge_2s: int


@dataclass
class RunResult:
    date: str
    era: str
    name: str
    path: Path
    rows: int
    median_period_sec: float
    duplicate_of: str
    all_data: Metrics
    moving: Metrics
    auto: Metrics


def summarize(intervals: list[Interval], selector=lambda item: True) -> Metrics:
    durations = {state: 0.0 for state in (*KNOWN_STATES, "unknown")}
    episodes: list[float] = []
    current_episode = 0.0

    for item in intervals:
        if not selector(item):
            if current_episode:
                episodes.append(current_episode)
                current_episode = 0.0
            continue
        state = item.carrier if item.carrier in KNOWN_STATES else "unknown"
        durations[state] += item.dt
        if state in {"float", "none"}:
            current_episode += item.dt
        elif current_episode:
            episodes.append(current_episode)
            current_episode = 0.0
    if current_episode:
        episodes.append(current_episode)

    known = durations["fixed"] + durations["float"] + durations["none"]
    nonfixed = durations["float"] + durations["none"]
    return Metrics(
        observed_sec=sum(durations.values()),
        fixed_sec=durations["fixed"],
        float_sec=durations["float"],
        none_sec=durations["none"],
        unknown_sec=durations["unknown"],
        fixed_pct_known=(100.0 * durations["fixed"] / known) if known else None,
        nonfixed_sec=nonfixed,
        episode_count=len(episodes),
        longest_episode_sec=max(episodes, default=0.0),
        episodes_ge_1s=sum(value >= 1.0 for value in episodes),
        episodes_ge_2s=sum(value >= 2.0 for value in episodes),
    )


def analyze(path: Path, duplicate_of: str = "") -> RunResult:
    rows: list[dict[str, str]] = []
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        elapsed_key = "elapsed_sec" if "elapsed_sec" in (reader.fieldnames or []) else "elapsed_s"
        carrier_key = "carrier" if "carrier" in (reader.fieldnames or []) else "heading_carrier"
        for row in reader:
            elapsed = as_float(row.get(elapsed_key))
            if elapsed is None:
                continue
            row["__elapsed"] = str(elapsed)
            row["__carrier"] = (row.get(carrier_key) or "unknown").strip().lower()
            rows.append(row)

    positive_deltas = [
        float(rows[index + 1]["__elapsed"]) - float(rows[index]["__elapsed"])
        for index in range(len(rows) - 1)
        if float(rows[index + 1]["__elapsed"]) > float(rows[index]["__elapsed"])
    ]
    period = statistics.median(positive_deltas) if positive_deltas else 0.05
    gap_limit = max(0.25, 5.0 * period)
    intervals: list[Interval] = []

    for index, row in enumerate(rows):
        if index + 1 < len(rows):
            dt = float(rows[index + 1]["__elapsed"]) - float(row["__elapsed"])
        else:
            dt = period
        if dt <= 0.0 or dt > gap_limit:
            continue
        speed = as_float(row.get("speed_mps"))
        if speed is None:
            speed = as_float(row.get("actual_speed_mps"))
        intervals.append(
            Interval(
                dt=dt,
                carrier=row["__carrier"],
                moving=(speed is not None and abs(speed) >= 0.10),
                auto=(row.get("trans_mode") == "0"),
            )
        )

    return RunResult(
        date=run_date(path),
        era=era(path),
        name=display_name(path),
        path=path,
        rows=len(rows),
        median_period_sec=period,
        duplicate_of=duplicate_of,
        all_data=summarize(intervals),
        moving=summarize(intervals, lambda item: item.moving),
        auto=summarize(intervals, lambda item: item.auto),
    )


def fmt(value: float | None, digits: int = 2) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def write_csv(results: list[RunResult], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "date", "era", "run", "path", "rows", "sample_period_sec", "duplicate_of",
        "observed_sec", "fixed_pct", "nonfixed_sec", "float_sec", "none_sec",
        "nonfixed_episodes", "longest_nonfixed_sec", "episodes_ge_1s", "episodes_ge_2s",
        "moving_observed_sec", "moving_fixed_pct", "moving_nonfixed_sec",
        "moving_nonfixed_episodes", "moving_longest_nonfixed_sec",
        "auto_observed_sec", "auto_fixed_pct", "auto_nonfixed_sec",
    ]
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for result in results:
            writer.writerow({
                "date": result.date,
                "era": result.era,
                "run": result.name,
                "path": str(result.path),
                "rows": result.rows,
                "sample_period_sec": fmt(result.median_period_sec, 4),
                "duplicate_of": result.duplicate_of,
                "observed_sec": fmt(result.all_data.observed_sec),
                "fixed_pct": fmt(result.all_data.fixed_pct_known, 3),
                "nonfixed_sec": fmt(result.all_data.nonfixed_sec),
                "float_sec": fmt(result.all_data.float_sec),
                "none_sec": fmt(result.all_data.none_sec),
                "nonfixed_episodes": result.all_data.episode_count,
                "longest_nonfixed_sec": fmt(result.all_data.longest_episode_sec),
                "episodes_ge_1s": result.all_data.episodes_ge_1s,
                "episodes_ge_2s": result.all_data.episodes_ge_2s,
                "moving_observed_sec": fmt(result.moving.observed_sec),
                "moving_fixed_pct": fmt(result.moving.fixed_pct_known, 3),
                "moving_nonfixed_sec": fmt(result.moving.nonfixed_sec),
                "moving_nonfixed_episodes": result.moving.episode_count,
                "moving_longest_nonfixed_sec": fmt(result.moving.longest_episode_sec),
                "auto_observed_sec": fmt(result.auto.observed_sec),
                "auto_fixed_pct": fmt(result.auto.fixed_pct_known, 3),
                "auto_nonfixed_sec": fmt(result.auto.nonfixed_sec),
            })


def write_markdown(results: list[RunResult], output: Path, csv_output: Path) -> None:
    unique = [result for result in results if not result.duplicate_of]
    recent = [result for result in unique if result.date >= "20260829"]
    baseline = [result for result in unique if "20260830" <= result.date < "20260916"]
    today = next((result for result in unique if result.date == "20260916"), None)

    def aggregate_fixed_pct(items: list[RunResult], metric_name: str) -> float | None:
        metrics = [getattr(item, metric_name) for item in items]
        fixed = sum(metric.fixed_sec for metric in metrics)
        known = fixed + sum(metric.nonfixed_sec for metric in metrics)
        return (100.0 * fixed / known) if known else None

    baseline_full_pct = aggregate_fixed_pct(baseline, "all_data")
    baseline_moving_pct = aggregate_fixed_pct(baseline, "moving")
    lines = [
        "# Heading carrier history comparison — 2026-09-16",
        "",
        "This report uses time-weighted carrier state from the field telemetry CSVs. "
        "It excludes derived analysis CSVs and exact duplicate copies. Large logger gaps are not "
        "attributed to the last state. `Moving` means `abs(speed_mps) >= 0.10 m/s`.",
        "",
        "## Finding",
        "",
    ]
    if today and baseline:
        today_nonfixed_rate = 100.0 - (today.moving.fixed_pct_known or 100.0)
        baseline_nonfixed_rate = 100.0 - (baseline_moving_pct or 100.0)
        rate_ratio = today_nonfixed_rate / baseline_nonfixed_rate if baseline_nonfixed_rate else math.inf
        previous_longest = max(item.all_data.longest_episode_sec for item in baseline)
        moving_rank = 1 + sum(
            (item.moving.fixed_pct_known or 100.0) < (today.moving.fixed_pct_known or 100.0)
            for item in baseline
        )
        lines += [
            f"Today's whole-run carrier was Fixed {fmt(today.all_data.fixed_pct_known)}% of known time, "
            f"versus {fmt(baseline_full_pct)}% across the nine AT340-era runs from 2026-08-30 through 2026-09-15.",
            "",
            f"While moving, today was Fixed {fmt(today.moving.fixed_pct_known)}%, versus "
            f"{fmt(baseline_moving_pct)}% historically. The moving non-Fixed rate was therefore "
            f"about {rate_ratio:.1f} times the historical aggregate. Today ranks {moving_rank} of "
            f"{len(baseline) + 1} from worst to best by moving Fixed percentage.",
            "",
            f"Today's two non-Fixed episodes totaled {today.all_data.nonfixed_sec:.2f} seconds and were "
            f"both Float. The longest was {today.all_data.longest_episode_sec:.2f} seconds, compared "
            f"with a previous AT340-era maximum of {previous_longest:.2f} seconds. Only "
            f"{today.moving.nonfixed_sec:.2f} seconds of today's non-Fixed time occurred while moving.",
            "",
            "Conclusion: today was noticeably worse than the normal post-AT340 baseline, especially "
            "because of one sustained Float interval while mostly stationary. It was not unprecedented: "
            "the 2026-08-30 AT340 perimeter had a lower moving Fixed percentage.",
            "",
        ]
    lines += [
        "## Same-day raw receiver audits",
        "",
        "These two 120-second stationary observations were recorded in the investigation handoff. "
        "They were made under trees, and the full raw JSON files are not present on this computer, "
        "so individual post-change episode lengths cannot be reconstructed here.",
        "",
        "| Stage | Frames | Carrier Fixed | Carrier Float | Other heading-invalid frames | Approx. Float time |",
        "|---|---:|---:|---:|---:|---:|",
        "| Before Heading USB NMEA cleanup | 599 | 599 (100%) | 0 | 2 Fixed-carrier, zero-baseline frames | 0.00 s |",
        "| After Heading USB NMEA cleanup | 598 | 502 (83.946%) | 96 | not reported | 19.27 s |",
        "",
        "The after-cleanup carrier snapshot was clearly worse than the before-cleanup snapshot, but this pair "
        "does not prove that disabling NMEA caused the change: the tests were sequential rather than "
        "simultaneous, the tractor was under trees, and carrier performance can change quickly with "
        "multipath and satellite geometry. The field mission CSV analyzed above was recorded before "
        "the NMEA-only change.",
        "",
        "## Comparable recent field runs",
        "",
        "| Date | Run | Era | Observed (s) | Fixed | Non-fixed (s) | Episodes | Longest (s) | Moving fixed | Moving non-fixed (s) |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for result in recent:
        lines.append(
            f"| {result.date} | `{result.name}` | {result.era} | "
            f"{result.all_data.observed_sec:.1f} | {fmt(result.all_data.fixed_pct_known)}% | "
            f"{result.all_data.nonfixed_sec:.2f} | {result.all_data.episode_count} | "
            f"{result.all_data.longest_episode_sec:.2f} | {fmt(result.moving.fixed_pct_known)}% | "
            f"{result.moving.nonfixed_sec:.2f} |"
        )

    lines += [
        "",
        "## Earlier runs (different antenna/coax investigation period)",
        "",
        "These are retained for history, but they are not a clean baseline for today's AT340/current-hardware comparison.",
        "",
        "| Date | Run | Observed (s) | Fixed | Non-fixed (s) | Episodes | Longest (s) |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for result in unique:
        if result.date >= "20260829":
            continue
        lines.append(
            f"| {result.date} | `{result.name}` | {result.all_data.observed_sec:.1f} | "
            f"{fmt(result.all_data.fixed_pct_known)}% | {result.all_data.nonfixed_sec:.2f} | "
            f"{result.all_data.episode_count} | {result.all_data.longest_episode_sec:.2f} |"
        )

    duplicates = [result for result in results if result.duplicate_of]
    lines += ["", "## Exclusions and caveats", ""]
    if duplicates:
        lines.append("Exact duplicate telemetry copies excluded from comparisons:")
        lines.append("")
        for result in duplicates:
            lines.append(f"- `{result.path}` duplicates `{result.duplicate_of}`")
        lines.append("")
    lines += [
        "- `heading_comparison_samples_20260828.csv` files are derived samples, not independent runs.",
        "- The pure-pursuit log is a second view of today's same session, not another run.",
        "- Fixed percentage is computed over known carrier time (`fixed`, `float`, or `none`).",
        "- Non-fixed time combines `float` and `none`; the detailed CSV keeps those durations separate.",
        "- A full-run percentage can be affected by where the tractor was parked and acquisition time. "
        "The moving columns are the fairer measure of field performance.",
        "",
        f"Detailed machine-readable results: `{csv_output.name}`",
        "",
    ]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines), encoding="utf-8")


def find_candidates(roots: list[Path]) -> list[Path]:
    candidates: list[Path] = []
    for root in roots:
        if not root.exists():
            continue
        for path in root.rglob("*.csv"):
            if path.name in DERIVED_NAMES:
                continue
            if path.name.startswith("pursuit_log_"):
                continue
            if run_date(path) == "unknown":
                continue
            try:
                header = path.open("r", encoding="utf-8-sig").readline().strip().split(",")
            except OSError:
                continue
            if "carrier" in header or "heading_carrier" in header:
                candidates.append(path)
    return sorted(candidates)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("roots", nargs="+", type=Path)
    parser.add_argument("--csv-output", type=Path, required=True)
    parser.add_argument("--markdown-output", type=Path, required=True)
    args = parser.parse_args()

    candidates = find_candidates(args.roots)
    seen_hashes: dict[str, str] = {}
    results: list[RunResult] = []
    for path in candidates:
        digest = sha256(path)
        duplicate_of = seen_hashes.get(digest, "")
        if not duplicate_of:
            seen_hashes[digest] = str(path)
        results.append(analyze(path, duplicate_of))

    results.sort(key=lambda item: (item.date, item.name))
    write_csv(results, args.csv_output)
    write_markdown(results, args.markdown_output, args.csv_output)
    print(f"Analyzed {len(results)} files ({sum(not item.duplicate_of for item in results)} unique).")
    print(f"CSV: {args.csv_output}")
    print(f"Markdown: {args.markdown_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
