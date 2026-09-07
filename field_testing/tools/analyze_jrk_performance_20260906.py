"""Normalize the 2026-09-05/06 JRK test logs for workbook reporting."""

from __future__ import annotations

import csv
import json
import re
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "field_testing" / "jrk" / "20260906" / "source_logs" / "home" / "al"
OUTPUT = ROOT / "field_testing" / "jrk" / "20260906" / "analysis"

SAMPLE_RE = re.compile(
    r"^(\d+),(PROBE|RETURN),(\d+),(\d+),(\d+),(\d+),(\d+)$"
)
BEGIN_RE = re.compile(r"^(PROBE_BEGIN|RETURN_STEP_BEGIN),target=(\d+)$")
PROBE_RESULT_RE = re.compile(
    r"^PROBE_RESULT,target=(\d+),result=([^,]+),peak_mA=(\d+),feedback=(\d+)$"
)
RETURN_RESULT_RE = re.compile(
    r"^(?:RETURN_STEP_RESULT,target=(\d+),|RETURN_RESULT,)"
    r"result=([^,]+),peak_mA=(\d+),feedback=(\d+)$"
)


def run_label(path: Path) -> str:
    match = re.search(r"run(\d*)_20260906", path.name)
    if not match:
        return path.stem
    return f"guard_run{match.group(1) or '1'}"


def movement_record(run: str, move_number: int, active: dict, result: str,
                    reported_peak: int, final_feedback: int) -> dict:
    samples = active["samples"]
    target = active["target"]
    start_feedback = samples[0]["feedback"] if samples else final_feedback
    duration_ms = samples[-1]["elapsed_ms"] if samples else 0
    signed_delta = final_feedback - start_feedback
    distance = abs(signed_delta)
    rate = distance / (duration_ms / 1000) if duration_ms else 0

    moving_currents = []
    for previous, current in zip(samples, samples[1:]):
        if current["feedback"] != previous["feedback"]:
            moving_currents.append(current["current_mA"])
    nonzero_currents = [s["current_mA"] for s in samples if s["current_mA"] > 0]

    direction = "Forward" if target < start_feedback else "Return"
    return {
        "move_id": f"{run}_{move_number:03d}",
        "run": run,
        "phase": active["phase"],
        "direction": direction,
        "target": target,
        "start_feedback": start_feedback,
        "final_feedback": final_feedback,
        "signed_feedback_change": signed_delta,
        "distance_counts": distance,
        "duration_ms": duration_ms,
        "rate_counts_per_s": round(rate, 3),
        "peak_current_mA": max(reported_peak, max((s["current_mA"] for s in samples), default=0)),
        "avg_moving_current_mA": round(sum(moving_currents) / len(moving_currents), 1)
        if moving_currents else 0,
        "avg_nonzero_current_mA": round(sum(nonzero_currents) / len(nonzero_currents), 1)
        if nonzero_currents else 0,
        "final_error_counts": abs(final_feedback - target),
        "result": result,
        "sample_count": len(samples),
    }


def parse_guard_log(path: Path) -> tuple[list[dict], list[dict]]:
    run = run_label(path)
    movements = []
    raw_samples = []
    active = None
    move_number = 0

    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip().replace("\x1b[0m", "")
        begin = BEGIN_RE.match(line)
        if begin:
            active = {
                "phase": "PROBE" if begin.group(1) == "PROBE_BEGIN" else "RETURN",
                "target": int(begin.group(2)),
                "samples": [],
                "raw_entries": [],
            }
            continue
        if line.startswith("RETURN_TO_NEUTRAL,"):
            active = {"phase": "RETURN", "target": None, "samples": [], "raw_entries": []}
            continue

        sample_match = SAMPLE_RE.match(line)
        if sample_match:
            sample = {
                "t_ms": int(sample_match.group(1)),
                "phase": sample_match.group(2),
                "target": int(sample_match.group(3)),
                "current_mA": int(sample_match.group(4)),
                "feedback": int(sample_match.group(5)),
                "peak_mA": int(sample_match.group(6)),
                "elapsed_ms": int(sample_match.group(7)),
            }
            if active is None:
                active = {
                    "phase": sample["phase"], "target": sample["target"],
                    "samples": [], "raw_entries": [],
                }
            if active["target"] is None:
                active["target"] = sample["target"]
            active["samples"].append(sample)
            raw_entry = {"run": run, **sample}
            raw_samples.append(raw_entry)
            active["raw_entries"].append(raw_entry)
            continue

        probe_result = PROBE_RESULT_RE.match(line)
        return_result = RETURN_RESULT_RE.match(line)
        result_match = probe_result or return_result
        if result_match and active:
            if probe_result:
                target = int(probe_result.group(1))
                result = probe_result.group(2)
                peak = int(probe_result.group(3))
                feedback = int(probe_result.group(4))
            else:
                target_text = return_result.group(1)
                target = int(target_text) if target_text else active["target"]
                result = return_result.group(2)
                peak = int(return_result.group(3))
                feedback = int(return_result.group(4))
            active["target"] = target
            move_number += 1
            movement = movement_record(run, move_number, active, result, peak, feedback)
            movements.append(movement)
            for raw_entry in active["raw_entries"]:
                raw_entry["move_id"] = movement["move_id"]
            active = None

    return movements, raw_samples


def parse_baseline(path: Path) -> list[dict]:
    rows = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = re.match(r"^(\d+),(-?\d+),(\d+),(\d+),(\d+)$", line.strip())
        if match:
            rows.append({
                "t_ms": int(match.group(1)),
                "step": int(match.group(2)),
                "target": int(match.group(3)),
                "current_mA": int(match.group(4)),
                "feedback": int(match.group(5)),
            })
    return rows


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    fieldnames = []
    for row in rows:
        for field in row:
            if field not in fieldnames:
                fieldnames.append(field)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    all_movements = []
    all_samples = []
    for path in sorted(SOURCE.glob("jrk_limit_guard_run*_20260906.txt")):
        movements, samples = parse_guard_log(path)
        all_movements.extend(movements)
        all_samples.extend(samples)

    baseline_path = SOURCE / "tractor2025" / "jrk_current_test_run2.txt"
    baseline = parse_baseline(baseline_path)

    final_runs = {"guard_run5", "guard_run6", "guard_run7"}
    successful_probes = [
        row for row in all_movements
        if row["run"] in final_runs and row["phase"] == "PROBE"
        and row["result"] == "TARGET_REACHED"
    ]
    grouped = defaultdict(list)
    for row in successful_probes:
        grouped[row["target"]].append(row)

    position_map = []
    for target in sorted(grouped, reverse=True):
        rows = grouped[target]
        position_map.append({
            "target": target,
            "trials": len(rows),
            "avg_final_feedback": round(sum(r["final_feedback"] for r in rows) / len(rows), 1),
            "max_final_error_counts": max(r["final_error_counts"] for r in rows),
            "avg_duration_ms": round(sum(r["duration_ms"] for r in rows) / len(rows), 1),
            "avg_rate_counts_per_s": round(sum(r["rate_counts_per_s"] for r in rows) / len(rows), 1),
            "avg_peak_current_A": round(sum(r["peak_current_mA"] for r in rows) / len(rows) / 1000, 3),
            "max_peak_current_A": round(max(r["peak_current_mA"] for r in rows) / 1000, 3),
            "avg_moving_current_A": round(sum(r["avg_moving_current_mA"] for r in rows) / len(rows) / 1000, 3),
        })

    summary = {
        "guard_movement_count": len(all_movements),
        "guard_sample_count": len(all_samples),
        "baseline_sample_count": len(baseline),
        "successful_probe_count": len(successful_probes),
        "lowest_successful_target": min(r["target"] for r in successful_probes),
        "lowest_feedback": min(r["final_feedback"] for r in successful_probes),
        "max_staged_probe_peak_A": round(max(r["peak_current_mA"] for r in successful_probes) / 1000, 3),
        "max_successful_return_peak_A": round(max(
            r["peak_current_mA"] for r in all_movements
            if r["phase"] == "RETURN" and r["result"] == "TARGET_REACHED"
        ) / 1000, 3),
        "position_count": len(position_map),
    }

    write_csv(OUTPUT / "jrk_movement_summary_20260906.csv", all_movements)
    write_csv(OUTPUT / "jrk_guard_samples_20260906.csv", all_samples)
    write_csv(OUTPUT / "jrk_position_performance_map_20260906.csv", position_map)
    write_csv(OUTPUT / "jrk_baseline_samples_20260905.csv", baseline)
    (OUTPUT / "jrk_analysis_summary_20260906.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
