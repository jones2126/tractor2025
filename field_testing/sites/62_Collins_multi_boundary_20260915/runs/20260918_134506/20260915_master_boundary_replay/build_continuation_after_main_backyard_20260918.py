#!/usr/bin/env python3
"""Build a separate all-1.0-m/s continuation after the main backyard."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
SOURCE_DIR = HERE / "generated_rings_only"
SOURCE_MISSION = SOURCE_DIR / "62_Collins_rings_only_resume_wp0091_20260916.txt"
SOURCE_AUDIT = SOURCE_DIR / "62_Collins_rings_only_resume_wp0091_audit_20260916.csv"
SOURCE_REPORT = SOURCE_DIR / "62_Collins_rings_only_resume_wp0091_report_20260916.json"
RUN_DIR = HERE.parents[1] / "runs" / "20260918_121342"
PURSUIT_LOG = RUN_DIR / "pursuit_log_20260918_121344.csv"

OUT = HERE / "generated_continuation_20260918"
OUTPUT_MISSION = OUT / "62_Collins_continuation_after_main_backyard_1mps_20260918.txt"
OUTPUT_AUDIT = OUT / "62_Collins_continuation_after_main_backyard_1mps_audit_20260918.csv"
OUTPUT_REPORT = OUT / "62_Collins_continuation_after_main_backyard_1mps_report_20260918.json"
OUTPUT_PREVIEW = OUT / "62_Collins_continuation_after_main_backyard_1mps_REVIEW_20260918.png"

FIRST_PHASE = "transition_05"
SPEED_MPS = 1.0
POSITION_TOLERANCE_M = 1.5
HEADING_TOLERANCE_DEG = 20.0
MAXIMUM_GAP_WITH_COORDINATE_ROUNDING_M = 0.5002


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def normalized_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def distance(first: dict[str, str], second: dict[str, str]) -> float:
    mean_lat = math.radians((float(first["lat"]) + float(second["lat"])) / 2.0)
    return math.hypot(
        (float(second["lon"]) - float(first["lon"])) * 111_320.0 * math.cos(mean_lat),
        (float(second["lat"]) - float(first["lat"])) * 110_540.0,
    )


def compass_heading(yaw_rad: float) -> float:
    return (90.0 - math.degrees(yaw_rad)) % 360.0


def heading_difference(first: float, second: float) -> float:
    return abs((first - second + 180.0) % 360.0 - 180.0)


def load_run_evidence() -> dict[str, object]:
    with PURSUIT_LOG.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    numeric = [
        row for row in rows
        if str(row.get("waypoint_idx", "")).strip().isdigit()
    ]
    max_index = max(int(row["waypoint_idx"]) for row in numeric)
    last = numeric[-1]

    speed_response = {}
    for commanded_speed in (0.5, 1.0):
        values = sorted(
            float(row["actual_speed_mps"])
            for row in rows
            if row.get("driving", "").lower() == "true"
            and row.get("actual_speed_mps", "")
            and row.get("speed_cmd_mps", "")
            and math.isclose(float(row["speed_cmd_mps"]), commanded_speed)
        )
        if not values:
            continue
        median_index = (len(values) - 1) // 2
        p90_index = math.floor(0.90 * (len(values) - 1))
        speed_response[f"{commanded_speed:.1f}"] = {
            "samples": len(values),
            "approximate_seconds_at_20hz": len(values) / 20.0,
            "median_actual_mps": values[median_index],
            "p90_actual_mps": values[p90_index],
            "percent_at_or_below_0_1_mps": (
                100.0 * sum(value <= 0.1 for value in values) / len(values)
            ),
        }
    return {
        "pursuit_log": PURSUIT_LOG.name,
        "pursuit_log_sha256": sha256(PURSUIT_LOG),
        "maximum_controller_waypoint_idx": max_index,
        "last_controller_waypoint_idx": int(last["waypoint_idx"]),
        "last_path_progress_m": float(last["path_progress_m"]),
        "last_position_lat": float(last["lat"]),
        "last_position_lon": float(last["lon"]),
        "last_handheld_mode": int(last["handheld_mode"]),
        "last_software_paused": last["software_paused"].lower() == "true",
        "commanded_vs_actual_speed": speed_response,
    }


def main() -> None:
    for required in (SOURCE_MISSION, SOURCE_AUDIT, SOURCE_REPORT, PURSUIT_LOG):
        if not required.is_file():
            raise FileNotFoundError(required)

    source_lines = SOURCE_MISSION.read_text(encoding="utf-8").splitlines()
    with SOURCE_AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        source_audit = list(csv.DictReader(handle))
    if len(source_lines) != len(source_audit):
        raise ValueError("source mission and audit row counts differ")

    start_index = next(
        index for index, row in enumerate(source_audit)
        if row["phase"] == FIRST_PHASE
    )
    if any(row["phase"].startswith("main_backyard_") for row in source_audit[start_index:]):
        raise ValueError("a main-backyard phase remains after the continuation cut")

    original_start = source_audit[0]
    continuation_start = source_audit[start_index]
    start_distance = distance(original_start, continuation_start)
    original_heading = compass_heading(float(original_start["yaw_rad"]))
    continuation_heading = compass_heading(float(continuation_start["yaw_rad"]))
    start_heading_difference = heading_difference(original_heading, continuation_heading)
    if start_distance > 0.25:
        raise ValueError(f"continuation is {start_distance:.3f} m from the reviewed start")
    if start_heading_difference > HEADING_TOLERANCE_DEG:
        raise ValueError(
            f"continuation heading differs by {start_heading_difference:.1f} degrees"
        )

    continuation_lines = []
    continuation_audit = []
    for waypoint, (line, source) in enumerate(
        zip(source_lines[start_index:], source_audit[start_index:]), 1
    ):
        values = line.split()
        if len(values) != 5:
            raise ValueError(f"source mission row has {len(values)} columns")
        values[4] = f"{SPEED_MPS:.2f}"
        continuation_lines.append(" ".join(values))
        row = dict(source)
        row["prior_resume_waypoint"] = source["waypoint"]
        row["waypoint"] = str(waypoint)
        row["speed_mps"] = f"{SPEED_MPS:.2f}"
        continuation_audit.append(row)

    gaps = [distance(first, second) for first, second in zip(
        continuation_audit, continuation_audit[1:]
    )]
    if max(gaps) > MAXIMUM_GAP_WITH_COORDINATE_ROUNDING_M:
        raise ValueError(f"maximum waypoint gap is {max(gaps):.6f} m")
    if {line.split()[4] for line in continuation_lines} != {"1.00"}:
        raise ValueError("continuation does not have a uniform 1.00 m/s speed")

    OUT.mkdir(parents=True, exist_ok=True)
    OUTPUT_MISSION.write_text(
        "\n".join(continuation_lines) + "\n", encoding="utf-8", newline="\n"
    )
    fieldnames = list(continuation_audit[0])
    fieldnames.remove("prior_resume_waypoint")
    fieldnames.insert(fieldnames.index("waypoint") + 1, "prior_resume_waypoint")
    with OUTPUT_AUDIT.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(continuation_audit)

    run_evidence = load_run_evidence()
    excluded = source_audit[:start_index]
    phase_counts = Counter(row["phase"] for row in continuation_audit)
    route_length = sum(gaps)
    report = {
        "status": "SUPERVISED_BLADES_OFF_CONTINUATION_REVIEW",
        "created_for": (
            "Operator-declared completion of worksheet segments 3 and 4 "
            "(main backyard 1 and 2) on 2026-09-18."
        ),
        "source_mission": SOURCE_MISSION.name,
        "source_mission_normalized_sha256": normalized_sha256(SOURCE_MISSION),
        "source_audit": SOURCE_AUDIT.name,
        "source_report": SOURCE_REPORT.name,
        "first_phase": FIRST_PHASE,
        "first_prior_resume_waypoint": int(continuation_audit[0]["prior_resume_waypoint"]),
        "first_source_waypoint": int(continuation_audit[0]["source_waypoint"]),
        "excluded_prior_resume_waypoints": start_index,
        "excluded_main_backyard_phases": sorted({row["phase"] for row in excluded}),
        "exclusion_note": (
            "Telemetry reached main_backyard_ring_19 but did not traverse its final "
            "tail or main_backyard_exit_connector in AUTO. Those rows are deliberately "
            "omitted because the operator declared the complete main-backyard worksheet "
            "area finished and will manually return to the reviewed start."
        ),
        "start_lat": float(continuation_audit[0]["lat"]),
        "start_lon": float(continuation_audit[0]["lon"]),
        "start_heading_compass_deg": continuation_heading,
        "distance_from_wp0091_start_m": start_distance,
        "heading_difference_from_wp0091_start_deg": start_heading_difference,
        "start_position_tolerance_m": POSITION_TOLERANCE_M,
        "start_heading_tolerance_deg": HEADING_TOLERANCE_DEG,
        "waypoints": len(continuation_lines),
        "phase_counts": dict(phase_counts),
        "speed_counts": {"1.00": len(continuation_lines)},
        "route_length_m": route_length,
        "estimated_runtime_minutes": route_length / SPEED_MPS / 60.0,
        "maximum_waypoint_gap_m": max(gaps),
        "lookahead_values_m": sorted({float(row["lookahead_m"]) for row in continuation_audit}),
        "run_evidence": run_evidence,
        "safety_guards": {
            "reacquire_max_advance_m": 30.0,
            "phase_locked_recovery": True,
            "required_heading_carrier_at_start": "fixed",
            "baseline_range_m": [0.8, 1.3],
            "maximum_heading_accuracy_deg": 1.0,
            "basic_runtime_heading_gate": True,
        },
        "mission_sha256": sha256(OUTPUT_MISSION),
    }
    OUTPUT_REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    all_x = [float(row["east_m"]) for row in source_audit]
    all_y = [float(row["north_m"]) for row in source_audit]
    x = [float(row["east_m"]) for row in continuation_audit]
    y = [float(row["north_m"]) for row in continuation_audit]
    fig, axis = plt.subplots(figsize=(12, 10), dpi=170)
    axis.plot(all_x, all_y, color="#c7cbd1", linewidth=0.7, label="original reviewed mission")
    axis.plot(x, y, color="#0b7285", linewidth=1.4, label="new continuation — 1.00 m/s")
    axis.scatter([x[0]], [y[0]], color="#2f9e44", s=65, zorder=4, label="continuation start")
    axis.scatter(
        [float(original_start["east_m"])], [float(original_start["north_m"])],
        marker="x", color="#e67700", s=75, zorder=4, label="waypoint-91 start",
    )
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.25)
    axis.set_xlabel("East of Segment 2 start (m)")
    axis.set_ylabel("North of Segment 2 start (m)")
    axis.set_title(
        f"62 Collins continuation after main backyard — {route_length:.0f} m, "
        f"{report['estimated_runtime_minutes']:.1f} min, all waypoints 1.00 m/s"
    )
    axis.legend(loc="best")
    fig.tight_layout()
    save_path = str(OUTPUT_PREVIEW.resolve())
    if os.name == "nt":
        save_path = "\\\\?\\" + save_path
    fig.savefig(save_path)
    plt.close(fig)

    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
