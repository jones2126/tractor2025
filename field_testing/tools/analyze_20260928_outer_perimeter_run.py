#!/usr/bin/env python3
"""Summarize the completed 2026-09-28 outer-perimeter field run."""

from __future__ import annotations

import csv
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np


REPO = Path(__file__).resolve().parents[2]
SITE = REPO / "field_testing/sites/62_Collins_multi_boundary_20260915"
RUN = SITE / "runs/20260928_outer_perimeter_field_test/outer_perimeter_20260928_120838.csv"
PACKAGE = SITE / "mission_plans/20260928_outer_perimeter_field_test/generated"
MISSION = PACKAGE / "62_Collins_outer_perimeter_field_test_1mps_20260928.txt"
AUDIT = PACKAGE / "62_Collins_outer_perimeter_field_test_1mps_20260928_audit.csv"


def number(value, default=math.nan):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def main():
    mission = np.array([[float(value) for value in line.split()] for line in MISSION.read_text().splitlines()])
    with AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        audit = list(csv.DictReader(handle))
    with RUN.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))

    lat0, lon0 = mission[0, :2]
    north_scale = 110_540.0
    east_scale = 111_320.0 * math.cos(math.radians(lat0))
    plan_xy = np.column_stack(((mission[:, 1] - lon0) * east_scale, (mission[:, 0] - lat0) * north_scale))
    gps_xy = np.array([
        [(number(row["lon"]) - lon0) * east_scale, (number(row["lat"]) - lat0) * north_scale]
        for row in rows
    ])
    finite = np.isfinite(gps_xy).all(axis=1)
    mode = np.array([int(number(row["trans_mode"], -1)) for row in rows])
    command = np.array([number(row["trans_cmd_vel_mps"]) for row in rows])
    actual = np.array([number(row["speed_mps"]) for row in rows])
    active = finite & (mode == 0) & (command > 0.05)

    active_indices = np.flatnonzero(active)
    nearest_distance = np.full(len(rows), np.nan)
    nearest_waypoint = np.full(len(rows), -1, dtype=int)
    for start in range(0, len(active_indices), 500):
        selected = active_indices[start:start + 500]
        delta = gps_xy[selected, None, :] - plan_xy[None, :, :]
        distance_sq = np.einsum("ijk,ijk->ij", delta, delta)
        closest = np.argmin(distance_sq, axis=1)
        nearest_waypoint[selected] = closest
        nearest_distance[selected] = np.sqrt(distance_sq[np.arange(len(selected)), closest])

    active_error = nearest_distance[active]
    phase_errors = {}
    phase_names = np.array([row["phase"] for row in audit])
    sample_phases = np.full(len(rows), "", dtype=object)
    valid_nearest = nearest_waypoint >= 0
    sample_phases[valid_nearest] = phase_names[nearest_waypoint[valid_nearest]]
    for phase in dict.fromkeys(phase_names):
        values = nearest_distance[active & (sample_phases == phase)]
        if len(values):
            phase_errors[phase] = {
                "samples": int(len(values)),
                "median_m": round(float(np.median(values)), 3),
                "p95_m": round(float(np.quantile(values, 0.95)), 3),
                "maximum_m": round(float(np.max(values)), 3),
            }

    worst = np.argsort(np.nan_to_num(nearest_distance, nan=-1.0))[-10:][::-1]
    result = {
        "rows": len(rows),
        "elapsed_seconds": round(number(rows[-1]["elapsed_sec"]), 2),
        "active_auto_samples": int(active.sum()),
        "rtk_fixed_percent": round(100 * sum(row["fix_quality"] == "RTK Fixed" for row in rows) / len(rows), 3),
        "heading_valid_percent": round(100 * sum(row["head_valid"].lower() == "true" for row in rows) / len(rows), 3),
        "heading_fixed_percent": round(100 * sum(row["carrier"].lower() == "fixed" for row in rows) / len(rows), 3),
        "radio_status_counts": Counter(row["radio_signal"] for row in rows),
        "firmware_counts": Counter(row["teensy_firmware"] for row in rows),
        "active_speed_mps": {
            "median": round(float(np.nanmedian(actual[active])), 3),
            "p95": round(float(np.nanquantile(actual[active], 0.95)), 3),
            "maximum": round(float(np.nanmax(actual[active])), 3),
        },
        "nearest_planned_path_error_m": {
            "median": round(float(np.median(active_error)), 3),
            "p90": round(float(np.quantile(active_error, 0.90)), 3),
            "p95": round(float(np.quantile(active_error, 0.95)), 3),
            "p99": round(float(np.quantile(active_error, 0.99)), 3),
            "maximum": round(float(np.max(active_error)), 3),
            "within_0_5m_percent": round(100 * float(np.mean(active_error <= 0.5)), 2),
            "within_1_0m_percent": round(100 * float(np.mean(active_error <= 1.0)), 2),
        },
        "steering": {
            "pwm_saturated_percent_active": round(100 * np.mean(np.array([row["steer_pwm_saturated"] == "1" for row in rows])[active]), 2),
            "minimum_pwm_clamped_percent_active": round(100 * np.mean(np.array([row["steer_min_pwm_clamped"] == "1" for row in rows])[active]), 2),
            "maximum_absolute_error_counts_active": int(np.nanmax(np.abs(np.array([number(row["steer_error"]) for row in rows])[active]))),
        },
        "phase_path_error": phase_errors,
        "largest_path_error_samples": [
            {
                "time": rows[index]["time"],
                "elapsed_sec": number(rows[index]["elapsed_sec"]),
                "error_m": round(float(nearest_distance[index]), 3),
                "nearest_waypoint": int(nearest_waypoint[index] + 1),
                "phase": audit[nearest_waypoint[index]]["phase"],
                "speed_mps": number(rows[index]["speed_mps"]),
            }
            for index in worst if nearest_waypoint[index] >= 0
        ],
    }
    print(json.dumps(result, indent=2, default=dict))


if __name__ == "__main__":
    main()
