#!/usr/bin/env python3
"""Detailed stripe and terminal-circling diagnostics for a collected field run.

This is intentionally separate from the normal run summarizer.  It aligns the
20 Hz pursuit log to the field logger, reconstructs contiguous stripe rows from
the mission audit, computes direction-aware geometric error, and writes compact
CSV/JSON/Markdown/SVG evidence artifacts without changing source logs.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


AUTO_MODE = 0
ALIGN_TOLERANCE_S = 0.15


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser()
    result.add_argument("--run-dir", required=True, type=Path)
    result.add_argument("--mission-dir", required=True, type=Path)
    result.add_argument("--site-name", required=True)
    result.add_argument("--run-id", required=True)
    result.add_argument("--output-dir", required=True, type=Path)
    return result


def numeric(frame: pd.DataFrame, columns: Iterable[str]) -> None:
    for column in columns:
        if column in frame:
            frame[column] = pd.to_numeric(frame[column], errors="coerce")


def weighted_mean(values: pd.Series, weights: pd.Series) -> float:
    mask = values.notna() & weights.gt(0)
    if not mask.any():
        return float("nan")
    return float(np.average(values[mask], weights=weights[mask]))


def weighted_abs_mean(values: pd.Series, weights: pd.Series) -> float:
    return weighted_mean(values.abs(), weights)


def weighted_pct(mask: pd.Series, weights: pd.Series) -> float:
    valid = mask.notna() & weights.gt(0)
    if not valid.any():
        return float("nan")
    return 100.0 * float(weights[valid & mask.fillna(False)].sum()) / float(
        weights[valid].sum()
    )


def wrap_degrees(values: pd.Series | np.ndarray) -> pd.Series | np.ndarray:
    return (values + 180.0) % 360.0 - 180.0


def direction_name(compass_deg: float) -> str:
    names = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
    return names[int(((compass_deg + 22.5) % 360.0) // 45.0)]


def read_inputs(args: argparse.Namespace):
    pursuit_path = args.run_dir / f"pursuit_log_{args.run_id}.csv"
    field_path = args.run_dir / f"field_test_{args.run_id}.csv"
    audit_path = (
        args.mission_dir / f"{args.site_name}_supervised_test_mission_audit.csv"
    )
    mission_path = (
        args.mission_dir / f"{args.site_name}_supervised_test_mission.txt"
    )

    pursuit = pd.read_csv(pursuit_path, low_memory=False)
    numeric(
        pursuit,
        (
            "timestamp",
            "elapsed_s",
            "cycle",
            "lat",
            "lon",
            "heading_compass_deg",
            "pos_x_m",
            "pos_y_m",
            "waypoint_idx",
            "target_x_m",
            "target_y_m",
            "lookahead_dist_m",
            "yt_m",
            "cross_track_err_m",
            "alpha_deg",
            "delta_deg",
            "steer_normalized",
            "speed_cmd_mps",
        ),
    )
    pursuit = pursuit.dropna(subset=["timestamp"]).sort_values("timestamp")
    pursuit["driving_bool"] = (
        pursuit["driving"].astype(str).str.strip().str.lower().eq("true")
    )

    field = pd.read_csv(field_path, low_memory=False)
    field["field_timestamp"] = pd.to_datetime(
        field["time"], utc=True, errors="coerce"
    ).map(lambda value: value.timestamp() if pd.notna(value) else np.nan)
    field_columns = (
        "elapsed_sec",
        "trans_mode",
        "speed_mps",
        "steer_setpoint",
        "steer_current",
        "steer_error",
        "steer_pwm",
        "jrk_target",
        "jrk_current",
    )
    numeric(field, field_columns)
    field = field.dropna(subset=["field_timestamp"]).sort_values(
        "field_timestamp"
    )

    aligned = pd.merge_asof(
        pursuit,
        field[["field_timestamp", *field_columns]].rename(
            columns={"speed_mps": "actual_speed_mps"}
        ),
        left_on="timestamp",
        right_on="field_timestamp",
        direction="nearest",
        tolerance=ALIGN_TOLERANCE_S,
    )
    dt = aligned["timestamp"].shift(-1) - aligned["timestamp"]
    aligned["time_weight_s"] = np.where(dt.between(0, 0.20), dt, 0.0)

    audit = pd.read_csv(audit_path)
    numeric(
        audit,
        (
            "waypoint",
            "headland_pass",
            "east_m",
            "north_m",
            "yaw_rad",
            "lookahead_m",
            "speed_mps",
            "local_radius_m",
        ),
    )
    audit = audit.dropna(subset=["waypoint", "east_m", "north_m"]).sort_values(
        "waypoint"
    )
    audit["waypoint_idx"] = audit["waypoint"].astype(int) - 1
    audit["stripe_id"] = np.nan
    stripe_id = 0
    in_stripe = False
    for index, row in audit.iterrows():
        if row["kind"] == "stripe":
            if not in_stripe:
                stripe_id += 1
            audit.at[index, "stripe_id"] = stripe_id
            in_stripe = True
        else:
            in_stripe = False

    mission = pd.read_csv(
        mission_path,
        sep=r"\s+",
        names=("lat", "lon", "yaw_rad", "lookahead_m", "speed_mps"),
    )
    mission["waypoint_idx"] = np.arange(len(mission))
    return aligned.reset_index(drop=True), audit.reset_index(drop=True), mission


def stripe_metadata(audit: pd.DataFrame) -> pd.DataFrame:
    rows = []
    origin_e = float(audit.iloc[0]["east_m"])
    origin_n = float(audit.iloc[0]["north_m"])
    for stripe_id, group in audit.dropna(subset=["stripe_id"]).groupby(
        "stripe_id", sort=True
    ):
        group = group.sort_values("waypoint_idx")
        first, last = group.iloc[0], group.iloc[-1]
        dx = float(last["east_m"] - first["east_m"])
        dy = float(last["north_m"] - first["north_m"])
        length = math.hypot(dx, dy)
        ux, uy = dx / length, dy / length
        compass = (90.0 - math.degrees(math.atan2(dy, dx))) % 360.0
        rows.append(
            {
                "stripe_id": int(stripe_id),
                "start_waypoint_idx": int(first["waypoint_idx"]),
                "end_waypoint_idx": int(last["waypoint_idx"]),
                "start_x_m": float(first["east_m"] - origin_e),
                "start_y_m": float(first["north_m"] - origin_n),
                "end_x_m": float(last["east_m"] - origin_e),
                "end_y_m": float(last["north_m"] - origin_n),
                "mean_north_m": float(group["north_m"].mean()),
                "length_m": length,
                "unit_x": ux,
                "unit_y": uy,
                "path_compass_deg": compass,
                "direction": direction_name(compass),
            }
        )
    result = pd.DataFrame(rows)
    result["north_rank"] = result["mean_north_m"].rank(
        method="dense", ascending=True
    ).astype(int)
    return result


def attach_stripe_metrics(
    aligned: pd.DataFrame, audit: pd.DataFrame, meta: pd.DataFrame
) -> pd.DataFrame:
    waypoint_labels = audit[["waypoint_idx", "kind", "stripe_id"]]
    result = aligned.merge(waypoint_labels, how="left", on="waypoint_idx")
    result = result.merge(meta, how="left", on="stripe_id")
    result["controller_signed_cte_m"] = result["yt_m"]
    result["controller_abs_cte_m"] = result["yt_m"].abs()
    result["heading_error_deg"] = wrap_degrees(
        result["heading_compass_deg"] - result["path_compass_deg"]
    )
    # Positive is geometrically left of the stripe's travel direction.
    offset_x = result["pos_x_m"] - result["start_x_m"]
    offset_y = result["pos_y_m"] - result["start_y_m"]
    result["geometric_signed_cte_m"] = (
        -result["unit_y"] * offset_x + result["unit_x"] * offset_y
    )
    result["geometric_abs_cte_m"] = result["geometric_signed_cte_m"].abs()
    result["speed_shortfall_mps"] = (
        result["speed_cmd_mps"] - result["actual_speed_mps"]
    )
    result["steer_tracking_abs"] = result["steer_error"].abs()
    result["steer_command_saturated"] = result["steer_normalized"].abs() >= 0.98
    result["steer_pwm_saturated"] = result["steer_pwm"].abs() >= 250
    return result


def summarize_group(group: pd.DataFrame) -> dict[str, float | int | str]:
    weights = group["time_weight_s"]
    return {
        "samples": int(len(group)),
        "duration_s": float(weights.sum()),
        "controller_signed_cte_m": weighted_mean(
            group["controller_signed_cte_m"], weights
        ),
        "controller_mean_abs_cte_m": weighted_abs_mean(
            group["controller_signed_cte_m"], weights
        ),
        "geometric_signed_cte_m": weighted_mean(
            group["geometric_signed_cte_m"], weights
        ),
        "geometric_mean_abs_cte_m": weighted_abs_mean(
            group["geometric_signed_cte_m"], weights
        ),
        "commanded_speed_mps": weighted_mean(group["speed_cmd_mps"], weights),
        "actual_speed_mps": weighted_mean(group["actual_speed_mps"], weights),
        "speed_shortfall_mps": weighted_mean(
            group["speed_shortfall_mps"], weights
        ),
        "heading_error_deg": weighted_mean(group["heading_error_deg"], weights),
        "heading_mean_abs_error_deg": weighted_abs_mean(
            group["heading_error_deg"], weights
        ),
        "steer_command_mean": weighted_mean(
            group["steer_normalized"], weights
        ),
        "steer_command_mean_abs": weighted_abs_mean(
            group["steer_normalized"], weights
        ),
        "steer_tracking_error": weighted_mean(group["steer_error"], weights),
        "steer_tracking_mean_abs_error": weighted_abs_mean(
            group["steer_error"], weights
        ),
        "steer_pwm_mean_abs": weighted_abs_mean(group["steer_pwm"], weights),
        "steer_command_saturation_pct": weighted_pct(
            group["steer_command_saturated"], weights
        ),
        "steer_pwm_saturation_pct": weighted_pct(
            group["steer_pwm_saturated"], weights
        ),
    }


def grouped_summary(frame: pd.DataFrame, by: str) -> pd.DataFrame:
    rows = []
    for key, group in frame.groupby(by, observed=True, sort=True):
        row = {by: key}
        row.update(summarize_group(group))
        rows.append(row)
    return pd.DataFrame(rows)


def linear_fit(x: pd.Series, y: pd.Series) -> dict[str, float]:
    mask = x.notna() & y.notna()
    xv, yv = x[mask].to_numpy(float), y[mask].to_numpy(float)
    if len(xv) < 3 or float(np.std(xv)) == 0:
        return {"n": int(len(xv)), "slope": float("nan"), "r2": float("nan")}
    slope, intercept = np.polyfit(xv, yv, 1)
    prediction = intercept + slope * xv
    ss_res = float(np.sum((yv - prediction) ** 2))
    ss_tot = float(np.sum((yv - np.mean(yv)) ** 2))
    return {
        "n": int(len(xv)),
        "slope": float(slope),
        "intercept": float(intercept),
        "r2": 1.0 - ss_res / ss_tot if ss_tot else float("nan"),
    }


def svg_plot(
    path: Path,
    title: str,
    panels: list[dict[str, object]],
    width: int = 1200,
    panel_height: int = 260,
) -> None:
    height = 70 + panel_height * len(panels)
    margin_left, margin_right = 85, 30
    plot_width = width - margin_left - margin_right
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width / 2}" y="32" text-anchor="middle" '
        'font-family="sans-serif" font-size="20" font-weight="bold">'
        f"{title}</text>",
    ]
    for panel_index, panel in enumerate(panels):
        top = 60 + panel_index * panel_height
        bottom = top + panel_height - 50
        left, right = margin_left, width - margin_right
        series = panel["series"]
        all_x = np.concatenate(
            [np.asarray(item["x"], dtype=float) for item in series]
        )
        all_y = np.concatenate(
            [np.asarray(item["y"], dtype=float) for item in series]
        )
        finite_x, finite_y = all_x[np.isfinite(all_x)], all_y[np.isfinite(all_y)]
        xmin, xmax = float(finite_x.min()), float(finite_x.max())
        ymin, ymax = float(finite_y.min()), float(finite_y.max())
        if xmin == xmax:
            xmin, xmax = xmin - 1, xmax + 1
        if ymin == ymax:
            ymin, ymax = ymin - 1, ymax + 1
        ypad = 0.08 * (ymax - ymin)
        ymin, ymax = ymin - ypad, ymax + ypad

        def sx(value: float) -> float:
            return left + (value - xmin) * (right - left) / (xmax - xmin)

        def sy(value: float) -> float:
            return bottom - (value - ymin) * (bottom - top) / (ymax - ymin)

        svg.append(
            f'<rect x="{left}" y="{top}" width="{plot_width}" '
            f'height="{bottom - top}" fill="#fafafa" stroke="#bbb"/>'
        )
        for tick in np.linspace(ymin, ymax, 5):
            ypixel = sy(float(tick))
            svg.append(
                f'<line x1="{left}" y1="{ypixel:.1f}" x2="{right}" '
                f'y2="{ypixel:.1f}" stroke="#ddd"/>'
            )
            svg.append(
                f'<text x="{left - 8}" y="{ypixel + 4:.1f}" text-anchor="end" '
                f'font-family="sans-serif" font-size="11">{tick:.2f}</text>'
            )
        if ymin <= 0 <= ymax:
            svg.append(
                f'<line x1="{left}" y1="{sy(0):.1f}" x2="{right}" '
                f'y2="{sy(0):.1f}" stroke="#777" stroke-dasharray="5 4"/>'
            )
        for item in series:
            points = [
                (sx(float(x)), sy(float(y)))
                for x, y in zip(item["x"], item["y"])
                if np.isfinite(x) and np.isfinite(y)
            ]
            color = str(item.get("color", "#1565c0"))
            svg.append(
                '<polyline fill="none" stroke="{}" stroke-width="2" points="{}"/>'.format(
                    color, " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
                )
            )
            for x, y in points:
                svg.append(
                    f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3.5" fill="{color}"/>'
                )
        svg.append(
            f'<text x="18" y="{(top + bottom) / 2}" text-anchor="middle" '
            f'transform="rotate(-90 18 {(top + bottom) / 2})" '
            f'font-family="sans-serif" font-size="13">{panel["ylabel"]}</text>'
        )
        svg.append(
            f'<text x="{(left + right) / 2}" y="{bottom + 35}" text-anchor="middle" '
            f'font-family="sans-serif" font-size="13">{panel["xlabel"]}</text>'
        )
        legend_x = left + 10
        for item in series:
            svg.append(
                f'<rect x="{legend_x}" y="{top + 9}" width="13" height="3" '
                f'fill="{item.get("color", "#1565c0")}"/>'
            )
            svg.append(
                f'<text x="{legend_x + 18}" y="{top + 15}" '
                f'font-family="sans-serif" font-size="11">{item["label"]}</text>'
            )
            legend_x += 180
    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")


def circling_diagnostics(
    aligned: pd.DataFrame, audit: pd.DataFrame, output_dir: Path
) -> dict[str, object]:
    window = aligned[aligned["elapsed_s"].between(3180.0, 3230.0)].copy()
    window = window[window["driving_bool"] & window["pos_x_m"].notna()].copy()
    heading_rad = np.radians(90.0 - window["heading_compass_deg"])
    dx = window["target_x_m"] - window["pos_x_m"]
    dy = window["target_y_m"] - window["pos_y_m"]
    window["target_rel_x_m"] = dx * np.cos(heading_rad) + dy * np.sin(heading_rad)
    window["target_rel_y_m"] = -dx * np.sin(heading_rad) + dy * np.cos(heading_rad)
    window["target_distance_m"] = np.hypot(dx, dy)

    origin_e = float(audit.iloc[0]["east_m"])
    origin_n = float(audit.iloc[0]["north_m"])
    audit_local = audit.copy()
    audit_local["x_m"] = audit_local["east_m"] - origin_e
    audit_local["y_m"] = audit_local["north_m"] - origin_n
    connector = audit_local[
        audit_local["waypoint_idx"].between(4015, 4060)
    ].copy()

    candidates = connector[["waypoint_idx", "x_m", "y_m"]].to_numpy(float)
    nearest_indices = []
    nearest_distances = []
    for row in window.itertuples():
        distances = np.hypot(
            candidates[:, 1] - row.pos_x_m, candidates[:, 2] - row.pos_y_m
        )
        choice = int(np.argmin(distances))
        nearest_indices.append(int(candidates[choice, 0]))
        nearest_distances.append(float(distances[choice]))
    window["nearest_connector_waypoint_idx"] = nearest_indices
    window["nearest_connector_distance_m"] = nearest_distances

    one_second = window.assign(second=window["elapsed_s"].round().astype(int))
    one_second = one_second.groupby("second", as_index=False).first()
    columns = (
        "elapsed_s",
        "pos_x_m",
        "pos_y_m",
        "heading_compass_deg",
        "waypoint_idx",
        "target_x_m",
        "target_y_m",
        "target_distance_m",
        "target_rel_x_m",
        "target_rel_y_m",
        "nearest_connector_waypoint_idx",
        "nearest_connector_distance_m",
        "yt_m",
        "steer_normalized",
        "actual_speed_mps",
    )
    one_second[list(columns)].to_csv(
        output_dir / "circling_window_1hz.csv", index=False
    )
    connector.to_csv(output_dir / "circling_connector_audit.csv", index=False)

    tail = aligned[
        aligned["driving_bool"]
        & aligned["trans_mode"].eq(AUTO_MODE)
        & aligned["waypoint_idx"].eq(4033)
        & aligned["elapsed_s"].notna()
    ].copy()
    tail_heading_rad = np.radians(90.0 - tail["heading_compass_deg"])
    tail_dx = tail["target_x_m"] - tail["pos_x_m"]
    tail_dy = tail["target_y_m"] - tail["pos_y_m"]
    tail["target_distance_m"] = np.hypot(tail_dx, tail_dy)
    tail["target_rel_x_m"] = (
        tail_dx * np.cos(tail_heading_rad) + tail_dy * np.sin(tail_heading_rad)
    )
    heading_unwrapped = np.unwrap(np.radians(tail["heading_compass_deg"].dropna()))
    rotation_deg = (
        math.degrees(float(np.sum(np.abs(np.diff(heading_unwrapped)))))
        if len(heading_unwrapped) > 1
        else 0.0
    )
    path_length = float(
        np.hypot(tail["pos_x_m"].diff(), tail["pos_y_m"].diff()).sum()
    )
    summary = {
        "requested_window_samples": int(len(window)),
        "window_waypoint_indices": sorted(
            int(value) for value in window["waypoint_idx"].dropna().unique()
        ),
        "target_behind_pct": float((window["target_rel_x_m"] < 0).mean() * 100),
        "target_distance_min_m": float(window["target_distance_m"].min()),
        "target_distance_max_m": float(window["target_distance_m"].max()),
        "later_connector_closer_pct": float(
            (
                (window["nearest_connector_waypoint_idx"] > window["waypoint_idx"])
                & (
                    window["nearest_connector_distance_m"]
                    < window["target_distance_m"]
                )
            ).mean()
            * 100
        ),
        "locked_waypoint_first_elapsed_s": float(tail["elapsed_s"].min()),
        "locked_waypoint_last_elapsed_s": float(tail["elapsed_s"].max()),
        "locked_duration_s": float(
            tail["elapsed_s"].max() - tail["elapsed_s"].min()
        ),
        "locked_path_length_m": path_length,
        "locked_accumulated_heading_change_deg": rotation_deg,
        "approx_full_rotations": rotation_deg / 360.0,
        "locked_target_distance_min_m": float(tail["target_distance_m"].min()),
        "locked_target_distance_max_m": float(tail["target_distance_m"].max()),
        "locked_target_behind_pct": float(
            (tail["target_rel_x_m"] < 0).mean() * 100
        ),
    }

    svg_plot(
        output_dir / "circling_event.svg",
        "Final connector lock: tractor path and planned connector",
        [
            {
                "xlabel": "Controller local X (m)",
                "ylabel": "Controller local Y (m)",
                "series": [
                    {
                        "x": connector["x_m"],
                        "y": connector["y_m"],
                        "label": "planned connector",
                        "color": "#ef6c00",
                    },
                    {
                        "x": window["pos_x_m"],
                        "y": window["pos_y_m"],
                        "label": "tractor 3180–3230 s",
                        "color": "#1565c0",
                    },
                    {
                        "x": window["target_x_m"].drop_duplicates(),
                        "y": window.loc[
                            ~window["target_x_m"].duplicated(), "target_y_m"
                        ],
                        "label": "selected target",
                        "color": "#c62828",
                    },
                ],
            },
            {
                "xlabel": "Elapsed time (s)",
                "ylabel": "Distance / body X (m)",
                "series": [
                    {
                        "x": one_second["elapsed_s"],
                        "y": one_second["target_distance_m"],
                        "label": "target distance",
                        "color": "#1565c0",
                    },
                    {
                        "x": one_second["elapsed_s"],
                        "y": one_second["target_rel_x_m"],
                        "label": "target body-forward X",
                        "color": "#c62828",
                    },
                    {
                        "x": one_second["elapsed_s"],
                        "y": one_second["nearest_connector_distance_m"],
                        "label": "nearest connector distance",
                        "color": "#2e7d32",
                    },
                ],
            },
        ],
    )
    return summary


def main() -> int:
    args = parser().parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    aligned, audit, _mission = read_inputs(args)
    meta = stripe_metadata(audit)
    detailed = attach_stripe_metrics(aligned, audit, meta)
    stripe = detailed[
        detailed["driving_bool"]
        & detailed["trans_mode"].eq(AUTO_MODE)
        & detailed["kind"].eq("stripe")
        & detailed["controller_signed_cte_m"].notna()
        & detailed["pos_x_m"].notna()
    ].copy()

    per_row = grouped_summary(stripe, "stripe_id").merge(
        meta[
            [
                "stripe_id",
                "north_rank",
                "mean_north_m",
                "start_waypoint_idx",
                "end_waypoint_idx",
                "length_m",
                "path_compass_deg",
                "direction",
            ]
        ],
        on="stripe_id",
    )
    per_row = per_row.sort_values("mean_north_m")
    per_row.to_csv(args.output_dir / "stripe_by_row.csv", index=False)

    stripe["geographic_quartile"] = pd.qcut(
        stripe["mean_north_m"],
        4,
        labels=("south", "south-mid", "north-mid", "north"),
        duplicates="drop",
    )
    geographic = grouped_summary(stripe, "geographic_quartile")
    geographic.to_csv(args.output_dir / "stripe_by_geographic_quartile.csv", index=False)
    by_direction = grouped_summary(stripe, "direction")
    by_direction.to_csv(args.output_dir / "stripe_by_direction.csv", index=False)
    direction_geography_rows = []
    for (direction, quartile), group in stripe.groupby(
        ["direction", "geographic_quartile"], observed=True, sort=True
    ):
        row = {"direction": direction, "geographic_quartile": quartile}
        row.update(summarize_group(group))
        direction_geography_rows.append(row)
    direction_geography = pd.DataFrame(direction_geography_rows)
    direction_geography.to_csv(
        args.output_dir / "stripe_by_direction_and_geography.csv", index=False
    )

    stripe["actual_speed_bin"] = pd.cut(
        stripe["actual_speed_mps"],
        bins=(-np.inf, 0.55, 0.65, 0.72, np.inf),
        labels=("<0.55", "0.55–0.65", "0.65–0.72", ">=0.72"),
    )
    grouped_summary(stripe, "actual_speed_bin").to_csv(
        args.output_dir / "stripe_by_actual_speed.csv", index=False
    )
    stripe["heading_error_bin"] = pd.cut(
        stripe["heading_error_deg"].abs(),
        bins=(-np.inf, 2, 5, 10, np.inf),
        labels=("0–2", "2–5", "5–10", ">10"),
    )
    grouped_summary(stripe, "heading_error_bin").to_csv(
        args.output_dir / "stripe_by_heading_error.csv", index=False
    )

    mid_north = float(per_row["mean_north_m"].median())
    stripe["north_half"] = np.where(
        stripe["mean_north_m"] >= mid_north, "north_half", "south_half"
    )
    halves = grouped_summary(stripe, "north_half")
    halves.to_csv(args.output_dir / "stripe_north_vs_south.csv", index=False)

    regression = {
        "row_mean_controller_abs_cte_vs_north": linear_fit(
            per_row["mean_north_m"], per_row["controller_mean_abs_cte_m"]
        ),
        "row_mean_controller_signed_cte_vs_north": linear_fit(
            per_row["mean_north_m"], per_row["controller_signed_cte_m"]
        ),
        "row_mean_geometric_abs_cte_vs_north": linear_fit(
            per_row["mean_north_m"], per_row["geometric_mean_abs_cte_m"]
        ),
        "row_mean_geometric_signed_cte_vs_north": linear_fit(
            per_row["mean_north_m"], per_row["geometric_signed_cte_m"]
        ),
        "row_actual_speed_vs_north": linear_fit(
            per_row["mean_north_m"], per_row["actual_speed_mps"]
        ),
        "row_steering_tracking_abs_vs_north": linear_fit(
            per_row["mean_north_m"], per_row["steer_tracking_mean_abs_error"]
        ),
    }

    circling = circling_diagnostics(aligned, audit, args.output_dir)
    summary = {
        "site_name": args.site_name,
        "run_id": args.run_id,
        "stripe_samples": int(len(stripe)),
        "stripe_duration_s": float(stripe["time_weight_s"].sum()),
        "stripe_rows_observed": int(per_row["stripe_id"].nunique()),
        "overall_stripe": summarize_group(stripe),
        "north_vs_south": halves.to_dict(orient="records"),
        "by_direction": by_direction.to_dict(orient="records"),
        "by_direction_and_geography": direction_geography.to_dict(
            orient="records"
        ),
        "row_regressions": regression,
        "circling": circling,
    }
    (args.output_dir / "diagnostic_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8"
    )

    svg_plot(
        args.output_dir / "stripe_row_performance.svg",
        "Straight-stripe performance by geographic northing",
        [
            {
                "xlabel": "Stripe mean northing (m; left=south, right=north)",
                "ylabel": "Mean absolute CTE (m)",
                "series": [
                    {
                        "x": per_row["mean_north_m"],
                        "y": per_row["controller_mean_abs_cte_m"],
                        "label": "controller lookahead CTE",
                        "color": "#c62828",
                    },
                    {
                        "x": per_row["mean_north_m"],
                        "y": per_row["geometric_mean_abs_cte_m"],
                        "label": "geometric line CTE",
                        "color": "#1565c0",
                    },
                ],
            },
            {
                "xlabel": "Stripe mean northing (m; left=south, right=north)",
                "ylabel": "Signed CTE (m; +left of travel)",
                "series": [
                    {
                        "x": per_row["mean_north_m"],
                        "y": per_row["controller_signed_cte_m"],
                        "label": "controller signed CTE",
                        "color": "#c62828",
                    },
                    {
                        "x": per_row["mean_north_m"],
                        "y": per_row["geometric_signed_cte_m"],
                        "label": "geometric signed CTE",
                        "color": "#1565c0",
                    },
                ],
            },
            {
                "xlabel": "Stripe mean northing (m; left=south, right=north)",
                "ylabel": "Speed (m/s)",
                "series": [
                    {
                        "x": per_row["mean_north_m"],
                        "y": per_row["commanded_speed_mps"],
                        "label": "commanded speed",
                        "color": "#555555",
                    },
                    {
                        "x": per_row["mean_north_m"],
                        "y": per_row["actual_speed_mps"],
                        "label": "actual speed",
                        "color": "#2e7d32",
                    },
                ],
            },
            {
                "xlabel": "Stripe mean northing (m; left=south, right=north)",
                "ylabel": "Steering diagnostic",
                "series": [
                    {
                        "x": per_row["mean_north_m"],
                        "y": per_row["steer_command_mean_abs"],
                        "label": "|Pure Pursuit command|",
                        "color": "#6a1b9a",
                    },
                    {
                        "x": per_row["mean_north_m"],
                        "y": per_row["steer_tracking_mean_abs_error"] / 100.0,
                        "label": "|steer tracking error| / 100",
                        "color": "#ef6c00",
                    },
                ],
            },
        ],
    )

    markdown = [
        f"# Diagnostic evidence: {args.site_name} / {args.run_id}",
        "",
        f"- Stripe samples: {len(stripe):,}",
        f"- Stripe analyzed duration: {stripe['time_weight_s'].sum():.1f} s",
        f"- Stripe rows observed: {per_row['stripe_id'].nunique()}",
        "",
        "## Generated evidence",
        "",
        "- `stripe_by_row.csv`",
        "- `stripe_north_vs_south.csv`",
        "- `stripe_by_geographic_quartile.csv`",
        "- `stripe_by_direction.csv`",
        "- `stripe_by_direction_and_geography.csv`",
        "- `stripe_by_actual_speed.csv`",
        "- `stripe_by_heading_error.csv`",
        "- `stripe_row_performance.svg`",
        "- `circling_window_1hz.csv`",
        "- `circling_connector_audit.csv`",
        "- `circling_event.svg`",
        "- `diagnostic_summary.json`",
    ]
    (args.output_dir / "README.md").write_text(
        "\n".join(markdown) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
