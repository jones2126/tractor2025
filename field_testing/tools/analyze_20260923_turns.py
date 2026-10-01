#!/usr/bin/env python3
"""Diagnose upper-left versus lower-right turns in the 2026-09-23 mission.

The script is intentionally read-only with respect to mission/controller inputs.
It recomputes geometric cross-track error from the recorded position and the
planned polyline; the pursuit log's ``cross_track_err_m`` is retained only for
comparison.
"""

from __future__ import annotations

import bisect
import csv
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent
SITE = ROOT / "sites" / "62_Collins_multi_boundary_20260915"
RUN = SITE / "runs" / "20260923_124933_master_manual_5hz"
PLAN = SITE / "mission_plans" / "20260915_master_boundary_replay" / "generated_master_manual_field_20260920"
PURSUIT = RUN / "pursuit_log_20260923_124935.csv"
FIELD = RUN / "master_manual_field_20260923_124933.csv"
MISSION = PLAN / "62_Collins_master_manual_resampled_1mps_20260920.txt"
AUDIT = PLAN / "62_Collins_master_manual_resampled_1mps_20260920_audit.csv"
OUTPUT = SITE / "analysis" / "20260923_turn_diagnosis"
REPLAY = SITE / "analysis" / "20260923_mission_replay" / "replay_20260923_124933_master_manual_5hz.html"
REPLAY_BUILDER = ROOT / "tools" / "build_tractor01_mission_replay_20260923.py"
MISSION_NOTES = PLAN.parent / "MASTER_MANUAL_FIELD_TEST_20260920.md"


@dataclass
class Turn:
    label: str
    start: int
    apex: int
    end: int
    direction: str
    region: str
    ring: int = 0


def read_inputs():
    mission = pd.read_csv(
        MISSION,
        sep=r"\s+",
        names=["lat", "lon", "yaw_rad", "lookahead_m", "speed_mps"],
    )
    audit = pd.read_csv(AUDIT)
    if len(mission) != len(audit):
        raise ValueError(f"mission/audit mismatch: {len(mission)} != {len(audit)}")
    mission["waypoint"] = np.arange(1, len(mission) + 1)
    mission["phase"] = audit["phase"].astype(str)
    lat0 = float(mission.lat.iloc[0])
    lon0 = float(mission.lon.iloc[0])
    sx = 111_320.0 * math.cos(math.radians(lat0))
    sy = 110_540.0
    mission["x"] = (mission.lon - lon0) * sx
    mission["y"] = (mission.lat - lat0) * sy

    pursuit = pd.read_csv(PURSUIT, skiprows=[1], low_memory=False)
    for name in pursuit.columns:
        if name not in {"fix_quality", "heading_carrier", "wait_reason", "handheld_state", "reacquire_state", "reacquire_detail"}:
            pursuit[name] = pd.to_numeric(pursuit[name], errors="coerce")
    pursuit = pursuit[pursuit.timestamp.notna()].reset_index(drop=True)

    field = pd.read_csv(FIELD, low_memory=False)
    parsed_time = pd.to_datetime(field["time"], utc=True, errors="coerce")
    field["timestamp"] = parsed_time.map(lambda value: value.timestamp() if pd.notna(value) else np.nan)
    field = field[field.timestamp.notna()].sort_values("timestamp").reset_index(drop=True)
    return mission, pursuit, field, (lat0, lon0, sx, sy)


def wrap_deg(values):
    return (np.asarray(values) + 180.0) % 360.0 - 180.0


def compass_heading(dx, dy):
    return (90.0 - math.degrees(math.atan2(dy, dx))) % 360.0


def planned_geometry(mission):
    x = mission.x.to_numpy(float)
    y = mission.y.to_numpy(float)
    dx = np.gradient(x)
    dy = np.gradient(y)
    heading = np.unwrap(np.arctan2(dy, dx))
    ds = np.maximum(np.hypot(dx, dy), 1e-6)
    curvature = np.gradient(heading) / ds
    # A 1.05 m centered window suppresses resampling noise while retaining U-turns.
    curvature = pd.Series(curvature).rolling(7, center=True, min_periods=1).median().to_numpy()
    mission["path_heading_rad"] = heading
    mission["curvature_inv_m"] = curvature
    seg = np.hypot(np.diff(x), np.diff(y))
    mission["path_s_m"] = np.r_[0.0, np.cumsum(seg)]
    return mission


def detect_ring_turns(mission, selected_rings=(4, 8, 13)):
    """Find southeast and northwest sustained-curvature corners on each ring."""
    heading = mission.path_heading_rad.to_numpy(float)
    curvature = mission.curvature_inv_m.to_numpy(float)
    all_turns = []
    selected = []
    selected_number = {ring: number for number, ring in enumerate(selected_rings, 1)}
    for ring in range(1, 19):  # ring 19 is a short inner remnant, not a matched loop
        phase = f"main_backyard_ring_{ring}"
        indices = np.flatnonzero(mission.phase.eq(phase).to_numpy())
        if len(indices) < 80:
            continue
        x = mission.x.iloc[indices].to_numpy(float)
        y = mission.y.iloc[indices].to_numpy(float)
        zx = (x - x.mean()) / max(x.std(), 1e-9)
        zy = (y - y.mean()) / max(y.std(), 1e-9)
        candidates = {
            "lower-right": int(indices[np.argmax(zx - zy)]),
            "upper-left": int(indices[np.argmax(-zx + zy)]),
        }
        lo, hi = int(indices[0]), int(indices[-1])
        for region, candidate in candidates.items():
            local = np.arange(max(lo, candidate - 70), min(hi, candidate + 70) + 1)
            active = local[np.abs(curvature[local]) >= 0.15]
            clusters = []
            for index in active:
                if not clusters or index - clusters[-1][-1] > 4:
                    clusters.append([int(index)])
                else:
                    clusters[-1].append(int(index))
            if not clusters:
                raise ValueError(f"no curve cluster for {phase} {region}")
            cluster = min(clusters, key=lambda values: min(abs(candidate - index) for index in values))
            a = max(lo, cluster[0] - 2)
            b = min(hi, cluster[-1] + 2)
            midpoint_heading = (heading[a] + heading[b]) / 2.0
            curve_indices = np.arange(a, b + 1)
            apex = int(curve_indices[np.argmin(np.abs(heading[curve_indices] - midpoint_heading))])
            delta = heading[b] - heading[a]
            side = "LR" if region == "lower-right" else "UL"
            turn = Turn(f"R{ring:02d}-{side}", a, apex, b,
                        "left" if delta > 0 else "right", region, ring)
            all_turns.append(turn)
            if ring in selected_number:
                turn.label = f"{side}-{selected_number[ring]}"
                selected.append(turn)
    return sorted(selected, key=lambda turn: (turn.ring, turn.region)), sorted(all_turns, key=lambda turn: (turn.ring, turn.region))


def add_geometric_tracking(pursuit, mission):
    ring_mask = mission.phase.str.match(r"main_backyard_ring_\d+$").to_numpy()
    ring_indices = np.flatnonzero(ring_mask)
    lo, hi = int(ring_indices[0]), int(ring_indices[-1])
    keep = (
        pursuit.pos_x_m.notna()
        & pursuit.pos_y_m.notna()
        & pursuit.waypoint_idx.between(lo - 50, hi + 50)
        & pursuit.driving.astype(str).str.lower().isin({"true", "1", "1.0"})
    )
    data = pursuit.loc[keep].copy().sort_values("timestamp").reset_index(drop=True)
    x = mission.x.to_numpy(float)
    y = mission.y.to_numpy(float)
    path_s = mission.path_s_m.to_numpy(float)
    results = []
    for row in data.itertuples(index=False):
        hint = max(lo, min(hi, int(row.waypoint_idx)))
        # The active waypoint is normally ~lookahead/spacing points ahead of the
        # tractor. The asymmetric window prevents a nearby adjacent pass from
        # being mistaken for the current segment.
        first = max(lo, hint - 100)
        last = min(hi, hint + 35)
        ax = x[first:last]
        ay = y[first:last]
        bx = x[first + 1:last + 1]
        by = y[first + 1:last + 1]
        vx, vy = bx - ax, by - ay
        denom = np.maximum(vx * vx + vy * vy, 1e-12)
        t = np.clip(((row.pos_x_m - ax) * vx + (row.pos_y_m - ay) * vy) / denom, 0.0, 1.0)
        qx, qy = ax + t * vx, ay + t * vy
        d2 = (row.pos_x_m - qx) ** 2 + (row.pos_y_m - qy) ** 2
        j = int(np.argmin(d2))
        seg_idx = first + j
        seg_len = math.hypot(vx[j], vy[j])
        signed = (vx[j] * (row.pos_y_m - qy[j]) - vy[j] * (row.pos_x_m - qx[j])) / max(seg_len, 1e-9)
        plan_compass = compass_heading(vx[j], vy[j])
        results.append(
            (
                seg_idx,
                path_s[seg_idx] + t[j] * seg_len,
                signed,
                abs(signed),
                plan_compass,
                float(wrap_deg(row.heading_compass_deg - plan_compass)) if pd.notna(row.heading_compass_deg) else np.nan,
                qx[j],
                qy[j],
            )
        )
    columns = [
        "nearest_segment", "geom_path_s_m", "geom_cte_signed_m", "geom_cte_abs_m",
        "planned_heading_compass_deg", "path_heading_error_deg", "nearest_x_m", "nearest_y_m",
    ]
    for number, name in enumerate(columns):
        data[name] = [result[number] for result in results]
    return data


def join_field(data, field):
    names = [
        "timestamp", "steer_setpoint", "steer_current", "steer_error", "steer_pwm",
        "steer_direction", "steer_pid_output", "expected_heading_error_deg",
        "fix_quality", "head_valid", "carrier", "relpos_heading_accuracy_deg",
    ]
    available = [name for name in names if name in field.columns]
    subset = field[available].copy()
    rename = {name: f"field_{name}" for name in available if name != "timestamp"}
    subset = subset.rename(columns=rename)
    for name in subset.columns:
        if name not in {"timestamp", "field_fix_quality", "field_carrier", "field_head_valid", "field_steer_direction"}:
            subset[name] = pd.to_numeric(subset[name], errors="coerce")
    return pd.merge_asof(
        data.sort_values("timestamp"), subset.sort_values("timestamp"), on="timestamp",
        direction="nearest", tolerance=0.08,
    )


def q(series, percentile):
    values = pd.to_numeric(series, errors="coerce").dropna().to_numpy(float)
    return float(np.percentile(values, percentile)) if len(values) else math.nan


def mean(series):
    return float(pd.to_numeric(series, errors="coerce").mean())


def median(series):
    return float(pd.to_numeric(series, errors="coerce").median())


def steering_lag_seconds(frame):
    target = pd.to_numeric(frame.field_steer_setpoint, errors="coerce").to_numpy(float)
    actual = pd.to_numeric(frame.field_steer_current, errors="coerce").to_numpy(float)
    valid = np.isfinite(target) & np.isfinite(actual)
    target, actual = target[valid], actual[valid]
    if len(target) < 20 or np.std(target) < 1 or np.std(actual) < 1:
        return math.nan
    best = (float("-inf"), 0)
    for lag in range(0, min(21, len(target) // 3)):
        a = target[: len(target) - lag or None]
        b = actual[lag:]
        if len(a) >= 10 and np.std(a) > 0 and np.std(b) > 0:
            corr = float(np.corrcoef(a, b)[0, 1])
            if np.isfinite(corr) and corr > best[0]:
                best = (corr, lag)
    dt = float(np.median(np.diff(frame.timestamp.to_numpy(float))))
    return best[1] * dt


def turn_metrics(turn, mission, tracking):
    s0 = float(mission.path_s_m.iloc[turn.start])
    sa = float(mission.path_s_m.iloc[turn.apex])
    s1 = float(mission.path_s_m.iloc[turn.end])
    span = s1 - s0
    recovery_end = s1 + 0.15 * span
    frame = tracking[tracking.geom_path_s_m.between(s0, recovery_end)].copy()
    frame["curve_progress"] = (frame.geom_path_s_m - s0) / span
    frame["turn_label"] = turn.label
    frame["turn_region"] = turn.region
    frame["turn_direction"] = turn.direction
    inside_sign = 1.0 if turn.direction == "left" else -1.0
    frame["inside_cte_m"] = frame.geom_cte_signed_m * inside_sign
    core = frame[frame.curve_progress.between(0.0, 1.0)].copy()
    apex = frame[frame.curve_progress.between(0.40, 0.60)]
    entry = frame[frame.curve_progress.between(0.00, 0.25)]
    exit_recovery = frame[frame.curve_progress.between(0.75, 1.15)]
    net_heading = float(mission.path_heading_rad.iloc[turn.end] - mission.path_heading_rad.iloc[turn.start])
    curve_length = span
    arc_equivalent_radius = curve_length / max(abs(net_heading), 1e-9)
    curve_curvature = np.abs(mission.curvature_inv_m.iloc[turn.start:turn.end + 1].to_numpy(float))
    curve_curvature = curve_curvature[curve_curvature >= 0.15]
    median_curvature = float(np.median(curve_curvature))
    radius = 1.0 / median_curvature
    p0, pa, p1 = mission.iloc[turn.start], mission.iloc[turn.apex], mission.iloc[turn.end]
    endpoint_spacing = math.hypot(p1.x - p0.x, p1.y - p0.y)
    logged = pd.to_numeric(core.cross_track_err_m, errors="coerce")
    geom_abs = core.geom_cte_abs_m
    pair = pd.DataFrame({"logged": logged, "geom": geom_abs}).dropna()
    corr = float(pair.corr().iloc[0, 1]) if len(pair) > 2 and pair.logged.std() > 0 and pair.geom.std() > 0 else math.nan
    rmse = float(np.sqrt(np.mean((pair.logged - pair.geom) ** 2))) if len(pair) else math.nan
    fix = core.fix_quality.astype(str).str.lower()
    carrier = core.heading_carrier.astype(str).str.lower()
    head_valid = core.head_valid.astype(str).str.lower().isin({"true", "1", "1.0"})
    target = pd.to_numeric(core.get("field_steer_setpoint"), errors="coerce")
    measured = pd.to_numeric(core.get("field_steer_current"), errors="coerce")
    steer_err = target - measured
    values = {
        "label": turn.label,
        "ring": turn.ring,
        "region": turn.region,
        "direction": turn.direction,
        "entry_waypoint": turn.start + 1,
        "apex_waypoint": turn.apex + 1,
        "exit_waypoint": turn.end + 1,
        "entry_lat": p0.lat, "entry_lon": p0.lon, "entry_x_m": p0.x, "entry_y_m": p0.y,
        "apex_lat": pa.lat, "apex_lon": pa.lon, "apex_x_m": pa.x, "apex_y_m": pa.y,
        "exit_lat": p1.lat, "exit_lon": p1.lon, "exit_x_m": p1.x, "exit_y_m": p1.y,
        "approach_heading_compass_deg": compass_heading(
            mission.x.iloc[turn.start + 1] - p0.x, mission.y.iloc[turn.start + 1] - p0.y),
        "curve_length_m": curve_length,
        "net_heading_change_deg": math.degrees(net_heading),
        "approx_radius_m": radius,
        "arc_equivalent_radius_m": arc_equivalent_radius,
        "approx_abs_curvature_inv_m": median_curvature,
        "endpoint_spacing_m": endpoint_spacing,
        "samples": len(core),
        "commanded_speed_mean_mps": mean(core.speed_cmd_mps),
        "actual_speed_mean_mps": mean(core.actual_speed_mps),
        "actual_speed_p90_mps": q(core.actual_speed_mps, 90),
        "cte_signed_median_m": median(core.geom_cte_signed_m),
        "cte_signed_mean_m": mean(core.geom_cte_signed_m),
        "cte_abs_median_m": median(core.geom_cte_abs_m),
        "cte_abs_mean_m": mean(core.geom_cte_abs_m),
        "cte_abs_p90_m": q(core.geom_cte_abs_m, 90),
        "cte_abs_p95_m": q(core.geom_cte_abs_m, 95),
        "cte_abs_max_m": float(core.geom_cte_abs_m.max()),
        "inside_cte_median_m": median(core.inside_cte_m),
        "entry_cte_signed_median_m": median(entry.geom_cte_signed_m),
        "entry_inside_cte_median_m": median(entry.inside_cte_m),
        "apex_cte_signed_median_m": median(apex.geom_cte_signed_m),
        "apex_inside_cte_median_m": median(apex.inside_cte_m),
        "apex_cte_abs_p95_m": q(apex.geom_cte_abs_m, 95),
        "exit_recovery_cte_signed_median_m": median(exit_recovery.geom_cte_signed_m),
        "exit_recovery_inside_cte_median_m": median(exit_recovery.inside_cte_m),
        "exit_recovery_cte_abs_p95_m": q(exit_recovery.geom_cte_abs_m, 95),
        "path_heading_error_median_deg": median(core.path_heading_error_deg),
        "path_heading_error_abs_p95_deg": q(np.abs(core.path_heading_error_deg), 95),
        "pure_pursuit_alpha_median_deg": median(core.alpha_deg),
        "steer_normalized_median": median(core.steer_normalized),
        "steer_command_saturated_pct": 100.0 * float((pd.to_numeric(core.steer_normalized, errors="coerce").abs() >= 0.99).mean()),
        "steer_target_counts_median": median(target),
        "steer_measured_counts_median": median(measured),
        "steer_error_counts_mean": mean(steer_err),
        "steer_error_counts_abs_p95": q(np.abs(steer_err), 95),
        "steering_lag_s": steering_lag_seconds(core),
        "lookahead_median_m": median(core.lookahead_dist_m),
        "rtk_fixed_pct": 100.0 * float(fix.eq("rtk fixed").mean()),
        "heading_valid_pct": 100.0 * float(head_valid.mean()),
        "carrier_fixed_pct": 100.0 * float(carrier.eq("fixed").mean()),
        "heading_accuracy_p95_deg": q(core.heading_accuracy_deg, 95),
        "logged_cte_mean_m": mean(logged),
        "logged_cte_p95_m": q(logged, 95),
        "logged_vs_geom_abs_corr": corr,
        "logged_vs_geom_abs_rmse_m": rmse,
    }
    return values, frame


def fmt(value, digits=3):
    return "—" if pd.isna(value) else f"{float(value):.{digits}f}"


def svg_escape(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def svg_polyline(points, color, width=2, dash=None, opacity=1.0):
    coords = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{width}" opacity="{opacity}"{extra}/>'


def save_plan_svg(mission, tracking, turns, path):
    width, height = 900, 760
    margin = 70
    ring_indices = np.flatnonzero(mission.phase.str.match(r"main_backyard_ring_(?:[1-9]|1[0-8])$").to_numpy())
    selected_lo = int(ring_indices[0])
    selected_hi = int(ring_indices[-1])
    plan = mission.iloc[selected_lo:selected_hi + 1]
    actual = tracking[tracking.geom_path_s_m.between(plan.path_s_m.iloc[0], plan.path_s_m.iloc[-1])]
    xmin = min(plan.x.min(), actual.pos_x_m.min()) - 0.8
    xmax = max(plan.x.max(), actual.pos_x_m.max()) + 0.8
    ymin = min(plan.y.min(), actual.pos_y_m.min()) - 0.8
    ymax = max(plan.y.max(), actual.pos_y_m.max()) + 0.8
    scale = min((width - 2 * margin) / (xmax - xmin), (height - 2 * margin) / (ymax - ymin))
    def tx(x): return margin + (x - xmin) * scale
    def ty(y): return height - margin - (y - ymin) * scale
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
             '<rect width="100%" height="100%" fill="white"/>',
             f'<text x="{width/2}" y="30" text-anchor="middle" font-family="sans-serif" font-size="21">September 23 turn geometry — planned path vs recorded tractor position</text>']
    for value in np.arange(math.floor(xmin / 5) * 5, xmax + 5, 5):
        parts += [f'<line x1="{tx(value):.1f}" y1="{margin}" x2="{tx(value):.1f}" y2="{height-margin}" stroke="#e5e5e5"/>',
                  f'<text x="{tx(value):.1f}" y="{height-margin+20}" text-anchor="middle" font-family="sans-serif" font-size="12">{value:.0f}</text>']
    for value in np.arange(math.floor(ymin / 5) * 5, ymax + 5, 5):
        parts += [f'<line x1="{margin}" y1="{ty(value):.1f}" x2="{width-margin}" y2="{ty(value):.1f}" stroke="#e5e5e5"/>',
                  f'<text x="{margin-10}" y="{ty(value)+4:.1f}" text-anchor="end" font-family="sans-serif" font-size="12">{value:.0f}</text>']
    parts += [svg_polyline([(tx(x), ty(y)) for x, y in zip(plan.x, plan.y)], "#2f5aa8", 3),
              svg_polyline([(tx(x), ty(y)) for x, y in zip(actual.pos_x_m, actual.pos_y_m)], "#d14b3e", 1.5, opacity=0.78)]
    label_offsets = {"LR-2": (12, -16), "LR-3": (-45, 18)}
    for turn in turns:
        p = mission.iloc[turn.apex]
        color = "#8b2f97" if turn.region == "lower-right" else "#198f65"
        ox, oy = label_offsets.get(turn.label, (7, -7))
        parts += [f'<circle cx="{tx(p.x):.1f}" cy="{ty(p.y):.1f}" r="5" fill="{color}"/>',
                  f'<text x="{tx(p.x)+ox:.1f}" y="{ty(p.y)+oy:.1f}" font-family="sans-serif" font-size="15" fill="{color}">{turn.label}</text>']
    parts += [f'<text x="{margin}" y="{height-25}" font-family="sans-serif" font-size="14">East (m); equal scale. Blue planned, red recorded. Lower-right labels purple; upper-left green.</text>', "</svg>"]
    path.write_text("\n".join(parts), encoding="utf-8")


def save_line_panels_svg(frames, metrics, path, kind):
    width, height = 1280, 720
    ml, mr, mt, mb = 80, 35, 70, 70
    plot_w, plot_h = width - ml - mr, height - mt - mb
    colors = {"LR-1":"#7b1fa2","LR-2":"#ab47bc","LR-3":"#5e35b1","LR-4":"#3949ab",
              "UL-1":"#00695c","UL-2":"#00897b","UL-3":"#43a047","UL-4":"#7cb342"}
    if kind == "cte":
        series = [(label, frame.curve_progress.to_numpy(), frame.geom_cte_signed_m.to_numpy()) for label, frame in frames.items()]
        title, ylabel = "Independent signed geometric CTE through each turn", "Signed CTE (m); + = tractor left of path"
        ymin, ymax = -max(abs(np.nanmin(y)) for _,_,y in series), max(abs(np.nanmax(y)) for _,_,y in series)
        lim = max(abs(ymin), abs(ymax), 0.1); ymin, ymax = -lim, lim
    else:
        raise ValueError(kind)
    def tx(v): return ml + np.asarray(v) * plot_w
    def ty(v): return mt + (ymax - np.asarray(v)) / (ymax - ymin) * plot_h
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">', '<rect width="100%" height="100%" fill="white"/>',
             f'<text x="{width/2}" y="30" text-anchor="middle" font-family="sans-serif" font-size="21">{svg_escape(title)}</text>',
             f'<line x1="{ml}" y1="{ty(0):.1f}" x2="{width-mr}" y2="{ty(0):.1f}" stroke="#999" stroke-width="1"/>',
             f'<rect x="{ml}" y="{mt}" width="{plot_w}" height="{plot_h}" fill="none" stroke="#222"/>']
    for value in np.linspace(ymin, ymax, 5):
        parts += [f'<line x1="{ml}" y1="{ty(value):.1f}" x2="{width-mr}" y2="{ty(value):.1f}" stroke="#e5e5e5"/>',
                  f'<text x="{ml-9}" y="{ty(value)+4:.1f}" text-anchor="end" font-family="sans-serif" font-size="12">{value:+.2f}</text>']
    for label, x, y in series:
        valid = np.isfinite(x) & np.isfinite(y) & (x >= 0) & (x <= 1)
        order = np.argsort(x[valid])
        points = list(zip(tx(x[valid][order]), ty(y[valid][order])))
        parts.append(svg_polyline(points, colors[label], 2, opacity=0.8))
    for value, label in ((0,"Entry"),(0.5,"Apex"),(1,"Exit")):
        parts += [f'<line x1="{tx(value):.1f}" y1="{mt}" x2="{tx(value):.1f}" y2="{mt+plot_h}" stroke="#bbb" stroke-dasharray="4 4"/>',
                  f'<text x="{tx(value):.1f}" y="{height-43}" text-anchor="middle" font-family="sans-serif" font-size="14">{label}</text>']
    for i, label in enumerate(sorted(frames)):
        x = ml + 15 + (i % 4) * 145; y = 48 + (i // 4) * 19
        parts += [f'<line x1="{x}" y1="{y-5}" x2="{x+24}" y2="{y-5}" stroke="{colors[label]}" stroke-width="3"/>',
                  f'<text x="{x+30}" y="{y}" font-family="sans-serif" font-size="14">{label}</text>']
    parts += [f'<text transform="translate(22 {mt+plot_h/2}) rotate(-90)" text-anchor="middle" font-family="sans-serif" font-size="15">{svg_escape(ylabel)}</text>',
              '<text x="640" y="695" text-anchor="middle" font-family="sans-serif" font-size="15">Normalized curve progress</text>', "</svg>"]
    path.write_text("\n".join(parts), encoding="utf-8")


def save_representative_svg(frames, metrics, path, field_name, title, ylabel, target_name=None):
    m = pd.DataFrame(metrics).set_index("label")
    bad = m[m.region.eq("lower-right")].cte_abs_p95_m.idxmax()
    matched_ring = int(m.loc[bad, "ring"])
    good = m[(m.region.eq("upper-left")) & (m.ring.eq(matched_ring))].index[0]
    labels = [bad, good]
    width, height = 1280, 590
    panel_w, top, bottom = 560, 70, 65
    all_y = []
    for label in labels:
        all_y.extend(pd.to_numeric(frames[label][field_name], errors="coerce").dropna().tolist())
        if target_name:
            all_y.extend(pd.to_numeric(frames[label][target_name], errors="coerce").dropna().tolist())
    ymin, ymax = min(all_y), max(all_y)
    pad = max((ymax-ymin)*0.08, 0.05); ymin -= pad; ymax += pad
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">', '<rect width="100%" height="100%" fill="white"/>',
             f'<text x="640" y="30" text-anchor="middle" font-family="sans-serif" font-size="21">{svg_escape(title)}</text>']
    for panel, label in enumerate(labels):
        left = 70 + panel * 620; h = height - top - bottom
        frame = frames[label]
        core = frame[frame.curve_progress.between(0, 1)].copy()
        x = core.curve_progress.to_numpy(float)
        def tx(v): return left + np.asarray(v) * panel_w
        def ty(v): return top + (ymax - np.asarray(v)) / (ymax - ymin) * h
        parts += [f'<rect x="{left}" y="{top}" width="{panel_w}" height="{h}" fill="none" stroke="#222"/>',
                  f'<text x="{left+panel_w/2}" y="{top+22}" text-anchor="middle" font-family="sans-serif" font-size="17">{label} ({m.loc[label,"region"]})</text>']
        for value in np.linspace(ymin, ymax, 5):
            parts.append(f'<line x1="{left}" y1="{ty(value):.1f}" x2="{left+panel_w}" y2="{ty(value):.1f}" stroke="#e5e5e5"/>')
            if panel == 0:
                parts.append(f'<text x="{left-8}" y="{ty(value)+4:.1f}" text-anchor="end" font-family="sans-serif" font-size="12">{value:.1f}</text>')
        y = pd.to_numeric(core[field_name], errors="coerce").to_numpy(float)
        valid = np.isfinite(x) & np.isfinite(y)
        parts.append(svg_polyline(list(zip(tx(x[valid]), ty(y[valid]))), "#d14b3e", 2))
        if target_name:
            yt = pd.to_numeric(core[target_name], errors="coerce").to_numpy(float)
            valid = np.isfinite(x) & np.isfinite(yt)
            parts.append(svg_polyline(list(zip(tx(x[valid]), ty(yt[valid]))), "#2f5aa8", 2))
        for v in (0, 0.5, 1):
            parts.append(f'<line x1="{tx(v):.1f}" y1="{top}" x2="{tx(v):.1f}" y2="{top+h}" stroke="#bbb" stroke-dasharray="4 4"/>')
        parts.append(f'<text x="{left+panel_w/2}" y="{height-22}" text-anchor="middle" font-family="sans-serif" font-size="14">Entry → apex → exit</text>')
    legend = "Red measured; blue target" if target_name else "Red actual; dashed verticals = entry/apex/exit"
    parts += [f'<text transform="translate(20 {top+(height-top-bottom)/2}) rotate(-90)" text-anchor="middle" font-family="sans-serif" font-size="15">{svg_escape(ylabel)}</text>',
              f'<text x="640" y="52" text-anchor="middle" font-family="sans-serif" font-size="14">{legend}; identical y-scale in both panels</text>', "</svg>"]
    path.write_text("\n".join(parts), encoding="utf-8")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_report(metrics, all_metrics, frames, all_frames, mission, turns, projection):
    table = pd.DataFrame(metrics)
    all_table = pd.DataFrame(all_metrics)
    lr = all_table[all_table.region.eq("lower-right")]
    ul = all_table[all_table.region.eq("upper-left")]
    combined = pd.concat(all_frames.values(), ignore_index=True)
    core = combined[combined.curve_progress.between(0, 1)]
    pair = core[["cross_track_err_m", "geom_cte_abs_m"]].dropna()
    overall_corr = pair.corr().iloc[0, 1]
    overall_rmse = float(np.sqrt(np.mean((pair.cross_track_err_m - pair.geom_cte_abs_m) ** 2)))

    def region_line(frame):
        return {
            "abs_mean": frame.cte_abs_mean_m.mean(), "p95": frame.cte_abs_p95_m.mean(),
            "max": frame.cte_abs_max_m.max(), "apex_inside": frame.apex_inside_cte_median_m.mean(),
            "exit_inside": frame.exit_recovery_inside_cte_median_m.mean(),
            "speed": frame.actual_speed_mean_mps.mean(), "steer": frame.steer_error_counts_abs_p95.mean(),
            "lag": frame.steering_lag_s.mean(), "heading": frame.path_heading_error_abs_p95_deg.mean(),
            "radius_mean": frame.approx_radius_m.mean(), "radius_median": frame.approx_radius_m.median(),
            "turn_angle": frame.net_heading_change_deg.abs().mean(),
            "logged": frame.logged_cte_mean_m.mean(), "logged_p95": frame.logged_cte_p95_m.mean(),
            "lookahead": frame.lookahead_median_m.mean(), "steer_cmd": frame.steer_normalized_median.mean(),
            "saturated": frame.steer_command_saturated_pct.mean(),
        }

    L, U = region_line(lr), region_line(ul)
    pivot = all_table.pivot(index="ring", columns="region")
    p95_count = int((pivot.cte_abs_p95_m["lower-right"] > pivot.cte_abs_p95_m["upper-left"]).sum())
    logged_count = int((pivot.logged_cte_mean_m["lower-right"] > pivot.logged_cte_mean_m["upper-left"]).sum())
    steer_count = int((pivot.steer_error_counts_abs_p95["lower-right"] > pivot.steer_error_counts_abs_p95["upper-left"]).sum())
    report = [
        "# September 23, 2026 main-backyard corner diagnosis", "", "## Corrected region selection", "",
        "This analysis uses the **southeast/lower-right corner of the nested `main_backyard_ring_*` paths**, not the recorded-manual stripes in the center. Each lower-right corner is paired with the northwest/upper-left corner from the same ring. Aggregate results cover complete rings 1–18; rings 4, 8, and 13 are labeled LR-1…3 and UL-1…3 as outer/middle/inner problem examples with healthy RTK/heading.", "",
        "## Bottom line", "",
        f"There is a deterministic difference. Across all 18 matched rings, lower-right mean absolute geometric CTE is **{L['abs_mean']:.3f} m** versus **{U['abs_mean']:.3f} m** upper-left. Mean per-ring P95 is **{L['p95']:.3f} versus {U['p95']:.3f} m**; lower-right is worse on {p95_count}/18 matched pairs. This is a real but modest centerline-tracking penalty of about {(L['p95']-U['p95'])*100:.1f} cm at P95.", "",
        f"The larger difference is controller/path geometry. Lower-right corners turn through **{L['turn_angle']:.1f}°** on average versus **{U['turn_angle']:.1f}°** upper-left, with median curvature radii **{L['radius_median']:.2f} versus {U['radius_median']:.2f} m**. Logged/dashboard `cross_track_err_m` averages **{L['logged']:.3f} versus {U['logged']:.3f} m** and is higher lower-right on {logged_count}/18 pairs.", "",
        f"Lower-right also demands roughly twice the right-steering command (median-command mean {L['steer_cmd']:+.3f} versus {U['steer_cmd']:+.3f}), with steering-error P95 **{L['steer']:.1f} versus {U['steer']:.1f} counts** and estimated lag **{L['lag']:.3f} versus {U['lag']:.3f} s**. Steering error is larger lower-right on {steer_count}/18 pairs. That makes **the sharper ~90° planned corner plus Pure Pursuit corner-cutting and higher steering demand** the strongest supported cause; mower-deck swept geometry can amplify the resulting inside gap.", "",
        "Positive signed CTE means tractor-left of travel. All selected and aggregate corners are right turns, so negative signed CTE—or positive direction-normalized `inside_cte`—means the tractor is inside the planned curve.", "",
        "## Selected matched turns", "",
        "|Turn|Ring|Region|Dir.|Turn °|Radius m|Entry/apex/exit WP|Actual m/s|Mean |CTE| m|P95 |CTE| m|Apex inside m|Exit inside m|Logged CTE mean m|Steer cmd|Steer error P95|Lag s|", "",
        "|---|---:|---|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in table.sort_values(["ring", "region"]).itertuples(index=False):
        report.append(f"|{row.label}|{row.ring}|{row.region}|{row.direction}|{abs(row.net_heading_change_deg):.1f}|{row.approx_radius_m:.2f}|{row.entry_waypoint}/{row.apex_waypoint}/{row.exit_waypoint}|{row.actual_speed_mean_mps:.2f}|{row.cte_abs_mean_m:.3f}|{row.cte_abs_p95_m:.3f}|{row.apex_inside_cte_median_m:+.3f}|{row.exit_recovery_inside_cte_median_m:+.3f}|{row.logged_cte_mean_m:.3f}|{row.steer_normalized_median:+.3f}|{row.steer_error_counts_abs_p95:.1f}|{row.steering_lag_s:.2f}|")
    report += [
        "", "## All-ring regional comparison", "", "|Metric|Lower-right|Upper-left|Difference|", "", "|---|---:|---:|---:|",
        f"|Mean absolute geometric CTE|{L['abs_mean']:.3f} m|{U['abs_mean']:.3f} m|{L['abs_mean']-U['abs_mean']:+.3f} m|",
        f"|Mean per-ring P95 geometric CTE|{L['p95']:.3f} m|{U['p95']:.3f} m|{L['p95']-U['p95']:+.3f} m|",
        f"|Mean planned turn angle|{L['turn_angle']:.1f}°|{U['turn_angle']:.1f}°|{L['turn_angle']-U['turn_angle']:+.1f}°|",
        f"|Median curvature radius|{L['radius_median']:.3f} m|{U['radius_median']:.3f} m|{L['radius_median']-U['radius_median']:+.3f} m|",
        f"|Logged/dashboard CTE mean|{L['logged']:.3f} m|{U['logged']:.3f} m|{L['logged']-U['logged']:+.3f} m|",
        f"|Logged/dashboard CTE P95|{L['logged_p95']:.3f} m|{U['logged_p95']:.3f} m|{L['logged_p95']-U['logged_p95']:+.3f} m|",
        f"|Inside CTE at apex|{L['apex_inside']:+.3f} m|{U['apex_inside']:+.3f} m|{L['apex_inside']-U['apex_inside']:+.3f} m|",
        f"|Actual speed|{L['speed']:.3f} m/s|{U['speed']:.3f} m/s|{L['speed']-U['speed']:+.3f} m/s|",
        f"|Median normalized steering command|{L['steer_cmd']:+.3f}|{U['steer_cmd']:+.3f}|{L['steer_cmd']-U['steer_cmd']:+.3f}|",
        f"|Steering error absolute P95|{L['steer']:.1f} counts|{U['steer']:.1f} counts|{L['steer']-U['steer']:+.1f} counts|",
        f"|Estimated steering lag|{L['lag']:.3f} s|{U['lag']:.3f} s|{L['lag']-U['lag']:+.3f} s|",
        f"|Path-heading error absolute P95|{L['heading']:.1f}°|{U['heading']:.1f}°|{L['heading']-U['heading']:+.1f}°|",
        f"|Median lookahead|{L['lookahead']:.3f} m|{U['lookahead']:.3f} m|{L['lookahead']-U['lookahead']:+.3f} m|",
        "", "## Selected locations and telemetry", "",
        "|Turn|Entry lat, lon|Apex lat, lon|Exit lat, lon|Approach °|Target / measured counts|Heading median / abs P95 °|Accuracy P95 °|RTK / valid / carrier fixed %|", "",
        "|---|---|---|---|---:|---:|---:|---:|---:|",
    ]
    for row in table.sort_values(["ring", "region"]).itertuples(index=False):
        report.append(f"|{row.label}|{row.entry_lat:.7f}, {row.entry_lon:.7f}|{row.apex_lat:.7f}, {row.apex_lon:.7f}|{row.exit_lat:.7f}, {row.exit_lon:.7f}|{row.approach_heading_compass_deg:.1f}|{row.steer_target_counts_median:.0f} / {row.steer_measured_counts_median:.0f}|{row.path_heading_error_median_deg:+.1f} / {row.path_heading_error_abs_p95_deg:.1f}|{row.heading_accuracy_p95_deg:.2f}|{row.rtk_fixed_pct:.0f} / {row.heading_valid_pct:.0f} / {row.carrier_fixed_pct:.0f}|")
    report += [
        "", "## Existing logged CTE check", "",
        f"Across all 36 matched corner cores, logged `cross_track_err_m` versus independently computed geometric |CTE| has correlation **{overall_corr:.3f}** and RMSE **{overall_rmse:.3f} m**. The log dictionary defines it as `abs(yt_m)`: lateral offset to the lookahead target in the tractor frame. It is useful for Pure Pursuit demand, but it is not nearest-path geometric CTE.",
        "", "## Answers to A–F", "",
        f"**A. Deterministic difference?** Yes. Lower-right geometric CTE P95 is {L['p95']:.3f} versus {U['p95']:.3f} m, and the controller-demand metric is higher on {logged_count}/18 matched rings.", "",
        f"**B. Strongest measured difference?** Planned sweep angle ({L['turn_angle']:.1f}° versus {U['turn_angle']:.1f}°), followed by logged lookahead-target offset ({L['logged']:.3f} versus {U['logged']:.3f} m) and steering demand/error.", "",
        "**C. Likely cause?** Primarily path geometry/Pure Pursuit corner-cutting under a much stronger right-turn demand, with steering response as a secondary contributor. Speed differs by only a few hundredths of a metre per second on average. RTK position stayed fixed and heading validity was 100%; isolated carrier/accuracy degradations do not repeat across the lower-right set. Terrain is not recoverable from these logs. Deck footprint/overlap likely converts the modest centerline error into visible uncut wedges.", "",
        f"**D. Where?** The repeatable bias is strongest around the apex: inside displacement averages {L['apex_inside']:+.3f} m lower-right versus {U['apex_inside']:+.3f} m upper-left. Both groups remain somewhat inside during exit/recovery.", "",
        "**E. CTE sign?** Yes. All matched corners are right turns, and the independently computed signed CTE is consistently negative/inside near the lower-right apexes.", "",
        f"**F. Dashboard CTE trustworthy?** Trustworthy as a lookahead-target/Pure Pursuit demand value, not as geometric path error. Correlation with independent |CTE| is {overall_corr:.3f}; RMSE is {overall_rmse:.3f} m.",
        "", "## Mowing geometry", "",
        "The mission was originally documented as a supervised blades-off tracking-validation build, not a deck-footprint coverage plan. The data supports a real lower-right tracking penalty, but it is only about 2–3 cm at P95. The visibly uncut grass is therefore most plausibly the combination of that inside bias with the tighter/full-90° corner and the physical deck trajectory. Exact swept coverage still requires antenna-to-rear-axle/deck offsets and effective cutting width.",
        "", "## Plots", "", "- [Plan view of the nested rings and selected matched corners](turn_plan_view.svg)",
        "- [Independent signed CTE through normalized turn progress](cte_normalized.svg)",
        "- [Steering target versus measured](steering_representative.svg)", "- [Speed through representative curves](speed_representative.svg)",
        "", "## Reproducibility", "", "Run from the repository root:", "",
        "```powershell", "& 'C:\\Users\\al532\\.cache\\codex-runtimes\\codex-primary-runtime\\dependencies\\python\\python.exe' field_testing/tools/analyze_20260923_turns.py", "```", "", "Exact source and provenance files:", "",
    ]
    for source in (PURSUIT, FIELD, MISSION, AUDIT, REPLAY, REPLAY_BUILDER, MISSION_NOTES):
        report.append(f"- `{source.relative_to(REPO)}` — SHA-256 `{sha256(source)}`")
    report += [
        "", "Key calculations:", "",
        "- Local east/north projection matches the replay builder: `x=(lon-lon0)*111320*cos(lat0)`, `y=(lat-lat0)*110540`.",
        "- Geometric CTE projects each tractor position to the closest segment in a waypoint-hinted local path window; sign is the 2-D cross product of path tangent with tractor-minus-projection.",
        "- Each ring is normalized independently; southeast is the maximum `(zEast-zNorth)` corner and northwest is the maximum `(-zEast+zNorth)` corner.",
        "- Metrics use the sustained-curvature core (|curvature| ≥ 0.15 1/m, short gaps bridged). Apex is the sample closest to half the net heading rotation.",
        "- Approximate radius is the reciprocal of median absolute planned curvature within the core. Exit/recovery uses normalized progress 0.75–1.15.",
        "", "## Next-step candidates (not implementation instructions)", "",
        "1. Measure the GNSS-reference-to-deck geometry and effective cutting width, then generate a swept-deck overlay for the southeast corners.",
        "2. Compare a larger-radius or two-stage southeast corner in replay, preserving the same ring spacing, before changing controller gains.",
        "3. If geometry alone is insufficient, evaluate curvature-aware lookahead/speed or right-turn steering feed-forward against these same 18 matched pairs.", "",
    ]
    (OUTPUT / "REPORT.md").write_text("\n".join(report), encoding="utf-8")


def print_summary(metrics):
    table = pd.DataFrame(metrics)
    print(table[["label", "region", "direction", "approx_radius_m", "actual_speed_mean_mps",
                 "cte_signed_median_m", "cte_abs_p95_m", "inside_cte_median_m",
                 "apex_cte_signed_median_m", "exit_recovery_cte_signed_median_m",
                 "steer_error_counts_abs_p95", "steering_lag_s", "rtk_fixed_pct"]].to_string(index=False))


def main():
    mission, pursuit, field, projection = read_inputs()
    mission = planned_geometry(mission)
    turns, all_turns = detect_ring_turns(mission)
    tracking = add_geometric_tracking(pursuit, mission)
    tracking = join_field(tracking, field)
    metrics = []
    frames = {}
    for turn in turns:
        values, frame = turn_metrics(turn, mission, tracking)
        metrics.append(values)
        frames[turn.label] = frame
    all_metrics = []
    all_frames = {}
    for turn in all_turns:
        values, frame = turn_metrics(turn, mission, tracking)
        all_metrics.append(values)
        all_frames[f"R{turn.ring:02d}-{turn.region}"] = frame
    OUTPUT.mkdir(parents=True, exist_ok=True)
    print(f"mission={len(mission)} pursuit={len(pursuit)} field={len(field)}")
    print("selected matched main-backyard rings: 4, 8, 13; aggregate rings: 1-18")
    for turn in turns:
        p = mission.iloc[turn.apex]
        net = math.degrees(mission.path_heading_rad.iloc[turn.end] - mission.path_heading_rad.iloc[turn.start])
        print(f"{turn.label:4s} {turn.direction:5s} wp {turn.start+1:5d}-{turn.apex+1:5d}-{turn.end+1:5d} "
              f"apex=({p.x:7.2f},{p.y:7.2f}) net={net:7.1f} deg")
    print_summary(metrics)
    metrics_frame = pd.DataFrame(metrics)
    metrics_frame.to_csv(OUTPUT / "turn_metrics.csv", index=False, float_format="%.9f")
    pd.DataFrame(all_metrics).to_csv(OUTPUT / "all_ring_turn_metrics.csv", index=False, float_format="%.9f")
    pd.concat(all_frames.values(), ignore_index=True).to_csv(
        OUTPUT / "all_ring_turn_samples.csv", index=False, float_format="%.9f"
    )
    pd.concat(frames.values(), ignore_index=True).to_csv(
        OUTPUT / "turn_samples.csv", index=False, float_format="%.9f"
    )
    save_plan_svg(mission, tracking, turns, OUTPUT / "turn_plan_view.svg")
    save_line_panels_svg(frames, metrics, OUTPUT / "cte_normalized.svg", "cte")
    save_representative_svg(frames, metrics, OUTPUT / "steering_representative.svg",
                            "field_steer_current", "Steering target vs measured — representative curves",
                            "Steering potentiometer counts", "field_steer_setpoint")
    save_representative_svg(frames, metrics, OUTPUT / "speed_representative.svg",
                            "actual_speed_mps", "Speed through representative curves",
                            "Speed (m/s)", "speed_cmd_mps")
    write_report(metrics, all_metrics, frames, all_frames, mission, turns, projection)
    (OUTPUT / "detected_turns.json").write_text(json.dumps([
        {**turn.__dict__, "entry_waypoint": turn.start + 1, "apex_waypoint": turn.apex + 1,
         "exit_waypoint": turn.end + 1} for turn in all_turns
    ], indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
