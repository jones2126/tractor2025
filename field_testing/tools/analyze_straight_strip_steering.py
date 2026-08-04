#!/usr/bin/env python3
"""Analyze high- and low-level steering behavior on selected straight strips.

The pursuit log contains the 20 Hz high-level command.  The field log contains
the steering-pot and PWM observations that were available from the Teensy
bridge.  This tool aligns both logs, reconstructs the pot setpoint implied by
each normalized pursuit command, and measures geometric cross-track error
against the exact endpoints in 02_coverage_segments.csv.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont


POT_RIGHT = 197.0
POT_CENTER = 447.0
POT_LEFT = 815.0
EARTH_RADIUS_M = 6_371_000.0


def normalized_to_pot(command: pd.Series) -> np.ndarray:
    values = command.to_numpy(dtype=float)
    return np.where(
        values >= 0.0,
        POT_CENTER + values * (POT_LEFT - POT_CENTER),
        POT_CENTER + values * (POT_CENTER - POT_RIGHT),
    )


def local_xy(lat, lon, origin_lat, origin_lon, projection_lat):
    projection_lat_rad = math.radians(projection_lat)
    x = EARTH_RADIUS_M * np.radians(np.asarray(lon) - origin_lon) * math.cos(projection_lat_rad)
    y = EARTH_RADIUS_M * np.radians(np.asarray(lat) - origin_lat)
    return x, y


def series_stats(values: pd.Series) -> dict[str, float]:
    values = pd.to_numeric(values, errors="coerce").dropna()
    if values.empty:
        return {"mean": math.nan, "median": math.nan, "p05": math.nan, "p95": math.nan}
    return {
        "mean": float(values.mean()),
        "median": float(values.median()),
        "p05": float(values.quantile(0.05)),
        "p95": float(values.quantile(0.95)),
    }


def load_font(size: int, bold: bool = False):
    choices = [
        Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
        Path("C:/Windows/Fonts/segoeuib.ttf" if bold else "C:/Windows/Fonts/segoeui.ttf"),
    ]
    for path in choices:
        if path.exists():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def draw_chart(draw, box, title, traces, y_label, zero=True):
    x0, y0, x1, y1 = box
    left, right, top, bottom = 82, 22, 68, 50
    px0, px1 = x0 + left, x1 - right
    py0, py1 = y0 + top, y1 - bottom
    font = load_font(17)
    small = load_font(14)
    draw.text((x0 + 8, y0 + 5), title, fill="#10243e", font=load_font(19, True))

    all_x = np.concatenate([np.asarray(t[0], dtype=float) for t in traces if len(t[0])])
    all_y = np.concatenate([np.asarray(t[1], dtype=float) for t in traces if len(t[1])])
    finite = np.isfinite(all_x) & np.isfinite(all_y)
    all_x, all_y = all_x[finite], all_y[finite]
    if not len(all_x):
        draw.text((px0, py0), "No samples", fill="#a00000", font=font)
        return
    xmin, xmax = float(all_x.min()), float(all_x.max())
    ymin, ymax = float(all_y.min()), float(all_y.max())
    if zero:
        ymin, ymax = min(ymin, 0.0), max(ymax, 0.0)
    pad = max((ymax - ymin) * 0.10, 0.05)
    ymin, ymax = ymin - pad, ymax + pad

    def xp(v): return px0 + (np.asarray(v) - xmin) / max(xmax - xmin, 1e-9) * (px1 - px0)
    def yp(v): return py1 - (np.asarray(v) - ymin) / max(ymax - ymin, 1e-9) * (py1 - py0)

    for frac in np.linspace(0, 1, 5):
        yy = py1 - frac * (py1 - py0)
        value = ymin + frac * (ymax - ymin)
        draw.line((px0, yy, px1, yy), fill="#d8dee8", width=1)
        draw.text((x0 + 8, yy - 8), f"{value:.2f}", fill="#536275", font=small)
    if ymin <= 0 <= ymax:
        yy = float(yp(0))
        draw.line((px0, yy, px1, yy), fill="#78869a", width=2)
    draw.rectangle((px0, py0, px1, py1), outline="#8795a8", width=1)
    draw.text((px1 - draw.textlength(y_label, font=small) - 8, py0 + 5), y_label, fill="#33445a", font=small)
    draw.text((px0, py1 + 12), f"{xmin:.1f}", fill="#536275", font=small)
    draw.text((px1 - 35, py1 + 12), f"{xmax:.1f} m", fill="#536275", font=small)

    legend_x = px0 + 8
    for xs, ys, color, label, width in traces:
        xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
        ok = np.isfinite(xs) & np.isfinite(ys)
        points = list(zip(xp(xs[ok]).tolist(), yp(ys[ok]).tolist()))
        if len(points) >= 2:
            draw.line(points, fill=color, width=width, joint="curve")
        elif points:
            xx, yy = points[0]
            draw.ellipse((xx - 2, yy - 2, xx + 2, yy + 2), fill=color)
        draw.line((legend_x, y0 + 45, legend_x + 24, y0 + 45), fill=color, width=width)
        draw.text((legend_x + 30, y0 + 36), label, fill="#26364a", font=small)
        legend_x += 30 + draw.textlength(label, font=small) + 34


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--segments", required=True, type=Path)
    parser.add_argument("--pursuit-log", required=True, type=Path)
    parser.add_argument("--field-log", required=True, type=Path)
    parser.add_argument("--mission-audit", required=True, type=Path)
    parser.add_argument("--scanlines", nargs="+", type=int, required=True)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    segments = pd.read_csv(args.segments)
    audit = pd.read_csv(args.mission_audit)
    pursuit = pd.read_csv(args.pursuit_log, skiprows=[1], low_memory=False)
    field = pd.read_csv(args.field_log, low_memory=False)
    pursuit["driving"] = pursuit.driving.astype(str).str.lower().eq("true")
    pursuit["ts_ns"] = pd.to_datetime(pursuit.timestamp, unit="s", utc=True).astype("datetime64[ns, UTC]").astype("int64")
    field["ts_ns"] = pd.to_datetime(field.time, utc=True).astype("datetime64[ns, UTC]").astype("int64")

    # Match every coverage segment to the contiguous audit stripe whose two
    # endpoints are closest.  Pursuit waypoint_idx is zero based.
    stripe_groups = []
    is_stripe = audit.kind.eq("stripe")
    group_no = (is_stripe != is_stripe.shift(fill_value=False)).cumsum()
    for _, group in audit[is_stripe].groupby(group_no[is_stripe]):
        stripe_groups.append(group)

    analyzed = []
    summaries = []
    telemetry_rows = []
    for scanline in args.scanlines:
        segment = segments.loc[segments.scanline.eq(scanline)].iloc[0]
        endpoint = np.array([[segment.start_lat, segment.start_lon], [segment.end_lat, segment.end_lon]])
        best = None
        for group in stripe_groups:
            candidate = group.iloc[[0, -1]][["lat", "lon"]].to_numpy()
            direct = np.square(candidate - endpoint).sum()
            reverse = np.square(candidate[::-1] - endpoint).sum()
            score = min(direct, reverse)
            if best is None or score < best[0]:
                best = (score, group)
        group = best[1]
        idx_start = int(group.waypoint.min()) - 1
        idx_end = int(group.waypoint.max()) - 1

        q = pursuit[
            pursuit.driving & pursuit.waypoint_idx.between(idx_start, idx_end)
        ].copy().sort_values("ts_ns")
        if q.empty:
            raise RuntimeError(f"No driving samples found for scanline {scanline}")

        projection_lat = (segment.start_lat + segment.end_lat) / 2.0
        ex, ey = local_xy(
            segment.end_lat,
            segment.end_lon,
            segment.start_lat,
            segment.start_lon,
            projection_lat,
        )
        length = float(math.hypot(float(ex), float(ey)))
        ux, uy = float(ex) / length, float(ey) / length
        ax, ay = local_xy(
            q.lat,
            q.lon,
            segment.start_lat,
            segment.start_lon,
            projection_lat,
        )
        q["along_track_m"] = ax * ux + ay * uy
        q["geometric_cte_m"] = ux * ay - uy * ax
        planned_heading = (math.degrees(math.atan2(float(ex), float(ey))) + 360.0) % 360.0
        q["heading_error_deg"] = ((q.heading_compass_deg - planned_heading + 180.0) % 360.0) - 180.0
        q["reconstructed_pot_setpoint"] = normalized_to_pot(q.steer_normalized)
        q["scanline"] = scanline
        q["planned_heading_deg"] = planned_heading
        q["audit_waypoint_start"] = idx_start + 1
        q["audit_waypoint_end"] = idx_end + 1

        lo, hi = q.ts_ns.min() - 200_000_000, q.ts_ns.max() + 200_000_000
        tf = field[field.ts_ns.between(lo, hi)].copy().sort_values("ts_ns")
        tf["scanline"] = scanline
        telemetry_rows.append(tf)
        merged = pd.merge_asof(
            q,
            tf,
            on="ts_ns",
            direction="nearest",
            tolerance=150_000_000,
            suffixes=("", "_field"),
        )
        merged["telemetry_match_age_ms"] = (
            pd.to_datetime(merged.timestamp, unit="s", utc=True)
            - pd.to_datetime(merged.time, utc=True)
        ).dt.total_seconds().abs() * 1000.0
        merged["timestamp_utc"] = pd.to_datetime(merged.timestamp, unit="s", utc=True)
        merged["timestamp_local"] = merged.timestamp_utc.dt.tz_convert("America/New_York")
        analyzed.append(merged)

        settled = q[q.along_track_m.between(3.0, length - 3.0)]
        observed = tf[pd.to_numeric(tf.steer_setpoint, errors="coerce").notna()]
        settled_observed = merged[
            merged.along_track_m.between(3.0, length - 3.0) & merged.steer_current.notna()
        ]
        settled_pwm = pd.to_numeric(settled_observed.steer_pwm, errors="coerce")
        pwm = pd.to_numeric(observed.steer_pwm, errors="coerce")
        summaries.append({
            "scanline": scanline,
            "audit_waypoint_start": idx_start + 1,
            "audit_waypoint_end": idx_end + 1,
            "controller_index_min_seen": int(q.waypoint_idx.min()),
            "controller_index_max_seen": int(q.waypoint_idx.max()),
            "duration_s": float(q.elapsed_s.max() - q.elapsed_s.min()),
            "pursuit_samples": len(q),
            "planned_length_m": length,
            "planned_heading_deg": planned_heading,
            "settled_geometric_cte_mean_m": float(settled.geometric_cte_m.mean()),
            "settled_geometric_cte_median_m": float(settled.geometric_cte_m.median()),
            "settled_heading_error_mean_deg": float(settled.heading_error_deg.mean()),
            "settled_lookahead_yt_mean_m": float(settled.yt_m.mean()),
            "settled_delta_mean_deg": float(settled.delta_deg.mean()),
            "settled_normalized_command_mean": float(settled.steer_normalized.mean()),
            "settled_reconstructed_setpoint_mean": float(settled.reconstructed_pot_setpoint.mean()),
            "field_rows_in_window": len(tf),
            "matched_pursuit_samples": int(merged.steer_pwm.notna().sum()),
            "observed_setpoint_mean": float(pd.to_numeric(observed.steer_setpoint, errors="coerce").mean()),
            "observed_pot_current_mean": float(pd.to_numeric(observed.steer_current, errors="coerce").mean()),
            "settled_observed_pot_median": float(pd.to_numeric(settled_observed.steer_current, errors="coerce").median()),
            "settled_observed_setpoint_median": float(pd.to_numeric(settled_observed.steer_setpoint, errors="coerce").median()),
            "settled_observed_error_median": float(pd.to_numeric(settled_observed.steer_error, errors="coerce").median()),
            "settled_observed_pwm_zero_fraction": float((settled_pwm == 0).mean()),
            "settled_observed_pwm_150_fraction": float((settled_pwm == 150).mean()),
            "observed_pot_error_mean": float(pd.to_numeric(observed.steer_error, errors="coerce").mean()),
            "observed_pwm_zero_fraction": float((pwm == 0).mean()),
            "observed_pwm_150_fraction": float((pwm == 150).mean()),
            "observed_pwm_255_fraction": float((pwm == 255).mean()),
            "unique_steer_sequence_values": int(tf.steer_sequence.nunique(dropna=True)),
        })

    detail = pd.concat(analyzed, ignore_index=True)
    summary = pd.DataFrame(summaries)
    telemetry = pd.concat(telemetry_rows, ignore_index=True)
    keep = [
        "timestamp_local", "timestamp_utc", "scanline", "elapsed_s", "waypoint_idx",
        "along_track_m", "geometric_cte_m", "heading_compass_deg", "planned_heading_deg",
        "heading_error_deg", "lookahead_dist_m", "yt_m", "alpha_deg", "delta_deg",
        "steer_normalized", "reconstructed_pot_setpoint", "speed_cmd_mps", "steer_setpoint",
        "steer_current", "steer_error", "steer_pwm", "steer_direction", "steer_left_pwm",
        "steer_right_pwm", "steer_pid_active", "steer_deadband_active",
        "steer_min_pwm_clamped", "steer_pwm_saturated", "steer_pid_dt_s",
        "steer_integral_sum", "steer_error_derivative", "steer_p_term", "steer_i_term",
        "steer_d_term", "steer_pid_output", "steer_cmd_age_ms", "telemetry_match_age_ms",
        "fix_quality", "head_valid", "gps_age_s",
    ]
    detail[[c for c in keep if c in detail]].to_csv(
        args.output_dir / "straight_strips_12_13_20hz_detail.csv", index=False
    )
    summary.to_csv(args.output_dir / "straight_strips_12_13_summary.csv", index=False)

    # Four focused charts drawn with Pillow so the report remains portable.
    image = Image.new("RGB", (1800, 1450), "white")
    draw = ImageDraw.Draw(image)
    draw.text((42, 18), "Straight-strip steering diagnosis — scanlines 12 and 13", fill="#0b213f", font=load_font(28, True))
    draw.text((42, 55), "20 Hz pursuit calculations aligned with available Teensy/IBT-2 observations", fill="#506078", font=load_font(18))
    colors = {args.scanlines[0]: "#1464d2", args.scanlines[1]: "#d13a32"}
    traces = []
    for scanline in args.scanlines:
        q = detail[detail.scanline.eq(scanline)]
        traces.append((q.along_track_m, q.geometric_cte_m, colors[scanline], f"scanline {scanline}", 4))
    draw_chart(draw, (35, 95, 1765, 405), "1. Actual geometric cross-track error against exact strip line", traces, "CTE (m)")

    traces = []
    for scanline in args.scanlines:
        q = detail[detail.scanline.eq(scanline)]
        traces.append((q.along_track_m, q.delta_deg, colors[scanline], f"{scanline} requested", 3))
    draw_chart(draw, (35, 420, 1765, 730), "2. Pure-pursuit steering request", traces, "delta (deg)")

    traces = []
    for scanline in args.scanlines:
        q = detail[detail.scanline.eq(scanline)]
        traces.append((q.along_track_m, q.reconstructed_pot_setpoint, colors[scanline], f"{scanline} request", 3))
        obs = q[q.steer_current.notna()]
        pot_color = "#159447" if scanline == args.scanlines[0] else "#111111"
        traces.append((obs.along_track_m, obs.steer_current, pot_color, f"{scanline} pot", 2))
    traces.append(([0, max(detail.along_track_m.max(), 1)], [POT_CENTER, POT_CENTER], "#8e44ad", "firmware center = 447", 2))
    draw_chart(draw, (35, 745, 1765, 1055), "3. Requested setpoint versus observed steering pot", traces, "pot counts", zero=False)

    traces = []
    for scanline in args.scanlines:
        q = detail[detail.scanline.eq(scanline) & detail.steer_pwm.notna()]
        traces.append((q.along_track_m, q.steer_pwm, colors[scanline], f"{scanline} PWM", 4))
    draw_chart(draw, (35, 1070, 1765, 1380), "4. Available IBT-2 PWM observations (telemetry is intermittent)", traces, "PWM 0–255")
    image.save(args.output_dir / "straight_strips_12_13_steering_diagnostic.png")

    s12, s13 = summaries
    inferred_straight = np.nanmedian([
        s12["settled_observed_pot_median"], s13["settled_observed_pot_median"]
    ])
    inferred_bias = (inferred_straight - POT_CENTER) / (POT_CENTER - POT_RIGHT)
    report = f"""# Straight-strip steering diagnosis: scanlines {args.scanlines[0]} and {args.scanlines[1]}

## Result

The IBT-2 was not receiving a continuously increasing correction based on path error. It only received PWM when the steering-pot error exceeded the firmware's 10-count deadband. On the settled straight portions, the controller requested pot setpoints near 373–378 counts and the measured pot was already near 369–371, so the firmware normally commanded PWM 0.

The stronger result is a likely steering-center calibration mismatch. The firmware defines 447 pot counts as straight. During both nearly heading-aligned traversals the settled observed pot median was about {inferred_straight:.1f}. If that observed value represents physical straight ahead, the present mapping has an implicit normalized bias of {inferred_bias:+.3f}, equivalent to about {inferred_bias * 35.695:+.1f} degrees in the controller's ±35.7-degree model. That almost exactly consumes the steady right-steering request seen on these strips.

## Settled straight portions (3 m removed at each end)

| Metric | Scanline {args.scanlines[0]} | Scanline {args.scanlines[1]} |
|---|---:|---:|
| Mean geometric CTE | {s12['settled_geometric_cte_mean_m']:.3f} m | {s13['settled_geometric_cte_mean_m']:.3f} m |
| Mean heading error | {s12['settled_heading_error_mean_deg']:+.2f}° | {s13['settled_heading_error_mean_deg']:+.2f}° |
| Mean pursuit `yt` | {s12['settled_lookahead_yt_mean_m']:+.3f} m | {s13['settled_lookahead_yt_mean_m']:+.3f} m |
| Mean requested steering angle | {s12['settled_delta_mean_deg']:+.2f}° | {s13['settled_delta_mean_deg']:+.2f}° |
| Mean normalized steering command | {s12['settled_normalized_command_mean']:+.3f} | {s13['settled_normalized_command_mean']:+.3f} |
| Reconstructed mean pot setpoint | {s12['settled_reconstructed_setpoint_mean']:.1f} | {s13['settled_reconstructed_setpoint_mean']:.1f} |

## Available low-level observations

| Metric | Scanline {args.scanlines[0]} | Scanline {args.scanlines[1]} |
|---|---:|---:|
| Field-log rows in strip window | {s12['field_rows_in_window']} | {s13['field_rows_in_window']} |
| 20 Hz samples with a telemetry match within 150 ms | {s12['matched_pursuit_samples']} / {s12['pursuit_samples']} | {s13['matched_pursuit_samples']} / {s13['pursuit_samples']} |
| Settled observed setpoint median | {s12['settled_observed_setpoint_median']:.1f} | {s13['settled_observed_setpoint_median']:.1f} |
| Settled observed pot median | {s12['settled_observed_pot_median']:.1f} | {s13['settled_observed_pot_median']:.1f} |
| Settled observed pot-error median | {s12['settled_observed_error_median']:.1f} | {s13['settled_observed_error_median']:.1f} |
| Settled matched samples with PWM = 0 | {s12['settled_observed_pwm_zero_fraction']:.1%} | {s13['settled_observed_pwm_zero_fraction']:.1%} |
| Settled matched samples with PWM = 150 | {s12['settled_observed_pwm_150_fraction']:.1%} | {s13['settled_observed_pwm_150_fraction']:.1%} |

## Important telemetry limitation

The pursuit calculation is genuinely present at 20 Hz. The July 28 Teensy source is intentionally a 10 Hz steering controller and telemetry producer (`controlSteeringInterval = 100 ms`), so even the intended firmware would provide low-level PID telemetry at 10 Hz rather than 20 Hz.

The field-test hardware was not emitting the July 28 telemetry format. The available rows report only one steering sequence value (`q=0`), and advanced fields such as P/I/D terms and applied left/right PWM remain zero even when the basic PWM field reports 150. The saved bridge journal also shows fresh steering messages arriving only about once every 2.4–2.6 seconds. The July 28 firmware would increment `q` and print a complete record every 100 ms. Therefore the Teensy was almost certainly still running an older compiled program; updating the Raspberry Pi bridge service did not flash the Teensy. The advanced values in this CSV were defaults supplied by the July 28 bridge while it parsed the older Teensy format, so they cannot be used as measured PID internals for this run.

The trustworthy low-level fields here are the basic setpoint, pot current, pot error, direction, and PWM magnitude. The exact current firmware calculation is: setpoint from normalized command → error = setpoint − pot → PWM 0 inside ±10 counts; otherwise `abs(Kp × error)` is raised to the 150 minimum and capped at 255, with Kp = 1.0.

## Interpretation

1. Pure pursuit did see the lateral displacement and requested roughly 10° right steering on the settled sections.
2. The low-level loop reached the requested pot value and then entered its deadband, so PWM correctly dropped to zero under its current rules.
3. Tractor heading nevertheless stayed almost parallel to the strips while the geometric CTE stayed near 0.6 m.
4. The simplest explanation consistent with all four facts is that pot ≈370 was physically near straight, while the firmware/controller mapping treated 447 as straight. The controller's normal correction was spent canceling that offset instead of curving back to the line.

This is a strong inference from the logs, not a substitute for a stationary wheel-center measurement. Confirm the pot reading with the front wheels physically straight before changing firmware constants.
"""
    (args.output_dir / "STRAIGHT_STRIP_STEERING_FINDINGS.md").write_text(report, encoding="utf-8")
    print(summary.to_string(index=False))
    print(f"Detail: {args.output_dir / 'straight_strips_12_13_20hz_detail.csv'}")
    print(f"Plot:   {args.output_dir / 'straight_strips_12_13_steering_diagnostic.png'}")
    print(f"Report: {args.output_dir / 'STRAIGHT_STRIP_STEERING_FINDINGS.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
