#!/usr/bin/env python3
"""Map and quantify a pure-pursuit field run.

The log's ``driving`` flag is used as an AUTO *proxy*.  The 2026-07-22 log
format does not record the Teensy's radio mode, so it cannot prove that the
physical switch was in AUTO.  Add a radio_mode column to future logs if that
distinction is required.

Usage:
  python analyze_pure_pursuit.py pursuit_log.csv mission.txt
  python analyze_pure_pursuit.py pursuit_log.csv mission.txt --out results

Mission rows are: latitude longitude yaw_radians lookahead_m speed_mps
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


EARTH_R = 6371000.0


def load_log(path: Path) -> pd.DataFrame:
    # Row 2 is the controller's human-readable units/description row.
    df = pd.read_csv(path, skiprows=[1])
    required = {"elapsed_s", "lat", "lon", "driving"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"log is missing columns: {', '.join(sorted(missing))}")
    for col in ("elapsed_s", "lat", "lon", "heading_compass_deg",
                "steer_normalized", "speed_cmd_mps", "cross_track_err_m"):
        if col in df:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    df["driving"] = df["driving"].astype(str).str.lower().eq("true")
    return df.dropna(subset=["lat", "lon"]).reset_index(drop=True)


def load_mission(path: Path) -> pd.DataFrame:
    rows = []
    for line_no, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"{path}:{line_no}: expected 5 values, got {len(fields)}")
        rows.append(tuple(map(float, fields)))
    if len(rows) < 2:
        raise ValueError("mission needs at least two valid waypoints")
    return pd.DataFrame(rows, columns=["lat", "lon", "yaw_rad",
                                       "lookahead_m", "speed_mps"])


def local_xy(lat, lon, lat0, lon0):
    """Equirectangular local coordinates: x east, y north, in metres."""
    lat = np.asarray(lat, dtype=float)
    lon = np.asarray(lon, dtype=float)
    x = EARTH_R * np.deg2rad(lon - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * np.deg2rad(lat - lat0)
    return x, y


def geometric_cte(log: pd.DataFrame, mission: pd.DataFrame) -> pd.DataFrame:
    """Signed distance to the nearest mission polyline segment.

    Positive means the tractor lies left of the mission direction; negative
    means right.  This is distinct from the controller's lookahead-target yt.
    """
    lat0, lon0 = mission.iloc[0][["lat", "lon"]]
    mx, my = local_xy(mission.lat, mission.lon, lat0, lon0)
    px, py = local_xy(log.lat, log.lon, lat0, lon0)
    a = np.column_stack((mx[:-1], my[:-1]))
    v = np.column_stack((np.diff(mx), np.diff(my)))
    vv = np.einsum("ij,ij->i", v, v)

    signed, nearest_seg, progress = [], [], []
    seg_lengths = np.sqrt(vv)
    cumulative = np.r_[0.0, np.cumsum(seg_lengths)]
    for point in np.column_stack((px, py)):
        w = point - a
        t = np.clip(np.einsum("ij,ij->i", w, v) / vv, 0.0, 1.0)
        q = a + t[:, None] * v
        d2 = np.einsum("ij,ij->i", point - q, point - q)
        i = int(np.argmin(d2))
        cross = v[i, 0] * (point[1] - q[i, 1]) - v[i, 1] * (point[0] - q[i, 0])
        sign = 1.0 if cross >= 0 else -1.0
        signed.append(sign * math.sqrt(float(d2[i])))
        nearest_seg.append(i)
        progress.append(cumulative[i] + t[i] * seg_lengths[i])

    out = log.copy()
    out["mission_cte_signed_m"] = signed
    out["mission_cte_abs_m"] = np.abs(signed)
    out["nearest_segment"] = nearest_seg
    out["mission_progress_m"] = progress
    return out


def contiguous_runs(mask: pd.Series) -> pd.Series:
    transitions = mask.ne(mask.shift(fill_value=False)).cumsum()
    ids = pd.Series(pd.NA, index=mask.index, dtype="Int64")
    ids.loc[mask] = transitions.loc[mask]
    dense = {old: new for new, old in enumerate(ids.dropna().unique(), 1)}
    return ids.map(dense).astype("Int64")


def metrics(values: pd.Series) -> dict[str, float]:
    x = values.dropna().to_numpy(dtype=float)
    if not len(x):
        return {}
    ax = np.abs(x)
    return {
        "samples": len(x),
        "mean_signed_m": float(np.mean(x)),
        "mae_m": float(np.mean(ax)),
        "rmse_m": float(np.sqrt(np.mean(x * x))),
        "median_abs_m": float(np.median(ax)),
        "p95_abs_m": float(np.percentile(ax, 95)),
        "max_abs_m": float(np.max(ax)),
        "std_signed_m": float(np.std(x)),
        "within_0.10m_pct": float(100 * np.mean(ax <= 0.10)),
        "within_0.25m_pct": float(100 * np.mean(ax <= 0.25)),
        "within_0.50m_pct": float(100 * np.mean(ax <= 0.50)),
    }


def make_map(df: pd.DataFrame, mission: pd.DataFrame, out: Path) -> None:
    # Dependency-free HTML generator. Leaflet and map tiles load from the internet
    # when the resulting file is opened, just as they do in a Folium map.
    mission_coords = mission[["lat", "lon"]].values.tolist()
    runs = []
    state_groups = df.driving.ne(df.driving.shift()).cumsum()
    for _, run in df.groupby(state_groups):
        runs.append({"driving": bool(run.driving.iloc[0]),
                     "coords": run[["lat", "lon"]].values.tolist()})
    auto = df[df.driving]
    sampled = auto.iloc[::max(1, len(auto) // 500)]
    points = [
        [r.lat, r.lon, r.elapsed_s, r.mission_cte_signed_m,
         r.get("steer_normalized", float("nan"))]
        for _, r in sampled.iterrows()
    ]
    payload = json.dumps({"mission": mission_coords, "runs": runs, "points": points})
    html = """<!doctype html><html><head><meta charset="utf-8">
<title>Pure Pursuit: Actual vs Target</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>html,body,#map{height:100%;margin:0}.legend{background:white;padding:8px;
line-height:1.5;box-shadow:0 1px 5px #777}.sw{display:inline-block;width:18px;
height:4px;margin-right:6px;vertical-align:middle}</style></head><body>
<div id="map"></div><script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>const d=__DATA__; const m=L.map('map');
L.tileLayer('https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png',
 {maxZoom:22,attribution:'&copy; OpenStreetMap &copy; CARTO'}).addTo(m);
const target=L.polyline(d.mission,{color:'#1769e0',weight:4}).bindTooltip('Mission target path').addTo(m);
const driving=L.layerGroup().addTo(m), waiting=L.layerGroup().addTo(m);
d.runs.forEach(r=>L.polyline(r.coords,{color:r.driving?'#e53935':'#777',
 weight:r.driving?4:2,opacity:.9}).addTo(r.driving?driving:waiting));
d.points.forEach(p=>L.circleMarker([p[0],p[1]],{radius:2,color:'#e53935'})
 .bindTooltip(`t=${p[2].toFixed(2)}s<br>mission CTE=${p[3].toFixed(3)} m<br>steer=${p[4].toFixed(3)}`).addTo(driving));
L.control.layers(null,{'Mission target':target,'Controller driving (AUTO proxy)':driving,
 'Not driving':waiting},{collapsed:false}).addTo(m);
const all=d.mission.concat(d.runs.flatMap(r=>r.coords)); m.fitBounds(all);
const Legend=L.Control.extend({onAdd:()=>{const x=L.DomUtil.create('div','legend');
x.innerHTML='<b>Pure Pursuit Track</b><br><span class="sw" style="background:#1769e0"></span>Mission target<br><span class="sw" style="background:#e53935"></span>Driving (AUTO proxy)<br><span class="sw" style="background:#777"></span>Not driving';return x}});
new Legend({position:'bottomleft'}).addTo(m);</script></body></html>""".replace("__DATA__", payload)
    out.write_text(html, encoding="utf-8")


def make_error_plot(df: pd.DataFrame, out: Path) -> None:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    auto = df.driving
    tracking = auto & (df.waypoint_idx > 0) if "waypoint_idx" in df else auto
    ax1.plot(df.loc[auto, "elapsed_s"], df.loc[auto, "mission_cte_signed_m"],
             color="#aaa", lw=.6, label="Driving, including path acquisition")
    ax1.plot(df.loc[tracking, "elapsed_s"], df.loc[tracking, "mission_cte_signed_m"],
             color="#d62728", lw=1, label="Active mission tracking (waypoint > 0)")
    ax1.axhline(0, color="black", lw=.7)
    ax1.axhspan(-.10, .10, color="#2ca02c", alpha=.12, label="±0.10 m")
    ax1.set_ylabel("Signed mission CTE (m)\n(+ left, − right)")
    ax1.grid(alpha=.25)
    ax1.legend(loc="best")
    if "steer_normalized" in df:
        ax2.plot(df.elapsed_s, df.steer_normalized, color="#1f77b4", lw=.8)
    ax2.set_ylabel("Steer command")
    ax2.set_xlabel("Controller elapsed time (s)")
    ax2.set_ylim(-1.05, 1.05)
    ax2.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def write_summary(df: pd.DataFrame, mission: pd.DataFrame, out: Path) -> None:
    auto = df[df.driving].copy()
    # idx==0 includes the approach/path-acquisition period in this log.  It
    # started about 23 m from the first waypoint and would swamp tracking stats.
    tracking = auto[auto.waypoint_idx > 0].copy() if "waypoint_idx" in auto else auto
    overall = metrics(tracking.mission_cte_signed_m)
    distance = max(0.0, tracking.mission_progress_m.max() - tracking.mission_progress_m.min())
    signs = np.sign(tracking.mission_cte_signed_m.to_numpy())
    crossings = int(np.sum(signs[1:] * signs[:-1] < 0)) if len(signs) > 1 else 0
    sat = float(100 * (tracking.steer_normalized.abs() >= .999).mean()) if "steer_normalized" in tracking else math.nan
    lines = [
        "Pure Pursuit Tracking Analysis",
        "=" * 31,
        f"Valid GPS samples: {len(df)}",
        f"Driving/AUTO-proxy samples: {len(auto)} ({100*len(auto)/len(df):.1f}%)",
        f"Active-tracking samples (waypoint_idx > 0): {len(tracking)}",
        f"Mission waypoints: {len(mission)}",
        "",
        "Geometric cross-track error during active mission tracking",
        "(nearest mission polyline; + left / - right of mission direction)",
    ]
    labels = {
        "mean_signed_m": "Bias / mean signed", "mae_m": "MAE",
        "rmse_m": "RMSE", "median_abs_m": "Median absolute",
        "p95_abs_m": "95th percentile absolute", "max_abs_m": "Maximum absolute",
        "std_signed_m": "Signed standard deviation",
    }
    for key, label in labels.items():
        lines.append(f"  {label:28s}: {overall.get(key, math.nan):.3f} m")
    for threshold in ("within_0.10m_pct", "within_0.25m_pct", "within_0.50m_pct"):
        lines.append(f"  {threshold.replace('_pct','').replace('within_','Within ±'):28s}: {overall.get(threshold, math.nan):.1f}%")
    lines += [
        f"  Steering saturation (|cmd|≥.999): {sat:.1f}%",
        f"  Error zero crossings: {crossings}",
        f"  Zero crossings per 10 m progress: {(10*crossings/distance if distance else math.nan):.2f}",
        "",
        "Interpretation",
        "  Bias: persistent left/right offset; inspect alignment/calibration.",
        "  RMSE/P95: overall and near-worst tracking accuracy.",
        "  Frequent zero crossings: weaving/oscillation; compare at the same speed.",
        "  Steering saturation: controller often requests more authority than available.",
        "",
        "Important: CSV cross_track_err_m is abs(yt), the lookahead target's lateral",
        "offset in the tractor frame. It is not geometric distance to the mission path.",
        "This report recomputes geometric CTE from GPS and the mission polyline.",
        "",
        "AUTO caveat: this log has no radio_mode field. 'driving=True' only proves that",
        "the controller sent cmd_vel; it is used here as an AUTO proxy.",
        "",
        "Path-acquisition caveat: waypoint_idx=0 is excluded from the headline CTE.",
        "This run began about 23 m from the first waypoint, so including its approach",
        "would measure start-position error rather than path-following accuracy.",
    ]
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv_file", type=Path)
    p.add_argument("mission_file", type=Path)
    p.add_argument("--out", "-o", type=Path, default=None)
    args = p.parse_args()
    out_dir = args.out or args.csv_file.with_suffix("").with_name(args.csv_file.stem + "_analysis")
    out_dir.mkdir(parents=True, exist_ok=True)

    log = load_log(args.csv_file)
    mission = load_mission(args.mission_file)
    analyzed = geometric_cte(log, mission)
    analyzed["auto_proxy_run"] = contiguous_runs(analyzed.driving)

    make_map(analyzed, mission, out_dir / "pursuit_map.html")
    make_error_plot(analyzed, out_dir / "cross_track_error.png")
    write_summary(analyzed, mission, out_dir / "tracking_summary.txt")
    analyzed.to_csv(out_dir / "tracking_samples.csv", index=False)

    # Distance bins make separate configurations easy to compare at like locations.
    auto = analyzed[analyzed.driving].copy()
    if "waypoint_idx" in auto:
        auto = auto[auto.waypoint_idx > 0].copy()
    auto["distance_bin_m"] = (auto.mission_progress_m // 5 * 5).astype(int)
    rows = []
    for distance_bin, group in auto.groupby("distance_bin_m"):
        rows.append({"distance_bin_start_m": distance_bin, **metrics(group.mission_cte_signed_m)})
    pd.DataFrame(rows).to_csv(out_dir / "cte_by_5m.csv", index=False)
    print(f"Wrote analysis to {out_dir.resolve()}")


if __name__ == "__main__":
    main()
