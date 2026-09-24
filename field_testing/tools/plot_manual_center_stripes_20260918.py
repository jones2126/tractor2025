"""Plot the recorded manual center-cut trace from the September 18 field log.

This is a review-only plot. It does not generate or modify mission waypoints.
"""

import argparse
import csv
import hashlib
import json
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
import numpy as np


DEFAULT_LOG = Path(
    "field_testing/sites/62_Collins_multi_boundary_20260915/runs/"
    "20260918_121342/partial_rings_master_20260918_121342.csv"
)
DEFAULT_OUTPUT = Path(
    "field_testing/sites/62_Collins_multi_boundary_20260915/analysis/"
    "manual_center_stripes_20260918_review.png"
)
DEFAULT_SOURCE = Path(
    "field_testing/sites/62_Collins_multi_boundary_20260915/analysis/"
    "manual_center_stripes_20260918_recorded_source.csv"
)
DEFAULT_REPORT = Path(
    "field_testing/sites/62_Collins_multi_boundary_20260915/analysis/"
    "manual_center_stripes_20260918_recorded_source_report.json"
)
START = datetime(2026, 9, 18, 16, 56, 19, tzinfo=timezone.utc)
END = datetime(2026, 9, 18, 17, 1, 31, tzinfo=timezone.utc)
LOCAL = timezone(timedelta(hours=-4), "EDT")


def load_trace(path: Path):
    rows = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            try:
                stamp = datetime.fromisoformat(row["time"].replace("Z", "+00:00"))
                if not START <= stamp <= END:
                    continue
                if row["steer_mode"] != "1":
                    raise ValueError(f"Non-manual sample in selected window: {stamp}")
                lat = float(row["lat"])
                lon = float(row["lon"])
                if not math.isfinite(lat) or not math.isfinite(lon):
                    continue
            except (TypeError, ValueError):
                # The CSV includes a human-readable second header row.
                if row.get("time", "").startswith("ISO"):
                    continue
                raise
            rows.append((stamp, lat, lon, row["fix_quality"]))
    if len(rows) < 2:
        raise ValueError("No usable manual GPS trace in the requested window")
    return rows


def export_recorded_source(log: Path, output: Path, report_path: Path):
    """Preserve every selected log coordinate verbatim; this is not a mission."""
    fields = ["time", "lat", "lon", "heading_deg", "fix_quality", "speed_mps", "steer_mode"]
    selected = []
    with log.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            try:
                stamp = datetime.fromisoformat(row["time"].replace("Z", "+00:00"))
            except (TypeError, ValueError):
                continue
            if START <= stamp <= END:
                if row["steer_mode"] != "1":
                    raise ValueError(f"Non-manual sample in selected window: {stamp}")
                selected.append({key: row[key] for key in fields})
    if not selected:
        raise ValueError("No selected manual samples")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(selected)
    consecutive_unique_positions = 1 + sum(
        (current["lat"], current["lon"]) != (previous["lat"], previous["lon"])
        for previous, current in zip(selected, selected[1:])
    )
    report = {
        "status": "REVIEW_SOURCE_ONLY_NOT_A_NAVIGATION_MISSION",
        "source_log": str(log).replace("\\", "/"),
        "source_log_sha256": hashlib.sha256(log.read_bytes()).hexdigest(),
        "recorded_source_csv": str(output).replace("\\", "/"),
        "recorded_source_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
        "selection_start_utc": START.isoformat(),
        "selection_end_utc": END.isoformat(),
        "first_recorded_time": selected[0]["time"],
        "last_recorded_time": selected[-1]["time"],
        "recorded_samples": len(selected),
        "consecutive_unique_positions": consecutive_unique_positions,
        "fix_quality_counts": {
            quality: sum(row["fix_quality"] == quality for row in selected)
            for quality in sorted({row["fix_quality"] for row in selected})
        },
        "first_position": {key: selected[0][key] for key in ("lat", "lon")},
        "last_position": {key: selected[-1][key] for key in ("lat", "lon")},
        "limitations": [
            "Raw 20 Hz logger positions include repeated GPS fixes; they are not mission waypoints.",
            "No connector to the existing master mission has been approved.",
            "Some selected positions had DGPS rather than RTK Fixed quality.",
        ],
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def plot_trace(rows, output: Path):
    stamps = [row[0] for row in rows]
    lat0, lon0 = rows[0][1:3]
    meters_per_degree_lat = 111_132.0
    meters_per_degree_lon = 111_320.0 * math.cos(math.radians(lat0))
    east = np.array([(row[2] - lon0) * meters_per_degree_lon for row in rows])
    north = np.array([(row[1] - lat0) * meters_per_degree_lat for row in rows])
    clock = mdates.date2num(stamps)
    points = np.column_stack((east, north))
    segments = np.stack((points[:-1], points[1:]), axis=1)

    fig, ax = plt.subplots(figsize=(10, 9), dpi=180)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#fafafa")
    line = LineCollection(
        segments,
        cmap="viridis",
        norm=Normalize(vmin=clock[0], vmax=clock[-1]),
        linewidth=2.6,
        alpha=0.95,
        zorder=2,
    )
    line.set_array(clock[:-1])
    ax.add_collection(line)

    # Open circles separate the endpoints from nearby overlapping passes.
    ax.scatter(east[0], north[0], s=180, marker="o", facecolor="white",
               edgecolor="#314c86", linewidth=2.4, zorder=5)
    ax.scatter(east[-1], north[-1], s=180, marker="s", facecolor="white",
               edgecolor="#a63835", linewidth=2.4, zorder=5)
    ax.annotate("START", (east[0], north[0]),
                xytext=(11, 13), textcoords="offset points", ha="left", fontsize=10,
                weight="bold", color="#243556", zorder=6)
    ax.annotate("END", (east[-1], north[-1]),
                xytext=(12, -19), textcoords="offset points", fontsize=10,
                weight="bold", color="#8a2826", zorder=6)

    # Label minute boundaries using the closest recorded sample.
    for minute in range(57, 62):
        hour = 16 if minute < 60 else 17
        minute_in_hour = minute if minute < 60 else minute - 60
        tick = datetime(2026, 9, 18, hour, minute_in_hour, tzinfo=timezone.utc)
        if not START < tick < END:
            continue
        idx = min(range(len(stamps)), key=lambda i: abs((stamps[i] - tick).total_seconds()))
        ax.scatter(east[idx], north[idx], s=32, marker="o", facecolor="white",
                   edgecolor="#333333", linewidth=1.1, zorder=4)

    span = max(float(np.ptp(east)), float(np.ptp(north)))
    pad = max(1.5, 0.08 * span)
    ax.set_xlim(float(east.min()) - pad, float(east.max()) + pad)
    ax.set_ylim(float(north.min()) - pad, float(north.max()) + pad)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(color="#d8d8d8", linewidth=0.6, alpha=0.9)
    ax.set_xlabel("East from first recorded point (m)", fontsize=11)
    ax.set_ylabel("North from first recorded point (m)", fontsize=11)
    fig.suptitle("Manually driven center stripes — September 18, 2026",
                 fontsize=15, weight="bold", y=0.985)
    fig.text(0.5, 0.95,
             "Recorded GPS trace only  ·  12:56:19–1:01:31 PM EDT  ·  Manual mode throughout",
             ha="center", va="top", fontsize=10, color="#555555")
    colorbar = fig.colorbar(line, ax=ax, fraction=0.045, pad=0.04)
    ticks = [datetime(2026, 9, 18, 16 if m < 60 else 17, m if m < 60 else m - 60,
                      tzinfo=timezone.utc) for m in range(57, 62)]
    colorbar.set_ticks(mdates.date2num(ticks))
    colorbar.ax.yaxis.set_major_formatter(mdates.DateFormatter("%I:%M %p", tz=LOCAL))
    colorbar.set_label("Time (EDT) — earlier to later", fontsize=10)
    ax.text(0.02, 0.02,
            f"{len(rows):,} logged positions  ·  20 Hz logger  ·  "
            f"{sum(r[3] == 'RTK Fixed' for r in rows):,} RTK Fixed / {len(rows):,} total",
            transform=ax.transAxes, fontsize=9, color="#444444",
            bbox={"facecolor": "white", "edgecolor": "#dddddd", "alpha": 0.9})
    fig.tight_layout(rect=(0, 0, 1, 0.925))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--source-output", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--report-output", type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()
    rows = load_trace(args.log)
    plot_trace(rows, args.output)
    report = export_recorded_source(args.log, args.source_output, args.report_output)
    print(f"Saved {args.output} from {len(rows)} recorded manual-mode positions")
    print(f"First: {rows[0][0].isoformat()}  Last: {rows[-1][0].isoformat()}")
    print(f"Preserved {report['recorded_samples']} exact logger rows in {args.source_output}")


if __name__ == "__main__":
    main()
