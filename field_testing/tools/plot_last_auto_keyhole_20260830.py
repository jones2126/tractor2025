#!/usr/bin/env python3
"""Plot the planned first keyhole against the tractor's final AUTO track."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_mission(path: Path) -> list[tuple[float, float]]:
    lat_lon: list[tuple[float, float]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            parts = line.split()
            if len(parts) == 5:
                lat_lon.append((float(parts[0]), float(parts[1])))

    ref_lat, ref_lon = lat_lon[0]
    cos_lat = math.cos(math.radians(ref_lat))
    return [
        (
            111320.0 * (lon - ref_lon) * cos_lat,
            110540.0 * (lat - ref_lat),
        )
        for lat, lon in lat_lon
    ]


def read_auto_exit_epochs(path: Path) -> list[float]:
    exits: list[float] = []
    previous_mode: str | None = None
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        for row in csv.DictReader(handle):
            mode = row.get("trans_mode", "")
            if previous_mode == "2" and mode != "2":
                try:
                    exits.append(datetime.fromisoformat(row["time"]).timestamp())
                except (KeyError, TypeError, ValueError):
                    pass
            previous_mode = mode
    return exits


def read_pursuit(
    path: Path,
    first_wp: int,
    last_wp: int,
    auto_stop_epoch: float,
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        next(reader, None)  # units/description row
        for row in reader:
            try:
                waypoint = int(row["waypoint_idx"])
                x = float(row["pos_x_m"])
                y = float(row["pos_y_m"])
                elapsed = float(row["elapsed_s"])
                timestamp = float(row["timestamp"])
            except (TypeError, ValueError):
                continue
            if (
                first_wp <= waypoint <= last_wp
                and row["driving"] == "True"
                and timestamp <= auto_stop_epoch
            ):
                rows.append(
                    {
                        "wp": waypoint,
                        "x": x,
                        "y": y,
                        "elapsed": elapsed,
                        "timestamp": timestamp,
                    }
                )
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--pursuit-log", type=Path, required=True)
    parser.add_argument("--field-log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--first-waypoint", type=int, default=3038)
    # WP 3039-3062 form the complete first RLR keyhole. Include a short
    # straight segment before and after it to make the intended maneuver clear.
    parser.add_argument("--last-waypoint", type=int, default=3068)
    args = parser.parse_args()

    mission = read_mission(args.mission)
    auto_exits = read_auto_exit_epochs(args.field_log)
    if not auto_exits:
        raise SystemExit("No Manual transition after AUTO found in field log")
    # The failed keyhole was in the final AUTO interval, so use its exit time.
    auto_stop_epoch = auto_exits[-1]
    actual = read_pursuit(
        args.pursuit_log,
        args.first_waypoint,
        args.last_waypoint,
        auto_stop_epoch,
    )
    if not actual:
        raise SystemExit("No matching AUTO pursuit samples found")

    # Logged waypoint indexes are zero-based, matching controller path indexes.
    planned = [
        (wp, *mission[wp])
        for wp in range(args.first_waypoint, min(args.last_waypoint + 1, len(mission)))
    ]

    fig, ax = plt.subplots(figsize=(12, 9), dpi=160)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#fafafa")

    px = [point[1] for point in planned]
    py = [point[2] for point in planned]
    ax.plot(
        px,
        py,
        color="#1769aa",
        linewidth=3.0,
        linestyle=(0, (6, 3)),
        marker="o",
        markersize=3.5,
        label="Target mission path",
        zorder=2,
    )

    actual_colors = {
        "entry": "#6b7280",
        3041: "#7b1fa2",
        3042: "#ef6c00",
        3043: "#c62828",
        3044: "#00838f",
    }
    actual_labels = {
        "entry": "Actual: targeting WP 3038–3040",
        3041: "Actual: targeting WP 3041 (circling begins)",
        3042: "Actual: targeting WP 3042",
        3043: "Actual: targeting WP 3043",
        3044: "Actual: targeting WP 3044",
    }
    grouped_actual: dict[object, list[dict[str, float]]] = {}
    for row in actual:
        key: object = row["wp"] if row["wp"] >= 3041 else "entry"
        grouped_actual.setdefault(key, []).append(row)
    for key in ("entry", 3041, 3042, 3043, 3044):
        rows = grouped_actual.get(key, [])
        if not rows:
            continue
        ax.plot(
            [row["x"] for row in rows],
            [row["y"] for row in rows],
            color=actual_colors[key],
            linewidth=2.7,
            label=actual_labels[key],
            zorder=3,
        )

    # The final active waypoint at the AUTO-to-Manual transition was 3044.
    # A lower waypoint index was "reached" in the controller sense: the active
    # index advanced past it, which does not necessarily mean the tractor drove
    # directly over that waypoint.
    active_when_stopped = actual[-1]["wp"]
    label_offsets = {
        3037: (-2, -24),
        3038: (-4, -28),
        3039: (-8, -30),
        3040: (-12, -30),
        3041: (8, -18),
        3042: (8, -3),
        3043: (8, 12),
        3044: (8, 25),
        3045: (8, 15),
        3046: (8, 14),
        3047: (8, 13),
        3048: (8, 12),
    }
    for wp in range(3037, 3049):
        x, y = mission[wp]
        if wp < active_when_stopped:
            facecolor, edgecolor, marker, text_color = "#2e7d32", "white", "o", "#1b5e20"
        elif wp == active_when_stopped:
            facecolor, edgecolor, marker, text_color = "#c62828", "white", "D", "#9b1c1c"
        else:
            facecolor, edgecolor, marker, text_color = "white", "#777777", "o", "#555555"
        ax.scatter(
            x,
            y,
            s=90,
            color=facecolor,
            edgecolor=edgecolor,
            marker=marker,
            linewidth=1.5,
            zorder=5,
        )
        ax.annotate(
            f"{wp}",
            (x, y),
            xytext=label_offsets[wp],
            textcoords="offset points",
            fontsize=9,
            weight="bold",
            color=text_color,
            zorder=6,
        )

    start = actual[0]
    end = actual[-1]
    ax.scatter(start["x"], start["y"], marker="^", s=120, color="#2e7d32", zorder=7)
    ax.annotate(
        "Entering first keyhole",
        (start["x"], start["y"]),
        xytext=(8, -20),
        textcoords="offset points",
        fontsize=10,
        color="#1b5e20",
    )
    ax.scatter(end["x"], end["y"], marker="*", s=190, color="#b71c1c", zorder=8)
    ax.annotate(
        "AUTO stopped here",
        (end["x"], end["y"]),
        xytext=(10, -24),
        textcoords="offset points",
        fontsize=10,
        weight="bold",
        color="#b71c1c",
        arrowprops={"arrowstyle": "->", "color": "#b71c1c", "lw": 1.2},
    )

    callout = (
        "Sustained circling began while WP 3041 was the active target.\n"
        "Green waypoint = controller advanced past it (not necessarily driven over)\n"
        "The controller became trapped chasing individual missed waypoints:\n"
        "WP 3041: 25.9 s / ~1.94 circles   •   WP 3042: 26.6 s / ~2.04 circles\n"
        "WP 3043: 23.2 s / ~1.89 circles   •   WP 3044: 6.4 s / ~0.5 circle before Manual"
    )
    ax.text(
        0.02,
        0.02,
        callout,
        transform=ax.transAxes,
        va="bottom",
        fontsize=10,
        color="#222222",
        bbox={"boxstyle": "round,pad=0.55", "facecolor": "white", "edgecolor": "#bbbbbb", "alpha": 0.94},
        zorder=10,
    )

    ax.set_title(
        "First keyhole: planned path vs. actual tractor path",
        fontsize=18,
        weight="bold",
        pad=18,
    )
    ax.text(
        0.5,
        1.012,
        "Actual-path color identifies the active waypoint; sustained circling starts at WP 3041",
        transform=ax.transAxes,
        ha="center",
        va="bottom",
        fontsize=11,
        color="#555555",
    )
    ax.set_xlabel("Local X — east (m)", fontsize=11)
    ax.set_ylabel("Local Y — north (m)", fontsize=11)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, color="#d9d9d9", linewidth=0.8, alpha=0.8)
    # Add waypoint-state keys without repeating them in the plotted path data.
    from matplotlib.lines import Line2D

    handles, labels = ax.get_legend_handles_labels()
    handles.extend(
        [
            Line2D([0], [0], marker="o", linestyle="none", markerfacecolor="#2e7d32", markeredgecolor="white", markersize=8),
            Line2D([0], [0], marker="D", linestyle="none", markerfacecolor="#c62828", markeredgecolor="white", markersize=8),
            Line2D([0], [0], marker="o", linestyle="none", markerfacecolor="white", markeredgecolor="#777777", markersize=8),
        ]
    )
    labels.extend(["Waypoint index advanced past", "Active waypoint when AUTO stopped", "Not reached"])
    ax.legend(handles, labels, loc="upper right", frameon=True, facecolor="white", edgecolor="#cccccc", fontsize=8.2)
    ax.margins(0.08)
    fig.tight_layout()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, bbox_inches="tight", facecolor="white")
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
