"""Build a review-only connector from recorded center stripes to the planned exit.

The existing master and continuation missions are never modified by this tool.
Recorded points retain their logged coordinates; the final curved merge is
constructed and explicitly marked as synthetic for operator review.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


SITE = Path("field_testing/sites/62_Collins_multi_boundary_20260915")
FIELD_LOG = SITE / "runs/20260918_121342/partial_rings_master_20260918_121342.csv"
STRIPES = SITE / "analysis/manual_center_stripes_20260918_recorded_source.csv"
AUDIT = (
    SITE
    / "mission_plans/20260915_master_boundary_replay/generated_rings_only/"
    / "62_Collins_rings_only_resume_wp0091_audit_20260916.csv"
)
OUT = SITE / "analysis"
OUTPUT_CSV = OUT / "manual_center_connector_REVIEW_ONLY_20260919.csv"
OUTPUT_REPORT = OUT / "manual_center_connector_REVIEW_ONLY_20260919.json"
OUTPUT_PNG = OUT / "manual_center_connector_REVIEW_ONLY_20260919.png"
SPLICE_TIME = "2026-09-18T17:01:50.018+00:00"
JOIN_WAYPOINT = 12775
RECORDED_SPACING_M = 0.20
BRIDGE_STEP_M = 0.35
MAX_POINT_GAP_M = 0.5002


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def point_distance(a: dict, b: dict) -> float:
    latitude = math.radians((float(a["lat"]) + float(b["lat"])) / 2)
    return math.hypot(
        (float(a["lon"]) - float(b["lon"])) * 111_320 * math.cos(latitude),
        (float(a["lat"]) - float(b["lat"])) * 111_132,
    )


def angle_difference(a: float, b: float) -> float:
    return abs((a - b + 180) % 360 - 180)


def three_point_radius(a: dict, b: dict, c: dict, lat0: float, lon0: float) -> float:
    first, middle, last = (xy(row, lat0, lon0) for row in (a, b, c))
    side1 = math.dist(first, middle)
    side2 = math.dist(middle, last)
    side3 = math.dist(first, last)
    double_area = abs(
        (middle[0] - first[0]) * (last[1] - middle[1])
        - (middle[1] - first[1]) * (last[0] - middle[0])
    )
    return side1 * side2 * side3 / (2 * double_area) if double_area > 1e-8 else math.inf


def xy(row: dict, lat0: float, lon0: float) -> tuple[float, float]:
    east = (float(row["lon"]) - lon0) * 111_320 * math.cos(math.radians(lat0))
    north = (float(row["lat"]) - lat0) * 111_132
    return east, north


def latlon(east: float, north: float, lat0: float, lon0: float):
    return (
        lat0 + north / 111_132,
        lon0 + east / (111_320 * math.cos(math.radians(lat0))),
    )


def tangent(compass_deg: float) -> np.ndarray:
    radians = math.radians(compass_deg)
    return np.array((math.sin(radians), math.cos(radians)))


def load_inputs():
    with STRIPES.open(newline="", encoding="utf-8-sig") as handle:
        stripe_rows = list(csv.DictReader(handle))
    stripe_end = stripe_rows[-1]
    with FIELD_LOG.open(newline="", encoding="utf-8-sig") as handle:
        logged = [
            row for row in csv.DictReader(handle)
            if row.get("time", "") >= stripe_end["time"]
            and row.get("time", "") <= SPLICE_TIME
            and row.get("steer_mode") == "1"
        ]
    with AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        audit = [row for row in csv.DictReader(handle) if row.get("waypoint", "").isdigit()]
    if not logged or logged[0]["time"] != stripe_end["time"]:
        raise ValueError("Recorded connector does not start at approved stripe endpoint")
    if logged[-1]["time"] != SPLICE_TIME:
        raise ValueError("Expected recorded splice time is absent")
    if any(row["fix_quality"] != "RTK Fixed" for row in logged):
        raise ValueError("Recorded connector includes a non-RTK position")
    if any(row["steer_mode"] != "1" for row in logged):
        raise ValueError("Recorded connector includes a non-Manual position")
    join = next(row for row in audit if int(row["waypoint"]) == JOIN_WAYPOINT)
    if join["phase"] != "main_backyard_exit_connector":
        raise ValueError("Selected rejoin waypoint is not on the planned exit connector")
    return stripe_end, logged, audit, join


def choose_recorded(logged):
    unique = [logged[0]]
    for row in logged[1:]:
        if (row["lat"], row["lon"]) != (unique[-1]["lat"], unique[-1]["lon"]):
            unique.append(row)
    selected = [unique[0]]
    for row in unique[1:-1]:
        if point_distance(selected[-1], row) >= RECORDED_SPACING_M:
            selected.append(row)
    if selected[-1]["time"] != unique[-1]["time"]:
        selected.append(unique[-1])
    if max(point_distance(a, b) for a, b in zip(selected, selected[1:])) > MAX_POINT_GAP_M:
        raise ValueError("Recorded point spacing exceeds mission review threshold")
    return selected


def curved_merge(recorded_end, join, lat0, lon0):
    p0 = np.array(xy(recorded_end, lat0, lon0))
    p3 = np.array(xy(join, lat0, lon0))
    separation = float(np.linalg.norm(p3 - p0))
    start_heading = float(recorded_end["heading_deg"])
    end_heading = (90 - math.degrees(float(join["yaw_rad"]))) % 360
    handle = 0.4 * separation
    p1 = p0 + handle * tangent(start_heading)
    p2 = p3 - handle * tangent(end_heading)
    count = math.ceil(separation / BRIDGE_STEP_M) + 2
    points = []
    for index in range(1, count):
        t = index / (count - 1)
        p = ((1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1
             + 3 * (1 - t) * t * t * p2 + t ** 3 * p3)
        if index == count - 1:
            points.append({"lat": join["lat"], "lon": join["lon"],
                           "origin": "planned_rejoin", "source_waypoint": str(JOIN_WAYPOINT)})
        else:
            lat, lon = latlon(float(p[0]), float(p[1]), lat0, lon0)
            points.append({"lat": f"{lat:.12f}", "lon": f"{lon:.12f}",
                           "origin": "synthetic_curved_merge", "source_waypoint": ""})
    if max(point_distance(a, b) for a, b in zip([recorded_end] + points, points)) > MAX_POINT_GAP_M:
        raise ValueError("Constructed merge point spacing exceeds review threshold")
    return points, separation, start_heading, end_heading


def draw_preview(stripe_end, connector, audit, lat0, lon0, recorded_count):
    fig, ax = plt.subplots(figsize=(10, 9), dpi=180)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#fafafa")

    # The existing route before and after the proposed join is context only.
    planned = [row for row in audit if 12760 <= int(row["waypoint"]) <= 12810]
    planned_pre = [row for row in planned if int(row["waypoint"]) <= JOIN_WAYPOINT]
    planned_post = [row for row in planned if int(row["waypoint"]) >= JOIN_WAYPOINT]
    for rows, color, style, label in (
        (planned_pre, "#9b9b9b", "--", "Planned exit before rejoin (replaced)"),
        (planned_post, "#323d49", "-", "Planned route after rejoin"),
    ):
        coords = np.array([xy(row, lat0, lon0) for row in rows])
        ax.plot(coords[:, 0], coords[:, 1], linestyle=style, color=color,
                linewidth=2, alpha=0.9, label=label, zorder=1)

    recorded = connector[:recorded_count]
    bridge = connector[recorded_count - 1:]
    recorded_xy = np.array([xy(row, lat0, lon0) for row in recorded])
    bridge_xy = np.array([xy(row, lat0, lon0) for row in bridge])
    ax.plot(recorded_xy[:, 0], recorded_xy[:, 1], color="#226b9d", linewidth=3,
            label="Recorded manual return", zorder=3)
    ax.plot(bridge_xy[:, 0], bridge_xy[:, 1], color="#d66a1f", linewidth=3,
            linestyle="--", label="Constructed curved merge — review", zorder=4)

    for index, row in enumerate(connector):
        if index == 0 or (index + 1) % 5 != 0:
            continue
        x, y = xy(row, lat0, lon0)
        ax.scatter(x, y, s=40, color="white", edgecolor="#222222", zorder=6)
        left_side = (index // 5) % 2 == 0
        offset = (-10, 8) if left_side else (10, -11)
        ax.annotate(f"C{index + 1}", (x, y), xytext=offset,
                    textcoords="offset points", ha="right" if left_side else "left",
                    fontsize=9, weight="bold", zorder=7)
    ax.scatter(*xy(stripe_end, lat0, lon0), s=130, marker="o", facecolor="white",
               edgecolor="#226b9d", linewidth=2.5, zorder=8)
    ax.scatter(*xy(connector[-1], lat0, lon0), s=150, marker="s", facecolor="white",
               edgecolor="#d66a1f", linewidth=2.5, zorder=8)
    start_xy = xy(stripe_end, lat0, lon0)
    end_xy = xy(connector[-1], lat0, lon0)
    ax.annotate("STRIPES END / C1", start_xy, xytext=(-2, -24),
                textcoords="offset points", ha="left", fontsize=10,
                weight="bold", color="#1b587e")
    ax.annotate("REJOIN W12775", end_xy, xytext=(11, 12),
                textcoords="offset points", fontsize=10, weight="bold", color="#a64913")
    all_xy = np.array([xy(row, lat0, lon0) for row in connector + planned])
    pad = max(2.5, 0.10 * max(np.ptp(all_xy[:, 0]), np.ptp(all_xy[:, 1])))
    ax.set_xlim(float(all_xy[:, 0].min()) - pad, float(all_xy[:, 0].max()) + pad)
    ax.set_ylim(float(all_xy[:, 1].min()) - pad, float(all_xy[:, 1].max()) + pad)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(color="#d9d9d9", linewidth=0.6)
    ax.set_xlabel("East from stripe endpoint (m)")
    ax.set_ylabel("North from stripe endpoint (m)")
    fig.suptitle("Center-stripe exit connector — review only", fontsize=15, weight="bold", y=0.98)
    fig.text(0.5, 0.947, "C5, C10, … label every fifth candidate point; orange section was not driven",
             ha="center", va="top", fontsize=10, color="#555555")
    ax.legend(loc="upper left", fontsize=9, framealpha=0.94)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(OUTPUT_PNG, bbox_inches="tight")
    plt.close(fig)


def main():
    stripe_end, logged, audit, join = load_inputs()
    recorded = choose_recorded(logged)
    lat0, lon0 = float(stripe_end["lat"]), float(stripe_end["lon"])
    bridge, distance, start_heading, end_heading = curved_merge(recorded[-1], join, lat0, lon0)
    connector = []
    for row in recorded:
        connector.append({"lat": row["lat"], "lon": row["lon"],
                          "origin": "recorded_manual", "source_waypoint": "",
                          "time": row["time"], "fix_quality": row["fix_quality"]})
    for row in bridge:
        connector.append({**row, "time": "", "fix_quality": ""})
    for index, row in enumerate(connector, 1):
        row["point"] = f"C{index}"
    gaps = [point_distance(a, b) for a, b in zip(connector, connector[1:])]
    bridge_radii = [
        three_point_radius(*connector[index - 1:index + 2], lat0, lon0)
        for index in range(len(recorded), len(connector) - 1)
    ]
    if max(gaps) > MAX_POINT_GAP_M:
        raise ValueError("Connector has an excessive point gap")
    OUT.mkdir(parents=True, exist_ok=True)
    columns = ["point", "origin", "time", "lat", "lon", "fix_quality", "source_waypoint"]
    with OUTPUT_CSV.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(connector)
    draw_preview(stripe_end, connector, audit, lat0, lon0, len(recorded))
    report = {
        "status": "REVIEW_ONLY_NOT_A_NAVIGATION_MISSION",
        "approved_stripe_end_time": stripe_end["time"],
        "recorded_connector_end_time": recorded[-1]["time"],
        "planned_rejoin_waypoint": JOIN_WAYPOINT,
        "planned_rejoin_phase": join["phase"],
        "recorded_points": len(recorded),
        "constructed_points_including_rejoin": len(bridge),
        "total_connector_points": len(connector),
        "constructed_bridge_endpoint_separation_m": round(distance, 3),
        "recorded_heading_at_bridge_deg": round(start_heading, 2),
        "planned_heading_at_rejoin_deg": round(end_heading, 2),
        "heading_difference_deg": round(angle_difference(start_heading, end_heading), 2),
        "maximum_candidate_point_gap_m": round(max(gaps), 3),
        "minimum_constructed_merge_three_point_radius_m": round(min(bridge_radii), 3),
        "source_field_log_sha256": sha256(FIELD_LOG),
        "source_stripe_trace_sha256": sha256(STRIPES),
        "source_planned_audit_sha256": sha256(AUDIT),
        "candidate_csv_sha256": sha256(OUTPUT_CSV),
        "limitations": [
            "No point in the constructed curved merge was manually driven or field verified.",
            "The existing master and continuation missions remain unchanged.",
            "Obstacle clearance and achievable turn radius require separate validation before operation.",
        ],
    }
    OUTPUT_REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in (
        "recorded_points", "constructed_points_including_rejoin",
        "total_connector_points", "constructed_bridge_endpoint_separation_m",
        "maximum_candidate_point_gap_m")}, indent=2))
    print(OUTPUT_PNG)


if __name__ == "__main__":
    main()
