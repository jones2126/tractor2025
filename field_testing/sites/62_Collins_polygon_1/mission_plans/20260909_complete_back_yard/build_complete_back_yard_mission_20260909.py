#!/usr/bin/env python3
"""Build the complete Polygon 1 -> 2 -> 3 backyard mission."""

from __future__ import annotations

import csv
import contextlib
import hashlib
import importlib.util
import io
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
PLANNER_DIR = REPO / "tractor_rpi" / "pure-pursuit" / "mission_planning"
sys.path.insert(0, str(PLANNER_DIR))

from site_planner_common_20260724 import (  # noqa: E402
    MissionPoint,
    deduplicate_points,
    densify_polyline,
    distance,
    load_boundary,
    polyline_length,
    segment_yaw,
    write_mission,
)
from site_coverage_planner_20260724 import valid_connector  # noqa: E402


P1_BOUNDARY = (
    HERE.parents[1] / "mission_plans" / "20260830_at340_combined_test"
    / "01_boundary" / "01_boundary_final.csv"
)
P1_COVERAGE = (
    HERE.parents[1]
    / "mission_plans"
    / "20260830_at340_combined_test"
    / "05_review_test"
    / "62_Collins_combined_14ring_21stripe_REVIEW_TEST_20260830.txt"
)
P1_COVERAGE_SHA256 = "eae4849dfb988082eb91e5f7b9aae602bbb14938cc450490bd333305d35b5ea7"
COMBINED_LOG = (
    HERE.parents[1]
    / "runs"
    / "20260908_polygons_2_and_3_perimeter"
    / "62_collins_polygon_2_20260908.csv"
)
P2_PACKAGE = (
    REPO / "field_testing" / "sites" / "62_Collins_polygon_2" / "mission_plans"
    / "20260909_polygon2_polygon3_ring_test"
)
P2_BUILDER = P2_PACKAGE / "build_polygon2_polygon3_ring_mission_20260909.py"
P2_MISSION = P2_PACKAGE / "62_Collins_polygon2_polygon3_rings_1mps_20260909.txt"
P2_AUDIT = P2_PACKAGE / "62_Collins_polygon2_polygon3_rings_1mps_20260909_audit.csv"

MISSION = HERE / "62_Collins_complete_back_yard_1mps_20260909.txt"
AUDIT = HERE / "62_Collins_complete_back_yard_1mps_20260909_audit.csv"
REPORT = HERE / "62_Collins_complete_back_yard_1mps_20260909_report.json"
PREVIEW = HERE / "62_Collins_complete_back_yard_1mps_20260909.png"

SPEED_MPS = 1.0
WAYPOINT_SPACING_M = 0.50
TURN_RADIUS_M = 1.90
BOUNDARY_OUTSET_M = 0.381
P1_RING_ROWS = 2976
P1_STRIPE_START_ROW = 3027


def portable_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def load_p2_builder():
    spec = importlib.util.spec_from_file_location("polygon23_builder", P2_BUILDER)
    if spec is None or spec.loader is None:
        raise ValueError("Cannot load the Polygon 2/3 mission builder")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def resample_log_windows(frame, windows):
    points = []
    all_fixed = True
    with COMBINED_LOG.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    for start_s, end_s in windows:
        selected = [row for row in rows if start_s <= float(row["elapsed_sec"]) <= end_s]
        all_fixed &= all(row["fix_quality"] == "RTK Fixed" for row in selected)
        window_xy = deduplicate_points(
            [frame.to_xy(float(row["lat"]), float(row["lon"])) for row in selected], 0.03
        )
        if points and distance(points[-1], window_xy[0]) < 0.10:
            window_xy = window_xy[1:]
        points.extend(window_xy)
    if not all_fixed or len(points) < 2:
        raise ValueError("Recorded Polygon 1-to-2 travel is incomplete or not RTK Fixed")
    source = LineString(points)
    # Keep the recorded route itself; deduplication above only removes the
    # stationary centimetre-scale repeats from the pause windows.
    dense = densify_polyline(points, WAYPOINT_SPACING_M)
    return dense, source


def append_points(route, points, phase, lookahead_m, *, source_rows=None):
    for index, point in enumerate(points):
        if route and distance(route[-1]["xy"], point) < 0.01:
            continue
        item = {"xy": point, "phase": phase, "lookahead_m": lookahead_m}
        if source_rows is not None:
            item["source_row"] = source_rows[index]
        route.append(item)


def main():
    if portable_sha256(P1_COVERAGE) != P1_COVERAGE_SHA256:
        raise ValueError("Reviewed Polygon 1 coverage mission changed")

    p2_builder = load_p2_builder()
    with contextlib.redirect_stdout(io.StringIO()):
        p2_builder.main()

    frame, p1_xy, _ = load_boundary(P1_BOUNDARY)
    p1_area = Polygon(p1_xy).buffer(BOUNDARY_OUTSET_M, join_style="round")
    p1_source_rows = [list(map(float, line.split())) for line in P1_COVERAGE.read_text().splitlines()]
    p1_route = [frame.to_xy(row[0], row[1]) for row in p1_source_rows]
    if not p1_area.buffer(0.03).covers(LineString(p1_route)):
        raise ValueError("Reviewed Polygon 1 coverage is outside its drive area")

    travel12, travel12_source = resample_log_windows(
        frame, [(26.59, 85.37), (116.78, 120.76)]
    )
    p1_end_pose = (*p1_route[-1], segment_yaw(p1_route[-2], p1_route[-1]))
    travel12_start_pose = (*travel12[0], segment_yaw(travel12[0], travel12[1]))
    p1_exit = valid_connector(
        p1_end_pose, travel12_start_pose, p1_area,
        TURN_RADIUS_M, WAYPOINT_SPACING_M,
    )
    if p1_exit is None:
        raise ValueError("No contained connector can leave Polygon 1 for the recorded travel path")

    with P2_AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        p23_audit = list(csv.DictReader(handle))
    p23_route = [frame.to_xy(float(row["lat"]), float(row["lon"])) for row in p23_audit]
    if distance(travel12[-1], p23_route[0]) > 0.05:
        raise ValueError("Recorded Polygon 1-to-2 travel does not meet the Polygon 2 start")

    route = []
    for index, (point, source) in enumerate(zip(p1_route, p1_source_rows), 1):
        phase = "polygon_1_rings" if index <= P1_RING_ROWS else (
            "polygon_1_internal_connector" if index < P1_STRIPE_START_ROW else "polygon_1_stripes"
        )
        append_points(route, [point], phase, source[3])
    append_points(route, [(x, y) for x, y, _yaw in p1_exit["points"]][1:],
                  "polygon_1_exit_connector", 1.50)
    append_points(route, travel12, "recorded_travel_polygon_1_to_2", 2.00)
    for point, source in zip(p23_route, p23_audit):
        phase = f"polygon_{source['polygon']}_{source['kind']}"
        append_points(route, [point], phase, float(source["lookahead_m"]))

    xy = [item["xy"] for item in route]
    gaps = [distance(a, b) for a, b in zip(xy, xy[1:])]
    if max(gaps) > WAYPOINT_SPACING_M + 0.001:
        raise ValueError(f"Master mission waypoint gap is {max(gaps):.3f} m")

    travel12_route = LineString(
        [item["xy"] for item in route if item["phase"] == "recorded_travel_polygon_1_to_2"]
    )
    travel12_deviation = travel12_route.hausdorff_distance(travel12_source)
    if travel12_deviation > 0.30:
        raise ValueError("Resampled Polygon 1-to-2 travel deviates from the recording")

    mission = []
    audit_rows = []
    for index, item in enumerate(route):
        yaw = segment_yaw(xy[index], xy[index + 1]) if index < len(xy) - 1 else segment_yaw(xy[-2], xy[-1])
        lat, lon = frame.to_latlon(*item["xy"])
        lookahead = float(item["lookahead_m"])
        mission.append(MissionPoint(lat, lon, yaw, lookahead, SPEED_MPS))
        audit_rows.append({
            "waypoint": index + 1, "phase": item["phase"],
            "lat": f"{lat:.9f}", "lon": f"{lon:.9f}",
            "east_m": f"{item['xy'][0]:.3f}", "north_m": f"{item['xy'][1]:.3f}",
            "yaw_rad": f"{yaw:.6f}", "lookahead_m": f"{lookahead:.2f}",
            "speed_mps": f"{SPEED_MPS:.2f}",
        })
    write_mission(MISSION, mission)
    with AUDIT.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(audit_rows[0]))
        writer.writeheader(); writer.writerows(audit_rows)

    phase_counts = {}
    for item in route:
        phase_counts[item["phase"]] = phase_counts.get(item["phase"], 0) + 1
    report = {
        "mission": MISSION.name,
        "waypoints": len(route),
        "route_length_m": round(polyline_length(xy), 3),
        "estimated_runtime_minutes_at_1mps": round(polyline_length(xy) / 60.0, 2),
        "commanded_speed_mps": SPEED_MPS,
        "maximum_waypoint_gap_m": round(max(gaps), 4),
        "turn_radius_m": TURN_RADIUS_M,
        "polygon_1_rings": 14,
        "polygon_1_stripes": 21,
        "polygon_2_rings": 2,
        "polygon_3_rings": 3,
        "polygon_1_exit_connector": {"mode": p1_exit["mode"], "length_m": p1_exit["length_m"]},
        "recorded_travel_polygon_1_to_2_length_m": round(travel12_source.length, 3),
        "recorded_travel_polygon_1_to_2_hausdorff_m": round(travel12_deviation, 4),
        "phase_waypoint_counts": phase_counts,
        "p1_source_sha256": portable_sha256(P1_COVERAGE),
        "p23_source_sha256": portable_sha256(P2_MISSION),
        "mission_sha256": portable_sha256(MISSION),
        "start": {"lat": mission[0].lat, "lon": mission[0].lon,
                  "heading_deg": (90 - math.degrees(mission[0].yaw_rad)) % 360},
        "end": {"lat": mission[-1].lat, "lon": mission[-1].lon,
                "heading_deg": (90 - math.degrees(mission[-1].yaw_rad)) % 360},
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    fig, ax = plt.subplots(figsize=(14, 12))
    styles = {
        "polygon_1_rings": ("#1f77b4", "Polygon 1 — 14 rings"),
        "polygon_1_internal_connector": ("#61a5d8", "Polygon 1 ring-to-stripe connector"),
        "polygon_1_stripes": ("#6f42c1", "Polygon 1 — 21 stripes"),
        "polygon_1_exit_connector": ("#d62728", "Planned P1 exit connector"),
        "recorded_travel_polygon_1_to_2": ("#ff7f0e", "Recorded travel to Polygon 2"),
        "polygon_2_ring": ("#008b8b", "Polygon 2 rings"),
        "polygon_2_connector": ("#63c7c7", "Polygon 2 connectors"),
        "polygon_0_recorded_transition": ("#e6a700", "Recorded P2 to P3 travel"),
        "polygon_3_ring": ("#2b8a3e", "Polygon 3 rings"),
        "polygon_3_connector": ("#78c679", "Polygon 3 connectors"),
    }
    shown = set()
    for start in range(len(route)):
        if start and route[start - 1]["phase"] == route[start]["phase"]:
            continue
        end = start + 1
        while end < len(route) and route[end]["phase"] == route[start]["phase"]:
            end += 1
        phase = route[start]["phase"]
        color, label = styles[phase]
        ax.plot([p[0] for p in xy[start:end]], [p[1] for p in xy[start:end]],
                color=color, lw=1.7, label=label if label not in shown else None)
        shown.add(label)
    ax.scatter(*xy[0], s=100, color="limegreen", edgecolor="black", zorder=6, label="Mission start")
    ax.scatter(*xy[-1], s=100, marker="X", color="red", edgecolor="black", zorder=6, label="Mission end")
    for i in range(35, len(xy) - 1, 85):
        ax.annotate("", xy=xy[i + 1], xytext=xy[i],
                    arrowprops={"arrowstyle": "-|>", "color": "black", "lw": 1.0,
                                "mutation_scale": 12})
    ax.set_aspect("equal", adjustable="box"); ax.grid(True, alpha=0.25)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title("Complete 62 Collins Back Yard Mission — 1.00 m/s")
    ax.legend(loc="upper left", fontsize=9); fig.tight_layout()
    fig.savefig(PREVIEW, dpi=180); plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
