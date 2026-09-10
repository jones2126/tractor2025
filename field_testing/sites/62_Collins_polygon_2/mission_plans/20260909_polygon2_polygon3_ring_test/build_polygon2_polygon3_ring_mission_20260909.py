#!/usr/bin/env python3
"""Build the supervised Polygon 2 -> Polygon 3 ring test mission."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, Polygon


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
PLANNER_DIR = REPO / "tractor_rpi" / "pure-pursuit" / "mission_planning"
sys.path.insert(0, str(PLANNER_DIR))

from site_planner_common_20260724 import (  # noqa: E402
    MissionPoint,
    circumcircle_radius,
    deduplicate_points,
    densify_polyline,
    distance,
    load_boundary,
    polyline_length,
    segment_yaw,
    write_mission,
)
from site_coverage_planner_20260724 import (  # noqa: E402
    append_connector,
    append_piece,
    make_headland_paths,
    path_end_pose,
    path_start_pose,
    plan_headland_chain,
    ring_variants,
    valid_connector,
)


P2_BOUNDARY = HERE.parents[1] / "01_boundary_final.csv"
P3_BOUNDARY = REPO / "field_testing" / "sites" / "62_Collins_polygon_3" / "01_boundary_final.csv"
SOURCE_LOG = (
    REPO
    / "field_testing"
    / "sites"
    / "62_Collins_polygon_1"
    / "runs"
    / "20260908_polygons_2_and_3_perimeter"
    / "62_collins_polygon_2_20260908.csv"
)
MISSION = HERE / "62_Collins_polygon2_polygon3_rings_1mps_20260909.txt"
AUDIT = HERE / "62_Collins_polygon2_polygon3_rings_1mps_20260909_audit.csv"
REPORT = HERE / "62_Collins_polygon2_polygon3_rings_1mps_20260909_report.json"
PREVIEW = HERE / "62_Collins_polygon2_polygon3_rings_1mps_20260909.png"

SPEED_MPS = 1.0
LOOKAHEAD_M = 2.0
SPACING_M = 0.50
LANE_SPACING_M = 0.9652
TURN_RADIUS_M = 1.90
BOUNDARY_OUTSET_M = 0.381
TRANSITION_START_S = 241.68
TRANSITION_END_S = 292.97
EXPECTED_SOURCE_SHA256 = "e8abf55919e3072c01453bc9456c3d204c2f150c8842091262d06a1176b990f9"


def select_transition(frame):
    samples = []
    with SOURCE_LOG.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            elapsed = float(row["elapsed_sec"])
            if TRANSITION_START_S <= elapsed <= TRANSITION_END_S:
                if row["fix_quality"] != "RTK Fixed":
                    raise ValueError("Recorded transition contains a non-fixed RTK sample")
                samples.append(frame.to_xy(float(row["lat"]), float(row["lon"])))
    samples = deduplicate_points(samples, 0.03)
    if len(samples) < 2:
        raise ValueError("Recorded transition did not contain enough moving samples")
    line = LineString(samples)
    # Remove centimetre-scale GPS wander while retaining the driven shape, then
    # densify each retained segment so no mission gap exceeds 0.50 m.
    simplified = line.simplify(0.05, preserve_topology=False)
    points = densify_polyline(list(simplified.coords), SPACING_M)
    return [(float(x), float(y)) for x, y in points], len(samples), line


def plan_inbound_chain(headlands, incoming_pose, drive_area):
    """Choose ring splice points requiring a contained inbound connector."""
    states = []
    for variant in ring_variants(headlands[0]["points"], True):
        connector = valid_connector(
            incoming_pose, variant["start_pose"], drive_area, TURN_RADIUS_M, SPACING_M
        )
        if connector is not None:
            states.append({
                **variant,
                "label": headlands[0]["label"],
                "headland_pass": headlands[0]["pass"],
                "cost": float(connector["length_m"]),
                "previous": None,
                "incoming": connector,
            })
    if not states:
        raise ValueError("No contained connector can enter Polygon 3 outer ring")

    for ring_info in headlands[1:]:
        new_states = []
        for variant in ring_variants(ring_info["points"], True):
            best = None
            for previous in states:
                connector = valid_connector(
                    previous["end_pose"], variant["start_pose"], drive_area,
                    TURN_RADIUS_M, SPACING_M,
                )
                if connector is None:
                    continue
                cost = previous["cost"] + float(connector["length_m"])
                if best is None or cost < best["cost"]:
                    best = {
                        **variant,
                        "label": ring_info["label"],
                        "headland_pass": ring_info["pass"],
                        "cost": cost,
                        "previous": previous,
                        "incoming": connector,
                    }
            if best is not None:
                new_states.append(best)
        if not new_states:
            raise ValueError(f"No contained connector can reach {ring_info['label']}")
        states = new_states

    state = min(states, key=lambda item: item["cost"])
    selected = []
    while state is not None:
        selected.append(state)
        state = state["previous"]
    return list(reversed(selected))


def main():
    source_sha = hashlib.sha256(SOURCE_LOG.read_bytes()).hexdigest()
    if source_sha != EXPECTED_SOURCE_SHA256:
        raise ValueError(
            "Combined perimeter source log changed; review it before rebuilding this mission"
        )
    frame, p2_xy, _ = load_boundary(P2_BOUNDARY)
    _, p3_xy, _ = load_boundary(P3_BOUNDARY, frame=frame)
    p2_boundary = Polygon(p2_xy)
    p3_boundary = Polygon(p3_xy)
    p2_drive = p2_boundary.buffer(BOUNDARY_OUTSET_M, join_style="round")
    p3_drive = p3_boundary.buffer(BOUNDARY_OUTSET_M, join_style="round")
    transition, raw_transition_points, transition_source = select_transition(frame)

    p2_headlands = make_headland_paths(
        p2_drive, 2, LANE_SPACING_M, TURN_RADIUS_M, outer_boundary=p2_boundary
    )
    p3_headlands = make_headland_paths(
        p3_drive, 3, LANE_SPACING_M, TURN_RADIUS_M, outer_boundary=p3_boundary
    )
    p2_selected, p2_outgoing = plan_headland_chain(
        p2_headlands, transition[0], path_start_pose(transition), True,
        p2_drive, TURN_RADIUS_M, SPACING_M,
    )
    p3_selected = plan_inbound_chain(p3_headlands, path_end_pose(transition), p3_drive)

    route = []
    for state in p2_selected:
        if state["incoming"] is not None:
            append_connector(route, state["incoming"])
            for item in route:
                if item["kind"] == "connector" and "polygon" not in item:
                    item["polygon"] = 2
        append_piece(route, state["ring"], "ring", SPACING_M,
                     headland_pass=int(state["headland_pass"]))
        for item in route:
            if item["kind"] == "ring" and "polygon" not in item:
                item["polygon"] = 2
    append_connector(route, p2_outgoing)
    for item in route:
        if item["kind"] == "connector" and "polygon" not in item:
            item["polygon"] = 2
    append_piece(route, transition, "recorded_transition", SPACING_M)
    for item in route:
        if item["kind"] == "recorded_transition" and "polygon" not in item:
            item["polygon"] = 0
    for state in p3_selected:
        append_connector(route, state["incoming"])
        for item in route:
            if item["kind"] == "connector" and "polygon" not in item:
                item["polygon"] = 3
        append_piece(route, state["ring"], "ring", SPACING_M,
                     headland_pass=int(state["headland_pass"]))
        for item in route:
            if item["kind"] == "ring" and "polygon" not in item:
                item["polygon"] = 3

    xy = [item["xy"] for item in route]
    if any(distance(a, b) > SPACING_M + 1e-6 for a, b in zip(xy, xy[1:])):
        raise ValueError("Mission contains a waypoint gap larger than 0.50 m")
    for item in route:
        if item["polygon"] == 2 and not p2_drive.buffer(0.03).covers(Point(item["xy"])):
            raise ValueError("Polygon 2 route point is outside its drive area")
        if item["polygon"] == 3 and not p3_drive.buffer(0.03).covers(Point(item["xy"])):
            raise ValueError("Polygon 3 route point is outside its drive area")
    for first, second in zip(route, route[1:]):
        if first["polygon"] == second["polygon"] == 2:
            if not p2_drive.buffer(0.03).covers(LineString([first["xy"], second["xy"]])):
                raise ValueError("Polygon 2 route segment is outside its drive area")
        if first["polygon"] == second["polygon"] == 3:
            if not p3_drive.buffer(0.03).covers(LineString([first["xy"], second["xy"]])):
                raise ValueError("Polygon 3 route segment is outside its drive area")

    mission = []
    audit_rows = []
    for index, item in enumerate(route):
        if index < len(route) - 1:
            yaw = segment_yaw(xy[index], xy[index + 1])
        else:
            yaw = segment_yaw(xy[index - 1], xy[index])
        lat, lon = frame.to_latlon(*item["xy"])
        mission.append(MissionPoint(lat, lon, yaw, LOOKAHEAD_M, SPEED_MPS))
        radius = math.inf
        if 0 < index < len(route) - 1:
            radius = circumcircle_radius(xy[index - 1], xy[index], xy[index + 1])
        audit_rows.append({
            "waypoint": index + 1, "polygon": item["polygon"], "kind": item["kind"],
            "ring_pass": item.get("headland_pass", ""), "lat": f"{lat:.9f}",
            "lon": f"{lon:.9f}", "east_m": f"{item['xy'][0]:.3f}",
            "north_m": f"{item['xy'][1]:.3f}", "yaw_rad": f"{yaw:.6f}",
            "lookahead_m": f"{LOOKAHEAD_M:.2f}", "speed_mps": f"{SPEED_MPS:.2f}",
            "local_radius_m": "inf" if math.isinf(radius) else f"{radius:.3f}",
        })

    write_mission(MISSION, mission)
    with AUDIT.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(audit_rows[0]))
        writer.writeheader(); writer.writerows(audit_rows)

    transition_route = LineString([item["xy"] for item in route if item["kind"] == "recorded_transition"])
    transition_deviation = transition_route.hausdorff_distance(transition_source)
    if transition_deviation > 0.30:
        raise ValueError(
            f"Resampled transition deviates {transition_deviation:.3f} m from its source"
        )
    report = {
        "mission": MISSION.name,
        "waypoints": len(route),
        "route_length_m": round(polyline_length(xy), 3),
        "commanded_speed_mps": SPEED_MPS,
        "lookahead_m": LOOKAHEAD_M,
        "waypoint_spacing_m": SPACING_M,
        "lane_spacing_m": LANE_SPACING_M,
        "turn_radius_m": TURN_RADIUS_M,
        "polygon_2_passes": 2,
        "polygon_3_passes": 3,
        "recorded_transition_elapsed_sec": [TRANSITION_START_S, TRANSITION_END_S],
        "recorded_transition_unique_source_points": raw_transition_points,
        "recorded_transition_length_m": round(transition_source.length, 3),
        "transition_resample_hausdorff_m": round(transition_deviation, 4),
        "source_log_sha256": source_sha,
        "mission_sha256": hashlib.sha256(MISSION.read_bytes()).hexdigest(),
        "start": {"lat": mission[0].lat, "lon": mission[0].lon,
                  "heading_deg": math.degrees(mission[0].yaw_rad) % 360.0},
        "end": {"lat": mission[-1].lat, "lon": mission[-1].lon,
                "heading_deg": math.degrees(mission[-1].yaw_rad) % 360.0},
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    fig, ax = plt.subplots(figsize=(13, 10))
    for polygon, color, label in [(p2_boundary, "#333333", "Polygon 2 boundary"),
                                   (p3_boundary, "#777777", "Polygon 3 boundary")]:
        bx, by = polygon.exterior.xy; ax.plot(bx, by, color=color, lw=1.5, label=label)
    styles = {
        (2, "ring"): ("#1261a0", "Polygon 2 rings"),
        (2, "connector"): ("#55a6d9", "Polygon 2 connectors"),
        (0, "recorded_transition"): ("#e68613", "Recorded P2 to P3 transition"),
        (3, "connector"): ("#70ad47", "Polygon 3 connectors"),
        (3, "ring"): ("#237a3b", "Polygon 3 rings"),
    }
    shown = set()
    for key, (color, label) in styles.items():
        groups = []
        current = []
        for i, item in enumerate(route):
            if (item["polygon"], item["kind"]) == key:
                current.append(i)
            elif current:
                groups.append(current); current = []
        if current:
            groups.append(current)
        for group in groups:
            ax.plot([xy[i][0] for i in group], [xy[i][1] for i in group], color=color,
                    lw=2.2, label=label if label not in shown else None)
            shown.add(label)
    ax.scatter(*xy[0], marker="o", s=90, color="limegreen", edgecolor="black", zorder=5, label="Mission start")
    ax.scatter(*xy[-1], marker="X", s=90, color="red", edgecolor="black", zorder=5, label="Mission end")
    ax.set_aspect("equal", adjustable="box"); ax.grid(True, alpha=.25)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title("Polygon 2 + Polygon 3 Ring Test — 1.00 m/s")
    ax.legend(loc="best"); fig.tight_layout(); fig.savefig(PREVIEW, dpi=180); plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
