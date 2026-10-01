#!/usr/bin/env python3
"""Build the supervised, blades-off 62 Collins perimeter field-test package."""

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
from shapely.geometry import LineString, Point, shape

from build_62_collins_site_inventory_rev003_20260926 import REPO, SITE, load_frame, to_xy


REVIEW = SITE / "mission_plans/20260927_outer_perimeter_1mps_REVIEW_ONLY"
INVENTORY = (
    SITE / "site_inventory/revisions/rev_003_20260926_REVIEW_ONLY"
    / "62_Collins_site_inventory_rev003_REVIEW_ONLY.geojson"
)
OUT = SITE / "mission_plans/20260928_outer_perimeter_field_test"
GENERATED = OUT / "generated"
MISSION = GENERATED / "62_Collins_outer_perimeter_field_test_1mps_20260928.txt"
AUDIT = GENERATED / "62_Collins_outer_perimeter_field_test_1mps_20260928_audit.csv"
REPORT = GENERATED / "62_Collins_outer_perimeter_field_test_1mps_20260928_report.json"
PREVIEW = GENERATED / "62_Collins_outer_perimeter_field_test_1mps_REVIEW.png"

SPACING_M = 0.20
SPEED_MPS = 1.0
NORMAL_LOOKAHEAD_M = 2.0
TIGHT_LOOKAHEAD_M = 1.0
TIGHT_ZONE_M = 3.0

BACKYARD = "AREA-BACKYARD-GARDENS"
FRONT = "AREA-FRONT-YARD"
OVERROAD = "AREA-OVER-ROAD"


def resample_line(line: LineString, spacing=SPACING_M):
    count = max(2, math.ceil(line.length / spacing))
    return [line.interpolate(index * line.length / count).coords[0] for index in range(count + 1)]


def straight(a, b):
    return resample_line(LineString([a, b]))


def read_review_paths():
    rows = list(csv.DictReader((REVIEW / "62_Collins_outer_perimeter_1mps_audit.csv").open(newline="", encoding="utf-8-sig")))
    paths = {}
    tight = {}
    for area_id in (BACKYARD, FRONT, OVERROAD):
        selected = [row for row in rows if row["area_id"] == area_id]
        paths[area_id] = [(float(row["east_m"]), float(row["north_m"])) for row in selected]
        tight_indices = [index for index, row in enumerate(selected) if row["sharp_turn_review"] == "yes"]
        radius = math.ceil(TIGHT_ZONE_M / SPACING_M)
        tight[area_id] = {
            candidate
            for index in tight_indices
            for candidate in range(max(0, index - radius), min(len(selected), index + radius + 1))
        }
    return paths, tight


def load_routes(frame):
    collection = json.loads(INVENTORY.read_text(encoding="utf-8"))
    routes = {}
    for item in collection["features"]:
        props = item["properties"]
        if props.get("geometry_role") == "recorded_centerline_between_areas":
            routes[props["asset_id"]] = to_xy(frame, shape(item["geometry"]))
    return routes


def nearest_index(points, target):
    return min(range(len(points) - 1), key=lambda index: math.dist(points[index], target))


def clockwise_arc(points, start_index, end_index):
    ring = points[:-1]
    if end_index >= start_index:
        return ring[start_index:end_index + 1]
    return ring[start_index:] + ring[:end_index + 1]


def shortest_ring_arc(points, start_index, end_index):
    """Use the shorter boundary arc for an access transit after a full lap."""
    clockwise = clockwise_arc(points, start_index, end_index)
    reverse_clockwise = list(reversed(clockwise_arc(points, end_index, start_index)))
    clockwise_length = LineString(clockwise).length if len(clockwise) > 1 else 0.0
    reverse_length = LineString(reverse_clockwise).length if len(reverse_clockwise) > 1 else 0.0
    return clockwise if clockwise_length <= reverse_length else reverse_clockwise


def append_phase(master, audit, points, phase, kind, lookaheads=None, source_prefix=None):
    if not points:
        return
    if master and math.dist(master[-1]["xy"], points[0]) < 0.01:
        points = points[1:]
        if lookaheads is not None:
            lookaheads = lookaheads[1:]
    for local_index, point in enumerate(points):
        lookahead = NORMAL_LOOKAHEAD_M if lookaheads is None else lookaheads[local_index]
        source_id = f"{source_prefix or phase}:{local_index + 1}"
        row = {
            "xy": point,
            "phase": phase,
            "kind": kind,
            "lookahead": lookahead,
            "speed": SPEED_MPS,
            "source_id": source_id,
        }
        master.append(row)
        audit.append(row)


def mission_rows(master, frame):
    result = []
    for index, row in enumerate(master):
        if index + 1 < len(master):
            target = master[index + 1]["xy"]
        else:
            target = row["xy"]
        if target == row["xy"] and index:
            target = row["xy"]
            source = master[index - 1]["xy"]
        else:
            source = row["xy"]
        yaw = math.atan2(target[1] - source[1], target[0] - source[0])
        lat, lon = frame.ll(*row["xy"])
        result.append({**row, "lat": lat, "lon": lon, "yaw": yaw})
    return result


def main():
    GENERATED.mkdir(parents=True, exist_ok=True)
    frame = load_frame()
    rings, tight = read_review_paths()
    routes = load_routes(frame)
    routes = {key: resample_line(value) for key, value in routes.items()}

    master = []
    audit = []
    connectors = []

    def add_connector(a, b, phase):
        points = straight(a, b)
        connectors.append({"phase": phase, "length_m": LineString(points).length})
        append_phase(
            master,
            audit,
            points,
            phase,
            "access_connector",
            source_prefix=f"C-{phase}",
        )

    def add_ring(area_id, phase):
        lookaheads = [TIGHT_LOOKAHEAD_M if index in tight[area_id] else NORMAL_LOOKAHEAD_M for index in range(len(rings[area_id]))]
        append_phase(master, audit, rings[area_id], phase, "outer_perimeter", lookaheads, area_id)

    append_phase(master, audit, routes["ROUTE-BASE-TO-BACKYARD"], "base_to_backyard", "recorded_transition", source_prefix="R1")
    add_connector(master[-1]["xy"], rings[BACKYARD][0], "enter_backyard_perimeter")
    add_ring(BACKYARD, "backyard_outer_perimeter")

    outbound = routes["ROUTE-BACKYARD-TO-FRONT"]
    exit_index = nearest_index(rings[BACKYARD], outbound[0])
    arc = shortest_ring_arc(rings[BACKYARD], 0, exit_index)
    append_phase(master, audit, arc, "backyard_boundary_exit_arc", "outer_perimeter", source_prefix="BX")
    add_connector(master[-1]["xy"], outbound[0], "backyard_to_front_access")
    append_phase(master, audit, outbound, "backyard_to_front", "recorded_transition", source_prefix="R2")

    add_connector(master[-1]["xy"], rings[FRONT][0], "enter_front_perimeter")
    add_ring(FRONT, "front_outer_perimeter")
    outbound = routes["ROUTE-FRONT-TO-OVERROAD"]
    exit_index = nearest_index(rings[FRONT], outbound[0])
    arc = shortest_ring_arc(rings[FRONT], 0, exit_index)
    append_phase(master, audit, arc, "front_boundary_outbound_arc", "outer_perimeter", source_prefix="FX1")
    add_connector(master[-1]["xy"], outbound[0], "front_to_overroad_access")
    append_phase(master, audit, outbound, "front_to_overroad", "recorded_transition", source_prefix="R3")

    add_connector(master[-1]["xy"], rings[OVERROAD][0], "enter_overroad_perimeter")
    add_ring(OVERROAD, "overroad_outer_perimeter")
    outbound = routes["ROUTE-OVERROAD-TO-FRONT"]
    exit_index = nearest_index(rings[OVERROAD], outbound[0])
    arc = shortest_ring_arc(rings[OVERROAD], 0, exit_index)
    append_phase(master, audit, arc, "overroad_boundary_exit_arc", "outer_perimeter", source_prefix="OX")
    add_connector(master[-1]["xy"], outbound[0], "overroad_to_front_access")
    append_phase(master, audit, outbound, "overroad_to_front", "recorded_transition", source_prefix="R4")

    inbound_index = nearest_index(rings[FRONT], master[-1]["xy"])
    add_connector(master[-1]["xy"], rings[FRONT][inbound_index], "reenter_front_for_return")
    return_route = routes["ROUTE-FRONT-TO-BASE"]
    outbound_index = nearest_index(rings[FRONT], return_route[0])
    arc = shortest_ring_arc(rings[FRONT], inbound_index, outbound_index)
    append_phase(master, audit, arc, "front_boundary_return_arc", "outer_perimeter", source_prefix="FX2")
    add_connector(master[-1]["xy"], return_route[0], "front_to_base_access")
    append_phase(master, audit, return_route, "front_to_base", "recorded_transition", source_prefix="R5")

    rows = mission_rows(master, frame)
    MISSION.write_text("".join(
        f"{row['lat']:.9f} {row['lon']:.9f} {row['yaw']:.6f} {row['lookahead']:.2f} {row['speed']:.2f}\n"
        for row in rows
    ), encoding="ascii", newline="\n")

    with AUDIT.open("w", newline="", encoding="utf-8-sig") as handle:
        columns = ["waypoint", "source_id", "phase", "kind", "east_m", "north_m", "lat", "lon", "yaw_rad", "lookahead_m", "speed_mps"]
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for waypoint, row in enumerate(rows, 1):
            writer.writerow({
                "waypoint": waypoint,
                "source_id": row["source_id"],
                "phase": row["phase"],
                "kind": row["kind"],
                "east_m": f"{row['xy'][0]:.3f}",
                "north_m": f"{row['xy'][1]:.3f}",
                "lat": f"{row['lat']:.9f}",
                "lon": f"{row['lon']:.9f}",
                "yaw_rad": f"{row['yaw']:.6f}",
                "lookahead_m": f"{row['lookahead']:.2f}",
                "speed_mps": f"{row['speed']:.2f}",
            })

    mission_sha = hashlib.sha256(MISSION.read_bytes().replace(b"\r\n", b"\n")).hexdigest()
    audit_sha = hashlib.sha256(AUDIT.read_bytes().replace(b"\r\n", b"\n")).hexdigest()
    maximum_gap = max(math.dist(a["xy"], b["xy"]) for a, b in zip(rows, rows[1:]))
    path_length = sum(math.dist(a["xy"], b["xy"]) for a, b in zip(rows, rows[1:]))
    phase_counts = {}
    for row in rows:
        phase_counts[row["phase"]] = phase_counts.get(row["phase"], 0) + 1

    report = {
        "status": "SUPERVISED_BLADES_OFF_FIELD_TEST",
        "mission_sha256": mission_sha,
        "audit_sha256": audit_sha,
        "waypoints": len(rows),
        "path_length_m": round(path_length, 3),
        "nominal_motion_time_minutes_at_1mps": round(path_length / 60.0, 2),
        "maximum_waypoint_gap_m": round(maximum_gap, 3),
        "speed_mps": SPEED_MPS,
        "normal_lookahead_m": NORMAL_LOOKAHEAD_M,
        "tight_join_lookahead_m": TIGHT_LOOKAHEAD_M,
        "tight_join_zone_m": TIGHT_ZONE_M,
        "turn_softening_radius_m": 2.0,
        "route_direction": "clockwise",
        "deck_edge_rule": "left deck edge follows outer perimeter",
        "phases": phase_counts,
        "connectors": connectors,
        "source_review_report": str((REVIEW / "62_Collins_outer_perimeter_1mps_report.json").relative_to(REPO)).replace("\\", "/"),
        "limitations": [
            "Mower deck must remain disengaged for this first supervised perimeter validation.",
            "Recorded between-area routes are reused; short access connectors are newly generated straight joins and require close supervision.",
            "The perimeter geometry is conservatively opened with a 2.0 m turn-softening radius; this deliberately stands farther inside narrow or sharply notched areas.",
            "Keep the handheld available and select Pause immediately for excessive cross-track error or an unsuitable connector.",
        ],
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    fig, axis = plt.subplots(figsize=(13, 10), dpi=170)
    colors = {"outer_perimeter": "#1565c0", "recorded_transition": "#00897b", "access_connector": "#d97706"}
    start = 0
    while start < len(rows):
        kind = rows[start]["kind"]
        end = start + 1
        while end < len(rows) and rows[end]["phase"] == rows[start]["phase"]:
            end += 1
        points = [row["xy"] for row in rows[start:end]]
        if len(points) > 1:
            x, y = zip(*points)
            axis.plot(x, y, color=colors[kind], linewidth=1.5)
        start = end
    tight_points = [row["xy"] for row in rows if row["lookahead"] == TIGHT_LOOKAHEAD_M]
    if tight_points:
        x, y = zip(*tight_points)
        axis.scatter(x, y, s=3, color="#dc2626", label="1.0 m tight-join lookahead")
    axis.scatter([rows[0]["xy"][0]], [rows[0]["xy"][1]], s=70, marker="*", color="#111827", label="Voice-guidance start")
    axis.plot([], [], color=colors["outer_perimeter"], label="Clockwise perimeter")
    axis.plot([], [], color=colors["recorded_transition"], label="Recorded between-area route")
    axis.plot([], [], color=colors["access_connector"], label="New access connector")
    axis.set_title("62 Collins supervised outer-perimeter field mission — BLADES OFF")
    axis.set_xlabel("Local east (m)")
    axis.set_ylabel("Local north (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, linewidth=0.3, alpha=0.3)
    axis.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(PREVIEW, bbox_inches="tight")
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
