#!/usr/bin/env python3
"""Build a review-only rings-only master mission from the labeled 2026-09-15 drive."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString, Point, Polygon


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
REVIEW = HERE.parents[1] / "runs" / "20260915_133923" / "boundary_review"
RAW_CAPTURE = REVIEW.parent / "field_test_20260915_133923.csv"
OUT = HERE / "generated_rings_only"
PLANNER = REPO / "tractor_rpi" / "pure-pursuit" / "mission_planning"
sys.path.insert(0, str(PLANNER))

from site_planner_common_20260724 import (  # noqa: E402
    LocalFrame,
    MissionPoint,
    deduplicate_points,
    densify_polyline,
    distance,
    segment_yaw,
    signed_area,
    write_mission,
)
from site_coverage_planner_20260724 import (  # noqa: E402
    append_connector,
    append_piece,
    make_headland_paths,
    path_end_pose,
    path_start_pose,
    ring_variants,
    valid_connector,
)


CRUISE_SPEED_MPS = 1.0
TIGHT_TURN_SPEED_MPS = 0.5
LOOKAHEAD_M = 2.0
WAYPOINT_SPACING_M = 0.50
LANE_SPACING_M = 0.9652  # 38-inch centerline spacing used by prior missions
TURN_RADIUS_M = 1.63     # review value: previously measured weaker right-turn radius
MAX_CONNECTOR_NET_TURN_DEG = 300.0
BOUNDARY_OUTSET_M = 0.381
POLE_EXTRA_CLEARANCE_M = 0.6096  # 24 inches
POLE_NUMERICAL_MARGIN_M = 0.02
POLE_LOOP_START = 13
POLE_LOOP_END = 63

FIELDS = (
    {"name": "main_backyard", "segments": (3, 4), "incoming": (2,), "outgoing": (5,)},
    {"name": "garden_right", "segments": (6,), "incoming": (5,), "outgoing": (7,)},
    {"name": "garden_left", "segments": (8,), "incoming": (7,), "outgoing": (9,)},
    {"name": "front_yard", "segments": (10,), "incoming": (9,), "outgoing": (11,)},
    {"name": "over_the_road", "segments": (13, 15), "incoming": (12,), "outgoing": (16,), "obstacle": True},
)


def load_segment(number):
    with (REVIEW / f"segment_{number:02d}_candidate_path.csv").open(
        newline="", encoding="utf-8-sig"
    ) as handle:
        return [
            {
                "lat": float(row["lat"]),
                "lon": float(row["lon"]),
                "heading_deg": float(row["heading_deg"]),
                "elapsed_sec": float(row["elapsed_sec"]),
                "segment": number,
            }
            for row in csv.DictReader(handle)
        ]


def rows_xy(frame, rows):
    return [frame.to_xy(row["lat"], row["lon"]) for row in rows]


def polygon_parts(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == "Polygon":
        return [geometry]
    if geometry.geom_type == "MultiPolygon":
        return list(geometry.geoms)
    return []


def largest_polygon(geometry):
    parts = polygon_parts(geometry)
    if not parts:
        raise ValueError(f"Expected polygon geometry, got {geometry.geom_type}")
    return max(parts, key=lambda item: item.area)


def build_boundary(frame, segments, numbers):
    points = deduplicate_points(
        [point for number in numbers for point in rows_xy(frame, segments[number])],
        0.02,
    )
    polygon = Polygon(points)
    if not polygon.is_valid:
        polygon = largest_polygon(polygon.buffer(0))
    return polygon, points


def feasible_headlands(drive_area, outer_boundary):
    accepted = None
    failure = None
    for passes in range(1, 21):
        try:
            candidate = make_headland_paths(
                drive_area,
                passes,
                LANE_SPACING_M,
                TURN_RADIUS_M,
                outer_boundary=outer_boundary,
            )
        except ValueError as exc:
            failure = str(exc)
            break
        accepted = candidate
    if not accepted:
        raise ValueError("No feasible ring passes")
    return accepted, failure


def zero_or_dubins(start_pose, end_pose, drive_area):
    gap = distance(start_pose[:2], end_pose[:2])
    heading_error = abs((end_pose[2] - start_pose[2] + math.pi) % (2 * math.pi) - math.pi)
    if gap <= 0.55 and heading_error <= math.radians(45):
        return {
            "mode": "recorded-tangent-join",
            "length_m": gap,
            "points": [start_pose, end_pose],
        }
    connector = valid_connector(
        start_pose, end_pose, drive_area, TURN_RADIUS_M, WAYPOINT_SPACING_M
    )
    if connector is None:
        return None
    yaws = [float(point[2]) for point in connector["points"]]
    net_turn = 0.0
    for first, second in zip(yaws, yaws[1:]):
        net_turn += (second - first + math.pi) % (2.0 * math.pi) - math.pi
    if abs(math.degrees(net_turn)) >= MAX_CONNECTOR_NET_TURN_DEG:
        return None
    return connector


def mission_speeds(route):
    """Use 0.5 m/s for reviewed tight maneuvers and 1.0 m/s elsewhere.

    Garden-left contains the three close/circular features labeled A-C in the
    review image, so that compact field stays slow throughout. All planned
    connectors are also slow, as are the innermost ring in every other field.
    """
    innermost_ring = {}
    for item in route:
        if item["kind"] == "ring":
            innermost_ring[item["field"]] = max(
                innermost_ring.get(item["field"], 0), int(item["ring"])
            )
    speeds = []
    for item in route:
        tight = (
            item["kind"] == "planned_connector"
            or item["field"] == "garden_left"
            or (
                item["kind"] == "ring"
                and int(item["ring"]) == innermost_ring[item["field"]]
            )
        )
        speeds.append(TIGHT_TURN_SPEED_MPS if tight else CRUISE_SPEED_MPS)
    return speeds


def plan_ring_chain(headlands, incoming_pose, outgoing_pose, clockwise, drive_area):
    states = []
    for variant in ring_variants(headlands[0]["points"], clockwise, maximum_variants=24):
        connector = zero_or_dubins(incoming_pose, variant["start_pose"], drive_area)
        if connector is None:
            continue
        states.append({
            **variant,
            "label": headlands[0]["label"],
            "headland_pass": headlands[0]["pass"],
            "cost": float(connector["length_m"]),
            "previous": None,
            "incoming": connector,
        })
    if not states:
        raise ValueError("No contained connector can enter the first ring")

    for ring_info in headlands[1:]:
        next_states = []
        for variant in ring_variants(ring_info["points"], clockwise, maximum_variants=24):
            best = None
            for previous in states:
                connector = zero_or_dubins(
                    previous["end_pose"], variant["start_pose"], drive_area
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
                next_states.append(best)
        if not next_states:
            raise ValueError(f"No contained connector can reach {ring_info['label']}")
        states = next_states

    finals = []
    for state in states:
        connector = zero_or_dubins(state["end_pose"], outgoing_pose, drive_area)
        if connector is not None:
            finals.append((state["cost"] + float(connector["length_m"]), state, connector))
    if not finals:
        raise ValueError("No contained connector can leave the last ring")
    _cost, state, outgoing = min(finals, key=lambda item: item[0])
    selected = []
    while state is not None:
        selected.append(state)
        state = state["previous"]
    selected.reverse()
    return selected, outgoing


def find_boundary_approach(
    boundary_points, headlands, outgoing_pose, recorded_clockwise, drive_area
):
    """Use a driven-boundary prefix when the field entrance cannot turn inward."""
    approach = densify_polyline(boundary_points, 2.0)
    for end_index in range(1, len(approach)):
        incoming_pose = path_end_pose(approach[:end_index + 1])
        for clockwise in (recorded_clockwise, not recorded_clockwise):
            first_variants = ring_variants(
                headlands[0]["points"], clockwise, maximum_variants=24
            )
            if not any(
                zero_or_dubins(incoming_pose, variant["start_pose"], drive_area)
                is not None
                for variant in first_variants
            ):
                continue
            try:
                selected, outgoing = plan_ring_chain(
                    headlands, incoming_pose, outgoing_pose, clockwise, drive_area
                )
            except ValueError:
                continue
            return approach[:end_index + 1], selected, outgoing, clockwise
    return None


def find_boundary_exit(
    boundary_points, headlands, incoming_pose, recorded_clockwise, drive_area
):
    """Join a ring chain to a safe driven-boundary suffix leading to the exit."""
    boundary = densify_polyline(boundary_points, 1.0)
    safe_area = drive_area.buffer(0.03)
    for start_index in range(len(boundary) - 2, 0, -1):
        suffix = boundary[start_index:]
        if len(suffix) < 2 or not safe_area.covers(LineString(suffix)):
            continue
        outgoing_pose = path_start_pose(suffix)
        for clockwise in (recorded_clockwise, not recorded_clockwise):
            try:
                selected, outgoing = plan_ring_chain(
                    headlands, incoming_pose, outgoing_pose, clockwise, drive_area
                )
            except ValueError:
                continue
            return selected, outgoing, suffix, clockwise
    return None


def recorded_transition(frame, segments, numbers):
    points = deduplicate_points(
        [point for number in numbers for point in rows_xy(frame, segments[number])],
        0.03,
    )
    return densify_polyline(points, WAYPOINT_SPACING_M)


def build_safe_pole_bypass(frame, segments, pole_polygon, pole_exclusion, drive_area):
    recorded = recorded_transition(frame, segments, (13, 14, 15))
    protected_interior = pole_exclusion.buffer(-0.005)
    safe_geometry = LineString(recorded).difference(pole_exclusion)
    parts = (
        [safe_geometry]
        if safe_geometry.geom_type == "LineString"
        else [part for part in safe_geometry.geoms if part.geom_type == "LineString"]
    )
    if len(parts) < 2:
        raise ValueError("Expanded pole exclusion did not split the recorded route")

    def orient_from(line, point):
        coords = list(line.coords)
        return coords if distance(coords[0], point) <= distance(coords[-1], point) else list(reversed(coords))

    prefix_line = min(parts, key=lambda line: line.distance(Point(recorded[0])))
    suffix_line = min(parts, key=lambda line: line.distance(Point(recorded[-1])))
    prefix = orient_from(prefix_line, recorded[0])
    suffix = orient_from(suffix_line, recorded[-1])
    suffix.reverse()  # orient from the exclusion back toward the recorded end
    entry, exit_point = prefix[-1], suffix[0]
    ring = LineString(pole_exclusion.exterior.coords)
    circumference = ring.length
    entry_s = ring.project(Point(entry))
    exit_s = ring.project(Point(exit_point))

    candidates = []
    for direction in (1.0, -1.0):
        arc_length = (
            (exit_s - entry_s) % circumference
            if direction > 0 else (entry_s - exit_s) % circumference
        )
        count = max(2, math.ceil(arc_length / 0.05) + 1)
        arc = [
            ring.interpolate((entry_s + direction * arc_length * i / (count - 1)) % circumference).coords[0]
            for i in range(count)
        ]
        candidate = [*prefix, *arc, *suffix]
        line = LineString(candidate)
        if drive_area.buffer(0.03).covers(line) and not line.intersects(protected_interior):
            candidates.append(candidate)
    if not candidates:
        raise ValueError("Neither expanded pole-clearance arc stays inside the over-road drive area")
    selected = min(candidates, key=lambda points: LineString(points).length)
    achieved_clearance = LineString(selected).distance(pole_polygon)
    if achieved_clearance + 0.005 < POLE_EXTRA_CLEARANCE_M:
        raise ValueError(
            "Generated pole bypass does not preserve the requested 24-inch "
            f"clearance (achieved {achieved_clearance:.3f} m)"
        )
    return selected


def measure_pole_turn(frame, segment_rows):
    """Measure the exact Segment 14 subset used to define the pole loop."""
    selected_rows = segment_rows[POLE_LOOP_START:POLE_LOOP_END + 1]
    xy = np.asarray(rows_xy(frame, selected_rows), dtype=float)
    x = xy[:, 0]
    y = xy[:, 1]
    design = np.column_stack((2.0 * x, 2.0 * y, np.ones(len(x))))
    cx, cy, _constant = np.linalg.lstsq(
        design, x * x + y * y, rcond=None
    )[0]
    radial = np.hypot(x - cx, y - cy)
    headings = np.unwrap(
        np.radians([row["heading_deg"] for row in selected_rows])
    )
    path_length = float(np.hypot(np.diff(x), np.diff(y)).sum())
    heading_change = float(headings[-1] - headings[0])

    start_elapsed = float(selected_rows[0]["elapsed_sec"])
    end_elapsed = float(selected_rows[-1]["elapsed_sec"])
    raw_rows = []
    with RAW_CAPTURE.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            elapsed = float(row["elapsed_sec"])
            if start_elapsed <= elapsed <= end_elapsed:
                raw_rows.append(row)
    setpoints = [float(row["steer_setpoint"]) for row in raw_rows]
    actuals = [float(row["steer_current"]) for row in raw_rows]
    return {
        "candidate_sequence_start": POLE_LOOP_START + 1,
        "candidate_sequence_end": POLE_LOOP_END + 1,
        "candidate_points": len(selected_rows),
        "closure_gap_m": distance(tuple(xy[0]), tuple(xy[-1])),
        "path_length_m": path_length,
        "signed_heading_change_deg": math.degrees(heading_change),
        "turn_direction_from_heading": "left" if heading_change < 0.0 else "right",
        "circle_fit_radius_m": float(radial.mean()),
        "circle_fit_radial_stddev_m": float(radial.std()),
        "arc_length_over_heading_change_radius_m": path_length / abs(heading_change),
        "raw_telemetry_samples": len(raw_rows),
        "steer_setpoint_median": float(np.median(setpoints)),
        "steer_actual_median": float(np.median(actuals)),
        "steer_setpoint_at_operating_left_885_fraction": (
            sum(value == 885.0 for value in setpoints) / len(setpoints)
        ),
        "firmware_limit_interpretation": (
            "teensy_main_20260914 defines 885 as operating hard left; "
            "the pole measurement therefore validates the left-turn radius, "
            "not the weaker right-turn radius"
        ),
    }


def append_labeled(route, points, kind, phase, field=None, ring=None):
    dense = densify_polyline(points, WAYPOINT_SPACING_M)
    # A phase boundary can contain the short gap between two consecutive
    # recorded segments.  The mission already traverses that straight line;
    # explicitly densify it so the waypoint-spacing guarantee also holds
    # across phase boundaries.
    if route and dense and distance(route[-1]["xy"], dense[0]) > WAYPOINT_SPACING_M:
        join = densify_polyline([route[-1]["xy"], dense[0]], WAYPOINT_SPACING_M)
        dense = [*join[1:-1], *dense]
    for point in dense:
        if route and distance(route[-1]["xy"], point) < 0.01:
            continue
        route.append({
            "xy": point,
            "kind": kind,
            "phase": phase,
            "field": field or "",
            "ring": ring or "",
        })


def append_planned_connector(route, connector, phase, field):
    points = [(x, y) for x, y, _yaw in connector["points"]]
    append_labeled(route, points, "planned_connector", phase, field)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    needed = set(range(2, 18))
    segments = {number: load_segment(number) for number in needed}
    frame = LocalFrame(segments[2][0]["lat"], segments[2][0]["lon"])

    pole_loop_xy = rows_xy(frame, segments[14][POLE_LOOP_START:POLE_LOOP_END + 1])
    pole_polygon = largest_polygon(Polygon(pole_loop_xy).buffer(0))
    pole_exclusion = pole_polygon.buffer(
        POLE_EXTRA_CLEARANCE_M + POLE_NUMERICAL_MARGIN_M,
        join_style="round",
    )
    pole_turn_measurement = measure_pole_turn(frame, segments[14])

    planned = {}
    for spec in FIELDS:
        boundary, boundary_points = build_boundary(frame, segments, spec["segments"])
        drive_area = boundary.buffer(BOUNDARY_OUTSET_M, join_style="round")
        outer_boundary = boundary
        if spec.get("obstacle"):
            drive_area = largest_polygon(drive_area.difference(pole_exclusion))
            # The expanded pole clearance cuts a safe notch into the recorded
            # outer boundary, so ring 1 follows that safe exterior instead.
            outer_boundary = largest_polygon(boundary.difference(pole_exclusion))
        all_headlands, first_failure = feasible_headlands(drive_area, outer_boundary)
        # The user requested inner rings only. The manually driven perimeter is
        # retained as the boundary source, but pass 1 is not part of coverage.
        headlands = [item for item in all_headlands if int(item["pass"]) > 1]
        if not headlands:
            raise ValueError(f"{spec['name']}: boundary has no feasible inner ring")
        incoming = recorded_transition(frame, segments, spec["incoming"])
        outgoing = recorded_transition(frame, segments, spec["outgoing"])
        recorded_clockwise = signed_area(boundary_points) < 0.0
        attempts = []
        selected = exit_connector = None
        clockwise = recorded_clockwise
        for candidate_direction in (recorded_clockwise, not recorded_clockwise):
            try:
                candidate_selected, candidate_exit = plan_ring_chain(
                    headlands,
                    path_end_pose(incoming),
                    path_start_pose(outgoing),
                    candidate_direction,
                    drive_area,
                )
            except ValueError as exc:
                attempts.append(str(exc))
                continue
            selected, exit_connector = candidate_selected, candidate_exit
            clockwise = candidate_direction
            break
        boundary_approach = []
        boundary_exit = []
        coverage_blocked = ""
        safe_obstacle_bypass = []
        if selected is None or exit_connector is None:
            fallback = find_boundary_approach(
                boundary_points,
                headlands,
                path_start_pose(outgoing),
                recorded_clockwise,
                drive_area,
            )
            if fallback is not None:
                boundary_approach, selected, exit_connector, clockwise = fallback
            else:
                exit_fallback = find_boundary_exit(
                    boundary_points,
                    headlands,
                    path_end_pose(incoming),
                    recorded_clockwise,
                    drive_area,
                )
                if exit_fallback is None:
                    if spec.get("obstacle"):
                        selected = []
                        exit_connector = None
                        coverage_blocked = (
                            "No contained 1.63 m-radius entry/exit chain exists for "
                            "the inner ring after applying the 24-inch pole clearance."
                        )
                        safe_obstacle_bypass = build_safe_pole_bypass(
                            frame, segments, pole_polygon, pole_exclusion, drive_area
                        )
                    else:
                        raise ValueError(f"{spec['name']}: {'; '.join(attempts)}")
                else:
                    selected, exit_connector, boundary_exit, clockwise = exit_fallback
        planned[spec["name"]] = {
            "spec": spec,
            "boundary": boundary,
            "drive_area": drive_area,
            "headlands": headlands,
            "selected": selected,
            "outgoing": exit_connector,
            "first_failure": first_failure,
            "clockwise": clockwise,
            "boundary_approach": boundary_approach,
            "boundary_exit": boundary_exit,
            "coverage_blocked": coverage_blocked,
            "safe_obstacle_bypass": safe_obstacle_bypass,
        }

    route = []
    append_labeled(route, recorded_transition(frame, segments, (2,)), "recorded_transition", "transition_02")
    transition_after = {
        "main_backyard": (5,),
        "garden_right": (7,),
        "garden_left": (9,),
        "front_yard": (11, 12),
        "over_the_road": (16, 17),
    }
    for spec in FIELDS:
        name = spec["name"]
        item = planned[name]
        if item["coverage_blocked"]:
            append_labeled(
                route,
                item["safe_obstacle_bypass"],
                "recorded_obstacle_bypass",
                "over_the_road_24in_clearance_bypass",
                name,
            )
            append_labeled(
                route,
                recorded_transition(frame, segments, transition_after[name]),
                "recorded_transition",
                "transition_" + "_".join(f"{number:02d}" for number in transition_after[name]),
            )
            continue
        if item["boundary_approach"]:
            append_labeled(
                route,
                item["boundary_approach"],
                "recorded_boundary_approach",
                f"{name}_recorded_boundary_approach",
                name,
            )
        for ring_index, selected in enumerate(item["selected"], 1):
            append_planned_connector(
                route, selected["incoming"], f"{name}_entry_or_ring_connector_{ring_index}", name
            )
            append_labeled(
                route,
                selected["ring"],
                "ring",
                f"{name}_ring_{ring_index}",
                name,
                ring_index,
            )
        append_planned_connector(route, item["outgoing"], f"{name}_exit_connector", name)
        if item["boundary_exit"]:
            append_labeled(
                route,
                item["boundary_exit"],
                "recorded_boundary_exit",
                f"{name}_recorded_boundary_exit",
                name,
            )
        append_labeled(
            route,
            recorded_transition(frame, segments, transition_after[name]),
            "recorded_transition",
            "transition_" + "_".join(f"{number:02d}" for number in transition_after[name]),
        )

    xy = [item["xy"] for item in route]
    gaps = [distance(a, b) for a, b in zip(xy, xy[1:])]
    speeds = mission_speeds(route)
    mission = []
    audit = []
    for index, (item, speed_mps) in enumerate(zip(route, speeds)):
        yaw = segment_yaw(xy[index], xy[index + 1]) if index < len(xy) - 1 else segment_yaw(xy[-2], xy[-1])
        lat, lon = frame.to_latlon(*item["xy"])
        mission.append(MissionPoint(lat, lon, yaw, LOOKAHEAD_M, speed_mps))
        audit.append({
            "waypoint": index + 1,
            "phase": item["phase"],
            "kind": item["kind"],
            "field": item["field"],
            "ring": item["ring"],
            "lat": f"{lat:.9f}",
            "lon": f"{lon:.9f}",
            "east_m": f"{item['xy'][0]:.3f}",
            "north_m": f"{item['xy'][1]:.3f}",
            "yaw_rad": f"{yaw:.6f}",
            "lookahead_m": f"{LOOKAHEAD_M:.2f}",
            "speed_mps": f"{speed_mps:.2f}",
        })

    mission_path = OUT / "62_Collins_rings_only_master_1mps_PARTIAL_REVIEW_ONLY_20260915.txt"
    write_mission(mission_path, mission)
    audit_path = OUT / "62_Collins_rings_only_master_1mps_audit_20260915.csv"
    with audit_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=audit[0].keys())
        writer.writeheader()
        writer.writerows(audit)

    route_line = LineString(xy)
    over_rows = [row for row in route if row["field"] == "over_the_road"]
    over_line = LineString([row["xy"] for row in over_rows])
    report = {
        "status": "SUPERVISED_BLADES_OFF_FIELD_TEST",
        "coverage_complete": False,
        "coverage_blocked_fields": [
            name for name, item in planned.items() if item["coverage_blocked"]
        ],
        "coverage_mode": "rings_only",
        "stripes_enabled": False,
        "cruise_speed_mps": CRUISE_SPEED_MPS,
        "tight_turn_speed_mps": TIGHT_TURN_SPEED_MPS,
        "tight_turn_speed_policy": (
            "0.5 m/s for every planned connector, all garden-left coverage "
            "(review features A-C), and each other field's innermost ring "
            "(including review features D-E); 1.0 m/s elsewhere."
        ),
        "maximum_connector_net_turn_deg": MAX_CONNECTOR_NET_TURN_DEG,
        "lookahead_m": LOOKAHEAD_M,
        "lane_spacing_m": LANE_SPACING_M,
        "lane_spacing_in": LANE_SPACING_M / 0.0254,
        "turn_radius_m": TURN_RADIUS_M,
        "turn_radius_note": "The 1.63 m value equals the previously measured weaker right-turn radius and has no geometric margin. Near-360-degree planned connectors are rejected, and reviewed tight features run at 0.5 m/s for the supervised blades-off test. The approximately 1.12 m pole circle was a hard-left maneuver, so it does not justify reducing the right-turn value. Stripe U-turns remain deferred.",
        "pole_turn_measurement": pole_turn_measurement,
        "pole_extra_clearance_m": POLE_EXTRA_CLEARANCE_M,
        "pole_extra_clearance_in": 24.0,
        "pole_planning_numerical_margin_m": POLE_NUMERICAL_MARGIN_M,
        "waypoints": len(mission),
        "route_length_m": route_line.length,
        "estimated_runtime_minutes": sum(
            gap / speeds[index] for index, gap in enumerate(gaps)
        ) / 60.0,
        "maximum_waypoint_gap_m": max(gaps),
        "minimum_over_road_route_distance_to_recorded_pole_loop_m": over_line.distance(pole_polygon),
        "over_road_route_enters_expanded_pole_exclusion_interior": over_line.intersects(pole_exclusion.buffer(-0.01)),
        "fields": {
            name: {
                "ring_count": len(item["selected"]),
                "geometrically_available_inner_ring_count": len(item["headlands"]),
                "manual_boundary_pass_included": bool(item["safe_obstacle_bypass"]),
                "manual_boundary_pass_note": (
                    "Segments 13 and 15 are retained; the Segment 14 pole portion "
                    "is moved outward to the expanded pole exclusion."
                    if item["safe_obstacle_bypass"] else ""
                ),
                "clockwise": item["clockwise"],
                "boundary_area_m2": item["boundary"].area,
                "drive_area_m2": item["drive_area"].area,
                "first_rejected_ring_reason": item["first_failure"],
                "coverage_blocked_reason": item["coverage_blocked"],
                "recorded_boundary_approach_m": (
                    LineString(item["boundary_approach"]).length
                    if item["boundary_approach"] else 0.0
                ),
                "recorded_boundary_exit_m": (
                    LineString(item["boundary_exit"]).length
                    if item["boundary_exit"] else 0.0
                ),
            }
            for name, item in planned.items()
        },
        "mission_sha256": hashlib.sha256(mission_path.read_bytes()).hexdigest(),
    }
    report_path = OUT / "62_Collins_rings_only_master_1mps_report_20260915.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    fig, axis = plt.subplots(figsize=(12, 10), dpi=170)
    colors = {
        "main_backyard": "#1565c0",
        "garden_right": "#2e7d32",
        "garden_left": "#6a1b9a",
        "front_yard": "#00838f",
        "over_the_road": "#ad5c00",
    }
    for name, item in planned.items():
        bx, by = item["boundary"].exterior.xy
        axis.plot(bx, by, color=colors[name], linewidth=0.8, linestyle=":", alpha=0.8)
    phase_groups = {}
    for row in route:
        phase_groups.setdefault(row["phase"], []).append(row["xy"])
    labeled = set()
    for phase, points in phase_groups.items():
        field = next((row["field"] for row in route if row["phase"] == phase), "")
        if field:
            color = colors[field]
            label = field.replace("_", " ") if field not in labeled else None
            if field == "over_the_road" and planned[field]["coverage_blocked"] and label:
                label = "over the road boundary only"
            labeled.add(field)
            width, style = (1.8, "-") if "ring" in phase else (1.1, "--")
        else:
            color, label, width, style = "#6f7782", None, 1.0, "--"
        if len(points) >= 2:
            x, y = zip(*points)
            axis.plot(x, y, color=color, linewidth=width, linestyle=style, label=label)
    ex, ey = pole_exclusion.exterior.xy
    axis.fill(ex, ey, color="#ef5350", alpha=0.35, label="Pole loop + 24 in clearance")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.25)
    axis.set_xlabel("East of Segment 2 start (m)")
    axis.set_ylabel("North of Segment 2 start (m)")
    axis.set_title(
        f"62 Collins partial rings-only master — {report['route_length_m']:.0f} m, "
        f"{report['estimated_runtime_minutes']:.1f} min, 1.0 m/s cruise / 0.5 m/s tight turns"
    )
    axis.legend(loc="best", fontsize=8)
    fig.tight_layout()
    preview_path = OUT / "62_Collins_rings_only_master_1mps_REVIEW_20260915.png"
    save_path = str(preview_path.resolve())
    if os.name == "nt":
        save_path = "\\\\?\\" + save_path
    fig.savefig(save_path)
    plt.close(fig)

    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
