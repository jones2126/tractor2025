#!/usr/bin/env python3
"""Build the rev003 Pure Pursuit mission candidate with six explicit turns.

The approved route order is retained. The six cross-segment heading jumps in
rev002 are replaced by forward-only Dubins transitions at the 1.4 m planning
radius. Output remains DRAFT - UNTESTED IN THE FIELD and does not enable the
field launcher.
"""
from __future__ import annotations

import csv
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
from shapely.geometry import LineString, Point, Polygon, mapping, shape
from shapely.ops import substring, transform, unary_union

STATUS = "DRAFT - UNTESTED IN THE FIELD"
CUT_WIDTH_M = 0.9652
DECK_WIDTH_M = 1.0668
CHUTE_EXTENSION_M = 0.127
TURN_RADIUS_M = 1.4
SPEED_MPS = 1.0
WAYPOINT_SPACING_M = 0.20

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
SITE = ROOT / "field_testing/sites/62_Collins_multi_boundary_20260915"
MOWABLE = SITE / (
    "site_inventory/assets/AREA-OVER-ROAD/revisions/"
    "rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_mowable_area.geojson"
)
SOURCE = HERE / "checkpoint_4_detailed_draft_rev002.geojson"
HELPERS = HERE / "build_checkpoint_4_detailed_draft.py"
OUT_GEOJSON = HERE / "checkpoint_6_controller_route_rev003.geojson"
OUT_VALIDATION = HERE / "checkpoint_6_controller_validation_rev003.json"
OUT_MISSION = HERE / "generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024.txt"
OUT_AUDIT = HERE / "generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024_audit.csv"

spec = importlib.util.spec_from_file_location("cp4_helpers", HELPERS)
helpers = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(helpers)
sample_dubins = helpers.sample_dubins
line_heading = helpers.line_heading
angle_difference = helpers.angle_difference
min_sampled_radius = helpers.min_sampled_radius


def sample_line(line: LineString, spacing: float = WAYPOINT_SPACING_M) -> list[Point]:
    if line.length <= 0:
        raise ValueError("Cannot sample zero-length line")
    distances = list(np.arange(0.0, line.length, spacing))
    if not distances or line.length - distances[-1] > 1e-6:
        distances.append(line.length)
    return [line.interpolate(float(distance)) for distance in distances]


def join_lines(*lines: LineString) -> LineString:
    coords = []
    for line in lines:
        part = list(line.coords)
        if coords and Point(coords[-1]).distance(Point(part[0])) < 0.02:
            part = part[1:]
        coords.extend(part)
    return LineString(coords)


def main() -> None:
    geographic = shape(json.loads(MOWABLE.read_text(encoding="utf-8"))["features"][0]["geometry"])
    ref_lat, ref_lon = geographic.centroid.y, geographic.centroid.x
    east_scale = 111_320.0 * math.cos(math.radians(ref_lat))
    north_scale = 110_540.0

    def project(geometry):
        return transform(lambda lon, lat, z=None: ((lon - ref_lon) * east_scale, (lat - ref_lat) * north_scale), geometry)

    def unproject(geometry):
        return transform(lambda x, y, z=None: (x / east_scale + ref_lon, y / north_scale + ref_lat), geometry)

    area = project(geographic)
    outer = Polygon(area.exterior)
    hole = Polygon(area.interiors[0])
    pole_center = hole.centroid
    source = json.loads(SOURCE.read_text(encoding="utf-8"))
    original = []
    for feature in sorted(source["features"], key=lambda item: item["properties"]["sequence"]):
        original.append({"properties": dict(feature["properties"]), "line": project(shape(feature["geometry"]))})
    by_id = {item["properties"]["segment_id"]: item for item in original}

    def score(line: LineString) -> tuple[float, dict]:
        center_outside = line.difference(outer).length
        center_pole = line.intersection(hole).length
        deck_pole = line.buffer(DECK_WIDTH_M / 2.0, cap_style=2, join_style=1).intersection(hole).area
        conservative_pole = line.buffer(DECK_WIDTH_M / 2.0 + CHUTE_EXTENSION_M, cap_style=2, join_style=1).intersection(hole).area
        envelope_outside = line.buffer(DECK_WIDTH_M / 2.0 + CHUTE_EXTENSION_M, cap_style=2, join_style=1).difference(outer).area
        value = (
            center_pole * 1e9 + conservative_pole * 1e9 + deck_pole * 1e9
            + center_outside * 1e7 + envelope_outside * 5.0 + line.length
        )
        return value, {
            "centerline_outside_m": center_outside,
            "centerline_pole_intersection_m": center_pole,
            "deck_pole_intersection_m2": deck_pole,
            "conservative_pole_intersection_m2": conservative_pole,
            "conservative_envelope_outside_m2": envelope_outside,
        }

    def smooth_obstacle_detour(source_line: LineString) -> tuple[LineString, dict]:
        """Replace a line/circle corner with tangent Dubins-circle-Dubins geometry."""
        start = Point(source_line.coords[0])
        end = Point(source_line.coords[-1])
        baseline = LineString([start.coords[0], end.coords[0]])
        travel_heading = math.atan2(end.y - start.y, end.x - start.x)
        center_distance = baseline.project(pole_center)
        options = []
        for buffer_distance in (5.0, 6.0, 7.0, 8.0, 9.0):
            before_distance = max(0.5, center_distance - buffer_distance)
            after_distance = min(baseline.length - 0.5, center_distance + buffer_distance)
            if after_distance <= before_distance + 2.0:
                continue
            before = baseline.interpolate(before_distance)
            after = baseline.interpolate(after_distance)
            prefix = substring(baseline, 0.0, before_distance)
            suffix = substring(baseline, after_distance, baseline.length)
            for direction in (1.0, -1.0):
                entry_choices = []
                exit_choices = []
                for anchor_angle in np.linspace(0.0, 2.0 * math.pi, 32, endpoint=False):
                    anchor = Point(
                        pole_center.x + TURN_RADIUS_M * math.cos(anchor_angle),
                        pole_center.y + TURN_RADIUS_M * math.sin(anchor_angle),
                    )
                    tangent_heading = anchor_angle + direction * math.pi / 2.0
                    incoming = []
                    for candidate in sample_dubins(
                        (before.x, before.y, travel_heading),
                        (anchor.x, anchor.y, tangent_heading),
                        TURN_RADIUS_M,
                    ):
                        value, measures = score(candidate["line"])
                        incoming.append((value, candidate["length"], candidate, measures))
                    outgoing = []
                    for candidate in sample_dubins(
                        (anchor.x, anchor.y, tangent_heading),
                        (after.x, after.y, travel_heading),
                        TURN_RADIUS_M,
                    ):
                        value, measures = score(candidate["line"])
                        outgoing.append((value, candidate["length"], candidate, measures))
                    if incoming:
                        entry_choices.append((anchor_angle, min(incoming)))
                    if outgoing:
                        exit_choices.append((anchor_angle, min(outgoing)))
                for entry_angle, entry in entry_choices:
                    for exit_angle, exit_item in exit_choices:
                        span = helpers.mod2pi(direction * (exit_angle - entry_angle))
                        if span < math.radians(15.0) or span > math.radians(300.0):
                            continue
                        count = max(12, int(math.ceil(span * TURN_RADIUS_M / 0.08)))
                        angles = [entry_angle + direction * span * i / count for i in range(count + 1)]
                        arc = LineString([
                            (pole_center.x + TURN_RADIUS_M * math.cos(angle), pole_center.y + TURN_RADIUS_M * math.sin(angle))
                            for angle in angles
                        ])
                        arc_value, arc_measures = score(arc)
                        entry_candidate = entry[2]
                        exit_candidate = exit_item[2]
                        combined = join_lines(prefix, entry_candidate["line"], arc, exit_candidate["line"], suffix)
                        combined_value, combined_measures = score(combined)
                        option_score = combined_value + 0.001 * (entry[0] + exit_item[0] + arc_value)
                        options.append((
                            option_score,
                            combined.length,
                            combined,
                            {
                                "direction": "counterclockwise" if direction > 0 else "clockwise",
                                "circle_span_deg": math.degrees(span),
                                "entry_word": entry_candidate["word"],
                                "exit_word": exit_candidate["word"],
                                "buffer_distance_m": buffer_distance,
                                **combined_measures,
                            },
                        ))
        if not options:
            raise RuntimeError("No smooth obstacle detour candidates")
        return min(options, key=lambda item: item[:2])[2:]

    smoothed_detours = {}
    detour_metrics = {}
    for segment_id in ("S011", "S013", "S015"):
        smoothed_detours[segment_id], detour_metrics[segment_id] = smooth_obstacle_detour(by_id[segment_id]["line"])

    # Entry: access heading at O-out -> a tangent pose farther along S003.
    access_in = by_id["S001"]["line"]
    perimeter_before = by_id["S003"]["line"]
    entry_start = access_in.coords[-1]
    entry_heading = line_heading(access_in, False)
    entry_options = []
    for target_distance in np.arange(1.0, min(18.0, perimeter_before.length - 1.0), 0.25):
        target = perimeter_before.interpolate(float(target_distance))
        target_a = perimeter_before.interpolate(max(0.0, float(target_distance) - 0.08))
        target_b = perimeter_before.interpolate(min(perimeter_before.length, float(target_distance) + 0.08))
        target_heading = math.atan2(target_b.y - target_a.y, target_b.x - target_a.x)
        for candidate in sample_dubins((*entry_start, entry_heading), (target.x, target.y, target_heading), TURN_RADIUS_M):
            value, measures = score(candidate["line"])
            entry_options.append((value, candidate["length"], target_distance, candidate, measures))
    if not entry_options:
        raise RuntimeError("No entry transition candidates")
    _, _, entry_trim, entry_candidate, entry_measures = min(entry_options)
    entry_transition = entry_candidate["line"]
    perimeter_before = substring(perimeter_before, float(entry_trim), perimeter_before.length)

    # Pole maneuver: tangent departure from S003, one full 1.4 m loop, tangent
    # return to S005. Search a common wide-side circle anchor.
    perimeter_after = by_id["S005"]["line"]
    original_circle = by_id["S004"]["line"]
    r0 = np.asarray(original_circle.coords[0]) - np.asarray((pole_center.x, pole_center.y))
    r1 = np.asarray(original_circle.coords[1]) - np.asarray((pole_center.x, pole_center.y))
    cross_z = r0[0] * r1[1] - r0[1] * r1[0]
    circle_direction = 1.0 if cross_z > 0 else -1.0
    pole_options = []
    for anchor_angle in np.linspace(0.0, 2.0 * math.pi, 72, endpoint=False):
        anchor = Point(
            pole_center.x + TURN_RADIUS_M * math.cos(anchor_angle),
            pole_center.y + TURN_RADIUS_M * math.sin(anchor_angle),
        )
        anchor_heading = anchor_angle + circle_direction * math.pi / 2.0
        best_in = None
        for setback in np.arange(0.5, min(12.0, perimeter_before.length - 0.5), 0.5):
            distance = perimeter_before.length - float(setback)
            point = perimeter_before.interpolate(distance)
            pa = perimeter_before.interpolate(max(0.0, distance - 0.08))
            pb = perimeter_before.interpolate(min(perimeter_before.length, distance + 0.08))
            heading = math.atan2(pb.y - pa.y, pb.x - pa.x)
            for candidate in sample_dubins((point.x, point.y, heading), (anchor.x, anchor.y, anchor_heading), TURN_RADIUS_M):
                value, measures = score(candidate["line"])
                item = (value, candidate["length"], distance, candidate, measures)
                if best_in is None or item[:2] < best_in[:2]:
                    best_in = item
        best_out = None
        for advance in np.arange(0.5, min(12.0, perimeter_after.length - 0.5), 0.5):
            distance = float(advance)
            point = perimeter_after.interpolate(distance)
            pa = perimeter_after.interpolate(max(0.0, distance - 0.08))
            pb = perimeter_after.interpolate(min(perimeter_after.length, distance + 0.08))
            heading = math.atan2(pb.y - pa.y, pb.x - pa.x)
            for candidate in sample_dubins((anchor.x, anchor.y, anchor_heading), (point.x, point.y, heading), TURN_RADIUS_M):
                value, measures = score(candidate["line"])
                item = (value, candidate["length"], distance, candidate, measures)
                if best_out is None or item[:2] < best_out[:2]:
                    best_out = item
        if best_in and best_out:
            pole_options.append((best_in[0] + best_out[0], best_in[1] + best_out[1], anchor_angle, best_in, best_out))
    if not pole_options:
        raise RuntimeError("No pole transition candidates")
    _, _, anchor_angle, pole_in, pole_out = min(pole_options)
    _, _, pole_before_end, pole_in_candidate, pole_in_measures = pole_in
    _, _, pole_after_start, pole_out_candidate, pole_out_measures = pole_out
    circle_angles = [anchor_angle + circle_direction * 2.0 * math.pi * i / 360 for i in range(361)]
    full_circle = LineString([
        (pole_center.x + TURN_RADIUS_M * math.cos(angle), pole_center.y + TURN_RADIUS_M * math.sin(angle))
        for angle in circle_angles
    ])
    pole_maneuver = join_lines(pole_in_candidate["line"], full_circle, pole_out_candidate["line"])
    perimeter_before = substring(perimeter_before, 0.0, float(pole_before_end))
    perimeter_after = substring(perimeter_after, float(pole_after_start), perimeter_after.length)

    # Exit: one explicit forward-only Dubins transition from the final stripe
    # to O-out. The selected LSR candidate remains inside the approved outer
    # boundary and is shorter than the rev002 sharp connector/perimeter path.
    last_stripe = by_id["S019"]["line"]
    access_out = by_id["S021"]["line"]
    exit_start_pose = (*last_stripe.coords[-1], line_heading(last_stripe, False))
    exit_end_pose = (*access_out.coords[0], line_heading(access_out, True))
    exit_options = []
    for candidate in sample_dubins(exit_start_pose, exit_end_pose, TURN_RADIUS_M):
        value, measures = score(candidate["line"])
        exit_options.append((value, candidate["length"], candidate["line"], candidate, measures))
    if not exit_options:
        raise RuntimeError("No exit transition candidates")
    _, _, exit_transition, exit_candidate, exit_measures = min(exit_options)

    replacements = {
        "S002": ("entry_transition", entry_transition, f"Dubins {entry_candidate['word']} at 1.4 m radius", "explicit left-turn entry from O-out to perimeter"),
        "S003": ("perimeter", perimeter_before, "trimmed approved perimeter", "clockwise before pole"),
        "S004": ("pole_loop", pole_maneuver, f"tangent Dubins {pole_in_candidate['word']} + full 1.4 m circle + Dubins {pole_out_candidate['word']}", "explicit leave-edge pole loop with tangent entry and exit"),
        "S005": ("perimeter", perimeter_after, "trimmed approved perimeter", "clockwise after pole"),
        "S020": ("exit_transition", exit_transition, f"direct Dubins {exit_candidate['word']} at 1.4 m radius", "explicit forward-only shorter exit to O-out"),
    }
    for segment_id, line in smoothed_detours.items():
        meta = detour_metrics[segment_id]
        replacements[segment_id] = (
            "stripe_with_pole_detour",
            line,
            f"tangent Dubins {meta['entry_word']} + {meta['direction']} 1.4 m obstacle arc + Dubins {meta['exit_word']}",
            "minimum-offset stripe detour with continuous entry and exit",
        )

    segments = []
    for item in original:
        props = item["properties"]
        segment_id = props["segment_id"]
        line = item["line"]
        new_props = dict(props)
        if segment_id in replacements:
            segment_type, line, source_text, note = replacements[segment_id]
            new_props.update({"segment_type": segment_type, "source": source_text, "note": note})
        new_props.update({"status": STATUS, "not_executable": False, "controller_route_revision": "003"})
        segments.append({"properties": new_props, "line": line})

    cumulative = 0.0
    output_features = []
    continuity = []
    joins = []
    internal_heading_changes = []
    turn_metrics = []
    for sequence, segment in enumerate(segments, start=1):
        line = segment["line"]
        props = segment["properties"]
        props.update({
            "sequence": sequence,
            "length_m": line.length,
            "cumulative_start_m": cumulative,
            "cumulative_end_m": cumulative + line.length,
            "speed_mps": SPEED_MPS,
        })
        cumulative += line.length
        coords = list(line.coords)
        headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(coords, coords[1:]) if Point(a).distance(Point(b)) > 1e-6]
        changes = [math.degrees(angle_difference(a, b)) for a, b in zip(headings, headings[1:])]
        internal_heading_changes.append({
            "segment_id": props["segment_id"],
            "maximum_sample_heading_change_deg": max(changes, default=0.0),
        })
        if props["segment_type"] in {"entry_transition", "pole_loop", "headland_turn", "exit_transition", "stripe_with_pole_detour"}:
            turn_metrics.append({
                "segment_id": props["segment_id"],
                "sampled_minimum_radius_m": min_sampled_radius(line),
            })
        output_features.append({"type": "Feature", "properties": props, "geometry": mapping(unproject(line))})

    for first, second in zip(segments, segments[1:]):
        gap = Point(first["line"].coords[-1]).distance(Point(second["line"].coords[0]))
        heading_change = math.degrees(angle_difference(line_heading(first["line"], False), line_heading(second["line"], True)))
        continuity.append({"from": first["properties"]["segment_id"], "to": second["properties"]["segment_id"], "gap_m": gap})
        joins.append({"from": first["properties"]["segment_id"], "to": second["properties"]["segment_id"], "heading_change_deg": heading_change})

    cutting = [segment["line"] for segment in segments if segment["properties"]["segment_type"] != "access"]
    planned_union = unary_union([line.buffer(CUT_WIDTH_M / 2.0, cap_style=2, join_style=1) for line in cutting])
    physical_union = unary_union([line.buffer(DECK_WIDTH_M / 2.0, cap_style=2, join_style=1) for line in cutting])
    conservative_union = unary_union([line.buffer(DECK_WIDTH_M / 2.0 + CHUTE_EXTENSION_M, cap_style=2, join_style=1) for line in cutting])
    route_line = unary_union([segment["line"] for segment in segments])
    validation = {
        "status": STATUS,
        "revision": "003",
        "parameters": {
            "planned_cut_width_and_lane_spacing_m": CUT_WIDTH_M,
            "physical_deck_width_m": DECK_WIDTH_M,
            "right_chute_extension_m_approx": CHUTE_EXTENSION_M,
            "planning_turn_radius_m": TURN_RADIUS_M,
            "speed_mps": SPEED_MPS,
            "stripe_heading_compass_deg": 339.5148697683819,
        },
        "route_concept": "approved",
        "field_authorized": False,
        "controller_file_created": True,
        "route": {
            "segment_count": len(segments),
            "total_length_m": cumulative,
            "estimated_motion_time_min": cumulative / 60.0,
            "forward_only": True,
        },
        "joins": {
            "continuity": continuity,
            "maximum_gap_m": max(item["gap_m"] for item in continuity),
            "heading_changes": joins,
            "maximum_heading_change_deg": max(item["heading_change_deg"] for item in joins),
            "joins_over_10_degrees": [item for item in joins if item["heading_change_deg"] > 10.0],
        },
        "internal_path_sampling": {
            "per_segment_maximum_heading_change": internal_heading_changes,
            "note": "Pure Pursuit intentionally rounds polyline changes according to lookahead and steering saturation; these are disclosed for the blades-off field test.",
        },
        "turns": turn_metrics,
        "coverage": {
            "approved_area_m2": area.area,
            "planned_38in_covered_m2": planned_union.intersection(area).area,
            "planned_38in_uncovered_m2": area.difference(planned_union).area,
            "physical_42in_covered_m2": physical_union.intersection(area).area,
            "physical_42in_uncovered_m2": area.difference(physical_union).area,
        },
        "containment_and_clearance": {
            "centerline_outside_outer_boundary_m": route_line.difference(outer).length,
            "physical_deck_intersection_with_pole_m2": physical_union.intersection(hole).area,
            "conservative_deck_plus_chute_intersection_with_pole_m2": conservative_union.intersection(hole).area,
            "conservative_envelope_outside_outer_boundary_m2": conservative_union.difference(outer).area,
        },
        "smoothing": {
            "entry": {"word": entry_candidate["word"], **entry_measures},
            "pole_entry": {"word": pole_in_candidate["word"], **pole_in_measures},
            "pole_exit": {"word": pole_out_candidate["word"], **pole_out_measures},
            "exit": exit_measures,
            "stripe_detours": detour_metrics,
        },
        "lookahead_policy": {
            "straight_access_perimeter_m": 2.0,
            "turn_headland_pole_detour_m": 1.5,
            "note": "Retains the previously approved provisional test values. Geometry, not lookahead alone, defines the new six turns.",
        },
    }

    OUT_GEOJSON.write_text(json.dumps({"type": "FeatureCollection", "features": output_features}, indent=2) + "\n", encoding="utf-8")
    OUT_VALIDATION.write_text(json.dumps(validation, indent=2) + "\n", encoding="utf-8")

    mission_rows = []
    route_distance = 0.0
    origin = Point(segments[0]["line"].coords[0])
    previous = None
    for segment in segments:
        props = segment["properties"]
        points = sample_line(segment["line"])
        if previous is not None and previous.distance(points[0]) < 0.01:
            points = points[1:]
        for index, point in enumerate(points):
            if previous is not None:
                route_distance += previous.distance(point)
            if index + 1 < len(points):
                target = points[index + 1]
                yaw = math.atan2(target.y - point.y, target.x - point.x)
            elif previous is not None:
                yaw = math.atan2(point.y - previous.y, point.x - previous.x)
            else:
                yaw = 0.0
            turn_type = props["segment_type"] in {"entry_transition", "pole_loop", "headland_turn", "exit_transition", "stripe_with_pole_detour"}
            lookahead = 1.5 if turn_type else 2.0
            geographic_point = unproject(point)
            mission_rows.append({
                "waypoint": len(mission_rows) + 1,
                "phase": props["phase"],
                "lat": geographic_point.y,
                "lon": geographic_point.x,
                "yaw_rad": yaw,
                "lookahead_m": lookahead,
                "speed_mps": SPEED_MPS,
                "east_m": point.x - origin.x,
                "north_m": point.y - origin.y,
                "distance_m": route_distance,
                "source_sequence": props["sequence"],
                "source_distance_m": route_distance - props["cumulative_start_m"],
                "source_segment_id": props["segment_id"],
                "source_kind": props["segment_type"],
                "geometry_status": "CONTROLLER_FORMAT_DRAFT_UNTESTED",
            })
            previous = point

    OUT_MISSION.parent.mkdir(parents=True, exist_ok=True)
    with OUT_MISSION.open("w", encoding="utf-8", newline="\n") as handle:
        for row in mission_rows:
            handle.write(
                f"{row['lat']:.9f} {row['lon']:.9f} {row['yaw_rad']:.9f} "
                f"{row['lookahead_m']:.2f} {row['speed_mps']:.2f}\n"
            )
    with OUT_AUDIT.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(mission_rows[0]))
        writer.writeheader()
        writer.writerows(mission_rows)

    print(f"Wrote route revision 003: {cumulative:.3f} m, {len(mission_rows)} waypoints")
    print(f"Maximum segment join: {validation['joins']['maximum_heading_change_deg']:.3f} deg")
    print(f"Joins over 10 deg: {len(validation['joins']['joins_over_10_degrees'])}")
    print(OUT_MISSION)


if __name__ == "__main__":
    main()
