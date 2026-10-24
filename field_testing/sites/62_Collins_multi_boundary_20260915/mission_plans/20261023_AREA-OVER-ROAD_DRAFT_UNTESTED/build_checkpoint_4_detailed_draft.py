#!/usr/bin/env python3
"""Build an auditable, non-executable Checkpoint 4 draft for selected strategy A."""

from __future__ import annotations

import csv
import html
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon as PolygonPatch
from shapely.geometry import LineString, Point, Polygon, mapping, shape
from shapely.ops import substring, transform, unary_union


STATUS = "DRAFT - UNTESTED IN THE FIELD"
CUT_WIDTH_M = 0.9652
DECK_WIDTH_M = 1.0668
CHUTE_EXTENSION_M = 0.127
TURN_RADIUS_M = 1.4
SPEED_MPS = 1.0
SAMPLE_STEP_M = 0.08
ROOT = Path(__file__).resolve().parents[5]
SITE = ROOT / "field_testing/sites/62_Collins_multi_boundary_20260915"
INVENTORY = SITE / "site_inventory/62_Collins_site_inventory.geojson"
MOWABLE = SITE / (
    "site_inventory/assets/AREA-OVER-ROAD/revisions/"
    "rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_mowable_area.geojson"
)
HERE = Path(__file__).resolve().parent
OUT_GEOJSON = HERE / "checkpoint_4_detailed_draft_rev002.geojson"
OUT_METRICS = HERE / "checkpoint_4_validation_rev002.json"
OUT_CSV = HERE / "checkpoint_4_segment_lineage_rev002.csv"
OUT_PNG = HERE / "checkpoint_4_detailed_overview_rev002.png"
OUT_HTML = HERE / "checkpoint_4_interactive_replay_rev002.html"


def mod2pi(value: float) -> float:
    return value % (2.0 * math.pi)


def dubins_words(alpha: float, beta: float, distance: float):
    """Return normalized Dubins candidates as (word, t, p, q)."""
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    candidates = []

    def add(word, values):
        if values is not None and all(math.isfinite(value) for value in values):
            candidates.append((word, *values))

    p2 = 2 + distance**2 - 2 * cab + 2 * distance * (sa - sb)
    if p2 >= 0:
        tmp = math.atan2(cb - ca, distance + sa - sb)
        add("LSL", (mod2pi(-alpha + tmp), math.sqrt(p2), mod2pi(beta - tmp)))

    p2 = 2 + distance**2 - 2 * cab + 2 * distance * (-sa + sb)
    if p2 >= 0:
        tmp = math.atan2(ca - cb, distance - sa + sb)
        add("RSR", (mod2pi(alpha - tmp), math.sqrt(p2), mod2pi(-beta + tmp)))

    p2 = -2 + distance**2 + 2 * cab + 2 * distance * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, distance + sa + sb) - math.atan2(-2.0, p)
        add("LSR", (mod2pi(-alpha + tmp), p, mod2pi(-beta + tmp)))

    p2 = distance**2 - 2 + 2 * cab - 2 * distance * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, distance - sa - sb) - math.atan2(2.0, p)
        add("RSL", (mod2pi(alpha - tmp), p, mod2pi(beta - tmp)))

    tmp = (6 - distance**2 + 2 * cab + 2 * distance * (sa - sb)) / 8.0
    if abs(tmp) <= 1:
        p = mod2pi(2 * math.pi - math.acos(tmp))
        t = mod2pi(alpha - math.atan2(ca - cb, distance - sa + sb) + p / 2.0)
        add("RLR", (t, p, mod2pi(alpha - beta - t + p)))

    tmp = (6 - distance**2 + 2 * cab + 2 * distance * (-sa + sb)) / 8.0
    if abs(tmp) <= 1:
        p = mod2pi(2 * math.pi - math.acos(tmp))
        t = mod2pi(-alpha - math.atan2(ca - cb, distance + sa - sb) + p / 2.0)
        add("LRL", (t, p, mod2pi(beta - alpha - t + p)))
    return candidates


def sample_dubins(start, end, radius: float, step: float = SAMPLE_STEP_M):
    """Return every forward-only Dubins candidate between two x/y/heading poses."""
    x0, y0, heading0 = start
    x1, y1, heading1 = end
    dx, dy = x1 - x0, y1 - y0
    distance_world = math.hypot(dx, dy)
    theta = math.atan2(dy, dx)
    alpha = mod2pi(heading0 - theta)
    beta = mod2pi(heading1 - theta)
    results = []
    for word, t, p, q in dubins_words(alpha, beta, distance_world / radius):
        x, y, heading = x0, y0, heading0
        points = [(x, y)]
        for kind, parameter in zip(word, (t, p, q)):
            segment_length = parameter * radius
            count = max(1, int(math.ceil(segment_length / step)))
            ds = segment_length / count
            for _ in range(count):
                if kind == "S":
                    x += ds * math.cos(heading)
                    y += ds * math.sin(heading)
                else:
                    curvature = (1.0 if kind == "L" else -1.0) / radius
                    next_heading = heading + curvature * ds
                    x += (math.sin(next_heading) - math.sin(heading)) / curvature
                    y += (-math.cos(next_heading) + math.cos(heading)) / curvature
                    heading = next_heading
                points.append((x, y))
        position_error = math.hypot(x - x1, y - y1)
        heading_error = angle_difference(heading, heading1)
        if position_error > 0.02 or heading_error > math.radians(1.0):
            continue
        points[-1] = (x1, y1)
        line = LineString(points)
        results.append({"word": word, "parameters": [t, p, q], "line": line, "length": line.length})
    return results


def orient_ring(ring: LineString, clockwise: bool) -> LineString:
    coords = list(ring.coords)[:-1]
    signed = 0.5 * sum(
        coords[i][0] * coords[(i + 1) % len(coords)][1]
        - coords[(i + 1) % len(coords)][0] * coords[i][1]
        for i in range(len(coords))
    )
    if clockwise == (signed > 0):
        coords.reverse()
    coords.append(coords[0])
    return LineString(coords)


def ring_path(ring: LineString, start: Point, end: Point) -> LineString:
    """Follow an oriented closed ring from exact projected start to end points."""
    length = ring.length
    start_d, end_d = ring.project(start), ring.project(end)
    if end_d >= start_d:
        part = substring(ring, start_d, end_d)
        coords = [start.coords[0], *list(part.coords)[1:-1], end.coords[0]]
    else:
        first = substring(ring, start_d, length)
        second = substring(ring, 0.0, end_d)
        coords = [start.coords[0], *list(first.coords)[1:], *list(second.coords)[1:-1], end.coords[0]]
    return LineString(coords)


def sample_circle(center: Point, radius: float, anchor: Point, clockwise: bool, count: int = 260) -> LineString:
    start = math.atan2(anchor.y - center.y, anchor.x - center.x)
    direction = -1.0 if clockwise else 1.0
    angles = [start + direction * 2.0 * math.pi * i / count for i in range(count + 1)]
    return LineString([(center.x + radius * math.cos(a), center.y + radius * math.sin(a)) for a in angles])


def line_heading(line: LineString, at_start: bool) -> float:
    coords = list(line.coords)
    a, b = (coords[0], coords[min(2, len(coords) - 1)]) if at_start else (coords[max(0, len(coords) - 3)], coords[-1])
    return math.atan2(b[1] - a[1], b[0] - a[0])


def angle_difference(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def min_sampled_radius(line: LineString) -> float | None:
    coords = np.asarray(line.coords)
    if len(coords) < 3:
        return None
    radii = []
    for a, b, c in zip(coords[:-2], coords[1:-1], coords[2:]):
        ab, bc, ac = np.linalg.norm(b - a), np.linalg.norm(c - b), np.linalg.norm(c - a)
        ab_vector = b - a
        ac_vector = c - a
        cross = abs(ab_vector[0] * ac_vector[1] - ab_vector[1] * ac_vector[0])
        if cross > 1e-7 and min(ab, bc, ac) > 1e-6:
            radii.append(ab * bc * ac / (2.0 * cross))
    return min(radii) if radii else None


def main() -> None:
    inventory = json.loads(INVENTORY.read_text(encoding="utf-8"))
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
    outer_coords = np.asarray(area.exterior.coords)[:-1]
    values, vectors = np.linalg.eigh(np.cov(outer_coords.T))
    long_axis = vectors[:, np.argmax(values)]
    if long_axis[1] < 0:
        long_axis = -long_axis
    cross_axis = np.array([-long_axis[1], long_axis[0]])
    angle_deg = math.degrees(math.atan2(long_axis[1], long_axis[0])) % 180.0
    compass = (90.0 - angle_deg) % 360.0

    feature_by_id = {
        feature["properties"].get("asset_id"): feature
        for feature in inventory["features"]
        if feature["properties"].get("asset_id")
    }
    f_out = project(shape(feature_by_id["ACCESS-FRONT-NORTH-OUTBOUND"]["geometry"]))
    o_out = project(shape(feature_by_id["ACCESS-OVERROAD-NORTH-INBOUND"]["geometry"]))

    # Clockwise perimeter centerline: left deck edge stays near the mapped grass edge.
    raw_outer_route = outer.buffer(-DECK_WIDTH_M / 2.0, join_style=1).exterior
    pole_center = hole.centroid
    conceptual_circle = Point(pole_center.x, pole_center.y).buffer(TURN_RADIUS_M, resolution=256).boundary
    intersections = raw_outer_route.intersection(conceptual_circle)
    junction_candidates = list(intersections.geoms) if hasattr(intersections, "geoms") else [intersections]
    junction = min(junction_candidates, key=lambda point: point.distance(o_out))
    oriented_outer = orient_ring(raw_outer_route, clockwise=True)
    if len(junction_candidates) != 2:
        raise RuntimeError(f"Expected two pole-circle/perimeter intersections, found {len(junction_candidates)}")
    first_intersection, second_intersection = junction_candidates
    first_to_second = ring_path(oriented_outer, first_intersection, second_intersection)
    second_to_first = ring_path(oriented_outer, second_intersection, first_intersection)
    if first_to_second.length <= second_to_first.length:
        loop_start, loop_end = first_intersection, second_intersection
        skipped_narrow_edge = first_to_second
        perimeter = second_to_first
    else:
        loop_start, loop_end = second_intersection, first_intersection
        skipped_narrow_edge = second_to_first
        perimeter = first_to_second

    start_angle = math.atan2(loop_start.y - pole_center.y, loop_start.x - pole_center.x)
    end_angle = math.atan2(loop_end.y - pole_center.y, loop_end.x - pole_center.x)
    pole_arc_options = []
    for direction in (1.0, -1.0):
        span = mod2pi(direction * (end_angle - start_angle))
        angles = [start_angle + direction * span * i / 220 for i in range(221)]
        arc = LineString([(pole_center.x + TURN_RADIUS_M * math.cos(a), pole_center.y + TURN_RADIUS_M * math.sin(a)) for a in angles])
        outside = arc.difference(outer).length
        samples = [arc.interpolate(fraction, normalized=True).distance(outer.boundary) for fraction in np.linspace(0.05, 0.95, 19)]
        pole_arc_options.append((outside, -sum(samples) / len(samples), arc.length, arc, direction))
    _, _, _, pole_bypass_arc, pole_direction = min(pole_arc_options)
    full_circle_angles = [start_angle + pole_direction * 2.0 * math.pi * i / 260 for i in range(261)]
    full_circle = LineString([(pole_center.x + TURN_RADIUS_M * math.cos(a), pole_center.y + TURN_RADIUS_M * math.sin(a)) for a in full_circle_angles])
    pole_loop = LineString(list(full_circle.coords) + list(pole_bypass_arc.coords)[1:])
    entry_distance = perimeter.project(o_out)
    entry_point = perimeter.interpolate(entry_distance)
    perimeter_before_pole = substring(perimeter, entry_distance, perimeter.length)
    perimeter_after_pole = substring(perimeter, 0.0, entry_distance)

    # Reuse the phase selected by the CP3 conservative 38-inch swath search.
    def stripe_groups(phase: float):
        cross_values = outer_coords @ cross_axis
        lower, upper = float(np.min(cross_values)), float(np.max(cross_values))
        index = math.floor((lower - phase) / CUT_WIDTH_M) - 1
        groups = []
        while True:
            offset = phase + index * CUT_WIDTH_M
            if offset > upper + CUT_WIDTH_M:
                break
            source = LineString([tuple(long_axis * d + cross_axis * offset) for d in (-200.0, 200.0)])
            intersection = area.intersection(source)
            pieces = list(intersection.geoms) if hasattr(intersection, "geoms") else [intersection]
            pieces = [piece for piece in pieces if piece.geom_type == "LineString" and piece.length > 0.25]
            if pieces:
                endpoints = [np.asarray(point) for piece in pieces for point in piece.coords]
                groups.append({
                    "offset": offset,
                    "pieces": pieces,
                    "low": np.asarray(min(endpoints, key=lambda point: point @ long_axis)),
                    "high": np.asarray(max(endpoints, key=lambda point: point @ long_axis)),
                })
            index += 1
        return groups

    best = None
    for phase in np.linspace(0.0, CUT_WIDTH_M, 40, endpoint=False):
        groups = stripe_groups(float(phase))
        parts = [part for group in groups for part in group["pieces"]]
        swaths = [perimeter.buffer(CUT_WIDTH_M / 2.0, cap_style=2), pole_loop.buffer(CUT_WIDTH_M / 2.0, cap_style=2)]
        swaths.extend(part.buffer(CUT_WIDTH_M / 2.0, cap_style=2) for part in parts)
        covered = unary_union(swaths).intersection(area)
        candidate = (area.difference(covered).area, sum(part.length for part in parts), float(phase), groups)
        if best is None or candidate[:2] < best[:2]:
            best = candidate
    _, _, phase, groups = best

    # Build each stripe, replacing a pole-crossing section with the safer side of the 1.4 m circle.
    base_stripes = []
    stripe_detour_meta = []
    for index, group in enumerate(groups):
        low, high = group["low"], group["high"]
        source = LineString([tuple(low), tuple(high)])
        cross = source.intersection(conceptual_circle)
        cross_points = list(cross.geoms) if hasattr(cross, "geoms") else ([] if cross.is_empty else [cross])
        deck_would_touch_pole_exclusion = source.buffer(DECK_WIDTH_M / 2.0, cap_style=2).intersects(hole)
        if deck_would_touch_pole_exclusion and len(cross_points) >= 2:
            cross_points.sort(key=lambda point: np.asarray(point.coords[0]) @ long_axis)
            first, second = cross_points[0], cross_points[-1]
            a0 = math.atan2(first.y - pole_center.y, first.x - pole_center.x)
            a1 = math.atan2(second.y - pole_center.y, second.x - pole_center.x)
            arc_options = []
            for direction in (1.0, -1.0):
                span = mod2pi(direction * (a1 - a0))
                angles = [a0 + direction * span * i / 120 for i in range(121)]
                arc = LineString([(pole_center.x + TURN_RADIUS_M * math.cos(a), pole_center.y + TURN_RADIUS_M * math.sin(a)) for a in angles])
                outside_length = arc.difference(outer).length
                boundary_clearance = arc.distance(outer.boundary)
                arc_options.append((outside_length, -boundary_clearance, arc.length, arc, direction))
            _, negative_clearance, _, arc, direction = min(arc_options)
            coords = [tuple(low), first.coords[0]] + list(arc.coords)[1:-1] + [second.coords[0], tuple(high)]
            stripe = LineString(coords)
            stripe_detour_meta.append({
                "lane_index": index,
                "side_selection": "larger outer-boundary clearance",
                "arc_direction": "counterclockwise" if direction > 0 else "clockwise",
                "minimum_outer_boundary_clearance_m": -negative_clearance,
            })
        else:
            stripe = source
        base_stripes.append(stripe)

    # Select a contained Dubins connection at each end, allowing independently shortened stripe ends.
    stripe_routes = [None] * len(base_stripes)
    turn_routes = []
    end_setbacks = {}
    for index in range(len(base_stripes) - 1):
        high_end = index % 2 == 1
        direction = long_axis if high_end else -long_axis
        start_base = groups[index]["high" if high_end else "low"]
        end_base = groups[index + 1]["high" if high_end else "low"]
        if float(np.linalg.norm(start_base - end_base)) > 10.0:
            setback = 1.5
            turn_routes.append({
                "line": None,
                "word": "SAFE_PERIMETER_RELOCATION",
                "setback_m": setback,
                "outside_centerline_length_m": None,
                "obstacle_intersection_length_m": None,
                "physical_envelope_outside_m2": None,
            })
            end_setbacks[(index, "high" if high_end else "low")] = setback
            end_setbacks[(index + 1, "high" if high_end else "low")] = setback
            continue
        candidates = []
        for setback in np.arange(1.5, 8.01, 0.25):
            start_xy = start_base - direction * setback
            end_xy = end_base - direction * setback
            start_pose = (float(start_xy[0]), float(start_xy[1]), math.atan2(direction[1], direction[0]))
            end_pose = (float(end_xy[0]), float(end_xy[1]), math.atan2(-direction[1], -direction[0]))
            for candidate in sample_dubins(start_pose, end_pose, TURN_RADIUS_M):
                line = candidate["line"]
                outside = line.difference(outer).length
                obstacle = line.intersection(hole).length
                physical_outside = line.buffer(DECK_WIDTH_M / 2.0, cap_style=2, join_style=1).difference(outer).area
                self_cross_penalty = 1_000.0 if not line.is_simple else 0.0
                score = outside * 10_000 + obstacle * 10_000 + self_cross_penalty + physical_outside * 5 + candidate["length"] + setback * 0.2
                candidates.append((score, outside, obstacle, physical_outside, setback, candidate))
        score, outside, obstacle, physical_outside, setback, chosen = min(candidates)
        turn_routes.append({
            "line": chosen["line"],
            "word": chosen["word"],
            "setback_m": float(setback),
            "outside_centerline_length_m": outside,
            "obstacle_intersection_length_m": obstacle,
            "physical_envelope_outside_m2": physical_outside,
        })
        end_setbacks[(index, "high" if high_end else "low")] = float(setback)
        end_setbacks[(index + 1, "high" if high_end else "low")] = float(setback)

    def trimmed_stripe(index: int) -> LineString:
        line = base_stripes[index]
        direction_low_to_high = index % 2 == 1
        low_setback = end_setbacks.get((index, "low"), 1.5)
        high_setback = end_setbacks.get((index, "high"), 1.5)
        start_d = low_setback
        end_d = max(start_d + 0.1, line.length - high_setback)
        coords = []
        count = max(2, int(math.ceil((end_d - start_d) / SAMPLE_STEP_M)))
        for d in np.linspace(start_d, end_d, count):
            point = line.interpolate(float(d))
            coords.append((point.x, point.y))
        result = LineString(coords)
        return result if direction_low_to_high else LineString(list(result.coords)[::-1])

    stripe_routes = [trimmed_stripe(index) for index in range(len(base_stripes))]

    # CP4 review adjustment: keep the first stripe where the perimeter pass ends,
    # then visit the far edge sliver and work back across the remaining adjacent
    # lanes. This removes the 102 m perimeter relocation without dropping the
    # sliver from coverage.
    stripe_order = [0, *range(len(stripe_routes) - 1, 0, -1)]
    stripe_routes = [stripe_routes[index] for index in stripe_order]

    # Rebuild every between-stripe maneuver for the reviewed order. Each
    # connection is a continuous, forward-only 1.4 m Dubins path rather than a
    # sharp straight-line join or a long safe-perimeter relocation.
    turn_routes = []
    for route_index in range(len(stripe_routes) - 1):
        start_line = stripe_routes[route_index]
        end_line = stripe_routes[route_index + 1]
        start_pose = (*start_line.coords[-1], line_heading(start_line, False))
        end_pose = (*end_line.coords[0], line_heading(end_line, True))
        candidates = []
        for candidate in sample_dubins(start_pose, end_pose, TURN_RADIUS_M):
            line = candidate["line"]
            outside = line.difference(outer).length
            obstacle = line.intersection(hole).length
            physical_outside = line.buffer(DECK_WIDTH_M / 2.0, cap_style=2, join_style=1).difference(outer).area
            self_cross_penalty = 1_000.0 if not line.is_simple else 0.0
            score = outside * 10_000 + obstacle * 10_000 + self_cross_penalty + physical_outside * 5 + line.length
            candidates.append((score, outside, obstacle, physical_outside, candidate))
        if not candidates:
            raise RuntimeError(f"No forward-only transition for reviewed stripe positions {route_index} and {route_index + 1}")
        _, outside, obstacle, physical_outside, chosen = min(candidates)
        turn_routes.append({
            "line": chosen["line"],
            "word": chosen["word"],
            "setback_m": None,
            "outside_centerline_length_m": outside,
            "obstacle_intersection_length_m": obstacle,
            "physical_envelope_outside_m2": physical_outside,
            "review_adjustment": "reordered stripe transition; replaces edge-sliver relocation",
        })

    # Replace the sharp perimeter-to-first-stripe join. Continue two metres on
    # the perimeter departure heading, then use a continuous 1.4 m right-turn
    # path to align with the first stripe.
    entry_heading = line_heading(perimeter_after_pole, False)
    entry_extension_end = Point(
        entry_point.x + 2.0 * math.cos(entry_heading),
        entry_point.y + 2.0 * math.sin(entry_heading),
    )
    entry_lead = LineString([entry_point.coords[0], entry_extension_end.coords[0]])
    entry_candidates = []
    first_stripe = stripe_routes[0]
    for candidate in sample_dubins(
        (*entry_extension_end.coords[0], entry_heading),
        (*first_stripe.coords[0], line_heading(first_stripe, True)),
        TURN_RADIUS_M,
    ):
        line = LineString(list(entry_lead.coords) + list(candidate["line"].coords)[1:])
        outside = line.difference(outer).length
        obstacle = line.intersection(hole).length
        physical_outside = line.buffer(DECK_WIDTH_M / 2.0, cap_style=2, join_style=1).difference(outer).area
        score = outside * 10_000 + obstacle * 10_000 + physical_outside * 5 + line.length
        entry_candidates.append((score, line, candidate["word"], outside, obstacle, physical_outside))
    if not entry_candidates:
        raise RuntimeError("No forward-only smoothed entry into the first stripe")
    _, stripe_entry, stripe_entry_word, _, _, _ = min(entry_candidates)
    safe_perimeter = LineString(
        list(perimeter_before_pole.coords)
        + list(pole_bypass_arc.coords)[1:]
        + list(perimeter_after_pole.coords)[1:]
    )
    final_stripe_end = Point(stripe_routes[-1].coords[-1])
    exit_join_distance = safe_perimeter.project(final_stripe_end)
    exit_join = safe_perimeter.interpolate(exit_join_distance)
    clockwise_exit_part = substring(safe_perimeter, exit_join_distance, safe_perimeter.length)
    reversed_safe_perimeter = LineString(list(safe_perimeter.coords)[::-1])
    reverse_join_distance = reversed_safe_perimeter.project(exit_join)
    counterclockwise_exit_part = substring(reversed_safe_perimeter, reverse_join_distance, reversed_safe_perimeter.length)
    perimeter_exit_part = min(
        (clockwise_exit_part, counterclockwise_exit_part),
        key=lambda candidate: candidate.length,
    )
    exit_relocation = LineString(
        [final_stripe_end.coords[0], exit_join.coords[0]] + list(perimeter_exit_part.coords)[1:]
    )

    segments = []

    def add(segment_type, line, phase_name, source, note="", extra=None):
        segment_id = f"S{len(segments) + 1:03d}"
        record = {
            "segment_id": segment_id,
            "segment_type": segment_type,
            "phase": phase_name,
            "line": line,
            "source": source,
            "note": note,
            "forward_only": True,
        }
        if extra:
            record.update(extra)
        segments.append(record)

    add("access", LineString([f_out.coords[0], o_out.coords[0]]), "access in", "owner-approved F-out/O-out corridor", "F-out to O-out")
    add("connector", LineString([o_out.coords[0], entry_point.coords[0]]), "enter area", "derived straight connector", "requires smoothing before executable mission")
    add("perimeter", perimeter_before_pole, "outer pass", "42-inch deck half-width inward offset", "clockwise before pole")
    add("pole_loop", pole_loop, "outer pass", "1.4 m circle at OBS-POLE-001", "leave-edge pole loop; junction heading must be reviewed")
    add("perimeter", perimeter_after_pole, "outer pass", "42-inch deck half-width inward offset", "clockwise after pole")
    add("headland_turn", stripe_entry, "enter stripes", f"2.0 m tangent lead plus Dubins {stripe_entry_word} at 1.4 m radius", "reviewed smoother right turn into first stripe")
    for route_position, stripe in enumerate(stripe_routes):
        lane_index = stripe_order[route_position]
        detour = next((item for item in stripe_detour_meta if item["lane_index"] == lane_index), None)
        add(
            "stripe_with_pole_detour" if detour else "stripe",
            stripe,
            "interior stripes",
            f"best-fit lane {lane_index + 1} at 38-inch spacing; reviewed route position {route_position + 1}",
            "minimum-offset stripe detour included" if detour else "straight stripe",
            {"lane_index": lane_index, "route_position": route_position, "detour": detour},
        )
        if route_position < len(turn_routes):
            turn = turn_routes[route_position]
            add(
                "headland_turn",
                turn["line"],
                "interior stripes",
                f"Dubins {turn['word']} at 1.4 m radius",
                "reviewed forward-only stripe transition; reordered to eliminate long relocation",
                {key: value for key, value in turn.items() if key != "line"},
            )
    add("connector", exit_relocation, "exit area", "join nearest cut edge then take the shorter safe-perimeter direction to O-out", "uses already-cut ground; joins require smoothing")
    add("access", LineString([o_out.coords[0], f_out.coords[0]]), "access out", "owner-approved F-out/O-out corridor", "O-out to F-out")

    # Validation and coverage metrics.
    cutting_segments = [segment["line"] for segment in segments if segment["segment_type"] not in ("access",)]
    planned_swaths = [line.buffer(CUT_WIDTH_M / 2.0, cap_style=2, join_style=1) for line in cutting_segments]
    physical_cut_swaths = [line.buffer(DECK_WIDTH_M / 2.0, cap_style=2, join_style=1) for line in cutting_segments]
    conservative_envelopes = [line.buffer(DECK_WIDTH_M / 2.0 + CHUTE_EXTENSION_M, cap_style=2, join_style=1) for line in cutting_segments]
    planned_union = unary_union(planned_swaths)
    physical_union = unary_union(physical_cut_swaths)
    envelope_union = unary_union(conservative_envelopes)
    covered = planned_union.intersection(area)
    physical_covered = physical_union.intersection(area)
    uncovered = area.difference(planned_union)
    physical_uncovered = area.difference(physical_union)
    planned_sum_inside = sum(swath.intersection(area).area for swath in planned_swaths)
    overlap = max(0.0, planned_sum_inside - covered.area)
    gap_geometries = list(uncovered.geoms) if hasattr(uncovered, "geoms") else [uncovered]
    gap_components = []
    for geometry in sorted(gap_geometries, key=lambda item: item.area, reverse=True):
        if geometry.geom_type == "Polygon" and geometry.area > 1e-5:
            gap_components.append({
                "area_m2": geometry.area,
                "centroid_local_east_m": geometry.centroid.x,
                "centroid_local_north_m": geometry.centroid.y,
                "bounds_local_m": list(geometry.bounds),
            })

    continuity = []
    heading_jumps = []
    for first, second in zip(segments[:-1], segments[1:]):
        gap = Point(first["line"].coords[-1]).distance(Point(second["line"].coords[0]))
        jump = math.degrees(angle_difference(line_heading(first["line"], False), line_heading(second["line"], True)))
        continuity.append({"from": first["segment_id"], "to": second["segment_id"], "gap_m": gap})
        heading_jumps.append({"from": first["segment_id"], "to": second["segment_id"], "heading_change_deg": jump})

    self_intersections = [segment["segment_id"] for segment in segments if not segment["line"].is_simple]
    route_crossings = []
    for i, first in enumerate(segments):
        for j in range(i + 2, len(segments)):
            second = segments[j]
            intersection = first["line"].intersection(second["line"])
            if not intersection.is_empty and first["segment_type"] != "access" and second["segment_type"] != "access":
                route_crossings.append({"first": first["segment_id"], "second": second["segment_id"], "geometry": intersection.geom_type})

    turn_metrics = []
    for segment in segments:
        radius = min_sampled_radius(segment["line"])
        if segment["segment_type"] in ("headland_turn", "pole_loop"):
            turn_metrics.append({
                "segment_id": segment["segment_id"],
                "type": segment["segment_type"],
                "construction": segment["source"],
                "sampled_minimum_radius_m": radius,
                "passes_1_4_m_test": bool(radius is not None and radius >= TURN_RADIUS_M - 0.03),
            })

    per_segment_clearance = []
    for segment in segments:
        if segment["segment_type"] == "access":
            continue
        deck_intersection = segment["line"].buffer(DECK_WIDTH_M / 2.0, cap_style=2, join_style=1).intersection(hole).area
        chute_intersection = segment["line"].buffer(DECK_WIDTH_M / 2.0 + CHUTE_EXTENSION_M, cap_style=2, join_style=1).intersection(hole).area
        per_segment_clearance.append({
            "segment_id": segment["segment_id"],
            "segment_type": segment["segment_type"],
            "centerline_to_pole_exclusion_m": segment["line"].distance(hole),
            "physical_deck_intersection_with_pole_m2": deck_intersection,
            "conservative_deck_plus_chute_intersection_with_pole_m2": chute_intersection,
        })

    non_access = unary_union(cutting_segments)
    metrics = {
        "status": STATUS,
        "scope": "AREA-OVER-ROAD selected Alternative A detailed draft",
        "not_executable": True,
        "revision": "002",
        "parameters": {
            "planned_cut_width_and_lane_spacing_m": CUT_WIDTH_M,
            "physical_deck_width_m": DECK_WIDTH_M,
            "right_chute_extension_m_approx": CHUTE_EXTENSION_M,
            "planning_turn_radius_m": TURN_RADIUS_M,
            "speed_mps": SPEED_MPS,
            "stripe_heading_compass_deg": compass,
            "stripe_phase_m": phase,
            "lane_count": len(groups),
        },
        "route": {
            "segment_count": len(segments),
            "total_length_m": sum(segment["line"].length for segment in segments),
            "estimated_motion_time_min": sum(segment["line"].length for segment in segments) / SPEED_MPS / 60.0,
            "forward_only": True,
            "reversal_count": 0,
        },
        "coverage": {
            "approved_area_m2": area.area,
            "planned_38in_covered_m2": covered.area,
            "planned_38in_coverage_percent": covered.area / area.area * 100.0,
            "planned_38in_uncovered_m2": uncovered.area,
            "actual_42in_cut_covered_m2": physical_covered.area,
            "actual_42in_cut_coverage_percent": physical_covered.area / area.area * 100.0,
            "actual_42in_cut_uncovered_m2": physical_uncovered.area,
            "planned_overlap_m2_sum_minus_union": overlap,
            "planned_swath_outside_outer_boundary_m2": planned_union.difference(outer).area,
            "physical_cut_outside_outer_boundary_m2": physical_union.difference(outer).area,
            "conservative_deck_plus_chute_envelope_outside_outer_boundary_m2": envelope_union.difference(outer).area,
            "gap_components": gap_components,
        },
        "containment_and_clearance": {
            "centerline_outside_outer_boundary_length_m": non_access.difference(outer).length,
            "minimum_centerline_to_outer_boundary_m": non_access.distance(outer.boundary),
            "minimum_centerline_to_pole_exclusion_m": non_access.distance(hole),
            "physical_cut_intersection_with_pole_exclusion_m2": physical_union.intersection(hole).area,
            "conservative_deck_plus_chute_intersection_with_pole_exclusion_m2": envelope_union.intersection(hole).area,
            "pole_loop_centerline_to_outer_boundary_m": pole_loop.distance(outer.boundary),
            "pole_loop_centerline_to_pole_exclusion_m": pole_loop.distance(hole),
            "per_segment_pole_clearance": per_segment_clearance,
        },
        "spacing": {
            "lane_offsets_m": [float(group["offset"]) for group in groups],
            "adjacent_spacing_m": [float(groups[i + 1]["offset"] - groups[i]["offset"]) for i in range(len(groups) - 1)],
        },
        "connectors": {
            "continuity": continuity,
            "maximum_position_gap_m": max(item["gap_m"] for item in continuity),
            "heading_jumps": heading_jumps,
            "maximum_heading_jump_deg": max(item["heading_change_deg"] for item in heading_jumps),
            "joins_over_10_degrees": [item for item in heading_jumps if item["heading_change_deg"] > 10.0],
            "note": "Nonzero heading jumps identify joins that require smoothing before an executable mission can be produced.",
        },
        "curvature": {"turn_segments": turn_metrics},
        "intersections": {
            "non_simple_segments": self_intersections,
            "nonadjacent_route_intersection_count": len(route_crossings),
            "nonadjacent_route_intersections": route_crossings,
            "note": "Crossing already-cut perimeter/headland ground is permitted; each crossing still requires replay review.",
        },
        "stripe_detours": stripe_detour_meta,
        "decision_summary": {
            "coverage_gap_preclassified_manual_only": False,
            "limited_outer_overhang_accepted_for_design_review": True,
            "blades_off_physical_verification_required": True,
        },
    }
    OUT_METRICS.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")

    # WGS84 GeoJSON with segment lineage. This is explicitly not a mission file.
    features = []
    cumulative = 0.0
    for sequence, segment in enumerate(segments, start=1):
        start_m, end_m = cumulative, cumulative + segment["line"].length
        cumulative = end_m
        properties = {
            "status": STATUS,
            "not_executable": True,
            "sequence": sequence,
            "segment_id": segment["segment_id"],
            "segment_type": segment["segment_type"],
            "phase": segment["phase"],
            "source": segment["source"],
            "note": segment["note"],
            "forward_only": True,
            "length_m": segment["line"].length,
            "cumulative_start_m": start_m,
            "cumulative_end_m": end_m,
            "speed_mps": SPEED_MPS,
        }
        features.append({"type": "Feature", "id": segment["segment_id"], "properties": properties, "geometry": mapping(unproject(segment["line"]))})
    geojson = {
        "type": "FeatureCollection",
        "name": "AREA-OVER-ROAD Checkpoint 4 detailed draft revision 002",
        "properties": {"status": STATUS, "not_executable": True, "coordinate_reference_system": "EPSG:4326"},
        "features": features,
    }
    OUT_GEOJSON.write_text(json.dumps(geojson, indent=2) + "\n", encoding="utf-8")

    with OUT_CSV.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = ["status", "not_executable", "sequence", "segment_id", "segment_type", "phase", "source", "note", "forward_only", "length_m", "cumulative_start_m", "cumulative_end_m", "speed_mps"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for feature in features:
            writer.writerow(feature["properties"])

    # Static audit map: route, planned coverage, residual gaps, and conservative envelope overhang.
    fig, (ax, gap_ax, pole_ax) = plt.subplots(1, 3, figsize=(18.0, 9.0), dpi=180, gridspec_kw={"width_ratios": [1.05, 1.0, 1.0]})
    fig.suptitle("Checkpoint 4 — selected Alternative A detailed draft", x=0.055, ha="left", fontsize=18, fontweight="bold")
    fig.text(0.055, 0.93, STATUS, fontsize=12, color="#b91c1c", fontweight="bold")

    colors = {"access": "#7c3aed", "connector": "#9333ea", "pole_loop": "#ea580c", "perimeter": "#16a34a", "stripe": "#2563eb", "stripe_with_pole_detour": "#0f766e", "headland_turn": "#dc2626"}
    for target in (ax, gap_ax, pole_ax):
        target.add_patch(PolygonPatch(list(outer.exterior.coords), closed=True, facecolor="#f8fafc", edgecolor="#15803d", linewidth=1.5))
        target.add_patch(PolygonPatch(list(hole.exterior.coords), closed=True, facecolor="white", edgecolor="#b91c1c", linewidth=1.4, hatch="///"))
        target.set_aspect("equal", adjustable="box")
        target.grid(True, color="#e2e8f0", linewidth=0.45)
        target.set_xlabel("Local east (m)")
    ax.set_ylabel("Local north (m)")
    for segment in segments:
        x, y = segment["line"].xy
        ax.plot(x, y, color=colors[segment["segment_type"]], linewidth=1.4 if segment["segment_type"] != "access" else 1.1, alpha=0.95)
        midpoint = segment["line"].interpolate(0.5, normalized=True)
        ax.text(midpoint.x, midpoint.y, segment["segment_id"], fontsize=5.7, color="#0f172a")
    ax.set_title("Auditable centerline and segment IDs", fontsize=12, fontweight="bold")
    ax.text(0.02, 0.02, f"{len(segments)} segments  |  {metrics['route']['total_length_m']:.1f} m  |  {metrics['route']['estimated_motion_time_min']:.1f} min\nRed: 1.4 m headland turns  |  Orange: pole loop\nPurple: unsmoothed connectors", transform=ax.transAxes, fontsize=8, va="bottom", bbox={"boxstyle": "round,pad=.35", "fc": "white", "ec": "#94a3b8", "alpha": .93})

    if not uncovered.is_empty:
        geoms = list(uncovered.geoms) if hasattr(uncovered, "geoms") else [uncovered]
        for gap_number, geometry in enumerate(sorted(geoms, key=lambda item: item.area, reverse=True), start=1):
            if geometry.geom_type == "Polygon":
                gap_ax.add_patch(PolygonPatch(list(geometry.exterior.coords), closed=True, facecolor="#facc15", edgecolor="#a16207", linewidth=.7, alpha=.85))
                if geometry.area >= 0.08:
                    gap_ax.text(geometry.centroid.x, geometry.centroid.y, f"G{gap_number}\n{geometry.area:.2f}", ha="center", va="center", fontsize=6, color="#713f12", fontweight="bold")
    for segment in segments:
        if segment["segment_type"] != "access":
            x, y = segment["line"].xy
            gap_ax.plot(x, y, color="#334155", linewidth=.55, alpha=.7)
    gap_ax.set_title("Coverage map\nYellow: conservative 38-in gaps", fontsize=12, fontweight="bold")
    gap_ax.text(0.02, 0.02, f"38-in coverage: {metrics['coverage']['planned_38in_coverage_percent']:.2f}%\n38-in gap: {metrics['coverage']['planned_38in_uncovered_m2']:.2f} m²\n42-in gap: {metrics['coverage']['actual_42in_cut_uncovered_m2']:.2f} m²\nNo gap is yet accepted as manual-only", transform=gap_ax.transAxes, fontsize=8, va="bottom", bbox={"boxstyle": "round,pad=.35", "fc": "white", "ec": "#94a3b8", "alpha": .93})

    overhang = envelope_union.difference(outer)
    geoms = list(overhang.geoms) if hasattr(overhang, "geoms") else [overhang]
    for geometry in geoms:
        if geometry.geom_type == "Polygon":
            pole_ax.add_patch(PolygonPatch(list(geometry.exterior.coords), closed=True, facecolor="#fca5a5", edgecolor="#b91c1c", linewidth=.45, alpha=.45))
    for segment in segments:
        if segment["segment_type"] in ("perimeter", "pole_loop", "stripe_with_pole_detour", "connector"):
            x, y = segment["line"].xy
            pole_ax.plot(x, y, color=colors[segment["segment_type"]], linewidth=1.2, alpha=.9)
    pole_ax.set_xlim(pole_center.x - 5.0, pole_center.x + 5.0)
    pole_ax.set_ylim(pole_center.y - 5.0, pole_center.y + 5.0)
    pole_ax.set_title("Pole detail\nRed: conservative deck + chute overhang", fontsize=12, fontweight="bold")
    pole_ax.text(0.02, 0.02, f"Loop centerline to grass edge: {metrics['containment_and_clearance']['pole_loop_centerline_to_outer_boundary_m']:.3f} m\nLoop centerline to pole exclusion: {metrics['containment_and_clearance']['pole_loop_centerline_to_pole_exclusion_m']:.3f} m\n42-in deck/pole overlap: {metrics['containment_and_clearance']['physical_cut_intersection_with_pole_exclusion_m2']:.3f} m²\nWorst-case deck+chute/pole overlap: {metrics['containment_and_clearance']['conservative_deck_plus_chute_intersection_with_pole_exclusion_m2']:.3f} m²", transform=pole_ax.transAxes, fontsize=8, va="bottom", bbox={"boxstyle": "round,pad=.35", "fc": "white", "ec": "#94a3b8", "alpha": .93})
    fig.text(.5, .018, "Non-executable design geometry. Purple joins and all red/yellow findings require owner review before mission generation.", ha="center", fontsize=9, color="#334155")
    fig.tight_layout(rect=(.02, .045, .99, .90))
    fig.savefig(OUT_PNG, bbox_inches="tight")
    plt.close(fig)

    # Dependency-free interactive replay, using local projected coordinates.
    replay_segments = []
    for segment in segments:
        replay_segments.append({
            "id": segment["segment_id"],
            "type": segment["segment_type"],
            "phase": segment["phase"],
            "length": segment["line"].length,
            "points": [[round(x, 4), round(y, 4)] for x, y in segment["line"].coords],
            "color": colors[segment["segment_type"]],
        })
    replay_data = {
        "status": STATUS,
        "segments": replay_segments,
        "outer": [[round(x, 4), round(y, 4)] for x, y in outer.exterior.coords],
        "hole": [[round(x, 4), round(y, 4)] for x, y in hole.exterior.coords],
        "summary": metrics["coverage"],
    }
    safe_json = json.dumps(replay_data).replace("</", "<\\/")
    html_text = f"""<!doctype html>
<html lang=\"en\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">
<title>Checkpoint 4 interactive replay — revision 002</title>
<style>body{{font:15px system-ui;margin:0;background:#f8fafc;color:#0f172a}}header,main{{max-width:1100px;margin:auto;padding:18px}}h1{{margin:.1rem 0}}.status{{color:#b91c1c;font-weight:800}}.card{{background:white;border:1px solid #cbd5e1;border-radius:10px;padding:14px}}svg{{width:100%;height:650px;background:#fff}}input{{width:100%}}.row{{display:flex;gap:18px;flex-wrap:wrap}}.row span{{min-width:170px}}button{{padding:8px 14px}}code{{font-size:13px}}</style></head>
<body><header><h1>Checkpoint 4 — Alternative A replay</h1><div class=\"status\">{html.escape(STATUS)}</div><p>Non-executable design replay. Drag the slider or press Play to follow the proposed forward-only sequence.</p></header>
<main><div class=\"card\"><svg id=\"map\" viewBox=\"0 0 900 650\"></svg><input id=\"slider\" type=\"range\" min=\"0\" max=\"1000\" value=\"0\"><div class=\"row\"><button id=\"play\">Play</button><span>Segment: <code id=\"segment\">S001</code></span><span>Phase: <code id=\"phase\">access in</code></span><span>Progress: <code id=\"progress\">0.0 m</code></span></div></div>
<p><strong>Legend:</strong> green perimeter, orange pole loop, blue/teal stripes, red headland turns, purple access/connectors. The replay shows centerlines; use the static overview for gaps and envelope overhang.</p></main>
<script>const data={safe_json};const svg=document.getElementById('map'),slider=document.getElementById('slider');const all=[...data.outer,...data.hole,...data.segments.flatMap(s=>s.points)];const xs=all.map(p=>p[0]),ys=all.map(p=>p[1]);const minX=Math.min(...xs)-2,maxX=Math.max(...xs)+2,minY=Math.min(...ys)-2,maxY=Math.max(...ys)+2;const sx=x=>40+(x-minX)/(maxX-minX)*820,sy=y=>620-(y-minY)/(maxY-minY)*580;const ns='http://www.w3.org/2000/svg';function path(points){{return points.map((p,i)=>(i?'L':'M')+sx(p[0]).toFixed(1)+','+sy(p[1]).toFixed(1)).join(' ')}}function add(points,stroke,width,fill='none',dash=''){{let p=document.createElementNS(ns,'path');p.setAttribute('d',path(points));p.setAttribute('stroke',stroke);p.setAttribute('stroke-width',width);p.setAttribute('fill',fill);if(dash)p.setAttribute('stroke-dasharray',dash);svg.appendChild(p);return p}}add(data.outer,'#15803d',2,'#f0fdf4');add(data.hole,'#b91c1c',2,'white');const routeEls=data.segments.map(s=>add(s.points,s.color,2,'none',s.type==='connector'?'6 4':''));const dot=document.createElementNS(ns,'circle');dot.setAttribute('r',6);dot.setAttribute('fill','#111827');svg.appendChild(dot);let total=data.segments.reduce((a,s)=>a+s.length,0),timer=null;function update(){{let target=+slider.value/1000*total,walk=0,chosen=data.segments[0],pt=chosen.points[0];routeEls.forEach(e=>e.setAttribute('opacity','.16'));for(let i=0;i<data.segments.length;i++){{let s=data.segments[i];if(target<=walk+s.length){{chosen=s;routeEls[i].setAttribute('opacity','1');let local=Math.max(0,target-walk),acc=0;pt=s.points[0];for(let j=1;j<s.points.length;j++){{let a=s.points[j-1],b=s.points[j],d=Math.hypot(b[0]-a[0],b[1]-a[1]);if(local<=acc+d){{let t=d?((local-acc)/d):0;pt=[a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])];break}}acc+=d;pt=b}}break}}walk+=s.length;routeEls[i].setAttribute('opacity','1')}}dot.setAttribute('cx',sx(pt[0]));dot.setAttribute('cy',sy(pt[1]));document.getElementById('segment').textContent=chosen.id+' / '+chosen.type;document.getElementById('phase').textContent=chosen.phase;document.getElementById('progress').textContent=target.toFixed(1)+' / '+total.toFixed(1)+' m'}}slider.oninput=update;document.getElementById('play').onclick=()=>{{if(timer){{clearInterval(timer);timer=null;return}}timer=setInterval(()=>{{slider.value=(+slider.value+2)%1001;update()}},30)}};update();</script></body></html>"""
    OUT_HTML.write_text(html_text, encoding="utf-8")


if __name__ == "__main__":
    main()
