#!/usr/bin/env python3
"""Build the 62 Collins three-area Zamboni coverage REVIEW package.

This produces review artifacts only.  It deliberately does not emit the
five-column Pure Pursuit mission format or a launcher.
"""

from __future__ import annotations

import csv
import hashlib
import heapq
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from shapely.geometry import GeometryCollection, LineString, MultiLineString, Point, Polygon, mapping, shape
from shapely.ops import substring, transform, unary_union


REPO = Path(__file__).resolve().parents[2]
SITE = REPO / "field_testing/sites/62_Collins_multi_boundary_20260915"
PLAN = SITE / "planning/20261005_zamboni_coverage"
OUT = PLAN / "generated_review_20261005"
INVENTORY = SITE / (
    "site_inventory/revisions/rev_003_20260926_REVIEW_ONLY/"
    "62_Collins_site_inventory_rev003_REVIEW_ONLY.geojson"
)
SOURCE_ROUTE = SITE / (
    "mission_plans/20260928_outer_perimeter_field_test/"
    "submitted_edits_review_REVIEW_ONLY/62_Collins_consolidated_route_REVIEW_ONLY.json"
)

PLANNER_DIR = REPO / "tractor_rpi/pure-pursuit/mission_planning"
sys.path.insert(0, str(PLANNER_DIR))
from site_planner_common_20260724 import dubins_candidates  # noqa: E402


STATUS = "REVIEW ONLY — NOT APPROVED FOR FIELD USE"
AREA_ORDER = ["AREA-BACKYARD-GARDENS", "AREA-FRONT-YARD", "AREA-OVER-ROAD"]
AREA_NAMES = {
    "AREA-BACKYARD-GARDENS": "Backyard and gardens",
    "AREA-FRONT-YARD": "Front yard",
    "AREA-OVER-ROAD": "Over the road",
}
ENTRY_ROUTE = "ROUTE-BASE-TO-BACKYARD"
BETWEEN_ROUTES = {
    "AREA-BACKYARD-GARDENS": "ROUTE-BACKYARD-TO-FRONT",
    "AREA-FRONT-YARD": "ROUTE-FRONT-TO-OVERROAD",
}
RETURN_ROUTES = ["ROUTE-OVERROAD-TO-FRONT", "ROUTE-FRONT-TO-BASE"]
REJECTED_OBSTACLE = "OBS-NEW-012"
DECK_WIDTH_M = 1.0668
HALF_DECK_M = DECK_WIDTH_M / 2.0
LANE_SPACING_M = 0.9652
HEADLAND_PASSES = 3
TARGET_TURN_RADIUS_M = 1.90
EVIDENCE_FALLBACK_RADIUS_M = 1.63
ZAMBONI_COLUMN_STEP = 4
OBSTACLE_REVIEW_MARGIN_M = 0.15
SAFE_GEOMETRY_GUARD_M = 0.02
WAYPOINT_SPACING_M = 0.25
MIN_STRIPE_LENGTH_M = 1.25


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def line_parts(geometry):
    if geometry.is_empty:
        return []
    if isinstance(geometry, LineString):
        return [geometry]
    if isinstance(geometry, MultiLineString):
        return list(geometry.geoms)
    if isinstance(geometry, GeometryCollection):
        result = []
        for item in geometry.geoms:
            result.extend(line_parts(item))
        return result
    return []


def polygon_parts(geometry):
    if geometry.is_empty:
        return []
    if isinstance(geometry, Polygon):
        return [geometry]
    return [item for item in getattr(geometry, "geoms", []) if isinstance(item, Polygon)]


def densify(points, spacing=WAYPOINT_SPACING_M):
    line = LineString(points)
    if line.length <= spacing:
        return [tuple(line.coords[0]), tuple(line.coords[-1])]
    count = max(1, math.ceil(line.length / spacing))
    return [tuple(line.interpolate(line.length * i / count).coords[0]) for i in range(count + 1)]


def yaw(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def start_pose(points):
    return points[0][0], points[0][1], yaw(points[0], points[1])


def end_pose(points):
    return points[-1][0], points[-1][1], yaw(points[-2], points[-1])


def local_frame(origin_lat, origin_lon):
    east_scale = 111_320.0 * math.cos(math.radians(origin_lat))
    north_scale = 110_540.0

    def to_xy(lon, lat, z=None):
        return (lon - origin_lon) * east_scale, (lat - origin_lat) * north_scale

    def to_ll(x, y):
        return origin_lat + y / north_scale, origin_lon + x / east_scale

    return to_xy, to_ll


def load_sources():
    inventory = json.loads(INVENTORY.read_text(encoding="utf-8"))
    source = json.loads(SOURCE_ROUTE.read_text(encoding="utf-8"))
    origin_lat = float(source["origin"]["lat"])
    origin_lon = float(source["origin"]["lon"])
    to_xy, to_ll = local_frame(origin_lat, origin_lon)

    areas = {}
    obstacles = {}
    obstacle_area = {}
    transitions = {}
    for feature in inventory["features"]:
        props = feature.get("properties", {})
        asset = props.get("asset_id")
        role = props.get("geometry_role")
        category = props.get("category")
        geometry = transform(to_xy, shape(feature["geometry"]))
        if category == "mowable_area" and role == "candidate_mowable_boundary":
            areas[asset] = geometry
        elif (
            category == "obstacle"
            and role in ("candidate_mowing_exclusion", "mowing_exclusion")
            and asset != REJECTED_OBSTACLE
        ):
            obstacles[asset] = geometry
            obstacle_area[asset] = props.get("related_area")
        elif category == "transition_route" and role == "recorded_centerline_between_areas":
            transitions[asset] = geometry

    missing = [asset for asset in AREA_ORDER if asset not in areas]
    if missing:
        raise ValueError(f"missing areas: {missing}")
    if len(obstacles) != 15:
        raise ValueError(f"expected 15 active obstacles after O012 removal, found {len(obstacles)}")
    return inventory, source, areas, obstacles, obstacle_area, transitions, to_ll


def rotate_ring(points, anchor, clockwise):
    body = list(points[:-1] if points[0] == points[-1] else points)
    signed = sum(
        a[0] * b[1] - b[0] * a[1]
        for a, b in zip(body, body[1:] + body[:1])
    )
    is_clockwise = signed < 0
    if is_clockwise != clockwise:
        body.reverse()
    index = min(range(len(body)), key=lambda i: math.dist(body[i], anchor))
    body = body[index:] + body[:index]
    body.append(body[0])
    return densify(body)


def astar_connector(start, goal, safe, resolution=0.60):
    """Return a contained routing-only fallback, never claimed as radius-valid."""
    minx, miny, maxx, maxy = safe.bounds
    pad = 1.0
    minx, miny, maxx, maxy = minx - pad, miny - pad, maxx + pad, maxy + pad

    def cell(point):
        return round((point[0] - minx) / resolution), round((point[1] - miny) / resolution)

    def xy(node):
        return minx + node[0] * resolution, miny + node[1] * resolution

    start_node, goal_node = cell(start), cell(goal)
    valid_cache = {}

    def valid(node):
        if node not in valid_cache:
            valid_cache[node] = safe.buffer(0.02).covers(Point(xy(node)))
        return valid_cache[node]

    def nearest_valid(node):
        if valid(node):
            return node
        for radius in range(1, 8):
            candidates = []
            for dx in range(-radius, radius + 1):
                for dy in (-radius, radius):
                    candidates.append((node[0] + dx, node[1] + dy))
            for dy in range(-radius + 1, radius):
                for dx in (-radius, radius):
                    candidates.append((node[0] + dx, node[1] + dy))
            candidates = [candidate for candidate in candidates if valid(candidate)]
            if candidates:
                return min(candidates, key=lambda candidate: math.dist(xy(candidate), xy(node)))
        return None

    start_node = nearest_valid(start_node)
    goal_node = nearest_valid(goal_node)
    if start_node is None or goal_node is None:
        return None
    queue = [(0.0, start_node)]
    cost = {start_node: 0.0}
    parent = {}
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while queue:
        _, current = heapq.heappop(queue)
        if current == goal_node:
            break
        for dx, dy in directions:
            nxt = current[0] + dx, current[1] + dy
            if not valid(nxt):
                continue
            segment = LineString([xy(current), xy(nxt)])
            if not safe.buffer(0.02).covers(segment):
                continue
            new_cost = cost[current] + math.hypot(dx, dy) * resolution
            if new_cost >= cost.get(nxt, float("inf")):
                continue
            cost[nxt] = new_cost
            parent[nxt] = current
            heuristic = math.dist(xy(nxt), xy(goal_node))
            heapq.heappush(queue, (new_cost + heuristic, nxt))
    if goal_node not in cost:
        return None
    nodes = [goal_node]
    while nodes[-1] != start_node:
        nodes.append(parent[nodes[-1]])
    nodes.reverse()
    points = [start] + [xy(node) for node in nodes] + [goal]
    simplified = LineString(points).simplify(0.25, preserve_topology=False)
    if safe.buffer(0.02).covers(simplified):
        points = list(simplified.coords)
    return densify(points)


def boundary_connector(start, goal, safe):
    """Route a review-only relocation along a containing polygon boundary.

    This is intentionally not claimed as curvature-valid. It avoids drawing a
    misleading straight chord when a contained Dubins connector is unavailable.
    """

    def joined(*groups):
        result = []
        for group in groups:
            for point in group:
                point = tuple(point)
                if not result or math.dist(result[-1], point) > 1e-6:
                    result.append(point)
        return result

    def forward_arc(ring, first, second):
        length = ring.length
        if second >= first:
            return list(substring(ring, first, second).coords)
        return joined(
            list(substring(ring, first, length).coords),
            list(substring(ring, 0.0, second).coords),
        )

    safe_test = safe.buffer(0.002)
    candidates = []
    for polygon in polygon_parts(safe):
        component_test = polygon.buffer(0.025)
        if not component_test.covers(Point(start)) or not component_test.covers(Point(goal)):
            continue
        for ring_coords in [polygon.exterior, *polygon.interiors]:
            ring = LineString(ring_coords.coords)
            first = ring.project(Point(start))
            second = ring.project(Point(goal))
            first_point = tuple(ring.interpolate(first).coords[0])
            second_point = tuple(ring.interpolate(second).coords[0])
            if not safe_test.covers(LineString([start, first_point])):
                continue
            if not safe_test.covers(LineString([second_point, goal])):
                continue
            forward = forward_arc(ring, first, second)
            backward = list(reversed(forward_arc(ring, second, first)))
            for arc in (forward, backward):
                points = joined([start, first_point], arc, [second_point, goal])
                line = LineString(points)
                if safe_test.covers(line):
                    candidates.append((line.length, points))
    if not candidates:
        return None
    return densify(min(candidates, key=lambda item: item[0])[1])


def make_connector(previous, following, safe, label):
    if not previous or not following:
        return None
    start = previous[-1]
    goal = following[0]
    if math.dist(start, goal) < 0.06:
        return None
    for radius in (TARGET_TURN_RADIUS_M, EVIDENCE_FALLBACK_RADIUS_M):
        candidates = sorted(
            dubins_candidates(end_pose(previous), start_pose(following), radius, 0.40),
            key=lambda item: float(item["length_m"]),
        )
        for candidate in candidates:
            points = [(float(item[0]), float(item[1])) for item in candidate["points"]]
            line = LineString(points)
            if safe.buffer(0.002).covers(line):
                return {
                    "points": densify(points),
                    "mode": candidate["mode"],
                    "radius_m": radius,
                    "radius_validated": True,
                    "contained": True,
                    "label": label,
                }
    fallback = boundary_connector(start, goal, safe)
    if fallback:
        return {
            "points": fallback,
            "mode": "CONTAINED_BOUNDARY_RELOCATION_REQUIRES_TURN_SMOOTHING",
            "radius_m": None,
            "radius_validated": False,
            "contained": True,
            "label": label,
        }
    # If even a contained routing path cannot be found, display the direct
    # chord in red solely to make the missing connection conspicuous.
    return {
        "points": [start, goal],
        "mode": "UNRESOLVED_STRAIGHT_REVIEW_CHORD",
        "radius_m": None,
        "radius_validated": False,
        "contained": safe.buffer(0.002).covers(LineString([start, goal])),
        "label": label,
    }


def add_piece(pieces, *, points, kind, label, area=None, cutting=True, **extra):
    points = densify(points)
    piece = {
        "sequence": len(pieces) + 1,
        "kind": kind,
        "label": label,
        "area": area,
        "cutting": cutting,
        "points": points,
        **extra,
    }
    pieces.append(piece)
    return piece


def connect_and_add(pieces, previous, following, safe, area, label, turns):
    connector = make_connector(previous, following, safe, label)
    if not connector:
        return None
    piece = add_piece(
        pieces,
        points=connector["points"],
        kind="soft_turn" if connector["radius_validated"] else "review_connector",
        label=label,
        area=area,
        cutting=True,
        connector_mode=connector["mode"],
        turn_radius_m=connector["radius_m"],
        radius_validated=connector["radius_validated"],
        contained=connector["contained"],
    )
    turns.append({
        "sequence": piece["sequence"],
        "area": area,
        "label": label,
        "mode": connector["mode"],
        "turn_radius_m": connector["radius_m"],
        "radius_validated": connector["radius_validated"],
        "contained": connector["contained"],
        "length_m": round(LineString(connector["points"]).length, 3),
    })
    return piece


def headland_paths(safe):
    result = []
    for pass_number in range(1, HEADLAND_PASSES + 1):
        inset = (pass_number - 1) * LANE_SPACING_M
        geometry = safe.buffer(-inset, join_style="round") if inset else safe
        polygons = sorted(polygon_parts(geometry), key=lambda polygon: polygon.area, reverse=True)
        if not polygons:
            continue
        # The user requested three major-section boundary runs.  Obstacle
        # boundaries remain exclusions, but are not counted as extra section
        # boundary runs.
        result.append({
            "pass": pass_number,
            "component": 1,
            "boundary_type": "outer",
            "points": list(polygons[0].exterior.coords),
        })
    return result


def candidate_stripes(stripe_domain, residual):
    minx, miny, maxx, maxy = stripe_domain.bounds
    start_x = math.floor(minx / LANE_SPACING_M) * LANE_SPACING_M + LANE_SPACING_M / 2.0
    columns = []
    column = 0
    x = start_x
    while x <= maxx + 1e-9:
        cutter = LineString([(x, miny - 5.0), (x, maxy + 5.0)])
        parts = sorted(line_parts(stripe_domain.intersection(cutter)), key=lambda line: line.centroid.y)
        accepted = []
        for part_index, part in enumerate(parts, 1):
            if part.length < MIN_STRIPE_LENGTH_M:
                continue
            contribution = part.buffer(HALF_DECK_M, cap_style="flat").intersection(residual).area
            if contribution < 0.12:
                continue
            accepted.append({
                "column": column,
                "part": part_index,
                "x": x,
                "line": part,
                "contribution_m2": contribution,
            })
        if accepted:
            columns.append((column, accepted))
        column += 1
        x += LANE_SPACING_M
    return columns


def stripe_cells(columns):
    """Track one stripe segment per column through obstacle split/merge events."""
    cells = []
    active = []
    for column, segments in columns:
        segments = sorted(segments, key=lambda item: item["line"].centroid.y)
        candidates = []
        for active_index, cell_index in enumerate(active):
            previous = cells[cell_index][-1]
            for segment_index, segment in enumerate(segments):
                distance = previous["line"].distance(segment["line"])
                if distance <= 2.75:
                    candidates.append((distance, active_index, segment_index, cell_index))
        used_active = set()
        used_segments = set()
        new_active = []
        for _distance, active_index, segment_index, cell_index in sorted(candidates):
            if active_index in used_active or segment_index in used_segments:
                continue
            cells[cell_index].append(segments[segment_index])
            used_active.add(active_index)
            used_segments.add(segment_index)
            new_active.append(cell_index)
        for segment_index, segment in enumerate(segments):
            if segment_index in used_segments:
                continue
            cells.append([segment])
            new_active.append(len(cells) - 1)
        active = new_active
    return sorted(cells, key=lambda cell: (-len(cell), min(item["x"] for item in cell)))


def zamboni_order(cell):
    # A four-column cycle provides about 3.86 m between ordinary reversals,
    # supporting the requested 1.90 m soft turn.  Every remainder group is
    # subsequently completed, so no stripe is omitted.
    return [
        item
        for remainder in range(ZAMBONI_COLUMN_STEP)
        for item in sorted(cell, key=lambda segment: (segment["column"], segment["part"]))
        if item["column"] % ZAMBONI_COLUMN_STEP == remainder
    ]


def plan_area(asset, area, area_obstacles, entry_point, pieces, turns, stripe_rows):
    obstacle_union = unary_union(list(area_obstacles.values())) if area_obstacles else GeometryCollection()
    cut_target = area.difference(obstacle_union.buffer(OBSTACLE_REVIEW_MARGIN_M))
    safe = area.buffer(-HALF_DECK_M, join_style="round")
    if not obstacle_union.is_empty:
        safe = safe.difference(
            obstacle_union.buffer(
                HALF_DECK_M + OBSTACLE_REVIEW_MARGIN_M + SAFE_GEOMETRY_GUARD_M,
                join_style="round",
            )
        )
    safe = safe.buffer(0)
    if safe.is_empty:
        raise ValueError(f"{asset} has no deck-center-safe area")

    prior_points = None
    for item in headland_paths(safe):
        anchor = entry_point if prior_points is None else prior_points[-1]
        options = []
        for clockwise in (True, False):
            ring = rotate_ring(item["points"], anchor, clockwise)
            if prior_points is None:
                score = math.dist(anchor, ring[0])
                connector = None
            else:
                connector = make_connector(prior_points, ring, safe, "headland splice")
                score = LineString(connector["points"]).length if connector else 0.0
                if connector and not connector["radius_validated"]:
                    score += 1000.0
            options.append((score, clockwise, ring, connector))
        _, clockwise, ring, connector = min(options, key=lambda item: item[0])
        if prior_points is not None and connector:
            connect_and_add(
                pieces, prior_points, ring, safe, asset,
                f"{asset} headland pass {item['pass']} splice", turns,
            )
        add_piece(
            pieces,
            points=ring,
            kind="headland",
            label=(
                f"{asset} boundary pass {item['pass']} "
                f"{item['boundary_type']} component {item['component']}"
            ),
            area=asset,
            cutting=True,
            pass_number=item["pass"],
            clockwise=clockwise,
        )
        prior_points = ring

    headland_lines = [
        LineString(piece["points"])
        for piece in pieces
        if piece["area"] == asset and piece["kind"] == "headland"
    ]
    headland_swath = unary_union([line.buffer(HALF_DECK_M) for line in headland_lines])
    residual = cut_target.difference(headland_swath)
    inset = 2.0 * LANE_SPACING_M
    # Reserve the outer three-pass headland for turning, while allowing stripe
    # segments to approach obstacle exclusions at the normal deck clearance.
    stripe_domain = area.buffer(-(HALF_DECK_M + inset), join_style="round")
    if not obstacle_union.is_empty:
        stripe_domain = stripe_domain.difference(
            obstacle_union.buffer(
                HALF_DECK_M + OBSTACLE_REVIEW_MARGIN_M + SAFE_GEOMETRY_GUARD_M,
                join_style="round",
            )
        )
    columns = candidate_stripes(stripe_domain, residual) if not stripe_domain.is_empty else []
    cells = stripe_cells(columns)
    ordered_count = 0
    for cell_number, cell in enumerate(cells, 1):
        ordered = zamboni_order(cell)
        for cell_index, stripe in enumerate(ordered, 1):
            ordered_count += 1
            raw = list(stripe["line"].coords)
            northbound = cell_index % 2 == 1
            if northbound and raw[0][1] > raw[-1][1]:
                raw.reverse()
            if not northbound and raw[0][1] < raw[-1][1]:
                raw.reverse()
            points = densify(raw)
            if prior_points is not None:
                connect_and_add(
                    pieces, prior_points, points, safe, asset,
                    f"{asset} to cell {cell_number} stripe C{stripe['column']:03d}-{stripe['part']}", turns,
                )
            group_number = stripe["column"] % ZAMBONI_COLUMN_STEP + 1
            group = f"G{group_number}-mod{ZAMBONI_COLUMN_STEP}"
            piece = add_piece(
                pieces,
                points=points,
                kind="stripe",
                label=f"{asset} cell {cell_number} stripe C{stripe['column']:03d}-{stripe['part']}",
                area=asset,
                cutting=True,
                stripe_column=stripe["column"],
                stripe_part=stripe["part"],
                stripe_cell=cell_number,
                zamboni_group=group,
                direction="northbound" if northbound else "southbound",
            )
            stripe_rows.append({
                "route_sequence": piece["sequence"],
                "area": asset,
                "area_name": AREA_NAMES[asset],
                "stripe_cell": cell_number,
                "zamboni_group": group,
                "stripe_column": stripe["column"],
                "stripe_part": stripe["part"],
                "direction": piece["direction"],
                "length_m": round(LineString(points).length, 3),
                "new_gap_coverage_m2": round(stripe["contribution_m2"], 3),
            })
            prior_points = points

    return {
        "cut_target": cut_target,
        "safe": safe,
        "end_point": prior_points[-1] if prior_points else entry_point,
        "headland_count": len(headland_lines),
        "stripe_count": ordered_count,
        "stripe_cell_count": len(cells),
    }


def add_recorded_transition(pieces, geometry, label, route_id):
    points = densify(list(geometry.coords))
    return add_piece(
        pieces,
        points=points,
        kind="recorded_transition",
        label=label,
        area=None,
        cutting=False,
        source_route=route_id,
    )


def geometry_to_ll(geometry, to_ll):
    return transform(lambda x, y, z=None: (to_ll(x, y)[1], to_ll(x, y)[0]), geometry)


def write_csv(path, rows, fields):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def build_geojson(pieces, area_results, obstacles, to_ll):
    features = []
    for asset, result in area_results.items():
        print(f"  coverage union: {AREA_NAMES[asset]}", flush=True)
        features.append({
            "type": "Feature",
            "properties": {"asset_id": asset, "category": "mowable_area", "status": STATUS},
            "geometry": mapping(geometry_to_ll(result["cut_target"], to_ll)),
        })
    for asset, geometry in obstacles.items():
        features.append({
            "type": "Feature",
            "properties": {"asset_id": asset, "category": "obstacle", "status": STATUS},
            "geometry": mapping(geometry_to_ll(geometry, to_ll)),
        })
    for piece in pieces:
        features.append({
            "type": "Feature",
            "properties": {key: value for key, value in piece.items() if key != "points"},
            "geometry": mapping(geometry_to_ll(LineString(piece["points"]), to_ll)),
        })
    return {"type": "FeatureCollection", "name": "62_Collins_zamboni_coverage_REVIEW_ONLY", "features": features}


def plot_polygon(axis, geometry, **kwargs):
    first = True
    for polygon in polygon_parts(geometry):
        x, y = polygon.exterior.xy
        axis.fill(x, y, label=kwargs.pop("label", None) if first else None, **kwargs)
        first = False
        for ring in polygon.interiors:
            x, y = ring.xy
            axis.fill(x, y, color="white")


def write_static(pieces, area_results, obstacles, obstacle_area, coverage):
    fig, axes = plt.subplots(2, 2, figsize=(18, 15))
    panels = [(None, "Complete three-section sequence")] + [
        (asset, AREA_NAMES[asset]) for asset in AREA_ORDER
    ]
    colors = {
        "headland": "#0f8a5f",
        "stripe": "#1565c0",
        "soft_turn": "#f59e0b",
        "review_connector": "#dc2626",
        "recorded_transition": "#6b7280",
    }
    for axis, (selected, title) in zip(axes.flat, panels):
        for asset, result in area_results.items():
            if selected is not None and asset != selected:
                continue
            plot_polygon(axis, result["cut_target"], color="#dff3df", alpha=0.5)
        for obstacle, geometry in obstacles.items():
            related = obstacle_area[obstacle]
            if selected is not None and related != selected:
                continue
            plot_polygon(axis, geometry, color="#ef4444", alpha=0.28)
            center = geometry.centroid
            axis.text(center.x, center.y, obstacle.replace("OBS-NEW-", "O"), fontsize=7, ha="center")
        for piece in pieces:
            if selected is not None and piece["area"] != selected:
                continue
            if selected is None or piece["area"] == selected:
                line = LineString(piece["points"])
                x, y = line.xy
                axis.plot(
                    x, y,
                    color=colors.get(piece["kind"], "#111827"),
                    linewidth=2.0 if piece["kind"] in ("headland", "stripe") else 1.2,
                    linestyle="--" if piece["kind"] == "review_connector" else "-",
                    alpha=0.95,
                )
                if piece["kind"] == "stripe":
                    axis.text(line.centroid.x, line.centroid.y, str(piece.get("stripe_column", "")), fontsize=5, alpha=0.7)
        axis.set_title(title)
        axis.set_aspect("equal", adjustable="datalim")
        axis.set_xlabel("Local east (m)")
        axis.set_ylabel("Local north (m)")
        axis.grid(True, alpha=0.18)
    legend = [
        Line2D([0], [0], color="#0f8a5f", lw=2, label="Three boundary/headland passes"),
        Line2D([0], [0], color="#1565c0", lw=2, label="North–south stripes"),
        Line2D([0], [0], color="#f59e0b", lw=2, label="Radius-validated soft turn"),
        Line2D([0], [0], color="#dc2626", lw=2, ls="--", label="Connector requiring redesign"),
        Line2D([0], [0], color="#6b7280", lw=2, label="Recorded between-area route"),
        Line2D([0], [0], color="#ef4444", lw=8, alpha=0.3, label="Obstacle exclusion (O012 removed)"),
    ]
    fig.legend(handles=legend, loc="lower center", ncol=3)
    fig.suptitle(
        "62 Collins Zamboni coverage — REVIEW ONLY — NOT APPROVED FOR FIELD USE\n"
        f"3 boundary passes per section, 38-in stripe spacing, 42-in deck · planned coverage {coverage['whole_site']['coverage_percent']:.2f}%",
        fontsize=16,
    )
    fig.tight_layout(rect=(0, 0.06, 1, 0.95))
    fig.savefig(OUT / "62_Collins_zamboni_coverage_REVIEW_ONLY.png", dpi=180)
    plt.close(fig)


def write_interactive(pieces, area_results, obstacles, obstacle_area, cumulative, report):
    payload = {
        "status": STATUS,
        "deckWidth": DECK_WIDTH_M,
        "length": cumulative[-1] if cumulative else 0,
        "coverage": report["whole_site"],
        "areas": [
            {
                "id": asset,
                "name": AREA_NAMES[asset],
                "polygons": [list(poly.exterior.coords) for poly in polygon_parts(result["cut_target"])],
            }
            for asset, result in area_results.items()
        ],
        "obstacles": [
            {"id": asset.replace("OBS-NEW-", "O"), "area": obstacle_area[asset], "points": list(geometry.exterior.coords), "center": list(geometry.centroid.coords)[0]}
            for asset, geometry in obstacles.items()
        ],
        "pieces": [
            {
                "sequence": piece["sequence"],
                "kind": piece["kind"],
                "label": piece["label"],
                "area": piece["area"],
                "points": [[round(x, 3), round(y, 3)] for x, y in piece["points"]],
                "radiusValidated": piece.get("radius_validated"),
                "turnRadius": piece.get("turn_radius_m"),
                "startDistance": round(piece["start_distance_m"], 3),
                "endDistance": round(piece["end_distance_m"], 3),
            }
            for piece in pieces
        ],
    }
    encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False)
    html = r'''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>62 Collins Zamboni coverage review</title><style>
html,body{height:100%;margin:0;font:14px system-ui;background:#f8fafc;color:#111827}.top,.bottom{display:flex;gap:12px;align-items:center;flex-wrap:wrap;padding:9px 12px;background:#fff;border-bottom:1px solid #d1d5db}.bottom{border-top:1px solid #d1d5db;border-bottom:0}.warn{font-weight:700;color:#b91c1c}.tag{padding:2px 7px;border-radius:999px;background:#e5e7eb}canvas{display:block;width:100%;height:calc(100vh - 120px);background:#fff}button,select,input{font:inherit}#timeline{flex:1;min-width:200px}.sw{display:inline-block;width:18px;border-top:3px solid;margin-right:4px}.small{color:#4b5563}
</style></head><body><div class="top"><span class="warn">REVIEW ONLY — NOT APPROVED FOR FIELD USE</span><span id="summary" class="tag"></span><label><input id="headlands" type="checkbox" checked> boundary passes</label><label><input id="stripes" type="checkbox" checked> stripes</label><label><input id="turns" type="checkbox" checked> turns</label><label><input id="obstacles" type="checkbox" checked> obstacles</label><label><input id="deck" type="checkbox" checked> deck</label><select id="area"><option value="all">All sections</option><option value="AREA-BACKYARD-GARDENS">Backyard/gardens</option><option value="AREA-FRONT-YARD">Front yard</option><option value="AREA-OVER-ROAD">Over road</option></select><button id="fit">Fit</button></div><canvas id="map"></canvas><div class="bottom"><button id="play">▶ Play</button><button id="back">−25 m</button><button id="forward">+25 m</button><select id="rate"><option value="5">5×</option><option value="15">15×</option><option value="30" selected>30×</option><option value="60">60×</option></select><input id="timeline" type="range" min="0" step="0.1"><span id="status" class="small"></span></div><script>
const D=__DATA__,C=document.getElementById('map'),X=C.getContext('2d'),$=id=>document.getElementById(id);let scale=1,ox=0,oy=0,distance=0,playing=false,last=null,drag=null;const colors={headland:'#0f8a5f',stripe:'#1565c0',soft_turn:'#f59e0b',review_connector:'#dc2626',recorded_transition:'#6b7280'};
function visible(p){const a=$('area').value;return a==='all'||p.area===a||(p.kind==='recorded_transition'&&a==='all')}function pt(p){return[ox+p[0]*scale,oy-p[1]*scale]}function path(points,color,width,dash=[]){if(points.length<2)return;X.save();X.beginPath();X.moveTo(...pt(points[0]));for(let i=1;i<points.length;i++)X.lineTo(...pt(points[i]));X.strokeStyle=color;X.lineWidth=width;X.setLineDash(dash);X.lineJoin='round';X.lineCap='round';X.stroke();X.restore()}function poly(points,fill,stroke){X.beginPath();X.moveTo(...pt(points[0]));for(let i=1;i<points.length;i++)X.lineTo(...pt(points[i]));X.closePath();X.fillStyle=fill;X.fill();X.strokeStyle=stroke;X.lineWidth=1;X.stroke()}
function activePiece(){return D.pieces.find(p=>distance>=p.startDistance&&distance<=p.endDistance)||D.pieces[D.pieces.length-1]}function pointAt(p,d){let remain=d-p.startDistance;for(let i=1;i<p.points.length;i++){const a=p.points[i-1],b=p.points[i],len=Math.hypot(b[0]-a[0],b[1]-a[1]);if(remain<=len)return[a[0]+(b[0]-a[0])*remain/len,a[1]+(b[1]-a[1])*remain/len,i];remain-=len}const q=p.points[p.points.length-1];return[q[0],q[1],p.points.length-1]}
function draw(){const r=C.getBoundingClientRect();X.clearRect(0,0,r.width,r.height);const av=$('area').value;D.areas.filter(a=>av==='all'||a.id===av).forEach(a=>a.polygons.forEach(p=>poly(p,'rgba(34,197,94,.08)','rgba(22,101,52,.35)')));if($('obstacles').checked)D.obstacles.filter(o=>av==='all'||o.area===av).forEach(o=>{poly(o.points,'rgba(239,68,68,.16)','#ef4444');const q=pt(o.center);X.fillStyle='#7f1d1d';X.font='11px system-ui';X.textAlign='center';X.fillText(o.id,q[0],q[1])});D.pieces.filter(visible).forEach(p=>{if(p.kind==='headland'&&!$('headlands').checked)return;if(p.kind==='stripe'&&!$('stripes').checked)return;if((p.kind==='soft_turn'||p.kind==='review_connector')&&!$('turns').checked)return;path(p.points,colors[p.kind]||'#111827',p.kind==='headland'||p.kind==='stripe'?2.2:1.5,p.kind==='review_connector'?[6,4]:[])});const p=activePiece();if(p){const q=pointAt(p,distance),xy=pt(q);if($('deck').checked){X.beginPath();X.arc(xy[0],xy[1],D.deckWidth/2*scale,0,Math.PI*2);X.fillStyle='rgba(21,101,192,.10)';X.fill();X.strokeStyle='#1565c0';X.stroke()}X.beginPath();X.arc(xy[0],xy[1],6,0,Math.PI*2);X.fillStyle='#16a34a';X.fill();X.strokeStyle='#fff';X.lineWidth=2;X.stroke();$('status').textContent=distance.toFixed(1)+' / '+D.length.toFixed(1)+' m · '+p.label+(p.turnRadius?' · radius '+p.turnRadius.toFixed(2)+' m':'')}}
function fit(){let pts=[];const av=$('area').value;D.areas.filter(a=>av==='all'||a.id===av).forEach(a=>a.polygons.forEach(p=>pts.push(...p)));if(!pts.length)D.pieces.filter(visible).forEach(p=>pts.push(...p.points));const r=C.getBoundingClientRect(),xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]),pad=35;scale=Math.min((r.width-2*pad)/(Math.max(...xs)-Math.min(...xs)),(r.height-2*pad)/(Math.max(...ys)-Math.min(...ys)));ox=pad-Math.min(...xs)*scale;oy=pad+Math.max(...ys)*scale;draw()}function resize(){const r=C.getBoundingClientRect(),q=devicePixelRatio||1;C.width=r.width*q;C.height=r.height*q;X.setTransform(q,0,0,q,0,0);fit()}function setD(v){distance=Math.max(0,Math.min(D.length,Number(v)));$('timeline').value=distance;draw()}function stop(){playing=false;last=null;$('play').textContent='▶ Play'}function animate(t){if(!playing)return;if(last!==null)setD(distance+(t-last)/1000*Number($('rate').value));last=t;if(distance>=D.length){stop();return}requestAnimationFrame(animate)}$('play').onclick=()=>{if(playing){stop();return}if(distance>=D.length)setD(0);playing=true;$('play').textContent='❚❚ Pause';requestAnimationFrame(animate)};$('back').onclick=()=>{stop();setD(distance-25)};$('forward').onclick=()=>{stop();setD(distance+25)};$('timeline').max=D.length;$('timeline').oninput=e=>{stop();setD(e.target.value)};$('fit').onclick=fit;$('area').onchange=fit;['headlands','stripes','turns','obstacles','deck'].forEach(id=>$(id).onchange=draw);C.addEventListener('wheel',e=>{e.preventDefault();const r=C.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,k=e.deltaY<0?1.15:1/1.15,n=Math.max(.2,Math.min(80,scale*k));ox=mx-(mx-ox)*n/scale;oy=my-(my-oy)*n/scale;scale=n;draw()},{passive:false});C.onpointerdown=e=>{C.setPointerCapture(e.pointerId);drag=[e.clientX,e.clientY]};C.onpointermove=e=>{if(!drag)return;ox+=e.clientX-drag[0];oy+=e.clientY-drag[1];drag=[e.clientX,e.clientY];draw()};C.onpointerup=()=>drag=null;$('summary').textContent=D.pieces.length+' route pieces · '+D.coverage.coverage_percent.toFixed(2)+'% planned coverage · O012 removed';new ResizeObserver(resize).observe(C);requestAnimationFrame(fit);
</script></body></html>'''.replace("__DATA__", encoded)
    (OUT / "62_Collins_zamboni_coverage_INTERACTIVE_REVIEW_ONLY.html").write_text(html, encoding="utf-8")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    print("Loading reconciled site geometry...", flush=True)
    inventory, source, areas, obstacles, obstacle_area, transitions, to_ll = load_sources()
    pieces = []
    turns = []
    stripe_rows = []
    area_results = {}

    entry = add_recorded_transition(pieces, transitions[ENTRY_ROUTE], "Base to backyard", ENTRY_ROUTE)
    current_point = entry["points"][-1]
    for index, asset in enumerate(AREA_ORDER):
        print(f"Planning {AREA_NAMES[asset]}...", flush=True)
        related = {key: geometry for key, geometry in obstacles.items() if obstacle_area[key] == asset}
        result = plan_area(asset, areas[asset], related, current_point, pieces, turns, stripe_rows)
        area_results[asset] = result
        print(
            f"  {result['headland_count']} headland loops, {result['stripe_count']} stripe segments",
            flush=True,
        )
        current_point = result["end_point"]
        route_id = BETWEEN_ROUTES.get(asset)
        if route_id:
            transition = add_recorded_transition(
                pieces, transitions[route_id], f"{AREA_NAMES[asset]} to next section", route_id
            )
            current_point = transition["points"][-1]
    for route_id in RETURN_ROUTES:
        transition = add_recorded_transition(pieces, transitions[route_id], f"Return via {route_id}", route_id)
        current_point = transition["points"][-1]

    cumulative = [0.0]
    audit_rows = []
    for piece in pieces:
        line = LineString(piece["points"])
        piece["start_distance_m"] = cumulative[-1]
        piece["end_distance_m"] = cumulative[-1] + line.length
        for point_index, point in enumerate(piece["points"]):
            if audit_rows and math.dist((audit_rows[-1]["east_m"], audit_rows[-1]["north_m"]), point) < 0.001:
                continue
            if point_index + 1 < len(piece["points"]):
                heading = yaw(point, piece["points"][point_index + 1])
            elif point_index:
                heading = yaw(piece["points"][point_index - 1], point)
            else:
                heading = 0.0
            lat, lon = to_ll(*point)
            if audit_rows:
                cumulative.append(cumulative[-1] + math.dist((audit_rows[-1]["east_m"], audit_rows[-1]["north_m"]), point))
            audit_rows.append({
                "review_point": len(audit_rows) + 1,
                "route_piece": piece["sequence"],
                "piece_point": point_index + 1,
                "segment_type": piece["kind"],
                "label": piece["label"],
                "area": piece["area"] or "between_areas",
                "east_m": round(point[0], 3),
                "north_m": round(point[1], 3),
                "lat": f"{lat:.9f}",
                "lon": f"{lon:.9f}",
                "yaw_rad": round(heading, 6),
                "lookahead_m": 1.5 if piece["kind"] in ("soft_turn", "review_connector") else 2.0,
                "speed_mps": "1.00",
                "cutting": piece["cutting"],
                "pass_number": piece.get("pass_number", ""),
                "stripe_column": piece.get("stripe_column", ""),
                "stripe_part": piece.get("stripe_part", ""),
                "zamboni_group": piece.get("zamboni_group", ""),
                "turn_radius_m": piece.get("turn_radius_m", ""),
                "radius_validated": piece.get("radius_validated", ""),
                "cumulative_m": round(cumulative[-1], 3),
            })

    coverage_areas = {}
    print("Computing deck coverage and clearance...", flush=True)
    total_target = 0.0
    total_covered = 0.0
    obstacle_checks = []
    for asset, result in area_results.items():
        raw_cutting_lines = [
            LineString(piece["points"])
            for piece in pieces
            if piece["area"] == asset and piece["kind"] in ("headland", "stripe", "soft_turn")
        ]
        cutting_lines = [
            line.simplify(0.10, preserve_topology=False)
            for line in raw_cutting_lines
        ]
        related_lines = unary_union(cutting_lines)
        clearance_lines = unary_union(raw_cutting_lines)
        swath = related_lines.buffer(HALF_DECK_M, cap_style="round", resolution=8)
        covered = swath.intersection(result["cut_target"])
        gaps = result["cut_target"].difference(swath)
        target_area = result["cut_target"].area
        covered_area = covered.area
        total_target += target_area
        total_covered += covered_area
        coverage_areas[asset] = {
            "name": AREA_NAMES[asset],
            "target_area_m2": round(target_area, 3),
            "covered_area_m2": round(covered_area, 3),
            "uncovered_area_m2": round(gaps.area, 3),
            "coverage_percent": round(100.0 * covered_area / target_area, 3),
            "headland_path_count": result["headland_count"],
            "stripe_count": result["stripe_count"],
        }
        for obstacle, geometry in obstacles.items():
            if obstacle_area[obstacle] != asset:
                continue
            centerline_clearance = clearance_lines.distance(geometry)
            obstacle_checks.append({
                "asset_id": obstacle,
                "related_area": asset,
                "minimum_centerline_clearance_m": round(centerline_clearance, 3),
                "minimum_deck_edge_clearance_m": round(centerline_clearance - HALF_DECK_M, 3),
                "deck_intersection": clearance_lines.buffer(HALF_DECK_M).intersects(geometry),
            })
    whole_site = {
        "target_area_m2": round(total_target, 3),
        "covered_area_m2": round(total_covered, 3),
        "uncovered_area_m2": round(total_target - total_covered, 3),
        "coverage_percent": round(100.0 * total_covered / total_target, 3),
    }
    unresolved = [turn for turn in turns if not turn["radius_validated"] or not turn["contained"]]
    contained_relocations = [turn for turn in unresolved if turn["contained"]]
    uncontained_or_missing = [turn for turn in unresolved if not turn["contained"]]
    report = {
        "status": STATUS,
        "design": {
            "area_order": AREA_ORDER,
            "headland_passes_per_area": HEADLAND_PASSES,
            "stripe_orientation": "north_south",
            "zamboni_order": "four-column cycle within each connected cell; all four remainder groups completed",
            "zamboni_column_step": ZAMBONI_COLUMN_STEP,
            "deck_width_m": DECK_WIDTH_M,
            "lane_spacing_m": LANE_SPACING_M,
            "nominal_overlap_m": round(DECK_WIDTH_M - LANE_SPACING_M, 4),
            "target_turn_radius_m": TARGET_TURN_RADIUS_M,
            "evidence_fallback_radius_m": EVIDENCE_FALLBACK_RADIUS_M,
            "obstacle_review_margin_m": OBSTACLE_REVIEW_MARGIN_M,
            "safe_geometry_guard_m": SAFE_GEOMETRY_GUARD_M,
            "speed_mps": 1.0,
        },
        "inventory": {
            "source": INVENTORY.relative_to(REPO).as_posix(),
            "active_obstacle_count": len(obstacles),
            "rejected_obstacle": REJECTED_OBSTACLE,
            "rejected_reason": "Owner confirmed no obstacle exists at O012.",
        },
        "areas": coverage_areas,
        "whole_site": whole_site,
        "route": {
            "piece_count": len(pieces),
            "review_point_count": len(audit_rows),
            "length_m": round(cumulative[-1], 3),
            "estimated_runtime_minutes_at_1mps": round(cumulative[-1] / 60.0, 2),
            "stripe_count": len(stripe_rows),
            "turn_count": len(turns),
            "unresolved_turn_count": len(unresolved),
        },
        "turn_validation": {
            "all_radius_validated": not unresolved,
            "radius_validated_count": len(turns) - len(unresolved),
            "contained_relocations_requiring_smoothing_count": len(contained_relocations),
            "uncontained_or_unresolved_count": len(uncontained_or_missing),
            "unresolved": unresolved,
        },
        "obstacle_clearance": obstacle_checks,
        "field_authorization": "NONE",
    }

    manifest = {
        "status": STATUS,
        "inputs": [
            {"path": INVENTORY.relative_to(REPO).as_posix(), "sha256": sha256(INVENTORY)},
            {"path": SOURCE_ROUTE.relative_to(REPO).as_posix(), "sha256": sha256(SOURCE_ROUTE)},
        ],
        "generator": Path(__file__).relative_to(REPO).as_posix(),
        "rules": [
            "O012 / OBS-NEW-012 is rejected and absent from active obstacles.",
            "Three boundary/headland passes precede stripes in every area.",
            "Stripes are north-south and use a four-column Zamboni cycle within connected cells.",
            "No field launcher or five-column mission is generated.",
        ],
    }
    print("Writing tabular and geometry artifacts...", flush=True)
    (OUT / "input_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    (OUT / "coverage_validation_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    geojson = build_geojson(pieces, area_results, obstacles, to_ll)
    (OUT / "62_Collins_zamboni_coverage_REVIEW_ONLY.geojson").write_text(
        json.dumps(geojson, separators=(",", ":")), encoding="utf-8"
    )
    write_csv(
        OUT / "stripe_order.csv", stripe_rows,
        ["route_sequence", "area", "area_name", "stripe_cell", "zamboni_group", "stripe_column", "stripe_part", "direction", "length_m", "new_gap_coverage_m2"],
    )
    write_csv(
        OUT / "turn_report.csv", turns,
        ["sequence", "area", "label", "mode", "turn_radius_m", "radius_validated", "contained", "length_m"],
    )
    write_csv(
        OUT / "review_waypoint_audit.csv", audit_rows,
        ["review_point", "route_piece", "piece_point", "segment_type", "label", "area", "east_m", "north_m", "lat", "lon", "yaw_rad", "lookahead_m", "speed_mps", "cutting", "pass_number", "stripe_column", "stripe_part", "zamboni_group", "turn_radius_m", "radius_validated", "cumulative_m"],
    )
    print("Rendering static preview...", flush=True)
    write_static(pieces, area_results, obstacles, obstacle_area, {"whole_site": whole_site})
    print("Writing interactive replay...", flush=True)
    write_interactive(pieces, area_results, obstacles, obstacle_area, cumulative, report)

    readme = f"""# 62 Collins Zamboni coverage — review only

**{STATUS}**

This package sequences the three mowing sections as backyard/gardens, front
yard, and over-road. Each section begins with three boundary/headland passes,
then north-south stripes. Ordinary reversals span four columns (about 3.86 m), supporting a 1.90 m soft
turn; all four column groups are completed later. Recorded
between-area routes connect the sections.

- Active obstacles: {len(obstacles)} (`O012` / `OBS-NEW-012` removed)
- Deck: {DECK_WIDTH_M:.4f} m (42 in)
- Stripe spacing: {LANE_SPACING_M:.4f} m (38 in)
- Nominal overlap: {DECK_WIDTH_M - LANE_SPACING_M:.4f} m (4 in)
- Target turn radius: {TARGET_TURN_RADIUS_M:.2f} m
- Evidence-based fallback radius: {EVIDENCE_FALLBACK_RADIUS_M:.2f} m
- Additional review margin outside obstacle exclusions: {OBSTACLE_REVIEW_MARGIN_M:.2f} m
- Planned route length: {cumulative[-1]:.1f} m
- Estimated time at 1.00 m/s: {cumulative[-1] / 60.0:.1f} minutes
- Planned deck coverage: {whole_site['coverage_percent']:.2f}%
- Uncovered review area: {whole_site['uncovered_area_m2']:.2f} m²
- Radius-validated soft turns: {len(turns) - len(unresolved)}
- Contained boundary relocations requiring curvature smoothing: {len(contained_relocations)}
- Uncontained or missing connections: {len(uncontained_or_missing)}

## Review files

- `62_Collins_zamboni_coverage_REVIEW_ONLY.png` — full-site and per-section static review.
- `62_Collins_zamboni_coverage_INTERACTIVE_REVIEW_ONLY.html` — replay, layer controls, section filter, and deck footprint.
- `coverage_validation_report.json` — coverage, clearance, and turn validation.
- `stripe_order.csv` — exact Zamboni stripe order.
- `turn_report.csv` — every connector and its radius/containment result.
- `review_waypoint_audit.csv` — dense review points and lineage; not a mission file.
- `62_Collins_zamboni_coverage_REVIEW_ONLY.geojson` — reconciled review geometry.
- `input_manifest.json` — source hashes and planning rules.

No launcher, approval flag, or executable five-column mission is present.
Red dashed connectors follow contained review routes where possible, but still
require curvature smoothing. Any uncontained connector is a hard review blocker.
Neither category is approved for a later field mission.
"""
    (OUT / "README.md").write_text(readme, encoding="utf-8")
    print(json.dumps({"output": str(OUT), "report": report}, indent=2))


if __name__ == "__main__":
    main()
