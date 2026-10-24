#!/usr/bin/env python3
"""Build high-level, non-executable strategy alternatives for Checkpoint 3."""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, Polygon as PolygonPatch
from shapely.geometry import LineString, Polygon, shape
from shapely.ops import transform, unary_union


STATUS = "DRAFT - UNTESTED IN THE FIELD"
SPACING_M = 0.9652
CUT_WIDTH_M = 0.9652
DECK_WIDTH_M = 1.0668
TURN_RADIUS_M = 1.4
SPEED_MPS = 1.0
ROOT = Path(__file__).resolve().parents[5]
SITE = ROOT / "field_testing/sites/62_Collins_multi_boundary_20260915"
INVENTORY = SITE / "site_inventory/62_Collins_site_inventory.geojson"
MOWABLE = SITE / (
    "site_inventory/assets/AREA-OVER-ROAD/revisions/"
    "rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_mowable_area.geojson"
)
OUTPUT_PNG = Path(__file__).with_name("checkpoint_3_strategy_alternatives_rev001.png")
OUTPUT_JSON = Path(__file__).with_name("checkpoint_3_strategy_metrics_rev001.json")


def arrow_on_line(ax, line: LineString, color: str, fraction: float = 0.55, size: int = 11):
    a = line.interpolate(max(0.0, fraction - 0.035), normalized=True)
    b = line.interpolate(min(1.0, fraction + 0.035), normalized=True)
    ax.add_patch(
        FancyArrowPatch(
            (a.x, a.y),
            (b.x, b.y),
            arrowstyle="-|>",
            mutation_scale=size,
            linewidth=1.5,
            color=color,
            zorder=7,
        )
    )


def main() -> None:
    inventory = json.loads(INVENTORY.read_text(encoding="utf-8"))
    geographic = shape(json.loads(MOWABLE.read_text(encoding="utf-8"))["features"][0]["geometry"])
    ref_lat = geographic.centroid.y
    ref_lon = geographic.centroid.x
    east_scale = 111_320.0 * math.cos(math.radians(ref_lat))
    north_scale = 110_540.0

    def project(geometry):
        return transform(
            lambda lon, lat, z=None: ((lon - ref_lon) * east_scale, (lat - ref_lat) * north_scale),
            geometry,
        )

    area = project(geographic)
    outer = Polygon(area.exterior)
    hole = Polygon(area.interiors[0])
    coords = np.asarray(area.exterior.coords)[:-1]
    values, vectors = np.linalg.eigh(np.cov(coords.T))
    long_axis = vectors[:, np.argmax(values)]
    if long_axis[1] < 0:
        long_axis = -long_axis
    cross_axis = np.array([-long_axis[1], long_axis[0]])
    angle_deg = math.degrees(math.atan2(long_axis[1], long_axis[0])) % 180.0
    compass_a = (90.0 - angle_deg) % 360.0
    compass_b = (compass_a + 180.0) % 360.0

    outer_route = outer.buffer(-DECK_WIDTH_M / 2.0, join_style=1).exterior
    pole_center = np.array([hole.centroid.x, hole.centroid.y])
    theta = np.linspace(0.0, 2.0 * math.pi, 180)
    pole_loop = LineString(
        [(pole_center[0] + TURN_RADIUS_M * math.cos(t), pole_center[1] + TURN_RADIUS_M * math.sin(t)) for t in theta]
    )

    def stripe_groups(phase: float):
        cross_values = coords @ cross_axis
        lower, upper = float(np.min(cross_values)), float(np.max(cross_values))
        index = math.floor((lower - phase) / SPACING_M) - 1
        groups = []
        while True:
            offset = phase + index * SPACING_M
            if offset > upper + SPACING_M:
                break
            source = LineString(
                [tuple(long_axis * distance + cross_axis * offset) for distance in (-200.0, 200.0)]
            )
            intersection = area.intersection(source)
            pieces = list(intersection.geoms) if hasattr(intersection, "geoms") else [intersection]
            pieces = [piece for piece in pieces if piece.geom_type == "LineString" and piece.length > 0.25]
            if pieces:
                points = [np.asarray(point) for piece in pieces for point in piece.coords]
                groups.append(
                    {
                        "offset": offset,
                        "parts": pieces,
                        "low": min(points, key=lambda point: point @ long_axis),
                        "high": max(points, key=lambda point: point @ long_axis),
                    }
                )
            index += 1
        return groups

    best = None
    half_cut = CUT_WIDTH_M / 2.0
    for phase in np.linspace(0.0, SPACING_M, 40, endpoint=False):
        groups = stripe_groups(float(phase))
        parts = [part for group in groups for part in group["parts"]]
        swaths = [outer_route.buffer(half_cut, cap_style=2, join_style=2), pole_loop.buffer(half_cut, cap_style=2)]
        swaths.extend(part.buffer(half_cut, cap_style=2) for part in parts)
        covered = unary_union(swaths).intersection(area)
        uncovered = area.difference(covered).area
        stripe_length = sum(part.length for part in parts)
        candidate = (uncovered, stripe_length, float(phase), groups, covered)
        if best is None or candidate[:2] < best[:2]:
            best = candidate

    uncovered_area, stripe_length, phase, groups, covered = best
    lane_count = len(groups)
    split_lane_count = sum(1 for group in groups if len(group["parts"]) > 1)

    feature_by_id = {
        feature["properties"].get("asset_id"): feature
        for feature in inventory["features"]
        if feature["properties"].get("asset_id")
    }
    f_out = project(shape(feature_by_id["ACCESS-FRONT-NORTH-OUTBOUND"]["geometry"]))
    o_out = project(shape(feature_by_id["ACCESS-OVERROAD-NORTH-INBOUND"]["geometry"]))
    access_one_way = f_out.distance(o_out)
    base_length = outer_route.length + pole_loop.length + stripe_length + 2.0 * access_one_way
    coverage_pct = covered.area / area.area * 100.0
    min_boundary_clearance = pole_loop.distance(outer.boundary)
    min_obstacle_clearance = pole_loop.distance(hole)

    adjacent_order = list(range(lane_count))
    skip_order = [index for residue in range(3) for index in range(residue, lane_count, 3)]

    def endpoints(order, alternate=True):
        result = []
        for sequence, index in enumerate(order):
            low = groups[index]["low"]
            high = groups[index]["high"]
            if alternate and sequence % 2:
                result.append((high, low))
            else:
                result.append((low, high))
        return result

    adjacent_endpoints = endpoints(adjacent_order)
    skip_endpoints = endpoints(skip_order)
    adjacent_gaps = [float(np.linalg.norm(adjacent_endpoints[i][1] - adjacent_endpoints[i + 1][0])) for i in range(lane_count - 1)]
    skip_gaps = [float(np.linalg.norm(skip_endpoints[i][1] - skip_endpoints[i + 1][0])) for i in range(lane_count - 1)]
    adjacent_connector_length = sum(max(6.0, gap * 1.15) for gap in adjacent_gaps)
    skip_connector_length = sum(max(math.pi * TURN_RADIUS_M, gap * 1.10) for gap in skip_gaps)
    perimeter_return_length = (lane_count - 1) * outer_route.length / 2.0

    alternatives = [
        {
            "id": "A",
            "name": "Adjacent-lane snake",
            "order": adjacent_order,
            "route_length_m": base_length + adjacent_connector_length,
            "turns_or_connectors": f"{lane_count - 1} compact end turns; {split_lane_count} affected stripe bypass; perimeter pole loop",
            "risk": "Adjacent 0.965 m lane changes need compact omega/keyhole-like turns; feasibility at 1.4 m is unproven.",
        },
        {
            "id": "B",
            "name": "Three-lane skip phases",
            "order": skip_order,
            "route_length_m": base_length + skip_connector_length,
            "turns_or_connectors": "radius-friendly skip turns plus two phase relocations; affected stripe bypass; perimeter pole loop",
            "risk": "Phase changes create long relocation connectors and may cross finished cuts or require perimeter travel.",
        },
        {
            "id": "C",
            "name": "Same-direction stripes with perimeter returns",
            "order": adjacent_order,
            "route_length_m": base_length + perimeter_return_length,
            "turns_or_connectors": f"{lane_count - 1} perimeter returns; no immediate adjacent-lane U-turn; affected stripe bypass; pole loop",
            "risk": "Longest route; repeated perimeter traffic may rut and each return must merge safely into the next stripe.",
        },
    ]
    for alternative in alternatives:
        alternative["estimated_motion_time_min"] = alternative["route_length_m"] / SPEED_MPS / 60.0
        alternative["modeled_coverage_percent"] = coverage_pct
        alternative["modeled_uncovered_area_m2"] = uncovered_area
        alternative["minimum_centerline_boundary_clearance_m"] = min_boundary_clearance
        alternative["minimum_centerline_obstacle_clearance_m"] = min_obstacle_clearance

    metrics = {
        "status": STATUS,
        "scope": "AREA-OVER-ROAD high-level strategy comparison only",
        "not_executable": True,
        "inputs": {
            "planned_cut_width_m": CUT_WIDTH_M,
            "lane_spacing_m": SPACING_M,
            "physical_deck_width_m": DECK_WIDTH_M,
            "turn_radius_m": TURN_RADIUS_M,
            "speed_mps": SPEED_MPS,
            "long_axis_math_degrees": angle_deg,
            "long_axis_compass_headings_degrees": [compass_a, compass_b],
            "lane_count": lane_count,
            "split_lane_count_at_obstacle": split_lane_count,
            "access_round_trip_m": 2.0 * access_one_way,
            "common_cut_path_before_connectors_m": base_length,
        },
        "coverage_model": {
            "covered_area_m2": covered.area,
            "approved_area_m2": area.area,
            "coverage_percent": coverage_pct,
            "uncovered_area_m2": uncovered_area,
            "note": "Approximate buffered-swath model before detailed connector and asymmetric chute validation.",
        },
        "alternatives": alternatives,
    }
    OUTPUT_JSON.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")

    fig, axes = plt.subplots(1, 3, figsize=(17.0, 8.0), dpi=180, sharex=True, sharey=True)
    fig.suptitle("Checkpoint 3 — forward-only strategy alternatives", x=0.06, ha="left", fontsize=18, fontweight="bold")
    fig.text(0.06, 0.925, STATUS, fontsize=12, color="#b91c1c", fontweight="bold")

    for ax, alternative in zip(axes, alternatives):
        ax.add_patch(
            PolygonPatch(
                list(outer.exterior.coords),
                closed=True,
                facecolor="#f0fdf4",
                edgecolor="#15803d",
                linewidth=1.8,
                alpha=0.9,
            )
        )
        ax.add_patch(
            PolygonPatch(
                list(hole.exterior.coords),
                closed=True,
                facecolor="white",
                edgecolor="#b91c1c",
                linewidth=1.7,
                hatch="///",
            )
        )
        ox, oy = outer_route.xy
        ax.plot(ox, oy, color="#16a34a", linewidth=1.8)
        arrow_on_line(ax, outer_route, "#16a34a", 0.25)
        px, py = pole_loop.xy
        ax.plot(px, py, color="#ea580c", linewidth=1.8)
        arrow_on_line(ax, pole_loop, "#ea580c", 0.30)

        order = alternative["order"]
        same_direction = alternative["id"] == "C"
        sequence_endpoints = []
        for sequence, index in enumerate(order, start=1):
            group = groups[index]
            for part in group["parts"]:
                x, y = part.xy
                ax.plot(x, y, color="#2563eb", linewidth=1.4)
            low = group["low"]
            high = group["high"]
            if same_direction or sequence % 2 == 1:
                start, end = low, high
            else:
                start, end = high, low
            sequence_endpoints.append((start, end))
            mid = (start + end) / 2.0
            ax.add_patch(
                FancyArrowPatch(
                    tuple(mid - (end - start) * 0.04),
                    tuple(mid + (end - start) * 0.04),
                    arrowstyle="-|>",
                    mutation_scale=10,
                    linewidth=1.2,
                    color="#2563eb",
                )
            )
            ax.text(mid[0], mid[1], str(sequence), fontsize=7, color="#1e3a8a", fontweight="bold")

        if alternative["id"] in ("A", "B"):
            for start_end, next_start_end in zip(sequence_endpoints[:-1], sequence_endpoints[1:]):
                ax.add_patch(
                    FancyArrowPatch(
                        tuple(start_end[1]),
                        tuple(next_start_end[0]),
                        connectionstyle="arc3,rad=0.22",
                        arrowstyle="->",
                        mutation_scale=8,
                        linewidth=0.9,
                        linestyle="--",
                        color="#7c3aed" if alternative["id"] == "A" else "#dc2626",
                        alpha=0.85,
                    )
                )
        else:
            side_x = max(point[0] for point in outer.exterior.coords) + 1.5
            for sequence in range(lane_count - 1):
                first = sequence_endpoints[sequence][1]
                nxt = sequence_endpoints[sequence + 1][0]
                ax.plot([first[0], side_x, side_x, nxt[0]], [first[1], first[1], nxt[1], nxt[1]], color="#7c3aed", linewidth=0.8, linestyle="--", alpha=0.65)

        ax.plot([f_out.x, o_out.x], [f_out.y, o_out.y], color="#7c3aed", linewidth=1.6, linestyle=":")
        ax.add_patch(
            FancyArrowPatch(
                (f_out.x, f_out.y),
                (o_out.x, o_out.y),
                arrowstyle="<|-|>",
                mutation_scale=10,
                linewidth=1.0,
                color="#7c3aed",
            )
        )
        ax.set_title(f"{alternative['id']}. {alternative['name']}", fontsize=12, fontweight="bold")
        ax.text(
            0.02,
            0.02,
            f"1 Access  2 Perimeter + pole loop\n3 Numbered stripes  4 Exit\n"
            f"~{alternative['route_length_m']:.0f} m  |  ~{alternative['estimated_motion_time_min']:.1f} min\n"
            f"coverage ~{coverage_pct:.1f}%  |  uncovered ~{uncovered_area:.1f} m²",
            transform=ax.transAxes,
            fontsize=7.5,
            va="bottom",
            bbox={"boxstyle": "round,pad=0.3", "fc": "white", "ec": "#94a3b8", "alpha": 0.92},
        )
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, color="#e2e8f0", linewidth=0.5)
        ax.set_xlabel("Local east (m)")
    axes[0].set_ylabel("Local north (m)")
    fig.text(
        0.5,
        0.02,
        "High-level comparison only. Connectors and pole bypasses are conceptual and must be rebuilt and validated in Checkpoint 4.",
        ha="center",
        fontsize=9,
        color="#334155",
    )
    fig.tight_layout(rect=(0.02, 0.045, 0.99, 0.90))
    fig.savefig(OUTPUT_PNG, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
