#!/usr/bin/env python3
"""Build non-runnable boundary/obstacle validation paths for owner review."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon, mapping, shape
from shapely.ops import unary_union

from build_62_collins_site_inventory_rev003_20260926 import (
    DECK_RADIUS_M,
    REPO,
    SITE,
    largest_polygon,
    load_frame,
    to_xy,
)


SOURCE = (
    SITE
    / "site_inventory/revisions/rev_003_20260926_REVIEW_ONLY"
    / "62_Collins_site_inventory_rev003_REVIEW_ONLY.geojson"
)
OUT = SITE / "mission_plans/20260927_boundary_obstacle_validation_REVIEW_ONLY"
GEOJSON_OUT = OUT / "62_Collins_validation_paths_REVIEW_ONLY.geojson"
AUDIT_OUT = OUT / "62_Collins_validation_paths_audit.csv"
REPORT_OUT = OUT / "62_Collins_validation_paths_report.json"
PNG_OUT = OUT / "62_Collins_validation_paths_REVIEW.png"
README_OUT = OUT / "README.md"

# A first validation run should stay farther away than a finished mowing path.
REVIEW_MARGIN_M = 0.3048  # 12 inches
TRACTOR_CENTER_CLEARANCE_M = DECK_RADIUS_M + REVIEW_MARGIN_M
NOMINAL_SPEED_MPS = 0.35
NOMINAL_LOOKAHEAD_M = 1.0


def line_parts(geometry):
    if geometry.geom_type == "LineString":
        return [geometry]
    if geometry.geom_type == "MultiLineString":
        return list(geometry.geoms)
    return []


def polygon_parts(geometry):
    if geometry.geom_type == "Polygon":
        return [geometry]
    if geometry.geom_type == "MultiPolygon":
        return list(geometry.geoms)
    return []


def closed_line(ring):
    coordinates = list(ring.coords)
    if coordinates[0] != coordinates[-1]:
        coordinates.append(coordinates[0])
    return LineString(coordinates)


def feature(path_id, kind, asset_id, geometry, properties):
    return {
        "type": "Feature",
        "id": path_id,
        "properties": {
            "path_id": path_id,
            "kind": kind,
            "asset_id": asset_id,
            "status": "REVIEW_ONLY_NOT_RUNNABLE",
            "deck_radius_m": DECK_RADIUS_M,
            "additional_review_margin_m": REVIEW_MARGIN_M,
            "tractor_center_clearance_m": TRACTOR_CENTER_CLEARANCE_M,
            "nominal_speed_mps": NOMINAL_SPEED_MPS,
            "nominal_lookahead_m": NOMINAL_LOOKAHEAD_M,
            **properties,
        },
        "geometry": mapping(geometry),
    }


def draw_polygon(axis, geometry, color, alpha=0.10, width=1.2):
    for part in polygon_parts(geometry):
        x, y = part.exterior.xy
        axis.fill(x, y, facecolor=color, edgecolor=color, alpha=alpha, linewidth=width)
        for interior in part.interiors:
            x, y = interior.xy
            axis.fill(x, y, facecolor="white", edgecolor=color, alpha=1.0, linewidth=width)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    frame = load_frame()
    source = json.loads(SOURCE.read_text(encoding="utf-8"))

    areas = {}
    obstacles = {}
    obstacle_meta = {}
    for item in source["features"]:
        props = item["properties"]
        role = props.get("geometry_role")
        asset_id = props.get("asset_id")
        if role == "candidate_mowable_boundary":
            areas[asset_id] = largest_polygon(to_xy(frame, shape(item["geometry"])))
        elif role in {"candidate_mowing_exclusion", "mowing_exclusion"}:
            # The rev-3 candidate role is the active alternative; the one plain
            # mowing_exclusion is the carried-forward tree.
            obstacles[asset_id] = largest_polygon(to_xy(frame, shape(item["geometry"])))
            obstacle_meta[asset_id] = props

    expanded_obstacles = {
        asset_id: geometry.buffer(TRACTOR_CENTER_CLEARANCE_M)
        for asset_id, geometry in obstacles.items()
    }

    records = []
    plot_paths = []

    # The exterior path is inset far enough to keep the deck edge 12 inches
    # inside the proposed mowing boundary. Holes are evaluated separately.
    for area_id, area in areas.items():
        shell = Polygon(area.exterior)
        inset = shell.buffer(-TRACTOR_CENTER_CLEARANCE_M)
        for component_number, component in enumerate(sorted(polygon_parts(inset), key=lambda p: p.area, reverse=True), 1):
            path = closed_line(component.exterior)
            conflict = path.intersection(unary_union(list(expanded_obstacles.values())))
            conflict_length = conflict.length
            status = "clear" if conflict_length <= 0.05 else "review_conflict_with_obstacle_clearance"
            path_id = f"{area_id}-OUTER-{component_number:02d}"
            record = {
                "path_id": path_id,
                "kind": "outer_boundary_validation",
                "asset_id": area_id,
                "related_area": area_id,
                "length_m": round(path.length, 3),
                "minimum_outer_edge_clearance_m": round(path.distance(shell.boundary), 3),
                "conflict_length_m": round(conflict_length, 3),
                "outside_safe_area_length_m": 0.0,
                "review_status": status,
            }
            records.append(record)
            plot_paths.append((record, path))

    # The obstacle path is outside the inferred physical keep-out by half the
    # deck width plus a 12-inch first-run margin.
    for asset_id, exclusion in sorted(obstacles.items()):
        related_area = obstacle_meta[asset_id].get("related_area")
        if related_area not in areas:
            related_area = min(areas, key=lambda key: exclusion.centroid.distance(areas[key]))
        shell = Polygon(areas[related_area].exterior)
        safe_center_region = shell.buffer(-DECK_RADIUS_M)
        other_expanded = unary_union([
            geometry.buffer(DECK_RADIUS_M)
            for other_id, geometry in obstacles.items()
            if other_id != asset_id
        ])
        ring_geometry = exclusion.buffer(TRACTOR_CENTER_CLEARANCE_M).boundary
        for component_number, path in enumerate(line_parts(ring_geometry), 1):
            outside_length = path.difference(safe_center_region).length
            other_conflict = path.intersection(other_expanded).length if not other_expanded.is_empty else 0.0
            if outside_length > 0.05:
                status = "review_conflict_outside_mowable_edge"
            elif other_conflict > 0.05:
                status = "review_conflict_with_other_obstacle"
            else:
                status = "clear"
            path_id = f"{asset_id}-LOOP-{component_number:02d}"
            record = {
                "path_id": path_id,
                "kind": "obstacle_validation_loop",
                "asset_id": asset_id,
                "related_area": related_area,
                "length_m": round(path.length, 3),
                "minimum_outer_edge_clearance_m": round(path.distance(shell.boundary), 3),
                "conflict_length_m": round(other_conflict, 3),
                "outside_safe_area_length_m": round(outside_length, 3),
                "review_status": status,
            }
            records.append(record)
            plot_paths.append((record, path))

    collection = {
        "type": "FeatureCollection",
        "name": "62 Collins boundary and obstacle validation paths review",
        "properties": {
            "status": "REVIEW_ONLY_NOT_RUNNABLE",
            "source": str(SOURCE.relative_to(REPO)).replace("\\", "/"),
            "mission_file_generated": False,
            "launcher_generated": False,
        },
        "features": [
            feature(
                record["path_id"],
                record["kind"],
                record["asset_id"],
                to_lonlat_geometry(frame, path),
                record,
            )
            for record, path in plot_paths
        ],
    }
    GEOJSON_OUT.write_text(json.dumps(collection, indent=2) + "\n", encoding="utf-8")

    columns = list(records[0].keys())
    with AUDIT_OUT.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(records)

    clear = [record for record in records if record["review_status"] == "clear"]
    conflicts = [record for record in records if record["review_status"] != "clear"]
    report = {
        "status": "REVIEW_ONLY_NOT_RUNNABLE",
        "source": str(SOURCE.relative_to(REPO)).replace("\\", "/"),
        "deck_radius_m": DECK_RADIUS_M,
        "additional_review_margin_m": REVIEW_MARGIN_M,
        "tractor_center_clearance_m": TRACTOR_CENTER_CLEARANCE_M,
        "nominal_speed_mps_for_future_field_build": NOMINAL_SPEED_MPS,
        "nominal_lookahead_m_for_future_field_build": NOMINAL_LOOKAHEAD_M,
        "path_count": len(records),
        "clear_path_count": len(clear),
        "conflict_path_count": len(conflicts),
        "conflicts": conflicts,
        "limitations": [
            "No controller-format mission file or launcher was generated.",
            "Paths are independent loops; ordering, start poses, and between-path connectors are intentionally unresolved.",
            "Obstacle identity and active geometry must be owner-reviewed before field authorization.",
            "The 12-inch margin is a conservative first-pass review value, not a final mowing clearance.",
            "Any path marked as a conflict requires geometry or route changes before it can be field-tested.",
        ],
    }
    REPORT_OUT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    fig, axis = plt.subplots(figsize=(14, 10), dpi=170)
    for area in areas.values():
        draw_polygon(axis, area, "#2563eb", alpha=0.08, width=1.0)
    for exclusion in obstacles.values():
        draw_polygon(axis, exclusion, "#dc2626", alpha=0.20, width=1.0)
    for record, path in plot_paths:
        clear_path = record["review_status"] == "clear"
        color = "#059669" if clear_path else "#dc2626"
        style = "-" if record["kind"] == "outer_boundary_validation" else "--"
        x, y = path.xy
        axis.plot(x, y, color=color, linestyle=style, linewidth=1.7, alpha=0.9)
        midpoint = path.interpolate(path.length / 2.0)
        axis.annotate(record["asset_id"], (midpoint.x, midpoint.y), fontsize=6, color=color)
    axis.plot([], [], color="#059669", label="Clear in geometric audit")
    axis.plot([], [], color="#dc2626", label="Conflict requiring review")
    axis.plot([], [], color="#222", linestyle="-", label="Outer boundary validation path")
    axis.plot([], [], color="#222", linestyle="--", label="Obstacle validation loop")
    axis.set_title("62 Collins boundary and obstacle validation paths — REVIEW ONLY")
    axis.set_xlabel("Local east (m)")
    axis.set_ylabel("Local north (m)")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, linewidth=0.3, alpha=0.3)
    axis.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(PNG_OUT, bbox_inches="tight")
    plt.close(fig)

    README_OUT.write_text(
        "# 62 Collins boundary and obstacle validation paths — review only\n\n"
        "This package is the first planning step toward a supervised, blades-off field validation. It is deliberately not runnable.\n\n"
        f"- Tractor-center paths use the 0.5334 m deck half-width plus a {REVIEW_MARGIN_M:.4f} m (12-inch) first-run margin.\n"
        f"- Future nominal test settings are {NOMINAL_SPEED_MPS:.2f} m/s and {NOMINAL_LOOKAHEAD_M:.2f} m lookahead; they are not yet authorized.\n"
        "- Solid paths validate proposed outer boundaries; dashed paths validate obstacle clearances.\n"
        "- Green paths pass the initial geometry-only audit. Red paths conflict with an edge or another obstacle and must be resolved.\n"
        "- No transitions, mission text file, dashboard, or launcher have been generated.\n"
        "- After owner review, the next build should select route direction/start points, use only approved paths, and add demonstrated connectors.\n\n"
        "Review the PNG, GeoJSON, and audit CSV before authorizing a field build.\n",
        encoding="utf-8",
        newline="\n",
    )

    print(json.dumps({
        "output": str(OUT),
        "paths": len(records),
        "clear": len(clear),
        "conflicts": len(conflicts),
    }, indent=2))


def to_lonlat_geometry(frame, geometry):
    """Convert a local XY line to GeoJSON lon/lat coordinates."""
    return LineString([tuple(reversed(frame.ll(x, y))) for x, y in geometry.coords])


if __name__ == "__main__":
    main()
