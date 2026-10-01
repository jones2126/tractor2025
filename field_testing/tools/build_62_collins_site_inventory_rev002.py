#!/usr/bin/env python3
"""Build 62 Collins site inventory revision 2 from Al's transition notes."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, mapping, shape
from shapely.ops import nearest_points, substring, transform, unary_union

from build_20260923_combined_coverage_review import REPO, REVIEW, SITE, Frame, polygons_of


REVISION = 2
REVISION_LABEL = "rev_002_20260925"
SITE_ID = "62_COLLINS"
REVIEW_DATE = "2026-09-25"
SOURCE_DATE = "2026-09-15"
DECK_WIDTH_M = 1.0668

OUT = SITE / "site_inventory"
HISTORY = OUT / "revisions" / REVISION_LABEL
REV1_GEOJSON = OUT / "revisions/rev_001_20260925/62_Collins_site_inventory.geojson"
REV1_MANIFEST = OUT / "revisions/rev_001_20260925/62_Collins_site_inventory_manifest.json"

CURRENT_GEOJSON = OUT / "62_Collins_site_inventory.geojson"
CURRENT_CSV = OUT / "62_Collins_site_inventory.csv"
CURRENT_MANIFEST = OUT / "62_Collins_site_inventory_manifest.json"
CURRENT_VALIDATION = OUT / "62_Collins_site_inventory_validation.json"
CURRENT_MAP = OUT / "62_Collins_site_inventory_map.png"
CURRENT_HTML = OUT / "62_Collins_site_inventory_map.html"
CURRENT_README = OUT / "README.md"

CSV_COLUMNS = [
    "inventory_revision", "asset_id", "asset_name", "category", "status",
    "field_validation", "geometry_roles", "source_date", "owner_review_date",
    "latitude", "longitude", "area_m2", "length_m", "start_location", "end_location",
    "directionality", "reverse_validated", "related_area", "from_access_point",
    "to_access_point", "source_reference", "notes",
]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_frame() -> Frame:
    with (REVIEW / "segment_02_candidate_path.csv").open(newline="", encoding="utf-8-sig") as handle:
        row = next(csv.DictReader(handle))
    return Frame(float(row["lat"]), float(row["lon"]))


def to_xy(frame: Frame, geometry):
    def convert(lon, lat, z=None):
        return frame.xy(lat, lon)

    return transform(convert, geometry)


def to_lonlat(frame: Frame, geometry):
    def convert(x, y, z=None):
        lat, lon = frame.ll(x, y)
        return lon, lat

    return transform(convert, geometry)


def load_segment(frame: Frame, segment: int):
    path = REVIEW / f"segment_{segment:02d}_candidate_path.csv"
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    line = LineString([frame.xy(float(row["lat"]), float(row["lon"])) for row in rows])
    return line, path


def points_in(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == "Point":
        return [geometry]
    if hasattr(geometry, "geoms"):
        result = []
        for part in geometry.geoms:
            result.extend(points_in(part))
        return result
    return []


def ordered_crossings(line, polygon):
    unique = {}
    for point in points_in(line.intersection(polygon.boundary)):
        distance = line.project(point)
        unique[round(distance, 6)] = (distance, point)
    return [item for _key, item in sorted(unique.items())]


def join_lines(lines):
    coordinates = []
    gaps = []
    for line in lines:
        part = list(line.coords)
        if not part:
            continue
        if coordinates:
            gap = math.dist(coordinates[-1], part[0])
            gaps.append(gap)
            if gap <= 0.001:
                part = part[1:]
        coordinates.extend(part)
    return LineString(coordinates), gaps


def append_point(line, point):
    coordinates = list(line.coords)
    gap = Point(coordinates[-1]).distance(point)
    if gap > 0.001:
        coordinates.append((point.x, point.y))
    return LineString(coordinates), gap


def properties(asset_id, name, category, status, geometry_role):
    return {
        "site_id": SITE_ID,
        "inventory_revision": REVISION,
        "inventory_revision_label": REVISION_LABEL,
        "asset_id": asset_id,
        "asset_name": name,
        "category": category,
        "status": status,
        "geometry_role": geometry_role,
        "owner_review_date": REVIEW_DATE,
    }


def feature(feature_id, props, geometry):
    return {
        "type": "Feature",
        "id": feature_id,
        "properties": props,
        "geometry": mapping(geometry),
    }


def index_row(props, geometry, **overrides):
    result = {column: "" for column in CSV_COLUMNS}
    result.update({
        "inventory_revision": REVISION,
        "asset_id": props["asset_id"],
        "asset_name": props["asset_name"],
        "category": props["category"],
        "status": props["status"],
        "field_validation": props.get("field_validation", ""),
        "geometry_roles": props["geometry_role"],
        "source_date": props.get("source_date", ""),
        "owner_review_date": REVIEW_DATE,
        "area_m2": round(geometry.area, 3) if geometry.geom_type in {"Polygon", "MultiPolygon"} else "",
        "length_m": round(geometry.length, 3) if geometry.geom_type in {"LineString", "MultiLineString"} else "",
        "start_location": props.get("start_location", ""),
        "end_location": props.get("end_location", ""),
        "directionality": props.get("directionality", ""),
        "reverse_validated": props.get("reverse_validated", ""),
        "related_area": props.get("related_area", ""),
        "from_access_point": props.get("from_access_point", ""),
        "to_access_point": props.get("to_access_point", ""),
        "source_reference": props.get("source_file", ""),
        "notes": props.get("notes", ""),
    })
    if geometry.geom_type == "Point":
        result["longitude"] = round(geometry.x, 9)
        result["latitude"] = round(geometry.y, 9)
    result.update(overrides)
    return result


def write_csv(rows):
    with CURRENT_CSV.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    HISTORY.mkdir(parents=True, exist_ok=True)
    frame = load_frame()
    rev1 = json.loads(REV1_GEOJSON.read_text(encoding="utf-8"))
    rev1_manifest = json.loads(REV1_MANIFEST.read_text(encoding="utf-8"))

    features = []
    rows = []
    areas_xy = {}

    # Carry the approved perimeter and obstacle definitions forward unchanged.
    for old in rev1["features"]:
        old_props = old["properties"]
        if old_props["category"] not in {"mowable_area", "obstacle"}:
            continue
        new_props = dict(old_props)
        new_props["inventory_revision"] = REVISION
        new_props["inventory_revision_label"] = REVISION_LABEL
        geometry_lonlat = shape(old["geometry"])
        geometry_xy = to_xy(frame, geometry_lonlat)
        features.append(feature(old["id"], new_props, geometry_lonlat))
        if new_props["category"] == "mowable_area":
            areas_xy[new_props["asset_id"]] = geometry_xy
            rows.append(index_row(new_props, geometry_xy))

    # One index row per physical obstacle, while GeoJSON retains each evidence role.
    for asset_id, asset_name, related_area in (
        ("OBS-TREE-001", "Tree at backyard/garden junction", "AREA-BACKYARD-GARDENS"),
        ("OBS-POLE-001", "Telephone pole over the road", "AREA-OVER-ROAD"),
    ):
        obstacle_features = [f for f in features if f["properties"]["asset_id"] == asset_id]
        roles = [f["properties"]["geometry_role"] for f in obstacle_features]
        exclusion = next(f for f in obstacle_features if f["properties"]["geometry_role"] == "mowing_exclusion")
        exclusion_xy = to_xy(frame, shape(exclusion["geometry"]))
        row_props = properties(asset_id, asset_name, "obstacle", "reviewed_not_field_validated", ";".join(roles))
        row_props.update({
            "field_validation": "recorded_evidence_with_reviewed_exclusion",
            "source_date": "2026-09-23" if asset_id == "OBS-TREE-001" else SOURCE_DATE,
            "related_area": related_area,
            "source_file": str(REV1_GEOJSON.relative_to(REPO)).replace("\\", "/"),
            "notes": "See GeoJSON evidence roles. The documented deck or clearance adjustment is already applied.",
        })
        rows.append(index_row(row_props, exclusion_xy))

    backyard = areas_xy["AREA-BACKYARD-GARDENS"]
    front = areas_xy["AREA-FRONT-YARD"]
    overroad = areas_xy["AREA-OVER-ROAD"]
    area_union = unary_union(list(areas_xy.values()))

    segments = {number: load_segment(frame, number) for number in (2, 9, 11, 12, 16, 17)}
    s2, p2 = segments[2]
    s9, p9 = segments[9]
    s11, p11 = segments[11]
    s12, p12 = segments[12]
    s16, p16 = segments[16]
    s17, p17 = segments[17]

    c2_backyard = ordered_crossings(s2, backyard)
    c9_backyard = ordered_crossings(s9, backyard)
    c11_front = ordered_crossings(s11, front)
    c16_overroad = ordered_crossings(s16, overroad)
    c17_front = ordered_crossings(s17, front)
    if not (len(c2_backyard) >= 1 and len(c9_backyard) == 1 and len(c11_front) >= 1 and len(c16_overroad) == 1 and len(c17_front) == 2):
        raise ValueError("Unexpected polygon crossing count; review access-point selection")

    ap_xy = {
        "ACCESS-BACKYARD-EAST": c2_backyard[0][1],
        "ACCESS-BACKYARD-NORTHEAST": c9_backyard[0][1],
        "ACCESS-FRONT-SOUTHWEST": nearest_points(Point(s9.coords[-1]), front.boundary)[1],
        "ACCESS-FRONT-NORTH-OUTBOUND": c11_front[-1][1],
        "ACCESS-OVERROAD-NORTH-OUTBOUND": nearest_points(Point(s12.coords[-1]), overroad.boundary)[1],
        "ACCESS-OVERROAD-NORTH-INBOUND": c16_overroad[0][1],
        "ACCESS-FRONT-NORTH-INBOUND": c17_front[0][1],
        "ACCESS-FRONT-WEST-RETURN": c17_front[1][1],
    }
    access_specs = {
        "ACCESS-BACKYARD-EAST": ("Backyard east entry/exit option", "AREA-BACKYARD-GARDENS", "bidirectional_option", 2, "Segment 2 first boundary crossing identified in Al's note b."),
        "ACCESS-BACKYARD-NORTHEAST": ("Backyard northeast entry/exit option", "AREA-BACKYARD-GARDENS", "bidirectional_option", 9, "Segment 9 boundary crossing identified in Al's note b."),
        "ACCESS-FRONT-SOUTHWEST": ("Front-yard southwest access", "AREA-FRONT-YARD", "entry_from_backyard", 9, "Nearest mowing-boundary point to the recorded segment 9 join; 0.898 m from the recorded tractor centerline endpoint."),
        "ACCESS-FRONT-NORTH-OUTBOUND": ("Front-yard north outbound access", "AREA-FRONT-YARD", "exit_to_overroad", 11, "Final segment 11 crossing when traveling outbound."),
        "ACCESS-OVERROAD-NORTH-OUTBOUND": ("Over-road outbound access", "AREA-OVER-ROAD", "entry_from_front", 12, "Nearest mowing-boundary point to the segment 12 endpoint."),
        "ACCESS-OVERROAD-NORTH-INBOUND": ("Over-road inbound access", "AREA-OVER-ROAD", "exit_to_front", 16, "Segment 16 crossing when returning from the over-road area."),
        "ACCESS-FRONT-NORTH-INBOUND": ("Front-yard north inbound access", "AREA-FRONT-YARD", "entry_from_overroad", 17, "First segment 17 boundary crossing when returning."),
        "ACCESS-FRONT-WEST-RETURN": ("Front-yard west return access", "AREA-FRONT-YARD", "exit_to_base", 17, "Second segment 17 boundary crossing when returning to base."),
    }
    for asset_id, point_xy in ap_xy.items():
        name, related_area, direction, source_segment, notes = access_specs[asset_id]
        point_lonlat = to_lonlat(frame, point_xy)
        props = properties(asset_id, name, "access_point", "reviewed_not_field_validated", "area_entry_exit_point")
        props.update({
            "field_validation": "derived_from_recorded_transition_and_reviewed_boundary",
            "source_date": SOURCE_DATE,
            "related_area": related_area,
            "directionality": direction,
            "source_file": ";".join((
                str(segments[source_segment][1].relative_to(REPO)).replace("\\", "/"),
                str(REV1_GEOJSON.relative_to(REPO)).replace("\\", "/"),
            )),
            "notes": notes,
        })
        features.append(feature(f"{asset_id}-POINT", props, point_lonlat))
        rows.append(index_row(props, point_lonlat))

    original_start_xy = Point(s2.coords[-1])
    original_start_lonlat = to_lonlat(frame, original_start_xy)
    start_props = properties("POINT-ORIGINAL-COVERAGE-START", "Original backyard coverage starting position", "site_reference_point", "field_recorded", "recorded_point")
    start_props.update({
        "field_validation": "field_recorded",
        "source_date": SOURCE_DATE,
        "related_area": "AREA-BACKYARD-GARDENS",
        "source_file": str(p2.relative_to(REPO)).replace("\\", "/"),
        "notes": "Original starting position identified in Al's note c; retained as a reference point, not a transition line.",
    })
    features.append(feature("POINT-ORIGINAL-COVERAGE-START", start_props, original_start_lonlat))
    rows.append(index_row(start_props, original_start_lonlat))

    # Store only the portions between mowing areas. Interior travel from segments 5 and 7 is intentionally omitted.
    route_1 = substring(s2, 0.0, c2_backyard[0][0])
    route_2_core = substring(s9, c9_backyard[0][0], s9.length)
    route_2, route_2_snap = append_point(route_2_core, ap_xy["ACCESS-FRONT-SOUTHWEST"])
    route_3a = substring(s11, c11_front[-1][0], s11.length)
    route_3, route_3_gaps = join_lines([route_3a, s12])
    route_3, route_3_snap = append_point(route_3, ap_xy["ACCESS-OVERROAD-NORTH-OUTBOUND"])
    route_4a = substring(s16, c16_overroad[0][0], s16.length)
    route_4b = substring(s17, 0.0, c17_front[0][0])
    route_4, route_4_gaps = join_lines([route_4a, route_4b])
    route_5 = substring(s17, c17_front[1][0], s17.length)

    route_specs = [
        ("ROUTE-BASE-TO-BACKYARD", "Base to backyard", route_1, "base_start", "AREA-BACKYARD-GARDENS", "", "ACCESS-BACKYARD-EAST", [2], [], "Recorded segment 2 clipped at the first backyard boundary crossing."),
        ("ROUTE-BACKYARD-TO-FRONT", "Backyard to front yard", route_2, "AREA-BACKYARD-GARDENS", "AREA-FRONT-YARD", "ACCESS-BACKYARD-NORTHEAST", "ACCESS-FRONT-SOUTHWEST", [9], [route_2_snap], "Recorded segment 9 clipped outside the mowing areas. The front boundary endpoint is derived from the nearest reviewed boundary."),
        ("ROUTE-FRONT-TO-OVERROAD", "Front yard to over-road area", route_3, "AREA-FRONT-YARD", "AREA-OVER-ROAD", "ACCESS-FRONT-NORTH-OUTBOUND", "ACCESS-OVERROAD-NORTH-OUTBOUND", [11, 12], route_3_gaps + [route_3_snap], "Outbound route combining the recorded front approach and driveway crossing."),
        ("ROUTE-OVERROAD-TO-FRONT", "Over-road area to front yard", route_4, "AREA-OVER-ROAD", "AREA-FRONT-YARD", "ACCESS-OVERROAD-NORTH-INBOUND", "ACCESS-FRONT-NORTH-INBOUND", [16, 17], route_4_gaps, "Inbound route combining the recorded driveway crossing and the first part of the return path."),
        ("ROUTE-FRONT-TO-BASE", "Front yard to base", route_5, "AREA-FRONT-YARD", "base_return", "ACCESS-FRONT-WEST-RETURN", "", [17], [], "Recorded return path after leaving the front-yard boundary."),
    ]
    route_lengths = {}
    for asset_id, name, route_xy, start, end, from_access, to_access, source_segments, joins, notes in route_specs:
        source_files = [REVIEW / f"segment_{segment:02d}_candidate_path.csv" for segment in source_segments]
        props = properties(asset_id, name, "transition_route", "approved_existing_between_areas", "recorded_centerline_between_areas")
        props.update({
            "field_validation": "recorded_path_clipped_at_reviewed_area_boundaries",
            "source_date": SOURCE_DATE,
            "source_segments": source_segments,
            "source_file": ";".join(str(path.relative_to(REPO)).replace("\\", "/") for path in source_files),
            "start_location": start,
            "end_location": end,
            "from_access_point": from_access,
            "to_access_point": to_access,
            "directionality": "recorded_direction_only",
            "reverse_validated": False,
            "length_m": round(route_xy.length, 3),
            "maximum_join_or_boundary_adjustment_m": round(max(joins), 3) if joins else 0.0,
            "notes": notes,
        })
        route_lonlat = to_lonlat(frame, route_xy)
        features.append(feature(f"{asset_id}-CENTERLINE", props, route_lonlat))
        rows.append(index_row(props, route_xy))
        route_lengths[asset_id] = route_xy.length

    # Ensure no stored route has meaningful travel inside a shaded mowing polygon.
    route_inside_lengths = {}
    for f in features:
        if f["properties"]["category"] != "transition_route":
            continue
        route_xy = to_xy(frame, shape(f["geometry"]))
        inside = route_xy.intersection(area_union.buffer(-0.02)).length
        route_inside_lengths[f["properties"]["asset_id"]] = inside
        if inside > 0.05:
            raise ValueError(f"{f['properties']['asset_id']} retains {inside:.3f} m inside a mowing polygon")

    inventory = {
        "type": "FeatureCollection",
        "name": "62 Collins site inventory revision 2",
        "properties": {
            "site_id": SITE_ID,
            "inventory_revision": REVISION,
            "inventory_revision_label": REVISION_LABEL,
            "created_date": REVIEW_DATE,
            "coordinate_reference_system": "EPSG:4326",
            "deck_width_m": DECK_WIDTH_M,
            "status": "reviewed_not_field_validated",
            "mission_generated": False,
        },
        "features": features,
    }
    ids = [f["id"] for f in features]
    geometry_checks = {
        f["id"]: {"geometry_type": f["geometry"]["type"], "valid": shape(f["geometry"]).is_valid, "empty": shape(f["geometry"]).is_empty}
        for f in features
    }
    if len(ids) != len(set(ids)) or any(not c["valid"] or c["empty"] for c in geometry_checks.values()):
        raise ValueError("Duplicate feature IDs or invalid geometry")

    CURRENT_GEOJSON.write_text(json.dumps(inventory, indent=2) + "\n", encoding="utf-8")
    write_csv(rows)

    manifest = {
        "site_id": SITE_ID,
        "inventory_revision": REVISION,
        "inventory_revision_label": REVISION_LABEL,
        "status": "reviewed_not_field_validated",
        "created_date": REVIEW_DATE,
        "supersedes": "rev_001_20260925",
        "counts": {
            "mowable_areas": 3,
            "obstacles": 2,
            "access_points": 8,
            "site_reference_points": 1,
            "transition_routes": 5,
            "geojson_features": len(features),
        },
        "mowable_area_total_m2": rev1_manifest["mowable_area_total_m2"],
        "transition_route_total_length_m": round(sum(route_lengths.values()), 3),
        "changes_from_revision_1": [
            "Removed internal transition assets TRANS-002 and TRANS-003.",
            "Removed all inventoried transition-line portions inside mowable polygons.",
            "Added eight named polygon entry/exit access points.",
            "Added the original backyard coverage starting position as a reference point.",
            "Reorganized the remaining recorded travel into five between-area routes.",
        ],
        "source_sha256": {
            str(REV1_GEOJSON.relative_to(REPO)).replace("\\", "/"): sha256(REV1_GEOJSON),
            **{
                str(path.relative_to(REPO)).replace("\\", "/"): sha256(path)
                for _segment, (_line, path) in segments.items()
            },
        },
        "rules": [
            "Transition routes contain no meaningful travel inside shaded mowing polygons.",
            "Coverage planners connect to transition routes through named access points.",
            "Backyard east and northeast points are retained as entry/exit options.",
            "The original coverage starting position is a reference point, not a transition route.",
            "Routes are approved only in their recorded direction.",
            "Obstacle mowing exclusions already include their documented adjustment; do not apply it twice.",
            "Revision 2 does not include a launchable mission.",
        ],
    }
    CURRENT_MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    validation = {
        "status": "PASS",
        "inventory_revision": REVISION,
        "asset_rows": len(rows),
        "geojson_features": len(features),
        "unique_feature_ids": len(ids) == len(set(ids)),
        "all_geometries_valid_and_nonempty": all(c["valid"] and not c["empty"] for c in geometry_checks.values()),
        "transition_inside_polygon_length_m": {key: round(value, 6) for key, value in route_inside_lengths.items()},
        "internal_transition_assets_removed": ["TRANS-002", "TRANS-003"],
        "geometry_checks": geometry_checks,
        "expected_counts": manifest["counts"],
        "mission_or_launcher_included": False,
    }
    CURRENT_VALIDATION.write_text(json.dumps(validation, indent=2) + "\n", encoding="utf-8")

    write_map(frame, features)
    write_html(frame, features)

    CURRENT_README.write_text(
        "# 62 Collins site inventory\n\n"
        "Revision 2 is the current site description for coverage planning. It preserves the three mowable areas and two "
        "obstacles from revision 1, removes transition lines inside mowing polygons, and represents travel with named "
        "entry/exit points and between-area routes. It does not contain a launchable mission.\n\n"
        "## Current files\n\n"
        "- `62_Collins_site_inventory.geojson` is the authoritative geometry.\n"
        "- `62_Collins_site_inventory.csv` is the asset index.\n"
        "- `62_Collins_site_inventory.xlsx` is the formatted asset index.\n"
        "- `62_Collins_site_inventory_map.html` is the interactive map.\n"
        "- `62_Collins_site_inventory_map.png` is the static map.\n"
        "- `62_Collins_site_inventory_manifest.json` records sources, counts, changes, and rules.\n"
        "- `62_Collins_site_inventory_validation.json` records geometry and route checks.\n\n"
        "## Revision 2 transition model\n\n"
        "Coverage routes inside a mowing polygon are generated by the coverage planner and are not site inventory. "
        "The inventory begins or ends a transition at a named polygon access point. Only recorded travel outside the "
        "mowing polygons is stored as a transition route.\n\n"
        "The two internal garden connectors from revision 1 are no longer current assets. The backyard east and "
        "northeast access points are retained as entry/exit options. The original backyard coverage starting position "
        "is retained separately as a recorded reference point.\n\n"
        "## Revision history\n\n"
        "Revision 1 remains unchanged in `revisions/rev_001_20260925/`. The revision 2 snapshot is in "
        f"`revisions/{REVISION_LABEL}/`.\n",
        encoding="utf-8",
        newline="\n",
    )

    for path in (CURRENT_GEOJSON, CURRENT_CSV, CURRENT_MANIFEST, CURRENT_VALIDATION, CURRENT_MAP, CURRENT_HTML, CURRENT_README):
        shutil.copyfile(path, HISTORY / path.name)

    print(json.dumps({
        "output": str(OUT.relative_to(REPO)).replace("\\", "/"),
        "revision": REVISION_LABEL,
        "counts": manifest["counts"],
        "transition_route_total_length_m": manifest["transition_route_total_length_m"],
        "route_inside_polygon_length_m": validation["transition_inside_polygon_length_m"],
    }, indent=2))


def draw_polygon(axis, geometry, face, edge, label):
    first = True
    for polygon in polygons_of(geometry):
        x, y = polygon.exterior.xy
        axis.fill(x, y, color=face, alpha=0.25)
        axis.plot(x, y, color=edge, linewidth=1.8, label=label if first else None)
        for interior in polygon.interiors:
            ix, iy = interior.xy
            axis.fill(ix, iy, color="white")
            axis.plot(ix, iy, color="#c62828", linewidth=1.8)
        first = False


def write_map(frame: Frame, features):
    figure, axis = plt.subplots(figsize=(13, 10), dpi=170)
    colors = {
        "AREA-BACKYARD-GARDENS": ("#90caf9", "#1565c0"),
        "AREA-FRONT-YARD": ("#ffcc80", "#ef6c00"),
        "AREA-OVER-ROAD": ("#bcaaa4", "#795548"),
    }
    for f in features:
        props = f["properties"]
        geometry = to_xy(frame, shape(f["geometry"]))
        if props["category"] == "mowable_area":
            face, edge = colors[props["asset_id"]]
            draw_polygon(axis, geometry, face, edge, props["asset_id"])
        elif props["category"] == "obstacle" and props["geometry_role"] == "mowing_exclusion":
            color = "#c62828" if props["asset_id"] == "OBS-TREE-001" else "#6a1b9a"
            draw_polygon(axis, geometry, color, color, props["asset_id"])
        elif props["category"] == "transition_route":
            x, y = geometry.xy
            axis.plot(x, y, color="#00897b", linewidth=2.0, label="Between-area routes" if props["asset_id"] == "ROUTE-BASE-TO-BACKYARD" else None)
        elif props["category"] == "access_point":
            axis.scatter([geometry.x], [geometry.y], s=42, color="#00acc1", edgecolor="white", linewidth=0.8, zorder=6)
            axis.annotate(props["asset_id"].replace("ACCESS-", "AP "), (geometry.x, geometry.y), xytext=(4, 4), textcoords="offset points", fontsize=6.5, color="#006064")
        elif props["category"] == "site_reference_point":
            axis.scatter([geometry.x], [geometry.y], marker="*", s=120, color="#263238", zorder=7, label="Original coverage start")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.18)
    axis.set_xlabel("east (m)")
    axis.set_ylabel("north (m)")
    axis.set_title("62 Collins site inventory — revision 2")
    axis.legend(loc="best", fontsize=7)
    figure.tight_layout()
    figure.savefig(CURRENT_MAP)
    plt.close(figure)


def write_html(frame: Frame, features):
    payload = []
    for f in features:
        payload.append({"id": f["id"], "p": f["properties"], "g": mapping(to_xy(frame, shape(f["geometry"])))})
    template = """<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>62 Collins site inventory revision 2</title><style>
*{box-sizing:border-box}body{margin:0;background:#10151c;color:#e8eef5;font:14px system-ui,sans-serif}header{height:68px;padding:10px 16px;background:#17202a;border-bottom:1px solid #34404e}h1{font-size:19px;margin:0 0 5px}.sub{color:#ffcc80}main{display:grid;grid-template-columns:335px 1fr;height:calc(100vh - 68px)}aside{padding:14px;overflow:auto;border-right:1px solid #34404e}canvas{width:100%;height:100%;background:#f8fafc}.row{margin:8px 0}.small{font-size:12px;color:#b8c4cf;line-height:1.45}button{background:#263545;color:#fff;border:1px solid #526579;border-radius:5px;padding:7px 10px}</style></head><body><header><h1>62 Collins site inventory — revision 2</h1><div class=\"sub\">Transition routes are stored only between shaded mowing areas.</div></header><main><aside>
<b>Layers</b><div class=\"row\"><label><input id=\"areas\" type=\"checkbox\" checked> Mowable areas</label></div><div class=\"row\"><label><input id=\"obstacles\" type=\"checkbox\" checked> Obstacles</label></div><div class=\"row\"><label><input id=\"routes\" type=\"checkbox\" checked> Between-area routes</label></div><div class=\"row\"><label><input id=\"points\" type=\"checkbox\" checked> Entry/exit and reference points</label></div><p><button id=\"fit\">Fit all</button></p><p class=\"small\">Green lines are recorded travel retained outside the mowing polygons. Cyan circles are named entry/exit points. The black star is the original backyard coverage starting position. Internal garden travel is intentionally omitted.</p>
</aside><canvas id=\"map\"></canvas></main><script>
const F=__PAYLOAD__,$=x=>document.getElementById(x),C=$('map'),X=C.getContext('2d');let s=8,ox=0,oy=0,drag=null;const colors={'AREA-BACKYARD-GARDENS':'#1565c0','AREA-FRONT-YARD':'#ef6c00','AREA-OVER-ROAD':'#795548','OBS-TREE-001':'#c62828','OBS-POLE-001':'#6a1b9a'};
function q(p){return[ox+p[0]*s,oy-p[1]*s]}function rings(g){if(g.type==='Polygon')return[g.coordinates];if(g.type==='MultiPolygon')return g.coordinates;return[]}function line(points,c,w=1,d=[]){X.save();X.strokeStyle=c;X.lineWidth=w;X.setLineDash(d);X.beginPath();points.forEach((p,i)=>{const a=q(p);i?X.lineTo(...a):X.moveTo(...a)});X.stroke();X.restore()}function poly(g,c){for(const P of rings(g)){X.save();X.fillStyle=c+'2c';X.strokeStyle=c;X.lineWidth=2;X.beginPath();for(const R of P){R.forEach((p,i)=>{const a=q(p);i?X.lineTo(...a):X.moveTo(...a)});X.closePath()}X.fill('evenodd');X.stroke();X.restore()}}function dot(p,c,r){const a=q(p);X.save();X.fillStyle=c;X.strokeStyle='#fff';X.lineWidth=1;X.beginPath();X.arc(a[0],a[1],r,0,Math.PI*2);X.fill();X.stroke();X.restore()}function coords(g,o=[]){if(g.type==='Point')o.push(g.coordinates);else if(g.type==='LineString')o.push(...g.coordinates);else for(const P of rings(g))for(const R of P)o.push(...R);return o}
function draw(){let r=C.getBoundingClientRect();X.clearRect(0,0,r.width,r.height);X.fillStyle='#f8fafc';X.fillRect(0,0,r.width,r.height);for(const f of F){let p=f.p;if(p.category==='mowable_area'&&$('areas').checked)poly(f.g,colors[p.asset_id]);if(p.category==='obstacle'&&p.geometry_role==='mowing_exclusion'&&$('obstacles').checked)poly(f.g,colors[p.asset_id]);if(p.category==='transition_route'&&$('routes').checked)line(f.g.coordinates,'#00897b',2);if(p.category==='access_point'&&$('points').checked)dot(f.g.coordinates,'#00acc1',5);if(p.category==='site_reference_point'&&$('points').checked)dot(f.g.coordinates,'#263238',6)}}function fit(){let a=[];F.forEach(f=>coords(f.g,a));let xs=a.map(p=>p[0]),ys=a.map(p=>p[1]),r=C.getBoundingClientRect(),pad=30;s=Math.min((r.width-2*pad)/(Math.max(...xs)-Math.min(...xs)),(r.height-2*pad)/(Math.max(...ys)-Math.min(...ys)));ox=pad-Math.min(...xs)*s;oy=pad+Math.max(...ys)*s;draw()}C.onwheel=e=>{e.preventDefault();let r=C.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,k=e.deltaY<0?1.15:1/1.15;ox=mx-(mx-ox)*k;oy=my-(my-oy)*k;s*=k;draw()};C.onpointerdown=e=>drag=[e.clientX,e.clientY,ox,oy];C.onpointermove=e=>{if(drag){ox=drag[2]+e.clientX-drag[0];oy=drag[3]+e.clientY-drag[1];draw()}};C.onpointerup=()=>drag=null;document.querySelectorAll('input').forEach(e=>e.onchange=draw);$('fit').onclick=fit;window.onresize=()=>{let r=C.getBoundingClientRect(),d=devicePixelRatio||1;C.width=r.width*d;C.height=r.height*d;X.setTransform(d,0,0,d,0,0);fit()};window.onresize();
</script></body></html>"""
    CURRENT_HTML.write_text(template.replace("__PAYLOAD__", json.dumps(payload, separators=(",", ":"))), encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
