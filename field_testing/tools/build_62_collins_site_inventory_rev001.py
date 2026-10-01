#!/usr/bin/env python3
"""Build revision 1 of the durable 62 Collins site inventory."""

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
from shapely.geometry import LineString, mapping, shape
from shapely.ops import transform

from build_20260923_combined_coverage_review import REPO, REVIEW, SITE, Frame, polygons_of


REVISION = 1
REVISION_LABEL = "rev_001_20260925"
SITE_ID = "62_COLLINS"
REVIEW_DATE = "2026-09-25"
SOURCE_DATE = "2026-09-15"
DECK_WIDTH_M = 1.0668
DECK_HALF_WIDTH_M = DECK_WIDTH_M / 2.0

SOURCE_GEOJSON = (
    SITE
    / "analysis/20260928_perimeter_candidate_v2_REVIEW_ONLY"
    / "62_Collins_perimeter_candidate_v2_REVIEW_ONLY.geojson"
)
SOURCE_REPORT = (
    SITE
    / "analysis/20260928_perimeter_candidate_v2_REVIEW_ONLY"
    / "62_Collins_perimeter_candidate_v2_report.json"
)
OUT = SITE / "site_inventory"
HISTORY = OUT / "revisions" / REVISION_LABEL

CURRENT_GEOJSON = OUT / "62_Collins_site_inventory.geojson"
CURRENT_CSV = OUT / "62_Collins_site_inventory.csv"
CURRENT_MANIFEST = OUT / "62_Collins_site_inventory_manifest.json"
CURRENT_VALIDATION = OUT / "62_Collins_site_inventory_validation.json"
CURRENT_MAP = OUT / "62_Collins_site_inventory_map.png"
CURRENT_HTML = OUT / "62_Collins_site_inventory_map.html"
CURRENT_README = OUT / "README.md"

AREA_SPECS = {
    "backyard_and_gardens": {
        "asset_id": "AREA-BACKYARD-GARDENS",
        "name": "Backyard and gardens",
        "notes": "Merged backyard, garden-left, and garden-right mowing area. Contains OBS-TREE-001 exclusion.",
    },
    "front_yard": {
        "asset_id": "AREA-FRONT-YARD",
        "name": "Front yard",
        "notes": "Existing front-yard perimeter retained in revision 1.",
    },
    "over_the_road": {
        "asset_id": "AREA-OVER-ROAD",
        "name": "Over the road",
        "notes": "Mowing area with the OBS-POLE-001 clearance removed at the perimeter notch.",
    },
}

TRANSITIONS = [
    ("TRANS-001", 2, "Base to backyard", "base", "AREA-BACKYARD-GARDENS", "forward_only", "Primary mission entry."),
    ("TRANS-002", 5, "Backyard to right garden", "backyard", "garden_right_zone", "forward_only", "Retained as a known-good internal connector; optional after area merge."),
    ("TRANS-003", 7, "Right garden to left garden", "garden_right_zone", "garden_left_zone", "forward_only", "Retained as a known-good internal connector; optional after area merge."),
    ("TRANS-004", 9, "Garden area to front yard", "AREA-BACKYARD-GARDENS", "AREA-FRONT-YARD", "forward_only", "Existing connection to the front yard."),
    ("TRANS-005", 11, "Front yard to driveway approach", "AREA-FRONT-YARD", "driveway_approach", "forward_only", "Approach to the driveway crossing."),
    ("TRANS-006", 12, "Driveway crossing outbound", "driveway_approach", "AREA-OVER-ROAD", "forward_only", "Guided outbound driveway crossing."),
    ("TRANS-007", 16, "Driveway crossing inbound", "AREA-OVER-ROAD", "driveway_approach", "forward_only", "Guided inbound driveway crossing; stored separately from outbound."),
    ("TRANS-008", 17, "Return to base", "driveway_approach", "base", "forward_only", "Existing return path to base."),
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


def line_length_m(frame: Frame, geometry) -> float:
    return to_xy(frame, geometry).length


def load_source_features():
    collection = json.loads(SOURCE_GEOJSON.read_text(encoding="utf-8"))
    by_name = {feature["properties"]["name"]: feature for feature in collection["features"]}
    expected = set(AREA_SPECS) | {
        "tree_recorded_innermost_centerline",
        "tree_fitted_centerline_boundary",
        "tree_deck_edge_exclusion",
        "telephone_pole_recorded_loop",
        "telephone_pole_clearance",
    }
    if not expected.issubset(by_name):
        raise ValueError(f"Missing source features: {sorted(expected - set(by_name))}")
    return by_name


def common_properties(asset_id: str, name: str, category: str, status: str):
    return {
        "site_id": SITE_ID,
        "inventory_revision": REVISION,
        "inventory_revision_label": REVISION_LABEL,
        "asset_id": asset_id,
        "asset_name": name,
        "category": category,
        "status": status,
        "owner_review_date": REVIEW_DATE,
    }


def make_feature(feature_id: str, properties: dict, geometry: dict):
    return {
        "type": "Feature",
        "id": feature_id,
        "properties": properties,
        "geometry": geometry,
    }


def transition_feature(frame: Frame, asset_id: str, segment: int, name: str, start: str, end: str, direction: str, notes: str):
    path = REVIEW / f"segment_{segment:02d}_candidate_path.csv"
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    coordinates = [[float(row["lon"]), float(row["lat"])] for row in rows]
    geometry = {"type": "LineString", "coordinates": coordinates}
    length_m = line_length_m(frame, shape(geometry))
    properties = common_properties(asset_id, name, "transition", "approved_existing")
    properties.update({
        "geometry_role": "recorded_centerline",
        "field_validation": "field_recorded_and_owner_accepted",
        "source_date": SOURCE_DATE,
        "source_segment": segment,
        "source_file": str(path.relative_to(REPO)).replace("\\", "/"),
        "source_sha256": sha256(path),
        "start_location": start,
        "end_location": end,
        "directionality": direction,
        "reverse_validated": False,
        "point_count": len(coordinates),
        "length_m": round(length_m, 3),
        "notes": notes,
    })
    return make_feature(f"{asset_id}-CENTERLINE", properties, geometry), length_m


def write_csv(rows):
    columns = [
        "inventory_revision", "asset_id", "asset_name", "category", "status",
        "field_validation", "geometry_roles", "source_date", "owner_review_date",
        "area_m2", "length_m", "start_location", "end_location", "directionality",
        "reverse_validated", "related_area", "source_reference", "notes",
    ]
    with CURRENT_CSV.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


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
    by_asset_role = {
        (f["properties"]["asset_id"], f["properties"]["geometry_role"]): to_xy(frame, shape(f["geometry"]))
        for f in features
    }
    figure, axis = plt.subplots(figsize=(13, 10), dpi=170)
    area_colors = {
        "AREA-BACKYARD-GARDENS": ("#90caf9", "#1565c0"),
        "AREA-FRONT-YARD": ("#ffcc80", "#ef6c00"),
        "AREA-OVER-ROAD": ("#bcaaa4", "#795548"),
    }
    for asset_id, (face, edge) in area_colors.items():
        draw_polygon(axis, by_asset_role[(asset_id, "mowable_boundary")], face, edge, asset_id)
    for feature in features:
        props = feature["properties"]
        geometry = to_xy(frame, shape(feature["geometry"]))
        if props["category"] == "transition":
            x, y = geometry.xy
            axis.plot(x, y, color="#00897b", linewidth=1.25, alpha=0.9)
            midpoint = geometry.interpolate(0.5, normalized=True)
            axis.annotate(props["asset_id"].replace("TRANS-", "T"), (midpoint.x, midpoint.y), fontsize=7, color="#00695c")
    tree_limit = by_asset_role[("OBS-TREE-001", "tested_tractor_centerline_limit")]
    tx, ty = tree_limit.xy
    axis.plot(tx, ty, color="#d81b60", linestyle="--", linewidth=1.5, label="OBS-TREE-001 tested centerline")
    tree_hole = by_asset_role[("OBS-TREE-001", "mowing_exclusion")]
    draw_polygon(axis, tree_hole, "#ef9a9a", "#c62828", "OBS-TREE-001 mowing exclusion")
    pole = by_asset_role[("OBS-POLE-001", "mowing_exclusion")]
    draw_polygon(axis, pole, "#ce93d8", "#6a1b9a", "OBS-POLE-001 clearance")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.18)
    axis.set_xlabel("east (m)")
    axis.set_ylabel("north (m)")
    axis.set_title("62 Collins site inventory — revision 1")
    axis.legend(loc="best", fontsize=7)
    figure.tight_layout()
    figure.savefig(CURRENT_MAP)
    plt.close(figure)


def write_html(frame: Frame, features):
    payload = []
    for feature in features:
        geometry = mapping(to_xy(frame, shape(feature["geometry"])))
        payload.append({"id": feature["id"], "p": feature["properties"], "g": geometry})
    template = """<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>62 Collins site inventory revision 1</title><style>
*{box-sizing:border-box}body{margin:0;background:#10151c;color:#e8eef5;font:14px system-ui,sans-serif}header{height:68px;padding:10px 16px;background:#17202a;border-bottom:1px solid #34404e}h1{font-size:19px;margin:0 0 5px}.sub{color:#ffcc80}main{display:grid;grid-template-columns:325px 1fr;height:calc(100vh - 68px)}aside{padding:14px;overflow:auto;border-right:1px solid #34404e}canvas{width:100%;height:100%;background:#f8fafc}.row{margin:8px 0}.small{font-size:12px;color:#b8c4cf;line-height:1.45}button{background:#263545;color:#fff;border:1px solid #526579;border-radius:5px;padding:7px 10px}</style></head><body><header><h1>62 Collins site inventory — revision 1</h1><div class=\"sub\">Reviewed geometry; mowing areas and obstacles remain unvalidated until the next field run.</div></header><main><aside>
<b>Layers</b><div class=\"row\"><label><input id=\"areas\" type=\"checkbox\" checked> Mowable areas</label></div><div class=\"row\"><label><input id=\"obstacles\" type=\"checkbox\" checked> Obstacles</label></div><div class=\"row\"><label><input id=\"transitions\" type=\"checkbox\" checked> Approved existing transitions</label></div><p><button id=\"fit\">Fit all</button></p><p class=\"small\">Solid colored outlines are mowing boundaries. Red and purple show obstacle exclusions. Green lines are recorded transition centerlines. T2 and T3 are retained internal connectors that may not be required by a new coverage plan.</p>
</aside><canvas id=\"map\"></canvas></main><script>
const F=__PAYLOAD__,$=x=>document.getElementById(x),C=$('map'),X=C.getContext('2d');let s=8,ox=0,oy=0,drag=null;const colors={'AREA-BACKYARD-GARDENS':'#1565c0','AREA-FRONT-YARD':'#ef6c00','AREA-OVER-ROAD':'#795548','OBS-TREE-001':'#c62828','OBS-POLE-001':'#6a1b9a'};
function q(p){return[ox+p[0]*s,oy-p[1]*s]}function rings(g){if(g.type==='Polygon')return[g.coordinates];if(g.type==='MultiPolygon')return g.coordinates;return[]}function line(points,c,w=1,d=[]){X.save();X.strokeStyle=c;X.lineWidth=w;X.setLineDash(d);X.beginPath();points.forEach((p,i)=>{const a=q(p);i?X.lineTo(...a):X.moveTo(...a)});X.stroke();X.restore()}function poly(g,c){for(const P of rings(g)){X.save();X.fillStyle=c+'2c';X.strokeStyle=c;X.lineWidth=2;X.beginPath();for(const R of P){R.forEach((p,i)=>{const a=q(p);i?X.lineTo(...a):X.moveTo(...a)});X.closePath()}X.fill('evenodd');X.stroke();X.restore()}}function coords(g,o=[]){if(g.type==='LineString')o.push(...g.coordinates);else for(const P of rings(g))for(const R of P)o.push(...R);return o}
function draw(){let r=C.getBoundingClientRect();X.clearRect(0,0,r.width,r.height);X.fillStyle='#f8fafc';X.fillRect(0,0,r.width,r.height);for(const f of F){let p=f.p;if(p.category==='mowable_area'&&$('areas').checked)poly(f.g,colors[p.asset_id]);if(p.category==='obstacle'&&$('obstacles').checked){if(f.g.type==='LineString')line(f.g.coordinates,colors[p.asset_id],1.4,[4,3]);else poly(f.g,colors[p.asset_id])}if(p.category==='transition'&&$('transitions').checked)line(f.g.coordinates,'#00897b',1.5)}}function fit(){let a=[];F.forEach(f=>coords(f.g,a));let xs=a.map(p=>p[0]),ys=a.map(p=>p[1]),r=C.getBoundingClientRect(),pad=30;s=Math.min((r.width-2*pad)/(Math.max(...xs)-Math.min(...xs)),(r.height-2*pad)/(Math.max(...ys)-Math.min(...ys)));ox=pad-Math.min(...xs)*s;oy=pad+Math.max(...ys)*s;draw()}C.onwheel=e=>{e.preventDefault();let r=C.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,k=e.deltaY<0?1.15:1/1.15;ox=mx-(mx-ox)*k;oy=my-(my-oy)*k;s*=k;draw()};C.onpointerdown=e=>drag=[e.clientX,e.clientY,ox,oy];C.onpointermove=e=>{if(drag){ox=drag[2]+e.clientX-drag[0];oy=drag[3]+e.clientY-drag[1];draw()}};C.onpointerup=()=>drag=null;document.querySelectorAll('input').forEach(e=>e.onchange=draw);$('fit').onclick=fit;window.onresize=()=>{let r=C.getBoundingClientRect(),d=devicePixelRatio||1;C.width=r.width*d;C.height=r.height*d;X.setTransform(d,0,0,d,0,0);fit()};window.onresize();
</script></body></html>"""
    CURRENT_HTML.write_text(template.replace("__PAYLOAD__", json.dumps(payload, separators=(",", ":"))), encoding="utf-8", newline="\n")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    HISTORY.mkdir(parents=True, exist_ok=True)
    frame = load_frame()
    source = load_source_features()
    report = json.loads(SOURCE_REPORT.read_text(encoding="utf-8"))
    features = []
    index_rows = []

    for source_name, spec in AREA_SPECS.items():
        geometry = source[source_name]["geometry"]
        local = to_xy(frame, shape(geometry))
        props = common_properties(spec["asset_id"], spec["name"], "mowable_area", "reviewed_not_field_validated")
        props.update({
            "geometry_role": "mowable_boundary",
            "field_validation": "pending_next_field_run",
            "source_date": "2026-09-23",
            "source_file": str(SOURCE_GEOJSON.relative_to(REPO)).replace("\\", "/"),
            "area_m2": round(local.area, 3),
            "perimeter_m": round(local.length, 3),
            "deck_width_m": DECK_WIDTH_M,
            "notes": spec["notes"],
        })
        features.append(make_feature(f"{spec['asset_id']}-BOUNDARY", props, geometry))
        index_rows.append({
            "inventory_revision": REVISION, "asset_id": spec["asset_id"], "asset_name": spec["name"],
            "category": "mowable_area", "status": props["status"], "field_validation": props["field_validation"],
            "geometry_roles": "mowable_boundary", "source_date": "2026-09-23", "owner_review_date": REVIEW_DATE,
            "area_m2": round(local.area, 3), "length_m": round(local.length, 3), "start_location": "", "end_location": "",
            "directionality": "", "reverse_validated": "", "related_area": spec["asset_id"],
            "source_reference": str(SOURCE_GEOJSON.relative_to(REPO)).replace("\\", "/"), "notes": spec["notes"],
        })

    obstacle_specs = [
        ("OBS-TREE-001", "Tree at backyard/garden junction", "tree_recorded_innermost_centerline", "recorded_centerline_evidence", "field_recorded", "Raw points from the innermost complete manual loop."),
        ("OBS-TREE-001", "Tree at backyard/garden junction", "tree_fitted_centerline_boundary", "tested_tractor_centerline_limit", "derived_from_field_recording", "Circle fitted to the innermost manual loop; this is evidence, not an extra buffer."),
        ("OBS-TREE-001", "Tree at backyard/garden junction", "tree_deck_edge_exclusion", "mowing_exclusion", "reviewed_not_field_validated", "Deck-edge exclusion derived by moving 0.5334 m inward from the tested centerline."),
        ("OBS-POLE-001", "Telephone pole over the road", "telephone_pole_recorded_loop", "recorded_centerline_evidence", "field_recorded", "Recorded loop from source segment 14, indices 13 through 63."),
        ("OBS-POLE-001", "Telephone pole over the road", "telephone_pole_clearance", "mowing_exclusion", "reviewed_not_field_validated", "Recorded loop expanded by 0.6096 m plus a 0.02 m numerical margin."),
    ]
    obstacle_roles = {"OBS-TREE-001": [], "OBS-POLE-001": []}
    obstacle_areas = {"OBS-TREE-001": None, "OBS-POLE-001": None}
    for asset_id, asset_name, source_name, role, validation, notes in obstacle_specs:
        source_feature = source[source_name]
        local = to_xy(frame, shape(source_feature["geometry"]))
        props = common_properties(asset_id, asset_name, "obstacle", "reviewed_not_field_validated")
        props.update({
            "geometry_role": role,
            "field_validation": validation,
            "source_date": "2026-09-23" if asset_id == "OBS-TREE-001" else SOURCE_DATE,
            "source_file": str(SOURCE_GEOJSON.relative_to(REPO)).replace("\\", "/"),
            "related_area": "AREA-BACKYARD-GARDENS" if asset_id == "OBS-TREE-001" else "AREA-OVER-ROAD",
            "deck_half_width_m": DECK_HALF_WIDTH_M if asset_id == "OBS-TREE-001" and role == "mowing_exclusion" else None,
            "clearance_applied_m": 0.6296 if asset_id == "OBS-POLE-001" and role == "mowing_exclusion" else None,
            "area_m2": round(local.area, 3) if local.geom_type in {"Polygon", "MultiPolygon"} else None,
            "length_m": round(local.length, 3),
            "notes": notes,
        })
        features.append(make_feature(f"{asset_id}-{role.upper().replace('_', '-')}", props, source_feature["geometry"]))
        obstacle_roles[asset_id].append(role)
        if role == "mowing_exclusion":
            obstacle_areas[asset_id] = round(local.area, 3)
    for asset_id, name, related in (
        ("OBS-TREE-001", "Tree at backyard/garden junction", "AREA-BACKYARD-GARDENS"),
        ("OBS-POLE-001", "Telephone pole over the road", "AREA-OVER-ROAD"),
    ):
        index_rows.append({
            "inventory_revision": REVISION, "asset_id": asset_id, "asset_name": name, "category": "obstacle",
            "status": "reviewed_not_field_validated", "field_validation": "recorded_evidence_with_reviewed_exclusion",
            "geometry_roles": ";".join(obstacle_roles[asset_id]),
            "source_date": "2026-09-23" if asset_id == "OBS-TREE-001" else SOURCE_DATE,
            "owner_review_date": REVIEW_DATE, "area_m2": obstacle_areas[asset_id], "length_m": "",
            "start_location": "", "end_location": "", "directionality": "", "reverse_validated": "",
            "related_area": related, "source_reference": str(SOURCE_GEOJSON.relative_to(REPO)).replace("\\", "/"),
            "notes": "See GeoJSON feature roles; do not apply the recorded deck/clearance adjustment a second time.",
        })

    transition_lengths = {}
    for asset_id, segment, name, start, end, direction, notes in TRANSITIONS:
        feature, length_m = transition_feature(frame, asset_id, segment, name, start, end, direction, notes)
        features.append(feature)
        transition_lengths[asset_id] = length_m
        props = feature["properties"]
        index_rows.append({
            "inventory_revision": REVISION, "asset_id": asset_id, "asset_name": name, "category": "transition",
            "status": props["status"], "field_validation": props["field_validation"], "geometry_roles": props["geometry_role"],
            "source_date": SOURCE_DATE, "owner_review_date": REVIEW_DATE, "area_m2": "", "length_m": round(length_m, 3),
            "start_location": start, "end_location": end, "directionality": direction, "reverse_validated": False,
            "related_area": f"{start} -> {end}", "source_reference": props["source_file"], "notes": notes,
        })

    inventory = {
        "type": "FeatureCollection",
        "name": "62 Collins site inventory revision 1",
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
    ids = [feature["id"] for feature in features]
    geometry_checks = {
        feature["id"]: {
            "geometry_type": feature["geometry"]["type"],
            "valid": shape(feature["geometry"]).is_valid,
            "empty": shape(feature["geometry"]).is_empty,
        }
        for feature in features
    }
    if len(ids) != len(set(ids)):
        raise ValueError("Inventory feature IDs are not unique")
    if any(not check["valid"] or check["empty"] for check in geometry_checks.values()):
        raise ValueError("Inventory contains an invalid or empty geometry")
    if len(features) != 16 or len(index_rows) != 13:
        raise ValueError("Unexpected inventory feature or asset count")
    CURRENT_GEOJSON.write_text(json.dumps(inventory, indent=2) + "\n", encoding="utf-8")
    write_csv(index_rows)
    write_map(frame, features)
    write_html(frame, features)

    manifest = {
        "site_id": SITE_ID,
        "inventory_revision": REVISION,
        "inventory_revision_label": REVISION_LABEL,
        "status": "reviewed_not_field_validated",
        "created_date": REVIEW_DATE,
        "counts": {"mowable_areas": 3, "obstacles": 2, "transitions": 8, "geojson_features": len(features)},
        "mowable_area_total_m2": round(sum(report["areas"][key]["area_m2"] for key in AREA_SPECS), 3),
        "transition_total_length_m": round(sum(transition_lengths.values()), 3),
        "source_sha256": {
            str(SOURCE_GEOJSON.relative_to(REPO)).replace("\\", "/"): sha256(SOURCE_GEOJSON),
            str(SOURCE_REPORT.relative_to(REPO)).replace("\\", "/"): sha256(SOURCE_REPORT),
            **{
                str((REVIEW / f"segment_{segment:02d}_candidate_path.csv").relative_to(REPO)).replace("\\", "/"): sha256(REVIEW / f"segment_{segment:02d}_candidate_path.csv")
                for _asset_id, segment, *_rest in TRANSITIONS
            },
        },
        "rules": [
            "Mission builders must select features by stable asset_id and geometry_role.",
            "Obstacle mowing exclusions already contain the documented deck or clearance adjustment; do not apply it twice.",
            "Transition direction is validated only as recorded. Reverse use requires review.",
            "TRANS-002 and TRANS-003 are retained internal connectors and may be omitted by a merged-area coverage plan.",
            "Revision 1 does not include a launchable mission.",
        ],
    }
    CURRENT_MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    validation = {
        "status": "PASS",
        "inventory_revision": REVISION,
        "asset_rows": len(index_rows),
        "geojson_features": len(features),
        "unique_feature_ids": len(ids) == len(set(ids)),
        "all_geometries_valid_and_nonempty": all(check["valid"] and not check["empty"] for check in geometry_checks.values()),
        "geometry_checks": geometry_checks,
        "expected_counts": manifest["counts"],
        "mowable_area_total_m2": manifest["mowable_area_total_m2"],
        "transition_total_length_m": manifest["transition_total_length_m"],
        "mission_or_launcher_included": False,
    }
    CURRENT_VALIDATION.write_text(json.dumps(validation, indent=2) + "\n", encoding="utf-8")
    CURRENT_README.write_text(
        "# 62 Collins site inventory\n\n"
        "Revision 1 is the durable site description for coverage planning. It contains three mowable areas, "
        "two obstacles, and eight approved existing transition paths. It does not contain a launchable mission.\n\n"
        "## Current files\n\n"
        "- `62_Collins_site_inventory.geojson` is the authoritative machine-readable geometry.\n"
        "- `62_Collins_site_inventory.csv` is the human-readable asset index.\n"
        "- `62_Collins_site_inventory.xlsx` is a formatted copy of the same index.\n"
        "- `62_Collins_site_inventory_map.html` is the interactive review map.\n"
        "- `62_Collins_site_inventory_map.png` is the static review map.\n"
        "- `62_Collins_site_inventory_manifest.json` records counts, source hashes, and inventory rules.\n\n"
        "- `62_Collins_site_inventory_validation.json` records the geometry and count checks.\n\n"
        "## Geometry rules\n\n"
        "Mowable polygons describe the permitted deck coverage. Obstacle features separately retain recorded evidence, "
        "tested tractor-center limits, and mowing exclusions. Transition paths are recorded tractor centerlines and are "
        "approved only in their original direction.\n\n"
        "The tree mowing exclusion already includes the 0.5334 m deck-edge derivation. The telephone-pole exclusion "
        "already includes 0.6096 m clearance and a 0.02 m numerical margin. Do not add either adjustment a second time.\n\n"
        "## Status\n\n"
        "Revision 1 was reviewed on 2026-09-25. New mowing boundaries and obstacle exclusions remain "
        "`reviewed_not_field_validated` until the next field run. Existing transitions are stored as "
        "`approved_existing` and remain direction-specific.\n\n"
        "## Revision history\n\n"
        f"The immutable revision snapshot is in `revisions/{REVISION_LABEL}/`.\n",
        encoding="utf-8",
        newline="\n",
    )

    for path in (CURRENT_GEOJSON, CURRENT_CSV, CURRENT_MANIFEST, CURRENT_VALIDATION, CURRENT_MAP, CURRENT_HTML, CURRENT_README):
        shutil.copyfile(path, HISTORY / path.name)

    print(json.dumps({
        "output": str(OUT.relative_to(REPO)).replace("\\", "/"),
        "revision": REVISION_LABEL,
        "counts": manifest["counts"],
        "mowable_area_total_m2": manifest["mowable_area_total_m2"],
        "transition_total_length_m": manifest["transition_total_length_m"],
    }, indent=2))


if __name__ == "__main__":
    main()
