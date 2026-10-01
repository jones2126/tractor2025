#!/usr/bin/env python3
"""Build review-only 62 Collins perimeter candidate v2 from Al's map notes."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString, Point, Polygon, box, mapping, shape
from shapely.ops import transform, unary_union

from build_20260923_combined_coverage_review import (
    DECK_RADIUS_M,
    MANUAL,
    REPO,
    REVIEW,
    SITE,
    Frame,
    largest_polygon,
    polygons_of,
)


SOURCE_GEOJSON = (
    SITE
    / "analysis/20260923_combined_coverage_review/combined_coverage_boundaries_REVIEW_ONLY.geojson"
)
SOURCE_REPORT = (
    SITE / "analysis/20260923_combined_coverage_review/combined_coverage_report.json"
)
OUT = SITE / "analysis/20260928_perimeter_candidate_v2_REVIEW_ONLY"

TREE_SOURCE_ROWS = (3107, 3584)
POLE_LOOP_START = 13
POLE_LOOP_END = 63
POLE_EXTRA_CLEARANCE_M = 0.6096
POLE_NUMERICAL_MARGIN_M = 0.02

COLORS = {
    "backyard_and_gardens": "#1565c0",
    "front_yard": "#ef6c00",
    "over_the_road": "#795548",
    "tree": "#c62828",
    "pole": "#6a1b9a",
}


def load_frame() -> Frame:
    with (REVIEW / "segment_02_candidate_path.csv").open(
        newline="", encoding="utf-8-sig"
    ) as handle:
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

    converted = transform(convert, geometry)
    return converted if converted.is_valid else converted.buffer(0)


def load_v1(frame: Frame):
    collection = json.loads(SOURCE_GEOJSON.read_text(encoding="utf-8"))
    result = {}
    for feature in collection["features"]:
        properties = feature["properties"]
        if properties["kind"] == "proposed_full_area":
            result[properties["area"]] = to_xy(frame, shape(feature["geometry"]))
    expected = {"main_backyard", "garden_right", "garden_left", "front_yard", "over_the_road"}
    if set(result) != expected:
        raise ValueError(f"Unexpected v1 perimeter fields: {sorted(result)}")
    return result


def fill_holes(polygon):
    parts = polygons_of(polygon.buffer(0))
    filled = [Polygon(part.exterior) for part in parts]
    return unary_union(filled).buffer(0)


def smooth_garden_right_crown(polygon):
    """Replace only the sawtooth crown with a bounded quadratic curve."""
    polygon = largest_polygon(polygon.buffer(0))
    coordinates = list(polygon.exterior.coords)[:-1]
    candidates = [
        (index, point)
        for index, point in enumerate(coordinates)
        if -30.6 <= point[0] <= -24.0 and point[1] >= 14.0
    ]
    if len(candidates) < 4:
        raise ValueError("Garden-right crown anchors were not found")
    start_index = candidates[0][0]
    end_index = candidates[-1][0]
    if start_index >= end_index:
        raise ValueError("Garden-right crown is not a contiguous exterior arc")
    start = coordinates[start_index]
    end = coordinates[end_index]
    demonstrated_top = max(point[1] for _index, point in candidates)
    target_apex = demonstrated_top - 0.10
    control = (
        (start[0] + end[0]) / 2.0,
        2.0 * target_apex - (start[1] + end[1]) / 2.0,
    )
    curve = []
    for step in range(25):
        t = step / 24.0
        one = 1.0 - t
        curve.append((
            one * one * start[0] + 2.0 * one * t * control[0] + t * t * end[0],
            one * one * start[1] + 2.0 * one * t * control[1] + t * t * end[1],
        ))
    replaced = coordinates[:start_index] + curve + coordinates[end_index + 1 :]
    smoothed = Polygon(replaced).buffer(0)
    smoothed = largest_polygon(smoothed)
    details = {
        "start_xy_m": [start[0], start[1]],
        "end_xy_m": [end[0], end[1]],
        "source_max_y_m": demonstrated_top,
        "smoothed_apex_y_m": target_apex,
        "replacement_points": len(curve),
    }
    return smoothed, details


def load_tree_loop(frame: Frame):
    points = []
    start_time = end_time = None
    with MANUAL.open(newline="", encoding="utf-8-sig") as handle:
        for source_row, row in enumerate(csv.DictReader(handle), 2):
            if not TREE_SOURCE_ROWS[0] <= source_row <= TREE_SOURCE_ROWS[1]:
                continue
            try:
                valid = (
                    row["trans_mode"] == "1"
                    and row["fix_quality"] == "RTK Fixed"
                    and row["head_valid"].strip().lower() == "true"
                    and float(row["speed_mps"] or 0.0) >= 0.12
                )
                if not valid:
                    continue
                point = frame.xy(float(row["lat"]), float(row["lon"]))
            except (KeyError, TypeError, ValueError):
                continue
            if not points or math.dist(points[-1], point) >= 0.05:
                points.append(point)
                start_time = start_time or row["time"]
                end_time = row["time"]
    if len(points) < 20:
        raise ValueError("Too few valid points in the agreed tree loop")
    coordinates = np.asarray(points)
    matrix = np.c_[2.0 * coordinates[:, 0], 2.0 * coordinates[:, 1], np.ones(len(points))]
    squared = np.sum(coordinates * coordinates, axis=1)
    center_x, center_y, constant = np.linalg.lstsq(matrix, squared, rcond=None)[0]
    radius = math.sqrt(constant + center_x * center_x + center_y * center_y)
    residuals = np.hypot(coordinates[:, 0] - center_x, coordinates[:, 1] - center_y) - radius
    centerline_polygon = Point(center_x, center_y).buffer(radius, resolution=48)
    deck_edge_hole = centerline_polygon.buffer(-DECK_RADIUS_M, join_style="round")
    deck_edge_hole = largest_polygon(deck_edge_hole.buffer(0))
    line = LineString(points)
    return line, centerline_polygon, deck_edge_hole, {
        "source_rows": list(TREE_SOURCE_ROWS),
        "start_time": start_time,
        "end_time": end_time,
        "points": len(points),
        "path_length_m": line.length,
        "closure_gap_m": math.dist(points[0], points[-1]),
        "fitted_center_xy_m": [center_x, center_y],
        "fitted_centerline_radius_m": radius,
        "fit_radial_rms_m": float(np.sqrt(np.mean(residuals * residuals))),
        "fit_radial_max_abs_m": float(np.max(np.abs(residuals))),
        "centerline_enclosed_area_m2": centerline_polygon.area,
        "deck_edge_hole_area_m2": deck_edge_hole.area,
        "deck_edge_inset_m": DECK_RADIUS_M,
    }


def load_pole(frame: Frame):
    with (REVIEW / "segment_14_candidate_path.csv").open(
        newline="", encoding="utf-8-sig"
    ) as handle:
        rows = list(csv.DictReader(handle))
    selected = rows[POLE_LOOP_START : POLE_LOOP_END + 1]
    points = [frame.xy(float(row["lat"]), float(row["lon"])) for row in selected]
    loop = largest_polygon(Polygon(points).buffer(0))
    exclusion = loop.buffer(
        POLE_EXTRA_CLEARANCE_M + POLE_NUMERICAL_MARGIN_M,
        join_style="round",
    )
    return LineString(points), loop, exclusion


def polygon_payload(geometry):
    return mapping(geometry) if not geometry.is_empty else None


def line_payload(line):
    return [[round(x, 3), round(y, 3)] for x, y in line.coords]


def load_manual_trail(frame: Frame):
    points = []
    with MANUAL.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            try:
                if row["trans_mode"] != "1" or float(row["speed_mps"] or 0.0) < 0.12:
                    continue
                point = frame.xy(float(row["lat"]), float(row["lon"]))
            except (KeyError, TypeError, ValueError):
                continue
            if not points or math.dist(points[-1], point) >= 0.15:
                points.append(point)
    return points


def write_html(payload):
    template = """<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>62 Collins perimeter candidate v2</title><style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#10151c;color:#e8eef5;font:14px system-ui,sans-serif}header{height:70px;padding:11px 16px;background:#17202a;border-bottom:1px solid #34404e}h1{font-size:19px;margin:0 0 5px}.warn{color:#ffcc80}#layout{display:grid;grid-template-columns:315px 1fr;height:calc(100vh - 70px)}aside{padding:14px;overflow:auto;border-right:1px solid #34404e;background:#141b24}label{display:block;margin:9px 0}button{background:#263545;color:#fff;border:1px solid #526579;border-radius:5px;padding:7px 10px;margin-right:5px}canvas{width:100%;height:100%;display:block;background:#f8fafc}.small{font-size:12px;color:#afbdca;line-height:1.45}.legend{display:grid;grid-template-columns:14px 1fr;gap:7px;margin:8px 0}.sw{height:12px;margin-top:3px}
</style></head><body><header><h1>62 Collins perimeter candidate v2</h1><div class="warn">REVIEW ONLY — no mission or launcher has been generated.</div></header><div id="layout"><aside>
<b>Layers</b><label><input id="v2" type="checkbox" checked> Candidate v2</label><label><input id="v1" type="checkbox" checked> Candidate v1 outlines</label><label><input id="manual" type="checkbox"> September 23 manual trail</label><label><input id="treeLine" type="checkbox" checked> Recorded tree centerline loop</label><label><input id="obstacles" type="checkbox" checked> Tree and pole exclusions</label><label><input id="notes" type="checkbox" checked> Review-note markers</label>
<p><button id="fit">Fit all</button><button id="reset">Reset</button></p>
<div class="legend"><span class="sw" style="background:#1565c0"></span><span>Backyard and gardens</span><span class="sw" style="background:#ef6c00"></span><span>Front yard</span><span class="sw" style="background:#795548"></span><span>Over the road</span><span class="sw" style="background:#c62828"></span><span>Tree deck-edge hole</span><span class="sw" style="background:#6a1b9a"></span><span>Telephone-pole clearance</span></div>
<p class="small">V2 dissolves the backyard and both garden regions into one polygon, keeps the original subarea names only as planning metadata, replaces the garden-right crown with a bounded smooth curve, and retains two explicit obstacle constraints.</p>
</aside><canvas id="map"></canvas></div><script>
const D=__PAYLOAD__,$=id=>document.getElementById(id),C=$('map'),X=C.getContext('2d');let scale=8,ox=0,oy=0,drag=null;
function screen(p){return[ox+p[0]*scale,oy-p[1]*scale]}function rings(g){if(!g)return[];return g.type==='Polygon'?[g.coordinates]:g.coordinates}
function fill(g,color,stroke,w=1){if(!g)return;X.save();X.fillStyle=color;X.strokeStyle=stroke;X.lineWidth=w;for(const poly of rings(g)){X.beginPath();for(const ring of poly){ring.forEach((p,i)=>{const q=screen(p);i?X.lineTo(...q):X.moveTo(...q)});X.closePath()}X.fill('evenodd');X.stroke()}X.restore()}
function line(points,color,w=1,dash=[]){if(!points?.length)return;X.save();X.strokeStyle=color;X.lineWidth=w;X.setLineDash(dash);X.beginPath();points.forEach((p,i)=>{const q=screen(p);i?X.lineTo(...q):X.moveTo(...q)});X.stroke();X.restore()}
function outline(g,color,w=1,dash=[]){for(const poly of rings(g)){for(const ring of poly)line(ring,color,w,dash)}}
function label(p,t){const q=screen(p);X.save();X.fillStyle='#111';X.font='bold 14px system-ui';X.fillText(t,q[0]+5,q[1]-5);X.restore()}
function draw(){const r=C.getBoundingClientRect();X.clearRect(0,0,r.width,r.height);X.fillStyle='#f8fafc';X.fillRect(0,0,r.width,r.height);if($('v2').checked){fill(D.v2.backyard_and_gardens,'#1565c02b','#1565c0',2.2);fill(D.v2.front_yard,'#ef6c002b','#ef6c00',2.2);fill(D.v2.over_the_road,'#7955482b','#795548',2.2)}if($('v1').checked)Object.values(D.v1).forEach((g,i)=>outline(g,'#607d8b',1,[5,4]));if($('manual').checked)line(D.manual,'#ff1744',.8);if($('treeLine').checked){line(D.treeLine,'#d81b60',1.5);outline(D.treeFit,'#d81b60',1.8,[4,3])}if($('obstacles').checked){fill(D.treeHole,'#ef535088','#c62828',1.5);fill(D.poleExclusion,'#ab47bc66','#6a1b9a',1.5);line(D.poleLine,'#6a1b9a',1.4)}if($('notes').checked)D.notes.forEach(n=>label(n.xy,n.label))}
function allcoords(g,out=[]){for(const poly of rings(g))for(const ring of poly)out.push(...ring);return out}function fitView(){let p=[];Object.values(D.v2).forEach(g=>allcoords(g,p));let xs=p.map(q=>q[0]),ys=p.map(q=>q[1]),r=C.getBoundingClientRect(),pad=35;scale=Math.min((r.width-2*pad)/(Math.max(...xs)-Math.min(...xs)),(r.height-2*pad)/(Math.max(...ys)-Math.min(...ys)));ox=pad-Math.min(...xs)*scale;oy=pad+Math.max(...ys)*scale;draw()}
C.onwheel=e=>{e.preventDefault();const r=C.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,s=e.deltaY<0?1.15:1/1.15;ox=mx-(mx-ox)*s;oy=my-(my-oy)*s;scale*=s;draw()};C.onpointerdown=e=>drag=[e.clientX,e.clientY,ox,oy];C.onpointermove=e=>{if(drag){ox=drag[2]+e.clientX-drag[0];oy=drag[3]+e.clientY-drag[1];draw()}};C.onpointerup=()=>drag=null;C.onpointerleave=()=>drag=null;document.querySelectorAll('input').forEach(i=>i.onchange=draw);$('fit').onclick=fitView;$('reset').onclick=()=>{$('v2').checked=$('v1').checked=$('treeLine').checked=$('obstacles').checked=$('notes').checked=true;$('manual').checked=false;draw()};window.onresize=()=>{const r=C.getBoundingClientRect(),d=devicePixelRatio||1;C.width=r.width*d;C.height=r.height*d;X.setTransform(d,0,0,d,0,0);fitView()};window.onresize();
</script></body></html>"""
    (OUT / "62_Collins_perimeter_candidate_v2_INTERACTIVE_REVIEW.html").write_text(
        template.replace("__PAYLOAD__", json.dumps(payload, separators=(",", ":"))),
        encoding="utf-8",
        newline="\n",
    )


def draw_polygon(axis, geometry, color, label, alpha=0.16, width=2.0):
    first = True
    for polygon in polygons_of(geometry):
        x, y = polygon.exterior.xy
        axis.fill(x, y, color=color, alpha=alpha)
        axis.plot(x, y, color=color, linewidth=width, label=label if first else None)
        for interior in polygon.interiors:
            ix, iy = interior.xy
            axis.fill(ix, iy, color="white", alpha=1.0)
            axis.plot(ix, iy, color=color, linewidth=width)
        first = False


def write_preview(v1, v2, tree_line, tree_fit, tree_hole, pole_line, pole_exclusion, crown_details):
    figure, axis = plt.subplots(figsize=(12, 10), dpi=170)
    for geometry in v1.values():
        for polygon in polygons_of(geometry):
            x, y = polygon.exterior.xy
            axis.plot(x, y, color="#78909c", linewidth=0.9, linestyle="--", alpha=0.75)
    draw_polygon(axis, v2["backyard_and_gardens"], COLORS["backyard_and_gardens"], "backyard and gardens")
    draw_polygon(axis, v2["front_yard"], COLORS["front_yard"], "front yard")
    draw_polygon(axis, v2["over_the_road"], COLORS["over_the_road"], "over the road")
    tx, ty = tree_line.xy
    axis.plot(tx, ty, color="#d81b60", linewidth=1.8, label="recorded innermost tree loop")
    fx, fy = tree_fit.exterior.xy
    axis.plot(fx, fy, color="#d81b60", linewidth=1.4, linestyle="--", label="fitted tree centerline boundary")
    hx, hy = tree_hole.exterior.xy
    axis.fill(hx, hy, color="#ef5350", alpha=0.65, label="tree deck-edge exclusion")
    px, py = pole_line.xy
    axis.plot(px, py, color="#6a1b9a", linewidth=1.4, label="recorded pole loop")
    ex, ey = pole_exclusion.exterior.xy
    axis.plot(ex, ey, color="#6a1b9a", linewidth=1.8, linestyle=":", label="pole clearance")
    axis.plot([], [], color="#78909c", linestyle="--", label="candidate v1 outlines")
    axis.annotate("1  joined", xy=(-44, -14), xytext=(-54, -7), arrowprops={"arrowstyle": "->", "color": "#333"})
    tree_center = tree_hole.centroid
    axis.annotate("2  tree exclusion", xy=(tree_center.x, tree_center.y), xytext=(-11, -16), arrowprops={"arrowstyle": "->", "color": "#333"})
    crown_xy = ((crown_details["start_xy_m"][0] + crown_details["end_xy_m"][0]) / 2.0, crown_details["smoothed_apex_y_m"])
    axis.annotate("3  smoothed crown", xy=crown_xy, xytext=(-18, 20), arrowprops={"arrowstyle": "->", "color": "#333"})
    pole_center = pole_exclusion.centroid
    axis.annotate("4  telephone pole", xy=(pole_center.x, pole_center.y), xytext=(42, 2), arrowprops={"arrowstyle": "->", "color": "#333"})
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.22)
    axis.set_xlabel("east (m)")
    axis.set_ylabel("north (m)")
    axis.set_title("62 Collins perimeter candidate v2 — review only")
    axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(OUT / "62_Collins_perimeter_candidate_v2_REVIEW.png")
    plt.close(figure)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    frame = load_frame()
    v1 = load_v1(frame)
    garden_right_smoothed, crown_details = smooth_garden_right_crown(v1["garden_right"])
    combined_filled = fill_holes(
        unary_union([v1["main_backyard"], v1["garden_left"], garden_right_smoothed])
    )
    combined_filled = largest_polygon(combined_filled)

    tree_line, tree_centerline_polygon, tree_hole, tree_report = load_tree_loop(frame)
    if not combined_filled.covers(tree_hole):
        raise ValueError("Tree exclusion is not contained by the combined backyard/gardens polygon")
    backyard_and_gardens = largest_polygon(combined_filled.difference(tree_hole).buffer(0))

    pole_line, pole_loop, pole_exclusion = load_pole(frame)
    over_the_road = largest_polygon(v1["over_the_road"].difference(pole_exclusion).buffer(0))
    front_yard = largest_polygon(v1["front_yard"].buffer(0))
    v2 = {
        "backyard_and_gardens": backyard_and_gardens,
        "front_yard": front_yard,
        "over_the_road": over_the_road,
    }

    features = []
    for name, geometry in v2.items():
        features.append({
            "type": "Feature",
            "properties": {"name": name, "kind": "mowable_area", "review_only": True},
            "geometry": mapping(to_lonlat(frame, geometry)),
        })
    for name, kind, geometry in (
        ("tree_recorded_innermost_centerline", "tested_centerline_evidence", tree_line),
        ("tree_fitted_centerline_boundary", "tested_centerline_limit", tree_centerline_polygon.boundary),
        ("tree_deck_edge_exclusion", "obstacle_hole", tree_hole),
        ("telephone_pole_recorded_loop", "obstacle_survey", pole_loop),
        ("telephone_pole_clearance", "obstacle_exclusion", pole_exclusion),
    ):
        features.append({
            "type": "Feature",
            "properties": {"name": name, "kind": kind, "review_only": True},
            "geometry": mapping(to_lonlat(frame, geometry)),
        })
    geojson = {"type": "FeatureCollection", "features": features}
    geojson_path = OUT / "62_Collins_perimeter_candidate_v2_REVIEW_ONLY.geojson"
    geojson_path.write_text(json.dumps(geojson, indent=2) + "\n", encoding="utf-8")

    manual_trail = load_manual_trail(frame)
    payload = {
        "v1": {name: polygon_payload(geometry) for name, geometry in v1.items()},
        "v2": {name: polygon_payload(geometry) for name, geometry in v2.items()},
        "manual": [[round(x, 3), round(y, 3)] for x, y in manual_trail],
        "treeLine": line_payload(tree_line),
        "treeFit": polygon_payload(tree_centerline_polygon),
        "treeHole": polygon_payload(tree_hole),
        "poleLine": line_payload(pole_line),
        "poleExclusion": polygon_payload(pole_exclusion),
        "notes": [
            {"label": "1  joined", "xy": [-44.0, -14.0]},
            {"label": "2  tree", "xy": [tree_hole.centroid.x, tree_hole.centroid.y]},
            {"label": "3  smoothed", "xy": [(crown_details["start_xy_m"][0] + crown_details["end_xy_m"][0]) / 2.0, crown_details["smoothed_apex_y_m"]]},
            {"label": "4  pole", "xy": [pole_exclusion.centroid.x, pole_exclusion.centroid.y]},
        ],
    }
    write_html(payload)
    write_preview(v1, v2, tree_line, tree_centerline_polygon, tree_hole, pole_line, pole_exclusion, crown_details)

    report = {
        "status": "REVIEW_ONLY_NOT_FIELD_READY",
        "source_review": str(SOURCE_GEOJSON.relative_to(REPO)),
        "source_notes": r"C:\Users\al532\Downloads\20260928\review of map.pptx",
        "deck_width_m": 2.0 * DECK_RADIUS_M,
        "operations": {
            "joined": ["main_backyard", "garden_left", "garden_right"],
            "garden_right_crown": crown_details,
            "tree": tree_report,
            "telephone_pole": {
                "source_segment": 14,
                "loop_indices_zero_based": [POLE_LOOP_START, POLE_LOOP_END],
                "recorded_loop_area_m2": pole_loop.area,
                "extra_clearance_m": POLE_EXTRA_CLEARANCE_M,
                "numerical_margin_m": POLE_NUMERICAL_MARGIN_M,
                "exclusion_area_m2": pole_exclusion.area,
            },
        },
        "areas": {
            name: {
                "area_m2": geometry.area,
                "perimeter_m": geometry.length,
                "interior_holes": sum(len(part.interiors) for part in polygons_of(geometry)),
                "valid": geometry.is_valid,
            }
            for name, geometry in v2.items()
        },
        "source_sha256": {
            str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (SOURCE_GEOJSON, SOURCE_REPORT, MANUAL, REVIEW / "segment_14_candidate_path.csv")
        },
        "limitations": [
            "Review-only perimeter geometry; no mission or launcher is generated.",
            "The tree deck-edge hole assumes a symmetric 42-inch deck centered laterally on the GNSS path.",
            "The garden-right crown replacement is bounded below the highest demonstrated manual coverage point.",
        ],
    }
    (OUT / "62_Collins_perimeter_candidate_v2_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    (OUT / "README.md").write_text(
        "# 62 Collins perimeter candidate v2\n\n"
        "This review-only package applies the four notes in `review of map.pptx`. "
        "It does not contain a mission or launcher.\n\n"
        "Open `62_Collins_perimeter_candidate_v2_INTERACTIVE_REVIEW.html` and compare "
        "the dashed candidate-v1 outlines with the solid candidate-v2 perimeter.\n\n"
        "Rebuild from the repository root with:\n\n"
        "```powershell\npython field_testing/tools/build_20260928_perimeter_candidate_v2.py\n```\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "output": str(OUT.relative_to(REPO)),
        "tree": tree_report,
        "crown": crown_details,
        "areas": report["areas"],
    }, indent=2))


if __name__ == "__main__":
    main()
