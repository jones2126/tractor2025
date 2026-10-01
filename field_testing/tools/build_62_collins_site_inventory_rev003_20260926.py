#!/usr/bin/env python3
"""Build review-only 62 Collins inventory revision 3 from the 2026-09-26 survey."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString, MultiPoint, Point, Polygon, mapping, shape
from shapely.ops import transform

from build_20260923_combined_coverage_review import REVIEW, SITE, Frame, largest_polygon


REPO = Path(__file__).resolve().parents[2]
RUN = SITE / "runs/20260926_phone_manual_perimeter_extension"
SOURCE = RUN / "field_test_20260926_121311.csv"
REV2 = SITE / "site_inventory/62_Collins_site_inventory.geojson"
OUT = SITE / "site_inventory/revisions/rev_003_20260926_REVIEW_ONLY"
ANALYSIS = SITE / "analysis/20260926_perimeter_inventory_update_REVIEW_ONLY"
DECK_RADIUS_M = 1.0668 / 2.0
OWNER_PATH_BUFFER_M = 0.3048
PATH_BASED_OBSTACLES = {"OBS-NEW-003", "OBS-NEW-005", "OBS-NEW-011"}

GEOJSON_OUT = OUT / "62_Collins_site_inventory_rev003_REVIEW_ONLY.geojson"
CSV_OUT = OUT / "62_Collins_site_inventory_rev003_owner_review_worksheet.csv"
REPORT_OUT = OUT / "62_Collins_site_inventory_rev003_report.json"
PNG_OUT = OUT / "62_Collins_site_inventory_rev003_REVIEW.png"
HTML_OUT = OUT / "62_Collins_site_inventory_rev003_INTERACTIVE_REVIEW.html"
README_OUT = OUT / "README.md"

AREA_IDS = {
    "backyard": "AREA-BACKYARD-GARDENS",
    "front": "AREA-FRONT-YARD",
    "overroad": "AREA-OVER-ROAD",
}

PERIMETER_SPECS = [
    {
        "id": "PERIMETER-EVIDENCE-BACKYARD-SOUTHEAST",
        "area": AREA_IDS["backyard"],
        "name": "Backyard southeast extension",
        "parts": [
            ("line", "2026-09-26T16:30:19+00:00", "2026-09-26T16:32:02+00:00"),
            ("range", "2026-09-26T16:32:02+00:00", "2026-09-26T16:33:11+00:00"),
            ("points", [
                "2026-09-26T16:33:11+00:00",
                "2026-09-26T16:37:48+00:00",
                "2026-09-26T16:35:09+00:00",
                "2026-09-26T16:37:56+00:00",
            ]),
            ("range", "2026-09-26T16:37:56+00:00", "2026-09-26T16:40:54+00:00"),
        ],
        "method": "owner-reviewed straight connector and ordered timestamp vertices, followed by recorded arc",
    },
    {
        "id": "PERIMETER-EVIDENCE-BACKYARD-SOUTH",
        "area": AREA_IDS["backyard"],
        "name": "Backyard south extension",
        "ranges": [
            ("2026-09-26T16:43:47+00:00", "2026-09-26T16:44:58+00:00"),
            ("2026-09-26T16:48:18+00:00", "2026-09-26T16:51:33+00:00"),
        ],
        "method": "two demonstrated outer arcs joined at repeated southern point",
    },
    {
        "id": "PERIMETER-EVIDENCE-FRONT",
        "area": AREA_IDS["front"],
        "name": "Front-yard perimeter circuit",
        "ranges": [("2026-09-26T16:52:53+00:00", "2026-09-26T16:56:18+00:00")],
        "method": "complete legacy-handheld perimeter circuit",
    },
    {
        "id": "PERIMETER-EVIDENCE-OVERROAD",
        "area": AREA_IDS["overroad"],
        "name": "Over-road perimeter circuit",
        "ranges": [("2026-09-26T17:06:01+00:00", "2026-09-26T17:10:11+00:00")],
        "method": "complete legacy-handheld perimeter circuit",
    },
]

OBSTACLE_SPECS = [
    ("OBS-NEW-001", "Unclassified surveyed obstacle 1", "2026-09-26T16:20:27+00:00", "2026-09-26T16:22:08+00:00", "high"),
    ("OBS-NEW-002", "Unclassified surveyed obstacle 2", "2026-09-26T16:29:15+00:00", "2026-09-26T16:30:07+00:00", "high"),
    ("OBS-NEW-003", "Unclassified surveyed obstacle 3", "2026-09-26T16:41:04+00:00", "2026-09-26T16:42:27+00:00", "high"),
    ("OBS-NEW-004", "Unclassified surveyed obstacle 4", "2026-09-26T16:42:50+00:00", "2026-09-26T16:43:47+00:00", "high"),
    ("OBS-NEW-005", "Unclassified surveyed obstacle 5", "2026-09-26T16:45:03+00:00", "2026-09-26T16:46:31+00:00", "high"),
    ("OBS-NEW-006", "Unclassified surveyed obstacle 6", "2026-09-26T16:46:51+00:00", "2026-09-26T16:47:52+00:00", "high"),
    ("OBS-NEW-007", "Unclassified surveyed obstacle 7", "2026-09-26T16:56:23+00:00", "2026-09-26T16:57:14+00:00", "high"),
    ("OBS-NEW-008", "Owner-confirmed surveyed obstacle 8", "2026-09-26T16:57:40+00:00", "2026-09-26T16:58:46+00:00", "owner_confirmed"),
    ("OBS-NEW-009", "Unclassified surveyed obstacle 9", "2026-09-26T16:58:51+00:00", "2026-09-26T16:59:57+00:00", "high"),
    ("OBS-NEW-010", "Unclassified surveyed obstacle 10", "2026-09-26T17:00:02+00:00", "2026-09-26T17:01:23+00:00", "high"),
    ("OBS-NEW-011", "Unclassified surveyed obstacle 11", "2026-09-26T17:01:54+00:00", "2026-09-26T17:03:38+00:00", "high"),
    ("OBS-NEW-012", "Unclassified surveyed obstacle 12", "2026-09-26T17:04:49+00:00", "2026-09-26T17:05:10+00:00", "low_single_loop"),
    ("OBS-NEW-013", "Owner-confirmed surveyed obstacle 13", "2026-09-26T16:18:12+00:00", "2026-09-26T16:18:49+00:00", "owner_confirmed"),
    ("OBS-NEW-014", "Owner-confirmed surveyed obstacle 14", "2026-09-26T16:51:48+00:00", "2026-09-26T16:52:41+00:00", "owner_confirmed"),
]

# Keep the complete replay interval, but exclude straight ingress/egress portions
# from the circular center calculation for these owner-reviewed obstacles.
CIRCLE_FIT_INTERVALS = {
    "OBS-NEW-004": ("2026-09-26T16:42:59+00:00", "2026-09-26T16:43:42+00:00"),
    "OBS-NEW-006": ("2026-09-26T16:46:56+00:00", "2026-09-26T16:47:48+00:00"),
    "OBS-NEW-013": ("2026-09-26T16:18:23+00:00", "2026-09-26T16:18:42+00:00"),
    "OBS-NEW-014": ("2026-09-26T16:52:03+00:00", "2026-09-26T16:52:37+00:00"),
}

REJECTED_OBSTACLES = []

POLE_REFINEMENT = (
    "OBS-POLE-001",
    "Telephone pole over the road",
    "2026-09-26T17:03:58+00:00",
    "2026-09-26T17:04:39+00:00",
    "high_existing_obstacle_resurvey",
)


def parse_time(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


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

    converted = transform(convert, geometry)
    if converted.geom_type in {"Polygon", "MultiPolygon"} and not converted.is_valid:
        converted = converted.buffer(0)
    return converted


def load_rows(frame: Frame):
    rows = []
    with SOURCE.open(newline="", encoding="utf-8-sig") as handle:
        for source_row, row in enumerate(csv.DictReader(handle), 2):
            try:
                timestamp = parse_time(row["time"])
                speed = float(row.get("speed_mps") or 0.0)
                lat, lon = float(row["lat"]), float(row["lon"])
            except (KeyError, TypeError, ValueError):
                continue
            if row.get("trans_mode") not in {"0", "1"} or speed < 0.12:
                continue
            x, y = frame.xy(lat, lon)
            heading_accuracy = None
            try:
                heading_accuracy = float(row.get("relpos_heading_accuracy_deg") or "")
            except ValueError:
                pass
            head_valid = str(row.get("head_valid", "")).strip().lower() == "true"
            carrier = str(row.get("carrier", "") or "none").strip().lower()
            if head_valid and carrier == "fixed" and heading_accuracy is not None and heading_accuracy <= 1.0:
                heading_quality = "good"
            elif head_valid and carrier in {"fixed", "float"} and (heading_accuracy is None or heading_accuracy <= 5.0):
                heading_quality = "moderate"
            else:
                heading_quality = "poor"
            rows.append({
                "source_row": source_row,
                "time": timestamp,
                "time_text": row["time"],
                "mode": row["trans_mode"],
                "x": x,
                "y": y,
                "lat": lat,
                "lon": lon,
                "speed": speed,
                "heading": float(row["heading_deg"]) if row.get("heading_deg") else None,
                "heading_accuracy": heading_accuracy,
                "heading_quality": heading_quality,
                "head_valid": head_valid,
                "carrier": carrier,
                "fix_quality": str(row.get("fix_quality", "") or "Unknown"),
            })
    return rows


def select(rows, start_text, end_text):
    start, end = parse_time(start_text), parse_time(end_text)
    selected = [row for row in rows if start <= row["time"] <= end]
    if len(selected) < 10:
        raise ValueError(f"Too few points in {start_text} to {end_text}: {len(selected)}")
    sampled = []
    previous = None
    for row in selected:
        if previous is None or math.hypot(row["x"] - previous["x"], row["y"] - previous["y"]) >= 0.05:
            sampled.append(row)
            previous = row
    return sampled


def combine_ranges(rows, ranges):
    combined = []
    gaps = []
    for start, end in ranges:
        part = select(rows, start, end)
        if combined:
            gaps.append(math.hypot(part[0]["x"] - combined[-1]["x"], part[0]["y"] - combined[-1]["y"]))
        combined.extend(part)
    return combined, gaps


def nearest_row(rows, time_text):
    target = parse_time(time_text)
    return min(rows, key=lambda row: abs((row["time"] - target).total_seconds()))


def combine_parts(rows, parts):
    """Build an owner-reviewed path from recorded ranges and explicit straight vertices."""
    combined = []
    gaps = []
    for part in parts:
        kind = part[0]
        if kind == "range":
            selected = select(rows, part[1], part[2])
        elif kind == "line":
            selected = [nearest_row(rows, part[1]), nearest_row(rows, part[2])]
        elif kind == "points":
            selected = [nearest_row(rows, stamp) for stamp in part[1]]
        else:
            raise ValueError(f"Unknown perimeter part: {kind}")
        if combined:
            gaps.append(math.hypot(selected[0]["x"] - combined[-1]["x"], selected[0]["y"] - combined[-1]["y"]))
            if selected[0]["source_row"] == combined[-1]["source_row"]:
                selected = selected[1:]
        combined.extend(selected)
    return combined, gaps


def path_polygon(points):
    polygon = Polygon([(point["x"], point["y"]) for point in points]).buffer(0)
    return largest_polygon(polygon).simplify(0.08, preserve_topology=True)


def fit_circle(points):
    coordinates = np.asarray([(point["x"], point["y"]) for point in points])
    matrix = np.c_[2.0 * coordinates[:, 0], 2.0 * coordinates[:, 1], np.ones(len(points))]
    squared = np.sum(coordinates * coordinates, axis=1)
    center_x, center_y, constant = np.linalg.lstsq(matrix, squared, rcond=None)[0]
    radius = math.sqrt(max(0.0, constant + center_x * center_x + center_y * center_y))
    radial = np.hypot(coordinates[:, 0] - center_x, coordinates[:, 1] - center_y)
    residuals = radial - radius
    centerline_limit = Point(center_x, center_y).buffer(radius, resolution=64)
    exclusion_radius = max(0.15, radius - DECK_RADIUS_M)
    exclusion = Point(center_x, center_y).buffer(exclusion_radius, resolution=64)
    return centerline_limit, exclusion, {
        "center_xy_m": [float(center_x), float(center_y)],
        "centerline_radius_m": float(radius),
        "mowing_exclusion_radius_m": float(exclusion_radius),
        "recorded_centerline_min_radius_m": float(np.min(radial)),
        "recorded_centerline_max_radius_m": float(np.max(radial)),
        "closest_implied_deck_edge_radius_m": float(max(0.0, np.min(radial) - DECK_RADIUS_M)),
        "radial_rms_m": float(np.sqrt(np.mean(residuals * residuals))),
        "radial_max_abs_m": float(np.max(np.abs(residuals))),
        "recorded_points": len(points),
        "recorded_path_length_m": LineString(coordinates).length,
        "closure_gap_m": math.dist(coordinates[0], coordinates[-1]),
    }


def polygon_obstacle_candidate(points):
    """Derive an assumption-light polygon alternative from the recorded loop."""
    centerline_limit = MultiPoint([(point["x"], point["y"]) for point in points]).convex_hull
    if centerline_limit.geom_type != "Polygon":
        centerline_limit = centerline_limit.buffer(0.10)
    exclusion = centerline_limit.buffer(-DECK_RADIUS_M)
    if exclusion.is_empty:
        exclusion = centerline_limit.centroid.buffer(0.15, resolution=32)
    return largest_polygon(exclusion.buffer(0)).simplify(0.04, preserve_topology=True)


def minimum_radial_path(points, center_xy, bins=72):
    """Approximate the closest complete GPS loop without replacing it by a circle."""
    cx, cy = center_xy
    samples = [[] for _ in range(bins)]
    for point in points:
        dx, dy = point["x"] - cx, point["y"] - cy
        angle = (math.atan2(dy, dx) + 2.0 * math.pi) % (2.0 * math.pi)
        samples[min(bins - 1, int(angle / (2.0 * math.pi) * bins))].append(math.hypot(dx, dy))
    known = {index: min(values) for index, values in enumerate(samples) if values}
    if len(known) < bins * 0.75:
        raise ValueError(f"Recorded path does not cover enough angles for a minimum loop: {len(known)}/{bins}")
    radii = []
    known_indices = sorted(known)
    for index in range(bins):
        if index in known:
            radii.append(known[index])
            continue
        previous = max((candidate for candidate in known_indices if candidate < index), default=known_indices[-1] - bins)
        following = min((candidate for candidate in known_indices if candidate > index), default=known_indices[0] + bins)
        fraction = (index - previous) / (following - previous)
        radii.append(known[previous % bins] + fraction * (known[following % bins] - known[previous % bins]))
    coordinates = []
    for index, radius in enumerate(radii):
        angle = (index + 0.5) * 2.0 * math.pi / bins
        coordinates.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return Polygon(coordinates).buffer(0).simplify(0.04, preserve_topology=True)


def feature(feature_id, properties, geometry):
    return {
        "type": "Feature",
        "id": feature_id,
        "properties": properties,
        "geometry": mapping(geometry),
    }


def draw_polygon(axis, geometry, color, alpha=0.16, width=1.8):
    parts = [geometry] if geometry.geom_type == "Polygon" else list(geometry.geoms)
    for part in parts:
        x, y = part.exterior.xy
        axis.fill(x, y, facecolor=color, edgecolor=color, alpha=alpha, linewidth=width)
        for ring in part.interiors:
            x, y = ring.xy
            axis.fill(x, y, facecolor="white", edgecolor=color, alpha=1.0, linewidth=width)


def write_html(payload):
    template = """<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>62 Collins inventory revision 3 review and replay</title><style>
*{box-sizing:border-box}body{margin:0;background:#101820;color:#eef5fa;font:14px system-ui,sans-serif}header{padding:10px 16px;background:#172431;border-bottom:1px solid #3d5060}h1{font-size:19px;margin:0 0 4px}.warn{color:#ffd180}main{display:grid;grid-template-columns:365px 1fr;height:calc(100vh - 68px)}aside{padding:14px;overflow:auto;border-right:1px solid #3d5060}h2{font-size:14px;color:#8dd5ff;margin:16px 0 7px}label{display:block;margin:7px 0}.small{font-size:12px;color:#b9cbd8;line-height:1.45}.card{background:#1d2c39;border-radius:7px;padding:9px;margin:8px 0}button,select{background:#2a3e4f;color:#fff;border:1px solid #5b7182;border-radius:5px;padding:6px 8px}button{cursor:pointer}.row{display:flex;gap:6px;align-items:center;flex-wrap:wrap}.grow{flex:1}.clock{font:700 15px ui-monospace,monospace;color:#8dd5ff}input[type=range]{width:100%}.key{display:inline-block;width:17px;height:4px;margin:0 6px 2px 0}.details{min-height:88px;white-space:normal}canvas{width:100%;height:100%;display:block;background:#f8fafc}@media(max-width:760px){main{grid-template-columns:1fr;grid-template-rows:auto 70vh}aside{max-height:48vh}}
</style></head><body><header><h1>62 Collins site inventory — revision 3 candidate and field replay</h1><div class="warn">REVIEW ONLY — no autonomous mission has been generated.</div></header><main><aside>
<h2>Map layers</h2>
<label><input id="old" type="checkbox" checked> Revision 2 outlines</label><label><input id="areas" type="checkbox" checked> Revision 3 candidate areas</label><label><input id="added" type="checkbox" checked> Proposed added mowing area</label><label><input id="evidence" type="checkbox"> Recorded perimeter evidence</label>
<label><input id="travel" type="checkbox" checked> Actual path travelled</label><div class="small"><span class="key" style="background:#087f5b"></span>Phone <span class="key" style="background:#1864ab;margin-left:12px"></span>Legacy handheld</div>
<h2>Obstacle comparison</h2>
<label><input id="circleModel" type="checkbox" checked> Active reviewed keep-out shapes</label><label><input id="centerlineModel" type="checkbox" checked> Fitted GPS centerline circles</label><label><input id="polygonModel" type="checkbox"> Polygon alternatives</label><label><input id="loops" type="checkbox" checked> Recorded obstacle loops</label><label><input id="labels" type="checkbox" checked> Obstacle labels</label>
<div class="small"><span class="key" style="background:#ea580c"></span>Active reviewed keep-out <span class="key" style="background:#0369a1;margin-left:10px"></span>Fitted GPS circle<br><span class="key" style="background:#6a1b9a"></span>Polygon alternative <span class="key" style="background:#d81b60;margin-left:10px"></span>Recorded GPS loop <span style="color:#fff;margin-left:10px">●</span> Derived center</div>
<div class="row"><select id="obstacleSelect" class="grow"></select><button id="focusObstacle">Focus</button><button id="replayObstacle">Replay selected</button></div><div id="obstacleDetails" class="card details small"></div>
<h2>Replay</h2><div class="card"><div class="row"><button id="play">▶ Play</button><button id="back">−10 s</button><button id="forward">+10 s</button><button id="resetReplay">Reset</button><select id="speed"><option value="1">1×</option><option value="5">5×</option><option value="10">10×</option><option value="20" selected>20×</option><option value="50">50×</option></select></div><input id="timeline" type="range" min="0" value="0" step="0.1"><div class="row"><span id="clock" class="clock"></span><span id="replayStatus" class="small"></span></div><div class="small" style="margin-top:7px"><b>Position circle — RTK:</b> <span style="color:#16a34a">● Fixed</span> · <span style="color:#f59e0b">● Float</span> · <span style="color:#38bdf8">● DGPS</span> · <span style="color:#dc2626">● Other</span><br><b>Heading line:</b> <span style="color:#16a34a">good</span> · <span style="color:#f59e0b">moderate</span> · <span style="color:#dc2626">poor/invalid</span></div></div>
<div class="card">Backyard added: <b>__BACK__ m²</b><br>Front added: <b>__FRONT__ m²</b><br>Over-road added: <b>__ROAD__ m²</b></div><p class="small">Dashed circles are fitted references, not clearance guarantees. OBS-NEW-003, 005, and 011 use their minimum recorded GPS loop plus a 12-inch outward review buffer. OBS-NEW-012 remains lower confidence.</p>
</aside><canvas id="map"></canvas></main><script>
const D=__PAYLOAD__,$=id=>document.getElementById(id),C=$('map'),X=C.getContext('2d');let s=8,ox=0,oy=0,drag=null,playing=false,replayT=0,replayStop=null,lastFrame=null;const duration=D.replay.at(-1)[0];$('timeline').max=duration;
function pt(p){return[ox+p[0]*s,oy-p[1]*s]}function polys(g){return g.type==='Polygon'?[g.coordinates]:g.type==='MultiPolygon'?g.coordinates:[]}function path(a,close=false){X.beginPath();a.forEach((p,i)=>{let q=pt(p);i?X.lineTo(...q):X.moveTo(...q)});if(close)X.closePath()}function fill(g,c,stroke,w=1){if(!g)return;X.save();X.fillStyle=c;X.strokeStyle=stroke;X.lineWidth=w;for(const p of polys(g)){X.beginPath();for(const r of p){r.forEach((q,i)=>{q=pt(q);i?X.lineTo(...q):X.moveTo(...q)});X.closePath()}X.fill('evenodd');X.stroke()}X.restore()}function line(a,c,w=1,d=[]){if(!a?.length)return;X.save();X.strokeStyle=c;X.lineWidth=w;X.setLineDash(d);path(a);X.stroke();X.restore()}function outline(g,c,w=1,d=[]){for(const p of polys(g))for(const r of p)line(r,c,w,d)}function label(p,t,c='#111'){let q=pt(p);X.fillStyle=c;X.font='bold 11px system-ui';X.fillText(t,q[0]+4,q[1]-4)}
function replayIndex(t){let lo=0,hi=D.replay.length-1;while(lo<hi){let m=Math.ceil((lo+hi)/2);if(D.replay[m][0]<=t)lo=m;else hi=m-1}return lo}function drawRoute(mode,color,width=1.2,alpha=.75){X.save();X.strokeStyle=color;X.globalAlpha=alpha;X.lineWidth=width;X.beginPath();let active=false;for(const p of D.replay){if(p[3]!==mode||p[6]){active=false;if(p[3]!==mode)continue}let q=pt([p[1],p[2]]);active?X.lineTo(...q):X.moveTo(...q);active=true}X.stroke();X.restore()}
function rtkColor(f){if(f==='RTK Fixed')return'#16a34a';if(f==='RTK Float')return'#f59e0b';if(f==='DGPS')return'#38bdf8';return'#dc2626'}function headingColor(q){return q==='good'?'#16a34a':q==='moderate'?'#f59e0b':'#dc2626'}
function drawReplay(){let i=replayIndex(replayT),now=D.replay[i],start=Math.max(0,i-1);while(start>0&&D.replay[start][0]>replayT-30&&!D.replay[start][6])start--;let trail=[];for(let n=start;n<=i;n++){let p=D.replay[n];if(!p[6]||n===i)trail.push([p[1],p[2]])}line(trail,now[3]==='0'?'#00a67a':'#1971c2',3);let q=pt([now[1],now[2]]);X.beginPath();X.arc(...q,8,0,Math.PI*2);X.fillStyle=rtkColor(now[7]);X.fill();X.strokeStyle='#fff';X.lineWidth=2.5;X.stroke();if(now[5]!=null){let a=now[5]*Math.PI/180,tip=[q[0]+Math.sin(a)*21,q[1]-Math.cos(a)*21];X.beginPath();X.moveTo(...q);X.lineTo(...tip);X.strokeStyle=headingColor(now[8]);X.lineWidth=3;X.stroke()}updateReplayText(now)}
function drawObstacleCenter(o,selected){let q=pt(o.center);X.beginPath();X.arc(...q,selected?5:3.5,0,Math.PI*2);X.fillStyle='#111827';X.fill();X.strokeStyle='#fff';X.lineWidth=1.5;X.stroke();if(selected&&o.circleRadius!=null){let outer=[o.center[0],o.center[1]+o.centerlineRadius];if(o.model==='circle_fit_minus_half_deck'){let inner=[o.center[0]+o.circleRadius,o.center[1]];line([o.center,inner],'#ea580c',2);label(inner,'deck-edge r='+o.circleRadius.toFixed(3)+' m','#c2410c')}line([o.center,outer],'#0369a1',2,[6,4]);label(outer,'GPS-fit r='+o.centerlineRadius.toFixed(3)+' m','#075985')}}
function draw(){let r=C.getBoundingClientRect();X.fillStyle='#f8fafc';X.fillRect(0,0,r.width,r.height);if($('old').checked)Object.values(D.old).forEach(g=>outline(g,'#78909c',1,[5,4]));if($('areas').checked){fill(D.areas.backyard,'#1565c022','#1565c0',2);fill(D.areas.front,'#ef6c0022','#ef6c00',2);fill(D.areas.overroad,'#79554822','#795548',2)}if($('added').checked)Object.values(D.added).forEach(g=>fill(g,'#43a04755','#2e7d32',1));if($('evidence').checked)D.perimeters.forEach(a=>line(a,'#00897b',1.5));if($('travel').checked){drawRoute('0','#087f5b');drawRoute('1','#1864ab')}let chosen=$('obstacleSelect').value;D.obstacles.forEach(o=>{let selected=o.id===chosen,w=selected?3:1.4;if($('circleModel').checked)fill(o.circle,o.low?'#facc1538':'#fb923c3d',o.low?'#ca8a04':'#ea580c',w);if($('centerlineModel').checked)outline(o.centerlineCircle,'#0369a1',selected?3:1.5,[7,5]);if($('polygonModel').checked)fill(o.polygon,'#8e24aa33','#6a1b9a',w);if($('loops').checked)line(o.evidence,'#d81b60',selected?2.5:1.2);drawObstacleCenter(o,selected);if($('labels').checked)label(o.center,o.id,selected?'#6a1b9a':'#111')});drawReplay()}
function updateReplayText(p){let stamp=new Date(Date.parse(D.replayStart)+replayT*1000),accuracy=p[9]==null?'--':p[9].toFixed(2)+'°';$('clock').textContent=stamp.toLocaleTimeString();$('replayStatus').textContent=(p[3]==='0'?'Phone':'Legacy handheld')+' · '+p[4].toFixed(2)+' m/s · '+p[7]+' · heading '+p[8]+' ('+p[10]+', '+accuracy+') · '+Math.round(replayT)+' / '+Math.round(duration)+' s'}
function coords(g,o=[]){for(const p of polys(g))for(const r of p)o.push(...r);return o}function fit(){let a=[];Object.values(D.areas).forEach(g=>coords(g,a));let xs=a.map(p=>p[0]),ys=a.map(p=>p[1]),r=C.getBoundingClientRect(),p=30;s=Math.min((r.width-2*p)/(Math.max(...xs)-Math.min(...xs)),(r.height-2*p)/(Math.max(...ys)-Math.min(...ys)));ox=p-Math.min(...xs)*s;oy=p+Math.max(...ys)*s;draw()}
function showObstacle(){let o=D.obstacles.find(x=>x.id===$('obstacleSelect').value);if(!o)return;let range=o.recordedMin==null?'':('<br>Recorded GPS radial range: '+o.recordedMin.toFixed(3)+'–'+o.recordedMax.toFixed(3)+' m<br>Closest implied inner deck edge: '+o.closestDeckEdge.toFixed(3)+' m'),method=o.model==='minimum_recorded_gps_loop_plus_12in_buffer'?('<b>Active model:</b> minimum recorded GPS loop + '+o.ownerBuffer.toFixed(4)+' m (12 in) outward buffer'):('<b>Active model:</b> fitted circle minus half-deck width');$('obstacleDetails').innerHTML='<b>'+o.id+'</b><br>Confidence: '+o.confidence+'<br>'+method+'<br>Center: '+o.centerLatLon[0].toFixed(9)+', '+o.centerLatLon[1].toFixed(9)+'<br>Fitted GPS centerline radius: '+o.centerlineRadius.toFixed(3)+' m'+(o.model==='circle_fit_minus_half_deck'?'<br>Inferred inner deck-edge radius: '+o.circleRadius.toFixed(3)+' m':'<br>Active path-based keep-out area: '+o.activeArea.toFixed(3)+' m²')+'<br>Deck half-width used: '+D.deckRadius.toFixed(4)+' m'+range+'<br>Polygon alternative area: '+o.polygonArea.toFixed(3)+' m²'+(o.rms==null?'':'<br>Circle-fit RMS: '+o.rms.toFixed(3)+' m')+'<br>Evidence: '+o.start.replace('T',' ').slice(0,19)+' to '+o.end.replace('T',' ').slice(0,19);$('replayObstacle').disabled=o.replayStart==null;draw()}
D.obstacles.forEach(o=>{let option=document.createElement('option');option.value=o.id;option.textContent=o.id+(o.low?' — low confidence':'')+(o.model==='minimum_recorded_gps_loop_plus_12in_buffer'?' — path + 12 in':'');$('obstacleSelect').appendChild(option)});$('obstacleSelect').onchange=showObstacle;$('focusObstacle').onclick=()=>{let o=D.obstacles.find(x=>x.id===$('obstacleSelect').value),r=C.getBoundingClientRect();s=28;ox=r.width/2-o.center[0]*s;oy=r.height/2+o.center[1]*s;draw()};$('replayObstacle').onclick=()=>{let o=D.obstacles.find(x=>x.id===$('obstacleSelect').value);if(o.replayStart==null)return;replayT=Math.max(0,o.replayStart-2);replayStop=Math.min(duration,o.replayEnd+2);$('timeline').value=replayT;$('speed').value='5';playing=true;lastFrame=null;$('play').textContent='❚❚ Pause';draw();requestAnimationFrame(animate)};document.querySelectorAll('input[type=checkbox]').forEach(i=>i.onchange=draw);
$('timeline').oninput=()=>{replayStop=null;replayT=+$('timeline').value;draw()};$('play').onclick=()=>{playing=!playing;replayStop=null;$('play').textContent=playing?'❚❚ Pause':'▶ Play';lastFrame=null;if(playing)requestAnimationFrame(animate)};$('resetReplay').onclick=()=>{playing=false;replayStop=null;$('play').textContent='▶ Play';replayT=0;$('timeline').value=0;draw()};$('back').onclick=()=>{replayStop=null;replayT=Math.max(0,replayT-10);$('timeline').value=replayT;draw()};$('forward').onclick=()=>{replayStop=null;replayT=Math.min(duration,replayT+10);$('timeline').value=replayT;draw()};function animate(ts){if(!playing)return;if(lastFrame!=null)replayT+=(ts-lastFrame)/1000*(+$('speed').value);lastFrame=ts;let stop=replayStop==null?duration:replayStop;if(replayT>=stop){replayT=stop;playing=false;replayStop=null;$('play').textContent='▶ Play'}$('timeline').value=replayT;draw();if(playing)requestAnimationFrame(animate)}
C.onpointerdown=e=>drag=[e.clientX,e.clientY,ox,oy];C.onpointermove=e=>{if(drag){ox=drag[2]+e.clientX-drag[0];oy=drag[3]+e.clientY-drag[1];draw()}};C.onpointerup=()=>drag=null;C.onpointerleave=()=>drag=null;C.onwheel=e=>{e.preventDefault();let r=C.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,k=e.deltaY<0?1.15:1/1.15;ox=mx-(mx-ox)*k;oy=my-(my-oy)*k;s*=k;draw()};onresize=()=>{let r=C.getBoundingClientRect(),d=devicePixelRatio||1;C.width=r.width*d;C.height=r.height*d;X.setTransform(d,0,0,d,0,0);fit()};showObstacle();onresize();
</script></body></html>"""
    html = template.replace("__PAYLOAD__", json.dumps(payload, separators=(",", ":")))
    html = html.replace("__BACK__", f"{payload['stats']['backyard_added']:.1f}")
    html = html.replace("__FRONT__", f"{payload['stats']['front_added']:.1f}")
    html = html.replace("__ROAD__", f"{payload['stats']['overroad_added']:.1f}")
    HTML_OUT.write_text(html, encoding="utf-8", newline="\n")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    ANALYSIS.mkdir(parents=True, exist_ok=True)
    frame = load_frame()
    rows = load_rows(frame)
    rev2 = json.loads(REV2.read_text(encoding="utf-8"))

    old_areas = {}
    carried = []
    existing_tree_exclusion = None
    for item in rev2["features"]:
        props = item["properties"]
        geometry = to_xy(frame, shape(item["geometry"]))
        if props.get("category") == "mowable_area":
            old_areas[props["asset_id"]] = geometry
        elif props.get("category") == "obstacle" and props.get("asset_id") == "OBS-POLE-001":
            continue
        else:
            if props.get("asset_id") == "OBS-TREE-001" and props.get("geometry_role") == "mowing_exclusion":
                existing_tree_exclusion = geometry
            copied = json.loads(json.dumps(item))
            copied["properties"]["inventory_revision"] = 3
            copied["properties"]["inventory_revision_label"] = "rev_003_20260926_REVIEW_ONLY"
            carried.append(copied)

    candidate_areas = dict(old_areas)
    perimeter_records = []
    perimeter_lines = []
    for spec in PERIMETER_SPECS:
        selected, gaps = combine_parts(rows, spec["parts"]) if "parts" in spec else combine_ranges(rows, spec["ranges"])
        evidence = LineString([(p["x"], p["y"]) for p in selected]).simplify(0.08)
        proposed = path_polygon(selected)
        before = candidate_areas[spec["area"]]
        after = before.union(proposed).buffer(0)
        candidate_areas[spec["area"]] = largest_polygon(after)
        perimeter_lines.append(evidence)
        perimeter_records.append({
            **spec,
            "points": len(selected),
            "path_length_m": evidence.length,
            "closure_gap_m": math.dist(evidence.coords[0], evidence.coords[-1]),
            "range_join_gaps_m": gaps,
            "proposed_polygon_area_m2": proposed.area,
            "area_added_at_step_m2": candidate_areas[spec["area"]].area - before.area,
        })

    obstacle_records = []
    obstacle_geometries = []
    for asset_id, name, start, end, confidence in [*OBSTACLE_SPECS, POLE_REFINEMENT]:
        selected = select(rows, start, end)
        evidence = LineString([(p["x"], p["y"]) for p in selected]).simplify(0.04)
        fit_start, fit_end = CIRCLE_FIT_INTERVALS.get(asset_id, (start, end))
        fit_selected = select(rows, fit_start, fit_end)
        centerline, circle_exclusion, metrics = fit_circle(fit_selected)
        polygon_exclusion = polygon_obstacle_candidate(selected)
        geometry_method = "circle_fit_minus_half_deck"
        owner_path = None
        exclusion = circle_exclusion
        if asset_id in PATH_BASED_OBSTACLES:
            owner_path = minimum_radial_path(selected, metrics["center_xy_m"])
            exclusion = owner_path.buffer(OWNER_PATH_BUFFER_M).buffer(0)
            geometry_method = "minimum_recorded_gps_loop_plus_12in_buffer"
        related_area = min(candidate_areas, key=lambda key: Point(metrics["center_xy_m"]).distance(candidate_areas[key]))
        center_lat, center_lon = frame.ll(metrics["center_xy_m"][0], metrics["center_xy_m"][1])
        record = {
            "asset_id": asset_id,
            "asset_name": name,
            "confidence": confidence,
            "start": start,
            "end": end,
            "circle_fit_start": fit_start,
            "circle_fit_end": fit_end,
            "related_area": related_area,
            "center_latitude": center_lat,
            "center_longitude": center_lon,
            "polygon_exclusion_area_m2": polygon_exclusion.area,
            "geometry_method": geometry_method,
            "owner_path_buffer_m": OWNER_PATH_BUFFER_M if owner_path is not None else None,
            "active_exclusion_area_m2": exclusion.area,
            **metrics,
        }
        obstacle_records.append(record)
        obstacle_geometries.append((record, evidence, centerline, exclusion, polygon_exclusion, owner_path))
        # High-confidence loops become candidate holes. The single-loop item stays evidence-only.
        if not confidence.startswith("low"):
            candidate_areas[related_area] = candidate_areas[related_area].difference(exclusion).buffer(0)

    features = []
    area_names = {
        AREA_IDS["backyard"]: "Backyard and gardens",
        AREA_IDS["front"]: "Front yard",
        AREA_IDS["overroad"]: "Over the road",
    }
    for asset_id, geometry_xy in candidate_areas.items():
        props = {
            "site_id": "62_COLLINS",
            "inventory_revision": 3,
            "inventory_revision_label": "rev_003_20260926_REVIEW_ONLY",
            "asset_id": asset_id,
            "asset_name": area_names[asset_id],
            "category": "mowable_area",
            "status": "review_only_owner_confirmation_required",
            "geometry_role": "candidate_mowable_boundary",
            "source_date": "2026-09-26",
            "source_file": str(SOURCE.relative_to(REPO)).replace("\\", "/"),
            "area_m2": round(geometry_xy.area, 3),
            "perimeter_m": round(geometry_xy.length, 3),
            "notes": "Revision 2 retained, with only outward demonstrated additions and high-confidence surveyed obstacle holes applied.",
        }
        features.append(feature(f"{asset_id}-BOUNDARY-REV003", props, to_lonlat(frame, geometry_xy)))
        added = geometry_xy.difference(old_areas[asset_id]).buffer(0)
        if not added.is_empty:
            change_props = {
                **props,
                "category": "perimeter_change",
                "geometry_role": "proposed_added_area",
                "area_m2": round(added.area, 3),
            }
            features.append(feature(f"{asset_id}-ADDED-AREA-REV003", change_props, to_lonlat(frame, added)))

    for spec, line in zip(PERIMETER_SPECS, perimeter_lines):
        props = {
            "site_id": "62_COLLINS",
            "inventory_revision": 3,
            "inventory_revision_label": "rev_003_20260926_REVIEW_ONLY",
            "asset_id": spec["id"],
            "asset_name": spec["name"],
            "category": "perimeter_evidence",
            "status": "field_recorded_review_only",
            "geometry_role": "recorded_tractor_centerline",
            "related_area": spec["area"],
            "source_date": "2026-09-26",
            "source_file": str(SOURCE.relative_to(REPO)).replace("\\", "/"),
            "notes": spec["method"],
        }
        features.append(feature(spec["id"], props, to_lonlat(frame, line)))

    for record, evidence, centerline, exclusion, polygon_exclusion, owner_path in obstacle_geometries:
        common = {
            "site_id": "62_COLLINS",
            "inventory_revision": 3,
            "inventory_revision_label": "rev_003_20260926_REVIEW_ONLY",
            "asset_id": record["asset_id"],
            "asset_name": record["asset_name"],
            "category": "obstacle",
            "status": "owner_confirmed_obstacle_review_geometry" if record["confidence"] == "owner_confirmed" else ("candidate_owner_identification_required" if record["asset_id"].startswith("OBS-NEW") else "existing_obstacle_resurveyed"),
            "confidence": record["confidence"],
            "related_area": record["related_area"],
            "source_date": "2026-09-26",
            "source_file": str(SOURCE.relative_to(REPO)).replace("\\", "/"),
            "source_time_start": record["start"],
            "source_time_end": record["end"],
            "deck_edge_inset_m": DECK_RADIUS_M,
            "center_latitude": round(record["center_latitude"], 9),
            "center_longitude": round(record["center_longitude"], 9),
            "fitted_centerline_radius_m": round(record["centerline_radius_m"], 3),
            "circle_exclusion_radius_m": round(record["mowing_exclusion_radius_m"], 3),
            "geometry_method": record["geometry_method"],
            "owner_path_buffer_m": record["owner_path_buffer_m"],
            "polygon_exclusion_area_m2": round(record["polygon_exclusion_area_m2"], 3),
        }
        for suffix, role, geometry in (
            ("CENTER-POINT", "derived_center_point", Point(record["center_xy_m"])),
            ("RECORDED-CENTERLINE-EVIDENCE", "recorded_centerline_evidence", evidence),
            ("TESTED-CENTERLINE-LIMIT", "tested_tractor_centerline_limit", centerline.boundary),
            ("MOWING-EXCLUSION", "candidate_mowing_exclusion", exclusion),
            ("POLYGON-EXCLUSION-ALTERNATIVE", "candidate_polygon_mowing_exclusion", polygon_exclusion),
        ):
            props = {**common, "geometry_role": role}
            features.append(feature(f"{record['asset_id']}-{suffix}-REV003", props, to_lonlat(frame, geometry)))
        if owner_path is not None:
            props = {**common, "geometry_role": "minimum_recorded_gps_loop"}
            features.append(feature(f"{record['asset_id']}-MINIMUM-RECORDED-GPS-LOOP-REV003", props, to_lonlat(frame, owner_path.boundary)))

    if existing_tree_exclusion is not None:
        tree_center = existing_tree_exclusion.centroid
        tree_lat, tree_lon = frame.ll(tree_center.x, tree_center.y)
        tree_radius = math.sqrt(existing_tree_exclusion.area / math.pi)
        tree_props = {
            "site_id": "62_COLLINS",
            "inventory_revision": 3,
            "inventory_revision_label": "rev_003_20260926_REVIEW_ONLY",
            "asset_id": "OBS-TREE-001",
            "asset_name": "Tree at backyard/garden junction",
            "category": "obstacle",
            "status": "carried_forward_from_revision_2",
            "geometry_role": "derived_center_point",
            "related_area": AREA_IDS["backyard"],
            "center_latitude": round(tree_lat, 9),
            "center_longitude": round(tree_lon, 9),
            "fitted_centerline_radius_m": round(tree_radius + DECK_RADIUS_M, 3),
            "circle_exclusion_radius_m": round(tree_radius, 3),
            "source_date": "2026-09-23",
        }
        features.append(feature("OBS-TREE-001-CENTER-POINT-REV003", tree_props, to_lonlat(frame, tree_center)))

    features.extend(carried)
    collection = {
        "type": "FeatureCollection",
        "name": "62 Collins site inventory revision 3 review candidate",
        "properties": {
            "site_id": "62_COLLINS",
            "inventory_revision": 3,
            "inventory_revision_label": "rev_003_20260926_REVIEW_ONLY",
            "created_date": "2026-09-26",
            "status": "REVIEW_ONLY_NOT_FIELD_READY",
            "mission_generated": False,
        },
        "features": features,
    }
    GEOJSON_OUT.write_text(json.dumps(collection, indent=2) + "\n", encoding="utf-8")

    with CSV_OUT.open("w", newline="", encoding="utf-8-sig") as handle:
        columns = [
            "asset_id", "asset_name", "category", "status", "confidence", "related_area",
            "geometry_roles", "source_time_start", "source_time_end", "center_x_m", "center_y_m",
            "center_latitude", "center_longitude", "centerline_radius_m", "mowing_exclusion_radius_m",
            "recorded_centerline_min_radius_m", "recorded_centerline_max_radius_m",
            "closest_implied_deck_edge_radius_m", "geometry_method", "owner_path_buffer_m",
            "active_exclusion_area_m2", "polygon_exclusion_area_m2", "radial_rms_m", "notes",
        ]
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for asset_id, geometry in candidate_areas.items():
            writer.writerow({
                "asset_id": asset_id,
                "asset_name": area_names[asset_id],
                "category": "mowable_area",
                "status": "review_only_owner_confirmation_required",
                "geometry_roles": "candidate_mowable_boundary;proposed_added_area",
                "notes": f"candidate area={geometry.area:.3f} m2",
            })
        writer.writerow({
            "asset_id": "OBS-TREE-001",
            "asset_name": "Tree at backyard/garden junction",
            "category": "obstacle",
            "status": "carried_forward_from_revision_2",
            "confidence": "existing_reviewed_obstacle",
            "related_area": AREA_IDS["backyard"],
            "geometry_roles": "derived_center_point;recorded_centerline_evidence;tested_tractor_centerline_limit;mowing_exclusion",
            "center_x_m": round(existing_tree_exclusion.centroid.x, 3) if existing_tree_exclusion is not None else "",
            "center_y_m": round(existing_tree_exclusion.centroid.y, 3) if existing_tree_exclusion is not None else "",
            "center_latitude": round(frame.ll(existing_tree_exclusion.centroid.x, existing_tree_exclusion.centroid.y)[0], 9) if existing_tree_exclusion is not None else "",
            "center_longitude": round(frame.ll(existing_tree_exclusion.centroid.x, existing_tree_exclusion.centroid.y)[1], 9) if existing_tree_exclusion is not None else "",
            "centerline_radius_m": round(math.sqrt(existing_tree_exclusion.area / math.pi) + DECK_RADIUS_M, 3) if existing_tree_exclusion is not None else "",
            "mowing_exclusion_radius_m": round(math.sqrt(existing_tree_exclusion.area / math.pi), 3) if existing_tree_exclusion is not None else "",
            "polygon_exclusion_area_m2": round(existing_tree_exclusion.area, 3) if existing_tree_exclusion is not None else "",
            "notes": "Geometry carried forward unchanged from revision 2.",
        })
        for record in obstacle_records:
            writer.writerow({
                "asset_id": record["asset_id"],
                "asset_name": record["asset_name"],
                "category": "obstacle",
                "status": "owner_confirmed_obstacle_review_geometry" if record["confidence"] == "owner_confirmed" else ("candidate_owner_identification_required" if record["asset_id"].startswith("OBS-NEW") else "existing_obstacle_resurveyed"),
                "confidence": record["confidence"],
                "related_area": record["related_area"],
                "geometry_roles": "derived_center_point;recorded_centerline_evidence;tested_tractor_centerline_limit;candidate_mowing_exclusion;candidate_polygon_mowing_exclusion",
                "source_time_start": record["start"],
                "source_time_end": record["end"],
                "center_x_m": round(record["center_xy_m"][0], 3),
                "center_y_m": round(record["center_xy_m"][1], 3),
                "center_latitude": round(record["center_latitude"], 9),
                "center_longitude": round(record["center_longitude"], 9),
                "centerline_radius_m": round(record["centerline_radius_m"], 3),
                "mowing_exclusion_radius_m": round(record["mowing_exclusion_radius_m"], 3),
                "recorded_centerline_min_radius_m": round(record["recorded_centerline_min_radius_m"], 3),
                "recorded_centerline_max_radius_m": round(record["recorded_centerline_max_radius_m"], 3),
                "closest_implied_deck_edge_radius_m": round(record["closest_implied_deck_edge_radius_m"], 3),
                "geometry_method": record["geometry_method"],
                "owner_path_buffer_m": record["owner_path_buffer_m"],
                "active_exclusion_area_m2": round(record["active_exclusion_area_m2"], 3),
                "polygon_exclusion_area_m2": round(record["polygon_exclusion_area_m2"], 3),
                "radial_rms_m": round(record["radial_rms_m"], 3),
                "notes": "Owner-confirmed obstacle; geometry remains review-only." if record["confidence"] == "owner_confirmed" else "Generic obstacle identity pending owner review.",
            })
        for asset_id, asset_name, start, end, reason in REJECTED_OBSTACLES:
            writer.writerow({
                "asset_id": asset_id,
                "asset_name": asset_name,
                "category": "rejected_obstacle_candidate",
                "status": reason,
                "source_time_start": start,
                "source_time_end": end,
                "notes": "Owner confirmed this track was a turnaround, not an obstacle survey.",
            })

    added_stats = {
        key: candidate_areas[asset_id].area - old_areas[asset_id].area
        for key, asset_id in AREA_IDS.items()
    }
    report = {
        "status": "REVIEW_ONLY_NOT_FIELD_READY",
        "source_file": str(SOURCE.relative_to(REPO)).replace("\\", "/"),
        "source_sha256": sha256(SOURCE),
        "revision_2_source": str(REV2.relative_to(REPO)).replace("\\", "/"),
        "revision_2_sha256": sha256(REV2),
        "deck_radius_m": DECK_RADIUS_M,
        "perimeter_evidence": perimeter_records,
        "area_changes_m2": added_stats,
        "obstacles": obstacle_records,
        "rejected_obstacles": [
            {"asset_id": asset_id, "asset_name": name, "start": start, "end": end, "reason": reason}
            for asset_id, name, start, end, reason in REJECTED_OBSTACLES
        ],
        "limitations": [
            "No autonomous mission or launcher was generated.",
            "Generic obstacle identities require owner confirmation.",
            "OBS-NEW-012 is a single-loop candidate and was not subtracted from a mowing area.",
            "OBS-NEW-008 was restored as an owner-confirmed obstacle after replay review of nearly three repeated loops.",
            "OBS-NEW-004, OBS-NEW-006, OBS-NEW-013, and OBS-NEW-014 exclude their straight ingress/egress portions from the circle fit while retaining the full replay evidence.",
            "OBS-NEW-003, OBS-NEW-005, and OBS-NEW-011 use a minimum recorded GPS loop plus a 12-inch outward review buffer.",
            "Exterior boundaries were only expanded; candidate obstacle holes reduce mowable area internally.",
        ],
    }
    REPORT_OUT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    fig, axis = plt.subplots(figsize=(13, 10), dpi=170)
    colors = {AREA_IDS["backyard"]: "#1565c0", AREA_IDS["front"]: "#ef6c00", AREA_IDS["overroad"]: "#795548"}
    for asset_id, geometry in old_areas.items():
        draw_polygon(axis, geometry, "#90a4ae", alpha=0.05, width=1.0)
    for asset_id, geometry in candidate_areas.items():
        draw_polygon(axis, geometry, colors[asset_id], alpha=0.14, width=2.0)
        added = geometry.difference(old_areas[asset_id])
        if not added.is_empty:
            draw_polygon(axis, added, "#43a047", alpha=0.28, width=1.2)
    for line in perimeter_lines:
        x, y = line.xy
        axis.plot(x, y, color="#00897b", linewidth=1.0, alpha=0.8)
    if existing_tree_exclusion is not None:
        draw_polygon(axis, existing_tree_exclusion, "#c62828", alpha=0.38, width=1.2)
        center = existing_tree_exclusion.centroid
        axis.annotate("OBS-TREE-001", (center.x, center.y), xytext=(3, 3), textcoords="offset points", fontsize=7, fontweight="bold")
    for record, _evidence, _centerline, exclusion, _polygon_exclusion, _owner_path in obstacle_geometries:
        color = "#ff8f00" if record["confidence"].startswith("low") else "#c62828"
        draw_polygon(axis, exclusion, color, alpha=0.38, width=1.2)
        x, y = record["center_xy_m"]
        axis.annotate(record["asset_id"], (x, y), xytext=(3, 3), textcoords="offset points", fontsize=7, fontweight="bold")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, linewidth=0.3, alpha=0.35)
    axis.set_title("62 Collins site inventory revision 3 candidate\nGreen = proposed outward addition; red = high-confidence obstacle; orange = low-confidence")
    axis.set_xlabel("Local east (m)")
    axis.set_ylabel("Local north (m)")
    fig.tight_layout()
    fig.savefig(PNG_OUT, bbox_inches="tight")
    plt.close(fig)

    replay = []
    replay_start = rows[0]["time"]
    last_emitted = None
    previous_emitted = None
    for row in rows:
        if last_emitted is not None and row["mode"] == last_emitted["mode"] and (row["time"] - last_emitted["time"]).total_seconds() < 0.20:
            continue
        is_break = previous_emitted is None or row["mode"] != previous_emitted["mode"] or (row["time"] - previous_emitted["time"]).total_seconds() > 3.0
        replay.append([
            round((row["time"] - replay_start).total_seconds(), 3),
            round(row["x"], 3),
            round(row["y"], 3),
            row["mode"],
            round(row["speed"], 3),
            round(row["heading"], 2) if row["heading"] is not None else None,
            is_break,
            row["fix_quality"],
            row["heading_quality"],
            round(row["heading_accuracy"], 3) if row["heading_accuracy"] is not None else None,
            row["carrier"],
        ])
        last_emitted = row
        previous_emitted = row

    payload = {
        "old": {key.split("AREA-")[-1].lower(): mapping(value) for key, value in old_areas.items()},
        "areas": {
            "backyard": mapping(candidate_areas[AREA_IDS["backyard"]]),
            "front": mapping(candidate_areas[AREA_IDS["front"]]),
            "overroad": mapping(candidate_areas[AREA_IDS["overroad"]]),
        },
        "added": {key: mapping(candidate_areas[asset_id].difference(old_areas[asset_id])) for key, asset_id in AREA_IDS.items()},
        "perimeters": [[[round(x, 3), round(y, 3)] for x, y in line.coords] for line in perimeter_lines],
        "obstacles": ([{
            "id": "OBS-TREE-001",
            "center": [round(existing_tree_exclusion.centroid.x, 3), round(existing_tree_exclusion.centroid.y, 3)],
            "centerLatLon": [round(frame.ll(existing_tree_exclusion.centroid.x, existing_tree_exclusion.centroid.y)[0], 9), round(frame.ll(existing_tree_exclusion.centroid.x, existing_tree_exclusion.centroid.y)[1], 9)],
            "circle": mapping(existing_tree_exclusion),
            "centerlineCircle": mapping(existing_tree_exclusion.buffer(DECK_RADIUS_M)),
            "polygon": mapping(existing_tree_exclusion),
            "evidence": [],
            "low": False,
            "confidence": "existing_reviewed_obstacle",
            "model": "circle_fit_minus_half_deck",
            "ownerBuffer": None,
            "circleRadius": round(math.sqrt(existing_tree_exclusion.area / math.pi), 3),
            "centerlineRadius": round(math.sqrt(existing_tree_exclusion.area / math.pi) + DECK_RADIUS_M, 3),
            "recordedMin": None,
            "recordedMax": None,
            "closestDeckEdge": None,
            "polygonArea": round(existing_tree_exclusion.area, 3),
            "activeArea": round(existing_tree_exclusion.area, 3),
            "rms": None,
            "start": "2026-09-23",
            "end": "2026-09-23",
            "replayStart": None,
            "replayEnd": None,
        }] if existing_tree_exclusion is not None else []) + [
            {
                "id": record["asset_id"],
                "center": [round(record["center_xy_m"][0], 3), round(record["center_xy_m"][1], 3)],
                "centerLatLon": [round(frame.ll(record["center_xy_m"][0], record["center_xy_m"][1])[0], 9), round(frame.ll(record["center_xy_m"][0], record["center_xy_m"][1])[1], 9)],
                "circle": mapping(exclusion),
                "centerlineCircle": mapping(_centerline),
                "polygon": mapping(polygon_exclusion),
                "evidence": [[round(x, 3), round(y, 3)] for x, y in evidence.coords],
                "low": record["confidence"].startswith("low"),
                "confidence": record["confidence"],
                "model": record["geometry_method"],
                "ownerBuffer": record["owner_path_buffer_m"],
                "circleRadius": round(record["mowing_exclusion_radius_m"], 3),
                "centerlineRadius": round(record["centerline_radius_m"], 3),
                "recordedMin": round(record["recorded_centerline_min_radius_m"], 3),
                "recordedMax": round(record["recorded_centerline_max_radius_m"], 3),
                "closestDeckEdge": round(record["closest_implied_deck_edge_radius_m"], 3),
                "polygonArea": round(record["polygon_exclusion_area_m2"], 3),
                "activeArea": round(record["active_exclusion_area_m2"], 3),
                "rms": round(record["radial_rms_m"], 3),
                "start": record["start"],
                "end": record["end"],
                "replayStart": round((parse_time(record["start"]) - replay_start).total_seconds(), 3),
                "replayEnd": round((parse_time(record["end"]) - replay_start).total_seconds(), 3),
            }
            for record, evidence, _centerline, exclusion, polygon_exclusion, _owner_path in obstacle_geometries
        ],
        "deckRadius": DECK_RADIUS_M,
        "replay": replay,
        "replayStart": replay_start.isoformat(),
        "stats": {
            "backyard_added": added_stats["backyard"],
            "front_added": added_stats["front"],
            "overroad_added": added_stats["overroad"],
        },
    }
    write_html(payload)
    README_OUT.write_text(
        "# 62 Collins site inventory revision 3 — review only\n\n"
        "This candidate uses the 2026-09-26 phone/legacy-handheld field survey. Revision 2 remains unchanged.\n\n"
        "- The backyard southeast proposal includes the owner-specified straight connector and ordered timestamp vertices.\n"
        "- Complete circuits propose outward-only front-yard and over-road updates.\n"
        "- OBS-NEW-008, OBS-NEW-013, and OBS-NEW-014 capture owner-confirmed obstacles identified during replay review.\n"
        "- OBS-NEW-004, OBS-NEW-006, OBS-NEW-013, and OBS-NEW-014 use their owner-selected circular portions for center fitting while retaining their complete approach and departure tracks as replay evidence.\n"
        "- OBS-NEW-003, OBS-NEW-005, and OBS-NEW-011 use the minimum recorded GPS loop plus a 12-inch outward review buffer.\n"
        "- OBS-NEW-012 is a lower-confidence single loop and remains evidence-only.\n"
        "- The existing telephone pole was resurveyed.\n"
        "- The interactive review overlays the complete moving path and provides 1x-50x replay plus a Replay selected obstacle action.\n"
        "- Every active obstacle displays a dashed fitted-GPS circle for comparison with its recorded loop and active keep-out shape.\n"
        "- `62_Collins_site_inventory_rev003_owner_review_worksheet.csv` lists the areas and obstacles requiring review.\n"
        "- No autonomous mission or launcher was generated.\n\n"
        "Open `62_Collins_site_inventory_rev003_INTERACTIVE_REVIEW.html` for layer-by-layer review.\n",
        encoding="utf-8",
        newline="\n",
    )
    print(GEOJSON_OUT)
    print(PNG_OUT)
    print(HTML_OUT)


if __name__ == "__main__":
    main()
