#!/usr/bin/env python3
"""Build a review-only combined coverage/perimeter package for 2026-09-23.

The completed automatic mission was driven blades-off, so its 42-inch swath is
modeled rather than observed cutting.  The later manual session was a cleanup
cut.  Existing September 15 polygons remain the baseline; manual swath outside
those polygons is admitted conservatively and shown separately for review.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.geometry import LineString, MultiLineString, Point, Polygon, mapping
from shapely.ops import transform, unary_union


REPO = Path(__file__).resolve().parents[2]
SITE = REPO / "field_testing/sites/62_Collins_multi_boundary_20260915"
REVIEW = SITE / "runs/20260915_133923/boundary_review"
AUTO_RUN = SITE / "runs/20260923_124933_master_manual_5hz"
MANUAL_RUN = SITE / "runs/20260923_135328_manual_stripes"
PLAN = SITE / "mission_plans/20260915_master_boundary_replay/generated_master_manual_field_20260920"
OUT = SITE / "analysis/20260923_combined_coverage_review"

PURSUIT = AUTO_RUN / "pursuit_log_20260923_124935.csv"
MANUAL = MANUAL_RUN / "manual_stripes_20260923_135328.csv"
AUDIT = PLAN / "62_Collins_master_manual_resampled_1mps_20260920_audit.csv"
WORKSHEET = OUT / "manual_coverage_labeling_worksheet.csv"

DECK_WIDTH_M = 1.0668
DECK_RADIUS_M = DECK_WIDTH_M / 2.0
LANE_SPACING_M = 0.9652
MANUAL_EXTENSION_CENTER_LIMIT_M = 2.50
MIN_MOVING_SPEED_MPS = 0.12
MAX_SAMPLE_GAP_S = 1.0
MAX_POSITION_JUMP_M = 2.0
SIMPLIFY_M = 0.10

FIELD_SEGMENTS = {
    "main_backyard": (3, 4),
    "garden_right": (6,),
    "garden_left": (8,),
    "front_yard": (10,),
    "over_the_road": (13, 15),
}

COLORS = {
    "main_backyard": "#1565c0",
    "garden_right": "#2e7d32",
    "garden_left": "#7b1fa2",
    "front_yard": "#ef6c00",
    "over_the_road": "#795548",
}


@dataclass(frozen=True)
class Frame:
    lat0: float
    lon0: float

    @property
    def lon_scale(self) -> float:
        return 111_320.0 * math.cos(math.radians(self.lat0))

    def xy(self, lat: float, lon: float) -> tuple[float, float]:
        return ((lon - self.lon0) * self.lon_scale, (lat - self.lat0) * 110_540.0)

    def ll(self, x: float, y: float) -> tuple[float, float]:
        return (self.lat0 + y / 110_540.0, self.lon0 + x / self.lon_scale)


def parse_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "y"}


def parse_time(value: str) -> float:
    return datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp()


def polygons_of(geometry):
    if geometry.is_empty:
        return []
    if geometry.geom_type == "Polygon":
        return [geometry]
    if geometry.geom_type == "MultiPolygon":
        return list(geometry.geoms)
    if geometry.geom_type == "GeometryCollection":
        return [item for item in geometry.geoms if item.geom_type == "Polygon"]
    return []


def largest_polygon(geometry):
    parts = polygons_of(geometry)
    if not parts:
        raise ValueError(f"Expected polygonal geometry, got {geometry.geom_type}")
    return max(parts, key=lambda item: item.area)


def clean_polygon(geometry):
    geometry = geometry.buffer(0)
    if geometry.is_empty:
        return geometry
    geometry = geometry.simplify(SIMPLIFY_M, preserve_topology=True)
    return geometry if geometry.is_valid else geometry.buffer(0)


def read_segment(frame: Frame, number: int) -> list[tuple[float, float]]:
    path = REVIEW / f"segment_{number:02d}_candidate_path.csv"
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return [frame.xy(float(row["lat"]), float(row["lon"])) for row in csv.DictReader(handle)]


def build_baselines(frame: Frame):
    result = {}
    for name, numbers in FIELD_SEGMENTS.items():
        points = [point for number in numbers for point in read_segment(frame, number)]
        result[name] = largest_polygon(Polygon(points).buffer(0))
    return result


def audit_phases() -> dict[int, str]:
    with AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        return {int(row["waypoint"]): row["phase"] for row in csv.DictReader(handle)}


def direct_phase_area(phase: str) -> str | None:
    for name in FIELD_SEGMENTS:
        if phase.startswith(name + "_"):
            return name
    if phase == "over_the_road_24in_clearance_bypass":
        return "over_the_road"
    return None


def nearest_area(point: Point, baselines, maximum_m: float | None = None) -> tuple[str | None, float]:
    distances = {name: point.distance(polygon) for name, polygon in baselines.items()}
    name = min(distances, key=distances.get)
    distance = distances[name]
    if maximum_m is not None and distance > maximum_m:
        return None, distance
    return name, distance


def append_line(bucket: dict[str, list[LineString]], name: str, a, b):
    if a != b:
        bucket[name].append(LineString([a, b]))


def load_automatic_lines(frame: Frame, baselines):
    phases = audit_phases()
    lines = {name: [] for name in FIELD_SEGMENTS}
    trail = []
    prior = None
    with PURSUIT.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        next(reader, None)  # units/description row
        for row in reader:
            try:
                if not parse_bool(row["driving"]):
                    continue
                if row["fix_quality"] != "RTK Fixed" or not parse_bool(row["head_valid"]):
                    continue
                lat, lon = float(row["lat"]), float(row["lon"])
                timestamp = float(row["timestamp"])
                waypoint = int(row["waypoint_idx"]) + 1
            except (KeyError, TypeError, ValueError):
                continue
            point = frame.xy(lat, lon)
            phase = phases.get(waypoint, "")
            area = direct_phase_area(phase)
            if area is None and not phase.startswith("transition_"):
                area, distance = nearest_area(Point(point), baselines, 0.80)
                if distance > 0.80:
                    area = None
            trail.append(point)
            current = (timestamp, point, area)
            if prior is not None:
                dt = timestamp - prior[0]
                distance = math.dist(point, prior[1])
                if area and area == prior[2] and 0 < dt <= MAX_SAMPLE_GAP_S and distance <= MAX_POSITION_JUMP_M:
                    append_line(lines, area, prior[1], point)
            prior = current
    return lines, trail


def load_manual_segments(frame: Frame, baselines):
    segments = []
    trail = []
    prior = None
    with MANUAL.open(newline="", encoding="utf-8-sig") as handle:
        for row_number, row in enumerate(csv.DictReader(handle), 2):
            try:
                valid = (
                    row["trans_mode"] == "1"
                    and float(row["speed_mps"] or 0.0) >= MIN_MOVING_SPEED_MPS
                    and row["fix_quality"] == "RTK Fixed"
                    and parse_bool(row["head_valid"])
                )
                if not valid:
                    prior = None
                    continue
                timestamp = parse_time(row["time"])
                elapsed = float(row["elapsed_sec"])
                point = frame.xy(float(row["lat"]), float(row["lon"]))
            except (KeyError, TypeError, ValueError):
                prior = None
                continue
            if not trail or math.dist(trail[-1], point) >= 0.10:
                trail.append(point)
            current = {
                "row": row_number,
                "time": row["time"],
                "timestamp": timestamp,
                "elapsed": elapsed,
                "point": point,
            }
            if prior is not None:
                dt = timestamp - prior["timestamp"]
                length = math.dist(point, prior["point"])
                if 0 < dt <= MAX_SAMPLE_GAP_S and length < 0.10:
                    continue
                if 0 < dt <= MAX_SAMPLE_GAP_S and 0 < length <= MAX_POSITION_JUMP_M:
                    midpoint = Point(
                        (point[0] + prior["point"][0]) / 2.0,
                        (point[1] + prior["point"][1]) / 2.0,
                    )
                    area, outside_m = nearest_area(midpoint, baselines)
                    inside = baselines[area].covers(midpoint)
                    if inside:
                        classification = "interior_cut"
                        suggested = "yes"
                    elif outside_m <= MANUAL_EXTENSION_CENTER_LIMIT_M:
                        classification = "perimeter_extension_candidate"
                        suggested = "yes"
                    else:
                        classification = "transition_or_review"
                        suggested = "no"
                    segments.append({
                        "start_row": prior["row"],
                        "end_row": row_number,
                        "start_time": prior["time"],
                        "end_time": row["time"],
                        "start_elapsed_s": prior["elapsed"],
                        "end_elapsed_s": elapsed,
                        "area": area,
                        "classification": classification,
                        "suggested_include": suggested,
                        "outside_baseline_m": outside_m,
                        "length_m": length,
                        "line": LineString([prior["point"], point]),
                    })
            prior = current
    return segments, trail


def stabilize_manual_labels(segments, radius: int = 10):
    """Suppress sub-meter label chatter along a continuous manual path."""
    if not segments:
        return segments
    keys = [
        (item["area"], item["classification"], item["suggested_include"])
        for item in segments
    ]
    stable = []
    for index, segment in enumerate(segments):
        weights = {}
        for neighbor in range(max(0, index - radius), min(len(segments), index + radius + 1)):
            # Never smooth across a logger/time discontinuity.
            if abs(segments[neighbor]["start_elapsed_s"] - segment["start_elapsed_s"]) > 4.0:
                continue
            key = keys[neighbor]
            weights[key] = weights.get(key, 0.0) + max(segments[neighbor]["length_m"], 0.01)
        winner = max(weights, key=weights.get)
        item = dict(segment)
        item["area"], item["classification"], item["suggested_include"] = winner
        stable.append(item)
    return stable


def group_manual_segments(segments):
    groups = []
    for segment in segments:
        key = (segment["area"], segment["classification"], segment["suggested_include"])
        if (
            groups
            and groups[-1]["key"] == key
            and segment["start_elapsed_s"] - groups[-1]["end_elapsed_s"] <= MAX_SAMPLE_GAP_S
            and Point(segment["line"].coords[0]).distance(
                Point(groups[-1]["segments"][-1]["line"].coords[-1])
            ) <= 0.25
        ):
            group = groups[-1]
            group["end_row"] = segment["end_row"]
            group["end_time"] = segment["end_time"]
            group["end_elapsed_s"] = segment["end_elapsed_s"]
            group["length_m"] += segment["length_m"]
            group["max_outside_baseline_m"] = max(
                group["max_outside_baseline_m"], segment["outside_baseline_m"]
            )
            group["segments"].append(segment)
        else:
            groups.append({
                "key": key,
                "start_row": segment["start_row"],
                "end_row": segment["end_row"],
                "start_time": segment["start_time"],
                "end_time": segment["end_time"],
                "start_elapsed_s": segment["start_elapsed_s"],
                "end_elapsed_s": segment["end_elapsed_s"],
                "area": segment["area"],
                "classification": segment["classification"],
                "suggested_include": segment["suggested_include"],
                "length_m": segment["length_m"],
                "max_outside_baseline_m": segment["outside_baseline_m"],
                "segments": [segment],
            })
    for index, group in enumerate(groups, 1):
        group["candidate"] = index
    return groups


def existing_decisions():
    if not WORKSHEET.exists():
        return {}
    with WORKSHEET.open(newline="", encoding="utf-8-sig") as handle:
        return {int(row["candidate"]): row for row in csv.DictReader(handle)}


def matching_decision(decisions, group):
    decision = decisions.get(group["candidate"])
    if not decision:
        return {}
    try:
        same_source = (
            int(decision["source_start_row"]) == group["start_row"]
            and int(decision["source_end_row"]) == group["end_row"]
        )
    except (KeyError, TypeError, ValueError):
        return {}
    return decision if same_source else {}


def write_worksheet(groups):
    decisions = existing_decisions()
    fields = [
        "candidate", "include", "label", "area", "classification", "start_time", "end_time",
        "start_elapsed_s", "end_elapsed_s", "duration_s", "path_length_m",
        "max_center_outside_baseline_m", "source_start_row", "source_end_row", "notes",
    ]
    with WORKSHEET.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for group in groups:
            old = matching_decision(decisions, group)
            writer.writerow({
                "candidate": group["candidate"],
                "include": old.get("include", group["suggested_include"]),
                "label": old.get("label", group["classification"]),
                "area": old.get("area", group["area"]),
                "classification": group["classification"],
                "start_time": group["start_time"],
                "end_time": group["end_time"],
                "start_elapsed_s": f"{group['start_elapsed_s']:.2f}",
                "end_elapsed_s": f"{group['end_elapsed_s']:.2f}",
                "duration_s": f"{group['end_elapsed_s'] - group['start_elapsed_s']:.2f}",
                "path_length_m": f"{group['length_m']:.2f}",
                "max_center_outside_baseline_m": f"{group['max_outside_baseline_m']:.2f}",
                "source_start_row": group["start_row"],
                "source_end_row": group["end_row"],
                "notes": old.get("notes", ""),
            })


def apply_manual_decisions(groups):
    decisions = existing_decisions()
    accepted = {name: [] for name in FIELD_SEGMENTS}
    rejected = []
    for group in groups:
        decision = matching_decision(decisions, group)
        include = decision.get("include", group["suggested_include"]).strip().lower() in {"yes", "y", "true", "1"}
        area = decision.get("area", group["area"]).strip()
        lines = [segment["line"] for segment in group["segments"]]
        if include and area in accepted:
            accepted[area].extend(lines)
        else:
            rejected.extend(lines)
    return accepted, rejected


def swath(lines):
    if not lines:
        return Polygon()
    geometry = unary_union(lines)
    return clean_polygon(geometry.buffer(DECK_RADIUS_M, cap_style="round", join_style="round"))


def to_lonlat(frame: Frame, geometry):
    def convert(x, y, z=None):
        lat, lon = frame.ll(x, y)
        return lon, lat
    converted = transform(convert, geometry)
    return converted if converted.is_valid else converted.buffer(0)


def line_coords(line):
    return [[round(x, 3), round(y, 3)] for x, y in line.coords]


def geometry_payload(geometry):
    return mapping(geometry) if not geometry.is_empty else None


def write_html(payload):
    html = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>September 23 combined coverage review</title>
<style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#10151c;color:#e7edf5;font:14px system-ui,sans-serif}
header{padding:12px 16px;border-bottom:1px solid #34404e;background:#17202a}h1{font-size:18px;margin:0 0 5px}.warn{color:#ffcc80}
#layout{display:grid;grid-template-columns:310px 1fr;height:calc(100vh - 70px)}aside{padding:14px;overflow:auto;border-right:1px solid #34404e;background:#141b24}
label{display:block;margin:8px 0}.area{display:flex;align-items:center;gap:7px}.dot{width:11px;height:11px;border-radius:50%}
button{background:#263545;color:#fff;border:1px solid #506274;border-radius:5px;padding:7px 10px;margin:4px 3px 4px 0}canvas{width:100%;height:100%;display:block;background:#f8fafc}
.small{font-size:12px;color:#aebdca;line-height:1.45}.metric{display:grid;grid-template-columns:1fr auto;gap:4px 10px;margin-top:10px}.metric span:nth-child(even){font-variant-numeric:tabular-nums}
</style></head><body><header><h1>September 23, 2026 · combined coverage and proposed perimeter</h1><div class="warn">REVIEW ONLY — automatic mission swath is modeled; manual classifications require confirmation.</div></header>
<div id="layout"><aside>
<b>Layers</b>
<label><input id="baseline" type="checkbox" checked> September 15 baseline</label>
<label><input id="candidate" type="checkbox" checked> Proposed full area</label>
<label><input id="observed" type="checkbox" checked> Combined modeled swath</label>
<label><input id="gaps" type="checkbox" checked> Remaining modeled gaps</label>
<label><input id="autoTrail" type="checkbox"> Automatic actual trail</label>
<label><input id="manualTrail" type="checkbox" checked> Manual actual trail</label>
<label><input id="rejected" type="checkbox" checked> Excluded/manual review trail</label>
<p><b>Areas</b></p><div id="areas"></div>
<p><button id="fit">Fit all</button><button id="reset">Reset layers</button></p>
<div id="metrics"></div>
<p class="small">The proposed full area is the September 15 baseline plus included manual deck swath. Interior manual cuts change the modeled gap layer but only exterior-connected swath can change the perimeter. Edit the CSV worksheet and rebuild to change classifications.</p>
</aside><canvas id="map"></canvas></div>
<script>
const D=__PAYLOAD__; const $=id=>document.getElementById(id),C=$('map'),X=C.getContext('2d'); let scale=8,ox=0,oy=0,drag=null;
const enabled={}; Object.keys(D.colors).forEach(k=>enabled[k]=true);
const areas=document.getElementById('areas'); Object.entries(D.colors).forEach(([k,c])=>{let l=document.createElement('label');l.className='area';l.innerHTML=`<input type=checkbox checked data-area="${k}"><span class=dot style="background:${c}"></span>${k.replaceAll('_',' ')}`;areas.appendChild(l)});
areas.onchange=e=>{if(e.target.dataset.area){enabled[e.target.dataset.area]=e.target.checked;draw()}};
function size(){let r=C.getBoundingClientRect(),d=devicePixelRatio||1;C.width=r.width*d;C.height=r.height*d;X.setTransform(d,0,0,d,0,0);draw()}
function coords(g,out=[]){if(!g)return out;if(g.type==='Polygon')g.coordinates.forEach(r=>out.push(...r));else if(g.type==='MultiPolygon')g.coordinates.forEach(p=>p.forEach(r=>out.push(...r)));return out}
function pathRing(r){X.beginPath();r.forEach((p,i)=>{let q=screen(p);i?X.lineTo(...q):X.moveTo(...q)});X.closePath()}
function fillGeom(g,fill,stroke,w=1){if(!g)return;let ps=g.type==='Polygon'?[g.coordinates]:g.coordinates;X.save();X.fillStyle=fill;X.strokeStyle=stroke;X.lineWidth=w;ps.forEach(poly=>{X.beginPath();poly.forEach(r=>r.forEach((p,i)=>{let q=screen(p);i?X.lineTo(...q):X.moveTo(...q)}));X.fill('evenodd');X.stroke()});X.restore()}
function line(points,color,w=1,a=1){if(!points||points.length<2)return;X.save();X.globalAlpha=a;X.strokeStyle=color;X.lineWidth=w;X.beginPath();points.forEach((p,i)=>{let q=screen(p);i?X.lineTo(...q):X.moveTo(...q)});X.stroke();X.restore()}
function screen(p){return [ox+p[0]*scale,oy-p[1]*scale]}
function draw(){let r=C.getBoundingClientRect();X.clearRect(0,0,r.width,r.height);X.fillStyle='#f8fafc';X.fillRect(0,0,r.width,r.height);
 for(const [k,a] of Object.entries(D.areas)){if(!enabled[k])continue;let c=D.colors[k];if($('observed').checked)fillGeom(a.combined,'#90caf955',c+'88',.7);if($('candidate').checked)fillGeom(a.proposed,c+'25',c,2);if($('gaps').checked)fillGeom(a.gaps,'#ef535588','#c62828',.7);if($('baseline').checked)fillGeom(a.baseline,'transparent',c+'bb',1);}
 if($('autoTrail').checked)line(D.autoTrail,'#263238',.8,.55);if($('manualTrail').checked)line(D.manualTrail,'#ff1744',1.1,.8);if($('rejected').checked)D.rejected.forEach(p=>line(p,'#ff9800',2,.9));}
function fitView(){let pts=[];Object.values(D.areas).forEach(a=>pts.push(...coords(a.proposed)));let xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]),r=C.getBoundingClientRect(),pad=35;scale=Math.min((r.width-2*pad)/(Math.max(...xs)-Math.min(...xs)),(r.height-2*pad)/(Math.max(...ys)-Math.min(...ys)));ox=pad-Math.min(...xs)*scale;oy=pad+Math.max(...ys)*scale;draw()}
C.onwheel=e=>{e.preventDefault();let r=C.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,s=e.deltaY<0?1.15:1/1.15;ox=mx-(mx-ox)*s;oy=my-(my-oy)*s;scale*=s;draw()};C.onpointerdown=e=>drag=[e.clientX,e.clientY,ox,oy];C.onpointermove=e=>{if(drag){ox=drag[2]+e.clientX-drag[0];oy=drag[3]+e.clientY-drag[1];draw()}};C.onpointerup=()=>drag=null;C.onpointerleave=()=>drag=null;
document.querySelectorAll('aside input:not([data-area])').forEach(i=>i.onchange=draw);$('fit').onclick=fitView;$('reset').onclick=()=>{$('baseline').checked=$('candidate').checked=$('observed').checked=$('gaps').checked=$('manualTrail').checked=$('rejected').checked=true;$('autoTrail').checked=false;draw()};
$('metrics').innerHTML='<p><b>Area summary</b></p>'+Object.entries(D.report).map(([k,v])=>`<div class=metric><span>${k.replaceAll('_',' ')}</span><span>+${v.added_m2.toFixed(1)} m² · gaps ${v.gaps_m2.toFixed(1)} m²</span></div>`).join('');window.onresize=size;size();setTimeout(fitView,0);
</script></body></html>"""
    (OUT / "combined_coverage_review.html").write_text(
        html.replace("__PAYLOAD__", json.dumps(payload, separators=(",", ":"))),
        encoding="utf-8",
        newline="\n",
    )


def write_preview(baselines, proposed, gaps, manual_trail, rejected_lines):
    figure, axis = plt.subplots(figsize=(12, 10), dpi=170)
    for name, baseline in baselines.items():
        color = COLORS[name]
        first_part = True
        for polygon in polygons_of(proposed[name]):
            x, y = polygon.exterior.xy
            axis.fill(x, y, color=color, alpha=0.16)
            axis.plot(
                x,
                y,
                color=color,
                linewidth=2.0,
                label=name.replace("_", " ") if first_part else None,
            )
            first_part = False
        bx, by = baseline.exterior.xy
        axis.plot(bx, by, color=color, linewidth=1.0, linestyle="--", alpha=0.8)
        for polygon in polygons_of(gaps[name]):
            gx, gy = polygon.exterior.xy
            axis.fill(gx, gy, color="#ef5350", alpha=0.55)
    if manual_trail:
        mx, my = zip(*manual_trail)
        axis.plot(mx, my, color="#ff1744", linewidth=0.7, alpha=0.65, label="manual trail")
    for index, line in enumerate(rejected_lines):
        rx, ry = line.xy
        axis.plot(
            rx,
            ry,
            color="#ff9800",
            linewidth=1.6,
            alpha=0.9,
            label="excluded/review" if index == 0 else None,
        )
    axis.plot([], [], color="#c62828", linewidth=7, alpha=0.55, label="modeled coverage gap")
    axis.plot([], [], color="#455a64", linewidth=1.0, linestyle="--", label="September 15 baseline")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.22)
    axis.set_xlabel("east (m)")
    axis.set_ylabel("north (m)")
    axis.set_title("September 23 combined coverage — proposed perimeter review")
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axis.legend(unique.values(), unique.keys(), loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(OUT / "combined_coverage_review.png")
    plt.close(figure)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    with (REVIEW / "segment_02_candidate_path.csv").open(newline="", encoding="utf-8-sig") as handle:
        first = next(csv.DictReader(handle))
    frame = Frame(float(first["lat"]), float(first["lon"]))
    baselines = build_baselines(frame)
    automatic_lines, automatic_trail = load_automatic_lines(frame, baselines)
    manual_segments, manual_trail = load_manual_segments(frame, baselines)
    manual_segments = stabilize_manual_labels(manual_segments)
    groups = group_manual_segments(manual_segments)
    write_worksheet(groups)
    manual_lines, rejected_lines = apply_manual_decisions(groups)

    features = []
    areas_payload = {}
    report_fields = {}
    proposed_geometries = {}
    gap_geometries = {}
    for name, baseline in baselines.items():
        auto_swath = swath(automatic_lines[name])
        manual_swath = swath(manual_lines[name])
        combined = clean_polygon(unary_union([auto_swath, manual_swath]))
        proposed = unary_union([baseline, manual_swath]).buffer(0)
        gaps = clean_polygon(proposed.difference(combined))
        proposed_geometries[name] = proposed
        gap_geometries[name] = gaps
        added = proposed.difference(baseline)
        report_fields[name] = {
            "baseline_area_m2": baseline.area,
            "automatic_modeled_swath_m2": auto_swath.area,
            "included_manual_swath_m2": manual_swath.area,
            "combined_modeled_swath_m2": combined.area,
            "proposed_full_area_m2": proposed.area,
            "added_m2": added.area,
            "gaps_m2": gaps.area,
            "proposed_components": len(polygons_of(proposed)),
            "proposed_valid": proposed.is_valid,
        }
        areas_payload[name] = {
            "baseline": geometry_payload(baseline),
            "combined": geometry_payload(combined),
            "proposed": geometry_payload(proposed),
            "gaps": geometry_payload(gaps),
        }
        for kind, geometry in (("baseline", baseline), ("combined_modeled_swath", combined), ("proposed_full_area", proposed), ("modeled_gaps", gaps)):
            if geometry.is_empty:
                continue
            features.append({
                "type": "Feature",
                "properties": {"area": name, "kind": kind, "review_only": True},
                "geometry": mapping(to_lonlat(frame, geometry)),
            })

    geojson = {"type": "FeatureCollection", "features": features}
    (OUT / "combined_coverage_boundaries_REVIEW_ONLY.geojson").write_text(
        json.dumps(geojson, indent=2) + "\n", encoding="utf-8"
    )
    payload = {
        "colors": COLORS,
        "areas": areas_payload,
        "autoTrail": [[round(x, 3), round(y, 3)] for x, y in automatic_trail],
        "manualTrail": [[round(x, 3), round(y, 3)] for x, y in manual_trail],
        "rejected": [line_coords(line) for line in rejected_lines],
        "report": {name: {"added_m2": item["added_m2"], "gaps_m2": item["gaps_m2"]} for name, item in report_fields.items()},
    }
    write_html(payload)
    write_preview(baselines, proposed_geometries, gap_geometries, manual_trail, rejected_lines)
    report = {
        "status": "REVIEW_ONLY_NOT_FIELD_READY",
        "deck_width_m": DECK_WIDTH_M,
        "deck_model": "symmetric round buffer about GNSS centerline; fore/aft GNSS-to-deck offset not yet measured",
        "lane_spacing_m": LANE_SPACING_M,
        "manual_extension_center_limit_m": MANUAL_EXTENSION_CENTER_LIMIT_M,
        "manual_candidate_groups": len(groups),
        "manual_included_groups": sum(1 for row in existing_decisions().values() if row.get("include", "").lower() == "yes"),
        "sources": {
            str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (PURSUIT, MANUAL, AUDIT, REVIEW / "segment_labeling_worksheet.csv")
        },
        "fields": report_fields,
        "limitations": [
            "The automatic mission was driven with the deck disengaged; its swath is modeled, not observed cutting.",
            "The deck model is symmetric around the GNSS track and is least accurate in tight corners.",
            "Automatically suggested manual classifications must be reviewed in the worksheet.",
            "No mission or launcher is generated by this tool.",
        ],
    }
    (OUT / "combined_coverage_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (OUT / "README.md").write_text(
        "# September 23 combined coverage review\n\n"
        "Open `combined_coverage_review.html` in a modern browser. This package is review-only and does not contain a launchable mission.\n\n"
        "Review and edit `manual_coverage_labeling_worksheet.csv`, especially rows labeled `perimeter_extension_candidate` or `transition_or_review`, then rerun:\n\n"
        "```powershell\npython field_testing/tools/build_20260923_combined_coverage_review.py\n```\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "html": str((OUT / "combined_coverage_review.html").relative_to(REPO)),
        "worksheet": str(WORKSHEET.relative_to(REPO)),
        "manual_groups": len(groups),
        "fields": report_fields,
    }, indent=2))


if __name__ == "__main__":
    main()
