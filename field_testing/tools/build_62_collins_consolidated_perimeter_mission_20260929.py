#!/usr/bin/env python3
"""Build the approved 2026-09-29 consolidated 62 Collins field-test package.

The consolidated route is authoritative.  The only material geometry change made
here is a local clearance correction around OBS-NEW-011, where the 42-inch deck
envelope otherwise overlaps the revision-3 candidate mowing exclusion.
"""

from __future__ import annotations

import bisect
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, shape
from shapely.ops import nearest_points, transform
from shapely.strtree import STRtree


REPO = Path(__file__).resolve().parents[2]
SITE = REPO / "field_testing/sites/62_Collins_multi_boundary_20260915"
SOURCE_PACKAGE = SITE / "mission_plans/20260928_outer_perimeter_field_test"
SOURCE_ROUTE = SOURCE_PACKAGE / (
    "submitted_edits_review_REVIEW_ONLY/"
    "62_Collins_consolidated_route_REVIEW_ONLY.json"
)
SOURCE_REPORT = SOURCE_PACKAGE / (
    "submitted_edits_review_REVIEW_ONLY/"
    "62_Collins_submitted_mission_review_report.json"
)
SOURCE_MARKUP = SOURCE_PACKAGE / (
    "submitted_edits_review_REVIEW_ONLY/"
    "62_Collins_submitted_markup_SOURCE.json"
)
SOURCE_BUILDER = REPO / "field_testing/tools/build_62_collins_submitted_mission_review_20260928.py"
INVENTORY = SITE / (
    "site_inventory/revisions/rev_003_20260926_REVIEW_ONLY/"
    "62_Collins_site_inventory_rev003_REVIEW_ONLY.geojson"
)
PRIOR_RUN = SITE / "runs/20260928_outer_perimeter_field_test/outer_perimeter_20260928_120838.csv"
OUTDIR = SITE / "mission_plans/20260929_consolidated_perimeter_field_test"
GENERATED = OUTDIR / "generated"
STEM = "62_Collins_consolidated_perimeter_1mps_20260929"
MISSION = GENERATED / f"{STEM}.txt"
AUDIT = GENERATED / f"{STEM}_audit.csv"
REPORT = GENERATED / f"{STEM}_validation.json"
PREVIEW = GENERATED / f"{STEM}_full_route_APPROVED.png"
REPLAY = GENERATED / f"{STEM}_INTERACTIVE_APPROVED.html"
VERIFY = OUTDIR / "verify_consolidated_perimeter_20260929.py"
LAUNCHER = OUTDIR / "run_62_Collins_consolidated_perimeter_20260929.sh"
README = OUTDIR / "FIELD_TEST_README.md"
DASHBOARD = OUTDIR / "mission_dashboard_consolidated_perimeter_20260929.py"
STATUS = "SUPERVISED_BLADES_OFF_FIELD_TEST_APPROVED_20260929"
SPACING_M = 0.20
SPEED_MPS = 1.00
DECK_WIDTH_M = 1.0668
HALF_DECK_M = DECK_WIDTH_M / 2.0
TURN_REVIEW_RADIUS_M = 1.63
OBS11_TARGET_CENTERLINE_CLEARANCE_M = 0.90


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def distance(points) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def resample(points, spacing=SPACING_M):
    line = LineString(points)
    count = max(1, math.ceil(line.length / spacing))
    return [line.interpolate(line.length * index / count).coords[0] for index in range(count + 1)]


def local_transform(origin_lat, origin_lon):
    east_scale = 111_320.0 * math.cos(math.radians(origin_lat))
    north_scale = 110_540.0

    def convert(x, y, z=None):
        return ((x - origin_lon) * east_scale, (y - origin_lat) * north_scale)

    return convert


def collect_points(geometry):
    if geometry.geom_type == "Point":
        return [geometry]
    if hasattr(geometry, "geoms"):
        result = []
        for item in geometry.geoms:
            result.extend(collect_points(item))
        return result
    return []


def ring_arc(ring, first, last, direction, step=0.04):
    length = ring.length
    travel = (last - first) % length if direction > 0 else (first - last) % length
    count = max(2, math.ceil(travel / step))
    return [
        ring.interpolate((first + direction * travel * index / count) % length).coords[0]
        for index in range(count + 1)
    ]


def correct_obs11_clearance(points, entries, obstacle):
    """Replace only the unsafe P6 subpath with the closest obstacle-buffer arc."""
    source_line = LineString(points)
    measures = [0.0]
    for before, after in zip(points, points[1:]):
        measures.append(measures[-1] + math.dist(before, after))
    safe_boundary = obstacle.buffer(OBS11_TARGET_CENTERLINE_CLEARANCE_M, resolution=32).exterior
    safe_polygon = obstacle.buffer(OBS11_TARGET_CENTERLINE_CLEARANCE_M, resolution=32)
    p6_indices = [index for index, entry in enumerate(entries) if entry["segment_id"] == "P6"]
    if not p6_indices:
        raise ValueError("authoritative route does not contain submitted segment P6")
    first_index, last_index = min(p6_indices), max(p6_indices)
    p6_line = LineString(points[first_index : last_index + 1])
    crossings = collect_points(p6_line.intersection(safe_boundary))
    if len(crossings) < 2:
        raise ValueError("expected two OBS-NEW-011 safety-boundary crossings on P6")
    crossings.sort(key=p6_line.project)
    entry_point, exit_point = crossings[0], crossings[-1]
    entry_distance = source_line.project(entry_point)
    exit_distance = source_line.project(exit_point)
    if exit_distance <= entry_distance:
        raise ValueError("OBS-NEW-011 correction crossings are out of order")

    ring = LineString(safe_boundary.coords)
    ring_start = ring.project(nearest_points(ring, entry_point)[0])
    ring_end = ring.project(nearest_points(ring, exit_point)[0])
    original_subpath = LineString(
        [source_line.interpolate(entry_distance).coords[0]]
        + [
            point
            for index, point in enumerate(points)
            if entry_distance < measures[index] < exit_distance
        ]
        + [source_line.interpolate(exit_distance).coords[0]]
    )
    arcs = [
        ring_arc(ring, ring_start, ring_end, 1),
        ring_arc(ring, ring_start, ring_end, -1),
    ]
    arc = min(arcs, key=lambda candidate: LineString(candidate).hausdorff_distance(original_subpath))

    prefix = [point for index, point in enumerate(points) if measures[index] < entry_distance]
    suffix = [point for index, point in enumerate(points) if measures[index] > exit_distance]
    arc_points = resample(arc)
    if prefix and math.dist(prefix[-1], arc_points[0]) < 0.10:
        prefix.pop()
    if suffix and math.dist(arc_points[-1], suffix[0]) < 0.10:
        suffix.pop(0)
    candidate = prefix + arc_points + suffix
    # Preserve the authoritative samples outside G1.  Split only the handful
    # of source gaps that round just above 0.201 m.
    normalized = [candidate[0]]
    for point in candidate[1:]:
        gap = math.dist(normalized[-1], point)
        pieces = max(1, math.ceil(gap / 0.2011))
        start = normalized[-1]
        normalized.extend(
            (
                start[0] + (point[0] - start[0]) * index / pieces,
                start[1] + (point[1] - start[1]) * index / pieces,
            )
            for index in range(1, pieces + 1)
        )
    candidate = normalized
    candidate_line = LineString(candidate)
    actual_clearance = candidate_line.distance(obstacle)
    if actual_clearance + 1e-6 < HALF_DECK_M:
        raise ValueError("OBS-NEW-011 correction still intersects the deck envelope")
    candidate_entry_distance = candidate_line.project(Point(arc_points[0]))
    candidate_exit_distance = candidate_line.project(Point(arc_points[-1]))
    return candidate, {
        "change_id": "G1",
        "reason": "42-inch deck envelope overlapped OBS-NEW-011 candidate mowing exclusion",
        "source_segment": "P6",
        "source_distance_start_m": round(entry_distance, 3),
        "source_distance_end_m": round(exit_distance, 3),
        "candidate_distance_start_m": round(candidate_entry_distance, 3),
        "candidate_distance_end_m": round(candidate_exit_distance, 3),
        "target_centerline_clearance_m": OBS11_TARGET_CENTERLINE_CLEARANCE_M,
        "result_centerline_clearance_m": round(actual_clearance, 3),
        "result_deck_edge_clearance_m": round(actual_clearance - HALF_DECK_M, 3),
        "owner_review_required": False,
    }


def circumradius(a, b, c):
    ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
    cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    if cross < 1e-9:
        return math.inf
    return ab * bc * ca / (2.0 * cross)


def heading_change_degrees(a, b, c):
    before = math.atan2(b[1] - a[1], b[0] - a[0])
    after = math.atan2(c[1] - b[1], c[0] - b[0])
    return abs(math.degrees((after - before + math.pi) % (2.0 * math.pi) - math.pi))


def remove_micro_backtracks(points, entries):
    """Remove centimeter-scale join reversals without changing route intent."""
    points = list(points)
    entries = list(entries)
    changes = []
    while True:
        found = False
        for index in range(1, len(points) - 1):
            turn = heading_change_degrees(points[index - 1], points[index], points[index + 1])
            inbound = math.dist(points[index - 1], points[index])
            outbound = math.dist(points[index], points[index + 1])
            if turn <= 150.0 or min(inbound, outbound) >= 0.10:
                continue
            removed = entries[index]
            changes.append(
                {
                    "change_id": f"G{len(changes) + 2}",
                    "reason": "remove centimeter-scale near-180-degree join backtrack",
                    "source_sequence": removed["sequence"],
                    "source_distance_m": removed["distance_m"],
                    "source_segment": removed["segment_id"],
                    "removed_leg_m": round(min(inbound, outbound), 3),
                    "heading_change_before_deg": round(turn, 2),
                    "owner_review_required": False,
                }
            )
            del points[index]
            del entries[index]
            found = True
            break
        if not found:
            break

    join_occurrences = []
    while True:
        found = False
        for index in range(len(points) - 1):
            gap = math.dist(points[index], points[index + 1])
            if gap >= 0.10 or entries[index]["segment_id"] == entries[index + 1]["segment_id"]:
                continue
            # Retain the endpoint that produces the gentler connection.
            remove_index = index + 1
            if 0 < index and index + 2 < len(points):
                remove_first_turn = heading_change_degrees(points[index - 1], points[index + 1], points[index + 2])
                remove_second_turn = heading_change_degrees(points[index - 1], points[index], points[index + 2])
                if remove_first_turn < remove_second_turn:
                    remove_index = index
            removed = entries[remove_index]
            join_occurrences.append(
                {
                    "source_sequence": removed["sequence"],
                    "source_distance_m": removed["distance_m"],
                    "source_segment": removed["segment_id"],
                    "join_gap_m": round(gap, 3),
                }
            )
            del points[remove_index]
            del entries[remove_index]
            found = True
            break
        if not found:
            break
    if join_occurrences:
        changes.append(
            {
                "change_id": f"G{len(changes) + 2}",
                "reason": "remove sub-0.10 m duplicate points at segment joins",
                "occurrences": join_occurrences,
                "owner_review_required": False,
            }
        )
    return points, entries, changes


def clustered_events(events, separation=6):
    result = []
    for event in events:
        if result and event[0] - result[-1][0] <= separation:
            if event[1] > result[-1][1]:
                result[-1] = event
        else:
            result.append(event)
    return result


def chaikin(points, iterations=4):
    result = list(points)
    for _ in range(iterations):
        refined = [result[0]]
        for first, second in zip(result, result[1:]):
            refined.extend(
                [
                    (0.75 * first[0] + 0.25 * second[0], 0.75 * first[1] + 0.25 * second[1]),
                    (0.25 * first[0] + 0.75 * second[0], 0.25 * first[1] + 0.75 * second[1]),
                ]
            )
        refined.append(result[-1])
        result = refined
    return result


def smooth_implausible_turns(points):
    """Locally soften only instantaneous heading changes over 40 degrees."""
    original = list(points)
    original_line = LineString(original)
    events = []
    for index in range(1, len(original) - 1):
        turn = heading_change_degrees(original[index - 1], original[index], original[index + 1])
        if turn > 40.0:
            events.append(
                {
                    "_index": index,
                    "route_distance_m": round(original_line.project(Point(original[index])), 3),
                    "east_m": round(original[index][0], 3),
                    "north_m": round(original[index][1], 3),
                    "heading_change_before_deg": round(turn, 2),
                }
            )
    result = list(original)
    for event in reversed(events):
        index = event["_index"]
        first = max(0, index - 5)
        last = min(len(result) - 1, index + 5)
        start, control, end = result[first], result[index], result[last]
        approximate_length = distance(result[first : last + 1])
        count = max(2, math.ceil(approximate_length / SPACING_M))
        local = []
        for sample in range(count + 1):
            t = sample / count
            u = 1.0 - t
            local.append(
                (
                    u * u * start[0] + 2.0 * u * t * control[0] + t * t * end[0],
                    u * u * start[1] + 2.0 * u * t * control[1] + t * t * end[1],
                )
            )
        result[first : last + 1] = local
    if not events:
        return result, None
    smoothed_line = LineString(result)
    return result, {
        "change_id": "G5",
        "reason": "locally soften instantaneous heading changes over 40 degrees",
        "events": [{key: value for key, value in event.items() if key != "_index"} for event in events],
        "maximum_centerline_shift_m": round(
            max(Point(point).distance(original_line) for point in result), 3
        ),
        "owner_review_required": False,
    }


def intersection_audit(points):
    segments = [LineString([a, b]) for a, b in zip(points, points[1:])]
    tree = STRtree(segments)
    crossing_count = 0
    overlap_count = 0
    examples = []
    for first, segment in enumerate(segments):
        for second_value in tree.query(segment):
            second = int(second_value)
            if second <= first + 1:
                continue
            intersection = segment.intersection(segments[second])
            if intersection.is_empty:
                continue
            if intersection.geom_type in ("LineString", "MultiLineString"):
                overlap_count += 1
                kind = "overlap"
            else:
                crossing_count += 1
                kind = "crossing_or_touch"
            if len(examples) < 20:
                examples.append({"segment_a": first + 1, "segment_b": second + 1, "kind": kind})
    return {
        "is_simple": LineString(points).is_simple,
        "nonadjacent_crossings_or_touches": crossing_count,
        "nonadjacent_overlaps": overlap_count,
        "examples": examples,
        "classification": (
            "expected route reuse and out/back access-path retracing; review replay shows the sequence"
        ),
    }


def load_turn_markers():
    import importlib.util

    spec = importlib.util.spec_from_file_location("submitted_review_builder", SOURCE_BUILDER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    payload = module.build_payload(SOURCE_MARKUP)
    result = []
    for segment in payload["segments"]:
        for warning in segment.get("warnings", []):
            result.append(
                {
                    "segment_id": segment["id"],
                    "segment_label": segment["label"],
                    "east_m": warning["point"][0],
                    "north_m": warning["point"][1],
                    "source_review_radius_m": warning["radius"],
                }
            )
    if len(result) != 13:
        raise ValueError(f"expected 13 turn-review markers, found {len(result)}")
    return result


def demonstrated_turns(inventory, convert):
    result = []
    for asset in ("OBS-NEW-004", "OBS-NEW-006", "OBS-NEW-013"):
        feature = next(
            item
            for item in inventory["features"]
            if item.get("properties", {}).get("asset_id") == asset
            and item.get("properties", {}).get("geometry_role") == "recorded_centerline_evidence"
        )
        points = list(transform(convert, shape(feature["geometry"])).coords)
        radii = [
            circumradius(points[index - 4], points[index], points[index + 4])
            for index in range(4, len(points) - 4)
        ]
        finite = sorted(value for value in radii if math.isfinite(value))
        result.append(
            {
                "asset_id": asset,
                "minimum_observed_radius_m": round(finite[0], 3),
                "p10_observed_radius_m": round(finite[max(0, int(0.10 * len(finite)) - 1)], 3),
                "evidence": "revision-3 recorded centerline evidence from successful field driving",
            }
        )
    return result


def write_preview(source_points, candidate_points, obstacles, markers, change):
    figure, axis = plt.subplots(figsize=(14, 11))
    axis.plot(*zip(*source_points), color="#9ca3af", linewidth=1.0, label="authoritative consolidated route")
    axis.plot(*zip(*candidate_points), color="#0b63ce", linewidth=1.5, label="candidate mission")
    for asset, polygon in obstacles.items():
        x, y = polygon.exterior.xy
        axis.fill(x, y, color="#dc2626", alpha=0.12)
        axis.plot(x, y, color="#dc2626", linewidth=0.7)
        center = polygon.centroid
        axis.text(center.x, center.y, asset.replace("OBS-NEW-", "O"), fontsize=6, ha="center")
    for number, marker in enumerate(markers, 1):
        axis.scatter(marker["east_m"], marker["north_m"], marker="x", color="#f59e0b", s=30)
        axis.text(marker["east_m"] + 0.5, marker["north_m"] + 0.5, f"T{number}", fontsize=7)
    candidate_line = LineString(candidate_points)
    changed = [
        point
        for point in candidate_points
        if change["candidate_distance_start_m"] - 0.5
        <= candidate_line.project(Point(point))
        <= change["candidate_distance_end_m"] + 0.5
    ]
    if changed:
        axis.plot(*zip(*changed), color="#7c3aed", linewidth=3.0, label="G1 clearance correction")
    axis.scatter(*candidate_points[0], color="#16a34a", s=60, label="start")
    axis.scatter(*candidate_points[-1], color="#111827", s=60, label="parking end")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, linewidth=0.35, alpha=0.5)
    axis.set_xlabel("Local east (m)")
    axis.set_ylabel("Local north (m)")
    axis.set_title("62 Collins consolidated perimeter — APPROVED — blades off / supervised")
    axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(PREVIEW, dpi=180)
    plt.close(figure)


def replay_html(payload):
    encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    return """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>62 Collins mission review</title>
<style>html,body{margin:0;height:100%;font:14px system-ui;background:#101827;color:#e5e7eb}body{display:grid;grid-template-rows:auto 1fr auto}.top,.bottom{padding:10px 14px;background:#172033}.top{display:flex;gap:18px;align-items:center;flex-wrap:wrap}.warn{color:#fbbf24;font-weight:800}button,select,input{font:inherit}button{padding:6px 12px}canvas{width:100%;height:100%;display:block;background:#f8fafc}.bottom{display:flex;gap:12px;align-items:center;flex-wrap:wrap}.bottom input[type=range]{flex:1;min-width:240px}.tag{padding:3px 7px;border:1px solid #64748b;border-radius:999px}</style></head>
<body><div class="top"><span class="warn">APPROVED FOR SUPERVISED BLADES-OFF FIELD TEST</span><span>Physical e-stop and NRF handheld required</span><span class="tag" id="summary"></span><label><input id="source" type="checkbox" checked> authoritative route</label><label><input id="obstacles" type="checkbox" checked> obstacles</label><label><input id="turns" type="checkbox" checked> turn warnings</label><label><input id="deck" type="checkbox" checked> deck reference</label><button id="fit">Fit route</button></div><canvas id="map"></canvas><div class="bottom"><button id="play">▶ Play</button><button id="back">−10 m</button><button id="forward">+10 m</button><select id="rate"><option value="1">1×</option><option value="5">5×</option><option value="15">15×</option><option value="30">30×</option></select><input id="timeline" type="range" min="0" step="0.1"><span id="status"></span></div>
<script>const D=__DATA__;const C=document.getElementById('map'),X=C.getContext('2d');const $=id=>document.getElementById(id);let scale=1,ox=0,oy=0,distance=0,playing=false,last=null,drag=null;function resize(){const r=C.getBoundingClientRect(),q=devicePixelRatio||1;C.width=r.width*q;C.height=r.height*q;X.setTransform(q,0,0,q,0,0);draw()}function pt(p){return[ox+p[0]*scale,oy-p[1]*scale]}function fit(){const p=D.candidate,r=C.getBoundingClientRect(),xs=p.map(q=>q[0]),ys=p.map(q=>q[1]),pad=30;scale=Math.min((r.width-2*pad)/(Math.max(...xs)-Math.min(...xs)),(r.height-2*pad)/(Math.max(...ys)-Math.min(...ys)));ox=pad-Math.min(...xs)*scale;oy=pad+Math.max(...ys)*scale;draw()}function line(points,color,width,dash=[]){if(!points.length)return;X.save();X.beginPath();let q=pt(points[0]);X.moveTo(...q);for(let i=1;i<points.length;i++){q=pt(points[i]);X.lineTo(...q)}X.strokeStyle=color;X.lineWidth=width;X.setLineDash(dash);X.lineJoin='round';X.lineCap='round';X.stroke();X.restore()}function polygon(points){X.beginPath();let q=pt(points[0]);X.moveTo(...q);for(let i=1;i<points.length;i++)X.lineTo(...pt(points[i]));X.closePath();X.fillStyle='rgba(220,38,38,.15)';X.fill();X.strokeStyle='#dc2626';X.lineWidth=1;X.stroke()}function indexAt(d){let lo=0,hi=D.distance.length-1;while(lo<hi){const m=(lo+hi)>>1;if(D.distance[m]<d)lo=m+1;else hi=m}return lo}function draw(){const r=C.getBoundingClientRect();X.clearRect(0,0,r.width,r.height);if($('obstacles').checked)D.obstacles.forEach(o=>{polygon(o.points);const q=pt(o.center);X.fillStyle='#7f1d1d';X.font='10px system-ui';X.textAlign='center';X.fillText(o.id,q[0],q[1])});if($('source').checked)line(D.source,'#9ca3af',1.2,[5,4]);line(D.candidate,'#0b63ce',2.2);line(D.change,'#7c3aed',3.6);if($('turns').checked)D.markers.forEach((m,i)=>{const q=pt([m.east_m,m.north_m]);X.strokeStyle='#f59e0b';X.lineWidth=2;X.beginPath();X.moveTo(q[0]-5,q[1]-5);X.lineTo(q[0]+5,q[1]+5);X.moveTo(q[0]+5,q[1]-5);X.lineTo(q[0]-5,q[1]+5);X.stroke();X.fillStyle='#92400e';X.font='11px system-ui';X.fillText('T'+(i+1),q[0]+11,q[1]-7)});const i=indexAt(distance),p=D.candidate[i],a=D.candidate[Math.max(0,i-1)],b=D.candidate[Math.min(D.candidate.length-1,i+1)],h=Math.atan2(b[1]-a[1],b[0]-a[0]),q=pt(p);if($('deck').checked){X.beginPath();X.arc(q[0],q[1],D.deckWidth/2*scale,0,Math.PI*2);X.fillStyle='rgba(11,99,206,.10)';X.fill();X.strokeStyle='#0b63ce';X.stroke()}X.beginPath();X.arc(q[0],q[1],6,0,Math.PI*2);X.fillStyle='#16a34a';X.fill();X.strokeStyle='#fff';X.lineWidth=2;X.stroke();X.beginPath();X.moveTo(...q);X.lineTo(q[0]+Math.cos(h)*24,q[1]-Math.sin(h)*24);X.strokeStyle='#111827';X.lineWidth=2.5;X.stroke();$('status').textContent=distance.toFixed(1)+' / '+D.length.toFixed(1)+' m · heading '+((90-h*180/Math.PI+360)%360).toFixed(0)+'° · '+D.lookahead[i].toFixed(1)+' m lookahead'}function setD(v){distance=Math.max(0,Math.min(D.length,Number(v)));$('timeline').value=distance;draw()}function stop(){playing=false;last=null;$('play').textContent='▶ Play'}function animate(t){if(!playing)return;if(last!==null)setD(distance+(t-last)/1000*Number($('rate').value));last=t;if(distance>=D.length){stop();return}requestAnimationFrame(animate)}$('play').onclick=()=>{if(playing){stop();return}if(distance>=D.length)setD(0);playing=true;$('play').textContent='❚❚ Pause';requestAnimationFrame(animate)};$('back').onclick=()=>{stop();setD(distance-10)};$('forward').onclick=()=>{stop();setD(distance+10)};$('timeline').max=D.length;$('timeline').oninput=e=>{stop();setD(e.target.value)};$('fit').onclick=fit;['source','obstacles','turns','deck'].forEach(id=>$(id).onchange=draw);C.addEventListener('wheel',e=>{e.preventDefault();const r=C.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,k=e.deltaY<0?1.15:1/1.15,n=Math.max(.2,Math.min(80,scale*k));ox=mx-(mx-ox)*n/scale;oy=my-(my-oy)*n/scale;scale=n;draw()},{passive:false});C.onpointerdown=e=>{C.setPointerCapture(e.pointerId);drag=[e.clientX,e.clientY]};C.onpointermove=e=>{if(!drag)return;ox+=e.clientX-drag[0];oy+=e.clientY-drag[1];drag=[e.clientX,e.clientY];draw()};C.onpointerup=()=>drag=null;$('summary').textContent=D.candidate.length.toLocaleString()+' waypoints · '+D.length.toFixed(1)+' m · 13 turn reviews';new ResizeObserver(resize).observe(C);requestAnimationFrame(fit);</script></body></html>""".replace("__DATA__", encoded)


def verifier_text(mission_hash, audit_hash, waypoint_count, path_length):
    return f'''#!/usr/bin/env python3
"""Verify the exact approved 2026-09-29 consolidated perimeter package."""
import csv, hashlib, json, math
from pathlib import Path
HERE=Path(__file__).resolve().parent
G=HERE/"generated"
STEM="{STEM}"
MISSION=G/f"{{STEM}}.txt"; AUDIT=G/f"{{STEM}}_audit.csv"; REPORT=G/f"{{STEM}}_validation.json"
DASHBOARD=HERE/"mission_dashboard_consolidated_perimeter_20260929.py"
LAUNCHER=HERE/"run_62_Collins_consolidated_perimeter_20260929.sh"
EXPECTED_MISSION_SHA256="{mission_hash}"
EXPECTED_AUDIT_SHA256="{audit_hash}"
EXPECTED_ROWS={waypoint_count}
EXPECTED_PHASES=77
EXPECTED_LENGTH_M={path_length:.6f}
EXPECTED_STATUS="{STATUS}"
def digest(path): return hashlib.sha256(path.read_bytes().replace(b"\\r\\n",b"\\n")).hexdigest()
def main():
    for path in (MISSION,AUDIT,REPORT,DASHBOARD,LAUNCHER):
        if not path.is_file(): raise ValueError(f"missing required file: {{path}}")
    if digest(MISSION)!=EXPECTED_MISSION_SHA256: raise ValueError("mission checksum changed")
    if digest(AUDIT)!=EXPECTED_AUDIT_SHA256: raise ValueError("audit checksum changed")
    report=json.loads(REPORT.read_text(encoding="utf-8"))
    if report.get("status")!=EXPECTED_STATUS: raise ValueError("package status changed")
    if report.get("mission_sha256")!=EXPECTED_MISSION_SHA256 or report.get("audit_sha256")!=EXPECTED_AUDIT_SHA256: raise ValueError("report checksums disagree")
    rows=[line.split() for line in MISSION.read_text(encoding="ascii").splitlines()]
    if len(rows)!=EXPECTED_ROWS or any(len(row)!=5 for row in rows): raise ValueError("mission row or column count changed")
    if any(not all(math.isfinite(float(v)) for v in row) for row in rows): raise ValueError("non-finite mission value")
    if any(row[4]!="1.00" for row in rows): raise ValueError("speed must remain 1.00 m/s")
    with AUDIT.open(newline="",encoding="utf-8-sig") as handle: audit=list(csv.DictReader(handle))
    if len(audit)!=EXPECTED_ROWS: raise ValueError("audit row count changed")
    phases=[str(row.get("phase","")).strip() for row in audit]
    if any(not phase for phase in phases): raise ValueError("audit contains an empty mission phase")
    phase_blocks=[phase for index,phase in enumerate(phases) if index==0 or phase!=phases[index-1]]
    if len(phase_blocks)!=len(set(phase_blocks)): raise ValueError("audit mission phase reappears non-contiguously")
    if len(phase_blocks)!=EXPECTED_PHASES: raise ValueError("audit mission phase count changed")
    command_fields=("lat","lon","yaw_rad","lookahead_m","speed_mps")
    for index,(mission_row,audit_row) in enumerate(zip(rows,audit),1):
        if audit_row.get("waypoint")!=str(index): raise ValueError(f"audit waypoint sequence mismatch at row {{index}}")
        if any(audit_row.get(field)!=mission_row[offset] for offset,field in enumerate(command_fields)):
            raise ValueError(f"audit command fields disagree with mission at waypoint {{index}}")
    if report["geometry_validation"]["duplicate_consecutive_points"]!=0: raise ValueError("duplicate points reported")
    if report["geometry_validation"]["maximum_waypoint_gap_m"]>0.202: raise ValueError("waypoint gap exceeds 0.202 m")
    if report["geometry_validation"]["reversal_events_over_150_deg"]: raise ValueError("instantaneous reversal reported")
    if report["geometry_validation"]["heading_change_events_over_45_deg"]: raise ValueError("implausible per-waypoint heading change reported")
    if report["deck_envelope_validation"]["intersecting_obstacles"]: raise ValueError("deck envelope intersects an obstacle exclusion")
    if abs(report["candidate_route"]["length_m"]-EXPECTED_LENGTH_M)>0.001: raise ValueError("route length changed")
    dashboard_text=DASHBOARD.read_text(encoding="utf-8")
    if not all(marker in dashboard_text for marker in ("/api/note","tractor\\\\s+note","NOTE_JSONL","record_voice_note")): raise ValueError("hands-free voice-note dashboard feature is missing")
    if "APPROVED_FOR_FIELD=true" not in LAUNCHER.read_text(encoding="utf-8"): raise ValueError("approved launcher gate is not enabled")
    print("PASS: exact approved supervised blades-off field package verified.")
    print(f"      {{EXPECTED_ROWS:,}} waypoints; {{EXPECTED_LENGTH_M:.3f}} m; all speed commands 1.00 m/s.")
    print("      Approved for the initial directly supervised blades-off field test only.")
if __name__=="__main__":
    try: main()
    except (ValueError,KeyError,OSError) as exc: raise SystemExit(f"REVIEW PACKAGE VERIFICATION FAIL: {{exc}}")
'''


def launcher_text():
    return f'''#!/usr/bin/env bash
# Approved launcher for the initial supervised blades-off field test.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${{BASH_SOURCE[0]}}")" && pwd)"
TRACTOR_REPO="${{TRACTOR_REPO:-/home/al/tractor2025}}"
MISSION="${{SCRIPT_DIR}}/generated/{STEM}.txt"
AUDIT="${{SCRIPT_DIR}}/generated/{STEM}_audit.csv"
VERIFY="${{SCRIPT_DIR}}/verify_consolidated_perimeter_20260929.py"
PREFLIGHT="${{TRACTOR_REPO}}/tractor_rpi/testing/mission_preflight_20261002.py"
HEADING_CONFIG="${{TRACTOR_REPO}}/tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py"
CONTROLLER="${{TRACTOR_REPO}}/tractor_rpi/pure-pursuit/pure_pursuit_controller_20260915.py"
LOGGER="${{TRACTOR_REPO}}/tractor_rpi/field_test_logger_20260828.py"
WIFI_SERVER="${{TRACTOR_REPO}}/tractor_rpi/testing/webrtc/wifi_primary_control_20261003.py"
EXPECTED_FIRMWARE="teensy_main_20261003_wifi_v3"
APPROVED_FOR_FIELD=true

verify_only=false
dashboard_mode=false
configure_heading=false
case "${{1:-}}" in
    --verify-only) verify_only=true; shift ;;
    --dashboard) dashboard_mode=true; shift ;;
    --configure-heading) configure_heading=true; shift ;;
esac
[[ $# -eq 0 ]] || {{ echo "Usage: $0 [--verify-only|--dashboard|--configure-heading]" >&2; exit 2; }}

python3 "${{VERIFY}}"
if [[ "${{verify_only}}" == true ]]; then
    echo "Verification only; nothing was started."
    exit 0
fi

if [[ "${{APPROVED_FOR_FIELD}}" != true ]]; then
    echo "BLOCKED: package approval flag is not enabled." >&2
    echo "Do not bypass this gate without Al's replay/preview approval." >&2
    echo "No GPS configuration, logger, controller, or motion command was started." >&2
    exit 3
fi

if [[ "${{configure_heading}}" == true ]]; then
    echo "Stop rtcm-server before continuing. Applying the verified RAM-only Heading-F9P startup profile."
    sudo python3 -u "${{HEADING_CONFIG}}" --heading-startup --device-wait-seconds 30
    echo "Heading profile verified without an interactive confirmation."
    echo "Restart rtcm-server, then allow correction and heading solutions to settle before launch."
    exit 0
fi

for required in "${{MISSION}}" "${{AUDIT}}" "${{PREFLIGHT}}" "${{CONTROLLER}}" "${{LOGGER}}" "${{WIFI_SERVER}}"; do
    [[ -f "${{required}}" ]] || {{ echo "ERROR: required file not found: ${{required}}" >&2; exit 1; }}
done
pgrep -f '[p]ython3.*wifi_primary_control_20261003.py' >/dev/null || {{ echo "ERROR: Wi-Fi phone control server is not running" >&2; exit 1; }}
pgrep -f '[p]ython3.*field_test_logger_20260828.py' >/dev/null && {{ echo "ERROR: field logger already running" >&2; exit 1; }}
pgrep -f '[p]ython3.*pure_pursuit_controller_20260915.py' >/dev/null && {{ echo "ERROR: Pure Pursuit controller already running" >&2; exit 1; }}

echo "============================================================"
echo " 62 COLLINS CONSOLIDATED PERIMETER — BLADES OFF / SUPERVISED"
echo " Speed 1.00 m/s; left deck edge follows the outer perimeter"
echo " Keep the phone control page open and physical e-stop immediately available"
echo "============================================================"
echo "Keep the phone in Pause. Checking Wi-Fi heartbeat, Teensy bridge, RTK corrections, and heading..."
sleep 15
if [[ "${{dashboard_mode}}" == true ]]; then
    python3 "${{PREFLIGHT}}" --expected-firmware "${{EXPECTED_FIRMWARE}}"
else
    sudo python3 "${{PREFLIGHT}}" --expected-firmware "${{EXPECTED_FIRMWARE}}"
fi

python3 - <<'PY'
import json, socket, time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", 6003))
sock.settimeout(1.0)
latest = None
deadline = time.monotonic() + 5.0
while time.monotonic() < deadline:
    try:
        latest = json.loads(sock.recvfrom(65535)[0])
    except socket.timeout:
        continue
sock.close()
if latest is None:
    raise SystemExit("ERROR: no Teensy status on UDP 6003")
wifi = latest.get("wifi_control", {{}})
if wifi.get("mode") != 0:
    raise SystemExit(f"ERROR: phone must be in Pause; Wi-Fi mode={{wifi.get('mode')!r}}")
if wifi.get("heartbeat_fresh") != 1:
    raise SystemExit("ERROR: phone heartbeat is not fresh; keep the control page open")
if wifi.get("estop_latched") != 0:
    raise SystemExit("ERROR: Wi-Fi E-stop relay is latched")
try:
    command_age_ms = int(wifi.get("command_age_ms"))
except (TypeError, ValueError):
    raise SystemExit("ERROR: Wi-Fi command age is missing")
if command_age_ms > 500:
    raise SystemExit(f"ERROR: Wi-Fi command is stale ({{command_age_ms}} ms)")
print(f"PASS: phone control is live in Pause; command age={{command_age_ms}} ms; E-stop released")
PY

python3 - "${{MISSION}}" <<'PY'
import json, math, socket, sys, time
first=open(sys.argv[1],encoding="utf-8").readline().split()
start_lat,start_lon,start_yaw=map(float,first[:3])
target_heading=(90.0-math.degrees(start_yaw))%360.0
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);sock.bind(("",6010));sock.settimeout(5.0)
latest=None;deadline=time.monotonic()+5.0
while time.monotonic()<deadline:
    try: latest=json.loads(sock.recvfrom(65535)[0])
    except socket.timeout: break
sock.close()
if latest is None: raise SystemExit("ERROR: no navigation GPS packet on UDP 6010")
lat=float(latest["lat"]);lon=float(latest["lon"]);heading=float(latest["heading_deg"])
east=(lon-start_lon)*111320.0*math.cos(math.radians(start_lat));north=(lat-start_lat)*110540.0
position_error=math.hypot(east,north);heading_error=abs((heading-target_heading+180)%360-180)
print(f"Start error: distance={{position_error:.2f}} m, heading={{heading_error:.1f}} deg")
if latest.get("fix_quality")!="RTK Fixed": raise SystemExit("ERROR: RTK Fixed required")
if not latest.get("headValid") or str(latest.get("carrier","")).lower()!="fixed": raise SystemExit("ERROR: valid fixed-carrier heading required")
baseline=latest.get("relpos_length_m");accuracy=latest.get("relpos_heading_accuracy_deg")
if baseline is None or not .80<=float(baseline)<=1.30: raise SystemExit(f"ERROR: heading baseline {{baseline!r}} outside 0.80-1.30 m")
if accuracy is None or float(accuracy)>1.0: raise SystemExit(f"ERROR: heading accuracy {{accuracy!r}} exceeds 1.0 degree")
if position_error>1.5: raise SystemExit("ERROR: use dashboard voice guidance to move within 1.50 m of start")
if heading_error>20: raise SystemExit("ERROR: use dashboard voice guidance to align within 20 degrees")
PY

CONFIRMATION="RUN CONSOLIDATED PERIMETER BLADES OFF"
if [[ "${{dashboard_mode}}" != true ]]; then
    read -r -p "Type ${{CONFIRMATION}} to start: " response
    [[ "${{response}}" == "${{CONFIRMATION}}" ]] || {{ echo "Aborted; nothing started."; exit 1; }}
fi

log_dir="/home/al/field_logs/20260929_consolidated_perimeter_1mps"
mkdir -p "${{log_dir}}"
field_log="${{log_dir}}/consolidated_perimeter_$(date '+%Y%m%d_%H%M%S').csv"
logger_pid=""
cleanup() {{
    if [[ -n "${{logger_pid}}" ]] && kill -0 "${{logger_pid}}" 2>/dev/null; then kill "${{logger_pid}}"; wait "${{logger_pid}}" 2>/dev/null || true; fi
    echo "Controller shutdown preserves neutral safety behavior. Field log: ${{field_log}}"
}}
trap cleanup EXIT
trap 'exit 130' INT TERM
python3 -u "${{LOGGER}}" --output "${{field_log}}" & logger_pid=$!
sleep 2
kill -0 "${{logger_pid}}" 2>/dev/null || {{ echo "ERROR: logger stopped during startup" >&2; exit 1; }}
controller_args=(python3 -u "${{CONTROLLER}}" "${{MISSION}}" --mode live --gps-port 6010 --status-port 6003 --min-fix "RTK Fixed" --ip 127.0.0.1 --port 6004 --max-speed 1.00 --tracking-window 6.0 --audit-file "${{AUDIT}}" --reacquire-max-advance 5.0 --resume-stable-seconds 0.0 --basic-runtime-heading-gate --no-operator-cycle-after-safety-loss)
if [[ "${{dashboard_mode}}" == true ]]; then controller_args+=(--control-port 6011 --telemetry-port 6012); fi
"${{controller_args[@]}}"
'''


def dashboard_text():
    template = r'''#!/usr/bin/env python3
"""Voice-guidance and hands-free note dashboard for the approved perimeter test."""
from __future__ import annotations
import csv
import importlib.util
import json
import os
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path
from urllib import error as urllib_error
from urllib import request as urllib_request
from urllib.parse import urlparse

HERE=Path(__file__).resolve().parent
SOURCE=HERE.parents[4]/"tractor_rpi/pure-pursuit/mission_dashboard_20260910.py"
spec=importlib.util.spec_from_file_location("mission_dashboard_base",SOURCE)
dashboard=importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(dashboard)
dashboard.MISSION=HERE/"generated/__STEM__.txt"
dashboard.AUDIT=HERE/"generated/__STEM___audit.csv"
dashboard.LAUNCHER=HERE/"run_62_Collins_consolidated_perimeter_20260929.sh"
dashboard.EXPECTED_CONFIRMATION="RUN CONSOLIDATED PERIMETER BLADES OFF"

base_safe_to_start=dashboard.safe_to_start
def wifi_safe_to_start(state):
    ok,reason=base_safe_to_start(state)
    if not ok:
        return False,reason.replace("handheld","phone").replace("Handheld","Phone")
    snap=state.snapshot();wifi=snap.get("bridge",{}).get("wifi_control",{})
    if wifi.get("mode")!=0:
        return False,"Put the phone in Pause before starting"
    if wifi.get("heartbeat_fresh")!=1:
        return False,"Phone heartbeat is not fresh; keep the Wi-Fi control page open"
    if wifi.get("estop_latched")!=0:
        return False,"Wi-Fi E-stop is latched; reset it and remain in Pause"
    try: command_age_ms=int(wifi.get("command_age_ms"))
    except (TypeError,ValueError): return False,"Wi-Fi command age is missing"
    if command_age_ms>500:
        return False,f"Phone command is stale ({command_age_ms} ms)"
    return True,""
dashboard.safe_to_start=wifi_safe_to_start

def notify_wifi_dashboard_url(dashboard_url):
    message=(
        "Open this Tractor01 mission dashboard on the laptop. Keep the Wi-Fi "
        "phone control page in the phone's foreground and in Pause; keep the physical e-stop available. This "
        "temporary link includes the operator key.\n\n"+dashboard_url
    )
    request=urllib_request.Request(
        dashboard.NTFY_TOPIC_URL,data=message.encode("utf-8"),
        headers={"Title":"Tractor01 dashboard ready","Click":dashboard_url,"Tags":"tractor"},
        method="POST",
    )
    try:
        with urllib_request.urlopen(request,timeout=5) as response:
            if not 200<=response.status<300: raise RuntimeError(f"ntfy returned HTTP {response.status}")
        print(f"ntfy: preferred dashboard link sent to {dashboard.NTFY_TOPIC_URL}")
    except (urllib_error.URLError,OSError,RuntimeError) as exc:
        print(f"WARNING: could not send dashboard link to ntfy: {exc}")
dashboard.notify_dashboard_url=notify_wifi_dashboard_url

NOTE_DIR=Path(os.environ.get(
    "TRACTOR_VOICE_NOTE_DIR",
    "/home/al/field_logs/20260929_consolidated_perimeter_1mps",
))
NOTE_SESSION=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
NOTE_CSV=NOTE_DIR/f"voice_notes_{NOTE_SESSION}.csv"
NOTE_JSONL=NOTE_DIR/f"voice_notes_{NOTE_SESSION}.jsonl"
NOTE_LOCK=threading.Lock()
NOTE_FIELDS=[
    "note_id","server_timestamp_utc","client_timestamp","transcript",
    "mission_active","process_state","waypoint_index","waypoint_number",
    "route_distance_m","lat","lon","heading_deg","actual_speed_mps",
    "commanded_speed_mps","cross_track_error_m","lookahead_m","fix_quality",
    "heading_carrier","controller_age_s","gps_age_s",
]
with dashboard.AUDIT.open(newline="",encoding="utf-8-sig") as handle:
    AUDIT_ROWS=list(csv.DictReader(handle))


def clean_number(value):
    try: return float(value)
    except (TypeError,ValueError): return None


def record_voice_note(state,payload):
    transcript=" ".join(str(payload.get("transcript","")).split())
    if not transcript: raise ValueError("Voice note transcript is empty")
    if len(transcript)>500: raise ValueError("Voice note transcript exceeds 500 characters")
    snap=state.snapshot();controller=snap.get("controller",{});gps=snap.get("gps",{})
    try: waypoint_index=max(0,int(controller.get("waypoint_idx",0)))
    except (TypeError,ValueError): waypoint_index=0
    audit=AUDIT_ROWS[min(waypoint_index,max(0,len(AUDIT_ROWS)-1))] if AUDIT_ROWS else {}
    lat=clean_number(gps.get("lat"));lon=clean_number(gps.get("lon"));heading=clean_number(gps.get("heading_deg"))
    if lat is None: lat=clean_number(controller.get("lat"))
    if lon is None: lon=clean_number(controller.get("lon"))
    if heading is None: heading=clean_number(controller.get("heading_compass_deg"))
    note={
        "note_id":uuid.uuid4().hex,
        "server_timestamp_utc":datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "client_timestamp":str(payload.get("client_timestamp", "")),
        "transcript":transcript,
        "mission_active":bool(snap.get("mission_active")),
        "process_state":snap.get("process_state"),
        "waypoint_index":waypoint_index,
        "waypoint_number":waypoint_index+1,
        "route_distance_m":clean_number(audit.get("distance_m")),
        "lat":lat,"lon":lon,"heading_deg":heading,
        "actual_speed_mps":clean_number(controller.get("actual_speed_mps")),
        "commanded_speed_mps":clean_number(controller.get("speed_cmd_mps")),
        "cross_track_error_m":clean_number(controller.get("cross_track_err_m")),
        "lookahead_m":clean_number(controller.get("lookahead_dist_m")),
        "fix_quality":controller.get("fix_quality") or gps.get("fix_quality"),
        "heading_carrier":controller.get("heading_carrier") or gps.get("carrier"),
        "controller_age_s":clean_number(snap.get("controller_age_s")),
        "gps_age_s":clean_number(snap.get("gps_age_s")),
    }
    json_record=dict(note)
    json_record["telemetry"]={"controller":controller,"gps":gps,"bridge":snap.get("bridge",{})}
    with NOTE_LOCK:
        NOTE_DIR.mkdir(parents=True,exist_ok=True)
        new_csv=not NOTE_CSV.exists()
        with NOTE_CSV.open("a",newline="",encoding="utf-8") as handle:
            writer=csv.DictWriter(handle,fieldnames=NOTE_FIELDS)
            if new_csv: writer.writeheader()
            writer.writerow({key:note.get(key) for key in NOTE_FIELDS})
        with NOTE_JSONL.open("a",encoding="utf-8",newline="\n") as handle:
            handle.write(json.dumps(dashboard.json_safe(json_record),separators=(",",":"),allow_nan=False)+"\n")
    return note


base_handler_factory=dashboard.handler_factory
def voice_handler_factory(state,token,mission_payload):
    BaseHandler=base_handler_factory(state,token,mission_payload)
    class VoiceHandler(BaseHandler):
        def do_POST(self):
            if urlparse(self.path).path!="/api/note":
                return super().do_POST()
            if not self.authorized():
                self.send_json({"error":"Invalid operator key"},403);return
            length=int(self.headers.get("Content-Length","0"))
            if length>4096:
                self.send_json({"error":"Voice note request is too large"},413);return
            try:
                payload=json.loads(self.rfile.read(length) or b"{}")
                note=record_voice_note(state,payload)
            except (ValueError,OSError) as exc:
                self.send_json({"error":str(exc)},400);return
            self.send_json({"ok":True,"note":note})
    return VoiceHandler
dashboard.handler_factory=voice_handler_factory

replacements={
"Tractor01 — 62 Collins clear-sky resume":"Tractor01 — consolidated perimeter field test",
"Resumes at source waypoint 91. Recovery stays in the current phase and may advance at most 30 m.":"Approved blades-off field test. Voice notes listen for ‘Tractor note…’ while guidance is active.",
"Start the reviewed clear-sky resume mission at source waypoint 91 with blades off?":"Start the approved consolidated perimeter mission with blades off and direct supervision?",
"RUN PARTIAL RINGS BLADES OFF":dashboard.EXPECTED_CONFIRMATION,
"Keep the handheld with you.</b> After a browser Pause: select handheld Pause, press CLEAR PAUSE, confirm HANDHELD PAUSE, then select Auto.":"Keep the phone control page in the phone's foreground.</b> Use this dashboard on the laptop. After a dashboard Pause: select phone Pause, press CLEAR PAUSE, then use guarded Auto only when the route is clear.",
"Keep the handheld in Pause until the controller is ready.":"Keep the phone in Pause until the controller is ready.",
"HANDHELD PAUSE":"PHONE PAUSE",
"Handheld Pause":"Phone Pause",
"Handheld modes":"Control modes",
}
for original,updated in replacements.items():
    if dashboard.HTML.count(original)!=1: raise RuntimeError(f"Dashboard text changed; expected one occurrence of: {original}")
    dashboard.HTML=dashboard.HTML.replace(original,updated)

radio_fact="fact('Radio / steering state',(b.radio?.signal||'—')+' / '+(st.state||'—'))"
wifi_fact="fact('Wi-Fi heartbeat / steering state',(b.wifi_control?.heartbeat_fresh===1?'fresh':'stale')+' / '+(st.state||'—'))"
if dashboard.HTML.count(radio_fact)!=1: raise RuntimeError("Dashboard radio status fact changed")
dashboard.HTML=dashboard.HTML.replace(radio_fact,wifi_fact)

toolbar_old='<button id="guide" class="button guide">START VOICE GUIDANCE</button>'
toolbar_new=toolbar_old+'<span id="noteStatus" class="badge">VOICE NOTES READY</span>'
if dashboard.HTML.count(toolbar_old)!=1: raise RuntimeError("Dashboard voice-guidance button changed")
dashboard.HTML=dashboard.HTML.replace(toolbar_old,toolbar_new)

voice_js=r"""
const noteStatus=document.getElementById('noteStatus');
const SpeechRecognitionAPI=window.SpeechRecognition||window.webkitSpeechRecognition;
let noteRecognition=null,noteWanted=false,noteRunning=false,noteSpeechActive=false,noteRestartTimer=null,noteRestartDelay=500,pendingWakeUntil=0;
function setNoteStatus(text,kind=''){noteStatus.textContent=text;noteStatus.className='badge '+kind}
function scheduleNoteRestart(delay=500){clearTimeout(noteRestartTimer);if(noteWanted&&!noteSpeechActive)noteRestartTimer=setTimeout(startVoiceNotes,delay)}
function startVoiceNotes(){
  noteWanted=true;
  if(!SpeechRecognitionAPI){setNoteStatus('VOICE NOTES UNSUPPORTED','bad');return}
  if(noteRunning||noteSpeechActive)return;
  if(!noteRecognition){
    noteRecognition=new SpeechRecognitionAPI();noteRecognition.continuous=true;noteRecognition.interimResults=false;noteRecognition.lang='en-US';
    noteRecognition.onstart=()=>{noteRunning=true;noteRestartDelay=500;setNoteStatus('LISTENING: SAY TRACTOR NOTE','ok')};
    noteRecognition.onspeechstart=()=>setNoteStatus('HEARING SPEECH','warn');
    noteRecognition.onresult=e=>{for(let i=e.resultIndex;i<e.results.length;i++){if(!e.results[i].isFinal)continue;handleNoteTranscript(e.results[i][0].transcript)}};
    noteRecognition.onerror=e=>{noteRunning=false;if(e.error==='not-allowed'||e.error==='service-not-allowed'){noteWanted=false;setNoteStatus('MICROPHONE PERMISSION BLOCKED','bad')}else if(e.error==='network'){noteRestartDelay=5000;setNoteStatus('VOICE NOTES NEED INTERNET — RETRYING','bad')}else if(e.error!=='aborted'&&e.error!=='no-speech'){noteRestartDelay=1500;setNoteStatus('VOICE ERROR: '+e.error,'bad')}};
    noteRecognition.onend=()=>{noteRunning=false;scheduleNoteRestart(noteRestartDelay)};
  }
  try{noteRecognition.start()}catch(e){if(e.name!=='InvalidStateError'){setNoteStatus('VOICE START FAILED','bad');scheduleNoteRestart(1500)}}
}
function stopVoiceNotes(){noteWanted=false;noteSpeechActive=false;pendingWakeUntil=0;clearTimeout(noteRestartTimer);if(noteRecognition&&noteRunning){try{noteRecognition.stop()}catch(e){}}noteRunning=false;setNoteStatus('VOICE NOTES STOPPED','')}
function pauseVoiceNotesForSpeech(){noteSpeechActive=true;clearTimeout(noteRestartTimer);if(noteRecognition&&noteRunning){try{noteRecognition.stop()}catch(e){}}}
function resumeVoiceNotesAfterSpeech(){noteSpeechActive=false;scheduleNoteRestart(350)}
async function handleNoteTranscript(raw){
  const transcript=String(raw||'').trim(),match=transcript.match(/\btractor\s+note\b[\s,:-]*(.*)$/i);
  let note='';
  if(match){note=match[1].trim();pendingWakeUntil=note?0:Date.now()+8000;if(!note){setNoteStatus('SAY YOUR NOTE','warn');return}}
  else if(Date.now()<pendingWakeUntil){note=transcript;pendingWakeUntil=0}
  else{setNoteStatus('LISTENING: SAY TRACTOR NOTE','ok');return}
  if(!note)return;
  setNoteStatus('SAVING NOTE…','warn');
  try{await api('/api/note','POST',{transcript:note,client_timestamp:new Date().toISOString()});setNoteStatus('NOTE SAVED','ok');speak('Note saved.')}
  catch(e){setNoteStatus('NOTE SAVE FAILED','bad');speak('Voice note was not saved.')}
}
if(!SpeechRecognitionAPI)setNoteStatus('VOICE NOTES UNSUPPORTED','bad');
else if(!window.isSecureContext)setNoteStatus('VOICE NOTES READY — HTTP MAY BLOCK MIC','warn');
"""
anchor="guideBtn.onclick=()=>{"
if dashboard.HTML.count(anchor)!=1: raise RuntimeError("Dashboard guide handler changed")
dashboard.HTML=dashboard.HTML.replace(anchor,voice_js+"\n"+anchor)

old_speak="function speak(text){if(!voiceGuidance||!('speechSynthesis' in window))return;window.speechSynthesis.cancel();const utterance=new SpeechSynthesisUtterance(text);utterance.rate=.9;utterance.volume=1;window.speechSynthesis.speak(utterance)}"
new_speak="function speak(text){if(!voiceGuidance||!('speechSynthesis' in window))return;pauseVoiceNotesForSpeech();window.speechSynthesis.cancel();const utterance=new SpeechSynthesisUtterance(text);utterance.rate=.9;utterance.volume=1;utterance.onend=resumeVoiceNotesAfterSpeech;utterance.onerror=resumeVoiceNotesAfterSpeech;window.speechSynthesis.speak(utterance)}"
if dashboard.HTML.count(old_speak)!=1: raise RuntimeError("Dashboard speak function changed")
dashboard.HTML=dashboard.HTML.replace(old_speak,new_speak)

old_stop="function stopGuidance(message='Voice guidance stopped.'){\n  voiceGuidance=false;"
new_stop="function stopGuidance(message='Voice guidance stopped.'){\n  stopVoiceNotes();voiceGuidance=false;"
if dashboard.HTML.count(old_stop)!=1: raise RuntimeError("Dashboard stop-guidance function changed")
dashboard.HTML=dashboard.HTML.replace(old_stop,new_stop)

old_start="voiceGuidance=true;lastGuidanceAt=0;"
new_start="voiceGuidance=true;startVoiceNotes();lastGuidanceAt=0;"
if dashboard.HTML.count(old_start)!=1: raise RuntimeError("Dashboard start-guidance state changed")
dashboard.HTML=dashboard.HTML.replace(old_start,new_start)

if __name__=="__main__":
    print(f"Hands-free voice-note CSV:   {NOTE_CSV}")
    print(f"Hands-free voice-note JSONL: {NOTE_JSONL}")
    dashboard.main()
'''
    return template.replace("__STEM__", STEM)


def readme_text(report):
    return f'''# 62 Collins consolidated perimeter — approved field test

Al visually approved the static preview and interactive replay on 2026-09-29. This package is approved only for the initial **directly supervised, blades-off** field test. It is not a mowing or unattended mission.

## Candidate summary

- {report["candidate_route"]["waypoints"]:,} waypoints at approximately 0.20 m spacing
- {report["candidate_route"]["length_m"]:.3f} m total length
- 1.00 m/s at every waypoint
- 2.0 m normal lookahead, reduced to 1.5 or 1.0 m near reviewed tight turns
- Blades off and direct supervision required for the first field validation
- Start at local (0.000, 0.000); finish at parking near ({report["candidate_route"]["end_east_m"]:.3f}, {report["candidate_route"]["end_north_m"]:.3f})

## Material geometry changes

G1 changes only submitted segment P6 near OBS-NEW-011. The source centerline brought the 42-inch deck envelope into the revision-3 candidate obstacle exclusion. The replacement follows the nearest 0.90 m centerline-clearance arc, leaving approximately {report["material_geometry_changes"][0]["result_deck_edge_clearance_m"]:.3f} m between the nominal deck edge and the exclusion. This owner-approved change is purple in both previews.

G2 and G3 remove two centimeter-scale near-180° join backtracks. G4 removes four sub-0.10 m duplicate join points. G5 rounds seven instantaneous heading changes over 40° within short local windows; its maximum centerline shift is {report["material_geometry_changes"][-1]["maximum_centerline_shift_m"]:.3f} m. The gray source overlay makes all of these differences reviewable.

The other 12 turn-review markers retain the submitted geometry. Their radii are judged against successful field evidence at obstacles 004, 006, and 013 rather than rejected solely for being below 1.63 m.

## Review files

- `{PREVIEW.name}` — full-route static preview
- `{REPLAY.name}` — interactive Play/Pause replay with timeline, heading, obstacles, turn warnings, and deck reference
- `{REPORT.name}` — validation and every turn decision
- `{AUDIT.name}` — waypoint lineage, commands, and 77 unique contiguous phases used for phase-locked recovery
- `{MISSION.name}` — five-column mission file
- `{DASHBOARD.name}` — matching phone dashboard and voice guidance adapter

## Hands-free voice notes

Starting **Voice Guidance** also starts Chrome speech recognition. Accept the microphone permission before moving the tractor. During the run, begin each comment with **“Tractor note”**, for example:

- “Tractor note, move one foot left.”
- “Tractor note, clearance is too close.”

The dashboard says “Note saved” and writes both CSV and JSONL under `/home/al/field_logs/20260929_consolidated_perimeter_1mps/`. Each record includes timestamps, transcript, waypoint, route distance, GPS position, heading, speed, cross-track error, lookahead, fix quality, and a telemetry snapshot. Notes never alter the live mission.

Chrome may use an online recognition service, so internet access can be required. The dashboard displays `LISTENING`, `MICROPHONE PERMISSION BLOCKED`, `VOICE NOTES NEED INTERNET`, or another explicit status. Because guidance speech temporarily pauses recognition to prevent self-transcription, wait until the dashboard finishes speaking before saying “Tractor note.” Plain-HTTP microphone policy can vary by Chrome release; if Chrome blocks the tractor dashboard origin, use an HTTPS/trusted-origin setup before relying on voice notes.

## Safe verification

On Tractor01, after pulling the future approved commit, the review package can be checked without starting anything:

```bash
cd /home/al/tractor2025
python3 {VERIFY.relative_to(REPO).as_posix()}
```

or:

```bash
bash {LAUNCHER.relative_to(REPO).as_posix()} --verify-only
```

## Field sequence

The approved launcher retains all of these safeguards:

1. Blades disengaged; direct supervision; Wi-Fi phone control page open in Pause and physical e-stop ready.
2. Reconfigure and verify the Heading F9P with the non-interactive RAM-only 5 Hz startup profile before starting rtcm-server; this enables USB UBX NAV-RELPOSNED and disables targeted USB NMEA output.
3. Allow startup time for the Wi-Fi phone heartbeat, Teensy bridge, RTK corrections, and fixed heading.
4. Run preflight expecting `teensy_main_20261003_wifi_v3` and require a released Wi-Fi E-stop, RTK corrections, RTK Fixed, valid fixed-carrier heading, 0.80–1.30 m baseline, heading accuracy ≤1.0°, healthy JRK, and stationary phone Pause.
5. Keep the Wi-Fi control page in the phone's foreground, and open the mission dashboard on the laptop. Use guarded phone Manual to reach and align with the initial waypoint. Dashboard voice guidance is optional and may remain off.
6. Start field logging before Pure Pursuit and preserve neutral shutdown traps.
7. Run the first validation supervised and blades off; Pause immediately for unexpected clearance or tracking behavior.
'''


def main():
    GENERATED.mkdir(parents=True, exist_ok=True)
    source_raw = SOURCE_ROUTE.read_bytes()
    source = json.loads(source_raw)
    roundtrip_ok = source == json.loads(json.dumps(source, allow_nan=False))
    entries = source["route"]
    source_points = [(float(item["east_m"]), float(item["north_m"])) for item in entries]
    source_gaps = [math.dist(a, b) for a, b in zip(source_points, source_points[1:])]
    source_length = distance(source_points)
    origin_lat = float(source["origin"]["lat"])
    origin_lon = float(source["origin"]["lon"])
    convert = local_transform(origin_lat, origin_lon)

    inventory = json.loads(INVENTORY.read_text(encoding="utf-8"))
    obstacles = {
        feature["properties"]["asset_id"]: transform(convert, shape(feature["geometry"]))
        for feature in inventory["features"]
        if feature.get("properties", {}).get("geometry_role") in (
            "candidate_mowing_exclusion",
            "mowing_exclusion",
        )
    }
    cleaned_points, cleaned_entries, join_changes = remove_micro_backtracks(source_points, entries)
    candidate_points, geometry_change = correct_obs11_clearance(
        cleaned_points, cleaned_entries, obstacles["OBS-NEW-011"]
    )
    pre_smooth_line = LineString(candidate_points)
    g1_start_point = pre_smooth_line.interpolate(geometry_change["candidate_distance_start_m"])
    g1_end_point = pre_smooth_line.interpolate(geometry_change["candidate_distance_end_m"])
    candidate_points, sharp_turn_change = smooth_implausible_turns(candidate_points)
    candidate_line = LineString(candidate_points)
    geometry_change["candidate_distance_start_m"] = round(candidate_line.project(g1_start_point), 3)
    geometry_change["candidate_distance_end_m"] = round(candidate_line.project(g1_end_point), 3)
    final_obs11_clearance = candidate_line.distance(obstacles["OBS-NEW-011"])
    geometry_change["result_centerline_clearance_m"] = round(final_obs11_clearance, 3)
    geometry_change["result_deck_edge_clearance_m"] = round(final_obs11_clearance - HALF_DECK_M, 3)
    candidate_length = candidate_line.length
    cumulative = [0.0]
    for before, after in zip(candidate_points, candidate_points[1:]):
        cumulative.append(cumulative[-1] + math.dist(before, after))

    markers = load_turn_markers()
    demonstrated = demonstrated_turns(inventory, convert)
    demonstrated_floor = min(item["minimum_observed_radius_m"] for item in demonstrated)
    for marker in markers:
        marker_point = Point(marker["east_m"], marker["north_m"])
        marker["candidate_distance_m"] = round(candidate_line.project(marker_point), 3)
        candidate_near = candidate_line.interpolate(marker["candidate_distance_m"])
        marker["candidate_east_m"] = round(candidate_near.x, 3)
        marker["candidate_north_m"] = round(candidate_near.y, 3)
        nearest_asset, nearest_polygon = min(
            obstacles.items(), key=lambda item: marker_point.distance(item[1])
        )
        marker["nearest_obstacle"] = nearest_asset
        marker["source_marker_deck_clearance_m"] = round(
            marker_point.distance(nearest_polygon) - HALF_DECK_M, 3
        )
        if marker["segment_id"] == "P6":
            marker["decision"] = "MODIFIED_G1_FOR_OBS11_DECK_CLEARANCE"
            marker["rationale"] = (
                "Local route arc moved away from OBS-NEW-011; owner approved the replay on 2026-09-29."
            )
        else:
            marker["decision"] = "RETAINED_AS_SUBMITTED"
            marker["rationale"] = (
                f"No deck-envelope conflict at marker; {demonstrated_floor:.3f} m or tighter "
                "radii appear in successful obstacle 004/006/013 field evidence."
            )

    lookahead = []
    for along in cumulative:
        nearby_radii = [
            marker["source_review_radius_m"]
            for marker in markers
            if abs(marker["candidate_distance_m"] - along) <= 3.0
        ]
        if nearby_radii and min(nearby_radii) < 1.20:
            lookahead.append(1.0)
        elif nearby_radii:
            lookahead.append(1.5)
        else:
            lookahead.append(2.0)

    audit_rows = []
    mission_rows = []
    east_scale = 111_320.0 * math.cos(math.radians(origin_lat))
    north_scale = 110_540.0
    source_cursor = 0
    previous_segment_id = None
    route_phase_index = 0
    route_phase = ""
    for index, ((east, north), along) in enumerate(zip(candidate_points, cumulative)):
        before = candidate_points[max(0, index - 1)]
        after = candidate_points[min(len(candidate_points) - 1, index + 1)]
        yaw = math.atan2(after[1] - before[1], after[0] - before[0])
        lat = origin_lat + north / north_scale
        lon = origin_lon + east / east_scale
        search_end = min(len(entries), source_cursor + 40)
        source_index = min(
            range(source_cursor, search_end),
            key=lambda candidate_index: math.dist(
                (entries[candidate_index]["east_m"], entries[candidate_index]["north_m"]),
                (east, north),
            ),
        )
        source_cursor = source_index
        source_entry = entries[source_index]
        source_segment_id = source_entry["segment_id"]
        if source_segment_id != previous_segment_id:
            route_phase_index += 1
            route_phase = f"route_phase_{route_phase_index:03d}_{source_segment_id}"
            previous_segment_id = source_segment_id
        nearest_source_distance = float(source_entry["distance_m"])
        changed = (
            geometry_change["candidate_distance_start_m"] - 0.01
            <= along
            <= geometry_change["candidate_distance_end_m"] + 0.01
        )
        mission_rows.append((lat, lon, yaw, lookahead[index], SPEED_MPS))
        audit_rows.append(
            {
                "waypoint": index + 1,
                "phase": route_phase,
                "lat": f"{lat:.9f}",
                "lon": f"{lon:.9f}",
                "yaw_rad": f"{yaw:.9f}",
                "lookahead_m": f"{lookahead[index]:.2f}",
                "speed_mps": f"{SPEED_MPS:.2f}",
                "east_m": f"{east:.3f}",
                "north_m": f"{north:.3f}",
                "distance_m": f"{along:.3f}",
                "source_sequence": source_entry["sequence"],
                "source_distance_m": f"{nearest_source_distance:.3f}",
                "source_segment_id": source_segment_id,
                "source_kind": source_entry["kind"],
                "geometry_status": "G1_OBS11_CLEARANCE_CORRECTION" if changed else "AUTHORITATIVE_ROUTE",
            }
        )

    MISSION.write_text(
        "".join(
            f"{lat:.9f} {lon:.9f} {yaw:.9f} {ahead:.2f} {speed:.2f}\n"
            for lat, lon, yaw, ahead, speed in mission_rows
        ),
        encoding="ascii",
        newline="\n",
    )
    with AUDIT.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(audit_rows[0]))
        writer.writeheader()
        writer.writerows(audit_rows)

    candidate_gaps = [math.dist(a, b) for a, b in zip(candidate_points, candidate_points[1:])]
    heading_events = clustered_events(
        [
            (index, heading_change_degrees(candidate_points[index - 1], candidate_points[index], candidate_points[index + 1]))
            for index in range(1, len(candidate_points) - 1)
            if heading_change_degrees(candidate_points[index - 1], candidate_points[index], candidate_points[index + 1]) > 45.0
        ]
    )
    reversals = [event for event in heading_events if event[1] > 150.0]
    deck = candidate_line.buffer(HALF_DECK_M, cap_style=2, join_style=2)
    obstacle_results = []
    for asset, polygon in obstacles.items():
        centerline_clearance = candidate_line.distance(polygon)
        overlap = deck.intersection(polygon).area
        obstacle_results.append(
            {
                "asset_id": asset,
                "centerline_clearance_m": round(centerline_clearance, 3),
                "nominal_deck_edge_clearance_m": round(centerline_clearance - HALF_DECK_M, 3),
                "deck_intersection_area_m2": round(overlap, 6),
                "intersects": overlap > 1e-6,
            }
        )
    intersecting = [item["asset_id"] for item in obstacle_results if item["intersects"]]
    source_line = LineString(source_points)
    max_shift = max(Point(point).distance(source_line) for point in candidate_points)
    mean_shift = sum(Point(point).distance(source_line) for point in candidate_points) / len(candidate_points)
    geometry_change["maximum_centerline_shift_m"] = round(max_shift, 3)
    geometry_change["mean_route_shift_m"] = round(mean_shift, 4)

    report = {
        "status": STATUS,
        "safety_language": "BLADES OFF; DIRECTLY SUPERVISED; PHYSICAL E-STOP AND NRF HANDHELD REQUIRED",
        "authorization": "Approved by Al on 2026-09-29 for the initial directly supervised blades-off field test only.",
        "owner_visual_approval": {
            "approved": True,
            "date": "2026-09-29",
            "scope": "static PNG and interactive HTML replay; supervised blades-off path testing",
        },
        "source_route": str(SOURCE_ROUTE.relative_to(REPO)).replace("\\", "/"),
        "source_route_sha256": hashlib.sha256(source_raw).hexdigest(),
        "source_review_report": str(SOURCE_REPORT.relative_to(REPO)).replace("\\", "/"),
        "site_inventory": str(INVENTORY.relative_to(REPO)).replace("\\", "/"),
        "source_roundtrip_validation": {
            "parse_serialize_parse_equal": roundtrip_ok,
            "points": len(source_points),
            "calculated_length_m": round(source_length, 6),
            "stored_final_distance_m": entries[-1]["distance_m"],
            "maximum_gap_m": round(max(source_gaps), 6),
            "duplicate_consecutive_points": sum(gap <= 1e-9 for gap in source_gaps),
        },
        "candidate_route": {
            "waypoints": len(candidate_points),
            "recovery_phases": route_phase_index,
            "length_m": round(candidate_length, 6),
            "nominal_motion_time_minutes_at_1mps": round(candidate_length / 60.0, 3),
            "spacing_target_m": SPACING_M,
            "start_east_m": round(candidate_points[0][0], 3),
            "start_north_m": round(candidate_points[0][1], 3),
            "end_east_m": round(candidate_points[-1][0], 3),
            "end_north_m": round(candidate_points[-1][1], 3),
            "speed_mps": SPEED_MPS,
            "lookahead_policy_m": {"normal": 2.0, "reviewed_turn": 1.5, "tight_reviewed_turn": 1.0},
        },
        "geometry_validation": {
            "duplicate_consecutive_points": sum(gap <= 1e-9 for gap in candidate_gaps),
            "maximum_waypoint_gap_m": round(max(candidate_gaps), 6),
            "minimum_nonzero_waypoint_gap_m": round(min(gap for gap in candidate_gaps if gap > 0), 6),
            "heading_change_events_over_45_deg": [
                {"waypoint": index + 1, "change_deg": round(value, 2)} for index, value in heading_events
            ],
            "reversal_events_over_150_deg": [
                {"waypoint": index + 1, "change_deg": round(value, 2)} for index, value in reversals
            ],
            "self_intersections": intersection_audit(candidate_points),
            "interpretation": (
                "Non-simple geometry is expected where the approved sequence deliberately reuses access paths; "
                "unexpected discontinuities and instantaneous reversals are validation failures."
            ),
        },
        "deck_envelope_validation": {
            "deck_width_m": DECK_WIDTH_M,
            "half_width_m": HALF_DECK_M,
            "inventory_results": obstacle_results,
            "intersecting_obstacles": intersecting,
        },
        "material_geometry_changes": [geometry_change] + join_changes + ([sharp_turn_change] if sharp_turn_change else []),
        "turn_capability_field_evidence": demonstrated,
        "turn_marker_reviews": markers,
        "september_28_field_run_context": {
            "source": str(PRIOR_RUN.relative_to(REPO)).replace("\\", "/"),
            "active_path_error_m": {"median": 0.092, "p95": 0.260, "maximum": 1.905},
            "within_0_5m_percent": 99.04,
            "steering_pwm_saturated_percent_active": 0.09,
            "firmware": "teensy_main_20260926",
            "interpretation": (
                "The completed run supports sub-1.63 m turn capability but also justifies explicit deck-envelope review and direct supervision."
            ),
        },
        "field_launch_requirements": {
            "expected_firmware": "teensy_main_20260926",
            "heading_f9p_reconfiguration_expected": True,
            "startup_wait_required_for": ["NRF radio", "Teensy bridge", "RTK corrections", "heading solution"],
            "preflight": [
                "RTK correction stream",
                "RTK Fixed",
                "valid fixed-carrier heading",
                "0.80-1.30 m heading baseline",
                "heading accuracy <= 1.0 degree",
                "Teensy bridge and expected firmware identity",
                "JRK health",
                "stationary Pause safe starting mode",
            ],
            "voice_guidance_to_start": True,
            "logging_required": True,
            "safe_neutral_shutdown_required": True,
            "physical_estop_and_nrf_supervision_may_not_be_weakened": True,
        },
        "hands_free_voice_notes": {
            "enabled": True,
            "wake_phrase": "Tractor note",
            "browser": "Chrome Web Speech API",
            "outputs": ["CSV", "JSONL with telemetry snapshot"],
            "field_log_directory": "/home/al/field_logs/20260929_consolidated_perimeter_1mps",
            "informational_only_never_changes_live_mission": True,
            "online_recognition_may_require_internet": True,
            "plain_http_microphone_permission_may_be_blocked_by_chrome": True,
        },
        "unresolved_owner_action": None,
    }
    report["mission_sha256"] = sha256(MISSION)
    report["audit_sha256"] = sha256(AUDIT)
    REPORT.write_text(json.dumps(report, indent=2), encoding="utf-8", newline="\n")

    change_points = [
        list(point)
        for point, row in zip(candidate_points, audit_rows)
        if row["geometry_status"] != "AUTHORITATIVE_ROUTE"
    ]
    write_preview(source_points, candidate_points, obstacles, markers, geometry_change)
    replay_payload = {
        "status": STATUS,
        "source": [list(point) for point in source_points],
        "candidate": [list(point) for point in candidate_points],
        "distance": cumulative,
        "lookahead": lookahead,
        "length": candidate_length,
        "deckWidth": DECK_WIDTH_M,
        "change": change_points,
        "markers": markers,
        "obstacles": [
            {
                "id": asset.replace("OBS-NEW-", "O"),
                "points": [list(point) for point in polygon.exterior.coords],
                "center": [polygon.centroid.x, polygon.centroid.y],
            }
            for asset, polygon in obstacles.items()
        ],
    }
    REPLAY.write_text(replay_html(replay_payload), encoding="utf-8", newline="\n")
    VERIFY.write_text(
        verifier_text(report["mission_sha256"], report["audit_sha256"], len(candidate_points), candidate_length),
        encoding="utf-8",
        newline="\n",
    )
    LAUNCHER.write_text(launcher_text(), encoding="utf-8", newline="\n")
    DASHBOARD.write_text(dashboard_text(), encoding="utf-8", newline="\n")
    README.write_text(readme_text(report), encoding="utf-8", newline="\n")
    print(
        json.dumps(
            {
                "status": STATUS,
                "output": str(OUTDIR),
                "source_points": len(source_points),
                "source_length_m": round(source_length, 3),
                "candidate_waypoints": len(candidate_points),
                "candidate_length_m": round(candidate_length, 3),
                "maximum_shift_m": geometry_change["maximum_centerline_shift_m"],
                "deck_intersections": intersecting,
                "turn_markers": len(markers),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
