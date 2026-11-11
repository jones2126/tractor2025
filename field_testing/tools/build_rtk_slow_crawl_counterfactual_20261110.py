#!/usr/bin/env python3
"""Build a historical RTK-outage slow-crawl counterfactual for replay samples 1040-1340.

This is a route-constrained kinematic what-if, not a reconstruction and not a
motion authorization. It applies the recorded RTK outage times to a simulated
bicycle model: 0.30 m/s while the recorded fix is below RTK Fixed and 1.00 m/s
otherwise. Pure Pursuit steering is recomputed from the simulated pose and the
reviewed route.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import re


SITE = Path(__file__).resolve().parents[1] / "sites" / "62_Collins_multi_boundary_20260915"
DEFAULT_REPLAY = SITE / "analysis" / "20261109_phone_outer_perimeter_replay" / "replay_20261109_phone_outer_perimeter.html"
DEFAULT_EVIDENCE = SITE / "runs" / "20261109_phone_outer_perimeter" / "evidence" / "20261109_phone_outer_perimeter_collection"
DEFAULT_OUTPUT = SITE / "analysis" / "20261109_phone_outer_perimeter_replay" / "rtk_slow_crawl_counterfactual_samples_1040_1340.html"

START_SAMPLE = 1040
END_SAMPLE = 1340
SLOW_SPEED_MPS = 0.30
NORMAL_SPEED_MPS = 1.00
WHEELBASE_M = 1.27
MAX_STEER_RAD = 0.623
LOOKAHEAD_M = 2.0


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, default=DEFAULT_EVIDENCE)
    parser.add_argument("--replay-html", type=Path, default=DEFAULT_REPLAY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def number(value, default=None):
    try:
        result = float(value)
        return result if math.isfinite(result) else default
    except (TypeError, ValueError):
        return default


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_replay(path):
    text = path.read_text(encoding="utf-8")
    match = re.search(r"const D=(.*?);\s*const samples", text, re.S)
    if not match:
        raise ValueError(f"embedded replay data not found: {path}")
    return json.loads(match.group(1))


def load_pursuit(path, start_time, end_time, lat0, lon0, scale_x, scale_y):
    rows = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            stamp = number(row.get("timestamp"))
            x = number(row.get("pos_x_m"))
            y = number(row.get("pos_y_m"))
            heading = number(row.get("heading_compass_deg"))
            if x is None or y is None:
                latitude = number(row.get("lat"))
                longitude = number(row.get("lon"))
                if latitude is not None and longitude is not None:
                    x = (longitude - lon0) * scale_x
                    y = (latitude - lat0) * scale_y
            if stamp is None or x is None or y is None:
                continue
            if start_time <= stamp <= end_time:
                row["_t"], row["_x"], row["_y"], row["_heading"] = stamp, x, y, heading
                rows.append(row)
    if len(rows) < 2:
        raise ValueError("insufficient pursuit rows in requested replay interval")
    return rows


def load_route(path):
    rows = [
        [float(value) for value in line.split()]
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    lat0, lon0 = rows[0][:2]
    scale_x = 111_320.0 * math.cos(math.radians(lat0))
    scale_y = 110_540.0
    points = [((row[1] - lon0) * scale_x, (row[0] - lat0) * scale_y) for row in rows]
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.hypot(second[0] - first[0], second[1] - first[1]))
    return points, cumulative, lat0, lon0, scale_x, scale_y


def closest_route_index(route, x, y, hint):
    lower = max(0, hint - 40)
    upper = min(len(route), hint + 120)
    return min(range(lower, upper), key=lambda index: (route[index][0] - x) ** 2 + (route[index][1] - y) ** 2)


def simulate(rows, route, cumulative):
    initial = next((row for row in rows if row["_heading"] is not None), None)
    if initial is None:
        raise ValueError("no usable heading in requested replay interval")
    x, y = rows[0]["_x"], rows[0]["_y"]
    theta = math.radians(90.0 - initial["_heading"])
    hint = int(number(rows[0].get("waypoint_idx"), 0))
    result = [[round(x, 4), round(y, 4), round(rows[0]["_t"], 3), rows[0].get("fix_quality") != "RTK Fixed"]]
    distance = 0.0
    max_cte = 0.0
    for previous, current in zip(rows, rows[1:]):
        delta_t = max(0.0, min(0.2, current["_t"] - previous["_t"]))
        outage = previous.get("fix_quality") != "RTK Fixed"
        speed = SLOW_SPEED_MPS if outage else NORMAL_SPEED_MPS
        hint = closest_route_index(route, x, y, hint)
        target_distance = cumulative[hint] + LOOKAHEAD_M
        target_index = min(len(route) - 1, bisect.bisect_left(cumulative, target_distance))
        target_x, target_y = route[target_index]
        alpha = math.atan2(target_y - y, target_x - x) - theta
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        lookahead = max(0.25, math.hypot(target_x - x, target_y - y))
        steering = math.atan2(2.0 * WHEELBASE_M * math.sin(alpha), lookahead)
        steering = max(-MAX_STEER_RAD, min(MAX_STEER_RAD, steering))
        yaw_rate = speed / WHEELBASE_M * math.tan(steering)
        mid_theta = theta + 0.5 * yaw_rate * delta_t
        x += speed * math.cos(mid_theta) * delta_t
        y += speed * math.sin(mid_theta) * delta_t
        theta += yaw_rate * delta_t
        distance += speed * delta_t
        hint = closest_route_index(route, x, y, hint)
        max_cte = max(max_cte, math.hypot(route[hint][0] - x, route[hint][1] - y))
        result.append([round(x, 4), round(y, 4), round(current["_t"], 3), current.get("fix_quality") != "RTK Fixed"])
    return result, distance, max_cte, hint


def outage_intervals(rows):
    intervals = []
    active = None
    for index, row in enumerate(rows):
        bad = row.get("fix_quality") != "RTK Fixed"
        if bad and active is None:
            active = [index, index, row.get("fix_quality", "unavailable")]
        elif bad:
            active[1] = index
        elif active is not None:
            start, end, quality = active
            duration = row["_t"] - rows[start]["_t"]
            intervals.append({
                "start": round(rows[start]["_t"], 3),
                "end": round(row["_t"], 3),
                "duration": round(duration, 3),
                "distance": round(duration * SLOW_SPEED_MPS, 3),
                "quality": quality,
            })
            active = None
    if active is not None:
        start, end, quality = active
        duration = rows[end]["_t"] - rows[start]["_t"]
        intervals.append({
            "start": round(rows[start]["_t"], 3),
            "end": round(rows[end]["_t"], 3),
            "duration": round(duration, 3),
            "distance": round(duration * SLOW_SPEED_MPS, 3),
            "quality": quality,
        })
    return intervals


def html_document(data):
    encoded = json.dumps(data, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RTK slow-crawl counterfactual</title>
<style>
:root{{font:15px system-ui,sans-serif;color-scheme:dark}}body{{margin:0;padding:18px;background:#101820;color:#edf3f7}}main{{max-width:1450px;margin:auto}}h1{{font-size:1.45rem;margin:0 0 5px}}p{{color:#adbdc8;margin:5px 0 12px}}.badge{{display:inline-block;padding:4px 8px;border:1px solid #e8ae50;border-radius:999px;color:#ffd166;font-weight:700}}.controls,.legend,.metrics{{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin:10px 0}}button{{font:inherit;border:1px solid #678097;background:#243746;color:#fff;border-radius:5px;padding:6px 10px}}button:hover{{background:#345066}}canvas{{display:block;width:100%;height:auto;aspect-ratio:1200/720;background:#17232d;border:1px solid #506675;border-radius:4px;touch-action:none}}.legend span{{color:#c5d4de}}.legend i{{display:inline-block;width:18px;height:3px;margin-right:5px;vertical-align:middle}}.metric{{background:#1b2a34;padding:7px 10px;border-radius:4px}}.metric span{{display:block;color:#a5b8c5;font-size:.78rem}}table{{border-collapse:collapse;width:100%;margin-top:12px}}th,td{{padding:7px 9px;border-bottom:1px solid #435766;text-align:left}}th{{color:#a5b8c5}}.note{{font-size:.86rem}}@media(max-width:600px){{body{{padding:10px}}}}
</style></head><body><main>
<div class="badge">DRAFT - UNTESTED IN THE FIELD · HISTORICAL COUNTERFACTUAL</div>
<h1>RTK-outage slow-crawl what-if · replay samples 1040–1340</h1>
<p>Route-constrained bicycle-model estimate: 0.30 m/s only while the recorded position fix was below RTK Fixed; 1.00 m/s otherwise. This is not a reconstruction or a control recommendation.</p>
<div class="controls"><button id="zin">Zoom in</button><button id="zreset">Reset view</button><button id="zout">Zoom out</button><span>Mouse wheel zooms · drag pans</span></div>
<div class="legend"><span><i style="background:#8296a8"></i>target route</span><span><i style="background:#3bd7ab"></i>recorded path</span><span><i style="background:#b68cff"></i>hypothetical path</span><span><i style="background:#ffad5c"></i>hypothetical travel during RTK outage</span></div>
<canvas id="map" width="1200" height="720" role="img" aria-label="Top-down equal-scale comparison of target, recorded, and hypothetical paths"></canvas>
<div class="metrics" id="metrics"></div><table><thead><tr><th>Recorded outage</th><th>Fix state</th><th>Duration</th><th>Distance at 0.30 m/s</th></tr></thead><tbody id="outages"></tbody></table>
<p class="note">Assumptions: the recorded outage times still occur; moving-base heading remains usable; the reviewed route remains available; Pure Pursuit is recomputed from the simulated pose with 2.0 m lookahead, 1.27 m wheelbase, and ±0.623 rad steering limit. No tire slip, terrain disturbance, actuator lag, or GNSS-position error is modeled.</p>
<p class="note" id="sources"></p>
</main><script>
const D={encoded};const canvas=document.getElementById('map'),ctx=canvas.getContext('2d');let zoom=1,panX=0,panY=0,drag=null;
const all=[...D.target,...D.actual,...D.hypothetical].map(p=>[p[0],p[1]]);const raw={{minX:Math.min(...all.map(p=>p[0])),maxX:Math.max(...all.map(p=>p[0])),minY:Math.min(...all.map(p=>p[1])),maxY:Math.max(...all.map(p=>p[1]))}};
function tr(){{const dx=raw.maxX-raw.minX+8,dy=raw.maxY-raw.minY+8,cx=(raw.minX+raw.maxX)/2+panX,cy=(raw.minY+raw.maxY)/2+panY,s=Math.min((canvas.width-60)/(dx/zoom),(canvas.height-60)/(dy/zoom));return{{x:x=>canvas.width/2+(x-cx)*s,y:y=>canvas.height/2-(y-cy)*s,s}}}}
function path(points,T,color,width,dash=[]){{ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(T.x(p[0]),T.y(p[1])):ctx.moveTo(T.x(p[0]),T.y(p[1])));ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([])}}
function draw(){{const T=tr();ctx.fillStyle='#17232d';ctx.fillRect(0,0,canvas.width,canvas.height);ctx.strokeStyle='#29404d';ctx.lineWidth=1;for(let i=0;i<=12;i++){{ctx.beginPath();ctx.moveTo(i*100,0);ctx.lineTo(i*100,720);ctx.stroke()}}for(let i=0;i<=9;i++){{ctx.beginPath();ctx.moveTo(0,i*80);ctx.lineTo(1200,i*80);ctx.stroke()}}path(D.target,T,'#8296a8',2);path(D.actual,T,'#3bd7ab',2.2);path(D.hypothetical,T,'#b68cff',3,[9,5]);let segment=[];for(const p of D.hypothetical){{if(p[3])segment.push(p);else if(segment.length){{path(segment,T,'#ffad5c',4);segment=[]}}}}if(segment.length)path(segment,T,'#ffad5c',4);for(const [p,c,label] of [[D.actual[0],'#3bd7ab','start'],[D.actual.at(-1),'#3bd7ab','recorded end'],[D.hypothetical.at(-1),'#b68cff','hypothetical end']]){{ctx.beginPath();ctx.arc(T.x(p[0]),T.y(p[1]),6,0,Math.PI*2);ctx.fillStyle=c;ctx.fill();ctx.fillStyle='#edf3f7';ctx.font='14px system-ui';ctx.fillText(label,T.x(p[0])+9,T.y(p[1])-8)}}ctx.fillStyle='#edf3f7';ctx.font='14px system-ui';ctx.fillText(`equal scale · zoom ${{Math.round(zoom*100)}}%`,14,24)}}
function setZoom(v){{zoom=Math.max(.5,Math.min(20,v));draw()}}document.getElementById('zin').onclick=()=>setZoom(zoom*1.35);document.getElementById('zout').onclick=()=>setZoom(zoom/1.35);document.getElementById('zreset').onclick=()=>{{zoom=1;panX=panY=0;draw()}};canvas.onwheel=e=>{{e.preventDefault();setZoom(e.deltaY<0?zoom*1.18:zoom/1.18)}};canvas.onpointerdown=e=>{{drag=[e.clientX,e.clientY,panX,panY];canvas.setPointerCapture(e.pointerId)}};canvas.onpointermove=e=>{{if(!drag)return;const T=tr();panX=drag[2]-(e.clientX-drag[0])/T.s;panY=drag[3]+(e.clientY-drag[1])/T.s;draw()}};canvas.onpointerup=()=>drag=null;
const fmt=(v,n=1)=>Number(v).toFixed(n);document.getElementById('metrics').innerHTML=`<div class="metric"><span>Recorded window</span><b>${{fmt(D.windowSeconds)}} s</b></div><div class="metric"><span>RTK outages</span><b>${{D.outages.length}} · ${{fmt(D.outageSeconds,3)}} s</b></div><div class="metric"><span>Hypothetical outage travel</span><b>${{fmt(D.outageDistance,2)}} m</b></div><div class="metric"><span>Total hypothetical travel</span><b>${{fmt(D.hypotheticalDistance,1)}} m</b></div><div class="metric"><span>Modeled maximum route error</span><b>${{fmt(D.maxCte,2)}} m</b></div>`;
const clock=t=>new Date(t*1000).toLocaleTimeString('en-US',{{timeZone:'America/New_York'}});document.getElementById('outages').innerHTML=D.outages.map((o,i)=>`<tr><td>${{i+1}} · ${{clock(o.start)}}–${{clock(o.end)}}</td><td>${{o.quality}}</td><td>${{fmt(o.duration,3)}} s</td><td>${{fmt(o.distance,3)}} m</td></tr>`).join('');document.getElementById('sources').textContent='Sources: '+D.sources.map(s=>`${{s.name}} [SHA-256 ${{s.sha256}}]`).join(' · ');draw();
</script></body></html>"""


def main():
    args = arguments()
    replay = load_replay(args.replay_html)
    start_time = replay["samples"][START_SAMPLE - 1]["t"]
    end_time = replay["samples"][END_SAMPLE - 1]["t"]
    pursuit_path = args.evidence_root / "pursuit" / "pursuit_log_20261109_154326.csv"
    route_path = args.evidence_root / "package" / "62_Collins_outer_perimeter_field_test_1mps_20260928.txt"
    for path in (args.replay_html, pursuit_path, route_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    route, cumulative, lat0, lon0, scale_x, scale_y = load_route(route_path)
    rows = load_pursuit(pursuit_path, start_time, end_time, lat0, lon0, scale_x, scale_y)
    hypothetical, hypothetical_distance, max_cte, end_hint = simulate(rows, route, cumulative)
    actual = [[round(row["_x"], 4), round(row["_y"], 4), round(row["_t"], 3), row.get("fix_quality") != "RTK Fixed"] for row in rows]
    intervals = outage_intervals(rows)
    start_hint = int(number(rows[0].get("waypoint_idx"), 0))
    target = [[round(x, 4), round(y, 4)] for x, y in route[max(0, start_hint - 50):min(len(route), end_hint + 80)]]
    data = {
        "target": target,
        "actual": actual,
        "hypothetical": hypothetical,
        "outages": intervals,
        "windowSeconds": round(end_time - start_time, 3),
        "outageSeconds": round(sum(item["duration"] for item in intervals), 3),
        "outageDistance": round(sum(item["distance"] for item in intervals), 3),
        "hypotheticalDistance": round(hypothetical_distance, 3),
        "maxCte": round(max_cte, 3),
        "sources": [
            {"name": pursuit_path.name, "sha256": digest(pursuit_path)},
            {"name": route_path.name, "sha256": digest(route_path)},
            {"name": args.replay_html.name, "sha256": digest(args.replay_html)},
            {"name": Path(__file__).name, "sha256": digest(Path(__file__))},
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(html_document(data), encoding="utf-8", newline="\n")
    print(args.output)
    print(json.dumps({key: data[key] for key in ("windowSeconds", "outageSeconds", "outageDistance", "hypotheticalDistance", "maxCte")}, indent=2))


if __name__ == "__main__":
    main()
