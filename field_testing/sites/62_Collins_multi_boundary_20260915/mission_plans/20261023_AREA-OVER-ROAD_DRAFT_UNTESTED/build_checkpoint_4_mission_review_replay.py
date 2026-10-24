#!/usr/bin/env python3
"""Build a self-contained, timestamped review replay for CP4 route revision 002."""

from __future__ import annotations

import json
import math
from pathlib import Path

from shapely.geometry import LineString, Polygon, mapping, shape
from shapely.ops import transform, unary_union


STATUS = "DRAFT - UNTESTED IN THE FIELD"
CUT_WIDTH_M = 0.9652
DECK_WIDTH_M = 1.0668
HERE = Path(__file__).resolve().parent
ROOT = Path(__file__).resolve().parents[5]
MOWABLE = ROOT / (
    "field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/assets/"
    "AREA-OVER-ROAD/revisions/rev_001_20261023_DRAFT_UNTESTED/"
    "AREA-OVER-ROAD_mowable_area.geojson"
)
ROUTE = HERE / "checkpoint_4_detailed_draft_rev002.geojson"
METRICS = HERE / "checkpoint_4_validation_rev002.json"
OUTPUT = HERE / "checkpoint_4_mission_review_replay_rev005.html"


def polygon_rings(geometry):
    polygons = list(geometry.geoms) if hasattr(geometry, "geoms") else [geometry]
    return [list(polygon.exterior.coords) for polygon in polygons if polygon.geom_type == "Polygon" and polygon.area > 1e-5]


def main() -> None:
    mowable_wgs84 = shape(json.loads(MOWABLE.read_text(encoding="utf-8"))["features"][0]["geometry"])
    ref_lat, ref_lon = mowable_wgs84.centroid.y, mowable_wgs84.centroid.x
    east_scale = 111_320.0 * math.cos(math.radians(ref_lat))
    north_scale = 110_540.0

    def project(geometry):
        return transform(
            lambda lon, lat, z=None: ((lon - ref_lon) * east_scale, (lat - ref_lat) * north_scale),
            geometry,
        )

    area = project(mowable_wgs84)
    outer = Polygon(area.exterior)
    hole = Polygon(area.interiors[0])
    route_geojson = json.loads(ROUTE.read_text(encoding="utf-8"))
    metrics = json.loads(METRICS.read_text(encoding="utf-8"))

    colors = {
        "access": "#7c3aed",
        "connector": "#9333ea",
        "pole_loop": "#ea580c",
        "perimeter": "#16a34a",
        "stripe": "#2563eb",
        "stripe_with_pole_detour": "#0f766e",
        "headland_turn": "#dc2626",
    }
    segments = []
    for feature in route_geojson["features"]:
        properties = feature["properties"]
        line = project(shape(feature["geometry"]))
        segments.append(
            {
                "id": properties["segment_id"],
                "type": properties["segment_type"],
                "phase": properties["phase"],
                "note": properties["note"],
                "source": properties["source"],
                "start": properties["cumulative_start_m"],
                "end": properties["cumulative_end_m"],
                "length": properties["length_m"],
                "color": colors[properties["segment_type"]],
                "points": [[round(x, 4), round(y, 4)] for x, y in line.coords],
            }
        )

    cutting_lines = [project(shape(feature["geometry"])) for feature in route_geojson["features"] if feature["properties"]["segment_type"] != "access"]
    planned_union = unary_union([line.buffer(CUT_WIDTH_M / 2.0, cap_style=2, join_style=1) for line in cutting_lines])
    physical_union = unary_union([line.buffer(DECK_WIDTH_M / 2.0, cap_style=2, join_style=1) for line in cutting_lines])
    planned_gaps = area.difference(planned_union)
    physical_gaps = area.difference(physical_union)

    by_id = {segment["id"]: segment for segment in segments}
    questions = [
        {
            "id": "B1",
            "time": by_id["S002"]["start"],
            "title": "Access line into short connector — 152.4° join",
            "question": "The route concept is approved, but this instantaneous heading change cannot be sent to the controller. A continuous forward-only entry transition must replace it before field approval.",
        },
        {
            "id": "B2",
            "time": by_id["S003"]["start"],
            "title": "Short connector into perimeter — 90.0° join",
            "question": "This second entry join also needs a continuous radius-valid transition. It is only 0.08 seconds after the prior marker, so zoom in to distinguish them.",
        },
        {
            "id": "B3",
            "time": by_id["S004"]["start"],
            "title": "Perimeter into pole loop — 129.7° join",
            "question": "The intended leave-edge pole loop is approved. Its exact entry tangent must be rebuilt as one continuous forward-only maneuver without reducing obstacle clearance.",
        },
        {
            "id": "B4",
            "time": by_id["S005"]["start"],
            "title": "Pole loop back to perimeter — 130.1° join",
            "question": "The pole-loop exit must be tangent to the continuing perimeter while retaining the approved loop behavior and modeled pole/deck/chute clearance.",
        },
        {
            "id": "B5",
            "time": by_id["S020"]["start"],
            "title": "Last stripe into exit connector — 54.4° join",
            "question": "The approved shorter exit direction remains, but this join needs a continuous forward-only transition on already-cut ground.",
        },
        {
            "id": "B6",
            "time": by_id["S020"]["end"],
            "title": "Exit connector into O-out/F-out access — 62.4° join",
            "question": "This final join also has a 0.078 m position gap. It must be replaced by one continuous path before the dashboard Start action can be enabled.",
        },
    ]

    def point_at_time(timestamp):
        for segment in segments:
            if timestamp <= segment["end"] + 1e-9:
                fraction = 0.0 if segment["length"] <= 0 else max(0.0, min(1.0, (timestamp - segment["start"]) / segment["length"]))
                line = LineString(segment["points"])
                point = line.interpolate(fraction, normalized=True)
                return [round(point.x, 4), round(point.y, 4)]
        return segments[-1]["points"][-1]

    for question in questions:
        question["point"] = point_at_time(question["time"])

    data = {
        "status": STATUS,
        "speed_mps": 1.0,
        "total_seconds": metrics["route"]["total_length_m"],
        "segments": segments,
        "outer": [[round(x, 4), round(y, 4)] for x, y in outer.exterior.coords],
        "hole": [[round(x, 4), round(y, 4)] for x, y in hole.exterior.coords],
        "planned_gaps": [[[round(x, 4), round(y, 4)] for x, y in ring] for ring in polygon_rings(planned_gaps)],
        "physical_gaps": [[[round(x, 4), round(y, 4)] for x, y in ring] for ring in polygon_rings(physical_gaps)],
        "questions": questions,
        "coverage": metrics["coverage"],
        "display_rotation_rad": math.radians(
            metrics["parameters"]["stripe_heading_compass_deg"]
            if metrics["parameters"]["stripe_heading_compass_deg"] <= 180.0
            else metrics["parameters"]["stripe_heading_compass_deg"] - 360.0
        ),
    }
    payload = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")

    html_text = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AREA-OVER-ROAD mission review replay — revision 005</title>
<style>
:root{--ink:#0f172a;--muted:#475569;--line:#cbd5e1;--paper:#fff;--bg:#f1f5f9;--warn:#b91c1c;--question:#f59e0b}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.45 system-ui,-apple-system,Segoe UI,sans-serif}header{padding:18px 24px;background:#fff;border-bottom:1px solid var(--line)}h1{margin:0;font-size:24px}.status{color:var(--warn);font-weight:850;margin-top:3px}.sub{color:var(--muted);margin:5px 0 0}.layout{display:grid;grid-template-columns:minmax(520px,1.6fr) minmax(320px,.8fr);gap:16px;padding:16px;max-width:1500px;margin:auto}.card{background:var(--paper);border:1px solid var(--line);border-radius:12px;box-shadow:0 2px 8px #0f172a12}.map-card{padding:12px}.map-wrap{position:relative}svg{width:100%;height:min(72vh,800px);display:block;background:#fff;border-radius:8px}.controls{padding:12px 2px 2px}.time-row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}.clock{font:700 20px ui-monospace,Consolas,monospace;min-width:132px}button,select{border:1px solid #94a3b8;background:white;border-radius:7px;padding:7px 10px;cursor:pointer}button:hover{background:#f8fafc}input[type=range]{width:100%;accent-color:#2563eb}.timeline{position:relative;padding-top:12px}.tick{position:absolute;top:0;width:2px;height:10px;background:var(--question);cursor:pointer}.meta{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-top:8px}.meta div{background:#f8fafc;border-radius:7px;padding:7px}.meta small{display:block;color:var(--muted)}.layers{display:flex;gap:14px;flex-wrap:wrap;margin-top:8px;color:var(--muted)}.side{padding:14px;max-height:calc(100vh - 120px);overflow:auto}.side h2{margin:0 0 6px}.summary{color:var(--muted);margin:0 0 12px}.q{border:1px solid var(--line);border-left:5px solid var(--question);border-radius:8px;padding:10px;margin:9px 0}.q.active{background:#fffbeb;border-color:#f59e0b}.q-head{display:flex;justify-content:space-between;gap:8px;align-items:center}.q-title{font-weight:750}.q-time{font:700 13px ui-monospace,Consolas,monospace;color:#92400e}.q p{margin:7px 0}.jump{font-size:13px;padding:5px 8px}.answer{width:100%;min-height:54px;border:1px solid #cbd5e1;border-radius:6px;padding:7px;font:inherit}.export{width:100%;margin-top:10px;background:#0f172a;color:#fff}.legend{font-size:13px;color:var(--muted);margin-top:8px}.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:4px}@media(max-width:900px){.layout{grid-template-columns:1fr}.side{max-height:none}svg{height:62vh}.meta{grid-template-columns:1fr}}
</style></head>
<body><header><h1>AREA-OVER-ROAD draft mission review replay</h1><div class="status">DRAFT - UNTESTED IN THE FIELD</div><p class="sub">Approved route concept revision 002; non-executable replay revision 005. True top-down equal-scale view. Six timestamped join blockers remain before field-package approval. Mission clock assumes 1.0 m/s.</p></header>
<main class="layout"><section class="card map-card"><div class="map-wrap"><svg id="map" viewBox="0 0 920 760" preserveAspectRatio="xMidYMid meet" aria-label="True top-down interactive mission replay map with equal metric scale"></svg></div><div class="controls"><div class="time-row"><button id="play">Play</button><button id="pause">Pause</button><button id="restart">Restart</button><label>Speed <select id="rate"><option value="1">1×</option><option value="5" selected>5×</option><option value="10">10×</option><option value="20">20×</option></select></label><span class="clock" id="clock">00:00.0</span><button id="zoomIn" aria-label="Zoom in">Zoom in</button><button id="zoomOut" aria-label="Zoom out">Zoom out</button><button id="focus" aria-label="Focus on tractor">Focus tractor</button><button id="resetView" aria-label="Reset map view">Reset view</button></div><div class="timeline"><div id="ticks"></div><input id="slider" type="range" min="0" max="707.1791" step="0.01" value="0"></div><div class="meta"><div><small>Segment</small><strong id="segment">S001 · access</strong></div><div><small>Phase</small><strong id="phase">access in</strong></div><div><small>Distance</small><strong id="distance">0.0 m</strong></div></div><div class="layers"><label><input id="show38" type="checkbox" checked> 38-in modeled gaps</label><label><input id="show42" type="checkbox"> 42-in modeled gaps</label><label><input id="showAll" type="checkbox" checked> full route</label></div><div class="legend">True top-down · 1:1 metric aspect · mouse wheel or buttons zoom · drag to pan · long mowing edge vertical · <span class="dot" style="background:#16a34a"></span>perimeter &nbsp; <span class="dot" style="background:#ea580c"></span>pole loop &nbsp; <span class="dot" style="background:#2563eb"></span>stripes &nbsp; <span class="dot" style="background:#dc2626"></span>1.4 m turns &nbsp; <span class="dot" style="background:#9333ea"></span>access/connectors</div></div></section>
<aside class="card side"><h2>Questions by timestamp</h2><p class="summary">Click any timestamp to jump there. Add notes if useful, then use “Download responses” to save them.</p><div id="questions"></div><button class="export" id="export">Download responses (.txt)</button></aside></main>
<script>
const data=__PAYLOAD__;const svg=document.getElementById('map'),slider=document.getElementById('slider'),NS='http://www.w3.org/2000/svg';const ca=Math.cos(data.display_rotation_rad),sa=Math.sin(data.display_rotation_rad),rotate=p=>[p[0]*ca-p[1]*sa,p[0]*sa+p[1]*ca],rotateList=points=>points.map(rotate);data.outer=rotateList(data.outer);data.hole=rotateList(data.hole);data.planned_gaps=data.planned_gaps.map(rotateList);data.physical_gaps=data.physical_gaps.map(rotateList);data.segments.forEach(s=>s.points=rotateList(s.points));data.questions.forEach(q=>q.point=rotate(q.point));const all=[...data.outer,...data.hole,...data.segments.flatMap(s=>s.points)];const xs=all.map(p=>p[0]),ys=all.map(p=>p[1]);const minX=Math.min(...xs)-2,maxX=Math.max(...xs)+2,minY=Math.min(...ys)-2,maxY=Math.max(...ys)+2,plotW=824,plotH=664,dataW=maxX-minX,dataH=maxY-minY,metersToPixels=Math.min(plotW/dataW,plotH/dataH),originX=48+(plotW-dataW*metersToPixels)/2,originY=48+(plotH-dataH*metersToPixels)/2;const sx=x=>originX+(x-minX)*metersToPixels,sy=y=>760-originY-(y-minY)*metersToPixels;const fmt=t=>{let m=Math.floor(t/60),s=t-m*60;return String(m).padStart(2,'0')+':'+s.toFixed(1).padStart(4,'0')};function path(points){return points.map((p,i)=>(i?'L':'M')+sx(p[0]).toFixed(1)+','+sy(p[1]).toFixed(1)).join(' ')}function addPath(points,stroke,width,fill='none',opacity=1,dash=''){let p=document.createElementNS(NS,'path');p.setAttribute('d',path(points));p.setAttribute('stroke',stroke);p.setAttribute('stroke-width',width);p.setAttribute('fill',fill);p.setAttribute('opacity',opacity);if(dash)p.setAttribute('stroke-dasharray',dash);svg.appendChild(p);return p}addPath(data.outer,'#15803d',2,'#f0fdf4');addPath(data.hole,'#b91c1c',2,'white');const gap38=data.planned_gaps.map(g=>addPath(g,'#a16207',1,'#facc15',.68));const gap42=data.physical_gaps.map(g=>addPath(g,'#c2410c',1,'#fb923c',.62));gap42.forEach(e=>e.style.display='none');const routeEls=data.segments.map(s=>addPath(s.points,s.color,2.2,'none',.25,s.type==='connector'?'7 4':''));data.questions.forEach((q,i)=>{let g=document.createElementNS(NS,'g');g.style.cursor='pointer';let c=document.createElementNS(NS,'circle');c.setAttribute('cx',sx(q.point[0]));c.setAttribute('cy',sy(q.point[1]));c.setAttribute('r',11);c.setAttribute('fill','#f59e0b');c.setAttribute('stroke','#78350f');c.setAttribute('stroke-width','1.5');let t=document.createElementNS(NS,'text');t.setAttribute('x',sx(q.point[0]));t.setAttribute('y',sy(q.point[1])+4);t.setAttribute('text-anchor','middle');t.setAttribute('font-size','11');t.setAttribute('font-weight','800');t.textContent=i+1;g.append(c,t);g.onclick=()=>jump(q.time);svg.appendChild(g)});const dot=document.createElementNS(NS,'circle');dot.setAttribute('r',7);dot.setAttribute('fill','#111827');dot.setAttribute('stroke','white');dot.setAttribute('stroke-width','2');svg.appendChild(dot);const fullView={x:0,y:0,w:920,h:760};let view={...fullView},drag=null;function setView(){svg.setAttribute('viewBox',`${view.x} ${view.y} ${view.w} ${view.h}`)}function zoomAt(factor,cx=view.x+view.w/2,cy=view.y+view.h/2){const nw=Math.max(92,Math.min(fullView.w,view.w*factor)),nh=nw*fullView.h/fullView.w;const fx=(cx-view.x)/view.w,fy=(cy-view.y)/view.h;view={x:cx-fx*nw,y:cy-fy*nh,w:nw,h:nh};setView()}function eventPoint(e){const r=svg.getBoundingClientRect();return{x:view.x+(e.clientX-r.left)/r.width*view.w,y:view.y+(e.clientY-r.top)/r.height*view.h}}svg.addEventListener('wheel',e=>{e.preventDefault();const p=eventPoint(e);zoomAt(e.deltaY<0?.8:1.25,p.x,p.y)},{passive:false});svg.addEventListener('pointerdown',e=>{drag={p:eventPoint(e),x:view.x,y:view.y};svg.setPointerCapture(e.pointerId)});svg.addEventListener('pointermove',e=>{if(!drag)return;const p=eventPoint(e);view.x=drag.x-(p.x-drag.p.x);view.y=drag.y-(p.y-drag.p.y);setView()});svg.addEventListener('pointerup',()=>drag=null);svg.addEventListener('pointercancel',()=>drag=null);document.getElementById('zoomIn').onclick=()=>zoomAt(.8);document.getElementById('zoomOut').onclick=()=>zoomAt(1.25);document.getElementById('resetView').onclick=()=>{view={...fullView};setView()};document.getElementById('focus').onclick=()=>{const x=+dot.getAttribute('cx'),y=+dot.getAttribute('cy');view={x:x-230,y:y-190,w:460,h:380};setView()};const qbox=document.getElementById('questions');data.questions.forEach((q,i)=>{let d=document.createElement('div');d.className='q';d.id='card-'+q.id;d.innerHTML=`<div class="q-head"><span class="q-title">${i+1}. ${q.title}</span><button class="jump" data-time="${q.time}">${fmt(q.time)}</button></div><p>${q.question}</p><textarea class="answer" data-id="${q.id}" placeholder="Your note for ${q.id}"></textarea>`;qbox.appendChild(d)});document.querySelectorAll('.jump').forEach(b=>b.onclick=()=>jump(+b.dataset.time));const ticks=document.getElementById('ticks');data.questions.forEach(q=>{let t=document.createElement('span');t.className='tick';t.style.left=(q.time/data.total_seconds*100)+'%';t.title=q.id+' · '+fmt(q.time);t.onclick=()=>jump(q.time);ticks.appendChild(t)});let timer=null,last=null,position=0;function locate(time){for(let i=0;i<data.segments.length;i++){let s=data.segments[i];if(time<=s.end+.0001){let local=Math.max(0,time-s.start),target=local,acc=0,pt=s.points[0];for(let j=1;j<s.points.length;j++){let a=s.points[j-1],b=s.points[j],d=Math.hypot(b[0]-a[0],b[1]-a[1]);if(target<=acc+d){let f=d?(target-acc)/d:0;pt=[a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1])];break}acc+=d;pt=b}return{s,i,pt}}}let s=data.segments.at(-1);return{s,i:data.segments.length-1,pt:s.points.at(-1)}}function update(){position=Math.max(0,Math.min(data.total_seconds,position));slider.value=position;let time=position,{s,i,pt}=locate(time);routeEls.forEach((e,n)=>e.setAttribute('opacity',document.getElementById('showAll').checked?(n<i?'.95':n===i?'1':'.18'):(n===i?'1':'0')));dot.setAttribute('cx',sx(pt[0]));dot.setAttribute('cy',sy(pt[1]));document.getElementById('clock').textContent=fmt(time)+' / '+fmt(data.total_seconds);document.getElementById('segment').textContent=s.id+' · '+s.type;document.getElementById('phase').textContent=s.phase;document.getElementById('distance').textContent=time.toFixed(1)+' m';let nearest=null,best=1e9;data.questions.forEach(q=>{let d=Math.abs(q.time-time);if(d<best){best=d;nearest=q}});document.querySelectorAll('.q').forEach(e=>e.classList.remove('active'));if(best<6){let c=document.getElementById('card-'+nearest.id);c.classList.add('active')}}function jump(time){position=time;update()}function frame(now){if(last===null)last=now;let dt=(now-last)/1000,rate=+document.getElementById('rate').value;last=now;position=Math.min(data.total_seconds,position+dt*rate);update();if(position>=data.total_seconds){timer=null;last=null;return}timer=requestAnimationFrame(frame)}document.getElementById('play').onclick=()=>{if(!timer){last=null;timer=requestAnimationFrame(frame)}};document.getElementById('pause').onclick=()=>{if(timer)cancelAnimationFrame(timer);timer=null;last=null};document.getElementById('restart').onclick=()=>{document.getElementById('pause').click();jump(0)};slider.oninput=()=>{position=+slider.value;update()};document.getElementById('show38').onchange=e=>gap38.forEach(x=>x.style.display=e.target.checked?'':'none');document.getElementById('show42').onchange=e=>gap42.forEach(x=>x.style.display=e.target.checked?'':'none');document.getElementById('showAll').onchange=update;document.getElementById('export').onclick=()=>{let lines=[data.status,'AREA-OVER-ROAD Checkpoint 4 replay responses',''];data.questions.forEach((q,i)=>{let a=document.querySelector(`[data-id="${q.id}"]`).value.trim();lines.push(`${i+1}. ${fmt(q.time)} — ${q.title}`,q.question,'Response: '+(a||'[no response]'),'')});let blob=new Blob([lines.join('\n')],{type:'text/plain'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='AREA-OVER-ROAD_checkpoint_4_replay_responses_rev005.txt';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)};update();
</script></body></html>""".replace(
        "lines.join('" + chr(10) + "')",
        "lines.join(String.fromCharCode(10))",
    ).replace("__PAYLOAD__", payload)
    OUTPUT.write_text(html_text, encoding="utf-8")


if __name__ == "__main__":
    main()
