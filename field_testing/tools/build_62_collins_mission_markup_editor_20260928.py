#!/usr/bin/env python3
"""Build a standalone coordinate-preserving markup editor for the field mission."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
SITE = REPO / "field_testing/sites/62_Collins_multi_boundary_20260915"
PACKAGE = SITE / "mission_plans/20260928_outer_perimeter_field_test"
AUDIT = PACKAGE / "generated/62_Collins_outer_perimeter_field_test_1mps_20260928_audit.csv"
RUN = SITE / "runs/20260928_outer_perimeter_field_test/outer_perimeter_20260928_120838.csv"
MANUAL_RUN = SITE / "runs/20260926_phone_manual_perimeter_extension/field_test_20260926_121311.csv"
INVENTORY = SITE / "site_inventory/revisions/rev_003_20260926_REVIEW_ONLY/62_Collins_site_inventory_rev003_REVIEW_ONLY.geojson"
OUT = PACKAGE / "mission_markup_editor/62_Collins_mission_markup_editor.html"


def main():
    with AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        audit = list(csv.DictReader(handle))
    mission = [[round(float(row["east_m"]), 3), round(float(row["north_m"]), 3)] for row in audit]
    phases = [row["phase"] for row in audit]
    origin_lat = float(audit[0]["lat"])
    origin_lon = float(audit[0]["lon"])

    actual = []
    with RUN.open(newline="", encoding="utf-8-sig") as handle:
        for index, row in enumerate(csv.DictReader(handle)):
            if index % 8:
                continue
            try:
                lat, lon = float(row["lat"]), float(row["lon"])
            except (TypeError, ValueError):
                continue
            x = (lon - origin_lon) * 111_320.0 * math.cos(math.radians(origin_lat))
            y = (lat - origin_lat) * 110_540.0
            actual.append([round(x, 3), round(y, 3)])

    def xy(lat, lon):
        return [
            round((lon - origin_lon) * 111_320.0 * math.cos(math.radians(origin_lat)), 3),
            round((lat - origin_lat) * 110_540.0, 3),
        ]

    manual_segments = []
    segment = []
    previous_elapsed = None
    previous_point = None
    with MANUAL_RUN.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            try:
                lat, lon = float(row["lat"]), float(row["lon"])
                speed, elapsed = float(row["speed_mps"]), float(row["elapsed_sec"])
            except (TypeError, ValueError):
                continue
            if speed < 0.05 or row.get("fix_quality") not in {"RTK Fixed", "RTK Float"}:
                continue
            point = xy(lat, lon)
            if previous_elapsed is not None and (elapsed - previous_elapsed > 3.0 or math.dist(point, previous_point) > 2.0):
                if len(segment) > 1:
                    manual_segments.append(segment)
                segment = []
            if previous_point is None or math.dist(point, previous_point) >= 0.12:
                segment.append(point)
                previous_point = point
            previous_elapsed = elapsed
    if len(segment) > 1:
        manual_segments.append(segment)

    inventory = json.loads(INVENTORY.read_text(encoding="utf-8"))
    obstacles = []
    for feature in inventory["features"]:
        props = feature.get("properties", {})
        if props.get("geometry_role") not in {"candidate_mowing_exclusion", "mowing_exclusion"}:
            continue
        geometry = feature["geometry"]
        polygons = [geometry["coordinates"]] if geometry["type"] == "Polygon" else geometry["coordinates"]
        for polygon_number, polygon in enumerate(polygons, 1):
            rings = [[xy(lat, lon) for lon, lat in ring] for ring in polygon]
            outer = rings[0]
            center = [
                round(sum(point[0] for point in outer[:-1]) / max(1, len(outer) - 1), 3),
                round(sum(point[1] for point in outer[:-1]) / max(1, len(outer) - 1), 3),
            ]
            obstacles.append({
                "id": props.get("asset_id", feature.get("id", "obstacle")),
                "part": polygon_number,
                "rings": rings,
                "center": center,
            })

    data = json.dumps({
        "mission": mission,
        "phases": phases,
        "actual": actual,
        "manualSegments": manual_segments,
        "obstacles": obstacles,
        "originLat": origin_lat,
        "originLon": origin_lon,
        "source": "62_Collins_outer_perimeter_field_test_1mps_20260928",
    }, separators=(",", ":"))

    html = TEMPLATE.replace("__EDITOR_DATA__", data)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(html, encoding="utf-8", newline="\n")
    print(json.dumps({
        "output": str(OUT),
        "obstacles": len(obstacles),
        "manual_segments": len(manual_segments),
        "manual_points": sum(len(segment) for segment in manual_segments),
        "mission_points": len(mission),
        "actual_points": len(actual),
    }, indent=2))


TEMPLATE = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>62 Collins mission markup editor</title>
<style>
:root{font-family:system-ui,sans-serif;color-scheme:light dark;--bg:#f5f7fa;--panel:#fff;--text:#17212b;--muted:#607080;--line:#1565c0;--actual:#00897b;--manual:#7b2cbf;--obstacle:#f08c00;--green:#10a35a;--red:#e03131;--grid:#dce2e8;--border:#aab5bf} @media(prefers-color-scheme:dark){:root{--bg:#0f151b;--panel:#17212b;--text:#edf2f7;--muted:#aab8c5;--line:#5ca8ff;--actual:#35d0ba;--manual:#c77dff;--obstacle:#ffc078;--green:#42d982;--red:#ff6b6b;--grid:#33404c;--border:#52606d}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text)}header{padding:10px 14px;background:var(--panel);border-bottom:1px solid var(--border)}h1{font-size:1.15rem;margin:0 0 3px}.sub{font-size:.86rem;color:var(--muted)}.toolbar{display:flex;flex-wrap:wrap;gap:7px;align-items:center;padding:9px 12px;background:var(--panel);border-bottom:1px solid var(--border)}button{font:inherit;font-weight:650;padding:9px 12px;border:1px solid var(--border);border-radius:5px;background:var(--panel);color:var(--text);touch-action:manipulation}button.active{background:var(--text);color:var(--panel)}button.green{border-color:var(--green)}button.red{border-color:var(--red)}label{display:flex;gap:6px;align-items:center;font-size:.86rem}input[type=range]{width:110px}.wrap{position:relative;height:calc(100vh - 145px);min-height:430px;overflow:hidden}.wrap svg{display:block;width:100%;height:100%;background:var(--bg);touch-action:none}.grid{stroke:var(--grid);stroke-width:.08}.axis{stroke:var(--border);stroke-width:.12}.mission{fill:none;stroke:var(--line);stroke-width:.24;vector-effect:non-scaling-stroke}.actual{fill:none;stroke:var(--actual);stroke-width:.16;opacity:.48;vector-effect:non-scaling-stroke}.manual{fill:none;stroke:var(--manual);stroke-width:.18;opacity:.62;vector-effect:non-scaling-stroke}.obstacle{fill:var(--obstacle);fill-opacity:.16;stroke:var(--obstacle);stroke-width:.18;vector-effect:non-scaling-stroke}.obstacle-label{fill:var(--text);font:500 1.1px system-ui,sans-serif;paint-order:stroke;stroke:var(--bg);stroke-width:.3px;stroke-linejoin:round;pointer-events:none}.proposal{fill:none;stroke:var(--green);stroke-width:.45;stroke-linecap:round;stroke-linejoin:round;vector-effect:non-scaling-stroke}.attention{fill:none;stroke:var(--red);stroke-width:.55;stroke-linecap:round;stroke-linejoin:round;vector-effect:non-scaling-stroke}.cursor{fill:none;stroke:var(--text);stroke-width:.12;stroke-dasharray:.35 .25;pointer-events:none}.readout{position:absolute;left:10px;bottom:10px;background:var(--panel);border:1px solid var(--border);padding:6px 8px;border-radius:4px;font:12px ui-monospace,monospace}.legend{display:flex;gap:14px;flex-wrap:wrap;margin-left:auto;font-size:.82rem;color:var(--muted)}.sw{display:inline-block;width:18px;height:3px;vertical-align:middle;margin-right:4px}.help{padding:6px 12px;font-size:.82rem;color:var(--muted);background:var(--panel);border-top:1px solid var(--border)}@media(max-width:720px){.legend{width:100%;margin-left:0}.wrap{height:65vh}}
</style></head><body>
<header><h1>62 Collins mission markup editor</h1><div class="sub">Coordinate-preserving review of the revised perimeter mission and completed field track</div></header>
<div class="toolbar" role="toolbar" aria-label="Mission editing tools">
  <button id="pan" class="active">Pan / zoom</button><button id="draw" class="green">Draw proposed path</button><button id="mark" class="red">Mark missed area</button><button id="erase">Erase mission</button>
  <label>Brush <input id="brush" type="range" min="0.3" max="4" step="0.1" value="1.2"><span id="brushValue">1.2 m</span></label>
  <button id="undo">Undo</button><button id="redo">Redo</button><button id="fit">Fit all</button><button id="import">Import edits</button><input id="importFile" type="file" accept=".json,application/json" hidden><button id="export">Export edits</button><button id="png">Save PNG</button>
  <label><input id="obstacleToggle" type="checkbox" checked> Obstacles</label><label><input id="manualToggle" type="checkbox" checked> Sept 26 manual path</label><label><input id="actualToggle" type="checkbox" checked> Sept 28 mission track</label>
  <div class="legend"><span><i class="sw" style="background:var(--line)"></i>Mission</span><span><i class="sw" style="background:var(--actual)"></i>Sept 28</span><span><i class="sw" style="background:var(--manual)"></i>Sept 26 manual</span><span><i class="sw" style="background:var(--obstacle)"></i>Obstacle</span><span><i class="sw" style="background:var(--green)"></i>Proposed</span><span><i class="sw" style="background:var(--red)"></i>Missed area</span></div>
</div>
<div class="wrap"><svg id="map" role="img" aria-label="Editable mission map"><g id="viewport"><g id="grid"></g><g id="obstacles"></g><g id="manualPaths"></g><path id="actualPath" class="actual"></path><g id="missionPaths"></g><g id="proposals"></g><g id="attention"></g><circle id="cursor" class="cursor" r="1.2" visibility="hidden"></circle></g></svg><div id="readout" class="readout">Pan / zoom</div></div>
<div class="help">Mouse wheel: zoom. Pan tool: drag. Draw and mark tools: paint lines. Eraser: paint over blue mission segments. Exported JSON preserves local coordinates, latitude/longitude, erased waypoint ranges, and annotations.</div>
<script>
const DATA=__EDITOR_DATA__,svg=document.getElementById('map'),vp=document.getElementById('viewport'),missionLayer=document.getElementById('missionPaths'),proposalLayer=document.getElementById('proposals'),attentionLayer=document.getElementById('attention'),cursor=document.getElementById('cursor'),readout=document.getElementById('readout');
let tool='pan',drawing=false,lastClient=null,currentStroke=null,scale=1,tx=0,ty=0,history=[],future=[],erased=new Set(),proposals=[],attention=[];
const NS='http://www.w3.org/2000/svg',bounds=DATA.mission.reduce((b,p)=>[Math.min(b[0],p[0]),Math.min(b[1],p[1]),Math.max(b[2],p[0]),Math.max(b[3],p[1])],[Infinity,Infinity,-Infinity,-Infinity]);
const pathD=pts=>pts.map((p,i)=>(i?'L':'M')+p[0].toFixed(3)+' '+(-p[1]).toFixed(3)).join(' ');
document.getElementById('actualPath').setAttribute('d',pathD(DATA.actual));
const manualLayer=document.getElementById('manualPaths'),obstacleLayer=document.getElementById('obstacles');
for(const segment of DATA.manualSegments){const p=document.createElementNS(NS,'path');p.setAttribute('class','manual');p.setAttribute('d',pathD(segment));manualLayer.append(p)}
for(const obstacle of DATA.obstacles){for(const ring of obstacle.rings){const p=document.createElementNS(NS,'path');p.setAttribute('class','obstacle');p.setAttribute('d',pathD(ring)+' Z');obstacleLayer.append(p)}const t=document.createElementNS(NS,'text');t.setAttribute('class','obstacle-label');t.setAttribute('x',obstacle.center[0]);t.setAttribute('y',-obstacle.center[1]);t.setAttribute('text-anchor','middle');t.textContent=obstacle.id;obstacleLayer.append(t)}
function setTransform(){vp.setAttribute('transform',`translate(${tx} ${ty}) scale(${scale})`)}
function fit(){const r=svg.getBoundingClientRect(),w=bounds[2]-bounds[0],h=bounds[3]-bounds[1],pad=30;scale=Math.min((r.width-2*pad)/w,(r.height-2*pad)/h);tx=pad-bounds[0]*scale;ty=pad+bounds[3]*scale;setTransform()}
function local(e){const r=svg.getBoundingClientRect();return[(e.clientX-r.left-tx)/scale,-(e.clientY-r.top-ty)/scale]}
function renderMission(){missionLayer.replaceChildren();let part=[];const flush=()=>{if(part.length>1){const p=document.createElementNS(NS,'path');p.setAttribute('class','mission');p.setAttribute('d',pathD(part));missionLayer.append(p)}part=[]};DATA.mission.forEach((p,i)=>{if(erased.has(i)){flush()}else part.push(p)});flush()}
function renderStrokes(){proposalLayer.replaceChildren();attentionLayer.replaceChildren();for(const [items,layer,cls] of [[proposals,proposalLayer,'proposal'],[attention,attentionLayer,'attention']])for(const points of items){const p=document.createElementNS(NS,'path');p.setAttribute('class',cls);p.setAttribute('d',pathD(points));layer.append(p)}}
function snapshot(){history.push(JSON.stringify({erased:[...erased],proposals,attention}));if(history.length>100)history.shift();future=[]}
function restore(s){const v=JSON.parse(s);erased=new Set(v.erased);proposals=v.proposals;attention=v.attention;renderMission();renderStrokes()}
function setTool(name){tool=name;document.querySelectorAll('.toolbar button').forEach(b=>b.classList.remove('active'));document.getElementById(name).classList.add('active');readout.textContent=name==='pan'?'Pan / zoom':name==='draw'?'Draw proposed path':name==='mark'?'Mark missed area':'Erase mission'}
for(const id of ['pan','draw','mark','erase'])document.getElementById(id).onclick=()=>setTool(id);
document.getElementById('brush').oninput=e=>{document.getElementById('brushValue').textContent=Number(e.target.value).toFixed(1)+' m';cursor.setAttribute('r',e.target.value)};
svg.addEventListener('wheel',e=>{e.preventDefault();const r=svg.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,k=e.deltaY<0?1.15:1/1.15,n=Math.max(.5,Math.min(30,scale*k));tx=mx-(mx-tx)*n/scale;ty=my-(my-ty)*n/scale;scale=n;setTransform()},{passive:false});
svg.addEventListener('pointerdown',e=>{svg.setPointerCapture(e.pointerId);drawing=true;lastClient=[e.clientX,e.clientY];if(tool!=='pan'){snapshot();const p=local(e);if(tool==='draw'){currentStroke=[p];proposals.push(currentStroke)}else if(tool==='mark'){currentStroke=[p];attention.push(currentStroke)}else eraseAt(p);renderStrokes()}});
svg.addEventListener('pointermove',e=>{const p=local(e);cursor.setAttribute('cx',p[0]);cursor.setAttribute('cy',-p[1]);cursor.setAttribute('visibility',tool==='pan'?'hidden':'visible');readout.textContent=`${tool} · east ${p[0].toFixed(2)} m · north ${p[1].toFixed(2)} m`;if(!drawing)return;if(tool==='pan'){tx+=e.clientX-lastClient[0];ty+=e.clientY-lastClient[1];lastClient=[e.clientX,e.clientY];setTransform()}else if(tool==='erase'){eraseAt(p)}else if(!currentStroke.length||Math.hypot(p[0]-currentStroke.at(-1)[0],p[1]-currentStroke.at(-1)[1])>.12){currentStroke.push(p);renderStrokes()}});
svg.addEventListener('pointerup',()=>{drawing=false;currentStroke=null});svg.addEventListener('pointerleave',()=>cursor.setAttribute('visibility','hidden'));
function eraseAt(p){const radius=Number(document.getElementById('brush').value);DATA.mission.forEach((q,i)=>{if(Math.hypot(q[0]-p[0],q[1]-p[1])<=radius)erased.add(i)});renderMission()}
document.getElementById('undo').onclick=()=>{if(!history.length)return;future.push(JSON.stringify({erased:[...erased],proposals,attention}));restore(history.pop())};
document.getElementById('redo').onclick=()=>{if(!future.length)return;history.push(JSON.stringify({erased:[...erased],proposals,attention}));restore(future.pop())};document.getElementById('fit').onclick=fit;
document.getElementById('actualToggle').onchange=e=>document.getElementById('actualPath').style.display=e.target.checked?'':'none';
document.getElementById('manualToggle').onchange=e=>manualLayer.style.display=e.target.checked?'':'none';document.getElementById('obstacleToggle').onchange=e=>obstacleLayer.style.display=e.target.checked?'':'none';
const ll=p=>[DATA.originLon+p[0]/(111320*Math.cos(DATA.originLat*Math.PI/180)),DATA.originLat+p[1]/110540];
function erasedRanges(){const a=[...erased].sort((x,y)=>x-y),out=[];for(const n of a){const last=out.at(-1);if(last&&n===last[1]+1)last[1]=n;else out.push([n,n])}return out.map(r=>[r[0]+1,r[1]+1])}
const importFile=document.getElementById('importFile');
document.getElementById('import').onclick=()=>{importFile.value='';importFile.click()};
importFile.onchange=async()=>{const file=importFile.files[0];if(!file)return;try{const payload=JSON.parse(await file.text());if(payload.format!=='tractor2025-mission-markup-v1')throw new Error('This is not a tractor2025 mission-markup edits file.');if(payload.source!==DATA.source&&!confirm(`These edits were made for ${payload.source||'an unknown mission'}, not ${DATA.source}. Import them anyway?`))return;snapshot();const imported=new Set();for(const range of payload.erasedWaypointRanges||[]){const first=Math.max(1,Math.round(Number(range[0]))),last=Math.min(DATA.mission.length,Math.round(Number(range[1])));for(let n=first;n<=last;n++)imported.add(n-1)}const paths=items=>(items||[]).map(item=>item.localEastNorth).filter(points=>Array.isArray(points)&&points.length).map(points=>points.map(p=>[Number(p[0]),Number(p[1])]).filter(p=>Number.isFinite(p[0])&&Number.isFinite(p[1]))).filter(points=>points.length);erased=imported;proposals=paths(payload.proposedPaths);attention=paths(payload.missedAreaMarks);renderMission();renderStrokes();readout.textContent=`Imported ${file.name}: ${proposals.length} proposed path(s), ${attention.length} missed-area mark(s)`}catch(err){alert('Could not import edits: '+err.message)}};
document.getElementById('export').onclick=()=>{const payload={format:'tractor2025-mission-markup-v1',source:DATA.source,origin:{lat:DATA.originLat,lon:DATA.originLon},erasedWaypointRanges:erasedRanges(),proposedPaths:proposals.map((p,i)=>({id:`proposal-${i+1}`,localEastNorth:p,geojsonLonLat:p.map(ll)})),missedAreaMarks:attention.map((p,i)=>({id:`attention-${i+1}`,localEastNorth:p,geojsonLonLat:p.map(ll)}))};download(new Blob([JSON.stringify(payload,null,2)],{type:'application/json'}),'62_Collins_mission_edits.json')};
function cssColor(name){return getComputedStyle(document.documentElement).getPropertyValue(name).trim()}
function canvasLine(c,points,color,width,alpha=1){if(points.length<2)return;c.save();c.beginPath();c.moveTo(points[0][0],-points[0][1]);for(let i=1;i<points.length;i++)c.lineTo(points[i][0],-points[i][1]);c.strokeStyle=color;c.lineWidth=width/scale;c.globalAlpha=alpha;c.lineCap='round';c.lineJoin='round';c.stroke();c.restore()}
function retainedMissionSegments(){const result=[];let part=[];const flush=()=>{if(part.length>1)result.push(part);part=[]};DATA.mission.forEach((p,i)=>{if(erased.has(i))flush();else part.push(p)});flush();return result}
document.getElementById('png').onclick=()=>{const r=svg.getBoundingClientRect(),factor=2,canvas=document.createElement('canvas'),c=canvas.getContext('2d');canvas.width=Math.max(1,Math.round(r.width*factor));canvas.height=Math.max(1,Math.round(r.height*factor));c.scale(factor,factor);c.fillStyle=cssColor('--bg');c.fillRect(0,0,r.width,r.height);c.save();c.translate(tx,ty);c.scale(scale,scale);
  if(document.getElementById('obstacleToggle').checked){for(const obstacle of DATA.obstacles){for(const ring of obstacle.rings){if(ring.length<3)continue;c.beginPath();c.moveTo(ring[0][0],-ring[0][1]);for(let i=1;i<ring.length;i++)c.lineTo(ring[i][0],-ring[i][1]);c.closePath();c.save();c.fillStyle=cssColor('--obstacle');c.globalAlpha=.16;c.fill();c.restore();c.strokeStyle=cssColor('--obstacle');c.lineWidth=1.3/scale;c.stroke()}c.save();c.font=`500 ${11/scale}px system-ui,sans-serif`;c.textAlign='center';c.textBaseline='middle';c.lineWidth=3/scale;c.strokeStyle=cssColor('--bg');c.strokeText(obstacle.id,obstacle.center[0],-obstacle.center[1]);c.fillStyle=cssColor('--text');c.fillText(obstacle.id,obstacle.center[0],-obstacle.center[1]);c.restore()}}
  if(document.getElementById('manualToggle').checked)for(const segment of DATA.manualSegments)canvasLine(c,segment,cssColor('--manual'),1.3,.62);
  if(document.getElementById('actualToggle').checked)canvasLine(c,DATA.actual,cssColor('--actual'),1.2,.48);
  for(const segment of retainedMissionSegments())canvasLine(c,segment,cssColor('--line'),1.6);
  for(const points of proposals)canvasLine(c,points,cssColor('--green'),3);
  for(const points of attention)canvasLine(c,points,cssColor('--red'),3.5);
  c.restore();canvas.toBlob(blob=>{if(blob)download(blob,'62_Collins_mission_markup.png');else alert('The browser could not create the PNG.')},'image/png')};
function download(blob,name){const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)}
renderMission();renderStrokes();requestAnimationFrame(fit);window.addEventListener('resize',fit);
</script></body></html>'''


if __name__ == "__main__":
    main()
