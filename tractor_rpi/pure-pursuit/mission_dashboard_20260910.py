#!/usr/bin/env python3
"""Local web dashboard for starting and monitoring the backyard mission."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import secrets
import signal
import socket
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


REPO = Path(__file__).resolve().parents[2]
PACKAGE = (
    REPO / "field_testing" / "sites" / "62_Collins_polygon_1" / "mission_plans"
    / "20260909_complete_back_yard"
)
MISSION = PACKAGE / "62_Collins_complete_back_yard_1mps_20260909.txt"
AUDIT = PACKAGE / "62_Collins_complete_back_yard_1mps_20260909_audit.csv"
LAUNCHER = PACKAGE / "run_complete_back_yard_mission_20260909.sh"
CONTROL_PORT = 6011
TELEMETRY_PORT = 6012
STATUS_PORT = 6003
CMD_VEL_PORT = 6004
GPS_DASHBOARD_PORT = 6013
TRACTOR_LOCAL_IP = "192.168.1.151"
TRACTOR_ZEROTIER_IP = "192.168.193.76"
EXPECTED_CONFIRMATION = "RUN COMPLETE BACK YARD BLADES OFF"


HTML = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Tractor01 mission control</title>
<style>
:root{color-scheme:dark;font-family:system-ui,sans-serif}*{box-sizing:border-box}body{margin:0;background:#101418;color:#e8edf2}main{max-width:1500px;margin:auto;padding:14px}h1{margin:.2rem 0;font-size:1.5rem}.toolbar{display:flex;gap:10px;align-items:center;flex-wrap:wrap;padding:12px 0}.button{border:0;border-radius:6px;padding:12px 18px;font-weight:700;cursor:pointer}.button:disabled{opacity:.4;cursor:not-allowed}.start{background:#2fb344;color:#fff}.pause{background:#e03131;color:#fff}.resume{background:#1971c2;color:#fff}.messages{background:#495057;color:#fff}.badge{padding:7px 10px;border-radius:999px;background:#343a40;font-weight:700}.ok{background:#19713c}.warn{background:#9c640c}.bad{background:#9b2226}.layout{display:grid;grid-template-columns:minmax(0,2fr) minmax(330px,1fr);gap:14px}svg{width:100%;height:min(78vh,850px);background:#182028;border:1px solid #52606d}.side{min-width:0}.facts{display:grid;grid-template-columns:1fr 1fr;gap:1px;background:#52606d;border:1px solid #52606d}.fact{background:#182028;padding:8px;min-height:58px}.fact span{display:block;color:#9fb0bf;font-size:.78rem}.fact b{font-size:.96rem}.output{height:180px;overflow:auto;white-space:pre-wrap;background:#080b0d;border:1px solid #52606d;padding:8px;font:12px ui-monospace,monospace;margin-top:10px}.note{color:#b8c5cf;margin:.3rem 0 0}.mission{fill:none;stroke:#708090;stroke-width:1}.trail{fill:none;stroke:#35d0ba;stroke-width:2}.to-target{stroke:#ff5d8f;stroke-width:1.7;stroke-dasharray:5 3}.to-start{stroke:#ff922b;stroke-width:2.5;stroke-dasharray:8 4}.start-label{fill:#fff3bf;font:bold 16px system-ui,sans-serif;paint-order:stroke;stroke:#101418;stroke-width:4px}.heading{stroke:#ffd43b;stroke-width:2}.tractor{fill:#ffd43b;stroke:#111;stroke-width:1}.target{fill:#ff5d8f}.startpoint{fill:#2fb344}.endpoint{fill:#e03131}.progressbar{width:100%;height:9px;background:#343a40;border-radius:5px;overflow:hidden}.progressbar div{height:100%;background:#35d0ba;width:0}.safety{border-left:5px solid #e03131;background:#291719;padding:9px 12px;margin-bottom:10px}@media(max-width:850px){.layout{grid-template-columns:1fr}svg{height:60vh}.facts{grid-template-columns:1fr 1fr}}
</style></head><body><main>
<h1>Tractor01 — complete back yard mission</h1>
<p class="note">Live mission plan, actual tractor position, controller target, and drivetrain telemetry.</p>
<div class="toolbar"><button id="start" class="button start">START MISSION</button><button id="pause" class="button pause" disabled>PAUSE</button><button id="clearPause" class="button resume" disabled>CLEAR PAUSE</button><button id="messages" class="button messages">OPEN MESSAGES</button><span id="state" class="badge">CONNECTING</span><span id="age" class="badge">No telemetry</span></div>
<div class="safety"><b>Keep the handheld with you.</b> After a browser Pause: select handheld Pause, press CLEAR PAUSE, confirm HANDHELD PAUSE, then select Auto.</div>
<div class="progressbar"><div id="progress"></div></div>
<div class="layout"><svg id="map" viewBox="0 0 900 760" role="img" aria-label="Planned mission and live tractor path"></svg><div class="side"><div class="facts" id="facts"></div><div id="output" class="output"></div></div></div>
</main><script>
const key=new URLSearchParams(location.search).get('key')||'';
const headers={'Content-Type':'application/json','X-Operator-Key':key};
const svg=document.getElementById('map'),facts=document.getElementById('facts'),stateEl=document.getElementById('state'),ageEl=document.getElementById('age'),out=document.getElementById('output');
const startBtn=document.getElementById('start'),pauseBtn=document.getElementById('pause'),clearPauseBtn=document.getElementById('clearPause'),messagesBtn=document.getElementById('messages'),progress=document.getElementById('progress');
const TRAIL_SECONDS=30;let DATA=null,trailPoints=[];const NS='http://www.w3.org/2000/svg';
const el=(n,a={})=>{const x=document.createElementNS(NS,n);for(const[k,v]of Object.entries(a))x.setAttribute(k,v);return x};
let sx=x=>x,sy=y=>y,trail,tractor,target,targetLine,startLine,startLabel,heading;
function pathD(points){return points.map((p,i)=>(i?'L':'M')+sx(p.x).toFixed(1)+' '+sy(p.y).toFixed(1)).join(' ')}
function setupMap(){const xs=DATA.path.map(p=>p.x),ys=DATA.path.map(p=>p.y),pad=15,minX=Math.min(...xs)-pad,maxX=Math.max(...xs)+pad,minY=Math.min(...ys)-pad,maxY=Math.max(...ys)+pad;const scale=Math.min(830/(maxX-minX),700/(maxY-minY));const ox=35+(830-(maxX-minX)*scale)/2,oy=725-(700-(maxY-minY)*scale)/2;sx=x=>ox+(x-minX)*scale;sy=y=>oy-(y-minY)*scale;const mission=el('path',{class:'mission',d:pathD(DATA.path)});trail=el('path',{class:'trail'});targetLine=el('line',{class:'to-target',visibility:'hidden'});startLine=el('line',{class:'to-start',visibility:'hidden'});startLabel=el('text',{class:'start-label','text-anchor':'middle'});heading=el('line',{class:'heading',visibility:'hidden'});tractor=el('circle',{class:'tractor',r:7,visibility:'hidden'});target=el('circle',{class:'target',r:5,visibility:'hidden'});const s=DATA.path[0],e=DATA.path.at(-1);svg.append(mission,trail,el('circle',{class:'startpoint',cx:sx(s.x),cy:sy(s.y),r:6}),el('rect',{class:'endpoint',x:sx(e.x)-5,y:sy(e.y)-5,width:10,height:10}),targetLine,startLine,startLabel,heading,tractor,target)}
function fmt(v,d=2){return v===null||v===undefined||v===''?'—':Number(v).toFixed(d)}function fact(l,v){return `<div class="fact"><span>${l}</span><b>${v}</b></div>`}function mode(v){return Number(v)===2?'Pause':Number(v)===1?'Manual':Number(v)===0?'Auto':'—'}
async function api(path,method='GET',body=null){const r=await fetch(path,{method,headers,body:body?JSON.stringify(body):null});const j=await r.json();if(!r.ok)throw Error(j.error||r.statusText);return j}
async function command(name){try{if(name==='start'&&!confirm('Start the complete 40-minute blades-off mission? Keep the handheld in Pause until the controller is ready.'))return;const body=name==='start'?{confirmation:'RUN COMPLETE BACK YARD BLADES OFF'}:{};await api('/api/'+name,'POST',body)}catch(e){alert(e.message)}}
startBtn.onclick=()=>command('start');pauseBtn.onclick=()=>command('pause');clearPauseBtn.onclick=()=>command('clear-pause');
messagesBtn.onclick=()=>window.open('/messages?key='+encodeURIComponent(key),'_blank');
function finite(v){return v!==null&&v!==undefined&&v!==''&&Number.isFinite(Number(v))}
function axisText(value,positive,negative){if(Math.abs(value)<.1)return '';return Math.abs(value).toFixed(1)+' m '+(value>0?positive:negative)}
function draw(s){
  const c=s.controller||{},b=s.bridge||{},g=s.gps||{},st=b.steering||{},tr=b.transmission||{};
  const active=['STARTING','RUNNING','PAUSED','WAITING'].includes(s.process_state);
  const controllerFresh=s.controller_age_s!=null&&s.controller_age_s<1;
  const bridgeFresh=s.bridge_age_s!=null&&s.bridge_age_s<1;
  const gpsFresh=s.gps_age_s!=null&&s.gps_age_s<1;
  const softwarePaused=c.software_paused===true;
  const handheldPaused=bridgeFresh&&Number(st.mode)===2&&Number(tr.mode)===2&&st.state==='PAUSE';
  const shownState=softwarePaused?'SOFTWARE PAUSE':handheldPaused?'HANDHELD PAUSE':s.process_state;
  const pauseSource=softwarePaused?'Dashboard software hold':handheldPaused?'Handheld Pause':'None';
  startBtn.disabled=active;
  pauseBtn.disabled=!active||!controllerFresh||softwarePaused;
  clearPauseBtn.disabled=!active||!controllerFresh||!softwarePaused||!handheldPaused;
  clearPauseBtn.title=softwarePaused&&!handheldPaused?'Put the handheld in Pause before clearing software Pause':'';
  stateEl.textContent=shownState;
  stateEl.className='badge '+(shownState==='RUNNING'?'ok':shownState.includes('PAUSE')||shownState==='WAITING'?'warn':shownState==='FAILED'?'bad':'');
  ageEl.textContent=s.controller_age_s==null?'No controller telemetry':s.controller_age_s.toFixed(1)+' s telemetry age';
  ageEl.className='badge '+(controllerFresh?'ok':'bad');
  const idx=Number(c.waypoint_idx||0),total=Number(c.waypoints_total||DATA.path.length);
  progress.style.width=(100*Math.min(1,idx/Math.max(1,total))).toFixed(1)+'%';
  let p=null,positionHeading=null;
  if(controllerFresh&&finite(c.pos_x_m)&&finite(c.pos_y_m)){p={x:Number(c.pos_x_m),y:Number(c.pos_y_m)};positionHeading=c.heading_compass_deg}
  else if(gpsFresh&&finite(g.lat)&&finite(g.lon)){p={x:(Number(g.lon)-DATA.origin_lon)*DATA.lon_scale,y:(Number(g.lat)-DATA.origin_lat)*DATA.lat_scale};positionHeading=g.heading_deg}
  let startDistance=null,startDirections='—';
  if(p){
    const now=Date.now()/1000;p.t=now;const last=trailPoints.at(-1);
    if(!last||Math.hypot(p.x-last.x,p.y-last.y)>.03||now-last.t>=1)trailPoints.push(p);
    trailPoints=trailPoints.filter(q=>now-q.t<=TRAIL_SECONDS);
    trail.setAttribute('d',pathD(trailPoints));
    tractor.setAttribute('visibility','visible');
    tractor.setAttribute('cx',sx(p.x));tractor.setAttribute('cy',sy(p.y));
    const missionStart=DATA.path[0],toStartX=missionStart.x-p.x,toStartY=missionStart.y-p.y;
    startDistance=Math.hypot(toStartX,toStartY);
    startDirections=[axisText(toStartX,'east','west'),axisText(toStartY,'north','south')].filter(Boolean).join(', ')||'at start';
    if(!controllerFresh){
      startLine.setAttribute('visibility','visible');
      startLine.setAttribute('x1',sx(p.x));startLine.setAttribute('y1',sy(p.y));startLine.setAttribute('x2',sx(missionStart.x));startLine.setAttribute('y2',sy(missionStart.y));
      startLabel.setAttribute('x',(sx(p.x)+sx(missionStart.x))/2);startLabel.setAttribute('y',(sy(p.y)+sy(missionStart.y))/2-8);startLabel.textContent=startDistance.toFixed(1)+' m to start';
    }else{startLine.setAttribute('visibility','hidden');startLabel.textContent=''}
    if(controllerFresh&&finite(c.target_x_m)&&finite(c.target_y_m)){
      const q={x:Number(c.target_x_m),y:Number(c.target_y_m)};
      target.setAttribute('visibility','visible');targetLine.setAttribute('visibility','visible');
      target.setAttribute('cx',sx(q.x));target.setAttribute('cy',sy(q.y));
      targetLine.setAttribute('x1',sx(p.x));targetLine.setAttribute('y1',sy(p.y));targetLine.setAttribute('x2',sx(q.x));targetLine.setAttribute('y2',sy(q.y));
    }
    if(finite(positionHeading)){const h=(90-Number(positionHeading))*Math.PI/180;heading.setAttribute('visibility','visible');heading.setAttribute('x1',sx(p.x));heading.setAttribute('y1',sy(p.y));heading.setAttribute('x2',sx(p.x+2*Math.cos(h)));heading.setAttribute('y2',sy(p.y+2*Math.sin(h)))}
  }
  const phase=(DATA.phases[idx]||'—').replaceAll('_',' ');
  facts.innerHTML=fact('Distance to mission start',startDistance==null?'—':startDistance.toFixed(2)+' m')+fact('Drive toward start',startDirections)+fact('Mission phase',phase)+fact('Progress',idx+' / '+total+' ('+fmt(100*idx/Math.max(1,total),1)+'%)')+fact('Pause source',pauseSource)+fact('Controller state',c.controller_state||s.process_state)+fact('Wait reason',c.wait_reason||'—')+fact('Target / actual speed',fmt(c.speed_cmd_mps)+' / '+fmt(c.actual_speed_mps)+' m/s')+fact('Cross-track / lateral yt',fmt(c.cross_track_err_m,3)+' / '+fmt(c.yt_m,3)+' m')+fact('Target lookahead',fmt(c.lookahead_dist_m)+' m')+fact('Heading',fmt(positionHeading,1)+'°')+fact('Steering command',fmt(c.delta_deg,1)+'° / '+fmt(c.steer_normalized))+fact('Steering target / actual',(st.setpoint??'—')+' / '+(st.current??'—'))+fact('Steering error / PWM',(st.error??'—')+' / '+(st.pwm??'—'))+fact('JRK target / feedback',(tr.target??'—')+' / '+(tr.current??'—'))+fact('JRK motor current',tr.motor_current_mA==null?'—':tr.motor_current_mA+' mA')+fact('Radio / steering state',(b.radio?.signal||'—')+' / '+(st.state||'—'))+fact('Handheld modes',mode(st.mode)+' steering / '+mode(tr.mode)+' transmission')+fact('GPS / heading',(c.fix_quality||g.fix_quality||'—')+' / '+(String(c.head_valid??g.headValid).toLowerCase()==='true'?'valid':'invalid'));
  out.textContent=(s.output||[]).join('\n');out.scrollTop=out.scrollHeight;
}
async function poll(){try{const s=await api('/api/state');draw(s)}catch(e){stateEl.textContent='DISCONNECTED';stateEl.className='badge bad'}setTimeout(poll,250)}
(async()=>{try{DATA=await api('/api/mission');setupMap();poll()}catch(e){document.body.innerHTML='<main><h1>Dashboard access failed</h1><p>'+e.message+'</p></main>'}})();
</script></body></html>'''


MESSAGES_HTML = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Tractor01 mission messages</title><style>
:root{color-scheme:dark;font-family:system-ui,sans-serif}body{margin:0;background:#101418;color:#e8edf2}main{padding:18px;max-width:1400px;margin:auto}.toolbar{display:flex;gap:10px;align-items:center;margin:12px 0}.button{border:0;border-radius:6px;padding:11px 17px;background:#1971c2;color:white;font-weight:700;cursor:pointer}#status{color:#b8c5cf}pre{min-height:70vh;max-height:82vh;overflow:auto;white-space:pre-wrap;user-select:text;background:#080b0d;border:1px solid #52606d;padding:14px;font:15px/1.45 ui-monospace,monospace}
</style></head><body><main><h1>Tractor01 mission messages</h1><div class="toolbar"><button id="copy" class="button">COPY ALL MESSAGES</button><span id="status">Connecting…</span></div><pre id="output">Waiting for messages…</pre></main><script>
const key=new URLSearchParams(location.search).get('key')||'',out=document.getElementById('output'),statusEl=document.getElementById('status');
async function poll(){try{const r=await fetch('/api/state',{headers:{'X-Operator-Key':key}}),s=await r.json();if(!r.ok)throw Error(s.error||r.statusText);const text=(s.output||[]).join('\n')||'No mission messages yet.';if(out.textContent!==text){out.textContent=text;out.scrollTop=out.scrollHeight}statusEl.textContent='State: '+s.process_state}catch(e){statusEl.textContent='Disconnected: '+e.message}setTimeout(poll,500)}
document.getElementById('copy').onclick=async()=>{const text=out.textContent;try{if(!navigator.clipboard)throw Error('clipboard unavailable');await navigator.clipboard.writeText(text)}catch(e){const area=document.createElement('textarea');area.value=text;area.style.position='fixed';area.style.opacity='0';document.body.appendChild(area);area.select();document.execCommand('copy');area.remove()}statusEl.textContent='Messages copied'};
poll();
</script></body></html>'''


def json_safe(value):
    """Replace non-finite telemetry floats with JSON null, recursively."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


class MissionState:
    def __init__(self):
        self.lock = threading.Lock()
        self.controller = {}
        self.controller_time = 0.0
        self.bridge = {}
        self.bridge_time = 0.0
        self.gps = {}
        self.gps_time = 0.0
        self.process = None
        self.process_state = "READY"
        self.output = deque(maxlen=500)

    def snapshot(self):
        with self.lock:
            now = time.time()
            return {
                "controller": dict(self.controller),
                "controller_age_s": None if not self.controller_time else now - self.controller_time,
                "bridge": dict(self.bridge),
                "bridge_age_s": None if not self.bridge_time else now - self.bridge_time,
                "gps": dict(self.gps),
                "gps_age_s": None if not self.gps_time else now - self.gps_time,
                "process_state": self.process_state,
                "output": list(self.output),
            }


def udp_listener(state, port, kind):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try: sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError: pass
    sock.bind(("", port)); sock.settimeout(0.5)
    while True:
        try: data, _addr = sock.recvfrom(65535)
        except socket.timeout: continue
        try: message = json.loads(data.decode("utf-8"))
        except (ValueError, UnicodeDecodeError): continue
        with state.lock:
            if kind == "controller":
                state.controller = message; state.controller_time = time.time()
                if message.get("software_paused"):
                    state.process_state = "PAUSED"
                elif state.process_state not in ("COMPLETED", "FAILED"):
                    state.process_state = message.get("controller_state", "RUNNING")
            elif kind == "bridge":
                state.bridge = message; state.bridge_time = time.time()
            else:
                state.gps = message; state.gps_time = time.time()


def send_udp(port, payload, repeats=1):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    encoded = json.dumps(payload).encode("utf-8")
    for _ in range(repeats):
        sock.sendto(encoded, ("127.0.0.1", port)); time.sleep(0.02)
    sock.close()


def process_output(state, process):
    assert process.stdout is not None
    for line in process.stdout:
        with state.lock: state.output.append(line.rstrip())


def process_waiter(state, process):
    code = process.wait()
    with state.lock:
        state.process_state = "COMPLETED" if code == 0 else "FAILED"
        state.output.append(f"Mission launcher exited with status {code}.")
        state.process = None


def safe_to_start(state):
    snap = state.snapshot()
    if snap["bridge_age_s"] is None or snap["bridge_age_s"] > 1.0:
        return False, "No fresh Teensy status; keep tractor01 services running"
    steering = snap["bridge"].get("steering", {})
    transmission = snap["bridge"].get("transmission", {})
    try:
        steering_mode = int(steering.get("mode", -1))
        transmission_mode = int(transmission.get("mode", -1))
    except (TypeError, ValueError):
        return False, "Teensy mode telemetry is incomplete"
    if steering_mode != 2 or transmission_mode != 2:
        return False, "Put the handheld in Pause before starting"
    if steering.get("state") != "PAUSE":
        return False, f"Steering state is {steering.get('state')!r}, not PAUSE"
    return True, ""


def start_mission(state):
    ok, reason = safe_to_start(state)
    if not ok: raise RuntimeError(reason)
    with state.lock:
        if state.process is not None: raise RuntimeError("A mission is already active")
        state.controller = {}; state.controller_time = 0.0; state.output.clear()
        state.process_state = "STARTING"
        process = subprocess.Popen(
            ["bash", str(LAUNCHER), "--dashboard"], cwd=str(REPO),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            bufsize=1, start_new_session=True,
        )
        state.process = process
    threading.Thread(target=process_output, args=(state, process), daemon=True).start()
    threading.Thread(target=process_waiter, args=(state, process), daemon=True).start()


def pause_mission(state):
    snap = state.snapshot()
    with state.lock:
        if state.process is None: raise RuntimeError("No mission is active")
    if snap["controller_age_s"] is None or snap["controller_age_s"] > 1.0:
        raise RuntimeError("Controller telemetry is not live; use the handheld Pause")
    send_udp(CONTROL_PORT, {"command": "pause"}, repeats=3)
    send_udp(CMD_VEL_PORT, {"linear_x": 0.0, "angular_z": 0.0, "timestamp": time.time()}, repeats=5)
    with state.lock: state.process_state = "PAUSED"


def clear_pause(state):
    snap = state.snapshot()
    with state.lock:
        if state.process is None: raise RuntimeError("No mission is active")
    if snap["controller_age_s"] is None or snap["controller_age_s"] > 1.0:
        raise RuntimeError("Controller telemetry is not live; retain the handheld Pause")
    if snap["controller"].get("software_paused") is not True:
        raise RuntimeError("No dashboard software Pause is active")
    ok, reason = safe_to_start(state)
    if not ok:
        raise RuntimeError(f"Cannot clear software Pause: {reason}")
    send_udp(CONTROL_PORT, {"command": "resume"}, repeats=3)
    with state.lock: state.process_state = "WAITING"


def load_mission_payload():
    rows = [list(map(float, line.split())) for line in MISSION.read_text().splitlines()]
    lat0, lon0 = rows[0][0], rows[0][1]
    lon_scale = 111_320.0 * math.cos(math.radians(lat0))
    path = [{"x": (row[1] - lon0) * lon_scale, "y": (row[0] - lat0) * 110_540.0,
             "speed": row[4], "lookahead": row[3]} for row in rows]
    with AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        phases = [row.get("phase", "") for row in csv.DictReader(handle)]
    return {
        "path": path,
        "phases": phases,
        "waypoints": len(path),
        "origin_lat": lat0,
        "origin_lon": lon0,
        "lon_scale": lon_scale,
        "lat_scale": 110_540.0,
    }


def handler_factory(state, token, mission_payload):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def authorized(self):
            query_key = parse_qs(urlparse(self.path).query).get("key", [""])[0]
            return secrets.compare_digest(self.headers.get("X-Operator-Key", "") or query_key, token)

        def send_json(self, value, status=200):
            data = json.dumps(json_safe(value), allow_nan=False).encode("utf-8")
            self.send_response(status); self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store"); self.send_header("Content-Length", str(len(data)))
            self.end_headers(); self.wfile.write(data)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/":
                data = HTML.encode("utf-8"); self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data); return
            if path == "/messages":
                data = MESSAGES_HTML.encode("utf-8"); self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data); return
            if not self.authorized(): self.send_json({"error": "Invalid operator key"}, 403); return
            if path == "/api/state": self.send_json(state.snapshot()); return
            if path == "/api/mission": self.send_json(mission_payload); return
            self.send_json({"error": "Not found"}, 404)

        def do_POST(self):
            if not self.authorized(): self.send_json({"error": "Invalid operator key"}, 403); return
            length = int(self.headers.get("Content-Length", "0")); body = self.rfile.read(length)
            try: payload = json.loads(body or b"{}")
            except ValueError: self.send_json({"error": "Invalid JSON"}, 400); return
            path = urlparse(self.path).path
            try:
                if path == "/api/start":
                    if payload.get("confirmation") != EXPECTED_CONFIRMATION:
                        raise RuntimeError("Start confirmation was not accepted")
                    start_mission(state)
                elif path == "/api/pause": pause_mission(state)
                elif path == "/api/clear-pause": clear_pause(state)
                else: self.send_json({"error": "Not found"}, 404); return
            except RuntimeError as exc: self.send_json({"error": str(exc)}, 409); return
            self.send_json({"ok": True})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    args = parser.parse_args()
    for required in (MISSION, AUDIT, LAUNCHER):
        if not required.is_file(): raise SystemExit(f"Required file not found: {required}")
    token = secrets.token_urlsafe(18)
    state = MissionState()
    threading.Thread(target=udp_listener, args=(state, TELEMETRY_PORT, "controller"), daemon=True).start()
    threading.Thread(target=udp_listener, args=(state, STATUS_PORT, "bridge"), daemon=True).start()
    threading.Thread(target=udp_listener, args=(state, GPS_DASHBOARD_PORT, "gps"), daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.port), handler_factory(state, token, load_mission_payload()))
    print("Tractor01 mission dashboard")
    print(f"ZeroTier: http://{TRACTOR_ZEROTIER_IP}:{args.port}/?key={token}")
    print(f"Local:    http://{TRACTOR_LOCAL_IP}:{args.port}/?key={token}")
    print(f"Hostname: http://raspberrypi:{args.port}/?key={token}")
    print("Keep this terminal open. Press Ctrl+C to close the dashboard safely.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nClosing dashboard and stopping any active mission...")
    finally:
        with state.lock: process = state.process
        if process is not None:
            try: pause_mission(state)
            except RuntimeError: pass
            try: os.killpg(process.pid, signal.SIGINT)
            except (OSError, ProcessLookupError): pass
        server.server_close()


if __name__ == "__main__":
    main()
