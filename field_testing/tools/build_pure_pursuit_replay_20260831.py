#!/usr/bin/env python3
"""Build a self-contained step-through Pure Pursuit replay from field logs."""

from __future__ import annotations

import argparse
import bisect
import csv
import datetime as dt
import json
import math
from pathlib import Path


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mission", type=Path)
    parser.add_argument("pursuit_log", type=Path)
    parser.add_argument("field_log", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--waypoint-start", type=int, default=2970)
    parser.add_argument("--waypoint-end", type=int, default=3060)
    parser.add_argument("--sample-hz", type=float, default=5.0)
    return parser.parse_args()


def number(value, default=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def integer(value, default=None):
    value = number(value, default)
    return int(value) if value is not None else default


def truth(value):
    return str(value).strip().lower() in {"1", "true", "yes"}


def load_mission(path: Path):
    rows = []
    for line in path.read_text(encoding="ascii").splitlines():
        if not line.strip():
            continue
        lat, lon, yaw, lookahead, speed = map(float, line.split())
        rows.append((lat, lon, yaw, lookahead, speed))
    lat0, lon0 = rows[0][0], rows[0][1]
    east_scale = 111_320.0 * math.cos(math.radians(lat0))
    return [
        {
            "x": round((lon - lon0) * east_scale, 4),
            "y": round((lat - lat0) * 110_540.0, 4),
            "yaw": round(yaw, 6),
            "lookahead": round(lookahead, 3),
            "speed": round(speed, 3),
        }
        for lat, lon, yaw, lookahead, speed in rows
    ]


def load_field(path: Path):
    result = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            try:
                timestamp = dt.datetime.fromisoformat(row["time"]).timestamp()
            except (KeyError, TypeError, ValueError):
                continue
            result.append((timestamp, row))
    return result


def closest_field(field, timestamps, timestamp):
    index = bisect.bisect_left(timestamps, timestamp)
    choices = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(field)]
    if not choices:
        return {}
    return field[min(choices, key=lambda candidate: abs(field[candidate][0] - timestamp))][1]


def load_samples(path: Path, field, waypoint_start: int, waypoint_end: int, sample_hz: float):
    timestamps = [row[0] for row in field]
    interval = 1.0 / sample_hz
    samples = []
    last_time = -math.inf
    last_waypoint = None
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            timestamp = number(row.get("timestamp"))
            waypoint = integer(row.get("waypoint_idx"))
            if timestamp is None or waypoint is None or not truth(row.get("driving")):
                continue
            if waypoint < waypoint_start or waypoint > waypoint_end:
                continue
            if timestamp - last_time < interval and waypoint == last_waypoint:
                continue
            matched = closest_field(field, timestamps, timestamp)
            samples.append({
                "t": round(timestamp, 3),
                "elapsed": number(row.get("elapsed_s")),
                "x": number(row.get("pos_x_m")),
                "y": number(row.get("pos_y_m")),
                "heading": number(row.get("heading_compass_deg")),
                "recordedIdx": waypoint,
                "targetX": number(row.get("target_x_m")),
                "targetY": number(row.get("target_y_m")),
                "lookahead": number(row.get("lookahead_dist_m")),
                "yt": number(row.get("yt_m")),
                "alpha": number(row.get("alpha_deg")),
                "delta": number(row.get("delta_deg")),
                "steer": number(row.get("steer_normalized")),
                "speedCmd": number(row.get("speed_cmd_mps")),
                "fix": row.get("fix_quality", ""),
                "headValid": truth(row.get("head_valid")),
                "actualSpeed": number(matched.get("speed_mps")),
                "jrkTarget": integer(matched.get("jrk_target")),
                "jrkCurrent": integer(matched.get("jrk_current")),
                "steerSetpoint": integer(matched.get("steer_setpoint")),
                "steerCurrent": integer(matched.get("steer_current")),
                "steerError": integer(matched.get("steer_error")),
                "steerPwm": integer(matched.get("steer_pwm")),
                "steerSaturated": truth(matched.get("steer_pwm_saturated")),
                "mode": matched.get("steer_state", ""),
            })
            last_time = timestamp
            last_waypoint = waypoint
    if not samples:
        raise ValueError("No driving pursuit samples in the selected waypoint range")
    return samples


def html_document(payload):
    data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
    return r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pure Pursuit field-log replay</title>
<style>
:root{color-scheme:light dark;font-family:system-ui,sans-serif}body{margin:0;padding:16px;background:#101418;color:#e8edf2}main{max-width:1200px;margin:auto}.controls{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:10px 0}button,input{font:inherit}button{padding:7px 12px}input[type=range]{min-width:240px}.layout{display:grid;grid-template-columns:minmax(0,2fr) minmax(280px,1fr);gap:14px}svg{width:100%;height:auto;background:#182028;border:1px solid #52606d}.facts{display:grid;grid-template-columns:1fr 1fr;gap:1px;background:#52606d;border:1px solid #52606d}.fact{background:#182028;padding:7px}.fact span{display:block;color:#9fb0bf;font-size:.8rem}.fact b{font-weight:600}.note{color:#b8c5cf;font-size:.9rem}.mission{fill:none;stroke:#7790a3;stroke-width:1.2}.trail{fill:none;stroke:#44bba4;stroke-width:2}.heading{stroke:#ffd166;stroke-width:2}.recorded{stroke:#ef476f;stroke-width:1.5;stroke-dasharray:4 3}.simulated{stroke:#4cc9f0;stroke-width:2}.point{fill:#ffd166}.target-r{fill:#ef476f}.target-s{fill:#4cc9f0}@media(max-width:760px){.layout{grid-template-columns:1fr}.facts{grid-template-columns:1fr 1fr}}
</style></head><body><main>
<h1>Pure Pursuit replay: spiral exit and first keyhole</h1>
<div class="controls"><button id="prev">Previous</button><button id="play">Play</button><button id="next">Next</button><label>Step <input id="step" type="range" min="0" value="0"></label><output id="stepOut"></output></div>
<div class="controls"><label>Recomputed lookahead <input id="lookahead" type="range" min="0.5" max="3" step="0.25" value="1"></label><output id="lookOut">1.00 m</output><label><input id="recovery" type="checkbox"> Prototype forward reacquisition</label></div>
<p class="note">Red is the target recorded in the 2026-08-30 log. Blue replays the current target-selection rule. The optional prototype may advance to the nearest forward waypoint within 20 points when it is closer than the retained target.</p>
<div class="layout"><svg id="map" viewBox="0 0 760 620" role="img" aria-label="Mission, actual tractor trail, heading, and target points"></svg><div class="facts" id="facts"></div></div>
</main><script>
const DATA=__DATA__;
const svg=document.getElementById('map'), step=document.getElementById('step'), stepOut=document.getElementById('stepOut'), look=document.getElementById('lookahead'), lookOut=document.getElementById('lookOut'), recovery=document.getElementById('recovery'), facts=document.getElementById('facts');
step.max=DATA.samples.length-1; let timer=null, simulations=[];
const all=[...DATA.path,...DATA.samples]; const xs=all.map(d=>d.x), ys=all.map(d=>d.y), pad=2; const minX=Math.min(...xs)-pad,maxX=Math.max(...xs)+pad,minY=Math.min(...ys)-pad,maxY=Math.max(...ys)+pad;
const sx=x=>35+(x-minX)/(maxX-minX)*690, sy=y=>590-(y-minY)/(maxY-minY)*550;
const el=(name,attrs={})=>{const node=document.createElementNS('http://www.w3.org/2000/svg',name);for(const[k,v]of Object.entries(attrs))node.setAttribute(k,v);return node};
function pathD(points){return points.map((p,i)=>(i?'L':'M')+sx(p.x).toFixed(1)+' '+sy(p.y).toFixed(1)).join(' ')}
const mission=el('path',{class:'mission',d:pathD(DATA.path)}), trail=el('path',{class:'trail'}), recordedLine=el('line',{class:'recorded'}), simulatedLine=el('line',{class:'simulated'}), headingLine=el('line',{class:'heading'}), tractor=el('circle',{class:'point',r:5}), targetR=el('circle',{class:'target-r',r:5}), targetS=el('circle',{class:'target-s',r:4});
svg.append(mission,trail,recordedLine,simulatedLine,headingLine,tractor,targetR,targetS);
function recompute(){const L=Number(look.value), useRecovery=recovery.checked; simulations=[];let idx=Math.max(0,DATA.samples[0].recordedIdx-5);
 for(const s of DATA.samples){if(useRecovery){let best=idx,bestD=Infinity;for(let j=idx;j<Math.min(DATA.path.length,idx+21);j++){const p=DATA.path[j],d=Math.hypot(p.x-s.x,p.y-s.y);if(d<bestD){bestD=d;best=j}}const old=DATA.path[idx],oldD=Math.hypot(old.x-s.x,old.y-s.y);if(best>idx&&bestD<oldD)idx=best}
  let found=idx;for(let j=idx;j<DATA.path.length;j++){const p=DATA.path[j];if(Math.hypot(p.x-s.x,p.y-s.y)>L){found=j;break}}idx=found;const p=DATA.path[idx],dx=p.x-s.x,dy=p.y-s.y,h=(90-s.heading)*Math.PI/180,yt=-dx*Math.sin(h)+dy*Math.cos(h),delta=Math.max(-.623,Math.min(.623,Math.atan2(2*yt*1.27,L*L)));simulations.push({idx,x:p.x,y:p.y,distance:Math.hypot(dx,dy),yt,delta:delta*180/Math.PI,steer:delta/.623})}}
function fmt(v,d=2){return v==null?'—':Number(v).toFixed(d)}function fact(label,value){return `<div class="fact"><span>${label}</span><b>${value}</b></div>`}
function draw(){const i=Number(step.value),s=DATA.samples[i],q=simulations[i],h=(90-s.heading)*Math.PI/180;stepOut.value=`${i+1} / ${DATA.samples.length}`;trail.setAttribute('d',pathD(DATA.samples.slice(0,i+1)));tractor.setAttribute('cx',sx(s.x));tractor.setAttribute('cy',sy(s.y));headingLine.setAttribute('x1',sx(s.x));headingLine.setAttribute('y1',sy(s.y));headingLine.setAttribute('x2',sx(s.x+2*Math.cos(h)));headingLine.setAttribute('y2',sy(s.y+2*Math.sin(h)));recordedLine.setAttribute('x1',sx(s.x));recordedLine.setAttribute('y1',sy(s.y));recordedLine.setAttribute('x2',sx(s.targetX));recordedLine.setAttribute('y2',sy(s.targetY));simulatedLine.setAttribute('x1',sx(s.x));simulatedLine.setAttribute('y1',sy(s.y));simulatedLine.setAttribute('x2',sx(q.x));simulatedLine.setAttribute('y2',sy(q.y));targetR.setAttribute('cx',sx(s.targetX));targetR.setAttribute('cy',sy(s.targetY));targetS.setAttribute('cx',sx(q.x));targetS.setAttribute('cy',sy(q.y));
 facts.innerHTML=fact('Controller elapsed',fmt(s.elapsed,1)+' s')+fact('Compass heading',fmt(s.heading,1)+'°')+fact('Recorded waypoint',s.recordedIdx)+fact('Recomputed waypoint',q.idx)+fact('Recomputed target distance',fmt(q.distance,3)+' m')+fact('Lookahead',fmt(Number(look.value),2)+' m')+fact('Recorded lateral yt',fmt(s.yt,3)+' m')+fact('Recomputed lateral yt',fmt(q.yt,3)+' m')+fact('Recorded steering',fmt(s.delta,1)+'° / '+fmt(s.steer,2))+fact('Recomputed steering',fmt(q.delta,1)+'° / '+fmt(q.steer,2))+fact('Command / actual speed',fmt(s.speedCmd,2)+' / '+fmt(s.actualSpeed,2)+' m/s')+fact('JRK target / feedback',(s.jrkTarget??'—')+' / '+(s.jrkCurrent??'—'))+fact('Steering pot target / actual',(s.steerSetpoint??'—')+' / '+(s.steerCurrent??'—'))+fact('Pot error / PWM',(s.steerError??'—')+' / '+(s.steerPwm??'—'))+fact('Steering saturated',s.steerSaturated?'Yes':'No')+fact('GPS / heading',s.fix+' / '+(s.headValid?'valid':'invalid'));}
function refresh(){recompute();draw()}step.addEventListener('input',draw);look.addEventListener('input',()=>{lookOut.value=Number(look.value).toFixed(2)+' m';refresh()});recovery.addEventListener('change',refresh);document.getElementById('prev').onclick=()=>{step.value=Math.max(0,Number(step.value)-1);draw()};document.getElementById('next').onclick=()=>{step.value=Math.min(Number(step.max),Number(step.value)+1);draw()};document.getElementById('play').onclick=e=>{if(timer){clearInterval(timer);timer=null;e.target.textContent='Play';return}e.target.textContent='Pause';timer=setInterval(()=>{if(Number(step.value)>=Number(step.max)){clearInterval(timer);timer=null;e.target.textContent='Play';return}step.value=Number(step.value)+1;draw()},120)};refresh();
</script></body></html>'''.replace("__DATA__", data)


def main():
    args = arguments()
    mission = load_mission(args.mission)
    field = load_field(args.field_log)
    samples = load_samples(
        args.pursuit_log,
        field,
        args.waypoint_start,
        args.waypoint_end,
        args.sample_hz,
    )
    path_start = max(0, args.waypoint_start - 30)
    path_end = min(len(mission), args.waypoint_end + 60)
    payload = {
        "path": [dict(point, index=index) for index, point in enumerate(mission[path_start:path_end], path_start)],
        "samples": samples,
        "missionLength": len(mission),
        "pathStart": path_start,
    }
    # The browser simulation indexes the full mission. Pad the omitted prefix
    # with the first displayed point so absolute waypoint indices remain valid.
    payload["path"] = [payload["path"][0]] * path_start + payload["path"]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(html_document(payload), encoding="utf-8")
    print(f"Replay: {args.output}")
    print(f"Samples: {len(samples)} at up to {args.sample_hz:.1f} Hz")
    print(f"Waypoint window: {args.waypoint_start}..{args.waypoint_end}")


if __name__ == "__main__":
    main()
