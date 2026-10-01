#!/usr/bin/env python3
"""Build the focused 2026-09-26 phone/manual field-run review."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from collections import Counter
from datetime import datetime
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
SITE = REPO / "field_testing/sites/62_Collins_multi_boundary_20260915"
RUN = SITE / "runs/20260926_phone_manual_perimeter_extension"
CSV_PATH = RUN / "field_test_20260926_121311.csv"
PHONE_PATH = RUN / "wifi_manual_control_20260926_121332.jsonl"
INVENTORY_PATH = SITE / "site_inventory/62_Collins_site_inventory.geojson"
OUT = SITE / "analysis/20260926_phone_manual_perimeter_review"
OUT_HTML = OUT / "62_Collins_20260926_phone_manual_review.html"
OUT_JSON = OUT / "analysis_summary.json"
OUT_GEOJSON = OUT / "20260926_routes_and_control_events.geojson"
OUT_README = OUT / "README.md"

MODE_NAMES = {"0": "phone", "1": "legacy_handheld", "2": "pause", "9": "nrf_safety"}


def parse_time(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    position = (len(values) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return values[lower]
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def load_rows():
    with CSV_PATH.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        row["_time"] = parse_time(row["time"])
    return rows


def mode_intervals(rows, target_mode):
    intervals = []
    start = None
    for index, row in enumerate(rows):
        match = row["trans_mode"] == target_mode
        if match and start is None:
            start = index
        if start is not None and (not match or index == len(rows) - 1):
            end = index if match and index == len(rows) - 1 else index - 1
            first, last = rows[start], rows[end]
            intervals.append({
                "start": first["time"],
                "end": last["time"],
                "duration_s": round((last["_time"] - first["_time"]).total_seconds(), 3),
                "rows": end - start + 1,
                "lat": number(first.get("lat")),
                "lon": number(first.get("lon")),
            })
            start = None
    return intervals


def source_summary(rows, mode):
    selected = [row for row in rows if row["trans_mode"] == mode]
    moving = [row for row in selected if (number(row.get("speed_mps")) or 0.0) > 0.15]
    distance = 0.0
    moving_seconds = 0.0
    for first, second in zip(selected, selected[1:]):
        delta = (second["_time"] - first["_time"]).total_seconds()
        if not 0.0 < delta < 0.2:
            continue
        speed = max(0.0, number(first.get("speed_mps")) or 0.0)
        distance += speed * delta
        if speed > 0.15:
            moving_seconds += delta
    rssis = [number(row.get("wifi_rssi_dbm")) for row in moving]
    rssis = [value for value in rssis if value is not None]
    speeds = [number(row.get("speed_mps")) for row in selected]
    speeds = [value for value in speeds if value is not None]
    return {
        "rows": len(selected),
        "moving_rows": len(moving),
        "moving_seconds": round(moving_seconds, 1),
        "estimated_distance_m": round(distance, 1),
        "maximum_speed_mps": max(speeds) if speeds else None,
        "moving_wifi_median_dbm": round(percentile(rssis, 0.5), 1) if rssis else None,
        "moving_wifi_minimum_dbm": min(rssis) if rssis else None,
        "fix_quality": dict(Counter(row.get("fix_quality") for row in selected)),
        "heading_carrier": dict(Counter(row.get("carrier") for row in selected)),
    }


def sample_routes(rows):
    routes = {name: [] for name in ("phone", "legacy_handheld", "nrf_safety")}
    last_time = {name: None for name in routes}
    legacy_intervals = mode_intervals(rows, "1")
    legacy_main_start = parse_time(max(legacy_intervals, key=lambda item: item["duration_s"])["start"])
    for row in rows:
        name = MODE_NAMES.get(row["trans_mode"])
        if name not in routes:
            continue
        # Ignore the momentary mode-switch contact seen near startup; the
        # longest Manual interval is the actual legacy-handheld survey.
        if name == "legacy_handheld" and row["_time"] < legacy_main_start:
            continue
        lat, lon = number(row.get("lat")), number(row.get("lon"))
        speed = number(row.get("speed_mps")) or 0.0
        if lat is None or lon is None or (name != "nrf_safety" and speed <= 0.05):
            continue
        previous = last_time[name]
        if previous is not None and (row["_time"] - previous).total_seconds() < 0.25:
            continue
        routes[name].append({
            "lat": lat,
            "lon": lon,
            "time": row["time"],
            "speed": speed,
            "wifi": number(row.get("wifi_rssi_dbm")),
        })
        last_time[name] = row["_time"]
    return routes


def phone_analysis():
    records = [json.loads(line) for line in PHONE_PATH.read_text(encoding="utf-8").splitlines() if line.strip()]
    commands = [record for record in records if record.get("event") == "command"]
    drive = [record for record in commands if record.get("decision") == "drive"]
    expiries = [
        record for record in records
        if record.get("event") == "stop_burst" and record.get("reason") == "phone_freshness_expired"
    ]
    drive_values = [float(record["drive_percent"]) for record in drive]
    steer_values = [float(record["steering_percent"]) for record in drive]
    return records, expiries, {
        "records": len(records),
        "commands": len(commands),
        "drive_commands": len(drive),
        "freshness_expirations": len(expiries),
        "expiration_times": [record["time"] for record in expiries],
        "drive_percent": {
            "minimum": min(drive_values),
            "median": round(percentile(drive_values, 0.5), 1),
            "p95": round(percentile(drive_values, 0.95), 1),
            "maximum": max(drive_values),
            "reverse_rows": sum(value < 0 for value in drive_values),
            "neutral_rows": sum(value == 0 for value in drive_values),
            "forward_rows": sum(value > 0 for value in drive_values),
        },
        "steering_percent": {
            "minimum": min(steer_values),
            "median": round(percentile(steer_values, 0.5), 1),
            "p95": round(percentile(steer_values, 0.95), 1),
            "maximum": max(steer_values),
        },
    }


def nearest_row(rows, event_time):
    return min(rows, key=lambda row: abs((row["_time"] - event_time).total_seconds()))


def make_geojson(routes, phone_events, nrf_intervals):
    features = []
    for name, points in routes.items():
        if len(points) < 2:
            continue
        features.append({
            "type": "Feature",
            "properties": {"layer": name},
            "geometry": {"type": "LineString", "coordinates": [[point["lon"], point["lat"]] for point in points]},
        })
    for event in phone_events:
        features.append({
            "type": "Feature",
            "properties": {"layer": "phone_timeout", "time": event["time"], "wifi_rssi_dbm": event["wifi"]},
            "geometry": {"type": "Point", "coordinates": [event["lon"], event["lat"]]},
        })
    for event in nrf_intervals:
        features.append({
            "type": "Feature",
            "properties": {"layer": "nrf_safety", **event},
            "geometry": {"type": "Point", "coordinates": [event["lon"], event["lat"]]},
        })
    return {"type": "FeatureCollection", "features": features}


def html_page(inventory, routes, phone_events, nrf_events, summary):
    data = json.dumps({
        "inventory": inventory,
        "routes": routes,
        "phoneEvents": phone_events,
        "nrfEvents": nrf_events,
        "summary": summary,
    }, separators=(",", ":"))
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>62 Collins — 2026-09-26 control and perimeter review</title>
<style>
html,body{{margin:0;height:100%;font-family:system-ui,sans-serif;background:#f3f6f8;color:#14202a}}.app{{display:grid;grid-template-columns:310px 1fr;height:100%}}aside{{padding:18px;background:#111b25;color:#edf5fa;overflow:auto}}h1{{font-size:20px;margin:0 0 8px}}h2{{font-size:14px;margin:20px 0 8px;color:#8dd5ff}}label{{display:block;margin:9px 0}}.swatch{{display:inline-block;width:18px;height:4px;margin-right:8px;vertical-align:middle}}.note{{font-size:12px;line-height:1.45;color:#bfd0dc}}.stat{{padding:8px;margin:7px 0;background:#1b2a37;border-radius:8px;font-size:12px}}main{{position:relative;min-width:0}}canvas{{width:100%;height:100%;display:block;background:#f8fafb}}.legend{{position:absolute;right:12px;top:12px;padding:8px 10px;background:#fffffff0;border:1px solid #b6c2ca;border-radius:8px;font-size:11px}}@media(max-width:750px){{.app{{grid-template-columns:1fr;grid-template-rows:auto 70vh}}aside{{max-height:42vh}}}}
</style></head><body><div class="app"><aside>
<h1>2026-09-26 field review</h1><div class="note">Phone control, legacy handheld coverage, NRF safety events, and phone watchdog stops over inventory revision 2.</div>
<h2>Layers</h2>
<label><input type="checkbox" data-layer="inventory" checked> Inventory boundaries and obstacles</label>
<label><input type="checkbox" data-layer="phone" checked> <span class="swatch" style="background:#087f5b"></span>Phone-controlled travel</label>
<label><input type="checkbox" data-layer="legacy_handheld" checked> <span class="swatch" style="background:#1864ab"></span>Legacy-handheld travel</label>
<label><input type="checkbox" data-layer="nrf_safety" checked> <span class="swatch" style="background:#e67700"></span>NRF safety locations</label>
<label><input type="checkbox" data-layer="phone_timeout" checked> <span class="swatch" style="background:#c92a2a"></span>Phone freshness timeouts</label>
<h2>Run summary</h2>
<div class="stat">Phone distance estimate: <b>{summary['sources']['phone']['estimated_distance_m']} m</b><br>Moving time: {summary['sources']['phone']['moving_seconds']} s</div>
<div class="stat">Legacy-handheld distance estimate: <b>{summary['sources']['legacy_handheld']['estimated_distance_m']} m</b><br>Moving time: {summary['sources']['legacy_handheld']['moving_seconds']} s</div>
<div class="stat">Phone watchdog stops: <b>{summary['phone']['freshness_expirations']}</b><br>NRF safety episodes: <b>{summary['nrf']['episodes']}</b> ({summary['nrf']['total_duration_s']} s)</div>
<div class="note">Distances integrate GPS ground speed and are approximate. This is review evidence; it does not automatically revise authoritative boundaries.</div>
</aside><main><canvas id="map"></canvas><div class="legend">Drag to pan · wheel to zoom · colored rings mark failures</div></main></div>
<script>const D={data};const C=document.getElementById('map'),X=C.getContext('2d');let visible={{inventory:true,phone:true,legacy_handheld:true,nrf_safety:true,phone_timeout:true}};let pts=[];
function walk(c){{if(Array.isArray(c)&&typeof c[0]==='number')pts.push(c);else if(Array.isArray(c))c.forEach(walk)}}D.inventory.features.forEach(f=>walk(f.geometry.coordinates));Object.values(D.routes).flat().forEach(p=>pts.push([p.lon,p.lat]));
const lat0=pts.reduce((s,p)=>s+p[1],0)/pts.length,lon0=pts.reduce((s,p)=>s+p[0],0)/pts.length,ls=111320*Math.cos(lat0*Math.PI/180);function pr(p){{return [(p[0]-lon0)*ls,(p[1]-lat0)*110540]}}const mp=pts.map(pr),xs=mp.map(p=>p[0]),ys=mp.map(p=>p[1]);let base={{xmin:Math.min(...xs),xmax:Math.max(...xs),ymin:Math.min(...ys),ymax:Math.max(...ys)}},view={{...base}},drag=null;
function resize(){{C.width=C.clientWidth*devicePixelRatio;C.height=C.clientHeight*devicePixelRatio;draw()}}function sc(p){{let q=pr(p),pad=24*devicePixelRatio,w=C.width-2*pad,h=C.height-2*pad,s=Math.min(w/(view.xmax-view.xmin),h/(view.ymax-view.ymin));return [pad+(q[0]-view.xmin)*s,C.height-pad-(q[1]-view.ymin)*s]}}
function path(coords,close=false){{X.beginPath();coords.forEach((p,i)=>{{let q=sc(p);i?X.lineTo(...q):X.moveTo(...q)}});if(close)X.closePath()}}
function geom(g){{if(g.type==='Point')return;let groups=g.type==='Polygon'?g.coordinates:g.type==='MultiPolygon'?g.coordinates.flat():g.type==='LineString'?[g.coordinates]:[];groups.forEach(c=>{{path(c,g.type.includes('Polygon'));X.stroke();if(g.type.includes('Polygon'))X.fill()}})}}
function draw(){{X.clearRect(0,0,C.width,C.height);X.lineJoin='round';X.lineCap='round';if(visible.inventory)D.inventory.features.forEach(f=>{{let cat=f.properties.category;X.strokeStyle=cat==='obstacle'?'#d9480f':cat==='transition_route'?'#087f5b':'#1971c2';X.fillStyle=cat==='obstacle'?'#ff922b33':'#74c0fc22';X.lineWidth=(cat==='transition_route'?2:1.5)*devicePixelRatio;geom(f.geometry)}});for(const [name,color] of [['phone','#087f5b'],['legacy_handheld','#1864ab']])if(visible[name]){{let a=D.routes[name].map(p=>[p.lon,p.lat]);X.strokeStyle=color;X.lineWidth=2.2*devicePixelRatio;path(a);X.stroke()}}if(visible.nrf_safety)D.nrfEvents.forEach(e=>marker(e.lon,e.lat,'#e67700',7));if(visible.phone_timeout)D.phoneEvents.forEach(e=>marker(e.lon,e.lat,'#c92a2a',9))}}
function marker(lon,lat,color,r){{let p=sc([lon,lat]);X.beginPath();X.arc(...p,r*devicePixelRatio,0,Math.PI*2);X.fillStyle=color+'55';X.fill();X.strokeStyle=color;X.lineWidth=2*devicePixelRatio;X.stroke()}}
document.querySelectorAll('[data-layer]').forEach(e=>e.onchange=()=>{{visible[e.dataset.layer]=e.checked;draw()}});C.onpointerdown=e=>drag={{x:e.clientX,y:e.clientY,v:{{...view}}}};C.onpointermove=e=>{{if(!drag)return;let sx=(view.xmax-view.xmin)/C.clientWidth,sy=(view.ymax-view.ymin)/C.clientHeight,dx=(e.clientX-drag.x)*sx,dy=(e.clientY-drag.y)*sy;view={{xmin:drag.v.xmin-dx,xmax:drag.v.xmax-dx,ymin:drag.v.ymin+dy,ymax:drag.v.ymax+dy}};draw()}};onpointerup=()=>drag=null;C.onwheel=e=>{{e.preventDefault();let k=e.deltaY>0?1.15:.87,cx=(view.xmin+view.xmax)/2,cy=(view.ymin+view.ymax)/2,hw=(view.xmax-view.xmin)*k/2,hh=(view.ymax-view.ymin)*k/2;view={{xmin:cx-hw,xmax:cx+hw,ymin:cy-hh,ymax:cy+hh}};draw()}};addEventListener('resize',resize);resize();</script></body></html>"""


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rows = load_rows()
    inventory = json.loads(INVENTORY_PATH.read_text(encoding="utf-8"))
    routes = sample_routes(rows)
    _, expiries, phone_summary = phone_analysis()
    nrf_intervals = mode_intervals(rows, "9")
    phone_events = []
    for event in expiries:
        row = nearest_row(rows, parse_time(event["time"]))
        phone_events.append({
            "time": event["time"],
            "lat": number(row["lat"]),
            "lon": number(row["lon"]),
            "wifi": number(row["wifi_rssi_dbm"]),
            "speed": number(row["speed_mps"]),
        })

    nrf_events = [event for event in nrf_intervals if event["lat"] is not None and event["lon"] is not None]
    summary = {
        "source_files": {
            CSV_PATH.name: {"bytes": CSV_PATH.stat().st_size, "sha256": sha256(CSV_PATH)},
            PHONE_PATH.name: {"bytes": PHONE_PATH.stat().st_size, "sha256": sha256(PHONE_PATH)},
        },
        "rows": len(rows),
        "start": rows[0]["time"],
        "end": rows[-1]["time"],
        "duration_s": round((rows[-1]["_time"] - rows[0]["_time"]).total_seconds(), 3),
        "sources": {
            "phone": source_summary(rows, "0"),
            "legacy_handheld": source_summary(rows, "1"),
        },
        "phone": phone_summary,
        "nrf": {
            "episodes": len(nrf_intervals),
            "total_duration_s": round(sum(event["duration_s"] for event in nrf_intervals), 3),
            "first": nrf_intervals[0]["start"] if nrf_intervals else None,
            "last": nrf_intervals[-1]["end"] if nrf_intervals else None,
            "intervals": nrf_intervals,
        },
        "interpretation": [
            "NRF safety events are confined to the early 16:23:19-16:25:01 interval.",
            "Four later phone freshness expirations occurred with healthy NRF supervision.",
            "The phone and NRF interruptions are distinct failure mechanisms.",
        ],
    }

    OUT_JSON.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    OUT_GEOJSON.write_text(json.dumps(make_geojson(routes, phone_events, nrf_events), indent=2) + "\n", encoding="utf-8")
    OUT_HTML.write_text(html_page(inventory, routes, phone_events, nrf_events, summary), encoding="utf-8")
    OUT_README.write_text(
        "# 2026-09-26 phone/manual perimeter-extension review\n\n"
        "This review keeps the new route as evidence and does not automatically modify the authoritative site inventory.\n\n"
        f"- Field rows: **{len(rows):,}**\n"
        f"- Phone travel estimate: **{summary['sources']['phone']['estimated_distance_m']} m**\n"
        f"- Legacy-handheld travel estimate: **{summary['sources']['legacy_handheld']['estimated_distance_m']} m**\n"
        f"- Phone freshness expirations: **{phone_summary['freshness_expirations']}**\n"
        f"- NRF safety episodes: **{len(nrf_intervals)}**, totaling **{summary['nrf']['total_duration_s']} s**\n\n"
        "Open `62_Collins_20260926_phone_manual_review.html` for layer-by-layer spatial review.\n"
        "See `analysis_summary.json` for exact event times and source checksums.\n",
        encoding="utf-8",
    )
    print(OUT_HTML)
    print(OUT_JSON)


if __name__ == "__main__":
    main()
