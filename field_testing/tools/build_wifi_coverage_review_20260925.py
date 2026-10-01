#!/usr/bin/env python3
"""Build an interactive Wi-Fi coverage overlay for the 62 Collins inventory.

This is evidence/review output, not part of the authoritative site inventory.
It combines the revision-2 inventory geometry with the 2026-09-23 auto and
manual field-test logs and keeps the Pi and tractor-router measurements as
separate, toggleable layers.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path

from shapely.geometry import Point, shape


REPO = Path(__file__).resolve().parents[2]
SITE = REPO / "field_testing/sites/62_Collins_multi_boundary_20260915"
INVENTORY = SITE / "site_inventory/62_Collins_site_inventory.geojson"
LOGGER = REPO / "tractor_rpi/field_test_logger_20260828.py"
OUT = SITE / "analysis/20260923_wifi_coverage_review"
OUT_HTML = OUT / "62_Collins_wifi_coverage_review_20260923.html"
OUT_REPORT = OUT / "wifi_coverage_report_20260923.json"
OUT_README = OUT / "README.md"

RUNS = [
    SITE / "runs/20260915_133923/field_test_20260915_133923.csv",
    SITE / "runs/20260923_105953_master_manual_initial/master_manual_field_20260923_105953.csv",
    SITE / "runs/20260923_124933_master_manual_5hz/master_manual_field_20260923_124933.csv",
    SITE / "runs/20260923_135328_manual_stripes/manual_stripes_20260923_135328.csv",
    SITE / "runs/20260923_dual_f9p_5hz_manual/dual_f9p_5hz_manual_20260923.csv",
]

ACTIVE_MODES = {"1": "manual", "2": "auto"}
SAMPLE_INTERVAL_S = 0.5


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def signal_summary(rows, field):
    values = [value for row in rows if (value := number(row.get(field))) is not None]
    return {
        "rows": len(rows),
        "valid_rows": len(values),
        "missing_rows": len(rows) - len(values),
        "minimum_dbm": min(values) if values else None,
        "p05_dbm": round(percentile(values, 0.05), 1) if values else None,
        "median_dbm": round(percentile(values, 0.50), 1) if values else None,
        "maximum_dbm": max(values) if values else None,
        "below_minus_80_dbm_rows": sum(value < -80 for value in values),
    }


def missing_intervals(rows, field):
    intervals = []
    start = previous = None
    count = 0
    mode = None
    for row in rows:
        elapsed = number(row.get("elapsed_sec"))
        missing = row.get("trans_mode") in ACTIVE_MODES and number(row.get(field)) is None
        if missing and start is None:
            start = previous = elapsed
            count = 1
            mode = ACTIVE_MODES[row["trans_mode"]]
        elif missing:
            previous = elapsed
            count += 1
        elif start is not None:
            intervals.append({
                "mode": mode,
                "start_elapsed_s": round(start, 2),
                "end_elapsed_s": round(previous, 2),
                "duration_s": round(previous - start + 0.05, 2),
                "rows": count,
            })
            start = previous = None
            count = 0
            mode = None
    if start is not None:
        intervals.append({
            "mode": mode,
            "start_elapsed_s": round(start, 2),
            "end_elapsed_s": round(previous, 2),
            "duration_s": round(previous - start + 0.05, 2),
            "rows": count,
        })
    return intervals


def first_coordinate(geometry):
    coordinates = geometry["coordinates"]
    while isinstance(coordinates[0], list):
        coordinates = coordinates[0]
    return float(coordinates[1]), float(coordinates[0])


def project_geometry(geometry, lat0, lon0):
    lon_scale = 111_320.0 * math.cos(math.radians(lat0))

    def convert(value):
        if len(value) >= 2 and all(isinstance(item, (int, float)) for item in value[:2]):
            lon, lat = value[:2]
            return [round((lon - lon0) * lon_scale, 3), round((lat - lat0) * 110_540.0, 3)]
        return [convert(item) for item in value]

    return {"type": geometry["type"], "coordinates": convert(geometry["coordinates"])}


def load_data():
    inventory = json.loads(INVENTORY.read_text(encoding="utf-8"))
    lat0, lon0 = first_coordinate(inventory["features"][0]["geometry"])
    lon_scale = 111_320.0 * math.cos(math.radians(lat0))
    features = [
        {"id": feature["id"], "p": feature["properties"], "g": project_geometry(feature["geometry"], lat0, lon0)}
        for feature in inventory["features"]
    ]
    areas = {
        feature["properties"]["asset_id"]: shape(feature["geometry"])
        for feature in inventory["features"]
        if feature["properties"]["category"] == "mowable_area"
    }

    points = []
    run_reports = []
    combined_by_mode = defaultdict(list)
    all_active = []
    for path in RUNS:
        with path.open(newline="", encoding="utf-8-sig") as handle:
            rows = list(csv.DictReader(handle))
        active = [row for row in rows if row.get("trans_mode") in ACTIVE_MODES]
        all_active.extend(active)
        for row in active:
            combined_by_mode[ACTIVE_MODES[row["trans_mode"]]].append(row)

        area_rows = {
            mode_name: {area_id: 0 for area_id in areas}
            for mode_name in ACTIVE_MODES.values()
        }
        area_signal_rows = {
            mode_name: {
                area_id: {"pi_wifi": [], "tractor_router_wifi": []}
                for area_id in areas
            }
            for mode_name in ACTIVE_MODES.values()
        }
        outside_area_rows = {mode_name: 0 for mode_name in ACTIVE_MODES.values()}
        for row in active:
            lat = number(row.get("lat"))
            lon = number(row.get("lon"))
            if lat is None or lon is None:
                continue
            mode_name = ACTIVE_MODES[row["trans_mode"]]
            position = Point(lon, lat)
            matched = False
            for area_id, area in areas.items():
                if area.covers(position):
                    area_rows[mode_name][area_id] += 1
                    pi_value = number(row.get("wifi_rssi_dbm"))
                    router_value = number(row.get("router_wifi_rssi_dbm"))
                    if pi_value is not None:
                        area_signal_rows[mode_name][area_id]["pi_wifi"].append(pi_value)
                    if router_value is not None:
                        area_signal_rows[mode_name][area_id]["tractor_router_wifi"].append(router_value)
                    matched = True
            if not matched:
                outside_area_rows[mode_name] += 1

        last_bucket = None
        for row in active:
            lat = number(row.get("lat"))
            lon = number(row.get("lon"))
            elapsed = number(row.get("elapsed_sec"))
            if lat is None or lon is None or elapsed is None:
                continue
            bucket = math.floor(elapsed / SAMPLE_INTERVAL_S)
            if bucket == last_bucket:
                continue
            last_bucket = bucket
            points.append({
                "x": round((lon - lon0) * lon_scale, 3),
                "y": round((lat - lat0) * 110_540.0, 3),
                "pi": number(row.get("wifi_rssi_dbm")),
                "router": number(row.get("router_wifi_rssi_dbm")),
                "mode": ACTIVE_MODES[row["trans_mode"]],
                "date": path.parent.name[:8],
                "run": path.parent.name,
                "elapsed": round(elapsed, 2),
            })

        modes = {}
        for mode_number, mode_name in ACTIVE_MODES.items():
            mode_rows = [row for row in rows if row.get("trans_mode") == mode_number]
            if mode_rows:
                modes[mode_name] = {
                    "rows": len(mode_rows),
                    "pi_wifi": signal_summary(mode_rows, "wifi_rssi_dbm"),
                    "tractor_router_wifi": signal_summary(mode_rows, "router_wifi_rssi_dbm"),
                }
        area_signal_summary = {
            mode_name: {
                area_id: {
                    signal_name: {
                        "valid_rows": len(values),
                        "minimum_dbm": min(values) if values else None,
                        "p05_dbm": round(percentile(values, 0.05), 1) if values else None,
                        "median_dbm": round(percentile(values, 0.50), 1) if values else None,
                        "maximum_dbm": max(values) if values else None,
                    }
                    for signal_name, values in signals.items()
                }
                for area_id, signals in area_signal_rows[mode_name].items()
                if area_rows[mode_name][area_id]
            }
            for mode_name in ACTIVE_MODES.values()
        }
        run_reports.append({
            "run": path.parent.name,
            "source": str(path.relative_to(REPO)).replace("\\", "/"),
            "sha256": sha256(path),
            "rows": len(rows),
            "active_mode_rows": len(active),
            "active_rows_inside_inventory_areas": area_rows,
            "active_area_signal_summary": area_signal_summary,
            "active_rows_outside_inventory_areas": outside_area_rows,
            "modes": modes,
            "pi_wifi_missing_intervals": missing_intervals(rows, "wifi_rssi_dbm"),
            "tractor_router_wifi_missing_intervals": missing_intervals(rows, "router_wifi_rssi_dbm"),
        })

    combined = {
        mode: {
            "rows": len(rows),
            "pi_wifi": signal_summary(rows, "wifi_rssi_dbm"),
            "tractor_router_wifi": signal_summary(rows, "router_wifi_rssi_dbm"),
        }
        for mode, rows in sorted(combined_by_mode.items())
    }
    router_min_active = min(
        details["tractor_router_wifi"]["minimum_dbm"]
        for details in combined.values()
        if details["tractor_router_wifi"]["minimum_dbm"] is not None
    )
    pi_missing = [
        interval
        for run in run_reports
        for interval in run["pi_wifi_missing_intervals"]
    ]
    pi_missing_seconds = sum(interval["duration_s"] for interval in pi_missing)
    report = {
        "status": "INCONCLUSIVE_FOR_CONTINUITY",
        "review_date": "2026-09-25",
        "telemetry_date_range": ["2026-09-15", "2026-09-23"],
        "conclusion": (
            f"Recorded tractor-router RSSI stayed at or above {router_min_active:.0f} dBm in active auto/manual rows, "
            "but the logger has no router-reading timestamp or age and carries the last received "
            "value forward. The CSVs therefore cannot prove uninterrupted Wi-Fi connectivity."
        ),
        "notable_finding": (
            f"The Pi wlan0 reading was unavailable in {len(pi_missing)} active-mode intervals across "
            f"the included sessions ({pi_missing_seconds:.2f} seconds total). This is not the preferred "
            "tractor-router path, but it disproves a blanket claim that every logged Wi-Fi interface "
            "stayed associated."
        ),
        "scope": "Rows in transmission mode 1 (manual) or 2 (auto).",
        "session_selection": {
            "included_20260915_133923": (
                "Adds the missing geographic evidence: its active rows cover the backyard/gardens, "
                "front yard, and over-the-road areas."
            ),
            "excluded_20260918_121342": (
                "The partial-rings log adds backyard/manual and transition evidence but no front-yard "
                "or over-the-road active rows, so it would mostly duplicate mapped ground."
            ),
            "excluded_20260918_134506": (
                "The continuation run entered AUTO only briefly before the recorded NRF24 loss; its "
                "active AUTO rows do not fall inside any of the three inventory areas."
            ),
        },
        "sampling": f"Map samples are reduced to at most one point per {SAMPLE_INTERVAL_S:.1f} seconds per run.",
        "combined_by_mode": combined,
        "runs": run_reports,
        "limitations": [
            "router_wifi_* has no measurement timestamp, receive timestamp, or age field",
            "the field logger retains the last router value until another TCP message arrives",
            "RSSI presence does not measure command round-trip latency, packet loss, or phone-to-robot reachability",
            "the map is path evidence from these runs, not a prediction for unvisited ground",
        ],
        "recommended_instrumentation": [
            "add router_wifi_received_timestamp and router_wifi_age_s to every log row",
            "mark router data unavailable when age exceeds 10 seconds",
            "log an authenticated control-path heartbeat with sequence, round-trip latency, and loss count",
            "make the phone-control dead-man stop locally on the tractor when heartbeat age exceeds the tested bound",
        ],
        "source_sha256": {
            str(INVENTORY.relative_to(REPO)).replace("\\", "/"): sha256(INVENTORY),
            str(LOGGER.relative_to(REPO)).replace("\\", "/"): sha256(LOGGER),
        },
    }
    return features, points, report


def write_html(features, points, report):
    payload = json.dumps({"features": features, "points": points}, separators=(",", ":"))
    summary = report["combined_by_mode"]
    router_min = min(
        details["tractor_router_wifi"]["minimum_dbm"]
        for details in summary.values()
        if details["tractor_router_wifi"]["minimum_dbm"] is not None
    )
    template = r'''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>62 Collins Wi-Fi coverage review</title><style>
*{box-sizing:border-box}body{margin:0;background:#10151c;color:#e8eef5;font:14px system-ui,sans-serif}header{min-height:76px;padding:10px 16px;background:#17202a;border-bottom:1px solid #34404e}h1{font-size:19px;margin:0 0 5px}.sub{color:#ffcc80}main{display:grid;grid-template-columns:360px 1fr;height:calc(100vh - 76px)}aside{padding:14px;overflow:auto;border-right:1px solid #34404e}canvas{width:100%;height:100%;background:#f8fafc}.row{margin:8px 0}.small{font-size:12px;color:#b8c4cf;line-height:1.45}.warning{padding:9px;border:1px solid #b26a00;border-radius:6px;background:#302514;color:#ffe0a6;line-height:1.4}button{background:#263545;color:#fff;border:1px solid #526579;border-radius:5px;padding:7px 10px}.legend{display:grid;grid-template-columns:16px 1fr;gap:5px 7px;align-items:center;margin-top:10px}.swatch{width:14px;height:10px;border-radius:2px}#tip{position:fixed;display:none;pointer-events:none;background:#111c;color:#fff;border:1px solid #667;padding:7px;border-radius:5px;font-size:12px;z-index:5}</style></head><body><header><h1>62 Collins Wi-Fi coverage review — through 2026-09-23</h1><div class="sub">Recorded auto/manual evidence from September 15 and 23 over the revision-2 site inventory.</div></header><main><aside>
<div class="warning"><b>Continuity is not proven.</b><br>Router RSSI remained at or above __ROUTER_MIN__ dBm in active-mode rows, but the logger carries the last router value forward and records no reading age.</div>
<p><b>Wi-Fi layer</b></p><div class="row"><label><input name="wifi" id="routerWifi" type="radio" checked> Tractor-router upstream RSSI</label></div><div class="row"><label><input name="wifi" id="piWifi" type="radio"> Pi wlan0 RSSI</label></div><div class="row"><label><input name="wifi" id="noWifi" type="radio"> Hide Wi-Fi samples</label></div>
<p><b>Operating mode</b></p><div class="row"><label><input id="manual" type="checkbox" checked> Manual</label></div><div class="row"><label><input id="auto" type="checkbox" checked> Auto</label></div>
<p><b>Session date</b></p><div class="row"><label><input id="date15" type="checkbox" checked> September 15</label></div><div class="row"><label><input id="date23" type="checkbox" checked> September 23</label></div>
<p><b>Site layers</b></p><div class="row"><label><input id="areas" type="checkbox" checked> Mowable areas</label></div><div class="row"><label><input id="obstacles" type="checkbox" checked> Obstacles</label></div><div class="row"><label><input id="routes" type="checkbox"> Between-area routes</label></div><div class="row"><label><input id="points" type="checkbox"> Entry/exit and reference points</label></div><p><button id="fit">Fit all</button></p>
<div class="legend"><span class="swatch" style="background:#169c4b"></span><span>Strong: -65 dBm or better</span><span class="swatch" style="background:#f4b400"></span><span>Medium: -66 to -79 dBm</span><span class="swatch" style="background:#d93025"></span><span>Weak: below -80 dBm</span><span class="swatch" style="background:#20252b"></span><span>Unavailable measurement</span></div>
<p class="small">Hover a sample for run, mode, elapsed time, and RSSI. Colored dots are measurements along driven paths, not predictions between paths. The September 15 session adds front-yard and over-the-road evidence. The tractor-router layer is the intended field control path.</p>
</aside><canvas id="map"></canvas></main><div id="tip"></div><script>
const D=__PAYLOAD__,F=D.features,W=D.points,$=x=>document.getElementById(x),C=$('map'),X=C.getContext('2d'),TIP=$('tip');let s=8,ox=0,oy=0,drag=null;const colors={'AREA-BACKYARD-GARDENS':'#1565c0','AREA-FRONT-YARD':'#ef6c00','AREA-OVER-ROAD':'#795548','OBS-TREE-001':'#c62828','OBS-POLE-001':'#6a1b9a'};
function q(p){return[ox+p[0]*s,oy-p[1]*s]}function rings(g){if(g.type==='Polygon')return[g.coordinates];if(g.type==='MultiPolygon')return g.coordinates;return[]}function line(points,c,w=1){X.save();X.strokeStyle=c;X.lineWidth=w;X.beginPath();points.forEach((p,i)=>{const a=q(p);i?X.lineTo(...a):X.moveTo(...a)});X.stroke();X.restore()}function poly(g,c){for(const P of rings(g)){X.save();X.fillStyle=c+'24';X.strokeStyle=c;X.lineWidth=1.7;X.beginPath();for(const R of P){R.forEach((p,i)=>{const a=q(p);i?X.lineTo(...a):X.moveTo(...a)});X.closePath()}X.fill('evenodd');X.stroke();X.restore()}}function dot(p,c,r,stroke='#fff'){const a=q(p);X.save();X.fillStyle=c;X.strokeStyle=stroke;X.lineWidth=.7;X.beginPath();X.arc(a[0],a[1],r,0,Math.PI*2);X.fill();X.stroke();X.restore()}function coords(g,o=[]){if(g.type==='Point')o.push(g.coordinates);else if(g.type==='LineString')o.push(...g.coordinates);else for(const P of rings(g))for(const R of P)o.push(...R);return o}function rssiColor(v){return v==null?'#20252b':v>=-65?'#169c4b':v>=-80?'#f4b400':'#d93025'}function visibleMode(p){return(p.mode==='manual'&&$('manual').checked)||(p.mode==='auto'&&$('auto').checked)}function visibleDate(p){return(p.date==='20260915'&&$('date15').checked)||(p.date==='20260923'&&$('date23').checked)}function field(){return $('routerWifi').checked?'router':$('piWifi').checked?'pi':null}
function draw(){let r=C.getBoundingClientRect();X.clearRect(0,0,r.width,r.height);X.fillStyle='#f8fafc';X.fillRect(0,0,r.width,r.height);for(const f of F){let p=f.p;if(p.category==='mowable_area'&&$('areas').checked)poly(f.g,colors[p.asset_id]);if(p.category==='obstacle'&&p.geometry_role==='mowing_exclusion'&&$('obstacles').checked)poly(f.g,colors[p.asset_id]);if(p.category==='transition_route'&&$('routes').checked)line(f.g.coordinates,'#00897b',2);if(p.category==='access_point'&&$('points').checked)dot(f.g.coordinates,'#00acc1',5);if(p.category==='site_reference_point'&&$('points').checked)dot(f.g.coordinates,'#263238',6)}let k=field();if(k)for(const p of W)if(visibleMode(p)&&visibleDate(p))dot([p.x,p.y],rssiColor(p[k]),3,'#ffffff90')}
function fit(){let a=[];F.forEach(f=>coords(f.g,a));W.forEach(p=>a.push([p.x,p.y]));let xs=a.map(p=>p[0]),ys=a.map(p=>p[1]),r=C.getBoundingClientRect(),pad=30;s=Math.min((r.width-2*pad)/(Math.max(...xs)-Math.min(...xs)),(r.height-2*pad)/(Math.max(...ys)-Math.min(...ys)));ox=pad-Math.min(...xs)*s;oy=pad+Math.max(...ys)*s;draw()}
C.onwheel=e=>{e.preventDefault();let r=C.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,k=e.deltaY<0?1.15:1/1.15;ox=mx-(mx-ox)*k;oy=my-(my-oy)*k;s*=k;draw()};C.onpointerdown=e=>drag=[e.clientX,e.clientY,ox,oy];C.onpointermove=e=>{if(drag){ox=drag[2]+e.clientX-drag[0];oy=drag[3]+e.clientY-drag[1];draw();return}let r=C.getBoundingClientRect(),mx=e.clientX-r.left,my=e.clientY-r.top,k=field(),best=null,dist=64;if(k)for(const p of W){if(!visibleMode(p)||!visibleDate(p))continue;let a=q([p.x,p.y]),d=(a[0]-mx)**2+(a[1]-my)**2;if(d<dist){dist=d;best=p}}if(best){TIP.style.display='block';TIP.style.left=(e.clientX+12)+'px';TIP.style.top=(e.clientY+12)+'px';TIP.innerHTML=`<b>${k==='router'?'Tractor router':'Pi wlan0'}:</b> ${best[k]==null?'unavailable':best[k]+' dBm'}<br>${best.mode} · ${best.run}<br>elapsed ${best.elapsed.toFixed(2)} s`}else TIP.style.display='none'};C.onpointerleave=()=>TIP.style.display='none';C.onpointerup=()=>drag=null;document.querySelectorAll('input').forEach(e=>e.onchange=draw);$('fit').onclick=fit;window.onresize=()=>{let r=C.getBoundingClientRect(),d=devicePixelRatio||1;C.width=r.width*d;C.height=r.height*d;X.setTransform(d,0,0,d,0,0);fit()};window.onresize();
</script></body></html>'''
    OUT_HTML.write_text(
        template.replace("__PAYLOAD__", payload).replace("__ROUTER_MIN__", str(int(router_min))),
        encoding="utf-8",
        newline="\n",
    )


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    features, points, report = load_data()
    router_min = min(
        details["tractor_router_wifi"]["minimum_dbm"]
        for details in report["combined_by_mode"].values()
        if details["tractor_router_wifi"]["minimum_dbm"] is not None
    )
    pi_gap_count = sum(len(run["pi_wifi_missing_intervals"]) for run in report["runs"])
    pi_gap_seconds = sum(
        interval["duration_s"]
        for run in report["runs"]
        for interval in run["pi_wifi_missing_intervals"]
    )
    OUT_REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    write_html(features, points, report)
    OUT_README.write_text(
        "# 62 Collins Wi-Fi coverage review\n\n"
        "This evidence layer combines the revision-2 site inventory with the September 15 multi-area "
        "field log and four September 23 field telemetry logs. Open `62_Collins_wifi_coverage_review_20260923.html` "
        "to switch between tractor-router and Pi wlan0 RSSI and between manual and auto samples.\n\n"
        "## Finding\n\n"
        f"The recorded tractor-router values were no worse than {router_min:.0f} dBm during active manual/auto rows. "
        "That is encouraging, but it does not prove uninterrupted connectivity because the logger stores "
        "the last router value without a receive timestamp or age. Across the included sessions, the Pi "
        f"wlan0 field had {pi_gap_count} active-mode unavailable intervals totaling {pi_gap_seconds:.2f} seconds.\n\n"
        "This folder is analysis output only. It does not change the authoritative site inventory or create "
        "a launchable mission. See `wifi_coverage_report_20260923.json` for source hashes, per-run statistics, "
        "missing intervals, limitations, and recommended instrumentation.\n",
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps({
        "html": str(OUT_HTML.relative_to(REPO)).replace("\\", "/"),
        "report": str(OUT_REPORT.relative_to(REPO)).replace("\\", "/"),
        "map_samples": len(points),
        "status": report["status"],
    }, indent=2))


if __name__ == "__main__":
    main()
