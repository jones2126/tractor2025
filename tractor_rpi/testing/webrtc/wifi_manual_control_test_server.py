#!/usr/bin/env python3
"""ZeroTier test server for wifi_manual_control_prototype.html.

This intentionally does not connect to tractor hardware. It serves the page,
accepts bounded sample control JSON, prints it, and returns simulated status.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


PAGE_PATH = Path(__file__).with_name("wifi_manual_control_prototype.html")
VALID_MODES = {"auto", "manual", "pause"}


class TestState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.started = time.monotonic()
        self.sequence = 0
        self.last_command: dict[str, Any] | None = None
        self.last_command_at: float | None = None

    def accept_command(self, command: dict[str, Any]) -> None:
        with self.lock:
            self.last_command = command
            self.last_command_at = time.monotonic()

    def telemetry(self) -> dict[str, Any]:
        with self.lock:
            self.sequence += 1
            sequence = self.sequence
            age_ms = None if self.last_command_at is None else round((time.monotonic() - self.last_command_at) * 1000)

        elapsed = time.monotonic() - self.started
        wifi_rssi = round(-59 - 10 * (0.5 + 0.5 * math.sin(elapsed / 4.0)))
        wifi_level = "good" if wifi_rssi >= -66 else "warn"

        rtk_phase = elapsed % 24
        if rtk_phase < 16:
            rtk_status, rtk_level = "FIXED", "good"
        elif rtk_phase < 21:
            rtk_status, rtk_level = "FLOAT", "warn"
        else:
            rtk_status, rtk_level = "NO FIX", "bad"

        heading_degrees = round((elapsed * 9) % 360)
        heading_level = "warn" if int(elapsed) % 30 in (27, 28, 29) else "good"

        return {
            "type": "telemetry",
            "sequence": sequence,
            "server_time": datetime.now().astimezone().isoformat(timespec="seconds"),
            "wifi": {"rssi_dbm": wifi_rssi, "level": wifi_level},
            "rtk": {"status": rtk_status, "level": rtk_level},
            "heading": {"degrees": heading_degrees, "level": heading_level},
            "last_command_age_ms": age_ms,
        }


STATE = TestState()


MONITOR_HTML = b"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#101923"><title>Tractor Control Test Monitor</title>
<style>
body{margin:0;background:#091018;color:#f4f8fb;font:16px system-ui,sans-serif}main{max-width:900px;margin:auto;padding:24px}
h1{font-size:24px}.status{color:#42d37b}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:14px}
.card{padding:20px;background:#14202b;border:1px solid #334b5e;border-radius:14px}.label{color:#9cafbd;font-size:12px;text-transform:uppercase}
.value{margin-top:7px;color:#59b9ff;font-size:34px;font-weight:800;font-variant-numeric:tabular-nums}.details{margin-top:16px;color:#9cafbd}
@media(max-width:650px){.grid{grid-template-columns:1fr}.value{font-size:28px}}
</style></head><body><main><h1>Phone control values <span class="status" id="status">connecting...</span></h1>
<div class="grid"><div class="card"><div class="label">Mode</div><div class="value" id="mode">--</div></div>
<div class="card"><div class="label">Speed</div><div class="value" id="speed">--</div></div>
<div class="card"><div class="label">Steering</div><div class="value" id="steering">--</div></div></div>
<div class="details" id="details">Waiting for a phone command.</div></main>
<script>
const modeEl=document.getElementById('mode'),speedEl=document.getElementById('speed'),steeringEl=document.getElementById('steering');
const statusEl=document.getElementById('status'),detailsEl=document.getElementById('details');
async function update(){try{const r=await fetch('/api/state',{cache:'no-store'});const d=await r.json();const c=d.last_command;
if(c){modeEl.textContent=c.mode.toUpperCase();speedEl.textContent=(c.speed_mps>=0?'+':'')+c.speed_mps.toFixed(2)+' m/s';
steeringEl.textContent=(c.steering_percent>=0?'+':'')+c.steering_percent+'%';detailsEl.textContent='Last message: '+c.reason+' \\u2022 age '+d.age_ms+' ms';}
statusEl.textContent='LIVE';}catch(e){statusEl.textContent='DISCONNECTED';detailsEl.textContent=e.message;}}
update();setInterval(update,250);
</script></body></html>"""


def validate_command(data: Any) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise ValueError("JSON body must be an object")

    mode = str(data.get("mode", "")).lower()
    if mode not in VALID_MODES:
        raise ValueError("mode must be auto, manual, or pause")

    try:
        speed = float(data["speed_mps"])
        steering = float(data["steering_percent"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("speed_mps and steering_percent must be numbers") from exc

    if not -0.60 <= speed <= 0.60:
        raise ValueError("speed_mps is outside -0.60 to 0.60")
    if not -100 <= steering <= 100:
        raise ValueError("steering_percent is outside -100 to 100")

    return {
        "type": "control",
        "reason": str(data.get("reason", "update"))[:40],
        "mode": mode,
        "speed_mps": round(speed, 2),
        "steering_percent": round(steering),
        "client_time_ms": data.get("client_time_ms"),
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "TractorControlTest/1.0"

    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def send_bytes(self, status: HTTPStatus, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_bytes(status, "application/json; charset=utf-8", body)

    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(HTTPStatus.NO_CONTENT)
        self.end_headers()

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path in ("/", "/wifi_manual_control_prototype.html"):
            self.send_bytes(HTTPStatus.OK, "text/html; charset=utf-8", PAGE_PATH.read_bytes())
            return
        if path == "/api/telemetry":
            self.send_json(HTTPStatus.OK, STATE.telemetry())
            return
        if path == "/monitor":
            self.send_bytes(HTTPStatus.OK, "text/html; charset=utf-8", MONITOR_HTML)
            return
        if path == "/api/state":
            with STATE.lock:
                age_ms = None if STATE.last_command_at is None else round((time.monotonic() - STATE.last_command_at) * 1000)
                payload = {"last_command": STATE.last_command, "age_ms": age_ms}
            self.send_json(HTTPStatus.OK, payload)
            return
        self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        if urlparse(self.path).path != "/api/command":
            self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length <= 0 or content_length > 4096:
                raise ValueError("request body must be between 1 and 4096 bytes")
            data = json.loads(self.rfile.read(content_length).decode("utf-8"))
            command = validate_command(data)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
            self.send_json(HTTPStatus.BAD_REQUEST, {"accepted": False, "error": str(exc)})
            return

        STATE.accept_command(command)
        now = datetime.now().astimezone().strftime("%H:%M:%S")
        print(
            f"[{now}] {command['reason']:<14} mode={command['mode']:<6} "
            f"speed={command['speed_mps']:+.2f} m/s  steering={command['steering_percent']:+4.0f}%",
            flush=True,
        )
        self.send_json(HTTPStatus.OK, {"accepted": True, "command": command})

    def log_message(self, format: str, *args: Any) -> None:
        if args and str(args[1]).startswith("4"):
            super().log_message(format, *args)


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve the tractor phone-control test platform.")
    parser.add_argument(
        "--host",
        default="192.168.193.183",
        help="Listen address (default: this machine's ZeroTier address)",
    )
    parser.add_argument("--port", type=int, default=8765, help="TCP port (default: 8765)")
    args = parser.parse_args()

    if not PAGE_PATH.is_file():
        raise SystemExit(f"Missing interface page: {PAGE_PATH}")

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print("Tractor phone-control TEST server — no tractor hardware connection")
    print(f"Phone URL: http://192.168.193.183:{args.port}/")
    print(f"Listening on {args.host}:{args.port}; press Ctrl+C to stop")
    print("Received phone commands will appear below:\n")
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nStopping test server.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
