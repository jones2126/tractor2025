#!/usr/bin/env python3
"""
teleop_console.py — Tractor Teleoperation Console Server
=========================================================
Evolved from server_step7.py.  Single-process server providing two browser
interfaces:

  http://<rpi>:8080/              — Control screen (video + drive controls)
  http://<rpi>:8080/dashboard     — Telemetry dashboard (map + full diagnostics)

Architecture:
  UDP 6002 (GPS/RTK from rtcm_server) ──┐
                                         ├──> gps_rx / bridge_rx
  UDP 6003 (bridge from teensy_bridge) ──┘
       │
       ├── DataChannel  ──> Control page   (essential telemetry + video)
       └── WebSocket    ──> Dashboard page (full telemetry, no video)

  Control page ──> DataChannel cmd_vel ──> UDP 6004 ──> teensy_bridge ──> Teensy

Changes from server_step7.py:
  - NEW:     /dashboard route serves dashboard.html (file-based, not embedded)     ~Line 480
  - NEW:     /ws_telemetry WebSocket pushes full GPS+bridge JSON at ~5Hz           ~Line 500
  - NEW:     dashboard_clients set tracks connected dashboards                     ~Line 475
  - NEW:     /api/status health-check endpoint                                    ~Line 488
  - RENAMED: File from server_step7.py to teleop_console.py
  - KEPT:    All step7 control page HTML, signaling, steering, speed controls unchanged

Requirements (same as step7 — already installed):
  pip install aiortc aiohttp av depthai "numpy<2" --break-system-packages

Run:
  python ~/tractor2025/tractor/webrtc/teleop_console.py

IMPORTANT: The NRF24 radio mode switch must be in AUTO (mode 2, center position)
for cmd_vel commands to take effect on the Teensy.
"""

import asyncio, json, signal, time, threading, socket, os
from fractions import Fraction
from aiohttp import web
import numpy as np
import depthai as dai
import av

# ──────────────────────────────────────────────────────────────────────
# Configuration                                                 ~Line 45
# ──────────────────────────────────────────────────────────────────────
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# --- Tractor controller via UDP to teensy_serial_bridge ---------------
# Sends JSON {"linear_x": float, "angular_z": float} to UDP port 6004.
# The bridge converts this to serial "CMD,<x>,<z>\n" for the Teensy.
#
# On the Teensy (in auto mode=2):
#   linear_x  -> transmission bucket: -1.0=full fwd (bucket 9), 0.0=neutral (bucket 5), +1.0=full rev (bucket 0)
#   angular_z -> steering pot setpoint: 0=full left, 512=center, 1023=full right

CMD_VEL_TARGET_IP = "127.0.0.1"     # Bridge runs on same RPi
CMD_VEL_TARGET_PORT = 6004           # teensy_serial_bridge command listener

class TractorController:
    """Send cmd_vel commands via UDP to teensy_serial_bridge (port 6004)."""

    def __init__(self, ip=CMD_VEL_TARGET_IP, port=CMD_VEL_TARGET_PORT):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_linear_x = 0.0      # neutral speed
        self.last_angular_z = 512.0    # center steering
        self._send_count = 0
        print(f"[CMD] UDP command sender -> {ip}:{port}")

    def send_cmd_vel(self, linear_x: float, angular_z: float):
        """Send a raw cmd_vel command to the bridge.
        linear_x:  -1.0 (full forward) .. 0.0 (neutral) .. +1.0 (full reverse)
        angular_z: 0.0 (full left) .. 512.0 (center) .. 1023.0 (full right)
        """
        payload = json.dumps({"linear_x": linear_x, "angular_z": angular_z})
        self.sock.sendto(payload.encode(), (self.ip, self.port))
        self.last_linear_x = linear_x
        self.last_angular_z = angular_z
        self._send_count += 1
        if self._send_count % 50 == 0:
            print(f"[CMD] #{self._send_count}: linear_x={linear_x:.3f}, angular_z={angular_z:.1f}")

    def set_speed_bucket(self, bucket: int):
        """Send speed as a bucket number (0-9).
        Bucket 0 = full reverse, 5 = neutral, 9 = full forward.
        Internally converts to linear_x using the inverse of the Teensy's scaling:
          Teensy does: scaled_val = (linear_x + 1.0) * 511.5
          then maps scaled_val to bucket thresholds.
        We reverse this: pick the midpoint scaled_val for each bucket, then solve for linear_x.
        """
        bucket_midpoints = {
            0: 977, 1: 884, 2: 791, 3: 699, 4: 607,
            5: 515, 6: 422, 7: 330, 8: 238, 9: 96
        }
        bucket = max(0, min(9, int(bucket)))
        scaled = bucket_midpoints[bucket]
        linear_x = (scaled / 511.5) - 1.0
        self.send_cmd_vel(linear_x, self.last_angular_z)

    def set_steering_normalized(self, val: float):
        """Convert normalized steering (-1.0..+1.0) to pot units (0..1023).
        -1.0 = full left (pot 0), 0.0 = center (pot 512), +1.0 = full right (pot 1023).
        """
        pot_val = (val + 1.0) * 511.5
        pot_val = max(0.0, min(1023.0, pot_val))
        self.send_cmd_vel(self.last_linear_x, pot_val)

    def emergency_stop(self):
        """Set neutral speed (bucket 5) and center steering."""
        print("[CMD] E-STOP -> neutral speed + center steering")
        self.send_cmd_vel(0.0, 512.0)

TRACTOR = TractorController()

# ──────────────────────────────────────────────────────────────────────
# DepthAI video                                                ~Line 125
# ──────────────────────────────────────────────────────────────────────
# UNCHANGED from step7
FRAME_W, FRAME_H, FPS = 960, 540, 30

def make_pipeline() -> dai.Pipeline:
    p = dai.Pipeline()
    cam = p.create(dai.node.ColorCamera)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setPreviewSize(FRAME_W, FRAME_H)
    cam.setFps(FPS)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setInterleaved(False)
    cam.setPreviewKeepAspectRatio(True)
    xout = p.create(dai.node.XLinkOut); xout.setStreamName("preview")
    cam.preview.link(xout.input)
    return p

from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.mediastreams import VideoStreamTrack
from aiortc.rtcrtpsender import RTCRtpSender

class OakVideoTrack(VideoStreamTrack):
    # UNCHANGED from step7
    def __init__(self):
        super().__init__()
        self.ts = 0
        self.time_base = Fraction(1, FPS)
        self._frames = 0
        self._t0 = time.time()
        p = make_pipeline()
        self.dev = dai.Device(p)
        self.q = self.dev.getOutputQueue("preview", maxSize=4, blocking=True)
        print("[DepthAI] preview queue ready")

    def _get_bgr(self):
        pkt = self.q.get()
        return None if pkt is None else pkt.getCvFrame()

    async def recv(self):
        bgr = await asyncio.get_running_loop().run_in_executor(None, self._get_bgr)
        if bgr is None:
            bgr = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)

        self._frames += 1
        if self._frames % 60 == 0:
            dt = time.time() - self._t0
            fps = self._frames/dt if dt>0 else 0
            print(f"[DepthAI] sent frames ~{fps:.1f} fps")
            self._frames = 0; self._t0 = time.time()

        vf = av.VideoFrame.from_ndarray(bgr, format="bgr24")
        vf.pts = self.ts
        vf.time_base = self.time_base
        self.ts += 1
        return vf

    def close(self):
        try: self.dev.close()
        except Exception: pass

# ──────────────────────────────────────────────────────────────────────
# UDP telemetry receivers                                      ~Line 185
# ──────────────────────────────────────────────────────────────────────
# UNCHANGED from step7

class UdpTelemetryReceiver:
    """Generic UDP JSON listener. Keeps the latest parsed dict + timestamp."""
    def __init__(self, bind, port, label="UDP"):
        self.bind = bind; self.port = port; self.label = label
        self.sock = None; self.running = False
        self.latest = None; self.latest_ts = 0.0
        self.thread = None

    def start(self):
        if self.running: return
        self.running = True
        t = threading.Thread(target=self._run, daemon=True)
        t.start(); self.thread = t

    def _run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.bind, self.port))
            self.sock.settimeout(0.5)
            print(f"[{self.label}] listening on {self.bind}:{self.port}")
            while self.running:
                try:
                    data, _ = self.sock.recvfrom(65535)
                    try:
                        obj = json.loads(data.decode("utf-8", errors="ignore"))
                        self.latest = obj; self.latest_ts = time.time()
                    except Exception as e:
                        print(f"[{self.label}] JSON parse error:", e)
                except socket.timeout:
                    pass
        except Exception as e:
            print(f"[{self.label}] socket error:", e)
        finally:
            try:
                if self.sock: self.sock.close()
            except: pass

    def stop(self):
        self.running = False

gps_rx = UdpTelemetryReceiver("0.0.0.0", 6002, "GPS")        # GPS/RTK from rtcm_server
bridge_rx = UdpTelemetryReceiver("0.0.0.0", 6003, "BRIDGE")   # Teensy status from bridge

# ──────────────────────────────────────────────────────────────────────
# HTML pages — both served from files alongside this script    ~Line 235
# ──────────────────────────────────────────────────────────────────────
# CHANGED: Control page HTML extracted from embedded INDEX_HTML string
# to standalone control.html file.  Both pages now served consistently
# via FileResponse.

pcs = set()
dashboard_clients = set()

def _serve_html(filename):
    """Helper: return a FileResponse or a 404 with a helpful message."""
    path = os.path.join(SCRIPT_DIR, filename)
    if os.path.isfile(path):
        return web.FileResponse(path)
    return web.Response(
        text=f"{filename} not found — place it next to teleop_console.py",
        status=404)

async def index(_):                                             # ~Line 250
    return _serve_html("control.html")

async def serve_dashboard(request):                             # ~Line 255
    return _serve_html("dashboard.html")

# NEW: Health-check / debug endpoint                           ~Line 490
async def serve_status(request):
    return web.json_response({
        "connections":      len(pcs),
        "dashboards":       len(dashboard_clients),
        "cmd_vel_sent":     TRACTOR._send_count,
        "gps_latest_ts":    gps_rx.latest_ts,
        "bridge_latest_ts": bridge_rx.latest_ts,
    })

# NEW: Dashboard telemetry WebSocket (/ws_telemetry)           ~Line 502
async def ws_telemetry_handler(request):
    """Push full GPS+bridge telemetry to dashboard clients at ~5Hz.
    Dashboard is receive-only; no inbound messages expected."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    dashboard_clients.add(ws)
    print(f"[DASHBOARD] client connected ({len(dashboard_clients)} total)")

    try:
        while not ws.closed:
            now = time.time()
            payload = {}

            # GPS data from port 6002
            gps_data = gps_rx.latest
            gps_age = now - gps_rx.latest_ts if gps_rx.latest_ts else None
            if gps_data:
                gps_section = dict(gps_data)
                if gps_age is not None:
                    gps_section["_age"] = round(gps_age, 2)
                payload["gps"] = gps_section
            else:
                payload["gps"] = {"_note": "no GPS data"}

            # Bridge data from port 6003
            bridge_data = bridge_rx.latest
            bridge_age = now - bridge_rx.latest_ts if bridge_rx.latest_ts else None
            if bridge_data:
                bridge_section = dict(bridge_data)
                if bridge_age is not None:
                    bridge_section["_age"] = round(bridge_age, 2)
                payload["bridge"] = bridge_section
            else:
                payload["bridge"] = {"_note": "no bridge data"}

            payload["timestamp"] = round(now, 3)
            payload["gps_age"] = round(gps_age, 2) if gps_age else None
            payload["bridge_age"] = round(bridge_age, 2) if bridge_age else None

            try:
                await ws.send_json(payload)
            except Exception:
                break

            await asyncio.sleep(0.2)  # ~5 Hz — matches control page telemetry rate
    finally:
        dashboard_clients.discard(ws)
        print(f"[DASHBOARD] client disconnected ({len(dashboard_clients)} remaining)")

    return ws

# Control page WebSocket handler — UNCHANGED from step7         ~Line 555
async def ws_handler(request):
    ws = web.WebSocketResponse(); await ws.prepare(request)
    pc = RTCPeerConnection(); pcs.add(pc)

    # Video
    video = OakVideoTrack()
    sender = pc.addTrack(video)

    # Telemetry channel (server -> browser)
    tel = pc.createDataChannel("telemetry")
    async def telemetry_task():
        while tel.readyState != "open" and pc.connectionState not in ("failed","closed","disconnected"):
            await asyncio.sleep(0.05)
        print("[DC] telemetry open; starting feed")
        while pc.connectionState not in ("failed","closed","disconnected"):
            now = time.time()
            payload = {}

            # GPS data from port 6002
            gps_data = gps_rx.latest
            gps_age = now - gps_rx.latest_ts if gps_rx.latest_ts else None
            if gps_data:
                gps_section = dict(gps_data)
                if gps_age is not None and gps_age > 2.0:
                    gps_section["_stale"] = f"{gps_age:.1f}s"
                payload["gps"] = gps_section
            else:
                payload["gps"] = {"_note": "no GPS data"}

            # Bridge data from port 6003
            bridge_data = bridge_rx.latest
            bridge_age = now - bridge_rx.latest_ts if bridge_rx.latest_ts else None
            if bridge_data:
                bridge_section = dict(bridge_data)
                if bridge_age is not None and bridge_age > 2.0:
                    bridge_section["_stale"] = f"{bridge_age:.1f}s"
                payload["bridge"] = bridge_section
            else:
                payload["bridge"] = {"_note": "no bridge data"}

            try: tel.send(json.dumps(payload))
            except Exception: pass
            await asyncio.sleep(0.2)   # ~5 Hz
    tel_task = asyncio.create_task(telemetry_task())

    # Prefer H264 then VP8
    caps = RTCRtpSender.getCapabilities("video")
    prefer = [c for c in caps.codecs if c.mimeType.lower()=="video/h264"]
    prefer += [c for c in caps.codecs if c.mimeType.lower()=="video/vp8"]
    prefer += [c for c in caps.codecs if c.mimeType.lower() not in ("video/vp8","video/h264")]
    try:
        sender.setCodecPreferences(prefer)
        print("[WebRTC] codec order:", [c.mimeType for c in prefer])
    except Exception as e:
        print("[WebRTC] setCodecPreferences failed:", e)

    # Receive control channel from browser
    @pc.on("datachannel")
    def on_dc(channel):
        if channel.label != "control":
            return
        print("[DC] control channel open")
        @channel.on("message")
        def on_message(msg):
            try:
                obj = json.loads(msg)
            except Exception:
                print("[DC] control: invalid JSON:", msg)
                return
            # E-Stop: neutral speed + center steering
            if obj.get("eStop"):
                TRACTOR.emergency_stop()
            # Speed: UI sends position 1-10, we map to bucket 0-9
            if "speed_pos" in obj:
                pos = max(1, min(10, int(obj["speed_pos"])))
                TRACTOR.set_speed_bucket(pos - 1)
            # Steering: UI sends -1.0..+1.0
            if "steering" in obj:
                val = max(-1.0, min(1.0, float(obj["steering"])))
                TRACTOR.set_steering_normalized(val)

    @pc.on("connectionstatechange")
    async def _on_state():
        print("Peer connection state:", pc.connectionState)
        if pc.connectionState in ("failed","closed","disconnected"):
            video.close()
            tel_task.cancel()

    try:
        async for msg in ws:
            if msg.type != web.WSMsgType.TEXT:
                continue
            data = json.loads(msg.data)
            if data.get("type") == "offer" and "sdp" in data:
                print("Got SDP offer from client")
                await pc.setRemoteDescription(RTCSessionDescription(sdp=data["sdp"], type="offer"))
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                print("Sending SDP answer to client")
                await ws.send_json({"type": pc.localDescription.type, "sdp": pc.localDescription.sdp})
    finally:
        await ws.close(); await pc.close(); pcs.discard(pc)
        video.close()
        try: tel_task.cancel()
        except: pass
    return ws

# ──────────────────────────────────────────────────────────────────────
# Application lifecycle                                        ~Line 660
# ──────────────────────────────────────────────────────────────────────
async def _main():
    # Start both UDP listeners
    gps_rx.start()
    bridge_rx.start()

    app = web.Application()
    app.router.add_get("/",               index)                # Control page (UNCHANGED)
    app.router.add_get("/ws",             ws_handler)           # Control WebRTC signaling (UNCHANGED)
    app.router.add_get("/dashboard",      serve_dashboard)      # NEW: Dashboard page
    app.router.add_get("/ws_telemetry",   ws_telemetry_handler) # NEW: Dashboard telemetry WS
    app.router.add_get("/api/status",     serve_status)         # NEW: Health check

    runner = web.AppRunner(app); await runner.setup()
    site = web.TCPSite(runner, HTTP_HOST, HTTP_PORT); await site.start()

    print("═══════════════════════════════════════════════════════")
    print(f"  Teleop Console — http://0.0.0.0:{HTTP_PORT}")
    print(f"    Control:   http://<rpi>:{HTTP_PORT}/")
    print(f"    Dashboard: http://<rpi>:{HTTP_PORT}/dashboard")
    print(f"    Status:    http://<rpi>:{HTTP_PORT}/api/status")
    print(f"  GPS telemetry:    UDP 6002 (from rtcm_server)")
    print(f"  Bridge telemetry: UDP 6003 (from teensy_serial_bridge)")
    print(f"  Cmd_vel output:   UDP 6004 (to teensy_serial_bridge)")
    print("═══════════════════════════════════════════════════════")

    stop = asyncio.Future()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try: loop.add_signal_handler(sig, stop.set_result, True)
        except NotImplementedError: pass
    await stop
    gps_rx.stop()
    bridge_rx.stop()
    await runner.cleanup()

if __name__ == "__main__":
    asyncio.run(_main())
