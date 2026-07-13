#!/usr/bin/env python3
"""
server_step2.py — WebRTC video + one-way telemetry from UDP JSON (DepthAI v2.x)

Inside your venv:
  pip install --upgrade pip
  pip install aiortc aiohttp av depthai "numpy<2"

Run:
  source ~/webrtc-robot/bin/activate
  python ~/tractor2025/tractor/webrtc/server_step2.py
Open:
  http://<PI_IP>:8080/
"""

import asyncio, json, signal, time, threading, socket
from fractions import Fraction
from aiohttp import web
import numpy as np
import depthai as dai
import av
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.mediastreams import VideoStreamTrack
from aiortc.rtcrtpsender import RTCRtpSender

FRAME_W, FRAME_H, FPS = 640, 360, 30
UDP_PORT = 6002          # ← matches your Tkinter client
UDP_BIND = "0.0.0.0"     # listen on all interfaces

# --------------------------- DepthAI video (v2) ---------------------------

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

class OakVideoTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.dev = None; self.q = None
        self.ts = 0; self.time_base = Fraction(1, FPS)
        self._frames=0; self._t0=time.time()
        p = make_pipeline()
        self.dev = dai.Device(p)  # starts pipeline
        self.q = self.dev.getOutputQueue("preview", maxSize=4, blocking=True)
        print("[DepthAI] preview queue ready")

    def _get_bgr(self):
        pkt = self.q.get()
        return None if pkt is None else pkt.getCvFrame()

    async def recv(self):
        bgr = await asyncio.get_running_loop().run_in_executor(None, self._get_bgr)
        if bgr is None:
            bgr = np.zeros((FRAME_H, FRAME_W, 3), np.uint8)

        self._frames += 1
        if self._frames % 60 == 0:
            dt=time.time()-self._t0; fps=self._frames/dt if dt>0 else 0
            print(f"[DepthAI] sent frames ~{fps:.1f} fps"); self._frames=0; self._t0=time.time()

        vf = av.VideoFrame.from_ndarray(bgr, format="bgr24")
        vf.pts = self.ts; vf.time_base = self.time_base; self.ts += 1
        return vf

    def close(self):
        try:
            if self.dev: self.dev.close()
        except: pass

# --------------------------- UDP telemetry receiver ---------------------------

class UdpTelemetryReceiver:
    """
    Listens for UDP JSON packets on UDP_BIND:UDP_PORT.
    Keeps the latest parsed dict and a timestamp.
    """
    def __init__(self, bind=UDP_BIND, port=UDP_PORT):
        self.bind = bind; self.port = port
        self.sock = None
        self.latest = None
        self.latest_ts = 0.0
        self.running = False
        self.thread = None

    def start(self):
        if self.running: return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.bind, self.port))
            self.sock.settimeout(0.5)
            print(f"[UDP] listening on {self.bind}:{self.port}")
            while self.running:
                try:
                    data, _addr = self.sock.recvfrom(65535)
                    try:
                        obj = json.loads(data.decode("utf-8", errors="ignore"))
                        self.latest = obj
                        self.latest_ts = time.time()
                    except Exception as e:
                        print("[UDP] JSON parse error:", e)
                except socket.timeout:
                    pass
        except Exception as e:
            print("[UDP] socket error:", e)
        finally:
            try:
                if self.sock: self.sock.close()
            except: pass

    def stop(self):
        self.running = False

telemetry_rx = UdpTelemetryReceiver()

# --------------------------- Web app (video + telemetry DC) ---------------------------

INDEX_HTML = """<!doctype html><html><head><meta charset="utf-8"/>
<title>WebRTC Robot - Video + Telemetry</title>
<style>
body{font-family:system-ui,Arial;margin:0;padding:1rem;background:#111;color:#eee}
video{width:100%;max-width:960px;background:#000}
.row{display:flex;gap:1rem;max-width:960px;margin-top:.75rem}
pre{flex:1;background:#222;padding:.75rem;white-space:pre-wrap}
</style></head><body>
<h2>WebRTC Robot - Video + Telemetry</h2>
<video id="remoteVideo" playsinline autoplay></video>
<div class="row">
  <pre id="log"></pre>
  <pre id="telemetry">waiting…</pre>
</div>
<script>
const logEl = document.getElementById('log');
const telEl = document.getElementById('telemetry');
const log = (...a)=>{ console.log(...a); logEl.textContent += a.join(' ') + "\\n"; };

const ws = new WebSocket(`ws://`+location.host+`/ws`);
const pc = new RTCPeerConnection({ iceServers: [{urls:'stun:stun.l.google.com:19302'}] });

pc.onicegatheringstatechange = ()=> log('iceGatheringState:', pc.iceGatheringState);
pc.oniceconnectionstatechange = ()=> log('iceConnectionState:', pc.iceConnectionState);
pc.onconnectionstatechange = ()=> log('connectionState:', pc.connectionState);
pc.onsignalingstatechange = ()=> log('signalingState:', pc.signalingState);
pc.onicecandidate = e => log('icecandidate:', e.candidate ? 'have' : 'end');

pc.ontrack = e => {
  const v = document.getElementById('remoteVideo');
  v.muted = true; v.srcObject = e.streams[0];
  v.onloadedmetadata = ()=> v.play().catch(err=>log('video.play error', err));
  log('ontrack: got remote stream');
};

// Force a DATA m-line in the browser's offer
let probe = null;
ws.onopen = async () => {
  log('ws open');
  pc.addTransceiver('video', { direction: 'recvonly' });
  probe = pc.createDataChannel('probe');
  probe.onopen = () => { log('probe open'); probe.close(); };

  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);
  log('local offer created');
  ws.send(JSON.stringify({ type: offer.type, sdp: offer.sdp }));
};

pc.ondatachannel = (e) => {
  const ch = e.channel;
  log('ondatachannel:', ch.label);
  if (ch.label === "telemetry") {
    ch.onopen = () => log('telemetry: open');
    ch.onmessage = (m) => { telEl.textContent = m.data; };
    ch.onclose = () => log('telemetry: close');
  }
};

ws.onmessage = async (evt) => {
  const data = JSON.parse(evt.data);
  log('ws message:', data.type);
  if (data.type === 'answer') {
    await pc.setRemoteDescription({type:'answer', sdp:data.sdp});
    log('remote answer set');
  }
};
ws.onerror = e => log('ws error', e?.message || e);
ws.onclose  = () => log('ws closed');
</script></body></html>
"""

pcs = set()

async def index(_): return web.Response(text=INDEX_HTML, content_type="text/html")

async def ws_handler(request):
    ws = web.WebSocketResponse(); await ws.prepare(request)
    pc = RTCPeerConnection(); pcs.add(pc)

    # Video
    video = OakVideoTrack()
    sender = pc.addTrack(video)

    # Prefer VP8 then H264
    caps = RTCRtpSender.getCapabilities("video")
    prefer = [c for c in caps.codecs if c.mimeType.lower()=="video/vp8"]
    prefer += [c for c in caps.codecs if c.mimeType.lower()=="video/h264"]
    prefer += [c for c in caps.codecs if c.mimeType.lower() not in ("video/vp8","video/h264")]
    try: sender.setCodecPreferences(prefer); print("[WebRTC] codec order:", [c.mimeType for c in prefer])
    except Exception as e: print("[WebRTC] setCodecPreferences failed:", e)

    # Server-created telemetry channel
    tel = pc.createDataChannel("telemetry")
    @tel.on("open")
    def _open(): print("[DC] telemetry open")
    @tel.on("close")
    def _close(): print("[DC] telemetry close")

    running = True
    async def telemetry_task():
        # Wait for channel to open
        while running and tel.readyState != "open":
            await asyncio.sleep(0.05)
        print("[DC] telemetry task started")

        # Send latest UDP JSON ~5 Hz
        while running and pc.connectionState not in ("failed","closed","disconnected"):
            payload = {}
            now = time.time()
            latest = telemetry_rx.latest
            age = now - telemetry_rx.latest_ts if telemetry_rx.latest_ts else None
            if latest:
                payload = dict(latest)  # pass-through keys from your sender
                if age is not None and age > 1.5:
                    payload["_note"] = f"stale {(age):.1f}s"
            else:
                payload = {"_note": "no telemetry received yet"}
            try:
                tel.send(json.dumps(payload))
            except Exception:
                pass
            await asyncio.sleep(0.2)
        print("[DC] telemetry task stopped")

    ttask = asyncio.create_task(telemetry_task())

    @pc.on("connectionstatechange")
    async def _on_state():
        nonlocal running
        print("Peer connection state:", pc.connectionState)
        if pc.connectionState in ("failed","closed","disconnected"):
            running = False
            ttask.cancel()
            video.close()

    try:
        async for msg in ws:
            if msg.type != web.WSMsgType.TEXT: continue
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
        running = False; ttask.cancel(); video.close()
    return ws

async def _main():
    # start UDP listener
    telemetry_rx.start()

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    runner = web.AppRunner(app); await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", 8080); await site.start()
    print("Listening on http://0.0.0.0:8080")

    stop = asyncio.Future()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try: loop.add_signal_handler(sig, stop.set_result, True)
        except NotImplementedError: pass
    await stop
    telemetry_rx.stop()
    await runner.cleanup()

if __name__ == "__main__":
    asyncio.run(_main())
