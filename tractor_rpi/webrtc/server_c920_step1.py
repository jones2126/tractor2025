#!/usr/bin/env python3
"""
server_c920.py — WebRTC video-only demo for Logitech C920 USB Camera

Inside your venv:
  pip install --upgrade pip
  pip install aiortc aiohttp av opencv-python "numpy<2"
Run:
  source ~/webrtc-robot/bin/activate  # if using venv
  python ~/tractor2025/tractor/webrtc/server_c920.py
Open:
  http://<YOUR_IP>:8080/
Test camera: python -c "import cv2; cap=cv2.VideoCapture(0); ret,frame=cap.read(); print('Frame shape:', frame.shape if ret else 'Failed')"
"""

import asyncio, json, signal, time
from fractions import Fraction

from aiohttp import web
import numpy as np
import cv2  # MODIFIED: For USB camera capture
import av

from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.mediastreams import VideoStreamTrack
from aiortc.rtcrtpsender import RTCRtpSender

FRAME_W, FRAME_H, FPS = 640, 360, 30  # Same as original

# --------------------------- C920 Camera (OpenCV) ---------------------------  # MODIFIED

class C920VideoTrack(VideoStreamTrack):  # MODIFIED: Renamed and adapted
    """
    Pull frames from Logitech C920 via OpenCV.
    Non-blocking read, but wrapped for async.
    """
    def __init__(self):
        super().__init__()
        self._cap = None  # MODIFIED: cv2.VideoCapture
        self._frames = 0
        self._t0 = time.time()
        self._ts = 0
        self._time_base = Fraction(1, FPS)
        self._start_opencv()

    def _start_opencv(self):  # MODIFIED
        self._cap = cv2.VideoCapture(0)  # 0 = default camera (C920)
        if not self._cap.isOpened():
            raise RuntimeError("Failed to open camera. Check connection/index.")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        self._cap.set(cv2.CAP_PROP_FPS, FPS)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
        print("[C920] Camera opened at {}x{}@{}fps".format(FRAME_W, FRAME_H, FPS))

    def _read_frame(self):  # MODIFIED: Non-blocking cv2 read
        if self._cap is None:
            return None
        ret, bgr = self._cap.read()  # Returns (bool, frame) in BGR
        if not ret:
            return None
        # Resize if needed (C920 might not honor set exactly)
        bgr = cv2.resize(bgr, (FRAME_W, FRAME_H))
        return bgr

    async def recv(self):
        loop = asyncio.get_running_loop()
        bgr = await loop.run_in_executor(None, self._read_frame)

        if bgr is None:
            bgr = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)  # Black frame fallback

        self._frames += 1
        if self._frames % 60 == 0:
            dt = time.time() - self._t0
            fps = self._frames / dt if dt > 0 else 0
            print(f"[C920] sent frames ~{fps:.1f} fps")
            self._frames = 0
            self._t0 = time.time()

        vf = av.VideoFrame.from_ndarray(bgr, format="bgr24")
        vf.pts = self._ts
        vf.time_base = self._time_base
        self._ts += 1
        return vf

    def close(self):  # MODIFIED
        try:
            if self._cap:
                self._cap.release()
                print("[C920] Camera released")
        except Exception as e:
            print("[C920] close() exception:", e)

# --------------------------- Web server ---------------------------  # Unchanged except track reference

INDEX_HTML = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>WebRTC Robot - Video Only (C920)</title>  <!-- MODIFIED: Title -->
  <style>
    body{{font-family:system-ui,Arial;margin:0;padding:1rem;background:#111;color:#eee}}
    video{{width:100%;max-width:960px;background:#000}}
    #log{{max-width:960px;white-space:pre-wrap;background:#222;padding:.75rem;margin-top:.75rem}}
  </style>
</head>
<body>
  <h2>WebRTC Robot - Video Only (C920)</h2>  <!-- MODIFIED: Header -->
  <video id="remoteVideo" playsinline autoplay></video>
  <pre id="log"></pre>
  <script>
    const logEl = document.getElementById('log');
    const log = (...a)=>{{console.log(...a); logEl.textContent += a.join(' ') + "\\n";}};
    const ws = new WebSocket(`ws://`+location.host+`/ws`);

    const pc = new RTCPeerConnection({{ iceServers: [{{urls:'stun:stun.l.google.com:19302'}}] }});
    pc.onicegatheringstatechange = ()=> log('iceGatheringState:', pc.iceGatheringState);
    pc.oniceconnectionstatechange = ()=> log('iceConnectionState:', pc.iceConnectionState);
    pc.onconnectionstatechange = ()=> log('connectionState:', pc.connectionState);
    pc.onsignalingstatechange = ()=> log('signalingState:', pc.signalingState);
    pc.onicecandidate = e => log('icecandidate:', e.candidate ? 'have' : 'end');

    pc.ontrack = e => {{
      log('ontrack: got remote stream; kind=', e.track.kind);
      const v = document.getElementById('remoteVideo');
      v.muted = true;
      v.srcObject = e.streams[0];
      v.onloadedmetadata = () => {{ v.play().catch(err=>log('video.play error', err)); }};
    }};

    ws.onopen = async () => {{
      log('ws open');
      pc.addTransceiver('video', {{ direction: 'recvonly' }});
      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);
      log('local offer created');
      ws.send(JSON.stringify({{type: offer.type, sdp: offer.sdp}}));
    }};

    ws.onmessage = async (evt) => {{
      const data = JSON.parse(evt.data);
      log('ws message:', data.type);
      if (data.type === 'answer') {{
        await pc.setRemoteDescription({{type:'answer', sdp:data.sdp}});
        log('remote answer set');
      }}
    }};
    ws.onerror = e => log('ws error', e?.message || e);
    ws.onclose = () => log('ws closed');
  </script>
</body>
</html>"""

pcs = set()

async def index(_):
    return web.Response(text=INDEX_HTML, content_type="text/html")

async def ws_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    pc = RTCPeerConnection()
    pcs.add(pc)

    # attach C920 track  # MODIFIED
    video = C920VideoTrack()
    sender = pc.addTrack(video)

    # Force VP8 first (software-friendly), then H264, then anything else
    caps = RTCRtpSender.getCapabilities("video")
    prefer = []
    prefer += [c for c in caps.codecs if c.mimeType.lower() == "video/vp8"]
    prefer += [c for c in caps.codecs if c.mimeType.lower() == "video/h264"]
    prefer += [c for c in caps.codecs if c.mimeType.lower() not in ("video/vp8","video/h264")]
    try:
        sender.setCodecPreferences(prefer)
        print("[WebRTC] codec order:", [c.mimeType for c in prefer])
    except Exception as e:
        print("[WebRTC] setCodecPreferences failed:", e)

    @pc.on("connectionstatechange")
    async def _on_state():
        print("Peer connection state:", pc.connectionState)
        if pc.connectionState in ("failed","closed","disconnected"):
            video.close()

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
        await ws.close()
        await pc.close()
        pcs.discard(pc)
        video.close()
    return ws

async def _main():
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", 8080)
    await site.start()
    print("Listening on http://0.0.0.0:8080")

    stop = asyncio.Future()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try: loop.add_signal_handler(sig, stop.set_result, True)
        except NotImplementedError: pass
    await stop
    await runner.cleanup()

if __name__ == "__main__":
    asyncio.run(_main())