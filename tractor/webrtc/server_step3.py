#!/usr/bin/env python3
"""
server_step3.py — WebRTC video + real telemetry (UDP) + bidirectional controls (DepthAI v2.x)

- Video: OAK preview (640x360@30) over WebRTC (VP8/H264)
- Telemetry: UDP JSON on 0.0.0.0:6002 forwarded to browser via DataChannel "telemetry"
- Controls: Browser sends JSON on DataChannel "control" -> PCA9685 servos
    * speed_pos: 1..10 (integer), 4 = neutral (matches Tkinter mapping)
    * steering: -1.0..1.0 (float) (-1 left, +1 right)
    * eStop: true -> sets speed to neutral and steering to 0.0
"""

import asyncio, json, signal, time, threading, socket
from fractions import Fraction
from aiohttp import web
import numpy as np
import depthai as dai
import av

# --- Servo control (PCA9685) -------------------------------------------------
HAVE_SERVOS = True
try:
    import board, busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo
except Exception as e:
    print("[SERVOS] Libraries not available, running in DRY-RUN:", e)
    HAVE_SERVOS = False

SPEED_CH = 0
STEER_CH = 1
SPEED_NEUTRAL_POS = 4  # 1..10 scale

class ServoController:
    def __init__(self):
        self.enabled = False
        self.speed_servo = None
        self.steering_servo = None
        if HAVE_SERVOS:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                pca = PCA9685(i2c)
                pca.frequency = 50
                self.speed_servo = servo.Servo(pca.channels[SPEED_CH], min_pulse=500, max_pulse=2500)
                self.steering_servo = servo.Servo(pca.channels[STEER_CH], min_pulse=500, max_pulse=2500)
                self.enabled = True
                print("[SERVOS] PCA9685 ready (ch0=speed, ch1=steer)")
            except Exception as e:
                print("[SERVOS] Init failed, continuing in DRY-RUN:", e)
                self.enabled = False

    @staticmethod
    def _speed_pos_to_angle(position:int)->float:
        # Matches the Tkinter mapping:
        #  position<=4: reverse, 1->0°, 4->90° (each step 30°)
        #  position>=5: forward, 5->120°, 10->180° (each step 15° from 90)
        if position <= 4:
            angle = 90 + ((position - 4) * 30)  # steps -30°
        else:
            angle = 90 + ((position - 4) * 15)  # steps +15°
        return max(0, min(180, angle))

    @staticmethod
    def _steering_to_angle(val:float)->float:
        # -1 -> 0°, 0 -> 90°, +1 -> 180°
        return max(0, min(180, (val + 1.0) * 90.0))

    def set_speed_pos(self, position:int):
        angle = self._speed_pos_to_angle(int(position))
        if self.enabled:
            try: self.speed_servo.angle = angle
            except Exception as e: print("[SERVOS] speed_servo error:", e)
        print(f"[SERVOS] SPEED pos {position} -> {angle:.1f}°")

    def set_steering(self, val:float):
        angle = self._steering_to_angle(float(val))
        if self.enabled:
            try: self.steering_servo.angle = angle
            except Exception as e: print("[SERVOS] steering_servo error:", e)
        print(f"[SERVOS] STEER {val:.2f} -> {angle:.1f}°")

    def emergency_stop(self):
        print("[SERVOS] E-STOP -> neutral + straight")
        self.set_speed_pos(SPEED_NEUTRAL_POS)
        self.set_steering(0.0)

SERVOS = ServoController()

# --- DepthAI video -----------------------------------------------------------
FRAME_W, FRAME_H, FPS = 640, 360, 30

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

# --- UDP telemetry receiver ---------------------------------------------------
UDP_PORT = 6002
UDP_BIND = "0.0.0.0"

class UdpTelemetryReceiver:
    def __init__(self, bind=UDP_BIND, port=UDP_PORT):
        self.bind=bind; self.port=port
        self.sock=None; self.running=False
        self.latest=None; self.latest_ts=0.0
        self.thread=None
    def start(self):
        if self.running: return
        self.running=True
        t=threading.Thread(target=self._run, daemon=True)
        t.start(); self.thread=t
    def _run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.bind, self.port))
            self.sock.settimeout(0.5)
            print(f"[UDP] listening on {self.bind}:{self.port}")
            while self.running:
                try:
                    data, _ = self.sock.recvfrom(65535)
                    try:
                        obj = json.loads(data.decode("utf-8", errors="ignore"))
                        self.latest = obj; self.latest_ts = time.time()
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
    def stop(self): self.running=False

telemetry_rx = UdpTelemetryReceiver()

# --- Web app (video + telemetry + controls UI) --------------------------------
INDEX_HTML = """<!doctype html><html><head><meta charset="utf-8"/>
<title>WebRTC Robot - Video + Telemetry + Controls</title>
<style>
  :root {
    --panel-bg:#1e1e1e; --text:#eee; --muted:#aaa; --accent:#64b5f6;
  }
  html,body { background:#111; color:var(--text); margin:0; font-family:system-ui, Arial; }
  .wrap { display:grid; grid-template-columns: 660px 1fr; gap:16px; padding:16px; }
  video { width:640px; height:auto; background:#000; border-radius:8px; display:block; }
  .card { background:var(--panel-bg); padding:12px; border-radius:8px; }
  h2 { margin:0 0 8px 0 }
  .row { display:flex; gap:8px; align-items:center; margin:8px 0; flex-wrap:wrap; }
  .label { color:var(--muted); min-width:120px; }
  input[type="range"] { width:100%; }
  button { background:#333; color:var(--text); padding:8px 12px; border:0; border-radius:6px; cursor:pointer; }
  button.primary { background:var(--accent); color:#000; font-weight:600; }
  pre { white-space:pre-wrap; }
  table { width:100%; border-collapse:collapse; }
  th, td { text-align:left; padding:6px 8px; border-bottom:1px solid #333; }
  th { color:var(--muted); font-weight:600; width:180px; }
</style>
</head><body>
  <div class="wrap">
    <div>
      <h2>WebRTC Robot</h2>
      <video id="remoteVideo" playsinline autoplay></video>
      <div class="card" id="logs"><pre id="log"></pre></div>
    </div>

    <div class="card">
      <h3>Controls</h3>
      <div class="row">
        <div class="label">Steering (-1 .. +1)</div>
        <input id="steer" type="range" min="-1" max="1" value="0" step="0.01"/>
        <div id="steerVal">0.00</div>
      </div>
      <div class="row">
        <div class="label">Speed Position</div>
        <div id="speedBtns"></div>
      </div>
      <div class="row">
        <button id="neutral">Neutral</button>
        <button id="estop" class="primary">E-Stop</button>
      </div>

      <h3>Telemetry</h3>
      <table>
        <tr><th>RTK Status</th>   <td id="t_rtk">--</td></tr>
        <tr><th>Battery (V)</th> <td id="t_batt">--</td></tr>
        <tr><th>Speed (m/s)</th> <td id="t_speed">--</td></tr>
        <tr><th>Heading (°)</th> <td id="t_head">--</td></tr>
      </table>
    </div>
  </div>

<script>
const logEl = document.getElementById('log');
const log = (...a)=>{ console.log(...a); logEl.textContent += a.join(' ') + "\\n"; };

const ws = new WebSocket('ws://' + location.host + '/ws');
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

// --- DataChannels ---
let ctrl = null;
pc.ondatachannel = (e) => {
  if (e.channel.label === "telemetry") {
    e.channel.onopen    = ()=> log('telemetry: open');
    e.channel.onmessage = (m)=> { updateTelemetry(m.data); };
    e.channel.onclose   = ()=> log('telemetry: close');
  }
};

// Create local DataChannel "control" BEFORE createOffer so SDP has a data m-line
ctrl = pc.createDataChannel('control');
ctrl.onopen  = ()=> log('control: open');
ctrl.onclose = ()=> log('control: close');

ws.onopen = async () => {
  log('ws open');
  pc.addTransceiver('video', { direction: 'recvonly' });
  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);
  log('local offer created');
  ws.send(JSON.stringify({ type: offer.type, sdp: offer.sdp }));
};
ws.onmessage = async (evt) => {
  const data = JSON.parse(evt.data);
  log('ws message:', data.type);
  if (data.type === 'answer') {
    await pc.setRemoteDescription({type:'answer', sdp:data.sdp});
    log('remote answer set');
  }
};
ws.onerror = e => log('ws error', e && e.message ? e.message : e);
ws.onclose  = () => log('ws closed');

// --- UI wire-up ---
const steer = document.getElementById('steer');
const steerVal = document.getElementById('steerVal');
steer.addEventListener('input', ()=>{
  steerVal.textContent = (+steer.value).toFixed(2);
  sendCtrl({ steering: +steer.value });
});

const speedBtns = document.getElementById('speedBtns');
for (let i=1;i<=10;i++){
  const b = document.createElement('button');
  b.textContent = i;
  b.style.minWidth = '36px';
  b.onclick = () => sendCtrl({ speed_pos: i });
  speedBtns.appendChild(b);
}
document.getElementById('neutral').onclick = ()=> sendCtrl({ speed_pos: 4, steering: 0.0 });
document.getElementById('estop').onclick   = ()=> sendCtrl({ eStop: true });

function sendCtrl(obj){
  if (ctrl && ctrl.readyState === 'open'){
    ctrl.send(JSON.stringify(obj));
  } else {
    log('control channel not open yet');
  }
}

// --- Telemetry mapping to 4 simple fields ---
function updateTelemetry(jsonText){
  try{
    const obj = JSON.parse(jsonText);
    // Map keys from your UDP payloads to 4 fields
    const rtk = obj.fix_quality || (obj.headValid ? (obj.carrier || 'RTK') : 'No Fix');
    const batt = coalesce(obj.battery_v, obj.batteryVoltage, obj.batt_v);
    const speed = coalesce(obj.speed_mps, obj.actual_speed, obj.speed);
    const heading = coalesce(obj.heading_deg, obj.heading);

    setText('t_rtk',  rtk ?? '--');
    setText('t_batt', batt != null ? batt.toFixed ? batt.toFixed(2) : batt : '--');
    setText('t_speed',speed != null ? speed.toFixed ? speed.toFixed(2) : speed : '--');
    setText('t_head', heading != null ? heading.toFixed ? heading.toFixed(1) : heading : '--');
  }catch(e){
    console.error('telemetry parse error', e, jsonText);
  }
}
function coalesce(...vals){ for (const v of vals){ if (v!==undefined && v!==null) return v; } return null; }
function setText(id, t){ document.getElementById(id).textContent = String(t); }
</script>
</body></html>
"""

pcs = set()

async def index(_): 
    return web.Response(text=INDEX_HTML, content_type="text/html")

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
            payload = {}
            now = time.time()
            latest = telemetry_rx.latest
            age = now - telemetry_rx.latest_ts if telemetry_rx.latest_ts else None
            if latest:
                payload = dict(latest)
                if age is not None and age > 1.5:
                    payload["_note"] = f"stale {age:.1f}s"
            else:
                payload = {"_note": "no telemetry yet"}
            try: tel.send(json.dumps(payload))
            except Exception: pass
            await asyncio.sleep(0.2)
    tel_task = asyncio.create_task(telemetry_task())

    # Prefer VP8 then H264
    caps = RTCRtpSender.getCapabilities("video")
    prefer = [c for c in caps.codecs if c.mimeType.lower()=="video/vp8"]
    prefer += [c for c in caps.codecs if c.mimeType.lower()=="video/h264"]
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
        print("[DC] control open")
        @channel.on("message")
        def on_message(msg):
            print("[DC] control message:", msg)
            try:
                obj = json.loads(msg)
            except Exception:
                print("[DC] control got non-JSON:", msg)
                return
            if obj.get("eStop"):
                SERVOS.emergency_stop()
            if "speed_pos" in obj:
                pos = int(obj["speed_pos"])
                pos = max(1,min(10,pos))
                SERVOS.set_speed_pos(pos)
            if "steering" in obj:
                val = float(obj["steering"])
                val = max(-1.0, min(1.0, val))
                SERVOS.set_steering(val)

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

# Start UDP listener and aiohttp
async def _main():
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
