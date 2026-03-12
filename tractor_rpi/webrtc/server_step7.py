#!/usr/bin/env python3
"""
server_step7.py — WebRTC video + telemetry + tractor control via teensy_serial_bridge

CHANGED FROM STEP 6:
  - REMOVED: PCA9685 servo imports and ServoController class (lines ~17-80 of step6)
  - NEW:     TractorController class sends UDP cmd_vel to teensy_serial_bridge port 6004
  - CHANGED: DataChannel "control" handler calls TRACTOR instead of SERVOS
  - CHANGED: Telemetry also listens on port 6003 (bridge status) in addition to 6002 (GPS)
  - NEW:     Telemetry table shows bridge data (radio signal, steering, transmission mode)
  - CHANGED: Speed UI labels updated to match bucket system (bucket 5 = neutral)

Command path:
  Browser -> DataChannel "control" -> TractorController.send_cmd_vel()
    -> UDP port 6004 -> teensy_serial_bridge.py -> Serial CMD,x,z -> Teensy

Requirements (same venv as step6):
  pip install aiortc aiohttp av depthai "numpy<2"

Run:
  source ~/webrtc-robot/bin/activate
  python ~/tractor2025/tractor/webrtc/server_step7.py

IMPORTANT: The NRF24 radio mode switch must be in AUTO (mode 2, center position)
for cmd_vel commands to take effect on the Teensy.
"""

import asyncio, json, signal, time, threading, socket
from fractions import Fraction
from aiohttp import web
import numpy as np
import depthai as dai
import av

# --- Tractor controller via UDP to teensy_serial_bridge ----------------------
# NEW: Replaces PCA9685 ServoController from step6 (lines ~17-80)
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
        # Midpoint of each bucket's threshold range (from Teensy controlTransmission)
        # bucket 0: >=931          -> midpoint ~977
        # bucket 1: 838..930       -> midpoint ~884
        # bucket 2: 746..837       -> midpoint ~791
        # bucket 3: 654..745       -> midpoint ~699
        # bucket 4: 562..653       -> midpoint ~607
        # bucket 5: 469..561       -> midpoint ~515
        # bucket 6: 377..468       -> midpoint ~422
        # bucket 7: 285..376       -> midpoint ~330
        # bucket 8: 192..284       -> midpoint ~238
        # bucket 9: <192           -> midpoint ~96
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

# --- DepthAI video -----------------------------------------------------------
# UNCHANGED from step6
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
    # UNCHANGED from step6
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

# --- UDP telemetry receivers -------------------------------------------------
# CHANGED: Now listens on BOTH port 6002 (GPS/RTK from rtcm_server)
# AND port 6003 (bridge status from teensy_serial_bridge).
# Step6 only listened on 6002.

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

# NEW: Two receivers instead of one
gps_rx = UdpTelemetryReceiver("0.0.0.0", 6002, "GPS")        # GPS/RTK from rtcm_server
bridge_rx = UdpTelemetryReceiver("0.0.0.0", 6003, "BRIDGE")   # Teensy status from bridge

# --- Web app (video + telemetry + controls UI) --------------------------------
# CHANGED: Telemetry table expanded to show bridge data (radio, steering, mode).
# CHANGED: Speed labels reference "bucket" system with neutral at position 6 (bucket 5).
# CHANGED: Title bar shows "Teleop" to distinguish from previous servo-based versions.

INDEX_HTML = """<!doctype html><html><head><meta charset="utf-8"/>
<title>WebRTC Tractor Teleop</title>
<style>
  :root { --bg:#111; --panel:#1e1e1e; --text:#eee; --muted:#aaa; --accent:#64b5f6; --danger:#e53935; }
  *{box-sizing:border-box}
  html,body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,Arial;overflow:hidden;height:100vh;font-size:13px}
  header{background:#141414;display:flex;justify-content:space-between;align-items:center;gap:10px;padding:6px 12px;border-bottom:1px solid #222;z-index:10}
  .estop{background:var(--danger);color:#000;font-weight:800;border:0;border-radius:6px;padding:6px 12px;font-size:14px;cursor:pointer}
  .hdr-title{color:var(--muted);font-size:13px;font-weight:600}
  .mode-badge{background:#333;color:var(--accent);padding:3px 8px;border-radius:4px;font-size:11px;font-weight:600}
  .wrap{display:grid;grid-template-columns: 1fr 280px;gap:10px;padding:10px;max-width:100vw;height:calc(100vh - 42px);overflow:hidden}
  .card{background:var(--panel);border-radius:8px;padding:8px}
  .video-container{width:100%;max-height:calc(100vh - 250px)}
  video{width:100%;max-height:100%;height:auto;background:#000;border-radius:8px;display:block;object-fit:contain}
  .row{display:flex;gap:8px;align-items:center;margin-top:8px;flex-wrap:wrap}
  input[type="range"]{flex:1;min-width:0}
  table{width:100%;border-collapse:collapse;font-size:12px}
  th,td{text-align:left;padding:4px 6px;border-bottom:1px solid #333}
  th{color:var(--muted);font-weight:600;width:100px}
  .radio-group{display:grid;grid-template-columns:1fr;gap:4px;}
  .radio{display:flex;align-items:center;gap:6px;background:#252525;border-radius:6px;padding:4px 6px;font-size:12px}
  .presets{display:grid;grid-template-columns:repeat(7,1fr);gap:4px;width:100%}
  .preset{background:#333;color:#eee;border:0;border-radius:4px;padding:4px 2px;cursor:pointer;font-size:11px;text-align:center;white-space:nowrap}
  .preset:hover{background:#444}
  pre{white-space:pre-wrap;font-size:10px;max-height:100px;overflow-y:auto;margin:4px 0}
  h3{margin:0 0 6px 0;font-size:14px;font-weight:600}
  @media (max-width: 1200px) {
    .wrap{grid-template-columns:1fr}
    .presets{grid-template-columns:repeat(4,1fr)}
  }
</style>
</head><body>
  <header>
    <div style="display:flex;align-items:center;gap:10px">
      <span class="hdr-title">Tractor Teleop</span>
      <span class="mode-badge" id="modeLabel">MODE: --</span>
      <span class="mode-badge" id="radioLabel">RADIO: --</span>
    </div>
    <button class="estop" id="estopTop">EMERGENCY STOP</button>
  </header>

  <div class="wrap">
    <!-- Left: Video + Steering -->
    <div style="display:flex;flex-direction:column;gap:8px;overflow:hidden">
      <div class="card video-container">
        <video id="remoteVideo" playsinline autoplay></video>
      </div>

      <div class="card">
        <h3>Steering</h3>
        <div class="presets" id="steerPresets"></div>
        <div class="row">
          <input id="steer" type="range" min="-1" max="1" value="0" step="0.01"/>
          <div id="steerVal" style="min-width:40px;text-align:right;font-size:12px">0.00</div>
        </div>
      </div>

      <div class="card" style="flex:1;overflow:hidden;display:flex;flex-direction:column;min-height:0">
        <h3>Logs</h3>
        <pre id="log" style="flex:1"></pre>
      </div>
    </div>

    <!-- Right: Speed + Telemetry -->
    <div style="display:flex;flex-direction:column;gap:8px;overflow:hidden">
      <div class="card" style="flex:0 0 auto">
        <h3>Speed</h3>
        <div id="speedRadios" class="radio-group"></div>
      </div>

      <div class="card" style="flex:0 0 auto">
        <h3>Telemetry</h3>
        <table>
          <tr><th>RTK Status</th><td id="t_rtk">--</td></tr>
          <tr><th>Heading</th><td id="t_head">--</td></tr>
          <tr><th>Radio</th><td id="t_radio">--</td></tr>
          <tr><th>Steer Pot</th><td id="t_steer">--</td></tr>
          <tr><th>Trans Mode</th><td id="t_trans">--</td></tr>
        </table>
      </div>
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

// --- Steering UI (presets + slider) --- UNCHANGED from step6
const steer = document.getElementById('steer');
const steerVal = document.getElementById('steerVal');
steer.addEventListener('input', ()=>{
  steerVal.textContent = (+steer.value).toFixed(2);
  sendCtrl({ steering: +steer.value });
});
const presets = [
  ["Full Left",  -1.0],
  ["1/2 Left",   -0.5],
  ["1/4 Left",   -0.25],
  ["Straight",    0.0],
  ["1/4 Right",   0.25],
  ["1/2 Right",   0.5],
  ["Full Right",  1.0],
];
const steerPresets = document.getElementById('steerPresets');
for (const [label, val] of presets){
  const b = document.createElement('button');
  b.className = 'preset';
  b.textContent = label;
  b.onclick = ()=>{ steer.value = val; steer.dispatchEvent(new Event('input')); };
  steerPresets.appendChild(b);
}

// --- Speed Radio Buttons ---
// CHANGED: Labels now reference bucket system.  Position 6 = neutral (bucket 5).
const speedRadios = document.getElementById('speedRadios');
const speedLabels = {
  1: "Reverse 4 (Max)",
  2: "Reverse 3",
  3: "Reverse 2",
  4: "Reverse 1",
  5: "Creep Rev",
  6: "NEUTRAL (STOP)",
  7: "Creep Fwd",
  8: "Forward 1",
  9: "Forward 2",
  10:"Forward 3 (Max)"
};
function makeRadioRow(i){
  const row = document.createElement('label'); row.className='radio';
  const input = document.createElement('input'); input.type='radio'; input.name='speedpos'; input.value=String(i);
  input.onchange = ()=>{ if (input.checked) sendCtrl({ speed_pos: i }); };
  const txt = document.createElement('span'); txt.textContent = `${i}: ${speedLabels[i]}`;
  row.appendChild(input); row.appendChild(txt);
  return row;
}
for (let i=1;i<=10;i++){ speedRadios.appendChild(makeRadioRow(i)); }
// CHANGED: Default to position 6 (neutral = bucket 5) instead of position 4
const neutralRadio = speedRadios.querySelector('input[value="6"]'); if (neutralRadio){ neutralRadio.checked = true; }

document.getElementById('estopTop').onclick = ()=> sendCtrl({ eStop: true });

function sendCtrl(obj){
  if (ctrl && ctrl.readyState === 'open'){
    ctrl.send(JSON.stringify(obj));
  } else {
    log('control channel not open yet');
  }
}

// --- Telemetry display ---
// CHANGED: Updated to show combined GPS (port 6002) + bridge (port 6003) data
function updateTelemetry(jsonText){
  try{
    const obj = JSON.parse(jsonText);

    // GPS data (from port 6002 via rtcm_server)
    const gps = obj.gps || {};
    const rtk = gps.fix_quality || '--';
    const heading = gps.heading_deg;
    setText('t_rtk',  rtk);
    setText('t_head', heading != null ? heading.toFixed(1) + '°' : '--');

    // Bridge data (from port 6003 via teensy_serial_bridge)
    const bridge = obj.bridge || {};
    const radio = bridge.radio || {};
    const steerData = bridge.steering || {};
    const trans = bridge.transmission || {};

    setText('t_radio', radio.signal || '--');
    setText('t_steer', steerData.current != null ? steerData.current.toFixed(0) : '--');

    // Transmission mode label
    const modeMap = {0:'Pause', 1:'Manual', 2:'Auto', 9:'Safety'};
    const modeNum = trans.mode != null ? trans.mode : null;
    setText('t_trans', modeNum != null ? (modeMap[modeNum] || modeNum) : '--');

    // Header badges
    document.getElementById('modeLabel').textContent = 'MODE: ' + (modeNum != null ? (modeMap[modeNum] || modeNum) : '--');
    document.getElementById('radioLabel').textContent = 'RADIO: ' + (radio.signal || '--');

  }catch(e){
    console.error('telemetry parse error', e, jsonText);
  }
}
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
    # CHANGED: Now merges data from both GPS (6002) and bridge (6003) receivers
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

    # Prefer H264 then VP8 — UNCHANGED from step5/6
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
    # CHANGED: Calls TRACTOR methods instead of SERVOS methods
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
                TRACTOR.emergency_stop()                        # CHANGED from SERVOS
            # Speed: UI sends position 1-10, we map to bucket 0-9
            if "speed_pos" in obj:
                pos = max(1, min(10, int(obj["speed_pos"])))
                TRACTOR.set_speed_bucket(pos - 1)               # CHANGED: pos 1-10 -> bucket 0-9
            # Steering: UI sends -1.0..+1.0
            if "steering" in obj:
                val = max(-1.0, min(1.0, float(obj["steering"])))
                TRACTOR.set_steering_normalized(val)             # CHANGED: -> pot units 0-1023

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

async def _main():
    # Start both UDP listeners
    gps_rx.start()       # NEW: was single telemetry_rx in step6
    bridge_rx.start()    # NEW: bridge status on port 6003

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    runner = web.AppRunner(app); await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", 8080); await site.start()
    print("Listening on http://0.0.0.0:8080")
    print("  GPS telemetry:    UDP 6002 (from rtcm_server)")
    print("  Bridge telemetry: UDP 6003 (from teensy_serial_bridge)")
    print("  Cmd_vel output:   UDP 6004 (to teensy_serial_bridge)")

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
