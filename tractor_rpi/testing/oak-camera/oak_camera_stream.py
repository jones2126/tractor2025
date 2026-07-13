#!/usr/bin/env python3

import cv2
import depthai as dai
from flask import Flask, Response, render_template_string, request, jsonify
from flask_socketio import SocketIO, emit
import threading
import time
import json
import numpy as np

app = Flask(__name__)
app.config['SECRET_KEY'] = 'robot_teleoperation_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables
latest_frame = None
frame_lock = threading.Lock()
robot_status = {
    'battery': 85,
    'connection': 'Connected',
    'speed': 0,
    'direction': 0,
    'camera_tilt': 0
}

# Overlay settings
show_crosshair = True
show_grid = False
show_telemetry = True

def create_pipeline():
    """Create DepthAI pipeline for Oak camera - Updated API"""
    pipeline = dai.Pipeline()
    
    # Create Camera node (new API)
    cam_rgb = pipeline.create(dai.node.Camera)
    cam_rgb.setPreviewSize(1280, 720)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(30)
    
    # Create output
    rgb_out = pipeline.createXLinkOut()
    rgb_out.setStreamName("rgb")
    cam_rgb.preview.link(rgb_out.input)
    
    return pipeline

def add_overlays(frame):
    """Add teleoperation overlays to the frame"""
    global robot_status, show_crosshair, show_grid, show_telemetry
    
    height, width = frame.shape[:2]
    
    # Add crosshair
    if show_crosshair:
        center_x, center_y = width // 2, height // 2
        cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
        cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)
        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
    
    # Add grid
    if show_grid:
        grid_spacing = 80
        for x in range(0, width, grid_spacing):
            cv2.line(frame, (x, 0), (x, height), (100, 100, 100), 1)
        for y in range(0, height, grid_spacing):
            cv2.line(frame, (0, y), (width, y), (100, 100, 100), 1)
    
    # Add telemetry overlay
    if show_telemetry:
        # Background for telemetry
        cv2.rectangle(frame, (10, 10), (300, 120), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (300, 120), (255, 255, 255), 2)
        
        # Status text
        y_offset = 30
        cv2.putText(frame, f"Battery: {robot_status['battery']}%", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y_offset += 25
        cv2.putText(frame, f"Status: {robot_status['connection']}", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y_offset += 25
        cv2.putText(frame, f"Speed: {robot_status['speed']:.1f}", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y_offset += 25
        cv2.putText(frame, f"Direction: {robot_status['direction']:.1f}°", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Add timestamp
    timestamp = time.strftime("%H:%M:%S", time.localtime())
    cv2.putText(frame, timestamp, (width - 120, height - 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    return frame

def capture_frames():
    """Capture frames from Oak camera in a separate thread"""
    global latest_frame
    
    try:
        # Connect to device and start pipeline
        with dai.Device(create_pipeline()) as device:
            print(f"Connected to Oak camera: {device.getDeviceInfo().mxid}")
            
            # Get output queue
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            while True:
                in_rgb = q_rgb.get()
                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()
                    
                    # Add overlays
                    frame = add_overlays(frame)
                    
                    # Update latest frame thread-safely
                    with frame_lock:
                        latest_frame = frame.copy()
                
                time.sleep(0.01)
                
    except Exception as e:
        print(f"Error capturing frames: {e}")
        import traceback
        traceback.print_exc()

def generate_frames():
    """Generate frames for HTTP streaming"""
    global latest_frame
    
    while True:
        with frame_lock:
            if latest_frame is not None:
                frame = latest_frame.copy()
            else:
                # Create a black frame if no camera data
                frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                cv2.putText(frame, "Waiting for camera...", (50, 360), 
                           cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ret:
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.033)  # ~30 FPS

@app.route('/')
def index():
    """Serve the teleoperation interface"""
    html_template = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robot Teleoperation Station</title>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.5/socket.io.js"></script>
        <style>
            body { 
                font-family: 'Courier New', monospace; 
                background: #1a1a1a; 
                color: #00ff00; 
                margin: 0; 
                padding: 20px;
                overflow-x: auto;
            }
            .container { 
                display: flex; 
                gap: 20px; 
                min-width: 1400px;
            }
            .video-section { 
                flex: 2;
            }
            .video-stream { 
                border: 2px solid #00ff00; 
                border-radius: 10px; 
                max-width: 100%;
                background: #000;
            }
            .controls-section { 
                flex: 1; 
                background: #2a2a2a; 
                padding: 20px; 
                border-radius: 10px;
                border: 1px solid #00ff00;
            }
            .control-group {
                margin-bottom: 20px;
                padding: 15px;
                background: #333;
                border-radius: 5px;
            }
            .joystick {
                width: 200px;
                height: 200px;
                border: 2px solid #00ff00;
                border-radius: 50%;
                position: relative;
                background: #1a1a1a;
                margin: 10px auto;
                cursor: crosshair;
            }
            .joystick-knob {
                width: 40px;
                height: 40px;
                background: #00ff00;
                border-radius: 50%;
                position: absolute;
                top: 50%;
                left: 50%;
                transform: translate(-50%, -50%);
                transition: all 0.1s;
            }
            button {
                background: #00ff00;
                color: #000;
                border: none;
                padding: 10px 20px;
                margin: 5px;
                border-radius: 5px;
                cursor: pointer;
                font-weight: bold;
            }
            button:hover {
                background: #00cc00;
            }
            button:active {
                background: #009900;
            }
            .slider {
                width: 100%;
                margin: 10px 0;
            }
            .status-display {
                background: #000;
                padding: 10px;
                border: 1px solid #00ff00;
                border-radius: 5px;
                font-family: monospace;
            }
            h1, h2, h3 { color: #00ff00; text-align: center; }
            .emergency-stop {
                background: #ff0000 !important;
                color: #fff !important;
                font-size: 18px;
                padding: 15px 30px;
            }
            .overlay-controls label {
                display: block;
                margin: 10px 0;
            }
        </style>
    </head>
    <body>
        <h1>🤖 ROBOT TELEOPERATION STATION</h1>
        
        <div class="container">
            <div class="video-section">
                <h2>Live Video Feed</h2>
                <img id="videoStream" src="/video_feed" class="video-stream" alt="Live Stream">
                <p>Stream: <code>http://192.168.1.151:5000/video_feed</code></p>
            </div>
            
            <div class="controls-section">
                <h2>Robot Controls</h2>
                
                <div class="control-group">
                    <h3>Movement</h3>
                    <div class="joystick" id="joystick">
                        <div class="joystick-knob" id="joystick-knob"></div>
                    </div>
                    <div style="text-align: center;">
                        <div>Speed: <span id="speed-value">0.0</span></div>
                        <div>Direction: <span id="direction-value">0.0</span>°</div>
                    </div>
                </div>
                
                <div class="control-group">
                    <h3>Camera Control</h3>
                    <label>Tilt: <input type="range" class="slider" id="cameraTilt" min="-90" max="90" value="0"></label>
                    <div>Tilt: <span id="tilt-value">0</span>°</div>
                </div>
                
                <div class="control-group">
                    <h3>Quick Actions</h3>
                    <button onclick="sendCommand('forward')">Forward</button>
                    <button onclick="sendCommand('backward')">Backward</button>
                    <button onclick="sendCommand('left')">Turn Left</button>
                    <button onclick="sendCommand('right')">Turn Right</button>
                    <button onclick="sendCommand('stop')">Stop</button>
                    <button class="emergency-stop" onclick="emergencyStop()">🛑 EMERGENCY STOP</button>
                </div>
                
                <div class="control-group">
                    <h3>Overlay Settings</h3>
                    <div class="overlay-controls">
                        <label><input type="checkbox" id="crosshair" checked> Show Crosshair</label>
                        <label><input type="checkbox" id="grid"> Show Grid</label>
                        <label><input type="checkbox" id="telemetry" checked> Show Telemetry</label>
                    </div>
                </div>
                
                <div class="control-group">
                    <h3>Robot Status</h3>
                    <div class="status-display" id="status-display">
                        <div>Battery: <span id="battery">--</span>%</div>
                        <div>Connection: <span id="connection">--</span></div>
                        <div>Latency: <span id="latency">--</span>ms</div>
                    </div>
                </div>
            </div>
        </div>
        
        <script>
            const socket = io();
            
            // Joystick handling
            const joystick = document.getElementById('joystick');
            const knob = document.getElementById('joystick-knob');
            let isDragging = false;
            let joystickRect = joystick.getBoundingClientRect();
            
            function updateJoystick(x, y) {
                const centerX = joystickRect.width / 2;
                const centerY = joystickRect.height / 2;
                const maxRadius = centerX - 20;
                
                let deltaX = x - centerX;
                let deltaY = y - centerY;
                let distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                
                if (distance > maxRadius) {
                    deltaX = (deltaX / distance) * maxRadius;
                    deltaY = (deltaY / distance) * maxRadius;
                }
                
                knob.style.left = (centerX + deltaX) + 'px';
                knob.style.top = (centerY + deltaY) + 'px';
                
                // Calculate speed and direction
                const speed = (distance / maxRadius) * 100;
                const direction = Math.atan2(deltaX, -deltaY) * (180 / Math.PI);
                
                document.getElementById('speed-value').textContent = speed.toFixed(1);
                document.getElementById('direction-value').textContent = direction.toFixed(1);
                
                // Send to robot
                socket.emit('joystick_update', {speed: speed, direction: direction});
            }
            
            joystick.addEventListener('mousedown', (e) => {
                isDragging = true;
                joystickRect = joystick.getBoundingClientRect();
                updateJoystick(e.clientX - joystickRect.left, e.clientY - joystickRect.top);
            });
            
            document.addEventListener('mousemove', (e) => {
                if (isDragging) {
                    updateJoystick(e.clientX - joystickRect.left, e.clientY - joystickRect.top);
                }
            });
            
            document.addEventListener('mouseup', () => {
                if (isDragging) {
                    isDragging = false;
                    knob.style.left = '50%';
                    knob.style.top = '50%';
                    document.getElementById('speed-value').textContent = '0.0';
                    document.getElementById('direction-value').textContent = '0.0';
                    socket.emit('joystick_update', {speed: 0, direction: 0});
                }
            });
            
            // Camera tilt control
            document.getElementById('cameraTilt').addEventListener('input', (e) => {
                const tilt = e.target.value;
                document.getElementById('tilt-value').textContent = tilt;
                socket.emit('camera_tilt', {tilt: parseInt(tilt)});
            });
            
            // Overlay controls
            document.getElementById('crosshair').addEventListener('change', (e) => {
                socket.emit('overlay_setting', {crosshair: e.target.checked});
            });
            
            document.getElementById('grid').addEventListener('change', (e) => {
                socket.emit('overlay_setting', {grid: e.target.checked});
            });
            
            document.getElementById('telemetry').addEventListener('change', (e) => {
                socket.emit('overlay_setting', {telemetry: e.target.checked});
            });
            
            // Command functions
            function sendCommand(command) {
                socket.emit('robot_command', {command: command});
            }
            
            function emergencyStop() {
                socket.emit('emergency_stop');
                alert('EMERGENCY STOP ACTIVATED');
            }
            
            // Status updates
            socket.on('status_update', (data) => {
                document.getElementById('battery').textContent = data.battery;
                document.getElementById('connection').textContent = data.connection;
            });
            
            // Latency measurement
            setInterval(() => {
                const start = Date.now();
                socket.emit('ping', start);
            }, 1000);
            
            socket.on('pong', (start) => {
                const latency = Date.now() - start;
                document.getElementById('latency').textContent = latency;
            });
        </script>
    </body>
    </html>
    """
    return render_template_string(html_template)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# WebSocket event handlers
@socketio.on('joystick_update')
def handle_joystick(data):
    """Handle joystick input"""
    robot_status['speed'] = data['speed']
    robot_status['direction'] = data['direction']
    # TODO: Send commands to actual robot hardware
    print(f"Joystick: Speed={data['speed']:.1f}, Direction={data['direction']:.1f}")

@socketio.on('camera_tilt')
def handle_camera_tilt(data):
    """Handle camera tilt control"""
    robot_status['camera_tilt'] = data['tilt']
    # TODO: Control camera servo/gimbal
    print(f"Camera tilt: {data['tilt']}°")

@socketio.on('robot_command')
def handle_robot_command(data):
    """Handle robot commands"""
    command = data['command']
    print(f"Robot command: {command}")
    # TODO: Implement robot command logic

@socketio.on('emergency_stop')
def handle_emergency_stop():
    """Handle emergency stop"""
    robot_status['speed'] = 0
    robot_status['direction'] = 0
    print("EMERGENCY STOP ACTIVATED!")
    # TODO: Immediately stop all robot movement

@socketio.on('overlay_setting')
def handle_overlay_setting(data):
    """Handle overlay setting changes"""
    global show_crosshair, show_grid, show_telemetry
    if 'crosshair' in data:
        show_crosshair = data['crosshair']
    if 'grid' in data:
        show_grid = data['grid']
    if 'telemetry' in data:
        show_telemetry = data['telemetry']

@socketio.on('ping')
def handle_ping(data):
    """Handle latency ping"""
    emit('pong', data)

def status_update_thread():
    """Update robot status periodically"""
    while True:
        # TODO: Get actual robot status from hardware
        socketio.emit('status_update', robot_status)
        time.sleep(1)

if __name__ == '__main__':
    # Start frame capture thread
    capture_thread = threading.Thread(target=capture_frames, daemon=True)
    capture_thread.start()
    
    # Start status update thread
    status_thread = threading.Thread(target=status_update_thread, daemon=True)
    status_thread.start()
    
    print("Starting Oak Camera Teleoperation Station...")
    print("Access the interface at: http://192.168.1.151:5000")
    
    # Start Flask-SocketIO server
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)