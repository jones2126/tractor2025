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
    'speed_position': 6,  # Neutral position (0-12 range)
    'steering_position': 0.0,  # -1.0 to 1.0 range
    'rtk_fix': False,
    'autopilot': False,
    'latency': 0
}

# Overlay settings
show_crosshair = True
show_grid = False
show_telemetry = True

def create_pipeline():
    """Create pipeline using the working approach from oak_v3_streaming.py"""
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(30)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("video")
    cam_rgb.video.link(xout.input)

    return pipeline

def add_overlays(frame):
    """Add teleoperation overlays to the frame"""
    global robot_status, show_crosshair, show_grid, show_telemetry
    
    height, width = frame.shape[:2]
    print(f"Frame size: {width}x{height}, adding overlays...")  # Debug
    
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
    
    # Force telemetry overlay - make it more visible
    print("Adding telemetry overlay...")  # Debug
    
    # Make telemetry box larger and more visible
    box_width = 380
    box_height = 180
    cv2.rectangle(frame, (10, 10), (10 + box_width, 10 + box_height), (0, 0, 0), -1)
    cv2.rectangle(frame, (10, 10), (10 + box_width, 10 + box_height), (0, 255, 0), 3)
    
    # Add title
    cv2.putText(frame, "TELEMETRY", (20, 35), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    
    # Status text with larger font
    y_offset = 60
    cv2.putText(frame, f"Battery: {robot_status['battery']}%", (20, y_offset), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    y_offset += 30
    cv2.putText(frame, f"Connection: {robot_status['connection']}", (20, y_offset), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    y_offset += 30
    cv2.putText(frame, f"Latency: {robot_status['latency']}ms", (20, y_offset), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    y_offset += 30
    cv2.putText(frame, f"RTK Fix: {'YES' if robot_status['rtk_fix'] else 'NO'}", (20, y_offset), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if robot_status['rtk_fix'] else (0, 0, 255), 2)
    y_offset += 30
    cv2.putText(frame, f"Autopilot: {'ON' if robot_status['autopilot'] else 'OFF'}", (20, y_offset), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if robot_status['autopilot'] else (255, 255, 255), 2)
    
    # Add timestamp in bottom right
    timestamp = time.strftime("%H:%M:%S", time.localtime())
    cv2.putText(frame, timestamp, (width - 150, height - 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    print("Telemetry overlay added successfully")  # Debug
    return frame

def capture_frames_working():
    """Frame capture using the working approach with overlays"""
    global latest_frame
    
    while True:
        try:
            print("Connecting to device...")
            with dai.Device(create_pipeline()) as device:
                print("Pipeline started. Camera ready for streaming.")
                q_rgb = device.getOutputQueue(name="video", maxSize=4, blocking=False)

                frame_count = 0

                while True:
                    in_rgb = q_rgb.tryGet()
                    if in_rgb:
                        frame = in_rgb.getCvFrame()
                        
                        if frame is not None and frame.size > 0:
                            frame_count += 1
                            
                            # Add teleoperation overlays
                            frame = add_overlays(frame)
                            
                            # Update latest frame thread-safely
                            with frame_lock:
                                latest_frame = frame.copy()
                    
                    time.sleep(0.01)
                    
        except RuntimeError as e:
            print(f"[WARN] Device error: {e}")
            print("Retrying in 3 seconds...")
            time.sleep(3)

def generate_frames():
    """Generate frames for HTTP streaming"""
    global latest_frame
    
    while True:
        with frame_lock:
            if latest_frame is not None:
                frame = latest_frame.copy()
                
                # Double-check: add overlays here too as backup
                frame = add_overlays(frame)
                
            else:
                # Create a black frame if no camera data
                frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                cv2.putText(frame, "Waiting for camera...", (50, 360), 
                           cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
                
                # Add telemetry to waiting screen too
                frame = add_overlays(frame)
        
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
                padding: 10px;
                overflow: hidden;
            }
            .main-container {
                display: flex;
                gap: 15px;
                width: 100vw;
                height: calc(100vh - 60px);
                max-width: 100vw;
                box-sizing: border-box;
            }
            .video-section { 
                flex: 1;
                position: relative;
                display: flex;
                flex-direction: column;
                min-width: 0;
                overflow-y: auto;
                max-height: calc(100vh - 60px);
            }
            .video-stream { 
                border: 2px solid #00ff00; 
                border-radius: 10px; 
                width: 100%;
                height: 60vh;
                max-height: 60vh;
                background: #000;
                display: block;
                object-fit: cover;
            }
            .emergency-stop {
                position: absolute;
                top: 10px;
                right: 10px;
                background: #ff0000 !important;
                color: #fff !important;
                font-size: 18px;
                padding: 15px 30px;
                border: none;
                border-radius: 5px;
                cursor: pointer;
                font-weight: bold;
                z-index: 100;
            }
            .emergency-stop:hover {
                background: #cc0000 !important;
            }
            .steering-control {
                margin-top: 10px;
                padding: 10px;
                background: #2a2a2a;
                border: 1px solid #00ff00;
                border-radius: 10px;
                flex-shrink: 0;
            }
            .steering-buttons {
                display: flex;
                gap: 3px;
                justify-content: center;
                margin-top: 8px;
                flex-wrap: wrap;
            }
            .steering-btn {
                background: #00ff00;
                color: #000;
                border: none;
                padding: 6px 4px;
                border-radius: 3px;
                cursor: pointer;
                font-weight: bold;
                font-size: 10px;
                min-width: 55px;
            }
            .steering-btn:hover {
                background: #00cc00;
            }
            .steering-btn.active {
                background: #ffff00;
                color: #000;
            }
            .steering-slider-container {
                margin: 10px 0;
                text-align: center;
            }
            .steering-slider {
                width: 100%;
                height: 6px;
                background: #444;
                outline: none;
                border-radius: 3px;
                -webkit-appearance: none;
            }
            .steering-slider::-webkit-slider-thumb {
                -webkit-appearance: none;
                appearance: none;
                width: 16px;
                height: 16px;
                background: #00ff00;
                cursor: pointer;
                border-radius: 50%;
            }
            .steering-slider::-moz-range-thumb {
                width: 16px;
                height: 16px;
                background: #00ff00;
                cursor: pointer;
                border-radius: 50%;
                border: none;
            }
            .controls-section { 
                flex: 0 0 180px;
                background: #2a2a2a; 
                padding: 15px; 
                border-radius: 10px;
                border: 1px solid #00ff00;
                height: fit-content;
                max-height: calc(100vh - 80px);
                overflow-y: auto;
            }
            .control-group {
                margin-bottom: 15px;
                padding: 12px;
                background: #333;
                border-radius: 5px;
            }
            .speed-control {
                display: flex;
                flex-direction: column;
                gap: 4px;
            }
            .speed-radio {
                display: flex;
                align-items: center;
                gap: 6px;
                padding: 4px 6px;
                background: #444;
                border-radius: 3px;
                cursor: pointer;
                font-size: 11px;
            }
            .speed-radio input[type="radio"] {
                transform: scale(0.9);
            }
            .speed-radio.neutral {
                background: #555;
                border: 1px solid #ffff00;
                font-weight: bold;
                padding: 6px;
            }
            .speed-radio:hover {
                background: #555;
            }
            .overlay-settings {
                margin-top: 10px;
                padding: 10px;
                background: #2a2a2a;
                border: 1px solid #00ff00;
                border-radius: 10px;
                flex-shrink: 0;
            }
            .overlay-controls label {
                display: block;
                margin: 6px 0;
                cursor: pointer;
                font-size: 11px;
            }
            h1 { 
                color: #00ff00; 
                text-align: center; 
                margin: 5px 0 15px 0; 
                font-size: 24px;
            }
            h2 { 
                color: #00ff00; 
                text-align: center; 
                margin: 8px 0; 
                font-size: 16px;
            }
            h3 { 
                color: #00ff00; 
                text-align: center; 
                margin: 8px 0; 
                font-size: 14px;
            }
        </style>
    </head>
    <body>
        <h1>Robot Teleoperation Station</h1>
        
        <div class="main-container">
            <div class="video-section">
                <button class="emergency-stop" onclick="emergencyStop()">🛑 EMERGENCY STOP</button>
                <img id="videoStream" src="/video_feed" class="video-stream" alt="Live Stream">
                
                <div class="steering-control">
                    <h3>Steering Control</h3>
                    <div class="steering-buttons">
                        <button class="steering-btn" onclick="setSteering(-1.0)">Full Left</button>
                        <button class="steering-btn" onclick="setSteering(-0.5)">1/2 Left</button>
                        <button class="steering-btn" onclick="setSteering(-0.25)">1/4 Left</button>
                        <button class="steering-btn active" onclick="setSteering(0.0)">Straight</button>
                        <button class="steering-btn" onclick="setSteering(0.25)">1/4 Right</button>
                        <button class="steering-btn" onclick="setSteering(0.5)">1/2 Right</button>
                        <button class="steering-btn" onclick="setSteering(1.0)">Full Right</button>
                    </div>
                    <div class="steering-slider-container">
                        <input type="range" min="-100" max="100" value="0" class="steering-slider" id="steeringSlider" oninput="setSteeringSlider(this.value)">
                        <div style="margin-top: 5px;">
                            <span>Current: <span id="steering-value">0.0</span></span>
                        </div>
                    </div>
                </div>
                
                <div class="overlay-settings">
                    <h3>Overlay Settings</h3>
                    <div class="overlay-controls">
                        <label><input type="checkbox" id="crosshair" checked> Show Crosshair</label>
                        <label><input type="checkbox" id="grid"> Show Grid</label>
                        <label><input type="checkbox" id="telemetry" checked> Show Telemetry</label>
                    </div>
                </div>
            </div>
            
            <div class="controls-section">
                <h2>Speed Control</h2>
                
                <div class="control-group">
                    <div class="speed-control" id="speed-control">
                        <div class="speed-radio" onclick="setSpeed(12)">
                            <input type="radio" name="speed" value="12" id="speed12">
                            <label for="speed12">Forward 6 (Max)</label>
                        </div>
                        <div class="speed-radio" onclick="setSpeed(11)">
                            <input type="radio" name="speed" value="11" id="speed11">
                            <label for="speed11">Forward 5</label>
                        </div>
                        <div class="speed-radio" onclick="setSpeed(10)">
                            <input type="radio" name="speed" value="10" id="speed10">
                            <label for="speed10">Forward 4</label>
                        </div>
                        <div class="speed-radio" onclick="setSpeed(9)">
                            <input type="radio" name="speed" value="9" id="speed9">
                            <label for="speed9">Forward 3</label>
                        </div>
                        <div class="speed-radio" onclick="setSpeed(8)">
                            <input type="radio" name="speed" value="8" id="speed8">
                            <label for="speed8">Forward 2</label>
                        </div>
                        <div class="speed-radio" onclick="setSpeed(7)">
                            <input type="radio" name="speed" value="7" id="speed7">
                            <label for="speed7">Forward 1 (Min)</label>
                        </div>
                        <div class="speed-radio neutral" onclick="setSpeed(6)">
                            <input type="radio" name="speed" value="6" id="speed6" checked>
                            <label for="speed6">NEUTRAL (STOP)</label>
                        </div>
                        <div class="speed-radio" onclick="setSpeed(5)">
                            <input type="radio" name="speed" value="5" id="speed5">
                            <label for="speed5">Reverse 1</label>
                        </div>
                        <div class="speed-radio" onclick="setSpeed(4)">
                            <input type="radio" name="speed" value="4" id="speed4">
                            <label for="speed4">Reverse 2</label>
                        </div>
                        <div class="speed-radio" onclick="setSpeed(3)">
                            <input type="radio" name="speed" value="3" id="speed3">
                            <label for="speed3">Reverse 3 (Max)</label>
                        </div>
                    </div>
                </div>
            </div>
        </div>
        
        <script>
            const socket = io();
            let currentSteering = 0.0;
            let currentSpeed = 6;
            
            // Steering control with buttons
            function setSteering(value) {
                currentSteering = value;
                document.getElementById('steering-value').textContent = value.toFixed(1);
                document.getElementById('steeringSlider').value = value * 100;
                
                // Update button visual state
                document.querySelectorAll('.steering-btn').forEach(btn => {
                    btn.classList.remove('active');
                });
                event.target.classList.add('active');
                
                // Send to robot
                socket.emit('steering_update', {steering: value});
                sendCmdVel();
            }
            
            // Steering control with slider
            function setSteeringSlider(sliderValue) {
                const value = sliderValue / 100.0;  // Convert -100 to 100 range to -1.0 to 1.0
                currentSteering = value;
                document.getElementById('steering-value').textContent = value.toFixed(1);
                
                // Clear active button states when using slider
                document.querySelectorAll('.steering-btn').forEach(btn => {
                    btn.classList.remove('active');
                });
                
                // Send to robot
                socket.emit('steering_update', {steering: value});
                sendCmdVel();
            }
            
            // Speed control
            function setSpeed(position) {
                currentSpeed = position;
                document.getElementById('speed' + position).checked = true;
                
                // Send to robot
                socket.emit('speed_update', {position: position});
                sendCmdVel();
            }
            
            // Send combined cmd_vel command
            function sendCmdVel() {
                // Convert speed position to linear velocity (-1 to 1)
                let linear = 0;
                if (currentSpeed > 6) {
                    linear = (currentSpeed - 6) / 6.0;  // Forward 0 to 1
                } else if (currentSpeed < 6) {
                    linear = (currentSpeed - 6) / 3.0;  // Reverse -1 to 0
                }
                
                socket.emit('cmd_vel', {
                    linear: linear,
                    angular: currentSteering
                });
            }
            
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
            
            // Emergency stop
            function emergencyStop() {
                socket.emit('emergency_stop');
                // Reset to neutral
                setSpeed(6);
                setSteering(0.0);
                alert('EMERGENCY STOP ACTIVATED');
            }
            
            // Status updates
            socket.on('status_update', (data) => {
                // Status is now displayed in video overlay
            });
            
            // Latency measurement
            setInterval(() => {
                const start = Date.now();
                socket.emit('ping', start);
            }, 1000);
            
            socket.on('pong', (start) => {
                const latency = Date.now() - start;
                // Latency is now displayed in video overlay
            });
            
            // Make speed radio clickable areas work
            document.querySelectorAll('.speed-radio').forEach(radio => {
                radio.addEventListener('click', function() {
                    const input = this.querySelector('input[type="radio"]');
                    if (input) {
                        input.checked = true;
                        setSpeed(parseInt(input.value));
                    }
                });
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
@socketio.on('steering_update')
def handle_steering(data):
    """Handle steering input"""
    robot_status['steering_position'] = data['steering']
    print(f"Steering: {data['steering']:.2f}")

@socketio.on('speed_update')
def handle_speed(data):
    """Handle speed position update"""
    robot_status['speed_position'] = data['position']
    print(f"Speed position: {data['position']}")

@socketio.on('cmd_vel')
def handle_cmd_vel(data):
    """Handle ROS2 cmd_vel command"""
    linear = data['linear']
    angular = data['angular']
    print(f"CMD_VEL - Linear: {linear:.2f}, Angular: {angular:.2f}")
    # TODO: Publish to ROS2 /cmd_vel topic
    # Example:
    # twist_msg = Twist()
    # twist_msg.linear.x = linear
    # twist_msg.angular.z = angular
    # cmd_vel_publisher.publish(twist_msg)

@socketio.on('emergency_stop')
def handle_emergency_stop():
    """Handle emergency stop"""
    robot_status['speed_position'] = 6  # Neutral
    robot_status['steering_position'] = 0.0
    print("EMERGENCY STOP ACTIVATED!")
    # TODO: Immediately stop all robot movement
    # Send zero velocity command
    # twist_msg = Twist()  # All zeros
    # cmd_vel_publisher.publish(twist_msg)

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
        # Update latency for display
        robot_status['latency'] = np.random.randint(10, 50)  # Simulate latency
        
        # TODO: Get actual robot status from hardware/ROS2 topics
        # robot_status['battery'] = get_battery_level()
        # robot_status['rtk_fix'] = get_rtk_status()
        # robot_status['autopilot'] = get_autopilot_status()
        
        socketio.emit('status_update', robot_status)
        time.sleep(1)

if __name__ == '__main__':
    # Start frame capture thread
    capture_thread = threading.Thread(target=capture_frames_working, daemon=True)
    capture_thread.start()
    
    # Start status update thread
    status_thread = threading.Thread(target=status_update_thread, daemon=True)
    status_thread.start()
    
    print("Starting Robot Teleoperation Station...")
    print("Access the interface at: http://192.168.1.151:5000")
    
    # Start Flask-SocketIO server
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)