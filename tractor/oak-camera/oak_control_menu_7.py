#!/usr/bin/env python3

import cv2
import depthai as dai
from flask import Flask, Response, render_template_string
from flask_socketio import SocketIO, emit
import threading
import time
import numpy as np

app = Flask(__name__)
app.config['SECRET_KEY'] = 'robot_test_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables
latest_frame = None
frame_lock = threading.Lock()
current_speed_position = 4  # Neutral (1-10 range, 4 = Neutral)
current_steering = 0.0  # Steering from -1.0 (full left) to 1.0 (full right)

def create_pipeline():
    """Create simple DepthAI pipeline"""
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

def capture_frames():
    """Capture frames from Oak camera"""
    global latest_frame
    
    while True:
        try:
            print("Connecting to Oak camera...")
            with dai.Device(create_pipeline()) as device:
                print("Camera connected successfully")
                q_rgb = device.getOutputQueue(name="video", maxSize=4, blocking=False)

                while True:
                    in_rgb = q_rgb.tryGet()
                    if in_rgb:
                        frame = in_rgb.getCvFrame()
                        
                        if frame is not None and frame.size > 0:
                            with frame_lock:
                                latest_frame = frame.copy()
                    
                    time.sleep(0.01)
                    
        except RuntimeError as e:
            print(f"Camera error: {e}")
            print("Retrying in 3 seconds...")
            time.sleep(3)

def get_speed_description(position):
    """Get descriptive text for speed position (1-10 scale)"""
    if position > 4:
        level = position - 4
        desc = f"Forward ({level})"
        if level == 6:
            desc += " Max"
        return desc
    elif position < 4:
        reverse_level = 4 - position
        desc = f"Reverse ({reverse_level})"
        if reverse_level == 3:
            desc += " Max"
        return desc
    else:
        return "NEUTRAL (STOP)"

def generate_frames():
    """Generate video frames with GUARANTEED overlay"""
    global latest_frame, current_speed_position, current_steering
    
    frame_count = 0
    
    while True:
        frame_count += 1
        
        with frame_lock:
            if latest_frame is not None:
                frame = latest_frame.copy()
            else:
                # Create black frame if no camera
                frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                cv2.putText(frame, "Waiting for camera...", (400, 360), 
                           cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        
        # Draw light blue overlay with telemetry at top-left, moved down
        # BGR for vibrant light blue: (255, 200, 200)
        overlay_width = 500
        overlay_height = 160  # Increased height for steering info
        left = 50  # Top-left horizontal position
        top = 100  # Moved down from 50 to 100 pixels from top
        right = left + overlay_width
        bottom = top + overlay_height
        cv2.rectangle(frame, (left, top), (right, bottom), (255, 200, 200), -1)  # Light blue background
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 5)   # Thick green border
        
        # Telemetry text - adjusted y positions to match new top position
        cv2.putText(frame, "SPEED CONTROL", (left + 30, top + 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)  # Smaller title
        
        speed_text = f"Position {current_speed_position}, Mode: {get_speed_description(current_speed_position)}"
        cv2.putText(frame, speed_text, (left + 30, top + 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)  # Smaller speed text
        
        steering_text = f"Steering: {current_steering:.2f}"
        cv2.putText(frame, steering_text, (left + 30, top + 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)  # Smaller steering text
        
        # Encode as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ret:
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.033)  # ~30fps

@app.route('/')
def index():
    """Serve web page with speed controls"""
    return render_template_string('''
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robot Teleoperation Station</title>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.5/socket.io.min.js"></script>
        <style>
            body { 
                background: #1a1a1a; 
                color: #00ff00; 
                font-family: 'Courier New', monospace; 
                margin: 0;
                padding: 10px;
            }
            .container {
                display: flex;
                gap: 15px;
                height: calc(100vh - 20px);
            }
            .video-section {
                flex: 1;
                display: flex;
                flex-direction: column;
                position: relative;  /* For absolute positioning of emergency button */
            }
            .video { 
                border: 2px solid #00ff00; 
                width: 100%;
                height: 70vh;
                object-fit: cover;
                background: #000;
            }
            .emergency-button {
                position: absolute;
                top: 10px;
                right: 10px;
                background: red;
                color: white;
                padding: 10px 20px;
                font-weight: bold;
                border: none;
                border-radius: 5px;
                cursor: pointer;
                z-index: 10;
            }
            .controls-section {
                flex: 0 0 200px;
                background: #2a2a2a;
                padding: 15px;
                border: 1px solid #00ff00;
                border-radius: 10px;
                height: fit-content;
            }
            .speed-control, .steering-control {
                display: flex;
                flex-direction: column;
                gap: 6px;
            }
            .steering-buttons {
                display: flex;
                justify-content: space-between;
                gap: 5px;
            }
            .steering-button {
                flex: 1;
                background: #444;
                color: #00ff00;
                padding: 8px;
                border: none;
                border-radius: 3px;
                cursor: pointer;
                font-size: 12px;
            }
            .steering-button:hover {
                background: #555;
            }
            .steering-slider {
                width: 100%;
                margin: 10px 0;
            }
            .speed-radio {
                display: flex;
                align-items: center;
                gap: 8px;
                padding: 6px 8px;
                background: #444;
                border-radius: 3px;
                cursor: pointer;
                font-size: 12px;
            }
            .speed-radio input[type="radio"] {
                transform: scale(0.9);
            }
            .speed-radio.neutral {
                background: #555;
                border: 1px solid #ffff00;
                font-weight: bold;
            }
            .speed-radio:hover {
                background: #555;
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
        </style>
    </head>
    <body>
        <h1>Robot Teleoperation Station</h1>
        <div class="container">
            <div class="video-section">
                <img id="videoStream" src="/video_feed" class="video" alt="Live Stream">
                <button class="emergency-button" onclick="emergencyStop()">EMERGENCY STOP</button>
                <p style="color: white; text-align: center; margin-top: 10px;">
                    Light blue overlay working. Speed: <span id="speed-display">4 (NEUTRAL)</span> | Steering: <span id="steering-display-bottom">0.00</span>
                </p>
            </div>
            
            <div class="controls-section">
                <h2>Speed Control (1-10)</h2>
                <div class="speed-control">
                    <div class="speed-radio" onclick="setSpeed(1)">
                        <input type="radio" name="speed" value="1" id="speed1">
                        <label for="speed1">Reverse 3 (Max)</label>
                    </div>
                    <div class="speed-radio" onclick="setSpeed(2)">
                        <input type="radio" name="speed" value="2" id="speed2">
                        <label for="speed2">Reverse 2</label>
                    </div>
                    <div class="speed-radio" onclick="setSpeed(3)">
                        <input type="radio" name="speed" value="3" id="speed3">
                        <label for="speed3">Reverse 1</label>
                    </div>
                    <div class="speed-radio neutral" onclick="setSpeed(4)">
                        <input type="radio" name="speed" value="4" id="speed4" checked>
                        <label for="speed4">NEUTRAL (STOP)</label>
                    </div>
                    <div class="speed-radio" onclick="setSpeed(5)">
                        <input type="radio" name="speed" value="5" id="speed5">
                        <label for="speed5">Forward 1 (Min)</label>
                    </div>
                    <div class="speed-radio" onclick="setSpeed(6)">
                        <input type="radio" name="speed" value="6" id="speed6">
                        <label for="speed6">Forward 2</label>
                    </div>
                    <div class="speed-radio" onclick="setSpeed(7)">
                        <input type="radio" name="speed" value="7" id="speed7">
                        <label for="speed7">Forward 3</label>
                    </div>
                    <div class="speed-radio" onclick="setSpeed(8)">
                        <input type="radio" name="speed" value="8" id="speed8">
                        <label for="speed8">Forward 4</label>
                    </div>
                    <div class="speed-radio" onclick="setSpeed(9)">
                        <input type="radio" name="speed" value="9" id="speed9">
                        <label for="speed9">Forward 5</label>
                    </div>
                    <div class="speed-radio" onclick="setSpeed(10)">
                        <input type="radio" name="speed" value="10" id="speed10">
                        <label for="speed10">Forward 6 (Max)</label>
                    </div>
                </div>
                
                <h2>Steering Control</h2>
                <div class="steering-control">
                    <div class="steering-buttons">
                        <button class="steering-button" onclick="setSteering(-1.0)">Full Left</button>
                        <button class="steering-button" onclick="setSteering(-0.5)">1/2 Left</button>
                        <button class="steering-button" onclick="setSteering(-0.25)">1/4 Left</button>
                        <button class="steering-button" onclick="setSteering(0.0)">Straight</button>
                        <button class="steering-button" onclick="setSteering(0.25)">1/4 Right</button>
                        <button class="steering-button" onclick="setSteering(0.5)">1/2 Right</button>
                        <button class="steering-button" onclick="setSteering(1.0)">Full Right</button>
                    </div>
                    <input type="range" class="steering-slider" id="steeringSlider" min="-1" max="1" step="0.01" value="0" oninput="setSteering(this.value)">
                    <p style="text-align: center; margin-top: 5px;">Current: <span id="steering-display">0.00</span></p>
                </div>
            </div>
        </div>
        
        <script>
            const socket = io();
            
            socket.on('connect', function() {
                console.log('Connected to server');
            });
            
            socket.on('disconnect', function() {
                console.log('Disconnected from server');
            });
            
            function setSpeed(position) {
                document.getElementById('speed' + position).checked = true;
                
                // Update display
                let displayText = position + " ";
                if (position > 4) {
                    let level = position - 4;
                    displayText += "(Forward " + level;
                    if (level === 6) {
                        displayText += " Max";
                    }
                    displayText += ")";
                } else if (position < 4) {
                    let reverseLevel = 4 - position;
                    displayText += "(Reverse " + reverseLevel;
                    if (reverseLevel === 3) {
                        displayText += " Max";
                    }
                    displayText += ")";
                } else {
                    displayText += "(NEUTRAL)";
                }
                document.getElementById('speed-display').textContent = displayText;
                
                // Send to server
                socket.emit('speed_update', {position: position});
                console.log('Emitted speed_update with position:', position);
            }
            
            function setSteering(value) {
                value = parseFloat(value);
                document.getElementById('steeringSlider').value = value;
                const displayValue = value.toFixed(2);
                document.getElementById('steering-display').textContent = displayValue;
                document.getElementById('steering-display-bottom').textContent = displayValue;
                
                // Send to server
                socket.emit('steering_update', {value: value});
                console.log('Emitted steering_update with value:', value);
            }
            
            function emergencyStop() {
                setSpeed(4);  // Neutral
                setSteering(0.0);  // Straight
                alert('Emergency Stop Activated! Speed set to Neutral, Steering set to Straight.');
            }
            
            // Make speed radio areas clickable
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
    ''')

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('connect')
def handle_connect():
    print("Client connected")

@socketio.on('disconnect')
def handle_disconnect():
    print("Client disconnected")

@socketio.on('speed_update')
def handle_speed_update(data):
    """Handle speed position updates"""
    global current_speed_position
    try:
        current_speed_position = int(data['position'])
        print(f"Speed position set to: {current_speed_position}")
    except (KeyError, ValueError) as e:
        print(f"Error in speed_update: {e}")

@socketio.on('steering_update')
def handle_steering_update(data):
    """Handle steering updates"""
    global current_steering
    try:
        current_steering = float(data['value'])
        print(f"Steering set to: {current_steering}")
    except (KeyError, ValueError) as e:
        print(f"Error in steering_update: {e}")

if __name__ == '__main__':
    print("Starting Working Overlay + Speed Control...")
    
    # Start camera capture thread
    capture_thread = threading.Thread(target=capture_frames, daemon=True)
    capture_thread.start()
    
    print("Access test at: http://192.168.1.151:5002")
    
    # Start web server
    socketio.run(app, host='0.0.0.0', port=5002, debug=False)