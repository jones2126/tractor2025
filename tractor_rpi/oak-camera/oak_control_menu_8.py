#!/usr/bin/env python3

import cv2
import depthai as dai
from flask import Flask, Response, render_template_string
from flask_socketio import SocketIO, emit
import threading
import time
import numpy as np
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

app = Flask(__name__)
app.config['SECRET_KEY'] = 'robot_test_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables
latest_frame = None
frame_lock = threading.Lock()
current_speed_position = 4  # Neutral (1-10 range, 4 = Neutral)
current_steering = 0.0  # Steering from -1.0 (full left) to 1.0 (full right)

# PCA9685 setup for servo control
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Standard servo frequency

# Create servo objects for channels 0 (speed) and 1 (steering)
speed_servo = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)  # Channel 0 for speed
steering_servo = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)  # Channel 1 for steering

# Servo calibration mappings (adjust these based on your hardware)
# Speed: Position 1 (Reverse Max) = 0°, Position 10 (Forward Max) = 180°
SPEED_MIN_ANGLE = 0    # For position 1 (Reverse 3 Max)
SPEED_MAX_ANGLE = 180  # For position 10 (Forward 6 Max)
# Steering: -1.0 (Full Left) = 0°, 0.0 (Straight) = 90°, 1.0 (Full Right) = 180°
STEERING_MIN_ANGLE = 0   # Full left
STEERING_CENTER_ANGLE = 90  # Straight
STEERING_MAX_ANGLE = 180  # Full right

def set_speed_servo(position):
    """Set speed servo angle based on position (1-10)"""
    # Clamp position to 1-10 range
    position = max(1, min(10, position))
    # Linear mapping from position 1-10 to angles 0-180°
    angle = SPEED_MIN_ANGLE + (position - 1) * (SPEED_MAX_ANGLE - SPEED_MIN_ANGLE) / 9
    speed_servo.angle = round(angle)
    print(f"Speed position {position} -> Servo angle: {round(angle)}°")

def set_steering_servo(value):
    """Set steering servo angle based on value (-1.0 to 1.0)"""
    # Clamp value to -1.0 to 1.0
    value = max(-1.0, min(1.0, value))
    # Linear mapping: -1.0 -> 0°, 0.0 -> 90°, 1.0 -> 180°
    if value < 0:
        angle = STEERING_CENTER_ANGLE + value * (STEERING_CENTER_ANGLE - STEERING_MIN_ANGLE)
    else:
        angle = STEERING_CENTER_ANGLE + value * (STEERING_MAX_ANGLE - STEERING_CENTER_ANGLE)
    steering_servo.angle = round(angle)
    print(f"Steering value {value} -> Servo angle: {round(angle)}°")

def emergency_stop_servos():
    """Set both servos to neutral positions"""
    speed_servo.angle = (SPEED_MIN_ANGLE + SPEED_MAX_ANGLE) / 2  # ~90° neutral
    steering_servo.angle = STEERING_CENTER_ANGLE  # 90° straight
    print("Emergency stop: Servos set to neutral")

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
        
        # Draw light blue overlay with telemetry at top-left
        overlay_width = 500
        overlay_height = 160
        left = 50
        top = 100
        right = left + overlay_width
        bottom = top + overlay_height
        cv2.rectangle(frame, (left, top), (right, bottom), (255, 200, 200), -1)  # Light blue
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 5)   # Green border
        
        cv2.putText(frame, "SPEED CONTROL", (left + 30, top + 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        speed_text = f"Position {current_speed_position}, Mode: {get_speed_description(current_speed_position)}"
        cv2.putText(frame, speed_text, (left + 30, top + 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        steering_text = f"Steering: {current_steering:.2f}"
        cv2.putText(frame, steering_text, (left + 30, top + 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # Encode as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ret:
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.033)

@app.route('/')
def index():
    return render_template_string('''
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robot Teleoperation Station</title>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.5/socket.io.min.js"></script>
        <style>
            body { background: #1a1a1a; color: #00ff00; font-family: 'Courier New', monospace; margin: 0; padding: 10px; }
            .container { display: flex; gap: 15px; height: calc(100vh - 20px); }
            .video-section { flex: 1; display: flex; flex-direction: column; position: relative; }
            .video { border: 2px solid #00ff00; width: 100%; height: 70vh; object-fit: cover; background: #000; }
            .emergency-button { position: absolute; top: 10px; right: 10px; background: red; color: white; padding: 10px 20px; font-weight: bold; border: none; border-radius: 5px; cursor: pointer; z-index: 10; }
            .controls-section { flex: 0 0 200px; background: #2a2a2a; padding: 15px; border: 1px solid #00ff00; border-radius: 10px; height: fit-content; }
            .speed-control, .steering-control { display: flex; flex-direction: column; gap: 6px; }
            .steering-buttons { display: flex; justify-content: space-between; gap: 5px; }
            .steering-button { flex: 1; background: #444; color: #00ff00; padding: 8px; border: none; border-radius: 3px; cursor: pointer; font-size: 12px; }
            .steering-button:hover { background: #555; }
            .steering-slider { width: 100%; margin: 10px 0; }
            .speed-radio { display: flex; align-items: center; gap: 8px; padding: 6px 8px; background: #444; border-radius: 3px; cursor: pointer; font-size: 12px; }
            .speed-radio input[type="radio"] { transform: scale(0.9); }
            .speed-radio.neutral { background: #555; border: 1px solid #ffff00; font-weight: bold; }
            .speed-radio:hover { background: #555; }
            h1 { color: #00ff00; text-align: center; margin: 5px 0 15px 0; font-size: 24px; }
            h2 { color: #00ff00; text-align: center; margin: 8px 0; font-size: 16px; }
        </style>
    </head>
    <body>
        <h1>Robot Teleoperation Station</h1>
        <div class="container">
            <div class="video-section">
                <img id="videoStream" src="/video_feed" class="video" alt="Live Stream">
                <button class="emergency-button" onclick="emergencyStop()">EMERGENCY STOP</button>
                <p style="color: white; text-align: center; margin-top: 10px;">
                    Overlay working. Speed: <span id="speed-display">4 (NEUTRAL)</span> | Steering: <span id="steering-display-bottom">0.00</span>
                </p>
            </div>
            
            <div class="controls-section">
                <h2>Speed Control (1-10)</h2>
                <div class="speed-control">
                    <div class="speed-radio" onclick="setSpeed(1)"><input type="radio" name="speed" value="1" id="speed1"><label for="speed1">Reverse 3 (Max)</label></div>
                    <div class="speed-radio" onclick="setSpeed(2)"><input type="radio" name="speed" value="2" id="speed2"><label for="speed2">Reverse 2</label></div>
                    <div class="speed-radio" onclick="setSpeed(3)"><input type="radio" name="speed" value="3" id="speed3"><label for="speed3">Reverse 1</label></div>
                    <div class="speed-radio neutral" onclick="setSpeed(4)"><input type="radio" name="speed" value="4" id="speed4" checked><label for="speed4">NEUTRAL (STOP)</label></div>
                    <div class="speed-radio" onclick="setSpeed(5)"><input type="radio" name="speed" value="5" id="speed5"><label for="speed5">Forward 1 (Min)</label></div>
                    <div class="speed-radio" onclick="setSpeed(6)"><input type="radio" name="speed" value="6" id="speed6"><label for="speed6">Forward 2</label></div>
                    <div class="speed-radio" onclick="setSpeed(7)"><input type="radio" name="speed" value="7" id="speed7"><label for="speed7">Forward 3</label></div>
                    <div class="speed-radio" onclick="setSpeed(8)"><input type="radio" name="speed" value="8" id="speed8"><label for="speed8">Forward 4</label></div>
                    <div class="speed-radio" onclick="setSpeed(9)"><input type="radio" name="speed" value="9" id="speed9"><label for="speed9">Forward 5</label></div>
                    <div class="speed-radio" onclick="setSpeed(10)"><input type="radio" name="speed" value="10" id="speed10"><label for="speed10">Forward 6 (Max)</label></div>
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
            socket.on('connect', function() { console.log('Connected to server'); });
            socket.on('disconnect', function() { console.log('Disconnected from server'); });
            
            function setSpeed(position) {
                document.getElementById('speed' + position).checked = true;
                let displayText = position + " ";
                if (position > 4) {
                    let level = position - 4;
                    displayText += "(Forward " + level + (level === 6 ? " Max" : "") + ")";
                } else if (position < 4) {
                    let reverseLevel = 4 - position;
                    displayText += "(Reverse " + reverseLevel + (reverseLevel === 3 ? " Max" : "") + ")";
                } else {
                    displayText += "(NEUTRAL)";
                }
                document.getElementById('speed-display').textContent = displayText;
                socket.emit('speed_update', {position: position});
                console.log('Speed set to:', position);
            }
            
            function setSteering(value) {
                value = parseFloat(value);
                document.getElementById('steeringSlider').value = value;
                const displayValue = value.toFixed(2);
                document.getElementById('steering-display').textContent = displayValue;
                document.getElementById('steering-display-bottom').textContent = displayValue;
                socket.emit('steering_update', {value: value});
                console.log('Steering set to:', value);
            }
            
            function emergencyStop() {
                setSpeed(4);
                setSteering(0.0);
                alert('Emergency Stop Activated!');
            }
            
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
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('connect')
def handle_connect():
    print("Client connected")

@socketio.on('disconnect')
def handle_disconnect():
    print("Client disconnected")

@socketio.on('speed_update')
def handle_speed_update(data):
    global current_speed_position
    try:
        current_speed_position = int(data['position'])
        set_speed_servo(current_speed_position)  # Move speed servo
        print(f"Speed position set to: {current_speed_position}")
    except (KeyError, ValueError) as e:
        print(f"Error in speed_update: {e}")

@socketio.on('steering_update')
def handle_steering_update(data):
    global current_steering
    try:
        current_steering = float(data['value'])
        set_steering_servo(current_steering)  # Move steering servo
        print(f"Steering set to: {current_steering}")
    except (KeyError, ValueError) as e:
        print(f"Error in steering_update: {e}")

if __name__ == '__main__':
    print("Starting Robot Teleoperation with Servo Control...")
    
    # Initialize servos to neutral
    set_speed_servo(4)  # Neutral
    set_steering_servo(0.0)  # Straight
    
    # Start camera capture thread
    capture_thread = threading.Thread(target=capture_frames, daemon=True)
    capture_thread.start()
    
    print("Access at: http://192.168.1.151:5002")
    
    try:
        socketio.run(app, host='0.0.0.0', port=5002, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        emergency_stop_servos()
        try:
            pca.deinit()
        except:
            pass