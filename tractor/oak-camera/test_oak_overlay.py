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
current_speed_position = 6  # Neutral (0-12 range)

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

def generate_frames():
    """Generate video frames with GUARANTEED overlay"""
    global latest_frame, current_speed_position
    
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
        
        # Lowered position and reduced font size
        # NEW: Changed to light blue background with black text
        # BGR format: (255, 255, 0) is yellow, (255, 255, 255) is white
        cv2.rectangle(frame, (50, 450), (500, 600), (255, 200, 100), -1)  # Light blue background
        cv2.rectangle(frame, (50, 450), (500, 600), (0, 255, 0), 5)   # Thick green border
        
        # Reduced font size and adjusted y positions
        cv2.putText(frame, "SPEED CONTROL", (80, 500), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)  # Black text
        
        # Use current_speed_position variable for dynamic text
        cv2.putText(frame, f"Position: {current_speed_position}", (80, 550), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)  # Black text
        
        # Encode as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ret:
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.033)  # Normal speed

@app.route('/')
def index():
    """Serve web page with speed controls"""
    return render_template_string('''
    <!DOCTYPE html>
    <html>
    <head>
        <title>Working Overlay + Speed Control</title>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.5/socket.io.js"></script>
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
            }
            .video { 
                border: 2px solid #00ff00; 
                width: 100%;
                height: 70vh;
                object-fit: cover;
                background: #000;
            }
            .controls-section {
                flex: 0 0 200px;
                background: #2a2a2a;
                padding: 15px;
                border: 1px solid #00ff00;
                border-radius: 10px;
                height: fit-content;
            }
            .speed-control {
                display: flex;
                flex-direction: column;
                gap: 6px;
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
        <h1>Working Red Overlay + Speed Control</h1>
        <div class="container">
            <div class="video-section">
                <img id="videoStream" src="/video_feed" class="video" alt="Live Stream">
                <p style="color: white; text-align: center; margin-top: 10px;">
                    Red overlay working. Speed: <span id="speed-display">6 (NEUTRAL)</span>
                </p>
            </div>
            
            <div class="controls-section">
                <h2>Speed Control</h2>
                <div class="speed-control">
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
        
        <script>
            const socket = io();
            
            function setSpeed(position) {
                document.getElementById('speed' + position).checked = true;
                
                // Update display
                let displayText = position + " ";
                if (position > 6) {
                    displayText += "(Forward " + (position - 6) + ")";
                } else if (position < 6) {
                    displayText += "(Reverse " + (6 - position) + ")";
                } else {
                    displayText += "(NEUTRAL)";
                }
                document.getElementById('speed-display').textContent = displayText;
                
                // Send to server
                socket.emit('speed_update', {position: position});
                console.log('Speed set to position:', position);
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

@socketio.on('speed_update')
def handle_speed_update(data):
    """Handle speed position updates"""
    global current_speed_position
    current_speed_position = data['position']
    print(f"Speed position set to: {current_speed_position}")

if __name__ == '__main__':
    print("Starting Working Overlay + Speed Control...")
    
    # Start camera capture thread
    capture_thread = threading.Thread(target=capture_frames, daemon=True)
    capture_thread.start()
    
    print("Access test at: http://192.168.1.151:5002")
    
    # Start web server
    socketio.run(app, host='0.0.0.0', port=5002, debug=False)