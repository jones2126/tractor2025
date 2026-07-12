#!/usr/bin/env python3

import cv2
import depthai as dai
from flask import Flask, Response, render_template_string
import threading
import time
import numpy as np

app = Flask(__name__)

# Global variables
latest_frame = None
frame_lock = threading.Lock()

def create_simple_pipeline():
    """Create a simple working pipeline"""
    pipeline = dai.Pipeline()
    
    # Use ColorCamera (deprecated but working)
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(640, 480)  # Smaller size for stability
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setFps(15)  # Lower FPS for stability
    
    return pipeline, cam_rgb

def capture_frames_simple():
    """Simple frame capture with traditional device context"""
    global latest_frame
    
    try:
        pipeline, cam_rgb = create_simple_pipeline()
        
        # Create output queue before device connection
        video_queue = cam_rgb.preview.createOutputQueue(maxSize=2, blocking=False)
        print("Output queue created")
        
        # Use traditional device context (more stable)
        with dai.Device(pipeline) as device:
            print(f"Connected to Oak camera: {device.getDeviceInfo().mxid}")
            
            frame_count = 0
            start_time = time.time()
            
            while True:
                try:
                    # Get frame with timeout
                    frame_msg = video_queue.get(timeout=1000)  # 1 second timeout
                    if frame_msg is not None:
                        frame = frame_msg.getCvFrame()
                        
                        if frame is not None and frame.size > 0:
                            frame_count += 1
                            
                            # Add simple overlay
                            cv2.putText(frame, f"Frame: {frame_count}", (10, 30), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            cv2.putText(frame, f"Size: {frame.shape}", (10, 60), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
                            # Update latest frame
                            with frame_lock:
                                latest_frame = frame.copy()
                            
                            # Print progress every 30 frames
                            if frame_count % 30 == 0:
                                elapsed = time.time() - start_time
                                fps = frame_count / elapsed
                                print(f"Captured {frame_count} frames, FPS: {fps:.2f}")
                        
                except Exception as e:
                    print(f"Frame capture error: {e}")
                    time.sleep(0.1)
                    continue
                
                time.sleep(0.033)  # ~30 FPS max
                
    except Exception as e:
        print(f"Error in capture_frames_simple: {e}")
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
                # Create a waiting frame
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "Waiting for camera...", (50, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ret:
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.05)  # 20 FPS streaming

@app.route('/')
def index():
    """Simple web interface"""
    html_template = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Simple Oak Camera Stream</title>
        <style>
            body { 
                font-family: Arial, sans-serif; 
                background: #1a1a1a; 
                color: #00ff00; 
                text-align: center;
                padding: 20px;
            }
            .video-stream { 
                border: 2px solid #00ff00; 
                border-radius: 10px; 
                max-width: 90%;
            }
            h1 { color: #00ff00; }
            .info {
                background: #333;
                padding: 10px;
                border-radius: 5px;
                margin: 20px auto;
                max-width: 600px;
            }
        </style>
    </head>
    <body>
        <h1>🤖 Simple Oak Camera Stream</h1>
        
        <div class="info">
            <p>Direct video feed from Oak Camera</p>
            <p>Stream URL: <code>http://192.168.1.151:5000/video_feed</code></p>
            <p>Using DepthAI traditional pipeline approach</p>
        </div>
        
        <img id="videoStream" src="/video_feed" class="video-stream" alt="Live Stream">
        
        <script>
            // Auto-refresh if image fails to load
            document.getElementById('videoStream').onerror = function() {
                setTimeout(() => {
                    this.src = '/video_feed?' + Date.now();
                }, 1000);
            };
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

if __name__ == '__main__':
    print("Starting Simple Oak Camera Stream...")
    print("Access at: http://192.168.1.151:5000")
    
    # Start capture thread
    capture_thread = threading.Thread(target=capture_frames_simple, daemon=True)
    capture_thread.start()
    
    # Give camera time to initialize
    time.sleep(3)
    
    # Start Flask server
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
