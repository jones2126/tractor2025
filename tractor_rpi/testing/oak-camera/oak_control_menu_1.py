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
    """Create a simple working pipeline using the same pattern as your working script"""
    pipeline = dai.Pipeline()
    
    # Use ColorCamera with the same configuration as your working script
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(30)
    
    # Create XLinkOut node (same as your working script)
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("video")
    cam_rgb.video.link(xout.input)
    
    return pipeline

def capture_frames_simple():
    """Frame capture using the same pattern as your working script"""
    global latest_frame
    
    while True:
        try:
            print("Connecting to device...")
            with dai.Device(create_simple_pipeline()) as device:
                print("Pipeline started. Waiting for frames...")
                q_rgb = device.getOutputQueue(name="video", maxSize=4, blocking=False)
                
                frame_count = 0
                start_time = time.time()
                
                while True:
                    in_rgb = q_rgb.tryGet()
                    if in_rgb:
                        frame = in_rgb.getCvFrame()
                        
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
