#!/usr/bin/env python3
import depthai as dai
import cv2
import threading
import time
from flask import Flask, Response

app = Flask(__name__)
latest_frame = None
lock = threading.Lock()

def create_pipeline():
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    # Use CAM_A (the recommended name for the primary RGB sensor)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(30)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("video")
    cam_rgb.video.link(xout.input)

    return pipeline

def capture_frames_v3():
    global latest_frame
    while True:
        try:
            print("Connecting to device...")
            with dai.Device(create_pipeline()) as device:
                print("Pipeline started. Waiting for frames...")
                q_rgb = device.getOutputQueue(name="video", maxSize=4, blocking=False)

                while True:
                    in_rgb = q_rgb.tryGet()
                    if in_rgb:
                        frame = in_rgb.getCvFrame()
                        with lock:
                            latest_frame = frame
                    time.sleep(0.01)
        except RuntimeError as e:
            print(f"[WARN] Device error: {e}")
            print("Retrying in 3 seconds...")
            time.sleep(3)

@app.route('/')
def index():
    return '<h2>Oak Camera Stream</h2><img src="/video_feed" width="640">'

def generate_mjpeg():
    global latest_frame
    while True:
        with lock:
            if latest_frame is None:
                continue
            frame = latest_frame.copy()
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.05)

@app.route('/video_feed')
def video_feed():
    return Response(generate_mjpeg(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("Starting Oak Camera Teleoperation Station (DepthAI V3)...")
    print("Access the interface at: http://<your-pi-ip>:5000")
    capture_thread = threading.Thread(target=capture_frames_v3, daemon=True)
    capture_thread.start()
    app.run(host='0.0.0.0', port=5000, debug=False)
