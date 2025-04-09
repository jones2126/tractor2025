import cv2
import depthai as dai
from flask import Flask, render_template_string, Response, redirect, url_for

app = Flask(__name__)

# Global focus position tracker
focus_position = 135  # Start at mid-range for safety

# Pipeline setup
def create_pipeline():
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.createColorCamera()
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setInterleaved(False)

    xout_video = pipeline.createXLinkOut()
    xout_video.setStreamName("video")
    cam_rgb.preview.link(xout_video.input)

    control_in = pipeline.createXLinkIn()
    control_in.setStreamName('control')
    control_in.out.link(cam_rgb.inputControl)

    return pipeline

# Start the pipeline
pipeline = create_pipeline()
device = dai.Device(pipeline)
control_queue = device.getInputQueue('control')
video_queue = device.getOutputQueue(name="video", maxSize=4, blocking=False)

# Video feed generator
def generate_frames():
    while True:
        frame = video_queue.get().getCvFrame()
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Camera control functions
def set_focus(position):
    global focus_position
    ctrl = dai.CameraControl()
    ctrl.setManualFocus(position)
    control_queue.send(ctrl)
    focus_position = position

def set_auto_focus():
    ctrl = dai.CameraControl()
    ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)
    control_queue.send(ctrl)

# Routes
@app.route('/')
def index():
    return render_template_string('''
        <h1>OAK Camera Control Panel</h1>
        <img src="{{ url_for('video_feed') }}" width="640" height="480">
        <br><br>
        <b>Focus Control:</b><br>
        <a href="{{ url_for('focus_decrease') }}">🔍 Focus -</a>
        <a href="{{ url_for('focus_increase') }}">🔍 Focus +</a>
        <a href="{{ url_for('focus_auto') }}">🎯 Auto-Focus</a>
        <p>Current manual focus position: {{ focus_position }}</p>
    ''', focus_position=focus_position)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/focus/increase')
def focus_increase():
    global focus_position
    focus_position = min(focus_position + 10, 255)
    set_focus(focus_position)
    return redirect(url_for('index'))

@app.route('/focus/decrease')
def focus_decrease():
    global focus_position
    focus_position = max(focus_position - 10, 0)
    set_focus(focus_position)
    return redirect(url_for('index'))

@app.route('/focus/auto')
def focus_auto():
    set_auto_focus()
    return redirect(url_for('index'))

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
