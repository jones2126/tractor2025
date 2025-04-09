import cv2
import depthai as dai
from flask import Flask, Response

app = Flask(__name__)

def generate_frames():
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define a source - color camera
    cam_rgb = pipeline.createColorCamera()
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setInterleaved(False)

    # Create output
    xout_video = pipeline.createXLinkOut()
    xout_video.setStreamName("video")
    cam_rgb.preview.link(xout_video.input)

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        video_queue = device.getOutputQueue(name="video", maxSize=4, blocking=False)

        while True:
            video_frame = video_queue.get()
            frame = video_frame.getCvFrame()

            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            # Yield frame in byte format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)