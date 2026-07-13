import depthai as dai

pipeline = dai.Pipeline()
cam = pipeline.createColorCamera()
cam.setPreviewSize(640, 480)
cam.setInterleaved(False)

xout = pipeline.createXLinkOut()
xout.setStreamName("preview")
cam.preview.link(xout.input)

with dai.Device(pipeline) as device:
    q = device.getOutputQueue("preview")
    for i in range(50):
        frame = q.get()
        print(f"Got frame {i + 1}")

print("Test complete — camera is working.")
