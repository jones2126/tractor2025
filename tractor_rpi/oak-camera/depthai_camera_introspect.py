#!/usr/bin/env python3
import depthai as dai

print("DepthAI Version:", dai.__version__)

# Create a pipeline and a Camera node
pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.Camera, socket=dai.CameraBoardSocket.CAM_A)

print("\n=== Camera Node Type ===")
print(type(cam))

print("\n=== Camera Node Attributes/Methods ===")
print(dir(cam))

if hasattr(cam, "video"):
    print("\n=== Camera.video Type ===")
    print(type(cam.video))
    print("\n=== Camera.video Attributes/Methods ===")
    print(dir(cam.video))
else:
    print("\nCamera node has no 'video' attribute.")

# Optionally list all available node types
print("\n=== Available Node Types ===")
for name in dir(dai.node):
    if not name.startswith("_"):
        print(" -", name)
