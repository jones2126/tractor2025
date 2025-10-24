#!/usr/bin/env python3

import cv2
import depthai as dai

def test_working_api():
    """Test using the working API pattern from your example"""
    
    print(f"DepthAI version: {dai.__version__}")
    print("Testing with working API pattern...")
    
    # Create pipeline using the working pattern
    pipeline = dai.Pipeline()
    
    cam_rgb = pipeline.createColorCamera()
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setInterleaved(False)
    
    xout_video = pipeline.createXLinkOut()
    xout_video.setStreamName("video")
    cam_rgb.preview.link(xout_video.input)
    
    print("Pipeline created successfully!")
    
    # Test with device
    try:
        with dai.Device(pipeline) as device:
            print(f"Connected to: {device.getDeviceInfo().mxid}")
            
            video_queue = device.getOutputQueue(name="video", maxSize=4, blocking=False)
            
            print("Capturing test frame...")
            
            frame_count = 0
            while frame_count < 5:
                frame_msg = video_queue.get()
                if frame_msg is not None:
                    frame = frame_msg.getCvFrame()
                    if frame is not None and frame.size > 0:
                        frame_count += 1
                        print(f"Frame {frame_count}: {frame.shape}")
                        
                        if frame_count == 1:
                            cv2.imwrite('working_test.jpg', frame)
                            print("Saved working_test.jpg")
            
            print("✅ Working API test successful!")
            return True
            
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    test_working_api()
