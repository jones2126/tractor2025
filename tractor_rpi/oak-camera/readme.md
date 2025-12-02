# Oak-D Camera Setup and Test Guide for Raspberry Pi 5 (Ubuntu)

This guide explains how to set up and test an Oak-D or other Luxonis DepthAI camera on a Raspberry Pi 5 running Ubuntu, using a **powered USB hub** to ensure stable operation.

---

## 1. Prerequisites

- Raspberry Pi 5 with Ubuntu installed and updated:
  ```bash
  sudo apt update && sudo apt upgrade -y
  ```
- Internet connection (Ethernet or Wi-Fi)
- Powered USB 3.0 hub (recommended to prevent power brownouts)
- Python 3.12 (or newer) preinstalled with `pip`

---

## 2. Connect Hardware

1. Plug the powered USB hub into the Raspberry Pi 5.
2. Connect the Oak-D camera to the hub using a short, high-quality USB 3.0 cable.
3. Verify detection:
   ```bash
   dmesg | tail -n 20
   ```
   You should see messages like:
   ```
   usb 2-2: Product: Luxonis Device
   usb 2-2: Manufacturer: Intel Corporation
   usb 2-2: SerialNumber: <your-serial>
   ```

---

## 3. Install DepthAI (Version 2.30)

DepthAI 2.30.0.0 is the last stable 2.x release and is recommended for codebases that use `ColorCamera` and `XLinkOut`.

```bash
python3 -m pip install "depthai==2.30.0.0" --break-system-packages --force-reinstall
```

Clear any cached firmware to avoid mismatches:

```bash
rm -rf ~/.cache/luxonis
```

---

## 4. Install Additional Dependencies

```bash
python3 -m pip install numpy opencv-python flask --break-system-packages
```

---

## 5. Disable USB Autosuspend (Optional but Recommended)

To prevent random disconnects:

```bash
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend
```

Make it persistent by adding `usbcore.autosuspend=-1` to `/boot/firmware/cmdline.txt` and rebooting.

---

## 6. Run the Test Script

Navigate to your project folder:

```bash
cd ~/tractor2025/tractor/oak-camera
python3 oak_v3_streaming.py
```

You should see:
```
Pipeline started. Waiting for frames...
```

On another computer, open a browser and go to:

```
http://<your-pi-ip>:5000
```

You should see a live video stream from the Oak-D.

---

## 7. Troubleshooting

- **Repeated USB disconnects** → Ensure you are using a powered hub or external power for the camera.
- **X_LINK_ERROR** → Clear cache and try again: `rm -rf ~/.cache/luxonis`
- **No image** → Check that your USB hub is connected to a USB 3.0 port on the Pi (blue port).
- **Low FPS or lag** → If still unstable, test in USB2 mode as a fallback:
  ```bash
  sudo modprobe usbcore
  sudo sh -c 'echo 0 > /sys/bus/usb/devices/usb2/authorized_default'
  ```
  (limits bandwidth but confirms stability issues are USB3 related)

---

## 8. Verification Script (Optional)

For a quick test without Flask, use this script:

```python
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
    for i in range(50):  # Capture ~50 frames
        frame = q.get()
        print(f"Got frame {i+1}")
```

Run it with:

```bash
python3 quick_test.py
```

If you see "Got frame 1", "Got frame 2" ... without errors, the camera is functioning correctly.

---

## 9. Next Steps

- Integrate this pipeline into your ROS2 or teleoperation code.
- Optionally update to DepthAI 3.x in the future, but note that you will need to rewrite parts of your pipeline (ColorCamera → Camera, XLinkOut → MessageOut).

