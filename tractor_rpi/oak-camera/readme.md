# OAK-D Camera Setup and Test Guide — Raspberry Pi 5 (Ubuntu)

This guide covers setup and testing of a Luxonis OAK-D or OAK-D Lite camera on a Raspberry Pi 5 running Ubuntu, connected through the BIG7 powered USB hub.

> **Version lock:** Keep DepthAI at `2.30.0.0`. Version 3.x rewrites the API (`ColorCamera` → `Camera`, etc.) and is incompatible with this codebase.

---

## 1. Prerequisites

- Raspberry Pi 5 with Ubuntu installed and updated
- tractor2025 repo cloned at `~/tractor2025`
- BIG7 powered USB hub connected
- OAK-D or OAK-D Lite camera (**OAK-D Lite preferred for bench testing** — USB bus-powered, no external supply needed)

---

## 2. Connect Hardware and Verify Detection

1. Plug the OAK-D Lite into the BIG7 hub.
2. Verify detection:
   ```bash
   lsusb | grep 03e7
   dmesg | tail -n 10
   ```
   You should see:
   ```
   usb 4-1: New USB device found, idVendor=03e7, idProduct=2485
   usb 4-1: Product: Movidius MyriadX
   usb 4-1: Manufacturer: Movidius Ltd.
   ```

---

## 3. Write udev Rule (Non-root USB Access)

Required so depthai can open the camera without sudo:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/99-oak.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Unplug and replug the camera, then confirm:

```bash
lsusb | grep 03e7
```

---

## 4. Install DepthAI 2.30.0.0

Install pip first if not present (Ubuntu Server 24.04 does not include it by default):

```bash
sudo apt install python3-pip -y
```

Then install depthai:

```bash
python3 -m pip install "depthai==2.30.0.0" --break-system-packages --force-reinstall
```

Install additional dependencies:

```bash
python3 -m pip install numpy opencv-python flask --break-system-packages
```

Clear any cached firmware to avoid version mismatches:

```bash
rm -rf ~/.cache/luxonis
```

---

## 5. Disable USB Autosuspend (Recommended)

Prevents random disconnects during long sessions:

```bash
echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend
```

Make it persistent — add `usbcore.autosuspend=-1` to the end of `/boot/firmware/cmdline.txt` (keep everything on one line), then reboot.

---

## 6. Quick Connection Test

```bash
cd ~/tractor2025/tractor_rpi/oak-camera
python3 oak_quick_test.py
```

Expected output:
```
Got frame 1
Got frame 2
...
Got frame 50
```

If you see 50 frames without errors, the camera is working correctly.

---

## 7. Live Stream Test

```bash
cd ~/tractor2025/tractor_rpi/oak-camera
python3 oak_v3_streaming.py
```

Expected output:
```
Pipeline started. Waiting for frames...
```

On another computer, open a browser and go to:
```
http://<rpi-ip>:5000
```

You should see a live video stream.

---

## 8. Troubleshooting

| Symptom | Fix |
|---------|-----|
| Repeated USB disconnects | Ensure BIG7 hub has power supply connected |
| `X_LINK_ERROR` | `rm -rf ~/.cache/luxonis` then retry |
| No frames, no error | Check udev rule exists: `cat /etc/udev/rules.d/99-oak.rules` |
| Low FPS / lag | Test USB2 fallback: `sudo sh -c 'echo 0 > /sys/bus/usb/devices/usb2/authorized_default'` |
| `Permission denied` on USB | udev rule missing or not reloaded — redo Step 3 |

---

## 9. Notes on OAK-D Lite vs OAK-D

| | OAK-D Lite | OAK-D (full) |
|-|-----------|-------------|
| Power | USB bus-powered | May need external supply |
| API | Identical (depthai 2.x) | Identical |
| Stereo baseline | Shorter | Longer (better depth at distance) |
| Best for | Bench testing | Outdoor robot deployment |

Code written and tested on the Lite transfers directly to the full OAK-D.
