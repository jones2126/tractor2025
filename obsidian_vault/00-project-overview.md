# tractor2025 — Project Overview

Autonomous outdoor robot on a **Cub Cadet XT1 LT42 EFI** riding mower chassis.
**GitHub:** [jones2126/tractor2025](https://github.com/jones2126/tractor2025)

---

## Current Milestone Focus

**A — Radio Manual Control** ← IN PROGRESS
See [[milestones]] for full milestone list.

---

## Hardware Summary

| Component | Details |
|-----------|---------|
| Low-level controller | Teensy 4.1 — steering PID, transmission, NRF24 radio, e-stop |
| Primary compute | Raspberry Pi 5 — navigation, GPS, teleoperation |
| Steering | IBT-2 H-bridge, PID (kp=1.0, ki=0.0, kd=0.0), pot feedback A9 |
| Transmission | Pololu JRK G2 21v3 linear actuator, 10-bucket system, neutral=2985 |
| GPS | 2× ArduSimple ZED-F9P RTK, JSON on UDP 6002 at 20 Hz |
| Radio | NRF24L01, 14-byte struct at 10 Hz, channel 76, 250KBPS |
| Camera | OAK-D stereo (WebRTC teleoperation) — DepthAI must stay at 2.30.0.0 |
| LED tower | PCA9685 via I2C, channels 8–12 |

---

## RPi Connection

**tractor01 (primary)**

| | |
|-|-|
| Hostname | `raspberrypi` |
| User | `al` |
| Local IP | `192.168.1.151` |
| ZeroTier IP | `192.168.193.76` (network `ztuze7ml6g`) |
| Serial | `/dev/teensy` — 460800 baud |

**tractor02 (second RPi 5)**

| | |
|-|-|
| Hostname | `tractor02` |
| User | `al` |
| Local IP | `192.168.1.214` |
| ZeroTier IP | `192.168.193.48` (network `9f77fc393e0a16f8`) |

## Radio Modes

| Mode | Value | Behavior |
|------|-------|----------|
| Pause | 0 | Transmission neutral, steering hold |
| Manual | 1 | Direct RC pot control — switch UP on handheld |
| Auto | 2 | cmd_vel from navigation stack |

## UDP Ports

| Port | Content |
|------|---------|
| 6002 | GPS/RTK state JSON — rtcm_server → bridge, LED |
| 6003 | Teensy bridge status broadcast |
| 6004 | cmd_vel commands JSON |

---

## Known Issues / Critical Notes

1. Transmission neutral may need field recalibration — use `jrkG2_range_test` to read actual position
2. All grounds must be tied — previous Teensy damage traced to 328Ω between Teensy and JRK G2 grounds
3. DepthAI must stay at version `2.30.0.0` — version 3.0 is incompatible
4. Pixel hotspot MAC randomization can cause WiFi failures — clear `802-11-wireless.bssid` if needed
5. IBT-2 overcurrent latch — reset by pulling motor fuse 5 sec; EN pins hardwired to 5V (Gen2 wiring planned)

---

## Design Notes — Component Index

| Component | Notes location |
|-----------|---------------|
| Teensy 4.1 | `03-design/03-3-tractor/03-3-2-teensy-4-1/` |
| IBT-2 steering driver | `03-design/03-3-tractor/03-3-4-ibt2/` |
| JRK G2 transmission | `03-design/03-3-tractor/03-3-5-jrk-g2/` |
| NRF24 radio | `03-design/03-3-tractor/03-3-6-nrf24/` |
| OAK-D camera | `03-design/03-3-tractor/03-3-7-oak-camera/` |
| RPi 5 | `03-design/03-3-tractor/03-3-1-rpi5/` |
| RTK base station | `03-design/03-1-rtk-base/` |
| Radio control unit | `03-design/03-2-radio-control/` |
| IBT-2 Gen2 wiring plan | `05-future-ideas/IBT-2_Next_Generation.md` |

---

## Key Production Files

Local clone: `~/tractor2025/`

### Tractor Teensy 4.1 Firmware
**Current:** `tractor_teensy/src/teensy_main_20260518.cpp`
Steering PID, transmission (JRK G2), NRF24 radio receive, e-stop relay, serial bridge to RPi.
Archive of older versions: `tractor_teensy/src/archive/`

### RC Unit Firmware (Handheld Transmitter — Teensy 3.2)
**Current:** `radiocontrol_nrf24radio/src/Rccntrl_rf24_20260517.cpp`
NRF24 transmit at 10 Hz, ACK receive, 4× PL9823 LEDs, mode switch, pots.

### Teensy Serial Bridge (RPi)
**Current:** `tractor_rpi/teensy_serial_bridge_20251101.py`
Bidirectional serial↔UDP bridge, GPS status forwarding to Teensy, cmd_vel relay.

### RTCM Server / GPS Pipeline (Tractor RPi)
**Current:** `tractor_rpi/rtcm_server_20260306.py`
RTCM TCP→serial forwarding, GGA + RELPOSNED parsing, JSON broadcast on UDP 6002 at 20 Hz.

### LED Status Controller (RPi)
**Current:** `tractor_rpi/led_status_controller.py`
PCA9685 LED tower control based on GPS and radio status.

### Teleop Console (RPi)
**Current:** `tractor_rpi/webrtc/teleop_console.py`
aiohttp/aiortc WebRTC server, OAK-D video stream, telemetry dashboard.

### Bridgeville RTK Base Station
**Current:** `BridgevilleRTKBase/raspberry-pi/production/rtcm_server_0714.py`
Base station RTCM corrections server — Bridgeville PA (ZeroTier: 192.168.193.88).

### Testing / Utilities
**Location:** `tractor_rpi/testing/`
Includes: `git_sync.sh`, IBT-2 tests, JRK tests, Ethernet logger phases 1–4, IBT-2 Gen2.

---

## This Machine (Linux Desktop)

- OS: Ubuntu/Linux, AMD Radeon Mullins APU
- GPU driver: `amdgpu` (not `radeon`) — switching to amdgpu fixed UI hanging
- Verify: `lsmod | grep -E 'radeon|amdgpu'` — amdgpu should show >100 users
