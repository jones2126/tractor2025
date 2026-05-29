# Claude Code Context — Tractor Robot Project

## Working Directory
Start Claude Code from this directory: `/home/albert/tractor2025/`

## Who I Am
Al — hobbyist robotics developer. Not a professional software engineer.
Walk me through processes step by step when needed.

## The Robot Project
Autonomous outdoor robot on a **Cub Cadet XT1 LT42 EFI** riding mower chassis.
Full documentation: `obsidian_vault/00-project-overview.md`

### Hardware Summary
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

### RPi Connection
- Hostname: `raspberrypi` | User: `al` | Local IP: `192.168.1.151`
- ZeroTier IP: `192.168.193.76`
- Serial bridge: `460800 baud` USB (`/dev/teensy`)

### Radio Modes
| Mode | Value | Behavior |
|------|-------|----------|
| Pause | 0 | Transmission neutral, steering hold |
| Manual | 1 | Direct RC pot control — switch UP on handheld |
| Auto | 2 | cmd_vel from navigation stack |

### UDP Ports
| Port | Content |
|------|---------|
| 6002 | GPS/RTK state JSON — 20 Hz |
| 6003 | Teensy bridge status broadcast |
| 6004 | cmd_vel commands JSON |

### Current Firmware
| Board | File |
|-------|------|
| Tractor Teensy 4.1 | `tractor_teensy/src/teensy_main_20260518.cpp` |
| Handheld RC (Teensy 3.2) | `radiocontrol_nrf24radio/src/Rccntrl_rf24_20260517.cpp` |

### Active Development Focus (as of 2026-05-20)
- Steering PID tuning — currently kp=1.0, jerky; needs dynamic PWM and deadband work
- Transmission neutral calibration — firmware=2985, field may need ~2800
- IBT-2 Gen2 wiring planned (R_EN, L_EN, R_IS, L_IS) — not yet implemented

### Known Issues
1. Transmission neutral may need field recalibration (~2800 vs firmware 2985)
2. All grounds must be tied — previous Teensy damage from 328Ω between Teensy and JRK G2 grounds
3. DepthAI must stay at version `2.30.0.0` (3.0 is incompatible)
4. IBT-2 overcurrent latch — reset by pulling motor fuse 5 sec; R_EN/L_EN hardwired to 5V currently
5. Pixel hotspot MAC randomization can cause WiFi failures — clear `802-11-wireless.bssid` if needed

---

## Repo Structure
```
tractor2025/
├── CLAUDE.md                     ← this file
├── README.md
├── markdown_cross_reference.csv
├── obsidian_vault/               ← Obsidian engineering notebook (open as vault)
│   ├── 00-project-overview.md   ← hardware ref, key files, RPi connection
│   ├── 01-plan/milestones.md    ← project milestones
│   ├── 02-testing/              ← dated session logs (YYYYMMDD-topic.md)
│   ├── 03-design/               ← component notes, wiring, CAD prompts
│   ├── 04-reference/            ← laptop setup, git workflow, PDFs
│   ├── 05-future-ideas/         ← deferred work (IBT-2 Gen2, etc.)
│   └── attachments/             ← all images
├── tractor_teensy/              ← Teensy 4.1 firmware (PlatformIO)
│   └── src/teensy_main_20260518.cpp
├── radiocontrol_nrf24radio/     ← Handheld RC unit firmware (Teensy 3.2)
├── tractor_rpi/                 ← RPi 5 scripts and testing utilities
│   ├── f9p/                     ← dual F9P RTK scripts
│   ├── light-tower/             ← LED tower control
│   ├── oak-camera/              ← OAK-D WebRTC teleoperation
│   ├── pure-pursuit/            ← path following algorithm
│   ├── servo-board/             ← PCA9685 servo board scripts
│   ├── setup/                   ← RPi setup and install scripts
│   ├── teensy/                  ← Teensy serial bridge scripts
│   ├── testing/                 ← test sketches and scripts
│   └── webrtc/                  ← WebRTC streaming
└── BridgevilleRTKBase/          ← RTK base station (Bridgeville PA)
```

---

## This Machine (Linux Desktop)
- OS: Ubuntu/Linux, AMD Radeon Mullins APU
- GPU driver: `amdgpu` (not `radeon`) — switching to amdgpu fixed UI hanging
- Verify: `lsmod | grep -E 'radeon|amdgpu'` — amdgpu should show >100 users
