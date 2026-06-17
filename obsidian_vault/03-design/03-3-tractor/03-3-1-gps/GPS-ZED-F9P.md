---
title: GPS — ArduSimple ZED-F9P RTK
created: 2026-05-20
modified: 2026-05-20
type: design
status: active
tags:
  - type/design
  - status/active
  - subsystem/gps
---

# GPS — ArduSimple ZED-F9P RTK

**Hardware:** 2× ArduSimple SimpleRTK2B boards (u-blox ZED-F9P chipset)
**Role:** Centimeter-level RTK positioning for autonomous navigation

---

## Configuration

| Item | Detail |
|------|--------|
| Modules | 2× ArduSimple ZED-F9P (moving base + rover) |
| Output rate | 20 Hz |
| Protocol | JSON on UDP port 6002 |
| RTCM corrections | From Bridgeville RTK base station via ZeroTier |
| Connection | USB to RPi 5 |

---

## GPS Status Values

Used throughout firmware and serial bridge to report fix quality:

| Value | Meaning |
|-------|---------|
| 0 | Unset |
| 1 | No NMEA (no data from module) |
| 2 | GPS fix but no RTK |
| 3 | RTK Fix (centimeter accuracy) |

Status is broadcast by the Teensy ACK payload back to the handheld RC unit, which displays it on LED #4 (green=3, yellow=2, red=0-1).

---

## Software Pipeline (RPi 5)

```
ZED-F9P (USB)
    ↓
rtcm_server_20260617.py
    ├── Parses GGA + RELPOSNED sentences
    ├── Forwards RTCM corrections (TCP from Bridgeville base)
    └── Broadcasts JSON status on UDP 6002 at 20 Hz
         ↓
teensy_serial_bridge.py
    ├── Reads UDP 6002
    ├── Sends GPS,<status> to Teensy over serial at 5 Hz
    └── Teensy includes gps_status in NRF24 ACK payload
```

**Key files:**
- `tractor_rpi/rtcm_server_20260617.py` — main GPS pipeline
- `tractor_rpi/teensy_serial_bridge_20251101.py` — serial bridge

---

## RTK Base Station

Corrections come from the Bridgeville PA permanent base station.
See `03-design/03-1-rtk-base/` for base station details.

| | |
|-|-|
| Location | Bridgeville PA |
| ZeroTier IP | 192.168.193.88 |
| Repo | `BridgevilleRTKBase/raspberry-pi/production/` |

---

## UDP Port

| Port | Content |
|------|---------|
| 6002 | GPS/RTK state JSON — 20 Hz |

---

## Notes

