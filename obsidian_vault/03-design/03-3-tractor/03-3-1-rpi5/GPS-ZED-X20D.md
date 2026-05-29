---
title: GPS — ArduSimple ZED-X20D Dual-Antenna Heading
created: 2026-05-29
modified: 2026-05-29
type: design
status: active
tags:
  - type/design
  - status/active
  - subsystem/gps
---

# GPS — ArduSimple ZED-X20D Dual-Antenna Heading

**Hardware:** ArduSimple simpleRTK 4 Dual (u-blox ZED-X20D chipset)  
**Order:** ArduSimple #139044  
**Role:** True-North heading for autonomous navigation (replaces two-F9P moving baseline approach)

---

## How This Differs from the ZED-F9P Setup

| | ZED-F9P (moving baseline) | ZED-X20D |
|---|---|---|
| Hardware | 2 separate receivers — one base, one rover | 1 receiver with 2 antennas |
| Heading message | `UBX-NAV-RELPOSNED` | `UBX-NAV-DAHEADING` |
| "Relative" meaning | Position of rover *relative* to base, in NED frame | Heading of antenna baseline, same NED frame |
| Heading reference | True North (clockwise) | True North (clockwise) |

Both approaches give absolute heading referenced to True North — the word "relative" in RELPOSNED describes the *position* vector between the two receivers, not a relative angle. RELPOSNED is the wrong message for the X20D because the X20D is a single device; DAHEADING is its native output.

---

## Heading Definition

The heading output is the **clockwise angle from True North** to the baseline running from the master antenna (GPS1) to the slave antenna (GPS2). Mount GPS1 at the rear and GPS2 at the front of the tractor so that baseline heading = vehicle forward heading with no offset correction needed.

---

## Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| `CFG-RATE-MEAS` | 100 (10 Hz) | ArduSimple ships at 1000 (1 Hz) — this is a saved config, not a hardware limit |
| `CFG-MSGOUT-UBX-NAV-DAHEADING-USB` | 1 (enabled) | Native heading message for this device |

**Rate note:** ArduSimple preconfigures the module at 1 Hz. A June/July 2026 firmware update is expected to officially support higher rates. The 10 Hz setting can be written now and re-applied after any firmware flash.

---

## Firmware

- ArduSimple ships with firmware hardcoded to 1 Hz output
- Latest firmware (as of May 2026): `UBX_20_HDG200B01_TL2438.1438ae0d2967adc09de967f2d4c382bc.zip`
- Flash via u-center 2 with all checkboxes unselected (tool performs 2 passes automatically)
- Re-apply configuration after flashing

---

## Related

- [[GPS-ZED-F9P]] — original two-receiver RTK positioning setup
- [[03-1-rtk-base]] — Bridgeville PA RTK base station (still used for centimeter-level position fix)
