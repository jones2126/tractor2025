---
title: GPS — ArduSimple ZED-X20D Dual-Antenna Heading
created: 2026-05-29
modified: 2026-05-29
tested: 2026-05-29
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
| Heading message | `UBX-NAV-RELPOSNED` (class 0x01, ID 0x3C) | `UBX-NAV-DAHEADING` (class 0x01, ID 0x45) |
| "Relative" meaning | Position of rover *relative* to base, in NED frame | Heading of antenna baseline, same NED frame |
| Heading reference | True North (clockwise) | True North (clockwise) |

Both approaches give absolute heading referenced to True North — the word "relative" in RELPOSNED describes the *position* vector between the two receivers, not a relative angle. RELPOSNED is the wrong message for the X20D because the X20D is a single device; DAHEADING is its native output.

Note: The X20D uses message ID **0x45** for DAHEADING, not 0x3D (which is the ZED-F9H's ID). This was confirmed by reverse-engineering the live payload on 2026-05-29.

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

## Message Structure (0x01 0x45, 64-byte payload)

Confirmed by live bench test on 2026-05-29. Structure mirrors RELPOSNED but without refStationId.

| Offset | Type | Field | Units | Notes |
|--------|------|-------|-------|-------|
| 0 | U1 | version | — | = 1 |
| 1–3 | — | reserved | — | |
| 4–7 | U4 | iTOW | ms | GPS time of week |
| 8–11 | I4 | relPosN | cm | N component of GPS1→GPS2 baseline |
| 12–15 | I4 | relPosE | cm | E component |
| 16–19 | I4 | relPosD | cm | D component |
| 20–23 | U4 | relPosLength | cm | Baseline length |
| 24–27 | U4 | relPosHeading | 1e-5 deg | **True North clockwise heading** |
| 28–31 | — | reserved | — | |
| 32 | I1 | relPosHPN | 0.1 mm | HP correction for N |
| 33 | I1 | relPosHPE | 0.1 mm | HP correction for E |
| 34 | I1 | relPosHPD | 0.1 mm | HP correction for D |
| 35 | I1 | relPosHPLength | 0.1 mm | HP correction for length |
| 36–39 | U4 | accN | 0.1 mm | |
| 40–43 | U4 | accE | 0.1 mm | |
| 44–47 | U4 | accD | 0.1 mm | |
| 48–51 | U4 | accLength | 0.1 mm | |
| 52–55 | U4 | accHeading | 1e-5 deg | Heading accuracy |
| 56–59 | — | reserved | — | |
| 60–63 | U4 | flags | — | bit 8 = headingValid, bits 3–4 = carrSoln |

**Bench test results (GPS on desk, stationary):**
- Baseline: 82 cm, Heading: ~187.4°, Heading accuracy: ~0.69°, carrSoln: fixed

---

## Related

- [[GPS-ZED-F9P]] — original two-receiver RTK positioning setup
- [[03-1-rtk-base]] — Bridgeville PA RTK base station (still used for centimeter-level position fix)
- `tractor_rpi/testing/parseDAHEADING.py` — live parser for this message
- `tractor_rpi/testing/ubx_message_sniffer.py` — general UBX/NMEA message monitor
