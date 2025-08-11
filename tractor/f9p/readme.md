# Moving-Baseline RTK Using Dual ArduSimple ZED‑F9P's
_Last updated: Aug 2025_

This guide helps configure two u-blox ZED-F9P GPS receivers for Moving Baseline RTK to provide RTK Fix position (i.e. 2-5cm position accuracy) and heading for tractor navigation (e.g. Pure Pursuit path following).  The configuration settings are set to minimize the CPU load by limiting the messages to the select messages required and no more based on a starting point of a factory default reset. The base link position and heading will be updated at 10Hz.  The "base" link" F9P will pull RTCM correction using TCP socket 6001 over Wi-Fi. The F9P will publish lat, lon, RTK status and moving‑baseline RTK heading data via a UDP socket 4243. The distance between the two GPS antennaes will be ~1 meter. One goal is for heading accuracy to be +/- 0.5°, hopefully!


- **Base Link F9P** (rear, over base_link): acts as the moving base, is physically connected to the 'tractor' RPi 5 as /dev/gps-base-link, receives RTCM corrections via a TCP socket on the local network and sends RTCM to the front unit over UART1. Also streams NMEA GGA & RMC sentences to the 'tractor' RPi 5 over USB for navigation and status.  The script used to accomplish this is /home/al/tractor2025/tractor/rtcm_server.py.
- **Heading F9P** (front, ~1–2 m forward): receives RTCM from Base Link F9P via UART1 and outputs UBX‑NAV‑RELPOSNED (heading + baseline) to the 'tractor' RPi 5 over USB /dev/gps-heading.  For testing, the script /home/al/tractor2025/tractor/testing/gps_udp_listener.py is an example of reading the data published UDP_PORT 4242 from rtcm_server.py.

---

## 1. Wire both units
```
Base Link F9P UART1 TX  --->  Heading F9P UART1 RX (Optionally wire RX<->TX, but TX->RX is enough for now.)
Base Link F9P GND       --->  Heading F9P GND
Base Link F9P 3.3V      --->  Base Link F9P IOREF (do not cross IOREF between boards)
Heading F9P 3.3V        --->  Heading F9P IOREF

```

---

## Global order of operations (done per‑unit while connected over USB using u-center)
1. **Factory reset** the unit.
2. **Limit output messages first** (reduce load), verify on **Packet Console**.
3. **Enable only required ports/protocols**.
4. **Raise message rates to 10 Hz**.
5. **Save configuration** .

---

## 2 Base Link F9P (rear unit) — Configuration
This unit sends RTCM3 corrections to the Heading F9P via UART1, and output NMEA GGA & RMC sentences to the RPi 5 via USB. This F9P also receives RTCM corrections via TCP port 6001.

### A. Factory reset
`View → Configuration View (Ctrl+F9) → CFG → Revert to default configuration → Send`

### B. Configure messages starting at 1 Hz
`View → Configuration View → MSG (Messages)`  
Factory defaults enable a few NMEA sentences; disable all but **GGA** and **RMC** on **USB**:
- **Update** **F0‑00 NMEA‑GGA** → USB rate = **1**; Deselect all other ports
- **Update** **F0‑04 NMEA‑RMC** → USB rate = **1**; Deselect all other ports
- **Disable** all other messages.

**Verify** (View → Packet Console): you should see only `$GxGGA` and `$GxRMC` at **1 Hz**.  Disable any that are still showing.

#### **GGA Message Content (What You Get)**

```
$GPGGA,hhmmss.ss,ddmm.mmmm,N,dddmm.mmmm,W,q,ss,h.h,a.a,M,g.g,M,t.t,iii*hh

Where:
- hhmmss.ss: Time (UTC)
- ddmm.mmmm,N: Latitude 
- dddmm.mmmm,W: Longitude
- q: Fix quality (0=Invalid, 1=GPS, 2=DGPS, 4=RTK Fixed (best accuracy, ~2-5cm), 5=RTK Float (good accuracy, ~10-20cm))
- ss: Number of satellites
- h.h: HDOP (horizontal dilution of precision)
- a.a: Altitude above mean sea level
- g.g: Height of geoid above WGS84 ellipsoid
- t.t: Time since last DGPS update
- iii: DGPS station ID
```

```
$GNRMC,hhmmss.ss,A,ddmm.mmmm,N,dddmm.mmmm,W,s.s,c.c,ddmmyy,m.m,E,mode*hh

Where:
- hhmmss.ss: Time (UTC) - hours, minutes, seconds with decimal
- A: Status (A=Active/Valid, V=Void/Invalid)
- ddmm.mmmm,N: Latitude (degrees + decimal minutes, North/South)
- dddmm.mmmm,W: Longitude (degrees + decimal minutes, East/West)
- s.s: Speed over ground (knots)
- c.c: Course over ground (degrees true north)
- ddmmyy: Date (day, month, year)
- m.m: Magnetic variation (degrees)
- E: Magnetic variation direction (E=East, W=West)
- mode: Mode indicator (A=Autonomous, D=DGPS, R=RTK, F=RTK Float, E=Dead Reckoning)
- hh: Checksum
```


Enable these **RTCM3** messages on **UART1** (rate = **1**):

- **F5-4D RTCM3.3 1077** (GPS MSM7)
- **F5-57 RTCM3.3 1087** (GLONASS MSM7) — only if GLONASS enabled
- **F5-61 RTCM3.3 1097** (Galileo MSM7) — only if Galileo enabled
- **F5-7F RTCM3.3 1127** (BeiDou MSM7) — only if BeiDou enabled
- **F5-E6 RTCM3.3 1230** (GLONASS code‑phase biases) — only if GLONASS enabled
- **F5-FE RTCM3.3 4072.0** (u‑blox Ref‑Station PVT for moving base - drives RELPOSNED)
- **F5-FD RTCM3.3 4072.1** (u‑blox Additional Info for moving base - drives RELPOSNED)

### C. Configure UART1 to **output RTCM3** *(start at 1 Hz)*
`View → Configuration View → PRT (Ports)`
- **Target: UART1**
- **Protocol in**: **None**
- **Protocol out**: **5 - RTCM3**
- **Baud**: **115200**
- **Send**

### D. Configure USB to **receive RTCM3** (and optionally UBX) from the Pi
`View → Configuration View → PRT (Ports)`
- **Target: USB**
- **Protocol in**: **RTCM3** (from NTRIP client) **(+ UBX optional for future config)**
- **Protocol out**: **NMEA** (only GGA+RMC as set above)  
- **Send**

### E. Disable unused interfaces (optional CPU/IO savings)
`PRT (Ports)`: set **I2C**, **UART2**, **SPI** → **Protocol in/out = None** → **Send**


### F. **Access Navigation Configuration**
**Click "NAV5 (Navigation 5)"** in the left panel; 

##### **UBX-CFG-NAV5 Model Settings**

- **Dynamic Platform Model**: Select **"11-Mower"**
- **Fix Mode**: Select **"3 – Auto 2D/3D"**
- **UTC Standard**: Select **"0 – Automatic"**
- **Min SV Elevation**: leave as is **"10"**
- **C/N0 Thresholdd**: leave as is **"0"**
- **DR Timeout**: leave as is **"0"**
- **PDOP Mask**: leave as is **"25"**
- **TDOP Mask**: leave as is **"25"**
- **P Acc Mask**: Change to **"0"**; RTK Fix is usually < 0.02 m. Setting it to 0 m essentially disables this filter and lets the F9P output positions even in degraded conditions (useful if RTCM link drops temporarily).
- **P Acc ADR Mask**: leave as is **"0"**
- **T Acc Mask**: leave as is **"350"**
- **Static Hold Threshold**: leave as is **"0"**
- **Static Hold Exit Dist**: leave as is **"0"**
- **DGNSS Timeout**: leave as is **"60"**

##### **Apply Configuration**; **Click "Send"** to apply the navigation settings.



### E. Raise rates to **10 Hz** *(after verification)*
- `RATE (Rates):`
  - **Time Source**: → **1 - GPS Time**
  - **Measurement Period**: **100 ms** (10 Hz) - down from 1000
  - **Other items**: leave along
  - **Send**

### G. Save
`CFG → Save current configuration to **BBR + Flash + I2C + SPI-FLASH**.`

---

## 3. Heading F9P (front unit) — Configuration
This unit **receives RTCM3** on **UART1** from Base Link and outputs **UBX‑NAV‑RELPOSNED** on **USB** to the Pi.

### A. Factory reset
`CFG → Revert to default configuration → Send`

### B. Limit USB output to **only UBX‑NAV‑RELPOSNED** *(start at 1 Hz)*
`MSG (Messages):`
- **Disable all NMEA F0‑xx** on **USB** (USB rate = 0).
**Messages to Disable (set rate to 0 on USB):**
- **F0-00 NMEA-GGA** (Global positioning) → Rate: 0
- **F0-01 NMEA-GLL** (Geographic position) → Rate: 0
- **F0-02 NMEA-GSA** (DOP and active satellites) → Rate: 0
- **F0-03 NMEA-GSV** (Satellites in view) → Rate: 0
- **F0-04 NMEA-RMC** (Recommended minimum) → Rate: 0
- **F0-05 NMEA-VTG** (Track made good) → Rate: 0

- Enable **01‑3C UBX‑NAV‑RELPOSNED** on **USB** → **rate = 1**.

**Verify** (Packet Console): There should be no messages present.
**Verify** (Binary Console): only `UBX‑NAV‑RELPOSNED` at **1 Hz**.

### C. Configure UART1 to **receive RTCM3**
`PRT (Ports)` → **Target: UART1**
- **Protocol in**: **RTCM3**
- **Protocol out**: **None**
- **Baud**: **115200** → **Send**

### D. Time/Mode
`TMODE3 (Time Mode 3)` → **Mode = Disabled** (this is a rover in a moving‑baseline pair).

### E. Raise rates to **10 Hz** *(after verification)*
- `MSG`: set **UBX‑NAV‑RELPOSNED** on **USB** → **rate 10**.
- `RATE`: **100 ms** period, **10 Hz** navigation → **Send**.

### F. Disable unused interfaces (optional)
Disable **I2C**, **UART2**, **SPI** (Protocol in/out = None).

### G. Save
`CFG → Save current configuration to **BBR + Flash + I2C + SPI-FLASH**.`

---

## 4) Minimal Port/Message Settings (summary)

### Base Link F9P
| Port  | Protocol In            | Protocol Out | Messages Enabled (rates)                                                |
|------|-------------------------|--------------|--------------------------------------------------------------------------|
| USB  | **RTCM3** (+UBX opt.)   | **NMEA**     | **GGA**, **RMC** only (start **1 Hz**, then **10 Hz**)                   |
| UART1| None                    | **RTCM3**    | **4072.0**, **4072.1**, **1077**, **1087**(if GLONASS), **1097**(if GAL), **1127**(if BDS), **1230**(if GLONASS) — start **1 Hz**, then **10 Hz** |
| I2C  | None                    | None         | —                                                                        |
| UART2| None                    | None         | —                                                                        |
| SPI  | None                    | None         | —                                                                        |

### Heading F9P
| Port  | Protocol In | Protocol Out | Messages Enabled (rates)                 |
|------|-------------|--------------|------------------------------------------|
| USB  | None        | **UBX**      | **UBX‑NAV‑RELPOSNED** (start **1 Hz**, then **10 Hz**) |
| UART1| **RTCM3**   | None         | (no explicit messages; RTCM is a protocol stream)      |
| I2C  | None        | None         | —                                        |
| UART2| None        | None         | —                                        |
| SPI  | None        | None         | —                                        |

---

## 5) What to look for (Heading F9P → RELPOSNED)
- Flags should show: `fixOK=True`, `diff=True`, `relValid=True`, `carrier=fixed`, `moving=True`, `headValid=True`.
- **Length** ≈ your measured antenna separation.
- **Heading** stable (σHead < ~0.5° for ~1 m baseline in good sky).

**ENU yaw** (CCW from East) = `yaw_enu = wrap(90° − heading_deg)`.

---

## 6) Troubleshooting quick list
- **No RELPOSNED / empty** → Heading UART1 not receiving RTCM (check `PRT(UART1)` and `MON‑COMMS`).
- **headValid=False** or **carrier≠fixed** → RTCM stream incomplete: ensure **4072.0/4072.1** are enabled on Base UART1; keep MSM7 and (if used) **1230**.
- **Packet Console too chatty** → Re‑disable all but GGA/RMC (Base USB) and RELPOSNED (Heading USB).
- **Overflow** → Lower MSM rates to 5 Hz, keep 4072.x at 10 Hz; verify UART1 baud (115200).

---

## 7) u-blox references:
u‑blox ZED‑F9P Integration Manual:     https://www.u-blox.com/docs/UBX-18010802
u‑blox Moving Base Applications Note:  https://www.u-blox.com/docs/UBX-19009093