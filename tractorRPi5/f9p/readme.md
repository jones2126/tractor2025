# Moving Baseline RTK Configuration Guide

## Objectives and Goals

This guide helps configure two u-blox ZED-F9P GPS receivers for **Moving Baseline RTK** to provide both RTK Fix position (i.e. 2-5cm position accuracy) and heading for tractor navigation (e.g. Pure Pursuit path following).  The base link position will be updated at 10Hz.  The base link F9P will pull RTCM correction using a TCP socket via a USB connection and will output UBX messages to the heading F9P using UART connections between the two devices.  The distance between the two units will be ~1 meter. One goal is for heading accuracy to be +/- 0.5°.


---

## Hardware Connections

```
RTK Base Station → Pi 5 ←USB─ Base Link F9P ─UART1─ Heading F9P ─USB→ Pi 5
(RTCM corrections)    │     (rear, navigation)   (raw data)  (front, heading)
                      │
                      └─ Pure Pursuit Navigation System
```

### **Physical Wiring:**
- **Base Link F9P**: USB to Pi 5 (NMEA navigation data + RTCM input)
- **Heading F9P**: USB to Pi 5 (UBX heading messages)

---

### Wire UART1 Connection
Connect the two F9P units with 3-wire cable plus IOREF to 3V3 connections:
```
Base F9P            Heading F9P
UART1 TX       →    UART1 RX
UART1 RX       →    UART1 TX  
GND            →    GND
3V3 → IOREF   and   3V3 → IOREF 

```

**IOREF Configuration:**
- Connect each F9P's **3V3 pin to its own IOREF pin**
- **Do not cross-connect** IOREF between units



## Configuration Procedure

---

## Part 1: Base Link F9P Configuration (Rear Unit)

### Step 1: Factory Reset and Connection
1. Connect Base Link F9P to PC via USB
2. Open u-center
3. **Receiver → Connection** → Select F9P COM port
4. Verify NMEA messages are flowing
5. **Factory Reset**:
   - **new**
   - **View → Configuration View**   
   - **CFG → Choose 'Revert to default configuration'**
The items listed (0-BBR, 1-FLASH, 2-I2C EEPROM, 3-SPI FLASH) are just showing you what storage devices are available You don't need to select/highlight any of them   
   - **press SEND in lower left**
   - **old**
   - **UBX → CFG → CFG**    
   - **Clear to**: BBR, Flash, EEPROM
   - **Devices**: All devices
   - **Send**
   - **Receiver → Action → Reset Receiver** (warm start)

### Step 2: Set Navigation Rate to 10Hz
1. **View → Configuration View**
2. **RATES → CFG → RATE**
**Time Source:**: 1 - GPS time
**Measurement Period:**: 100 ms
**Measurement Frequency:**: 10 Hz
**Navigation Rate:**: 1
**Navigation Frequency:**: 10 Hz

### Step 3: Limit NMEA output to reduce load

**Current Status**: Factory default F9P outputs all NMEA messages (GGA, GLL, GSA, GSV, RMC, VTG, TXT). I only want GGA and RMC messages to start.

#### **Access Message Configuration**
1. **View** → **Configuration View** (or press CTRL + F9)
2. In the left panel, scroll down and click **"MSG (Messages)"**
3. This opens the message configuration interface
4. Scroll down to the messages that begin with 'F0-xx NMEA'

#### **Disable Unnecessary NMEA Messages**

For each unwanted NMEA message, set the USB output rate to **0** or uncheck the boxes.  Both work to disable.:

**Messages to Disable:**
- **NMEA-GLL** (Geographic position - Lat/Lon): Redundant with GGA
- **NMEA-GSA** (DOP and active satellites): Not needed for Pure Pursuit
- **NMEA-GSV** (Satellites in view): Not needed for Pure Pursuit
- **NMEA-RMC** (Recommended minimum): Redundant with GGA
- **NMEA-VTG** (Track made good/speed): Not needed for Pure Pursuit
- **NMEA-TXT** (Text transmission): Not needed for navigation

**Message to Keep Enabled:**
- **NMEA-GGA** (Global positioning fix): **Rate: 1** (keep enabled)
- **NMEA-RMC** (Recommended minimum): Redundant with GGA
  - Can be adjusted later.  I want just enough data for the Pure Pursuit algorithm, to know RTK Fix status and to compare heading and speed with other data sources.

#### **Configuration Steps**

For each message to disable:

1. **Select the NMEA message** (e.g., NMEA-GLL)
2. **Find the USB output rate setting**
3. **Set rate to 0** (disables output on USB)
4. **Click "Send"** to apply the change
5. **Repeat for all unwanted messages**

#### **Verify Configuration**

After disabling unwanted messages, your packet console should show:
- **Only NMEA GGA and RMC messages** flowing at 10Hz.  Copy and paste the screen.  Then count the number of lines in a one second interval.


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


### Step 4: Configure UBX Output on UART1
Navigate to **UBX → CFG → MSG** for each message:

**Enable Raw Measurement Data on UART1:**
```
UBX-RXM-RAWX (Raw measurements):
- Message Class: 0x02, Message ID: 0x15
- I/O Target: Check UART1, set rate to 1
- Send

UBX-RXM-SFRBX (Subframe buffer):
- Message Class: 0x02, Message ID: 0x13  
- I/O Target: Check UART1, set rate to 1
- Send

UBX-NAV-PVT (Position/Velocity/Time):
- Message Class: 0x01, Message ID: 0x07
- I/O Target: Check UART1, set rate to 1
- Send
```

### Step 5: Configure UART1 Port Settings
1. **UBX → CFG → PRT (Ports)**
2. **Port**: 1 (UART1)
3. **Protocol Out**: Check **UBX only** (uncheck NMEA)
4. **Protocol In**: Check **UBX** (for potential feedback)
5. **Baudrate**: 115200
6. **Data bits**: 8, **Stop bits**: 1, **Parity**: None
7. **Send**

### Step 6: Set Dynamic Model and RTK Settings
1. **UBX → CFG → NAV5**
2. **Dynamic Model**: Automotive (suitable for ground vehicles)
3. **Fix Mode**: 3D only
4. **Send**

### Step 7: Configure Survey-In Mode (Recommended)
1. **UBX → CFG → TMODE3**
2. **Mode**: Survey-In (1)
3. **Survey-in Min Duration**: 300 seconds
4. **Survey-in Position Accuracy**: 2.0 meters
5. **Send**

*Note: Survey-in establishes a more accurate local reference position for the base F9P*

### Step 8: Save Base Link Configuration
1. **UBX → CFG → CFG**
2. **Save to**: BBR (battery backed RAM) and Flash
3. **Devices**: All devices
4. **Send**

---

## Part 2: Heading F9P Configuration (Front Unit)

### Step 1: Connect and Reset Heading F9P
1. Connect Heading F9P to PC (different USB port)
2. Open **second instance** of u-center or switch connection
3. Select Heading F9P COM port
4. **Factory Reset** (same procedure as Step 1 above)

### Step 2: Set Navigation Rate to 10Hz
Same as Base Link Step 2:
1. **UBX → CFG → RATE**
2. **Measurement Rate**: 100 ms (10Hz)
3. **Send**

### Step 3: Configure UART1 for Input from Base
1. **UBX → CFG → PRT**
2. **Port**: 1 (UART1)
3. **Protocol In**: Check **UBX only** (receive from Base F9P)
4. **Protocol Out**: None (no output needed on UART1)
5. **Baudrate**: 115200
6. **Send**

### Step 4: Enable Relative Positioning Output on USB
1. **UBX → CFG → MSG**
2. **Message Class**: 0x01, **Message ID**: 0x3C (NAV-RELPOSNED)
3. **I/O Target**: Check **USB**, set rate to **1**
4. **Send**

*This is the key message containing heading and relative position data!*

### Step 5: Disable Unnecessary Messages on USB
Disable standard NMEA messages to reduce USB traffic:
- Set all NMEA messages (GGA, GLL, GSA, GSV, RMC, VTG) to rate **0** on USB
- Keep only UBX-NAV-RELPOSNED enabled

### Step 6: Configure as Moving Base Rover
1. **UBX → CFG → TMODE3**
2. **Mode**: Rover (0)
3. **Send**

### Step 7: Verify High-Precision Settings
Check **UBX → CFG → NAVHPG**:
- **dgnssMode**: Should be RTK (3)
- If not set, configure and **Send**

### Step 8: Save Heading F9P Configuration
1. **UBX → CFG → CFG**
2. **Save to**: BBR and Flash
3. **Devices**: All devices
4. **Send**

---

## Part 3: Physical Connection and Testing

### Step 1: Wire UART1 Connection
Connect the two F9P units with 4-wire cable:
```
Base F9P Pin    →    Heading F9P Pin
UART1 TX        →    UART1 RX
UART1 RX        →    UART1 TX  
GND             →    GND
(VCC not needed - both powered via USB)
```

### Step 2: Power Up and Verify
1. Connect both F9Ps to your Pi 5 via USB
2. Power up the system
3. Send RTCM corrections to Base F9P via USB
4. Monitor both units in u-center

### Step 3: Verify Operation
**Base Link F9P should show:**
- NMEA GGA messages on USB at 10Hz
- RTK Fixed status in GGA messages
- UART1 activity (raw data transmission)

**Heading F9P should show:**
- UBX-NAV-RELPOSNED messages on USB at 10Hz
- Relative position and heading data
- RTK status in RELPOSNED flags

---

## Expected Message Formats

### Base Link F9P Output (NMEA GGA):
```
$GPGGA,123456.00,4028.12345,N,08019.67890,W,4,12,1.2,123.4,M,-32.1,M,1.0,0*7F
                                              ↑
                                         RTK Fixed (4)
```

### Heading F9P Output (UBX-NAV-RELPOSNED):
```python
# Key fields in UBX-NAV-RELPOSNED:
relPosN: int32        # Relative North position (cm)
relPosE: int32        # Relative East position (cm)  
relPosD: int32        # Relative Down position (cm)
relPosHeading: uint32 # Heading (0.01° units)
flags: uint32         # Solution status flags
accHeading: uint32    # Heading accuracy (0.01°)
```

---

## Troubleshooting

### No RTK Fix on Base Link:
- Verify RTCM corrections are being received
- Check antenna placement and sky view
- Monitor survey-in progress in u-center

### No Heading Data from Front Unit:
- Verify UART1 wiring between F9P units
- Check UART1 configuration on both units
- Ensure raw data messages (RAWX, SFRBX) are enabled on Base Link UART1

### Low Update Rate:
- Verify CFG-RATE is set to 100ms on both units
- Check message rates are set to 1 (not 0)
- Monitor USB traffic to ensure no bottlenecks

### Poor Heading Accuracy:
- Verify 1-meter baseline distance
- Check antenna ground planes
- Ensure both units have clear sky view
- Verify RTK Fixed status on both units

---

## Integration with Navigation System

### NMEA Parser for Base Link (Python):
```python
import pynmea2

def parse_gga(nmea_sentence):
    msg = pynmea2.parse(nmea_sentence)
    if msg.sentence_type == 'GGA':
        return {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'fix_quality': msg.gps_qual,  # 4 = RTK Fixed
            'num_sats': msg.num_sats,
            'altitude': msg.altitude
        }
```

### UBX Parser for Heading (Python):
```python
def parse_relposned(ubx_data):
    # Parse UBX-NAV-RELPOSNED (0x01 0x3C)
    # Returns heading in degrees, relative position in meters
    # Implementation depends on UBX parsing library
    pass
```

---

## Performance Specifications

### Achieved Performance:
- **Position Update Rate**: 10Hz
- **Heading Update Rate**: 10Hz  
- **Position Accuracy**: 2-5cm (RTK Fixed)
- **Heading Accuracy**: 0.1-0.5° (1m baseline)
- **RTK Convergence Time**: 15-60 seconds
- **CPU Load**: Minimal Pi 5 processing (direct UART)

### System Requirements:
- **RTK Base Station**: Active with RTCM corrections
- **Antenna Separation**: 1 meter baseline minimum
- **Clear Sky View**: Both antennas need GPS signal reception
- **Stable Mounting**: Antennas must maintain fixed relative positions

---

## File Locations

Configuration backup files should be saved to:
```
/path/to/your/repo/docs/gps_configs/
├── base_link_f9p_config.txt    # u-center config export
├── heading_f9p_config.txt      # u-center config export  
└── moving_baseline_setup.md    # This documentation
```

## References

- [u-blox ZED-F9P Integration Manual](https://www.u-blox.com/docs/UBX-18010802)
- [u-blox Moving Base Applications Note](https://www.u-blox.com/docs/UBX-19009093)
- [UBX Protocol Specification](https://www.u-blox.com/docs/UBX-18010854)

---
 
**Last updated**: August 2025  
**Version**: 1.0
