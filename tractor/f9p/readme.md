# Moving Baseline RTK Configuration Guide

## Objectives and Goals

This guide helps configure two u-blox ZED-F9P GPS receivers for **Moving Baseline RTK** to provide both RTK Fix position (i.e. 2-5cm position accuracy) and heading for tractor navigation (e.g. Pure Pursuit path following).  The base link position will be updated at 10Hz.  The "base" link" F9P will pull RTCM correction using a TCP socket via a USB connection and will output UBX messages to the "heading" F9P via the UART1 connections between the two devices.  The distance between the two GPS antennaes will be ~1 meter. One goal is for heading accuracy to be +/- 0.5°, hopefully!


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
3. **Receiver → Baudrate → 115,200** - Set baudrate
4. **Receiver → Connection → COMxx** - Make connection
5. **Factory Reset**:
   - **View → Configuration View**   
   - **CFG → Choose 'Revert to default configuration'**
The items listed (0-BBR, 1-FLASH, 2-I2C EEPROM, 3-SPI FLASH) are just showing you what storage devices are available You don't need to select/highlight any of them   
   - **press SEND in lower left**

### Step 2: Limit NMEA output to reduce load

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

**Message to Keep / Enable, Only USB port:**
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
**View → Packet Console**
After disabling unwanted messages, your packet console should show:
- **Only NMEA GGA and RMC messages** flowing at 1Hz.  Now we need to change the output rate to be 10Hz.


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
### Step 3: Set Navigation Rate to 10Hz
1. **View → Configuration View**
2. **RATES → CFG → RATE**
**Time Source:**: 1 - GPS time
**Measurement Period:**: 100 ms (reduced from 1000 ms)
**Measurement Frequency:**: 10 Hz (should change automatically)
**Navigation Rate:**: 1
**Navigation Frequency:**: 10 Hz (should change automatically)

### Step 4: Configure UBX output on UART1 for moving baseline RTK

#### **Access Message Configuration**

1. **Open Configuration View**: Press **CTRL + F9**
2. **Navigate to Messages**: Click **"MSG (Messages)"** in the left panel
3. **This opens the UBX message configuration interface** Scroll down until you start to see 02-15 RXM-RAWX.  For me, they were after the '29-12 NAV2-VELNED' message.

#### **Configure Raw Measurement Messages**

You will configure three critical UBX messages to output on UART1 at 10Hz:

##### **UBX-RXM-RAWX (Raw Measurements)**

**What it contains**: Raw carrier phase and pseudorange measurements from all visible satellites

1. **Message dropdown**: Select **"02-15 RXM-RAWX"**
2. **I2C checkbox**: **Uncheck** ❌ (reduces Pi CPU load)
3. **UART1 checkbox**: **Check it** ✅
4. **UART1 rate**: Set to **1** (Will trigger 10Hz output, because the overall message rate is 10 Hz)
5. **UART2 checkbox**: **Uncheck** ❌ (reduces Pi CPU load)
6. **USB checkbox**: **Uncheck** ❌ (reduces Pi CPU load)
7. **SPI checkbox**: **Uncheck** ❌ (reduces Pi CPU load)
8. **Click "Send"**

##### **UBX-RXM-SFRBX (Subframe Buffer)**

**What it contains**: Navigation message subframes (ephemeris, almanac, ionospheric data)

1. **Message dropdown**: Select **"02-13 RXM-SFRBX"**
2. **I2C checkbox**: **Uncheck** ❌ (reduces Pi CPU load)
3. **UART1 checkbox**: **Check it** ✅
4. **UART1 rate**: Set to **1** (Will trigger 10Hz output, because the overall message rate is 10 Hz)
5. **UART2 checkbox**: **Uncheck** ❌ (reduces Pi CPU load)
6. **USB checkbox**: **Uncheck** ❌ (reduces Pi CPU load)
7. **SPI checkbox**: **Uncheck** ❌ (reduces Pi CPU load)
8. **Click "Send"**

##### **UBX-NAV-PVT (Position/Velocity/Time)**

**What it contains**: Navigation solution, time, and status information

1. **Message dropdown**: Select **"01-07 NAV-PVT"**
2. **I2C checkbox**: **Uncheck** ❌ (reduces Pi CPU load)
3. **UART1 checkbox**: **Check it** ✅
4. **UART1 rate**: Set to **1** (Will trigger 10Hz output, because the overall message rate is 10 Hz)
5. **UART2 checkbox**: **Uncheck** ❌ (reduces Pi CPU load)
6. **USB checkbox**: **Uncheck** ❌ (reduces Pi CPU load)
7. **SPI checkbox**: **Uncheck** ❌ (reduces Pi CPU load)
8. **Click "Send"**

#### **Configuration Summary**

After completing these steps, your Base Link F9P will output:

**On UART1 (to Heading F9P):**
- ✅ **RXM-RAWX**: Raw satellite measurements (10Hz)
- ✅ **RXM-SFRBX**: Navigation subframes (10Hz)
- ✅ **NAV-PVT**: Position/velocity/time (10Hz)

**On USB (to Raspberry Pi):**
- ✅ **NMEA GGA**: Position and RTK status (10Hz)
- ✅ **NMEA RMC**: Speed and course (10Hz)



### Step 5: Configure Port Settings (UART1 for UBX, others to reduce CPU load)
1. **Open Configuration View**: View -> Configuration View or CTRL + F9 
2. **Navigate to Ports**: Click **"PRT (Ports)"** in the left panel
2. **Target**: Select **"1 - UART1"**
3. **Protocol In**: Select **"0 - UBX"** (for any potential feedback)
4. **Protocol Out**: Select **"0 - UBX"** (for moving baseline messages)
5. **Baudrate**: Select **115200** (hopefully good for 10 Hz)
6. **Data bits**: 8, **Stop bits**: 1, **Parity**: None, **Bit Order**: LSB First
7. **Send**
---

#### **CPU Load Optimization - Just guessing at this point this helps**

**Objective**: Reduce F9P CPU load by disabling unused interfaces (I2C, UART2, SPI) 

##### **Interface Optimization Settings**

**Disable I2C Interface:**
1. **Target**: Select **"0 - I2C"**
2. **Protocol In**: Change to **"none"**
3. **Protocol Out**: Change to **"none"**
4. **Click "Send"**

**Disable UART2 Interface:**
1. **Target**: Select **"2 - UART2"**
2. **Protocol In**: Change to **"none"**
3. **Protocol Out**: Change to **"none"**
4. **Click "Send"**

**Disable SPI Interface:**
1. **Target dropdown**: Select **"4 - SPI"**
2. **Protocol In**: Change to **"none"**
3. **Protocol Out**: Change to **"none"**
4. **Click "Send"**

##### **Final Active Port Summary**
After optimization, only these interfaces remain active:

| Port | Protocol In | Protocol Out | Purpose |
|------|-------------|--------------|---------|
| **USB** | UBX+NMEA+RTCM3 | NMEA | Pi navigation + RTCM input |
| **UART1** | UBX | UBX | Raw data to Heading F9P |
| **I2C** | NONE | NONE | Disabled to reduce load |
| **UART2** | NONE | NONE | Disabled to reduce load |
| **SPI** | NONE | NONE | Disabled to reduce load |

---

### Step 6: Configure Dynamic Model and RTK Settings

**Objective**: Set the Base Link F9P's Navigation engine settings (Message UBX-CFG-NAV5) for '11 = mower' hoping to help RTK behavior for tractor navigation.  What I was able to find on this model suggested, "Intended for small, low-speed autonomous ground vehicles like lawn mowers, golf carts, and slow UGVs. Optimized for low speed (<10 m/s), frequent stops and starts, tight turns, and higher vibration. Assumes higher yaw rates than automotive."


#### **Configuration Steps**

##### **Access Navigation Configuration**
**CTRL + F9** (Configuration View); **Click "NAV5 (Navigation 5)"** in the left panel; 

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

---

### Step 7: Save Configuration Settings for Base Link GPS
1. **View -> Configuration View -> CFG → Select Save Current Configuration**
2. **Devices**: All devices
3. **Send**

---

## Part 2: Heading F9P Configuration (Front Unit)

### Step 1: Connect and Reset Heading F9P
### Step 2: Set Navigation Rate to 10Hz
### Step 3: Configure UART1 for Input from Base
### Step 4: Enable Relative Positioning Output on USB
### Step 5: Disable Unnecessary Messages on USB
### Step 6: Configure as Moving Base Rover
### Step 7: Verify High-Precision Settings - Check **UBX → CFG → NAVHPG**:- **dgnssMode**: Should be RTK
### Step 8: Save Heading F9P Configuration

# Heading F9P Configuration Guide - Moving Baseline RTK

**Objective**: Configure the Heading F9P (front GPS unit) to receive raw measurement data from the Base Link F9P via UART1 in order to calculate heading using moving baseline RTK. Output UBX-NAV-RELPOSNED messages with heading data via USB to be fed into navigation algorithm.

---

## Step 1: Factory Reset Heading F9P

### **Connect and Reset**
1. **Connect Heading F9P** to PC via USB (different port than Base Link)
2. **CTRL + F9** (Configuration View)
3. **Click "CFG (Configuration)"** in left panel
4. **Select "Revert to default configuration"** radio button
5. **Click "Send"**
6. **Wait for reset completion** (~10 seconds)

### **Verification**
After reset, verify - go to View -> Packet Console:
- **All NMEA messages** returning (GGA, GLL, GSA, GSV, RMC, VTG)
- **1Hz update rate** (default)
- **Clean factory state** established

---

## Step 2: Set Navigation Rate to 10Hz

### **Configure Update Rate**
1. **CTRL + F9** (Configuration View)
2. **Click "RATE (Rates)"** in left panel
3. **Set parameters**:
   - **Time Source**: 1 - GPS time
   - **Measurement Period**: 100 ms
   - **Measurement Frequency**: 10.00 Hz
   - **Navigation Rate**: 1
   - **Navigation Frequency**: 10.00 Hz
4. **Click "Send"**

---

## Step 3: Remove Unnecessary NMEA Messages

### **Disable All NMEA Messages on USB**
Since the Heading F9P will output UBX-NAV-RELPOSNED only, disable all NMEA:

1. **CTRL + F9** (Configuration View)
2. **Click "MSG (Messages)"** in left panel
3. **For each NMEA message, set USB rate to 0**:

**Messages to Disable (set rate to 0 on USB):**
- **F0-00 NMEA-GGA** (Global positioning) → Rate: 0
- **F0-01 NMEA-GLL** (Geographic position) → Rate: 0
- **F0-02 NMEA-GSA** (DOP and active satellites) → Rate: 0
- **F0-03 NMEA-GSV** (Satellites in view) → Rate: 0
- **F0-04 NMEA-RMC** (Recommended minimum) → Rate: 0
- **F0-05 NMEA-VTG** (Track made good) → Rate: 0

**For each message:**
1. **Select message** from dropdown (e.g., F0 00 GGA)
2. **USB checkbox**: **Uncheck** or set rate to **0**
3. **Click "Send"**
4. **Repeat for all NMEA messages**

### **Verification**
After disabling NMEA messages:
- **Packet console should show ZERO traffic**
- **Only system messages** (like GNTXT) may remain
- **Clean data stream** prepared for UBX output

---

## Step 4: Configure UART1 to Receive UBX Data from Base Link

### **Set UART1 as Input Port**
1. **CTRL + F9** (Configuration View)
2. **Click "PRT (Ports)"** in left panel
3. **Select "1 - UART1"** from Target dropdown

### **UART1 Configuration**
Configure UART1 to receive raw data from Base Link F9P:

- **Target**: 1 - UART1
- **Protocol In**: **0 - UBX** (receive UBX from Base Link)
- **Protocol Out**: **7 - NONE** (no output needed on UART1)
- **Baudrate**: **115200** (match Base Link output)
- **Databits**: 8
- **Stopbits**: 1
- **Parity**: None
- **Bit Order**: LSB First

4. **Click "Send"**

### **Purpose**
This configures UART1 to receive:
- **UBX-RXM-RAWX**: Raw measurement data from Base Link
- **UBX-RXM-SFRBX**: Subframe buffer data from Base Link  
- **UBX-NAV-PVT**: Position/velocity/time from Base Link

---

## Step 5: Enable UBX-NAV-RELPOSNED Output on USB

### **Configure Relative Position Output**
1. **CTRL + F9** (Configuration View)
2. **Click "MSG (Messages)"** in left panel
3. **Select "01 3C NAV-RELPOSNED"** from message dropdown

**If NAV-RELPOSNED not visible in dropdown:**
- Look for **"01 3C"** entries
- Try scrolling through entire dropdown list
- May be listed as **"NAV-RELPOSNED"** or **"RELPOSNED"**

### **NAV-RELPOSNED Configuration**
- **Message**: 01 3C NAV-RELPOSNED
- **I2C**: Unchecked (rate 0)
- **UART1**: Unchecked (rate 0) 
- **UART2**: Unchecked (rate 0)
- **USB**: **Checked** (rate **1**) ← **Key setting**
- **SPI**: Unchecked (rate 0)

4. **Click "Send"**

### **NAV-RELPOSNED Message Content**
This message provides the heading and relative position data:
```
Key fields in UBX-NAV-RELPOSNED:
- relPosN: Relative North position (cm)
- relPosE: Relative East position (cm)  
- relPosD: Relative Down position (cm)
- relPosHeading: Heading (0.01° units)
- flags: Solution status (RTK Fixed/Float)
- accHeading: Heading accuracy (0.01°)
```

---

## Step 6: Configure Time Mode

### **Configure Time Mode**
1. **CTRL + F9** (Configuration View)
2. **Click "TMODE3 (Time Mode 3)"** in left panel
3. **Set configuration**:
   - **Mode**: **0 - Disabled** (Rover mode for moving baseline)
   - Leave other settings at default

4. **Click "Send"**

### **Purpose**
This configures the Heading F9P as a moving baseline rover that:
- **Processes raw data** from Base Link F9P
- **Calculates relative position** to Base Link
- **Outputs heading information** in real-time

---

## Step 7: Disable Unused Interfaces (CPU Optimization)

### **Disable I2C Interface**
1. **Select "0 - I2C"** from Target dropdown
2. **Protocol In**: **7 - NONE**
3. **Protocol Out**: **7 - NONE**
4. **Click "Send"**

### **Disable UART2 Interface**
1. **Select "2 - UART2"** from Target dropdown
2. **Protocol In**: **7 - NONE**
3. **Protocol Out**: **7 - NONE**
4. **Click "Send"**

### **Disable SPI Interface**
1. **Select "4 - SPI"** from Target dropdown
2. **Protocol In**: **7 - NONE**
3. **Protocol Out**: **7 - NONE**
4. **Click "Send"**

### **Final Port Configuration Summary**
| Port | Protocol In | Protocol Out | Purpose |
|------|-------------|--------------|---------|
| **USB** | UBX | UBX | NAV-RELPOSNED output to Pi |
| **UART1** | UBX | NONE | Raw data input from Base Link |
| **I2C** | NONE | NONE | Disabled (CPU optimization) |
| **UART2** | NONE | NONE | Disabled (CPU optimization) |
| **SPI** | NONE | NONE | Disabled (CPU optimization) |

---

## Step 8: Save Configuration

### **Save to Non-Volatile Memory**
1. **CTRL + F9** (Configuration View)
2. **Click "CFG (Configuration)"** in left panel
3. **Select "Save current configuration"** radio button
4. **Ensure all device types are selected**:
   - BBR (battery backed RAM)
   - Flash
   - EEPROM
5. **Click "Send"**

### **Verification**
- **Configuration saved** message should appear
- **Settings persist** after power cycle
- **Ready for physical connection** to Base Link F9P

---


## Next Steps

1. **Physical UART1 connection** between Base Link and Heading F9P
2. **Power up both units** and verify data flow
3. **Connect RTCM corrections** to Base Link F9P
4. **Monitor NAV-RELPOSNED** messages for heading data
5. **Integration with navigation system** using UBX message parsing

---


## References

- [u-blox ZED-F9P Integration Manual](https://www.u-blox.com/docs/UBX-18010802)
- [u-blox Moving Base Applications Note](https://www.u-blox.com/docs/UBX-19009093)
- [UBX Protocol Specification](https://www.u-blox.com/docs/UBX-18010854)

---
 
**Last updated**: August 2025  

