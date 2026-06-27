#!/usr/bin/env python3
import serial
import time
import struct

PORT = "/dev/ttyUSB1"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

def send_cmd(hex_str, name="Command"):
    cmd = bytes.fromhex(hex_str)
    ser.write(cmd)
    ser.flush()
    time.sleep(1.5)  # Longer delay
    resp = ser.read(1024)
    print(f"{name} sent. Response: {resp.hex()[:120] if resp else 'None'}")
    return resp

print("=== SkyTraq PX1172RDP Base Station Setup ===")

# 1. Set NMEA output to very low rate (1 every 10 seconds)
send_cmd("a0 a1 00 03 0a 02 0a 00 0c 0d 0a", "Slow NMEA (10s)")

# 2. Disable most NMEA temporarily
send_cmd("a0 a1 00 02 0a 01 0b 0d 0a", "Disable NMEA")

# 3. Set RTK Base Mode
send_cmd("a0 a1 00 02 6a 07 6d 0d 0a", "RTK Base Mode")

# 4. Set Static Position
lat = 40.207149
lon = -80.077305
alt = 300.0
payload = bytearray([0x01, 0x01])
payload.extend(struct.pack('<d', lat))
payload.extend(struct.pack('<d', lon))
payload.extend(struct.pack('<f', alt))
payload.extend([0x00, 0x00, 0x00, 0x00, 0x01])
cmd = bytearray([0xa0, 0xa1, 0x00, len(payload) + 2, 0x6a, 0x06])
cmd.extend(payload)
checksum = 0
for b in cmd[4:]:
    checksum ^= b
cmd.extend([checksum, 0x0d, 0x0a])
send_cmd(cmd.hex(), "Set Static Position")

# 5. Enable RTCM
send_cmd("a0 a1 00 14 69 05 03 01 02 00 01 01 01 01 00 00 01 00 01 00 01 00 01 00 00 0d 0a", "Enable RTCM MSM4 + 1005")

print("\nSetup complete. Run diagnostic script.")
ser.close()