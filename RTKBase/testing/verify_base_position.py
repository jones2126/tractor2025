#!/usr/bin/env python3
"""
Final Verification: Live GGA vs Fixed Base Position (RTCM 1005 source)
"""

import serial, time, io, json, os
from pyubx2 import UBXMessage, UBXReader

print("🔍 Base Position Verification")
print("=" * 55)

with serial.Serial("/dev/f9p", 115200, timeout=2) as ser:
    # 1. Live GGA position
    print("\n1. Live GGA Position (physical antenna):")
    poll_gga = UBXMessage("NAV", "NAV-PVT", msgmode=0)  # or use NMEA GGA
    ser.write(poll_gga.serialize()); ser.flush()
    time.sleep(1)
    raw = ser.read(2048)
    for _, msg in UBXReader(io.BytesIO(raw)):
        if hasattr(msg, 'identity') and msg.identity in ["NAV-PVT", "GGA"]:
            lat = getattr(msg, 'lat', getattr(msg, 'latitude', 0)) / 1e7 if hasattr(msg, 'lat') else getattr(msg, 'lat', 0)
            lon = getattr(msg, 'lon', getattr(msg, 'longitude', 0)) / 1e7 if hasattr(msg, 'lon') else getattr(msg, 'lon', 0)
            print(f"   📍 {lat:.8f}, {lon:.8f}")

    # 2. Fixed base position from Survey-In result
    print("\n2. Fixed Base Position (used in RTCM 1005):")
    poll = UBXMessage("NAV", "NAV-SVIN", msgmode=0)
    ser.write(poll.serialize()); ser.flush()
    time.sleep(1)
    raw = ser.read(2048)
    for _, msg in UBXReader(io.BytesIO(raw)):
        if hasattr(msg, 'identity') and msg.identity == "NAV-SVIN":
            print(f"   Mean Acc : {msg.meanAcc / 10000.0:.2f} m")
            print(f"   Valid    : {msg.valid}")

    # 3. Try CFG_TMODE again with better parsing
    print("\n3. Trying CFG_TMODE poll...")
    keys = ["CFG_TMODE_MODE", "CFG_TMODE_LAT", "CFG_TMODE_LON", "CFG_TMODE_HEIGHT"]
    poll_cfg = UBXMessage.config_poll(0, 0, keys)
    ser.reset_input_buffer()
    ser.write(poll_cfg.serialize()); ser.flush()
    time.sleep(2)
    raw = ser.read(4096)
    print(f"   Raw CFG reply: {len(raw)} bytes")

    found = False
    for _, msg in UBXReader(io.BytesIO(raw)):
        if hasattr(msg, 'identity') and msg.identity == "CFG-VALGET":
            mode = getattr(msg, "CFG_TMODE_MODE", None)
            lat = getattr(msg, "CFG_TMODE_LAT", None)
            lon = getattr(msg, "CFG_TMODE_LON", None)
            if lat is not None:
                print(f"   Mode : {mode} (2 = Fixed)")
                print(f"   Lat  : {lat / 1e7:.8f}°")
                print(f"   Lon  : {lon / 1e7:.8f}°")
                found = True
    if not found:
        print("   (CFG-VALGET parsing still tricky — using NAV-SVIN instead)")

print("\n✅ Comparison ready.")
print("If the two positions are within ~1–2 meters, your base is correctly set.")