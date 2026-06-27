#!/usr/bin/env python3
"""
Diagnostic - Show Survey Status + Fixed Position
"""

import serial, time, io
from pyubx2 import UBXMessage, UBXReader

with serial.Serial("/dev/f9p", 115200, timeout=2) as ser:
    print("=== 1. NAV-SVIN Status ===")
    poll = UBXMessage("NAV", "NAV-SVIN", msgmode=0)
    ser.write(poll.serialize()); ser.flush()
    time.sleep(1)
    raw = ser.read(2048)
    for _, msg in UBXReader(io.BytesIO(raw)):
        if hasattr(msg, 'identity') and msg.identity == "NAV-SVIN":
            print(f"Active : {msg.active} | obs={msg.obs} | Acc={msg.meanAcc/10000.0:.2f}m")

    print("\n=== 2. CFG_TMODE Configuration ===")
    keys = ["CFG_TMODE_MODE", "CFG_TMODE_LAT", "CFG_TMODE_LON", "CFG_TMODE_HEIGHT"]
    poll = UBXMessage.config_poll(0, 0, keys)
    ser.reset_input_buffer()
    ser.write(poll.serialize()); ser.flush()
    time.sleep(2)
    raw = ser.read(2048)
    print(f"Raw CFG reply: {len(raw)} bytes")

    found = False
    for _, msg in UBXReader(io.BytesIO(raw)):
        if hasattr(msg, 'identity') and msg.identity == "CFG-VALGET":
            mode = getattr(msg, "CFG_TMODE_MODE", None)
            lat = getattr(msg, "CFG_TMODE_LAT", None)
            lon = getattr(msg, "CFG_TMODE_LON", None)
            h = getattr(msg, "CFG_TMODE_HEIGHT", None)
            
            print(f"Mode : {mode} ({'Fixed' if mode==2 else 'Survey-In' if mode==1 else 'Other'})")
            if lat is not None:
                print(f"Lat  : {lat/1e7:.8f}°")
                print(f"Lon  : {lon/1e7:.8f}°")
                print(f"Height : {h/1000.0:.3f} m")
                found = True
    if not found:
        print("No CFG-VALGET found in reply (this is the issue we need to fix).")