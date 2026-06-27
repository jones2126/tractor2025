#!/usr/bin/env python3
"""
F9P Configured Base Position Reader - Final Working Version
"""

import serial
import time
import io
from pyubx2 import UBXMessage, UBXReader

print("F9P Configured Base Position Reader")
print("=" * 55)

with serial.Serial("/dev/f9p", 115200, timeout=2) as ser:
    print("Sending poll for CFG_TMODE keys...\n")

    keys = ["CFG_TMODE_MODE", "CFG_TMODE_LAT", "CFG_TMODE_LON", "CFG_TMODE_HEIGHT"]
    poll = UBXMessage.config_poll(0, 0, keys)

    ser.reset_input_buffer()
    ser.write(poll.serialize())
    ser.flush()
    time.sleep(2.0)

    raw = ser.read(ser.in_waiting or 2048)
    print(f"Received {len(raw)} bytes\n")

    try:
        # Use BytesIO so UBXReader can iterate
        stream = io.BytesIO(raw)
        ubr = UBXReader(stream)

        found = False
        for _, parsed in ubr:
            if parsed is None:
                continue

            if hasattr(parsed, 'identity') and parsed.identity == "CFG-VALGET":
                print("✅ CFG-VALGET Response Found:")

                mode = getattr(parsed, "CFG_TMODE_MODE", None)
                lat  = getattr(parsed, "CFG_TMODE_LAT", None)
                lon  = getattr(parsed, "CFG_TMODE_LON", None)
                height = getattr(parsed, "CFG_TMODE_HEIGHT", None)

                print(f"   TMODE Mode : {mode} → {'Fixed' if mode == 2 else 'Survey-In' if mode == 1 else 'Disabled'}")

                if lat is not None and lon is not None:
                    print(f"   Latitude   : {lat / 1e7:.8f}°")
                    print(f"   Longitude  : {lon / 1e7:.8f}°")
                    print(f"   Height     : {height / 1000.0:.3f} m")
                    print("\n✅ This is the position the F9P is using for RTCM 1005 corrections.")
                    found = True
                else:
                    print("   Position keys not present in this message.")

        if not found:
            print("No CFG-VALGET message found in the reply.")

    except Exception as e:
        print(f"Parse error: {e}")
        print(f"Raw length: {len(raw)} bytes")

print("\nDone.")