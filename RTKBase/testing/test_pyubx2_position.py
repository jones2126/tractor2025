#!/usr/bin/env python3
"""
pyubx2 Test - GGA vs RTCM 1005
"""

import serial
import time
import os
import json
from pyubx2 import UBXReader

print("pyubx2 Mixed Stream Test - GGA vs RTCM 1005")
print("=" * 65)

# Show current GGA from JSON
json_path = "../current_position.json"   # adjust path if needed
if os.path.exists(json_path):
    try:
        with open(json_path) as f:
            gga_data = json.load(f)
        print(f"📍 GGA from JSON : {gga_data['lat']:.8f}, {gga_data['lon']:.8f}  (alt={gga_data.get('alt')}m)")
    except:
        pass

with serial.Serial("/dev/f9p", 115200, timeout=1) as ser:
    ubr = UBXReader(ser, protfilter=7, validate=1)   # NMEA + UBX + RTCM

    print("\nListening for messages...\n")

    try:
        for raw, parsed in ubr:
            if parsed is None:
                continue

            # === NMEA GGA ===
            if hasattr(parsed, 'msgID') and parsed.msgID == "GGA":
                print(f"📍 GGA Live Position : {parsed.lat:.8f}, {parsed.lon:.8f}  alt={parsed.alt}m")

            # === RTCM 1005 ===
            elif str(type(parsed)).find("RTCMMessage") != -1:
                # pyubx2 uses .identity for message type
                if hasattr(parsed, 'identity') and parsed.identity == "1005":
                    print("\n✅ RTCM 1005 FOUND - Base Reference Position (used for corrections)")
                    
                    # Extract fields safely
                    ref_id = getattr(parsed, 'refStationID', 'N/A')
                    lat = getattr(parsed, 'lat', None)
                    lon = getattr(parsed, 'lon', None)
                    height = getattr(parsed, 'height', None)
                    
                    print(f"   Reference Station ID : {ref_id}")
                    if lat is not None and lon is not None:
                        print(f"   Latitude             : {lat:.8f}°")
                        print(f"   Longitude            : {lon:.8f}°")
                        print(f"   Height               : {height} m")
                    else:
                        print("   (Position fields not directly exposed - raw payload available)")
                    
                    print("-" * 60)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"\nError: {e}")