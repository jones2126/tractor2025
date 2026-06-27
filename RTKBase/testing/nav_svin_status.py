#!/usr/bin/env python3
"""
Check current Survey-In status
"""

import serial
import time
import io
from pyubx2 import UBXMessage, UBXReader

with serial.Serial("/dev/f9p", 115200, timeout=2) as ser:
    poll = UBXMessage("NAV", "NAV-SVIN", msgmode=0)
    ser.reset_input_buffer()
    ser.write(poll.serialize())
    ser.flush()
    time.sleep(1.5)

    raw = ser.read(ser.in_waiting or 2048)
    print(f"Received {len(raw)} bytes\n")

    stream = io.BytesIO(raw)
    ubr = UBXReader(stream)

    found = False
    for _, msg in ubr:
        if hasattr(msg, 'identity') and msg.identity == "NAV-SVIN":
            print("NAV-SVIN Status:")
            print(f"  Active       : {msg.active}")
            print(f"  Observations : {msg.obs}")
            print(f"  Mean Acc     : {msg.meanAcc / 10000.0:.2f} m")
            print(f"  Duration     : {msg.dur} seconds")
            print(f"  Valid        : {msg.valid}")
            found = True

    if not found:
        print("No NAV-SVIN message found in reply.")