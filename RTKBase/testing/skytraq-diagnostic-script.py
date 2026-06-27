#!/usr/bin/env python3
"""
GPS Message Counter & Diagnostic
Counts NMEA and RTCM message types and prints summary every 15s.
"""

import serial
import time
import sys
from collections import defaultdict
from datetime import datetime

PORT = "/dev/ttyUSB1"   # Change to your port
BAUD = 115200
TIMEOUT = 1.0
SUMMARY_INTERVAL = 15   # seconds

def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    
    nmea_count = defaultdict(int)
    rtcm_count = defaultdict(int)
    start_time = time.time()
    last_summary = start_time
    buffer = b''

    print("Listening... (Ctrl+C to stop)\n")

    try:
        while True:
            data = ser.read(2048)
            if data:
                buffer += data
                
                # NMEA (ASCII lines)
                try:
                    text = buffer.decode('ascii', errors='ignore')
                    for line in text.splitlines():
                        if line.startswith('$'):
                            msg_type = line.split(',')[0][1:6] if ',' in line else line[1:6]
                            nmea_count[msg_type] += 1
                except:
                    pass
                
                # RTCM (binary)
                pos = 0
                while pos < len(buffer) - 6:
                    if buffer[pos] == 0xD3:
                        length = ((buffer[pos+1] & 0x03) << 8) | buffer[pos+2]
                        if pos + length + 6 <= len(buffer):
                            msg_type = (buffer[pos+3] << 4) | (buffer[pos+4] >> 4)
                            rtcm_count[msg_type] += 1
                            pos += length + 6
                            continue
                    pos += 1
                
                buffer = buffer[-2048:]  # Keep recent bytes

            # Periodic summary
            now = time.time()
            if now - last_summary >= SUMMARY_INTERVAL:
                elapsed = now - last_summary
                print(f"\n=== Summary @ {datetime.now().strftime('%H:%M:%S')} ===")
                print(f"Duration: {elapsed:.1f}s")
                
                print("\nNMEA Messages:")
                for t, c in sorted(nmea_count.items(), key=lambda x: -x[1]):
                    print(f"  {t}: {c} ({c/elapsed:.2f}/s)")
                
                print("\nRTCM Messages:")
                for t, c in sorted(rtcm_count.items(), key=lambda x: -x[1]):
                    print(f"  Type {t}: {c} ({c/elapsed:.2f}/s)")
                
                nmea_count.clear()
                rtcm_count.clear()
                last_summary = now

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()