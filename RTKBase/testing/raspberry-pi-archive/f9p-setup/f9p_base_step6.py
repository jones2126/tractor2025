#!/usr/bin/env python3
"""
f9p_save_nmea_hp.py
Configures NMEA High Precision Mode (CFG-NMEA) and saves it persistently
to all supported memories (BBR + Flash + EEPROM) — just like u-center.
"""

import time, serial
from pyubx2 import UBXMessage, UBXReader, SET

PORT = "/dev/f9p"   # change to COMx on Windows if needed
BAUD = 115200

def send_and_wait_ack(ser, msg, label):
    ser.reset_input_buffer()
    ser.write(msg.serialize())
    ser.flush()
    ubr = UBXReader(ser, protfilter=2)
    end = time.time() + 1.5
    while time.time() < end:
        if ser.in_waiting:
            raw, parsed = ubr.read()
            if parsed:
                if parsed.identity == "ACK-ACK":
                    print(f"  ✓ ACK for {label}")
                    return True
                elif parsed.identity == "ACK-NAK":
                    print(f"  ✗ NAK for {label}")
                    return False
        time.sleep(0.05)
    print(f"  ✗ No ACK for {label} (timeout)")
    return False

def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print(f"Connected to {PORT} @ {BAUD}")

        # 1. Configure NMEA High Precision mode
        print("Configuring NMEA High Precision Mode...")
        payload = bytes([
            0x00,       # filter
            0x4B,       # nmeaVersion (4.1) + flags
            0x00,       # numSV
            0x0A,       # flags (bit3=High Precision, bit1=compat)
            0x00,0x00,0x00,0x00,  # gnssToFilter
            0x00,       # svNumbering
            0x01,       # mainTalkerId
            0x00,       # gsvTalkerId
            0x00,       # version
            0x00,0x00,0x00,0x00,0x00,0x00  # reserved
        ])
        cfg_nmea = UBXMessage("CFG","CFG-NMEA", msgmode=SET, payload=payload)
        if not send_and_wait_ack(ser, cfg_nmea, "CFG-NMEA"):
            print("Stopping — failed to set CFG-NMEA")
            return

        # 2. Save configuration persistently (match u-center: deviceMask=0x17)
        print("Saving configuration to all memories...")
        save_payload = (
            (0x00000000).to_bytes(4,"little") +   # clearMask
            (0x0000FFFF).to_bytes(4,"little") +   # saveMask (all)
            (0x00000000).to_bytes(4,"little") +   # loadMask
            (0x17).to_bytes(1,"little")           # deviceMask (BBR+Flash+EEPROM)
        )
        cfg_save = UBXMessage("CFG","CFG-CFG", msgmode=SET, payload=save_payload)
        if not send_and_wait_ack(ser, cfg_save, "CFG-CFG (save)"):
            print("Failed to save configuration.")
            return

        print("  ✓ Configuration saved. Waiting 1.5s for flash commit...")
        time.sleep(1.5)

        # 3. Poll CFG-NMEA to confirm setting
        print("Polling CFG-NMEA to verify...")
        poll = UBXMessage("CFG","CFG-NMEA", msgmode=0)
        ser.reset_input_buffer()
        ser.write(poll.serialize())
        ser.flush()
        ubr = UBXReader(ser, protfilter=2)
        time.sleep(0.3)
        if ser.in_waiting:
            raw, parsed = ubr.read()
            print(f"CFG-NMEA after save: {parsed}")
        else:
            print("  ✗ No response to CFG-NMEA poll")

        print("Power cycle your F9P and rerun this script with only the poll step to confirm persistence.")

if __name__ == "__main__":
    main()
