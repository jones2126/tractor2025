#!/usr/bin/env python3
import serial
import time
import logging
from pyubx2 import UBXMessage, UBXReader, SET

PORT = "/dev/f9p"
BAUD = 115200

# Map NMEA sentence to UBX class/ID
NMEA_MSGS = {
    "GGA": 0x00,
    "GLL": 0x01,
    "GSA": 0x02,
    "GSV": 0x03,
    "RMC": 0x04,
    "VTG": 0x05,
    "GRS": 0x06,
    "GST": 0x07,
    "TXT": 0x41,
}

# output, but keeping
#     "GGA": 0x00,

def disable_nmea(ser):
    for name, msg_id in NMEA_MSGS.items():
        try:
            # Build CFG-MSG (Class=0xF0 for NMEA, msg_id varies)
            payload = b"\xF0" + bytes([msg_id]) + b"\x01" + b"\x00\x00\x00"  # target USB only
            msg = UBXMessage("CFG", "CFG-MSG", SET, payload=payload)
            ser.write(msg.serialize())
            logging.info(f"Sent disable for NMEA-{name} (0xF0,{msg_id:02X})")
            time.sleep(0.05)
        except Exception as e:
            logging.error(f"Failed to disable {name}: {e}")

def monitor_nmea(ser, duration=5):
    logging.info(f"Monitoring NMEA output for {duration}s...")
    start = time.time()
    lines = []
    while time.time() - start < duration:
        if ser.in_waiting:
            data = ser.readline().decode(errors="ignore").strip()
            if data.startswith("$"):
                tag = data[3:6]  # e.g. $GNGGA -> GGA
                lines.append(tag)
    if lines:
        unique = sorted(set(lines))
        logging.warning(f"NMEA still detected after disable: {', '.join(unique)}")
    else:
        logging.info("No NMEA detected after disable (success)")

def main():
    logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s")
    logging.info(f"Connecting to F9P on {PORT} @ {BAUD}")
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        disable_nmea(ser)
        time.sleep(0.5)
        monitor_nmea(ser)

if __name__ == "__main__":
    main()
