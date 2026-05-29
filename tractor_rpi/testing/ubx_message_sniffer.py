#!/usr/bin/env python3
"""
ubx_message_sniffer.py

Listens on a serial port and reports all UBX and NMEA message types seen,
with counts and approximate Hz rates. Useful for confirming what a u-blox
module is actually outputting after a firmware change.

Usage:
    python3 ubx_message_sniffer.py [port] [baud]

Defaults:
    port = /dev/ttyACM0
    baud = 115200

Example:
    python3 ubx_message_sniffer.py /dev/ttyACM0 115200
"""

import serial, struct, sys, time
from collections import defaultdict, deque

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

SYNC1, SYNC2 = 0xB5, 0x62
REPORT_INTERVAL = 2.0   # seconds between rate display
WINDOW = 20             # timestamps to keep per message type for rate calc

# Known UBX class/ID names for display
UBX_NAMES = {
    (0x01, 0x07): "NAV-PVT",
    (0x01, 0x3C): "NAV-RELPOSNED",
    (0x01, 0x3D): "NAV-DAHEADING",
    (0x01, 0x35): "NAV-SAT",
    (0x01, 0x21): "NAV-TIMEUTC",
    (0x01, 0x20): "NAV-TIMEGPS",
    (0x01, 0x22): "NAV-CLOCK",
    (0x01, 0x02): "NAV-POSLLH",
    (0x01, 0x12): "NAV-VELNED",
    (0x02, 0x13): "RXM-SFRBX",
    (0x02, 0x15): "RXM-RAWX",
    (0x05, 0x01): "ACK-ACK",
    (0x05, 0x00): "ACK-NAK",
    (0x0A, 0x04): "MON-VER",
    (0x0A, 0x09): "MON-HW",
}

def ubx_checksum(data: bytes):
    a = b = 0
    for x in data:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b

def hz(timestamps):
    if len(timestamps) < 2:
        return 0.0
    span = timestamps[-1] - timestamps[0]
    if span <= 0:
        return 0.0
    return (len(timestamps) - 1) / span

def main():
    counts = defaultdict(int)
    timestamps = defaultdict(lambda: deque(maxlen=WINDOW))
    last_report = time.time()

    print(f"Listening on {PORT} @ {BAUD} baud — Ctrl+C to stop\n")

    with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
        buf = bytearray()
        while True:
            data = ser.read(256)
            if data:
                buf += data

            now = time.time()
            i = 0
            while i < len(buf):
                # --- UBX frame ---
                if buf[i] == SYNC1 and i + 1 < len(buf) and buf[i+1] == SYNC2:
                    if i + 6 > len(buf):
                        break  # wait for more data
                    cls_, id_ = buf[i+2], buf[i+3]
                    length = struct.unpack_from("<H", buf, i+4)[0]
                    frame_end = i + 6 + length + 2
                    if frame_end > len(buf):
                        break  # wait for more data
                    ck_a, ck_b = ubx_checksum(buf[i+2:i+6+length])
                    if ck_a == buf[i+6+length] and ck_b == buf[i+7+length]:
                        key = f"UBX {UBX_NAMES.get((cls_, id_), f'0x{cls_:02X}/0x{id_:02X}')}"
                        counts[key] += 1
                        timestamps[key].append(now)
                        i = frame_end
                    else:
                        i += 1  # bad checksum, skip byte
                    continue

                # --- NMEA sentence ---
                if buf[i] == ord('$'):
                    end = buf.find(b'\n', i)
                    if end == -1:
                        break  # wait for more data
                    line = buf[i:end+1]
                    try:
                        sentence = line.decode('ascii', errors='ignore').strip()
                        if ',' in sentence:
                            key = f"NMEA {sentence[1:sentence.index(',')]}"
                            counts[key] += 1
                            timestamps[key].append(now)
                    except Exception:
                        pass
                    i = end + 1
                    continue

                i += 1

            buf = buf[i:]

            # --- periodic report ---
            if now - last_report >= REPORT_INTERVAL:
                last_report = now
                print(f"\n{'Message':<30} {'Count':>7}  {'Hz':>6}")
                print("-" * 48)
                for key in sorted(counts):
                    rate = hz(timestamps[key])
                    print(f"{key:<30} {counts[key]:>7}  {rate:>6.2f}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDone.")
        sys.exit(0)
