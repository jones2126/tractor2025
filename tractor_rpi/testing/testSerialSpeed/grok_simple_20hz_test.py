#!/usr/bin/env python3
import serial, time, numpy as np, subprocess, sys

PORT = '/dev/teensy'
BAUD = 921600
DUR = 20

# === STEP 1: WAIT FOR TEENSY_READY USING PySerial ===
print("Waiting for TEENSY_READY...")
ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=1)
try:
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if "TEENSY_READY" in line:
            print("TEENSY_READY received")
            break
finally:
    ser.close()

# === STEP 2: SET RAW MODE ===
print("Setting raw mode...")
subprocess.run(['stty', '-F', PORT, 'raw', '-echo', 'min', '0', 'time', '0'], check=True)
time.sleep(0.3)

# === STEP 3: REOPEN IN RAW MODE ===
ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=0, write_timeout=0)
ser.flushInput()
ser.flushOutput()
time.sleep(0.1)

# === STEP 4: RUN TEST ===
sent = echo = hb = 0
lat = []
t0 = time.time()
next_send = t0
line_buf = bytearray()

print("\n=== 20 Hz TEST RUNNING ===")
print(f"   Duration: {DUR}s\n")

while time.time() - t0 < DUR:
    now = time.time()

    if now >= next_send:
        ser.write(f"C{sent}\n".encode())
        sent += 1
        lat.append(now)
        next_send += 0.05

    while True:
        b = ser.read(1)
        if not b: break
        line_buf.extend(b)
        if b == b'\n':
            line = line_buf.decode(errors='ignore').strip()
            line_buf.clear()
            if line.startswith('E'):
                echo += 1
                if echo <= len(lat):
                    lat[echo-1] = (now - lat[echo-1]) * 1000
            elif line.startswith('H'):
                hb += 1

    time.sleep(0.0002)

# final drain
time.sleep(0.3)
while True:
    b = ser.read(1)
    if not b: break
    line_buf.extend(b)
    if b == b'\n':
        line = line_buf.decode(errors='ignore').strip()
        line_buf.clear()
        if line.startswith('E'): echo += 1
        if line.startswith('H'): hb += 1

ser.close()

# === RESULTS ===
print("\n=== RESULTS ===")
success = echo / sent * 100 if sent else 0
avg_lat = np.mean(lat) if lat else 0

print(f"Sent:       {sent}")
print(f"Echoed:     {echo} ({success:.2f}%)")
print(f"Heartbeats: {hb}")
print(f"Latency:    avg {avg_lat:.2f} ms")

print("\nPASS" if success >= 99.0 and hb >= DUR*20*0.95 else "\nFAIL")