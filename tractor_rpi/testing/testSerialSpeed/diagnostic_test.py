#!/usr/bin/env python3
"""
diagnostic_test.py
Slower, more careful test to diagnose the issue
"""

import serial
import time

SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 921600
TEST_DURATION = 10  # Shorter test

print("="*70)
print("DIAGNOSTIC TEST - Slower, more careful")
print("="*70)

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
time.sleep(2)
ser.reset_input_buffer()

print("\n1. Testing basic communication (5 commands, 1 per second):")
for i in range(5):
    ser.write(f"TEST{i}\n".encode())
    ser.flush()
    time.sleep(0.1)  # Wait 100ms for response
    
    responses = []
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        responses.append(line)
    
    print(f"  Sent: TEST{i}, Received: {responses}")
    time.sleep(0.9)  # Total 1 second

print("\n2. Testing 10 Hz (2 commands per second for 5 seconds):")
sent = 0
received = 0
missed = []

for i in range(10):
    ser.write(f"T{i}\n".encode())
    ser.flush()
    sent += 1
    time.sleep(0.05)  # Give it time to process
    
    got_echo = False
    start_wait = time.time()
    while time.time() - start_wait < 0.05:  # Wait up to 50ms
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('E,'):
                got_echo = True
                received += 1
                break
    
    if not got_echo:
        missed.append(i)
    
    time.sleep(0.45)  # Total 500ms per command

print(f"  Sent: {sent}, Received: {received}, Missed: {missed}")
print(f"  Success rate: {received/sent*100:.1f}%")

print("\n3. Testing 20 Hz (20 commands per second for 5 seconds):")
sent = 0
received = 0
hb_count = 0

start = time.time()
next_send = start + 0.05

while time.time() - start < 5:
    current = time.time()
    
    # Send at 20 Hz
    if current >= next_send:
        ser.write(f"C{sent}\n".encode())
        ser.flush()
        sent += 1
        next_send += 0.05
    
    # Read responses
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('E,'):
            received += 1
        elif line.startswith('H,'):
            hb_count += 1
    
    time.sleep(0.001)

# Final read
time.sleep(0.2)
while ser.in_waiting:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line.startswith('E,'):
        received += 1
    elif line.startswith('H,'):
        hb_count += 1

print(f"  Sent: {sent}, Received: {received}, Heartbeats: {hb_count}")
print(f"  Success rate: {received/sent*100:.1f}%")
print(f"  Expected ~100 heartbeats, got: {hb_count}")

ser.close()

print("\n" + "="*70)
print("ANALYSIS:")
print("="*70)
if received/sent > 0.99:
    print("✓ 20 Hz works at >99% success rate")
elif received/sent > 0.95:
    print("⚠ Close but not quite - 95-99% success")
    print("  Possible issues:")
    print("  - Python timing jitter")
    print("  - Serial buffer overflow")
    print("  - Teensy processing time")
else:
    print("✗ Significant message loss")
    print("  Likely causes:")
    print("  - Baud rate mismatch")
    print("  - Hardware flow control issues")
    print("  - Buffer size too small")
print("="*70)
