#!/usr/bin/env python3
"""
minimal_20hz_test.py
Minimal test matching minimal Teensy code
"""

import serial
import time
import numpy as np

SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 921600
TEST_DURATION = 20

print("="*70)
print("MINIMAL 20 Hz TEST")
print("="*70)

print(f"Opening {SERIAL_PORT}...")
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except Exception as e:
    print(f"ERROR: Could not open serial port: {e}")
    print("Make sure /dev/teensy exists and no other program is using it.")
    exit(1)

print("Waiting for Teensy to boot (5 seconds)...")
time.sleep(5)
ser.reset_input_buffer()
ser.reset_output_buffer()

# Wait for READY message
print("Looking for READY message...")
timeout = time.time() + 10
got_ready = False
while time.time() < timeout:
    if ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"  Received: {line}")
            if 'READY' in line:
                got_ready = True
                print("✓ Teensy is ready!\n")
                break
        except:
            pass
    time.sleep(0.1)

if not got_ready:
    print("WARNING: Did not receive READY message.")
    print("Continuing anyway...\n")

# Counters
sent = 0
echoes = 0
heartbeats = 0

# Timing
start = time.time()
last_status = start

# Calculate send schedule
send_schedule = [start + (i * 0.050) for i in range(int(TEST_DURATION * 20) + 5)]
send_idx = 0

print(f"Starting {TEST_DURATION} second test...\n")

while time.time() - start < TEST_DURATION:
    now = time.time()
    
    # Send at scheduled time
    if send_idx < len(send_schedule) and now >= send_schedule[send_idx]:
        try:
            # Minimal message - just "C\n"
            ser.write(b'C\n')
            ser.flush()
            sent += 1
            send_idx += 1
        except Exception as e:
            print(f"Send error: {e}")
    
    # Read responses
    try:
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            
            # test_teensy_minimal.cpp sends "E<number>" and "H<number>"
            if line.startswith('E'):
                echoes += 1
            elif line.startswith('H'):
                heartbeats += 1
    except Exception as e:
        # Ignore occasional read errors
        pass
    
    # Status every 5 seconds
    if now - last_status >= 5:
        print(f"[{int(now-start)}s] Sent: {sent}, Echoes: {echoes}, HB: {heartbeats}")
        last_status = now
    
    time.sleep(0.0001)

# Final read
time.sleep(0.2)
while ser.in_waiting:
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('E'):
            echoes += 1
        elif line.startswith('H'):
            heartbeats += 1
    except:
        pass

ser.close()

# Results
print("\n" + "="*70)
print("RESULTS")
print("="*70)

success_rate = (echoes / sent * 100) if sent > 0 else 0
expected_hb = TEST_DURATION * 20

print(f"Commands sent:      {sent}")
print(f"Echoes received:    {echoes} ({success_rate:.1f}%)")
print(f"Heartbeats:         {heartbeats} (expected ~{expected_hb})")

print("\nPass/Fail:")
passed = True

if success_rate > 99:
    print(f"  ✓ Success rate: {success_rate:.1f}%")
else:
    print(f"  ✗ Success rate: {success_rate:.1f}%")
    passed = False

if heartbeats > expected_hb * 0.95:
    print(f"  ✓ Heartbeats: {heartbeats}")
else:
    print(f"  ✗ Heartbeats: {heartbeats}")
    passed = False

print("\n" + "="*70)
if passed:
    print("✓✓ PASS - Minimal messages work at 20 Hz!")
else:
    print("✗✗ FAIL - Try diagnostic_test.py next")
print("="*70)
