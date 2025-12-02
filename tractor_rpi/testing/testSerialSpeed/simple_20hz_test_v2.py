#!/usr/bin/env python3
"""
simple_20hz_test.py
No-nonsense 20 Hz bidirectional test - just works!
"""

import serial
import time
import numpy as np

SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 921600
TEST_DURATION = 20

print("="*70)
print(f"SIMPLE 20 Hz TEST @ {BAUD_RATE} baud")
print("="*70)

# Open serial port
print(f"Opening {SERIAL_PORT}...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
time.sleep(1)

# Flush any startup garbage
ser.reset_input_buffer()
print("Connected! Starting test...\n")

# Counters
cmd_sent = 0
echo_recv = 0
hb_recv = 0
latencies = []
last_cmd_time = None

# Run test
start = time.time()
next_send = start + 0.05
last_status = start

while time.time() - start < TEST_DURATION:
    current = time.time()
    
    # Send command at 20 Hz
    if current >= next_send:
        try:
            ser.write(f"CMD,{cmd_sent}\n".encode())  # CHANGED: Shorter message (was ~20 bytes, now ~10 bytes)
            ser.flush()  # NEW: Wait for data to actually be transmitted
            cmd_sent += 1
            last_cmd_time = current
            next_send += 0.05
        except:
            pass
    
    # Read responses
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('ECHO'):
                echo_recv += 1
                if last_cmd_time:
                    latencies.append((current - last_cmd_time) * 1000)
            elif line.startswith('HEARTBEAT'):
                hb_recv += 1
        except:
            pass
    
    # Status every 5 seconds
    if current - last_status >= 5:
        elapsed = current - start
        print(f"[{elapsed:.0f}s] Sent: {cmd_sent}, Echoed: {echo_recv}, Heartbeats: {hb_recv}")
        last_status = current
    
    time.sleep(0.001)

# Final read
time.sleep(0.2)
while ser.in_waiting:
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('ECHO'):
            echo_recv += 1
        elif line.startswith('HEARTBEAT'):
            hb_recv += 1
    except:
        pass

ser.close()

# Results
print("\n" + "="*70)
print("RESULTS")
print("="*70)

success_rate = (echo_recv / cmd_sent * 100) if cmd_sent > 0 else 0
avg_lat = np.mean(latencies) if latencies else 0

print(f"Commands sent:      {cmd_sent}")
print(f"Echoes received:    {echo_recv} ({success_rate:.1f}%)")
print(f"Heartbeats:         {hb_recv}")
print(f"Average latency:    {avg_lat:.2f} ms")

if latencies:
    print(f"Min latency:        {np.min(latencies):.2f} ms")
    print(f"Max latency:        {np.max(latencies):.2f} ms")
    print(f"95th percentile:    {np.percentile(latencies, 95):.2f} ms")

print("\nPass/Fail:")
passed = True

if success_rate > 99:
    print(f"  ✓ Success rate: {success_rate:.1f}% (>99%)")
else:
    print(f"  ✗ Success rate: {success_rate:.1f}% (need >99%)")
    passed = False

if avg_lat < 10:
    print(f"  ✓ Latency: {avg_lat:.2f}ms (<10ms)")
else:
    print(f"  ✗ Latency: {avg_lat:.2f}ms (need <10ms)")
    passed = False

expected_hb = TEST_DURATION * 20
if hb_recv > expected_hb * 0.95:
    print(f"  ✓ Heartbeats: {hb_recv} (expected ~{expected_hb})")
else:
    print(f"  ✗ Heartbeats: {hb_recv} (expected ~{expected_hb})")
    passed = False

print("\n" + "="*70)
if passed:
    print("✓✓ OVERALL: PASS - 20 Hz operation works!")
else:
    print("✗✗ OVERALL: FAIL - Issues detected")
print("="*70)
