#!/usr/bin/env python3
"""
simple_20hz_test_v3.py
Improved timing and message handling
"""

import serial
import time
import numpy as np

SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 921600
TEST_DURATION = 20

print("="*70)
print(f"SIMPLE 20 Hz TEST v3 @ {BAUD_RATE} baud")
print("="*70)

# Open serial port
print(f"Opening {SERIAL_PORT}...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
time.sleep(1)

# Flush any startup garbage
ser.reset_input_buffer()
ser.reset_output_buffer()
print("Connected! Starting test...\n")

# Counters
cmd_sent = 0
echo_recv = 0
hb_recv = 0
latencies = []

# Timing
start = time.time()
last_status = start

# Use a more precise timer
try:
    # Try to use monotonic clock for better precision
    get_time = time.monotonic
except AttributeError:
    get_time = time.time

# Pre-calculate all send times
send_times = [start + (i * 0.050) for i in range(int(TEST_DURATION * 20) + 10)]
send_index = 0

while get_time() - start < TEST_DURATION:
    current = get_time()
    
    # Send command at precisely calculated times
    if send_index < len(send_times) and current >= send_times[send_index]:
        cmd_time = current
        try:
            # Very simple message
            msg = f"C{cmd_sent}\n"
            ser.write(msg.encode())
            ser.flush()  # Ensure it's sent
            cmd_sent += 1
            send_index += 1
            
            # Small delay to let Teensy process (1ms)
            time.sleep(0.001)
        except Exception as e:
            print(f"Send error: {e}")
    
    # Read all available responses
    try:
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if line.startswith('E,'):  # Echo message
                echo_recv += 1
                # Calculate latency if we can
                if cmd_sent > 0:
                    latencies.append((current - cmd_time) * 1000)
            elif line.startswith('H,'):  # Heartbeat message
                hb_recv += 1
            elif line and not line.startswith('STATS'):
                pass  # Ignore other messages
    except Exception as e:
        print(f"Read error: {e}")
    
    # Status every 5 seconds
    if current - last_status >= 5:
        elapsed = current - start
        print(f"[{elapsed:.0f}s] Sent: {cmd_sent}, Echoed: {echo_recv}, Heartbeats: {hb_recv}")
        last_status = current
    
    # Small sleep to prevent CPU spinning
    time.sleep(0.0001)

# Final read
time.sleep(0.2)
while ser.in_waiting:
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('E,'):
            echo_recv += 1
        elif line.startswith('H,'):
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
