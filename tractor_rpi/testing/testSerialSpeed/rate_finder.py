#!/usr/bin/env python3
"""
rate_finder.py
Tests different rates to find your optimal frequency
Works with test_teensy_minimal.cpp
"""

import serial
import time

SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 921600
TEST_DURATION = 10  # 10 seconds per test

print("="*70)
print("RATE FINDER - Finding your optimal frequency")
print("="*70)

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
time.sleep(3)
ser.reset_input_buffer()

# Test different rates
rates = [5, 10, 15, 20, 25]  # Hz
results = {}

for rate_hz in rates:
    interval = 1.0 / rate_hz
    
    print(f"\nTesting {rate_hz} Hz ({interval*1000:.1f}ms interval)...")
    
    sent = 0
    echoes = 0
    heartbeats = 0
    
    # Clear any old data
    time.sleep(0.5)
    ser.reset_input_buffer()
    
    start = time.time()
    next_send = start + interval
    
    while time.time() - start < TEST_DURATION:
        now = time.time()
        
        # Send at scheduled time
        if now >= next_send:
            try:
                ser.write(b'C\n')
                ser.flush()
                sent += 1
                next_send += interval
            except:
                pass
        
        # Read responses
        while ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('E'):
                    echoes += 1
                elif line.startswith('H'):
                    heartbeats += 1
            except:
                pass
        
        time.sleep(0.0001)
    
    # Final read
    time.sleep(0.3)
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('E'):
                echoes += 1
            elif line.startswith('H'):
                heartbeats += 1
        except:
            pass
    
    success_rate = (echoes / sent * 100) if sent > 0 else 0
    expected_hb = TEST_DURATION * rate_hz
    hb_rate = (heartbeats / expected_hb * 100) if expected_hb > 0 else 0
    
    results[rate_hz] = {
        'sent': sent,
        'echoes': echoes,
        'success': success_rate,
        'heartbeats': heartbeats,
        'hb_expected': expected_hb,
        'hb_rate': hb_rate
    }
    
    status = "✓" if success_rate > 99 else "✗"
    print(f"  {status} Sent: {sent}, Echoes: {echoes} ({success_rate:.1f}%)")
    print(f"     Heartbeats: {heartbeats}/{int(expected_hb)} ({hb_rate:.1f}%)")

ser.close()

# Summary
print("\n" + "="*70)
print("RESULTS SUMMARY")
print("="*70)
print(f"{'Rate':<8} {'Success':<12} {'Heartbeats':<15} {'Status'}")
print("-"*70)

best_rate = None
for rate_hz in rates:
    r = results[rate_hz]
    if r['success'] > 99 and r['hb_rate'] > 95:
        status = "✓ PASS"
        if best_rate is None:
            best_rate = rate_hz
    elif r['success'] > 95:
        status = "⚠ CLOSE"
    else:
        status = "✗ FAIL"
    
    print(f"{rate_hz:>3} Hz   {r['success']:>5.1f}%       {r['hb_rate']:>5.1f}%           {status}")

print("="*70)

if best_rate:
    print(f"\n✓ RECOMMENDATION: Use {best_rate} Hz or slower for >99% reliability")
elif results[rates[-1]]['success'] > 95:
    print(f"\n⚠ RECOMMENDATION: Maximum reliable rate is ~{rates[-1]} Hz at 97-98%")
    print("  Consider:")
    print("  - Accept 97-98% with retry logic")
    print("  - Use slower rate for guaranteed reliability")
else:
    print("\n✗ WARNING: Even 5 Hz has issues - possible hardware problem")
    print("  Check:")
    print("  - USB cable quality")
    print("  - USB port (try different one)")
    print("  - Other processes using serial port")

print()
