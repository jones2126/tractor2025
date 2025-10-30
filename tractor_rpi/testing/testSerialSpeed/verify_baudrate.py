#!/usr/bin/env python3
"""
verify_baudrate.py
Quick check to see what baud rate the Teensy is actually using
"""

import serial
import time

SERIAL_PORT = '/dev/teensy'
TEST_BAUD_RATES = [921600, 115200, 460800, 2000000]

def test_baud(baud_rate):
    """Try to read clean data at a specific baud rate"""
    print(f"\nTrying {baud_rate} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, baud_rate, timeout=2)
        time.sleep(0.5)
        
        good_lines = 0
        bad_lines = 0
        
        # Read for 2 seconds
        start = time.time()
        while time.time() - start < 2:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # Check if it's a valid message format
                        if line.startswith('HEARTBEAT,') and line.count(',') == 2:
                            parts = line.split(',')
                            if len(parts) == 3 and parts[1].isdigit() and parts[2].isdigit():
                                good_lines += 1
                                if good_lines <= 3:
                                    print(f"  ✓ {line}")
                        elif line.startswith('STATS,'):
                            good_lines += 1
                            if good_lines <= 3:
                                print(f"  ✓ {line}")
                        elif line.startswith('BAUD_RATE:'):
                            print(f"  ✓ {line}")
                            teensy_baud = line.split(':')[1].strip()
                            print(f"\n  Teensy reports: {teensy_baud} baud")
                            print(f"  Testing at: {baud_rate} baud")
                            if teensy_baud == str(baud_rate):
                                print(f"  ✓✓ MATCH! This is the correct baud rate!")
                            else:
                                print(f"  ✗✗ MISMATCH! Teensy is at {teensy_baud}, not {baud_rate}")
                            return True
                        else:
                            bad_lines += 1
                            if bad_lines <= 3:
                                print(f"  ✗ Garbled: {line[:50]}")
                except:
                    bad_lines += 1
        
        ser.close()
        
        if good_lines > 5:
            print(f"  ✓✓ SUCCESS! Got {good_lines} clean messages")
            return True
        elif good_lines > 0:
            print(f"  ⚠ Partial: {good_lines} good, {bad_lines} bad")
            return False
        else:
            print(f"  ✗ FAIL: Only garbled data ({bad_lines} bad lines)")
            return False
            
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False

def main():
    print("="*70)
    print("TEENSY BAUD RATE VERIFICATION")
    print("="*70)
    print(f"Port: {SERIAL_PORT}")
    print("\nTrying different baud rates to find which one works...\n")
    
    working_baud = None
    
    for baud in TEST_BAUD_RATES:
        if test_baud(baud):
            working_baud = baud
            break
        time.sleep(0.5)
    
    print("\n" + "="*70)
    if working_baud:
        print(f"RESULT: Teensy is running at {working_baud} baud")
        print("="*70)
        print(f"\nTo fix the Python test script:")
        print(f"  Edit test_single_baudrate.py line 11:")
        print(f"  BAUD_RATE = {working_baud}")
    else:
        print("RESULT: Could not determine baud rate")
        print("="*70)
        print("\nTroubleshooting:")
        print("  1. Check Teensy is connected: ls -l /dev/teensy")
        print("  2. Power cycle the Teensy")
        print("  3. Re-upload firmware: pio run -t upload")
        print("  4. Check for compile errors")

if __name__ == "__main__":
    main()
