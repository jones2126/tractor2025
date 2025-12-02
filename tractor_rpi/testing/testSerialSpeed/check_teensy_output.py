#!/usr/bin/env python3
"""
check_teensy_output.py
Just shows what the Teensy is sending (for debugging)
"""

import serial
import time

SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 921600

print("Opening serial port...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

print("Waiting for Teensy to boot (5 seconds)...")
time.sleep(5)

print("\nReading Teensy output for 10 seconds:")
print("-" * 60)

start = time.time()
while time.time() - start < 10:
    if ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"Received: '{line}'")
        except Exception as e:
            print(f"Error: {e}")
    time.sleep(0.01)

print("-" * 60)
print("\nIf you see 'H<number>' and 'READY', the Teensy is working!")
print("If you see nothing, there's a problem with the Teensy code or connection.")

ser.close()
