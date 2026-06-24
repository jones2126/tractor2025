#!/usr/bin/env python3
"""
esp32_timesync.py - Minimal boot-time time sync for ESP32
Run via systemd after network-online.target to ensure NTP is set first.
Connects to ESP32, sends SETTIME, confirms sync, and exits.
No download, no CSV handling.
"""

import serial
import time
import sys
from datetime import datetime, timezone

ESP32_PORT = "/dev/esp32"
BAUDRATE = 115200
TIMEOUT = 5
MAX_WAIT_SECONDS = 30  # Max time to wait for ESP32 to be ready


def log(message):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(f"[{timestamp}] {message}", flush=True)


def main():
    log("=== ESP32 Time Sync Started ===")

    # Connect
    try:
        ser = serial.Serial(
            port=ESP32_PORT,
            baudrate=BAUDRATE,
            timeout=TIMEOUT,
            dsrdtr=False,
            rtscts=False
        )
        time.sleep(2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        log(f"Connected to {ESP32_PORT}")
    except serial.SerialException as e:
        log(f"ERROR: Could not connect to {ESP32_PORT}: {e}")
        sys.exit(1)

    try:
        # Wait for ESP32 to be ready
        log(f"Waiting for ESP32 (up to {MAX_WAIT_SECONDS}s)...")
        ready = False
        start_wait = time.time()

        while time.time() - start_wait < MAX_WAIT_SECONDS:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    log(f"ESP32: {line}")
                if any(phrase in line for phrase in [
                    "Setup complete",
                    "Ready for Python",
                    "Data written to",
                    "Temperature reading",
                    "Debug: Time synced",
                    "WARNING: Logging paused"
                ]):
                    ready = True
                    log("ESP32 is ready")
                    break
            time.sleep(0.1)

        if not ready:
            log("ESP32 did not confirm ready state — sending SETTIME anyway")

        ser.reset_input_buffer()
        time.sleep(0.3)

        # Calculate timezone-adjusted epoch
        now_local = datetime.now()
        now_utc = datetime.now(timezone.utc).replace(tzinfo=None)
        offset_seconds = int((now_local - now_utc).total_seconds())
        current_epoch_local = int(time.time()) + offset_seconds

        log(f"Local time: {now_local.strftime('%Y-%m-%d %H:%M:%S')}")
        log(f"Timezone offset: UTC{offset_seconds/3600:+.1f} hours")
        log(f"Sending: SETTIME {current_epoch_local}")

        ser.write(f"SETTIME {current_epoch_local}\n".encode('utf-8'))

        # Wait for confirmation
        start_time = time.time()
        confirmed = False

        while time.time() - start_time < 10:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    log(f"ESP32: {line}")
                if line.startswith("TIME_SYNCED:"):
                    confirmed = True
                    # Read the timestamp confirmation line too
                    time.sleep(0.3)
                    while ser.in_waiting:
                        extra = ser.readline().decode('utf-8', errors='ignore').strip()
                        if extra:
                            log(f"ESP32: {extra}")
                    break
            time.sleep(0.1)

        if confirmed:
            log("Time sync confirmed — ESP32 will now log with wall clock timestamps")
            log("=== ESP32 Time Sync Completed Successfully ===")
            sys.exit(0)
        else:
            log("ERROR: No TIME_SYNCED confirmation received")
            log("=== ESP32 Time Sync Failed ===")
            sys.exit(1)

    finally:
        ser.close()


if __name__ == "__main__":
    main()
