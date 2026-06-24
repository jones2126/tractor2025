#!/usr/bin/env python3
"""
ESP32 Data Downloader for Bridgeville RTK Base Station
Handles status, download, and download_delete commands

Usage:
    python3 esp32_downloader.py status
    python3 esp32_downloader.py download [optional_filename]
    python3 esp32_downloader.py download_delete [optional_filename]
    python3 esp32_downloader.py          # interactive mode
"""

import serial
import time
import sys
from datetime import datetime, timezone
import os

ESP32_PORT = "/dev/esp32"
BAUDRATE = 115200
TIMEOUT = 5


def log(message):
    """Print message with timestamp"""
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(f"[{timestamp}] {message}")


def connect_esp32():
    """Connect to ESP32, avoiding reset via DTR/RTS"""
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
        log(f"Connected to ESP32 on {ESP32_PORT}")
        return ser
    except serial.SerialException as e:
        log(f"ERROR: Could not connect to {ESP32_PORT}: {e}")
        log("Is another program (like 'pio monitor') using the port?")
        return None


def sync_time(ser):
    """Synchronize ESP32 time with Pi current time"""
    log("Waiting for ESP32 to be ready...")
    esp32_alive = False
    start_wait = time.time()

    while time.time() - start_wait < 8:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                log(f"Boot: {line}")
                esp32_alive = True
            if any(phrase in line for phrase in [
                "Setup complete! Starting data logging",
                "Data written to",
                "Reading Renogy data",
                "Temperature reading",
                "Time synced=Yes"
            ]):
                log("ESP32 ready for commands")
                time.sleep(0.5)
                break
        time.sleep(0.1)

    if not esp32_alive:
        log("WARNING: No response from ESP32, attempting sync anyway...")

    ser.reset_input_buffer()

    now_local = datetime.now()
    now_utc = datetime.now(timezone.utc).replace(tzinfo=None)
    offset_seconds = int((now_local - now_utc).total_seconds())
    current_epoch_local = int(time.time()) + offset_seconds

    log(f"Local time: {now_local.strftime('%Y-%m-%d %H:%M:%S')}")
    log(f"Timezone: UTC{offset_seconds/3600:+.1f} hours")

    ser.write(f"SETTIME {current_epoch_local}\n".encode('utf-8'))
    time.sleep(0.5)

    start_time = time.time()
    while time.time() - start_time < 5:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("TIME_SYNCED:"):
                log(f"Time synced successfully: {line}")
                time.sleep(0.2)
                while ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("Current timestamp:"):
                        log(f"ESP32 {line}")
                    elif line:
                        log(f"RX: {line}")
                return True
            elif line:
                log(f"RX: {line}")
        time.sleep(0.1)

    log("WARNING: Time sync confirmation not received")
    return False


def get_status(ser):
    """Send STATUS command and print ESP32 response"""
    log("Requesting ESP32 status...")
    ser.reset_input_buffer()
    ser.write(b"STATUS\n")
    time.sleep(0.5)

    start_time = time.time()
    timeout = 10
    got_response = False

    while time.time() - start_time < timeout:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                log(f"STATUS: {line}")
                got_response = True
            if "COMMAND_COMPLETE" in line or "STATUS_END" in line:
                break
        elif got_response:
            # Give it a moment after last line before declaring done
            time.sleep(0.5)
            if not ser.in_waiting:
                break
        time.sleep(0.1)

    if not got_response:
        log("WARNING: No status response received from ESP32")
    return got_response


def download_csv_data(ser, output_file, delete_after=False):
    """Download CSV data from ESP32, optionally deleting after"""
    download_start = time.time()

    command = "DOWNLOAD_DELETE" if delete_after else "DOWNLOAD"
    log(f"Sending: {command}")
    ser.write(f"{command}\n".encode('utf-8'))
    time.sleep(0.5)

    log(f"Downloading to: {output_file}")
    log(f"Delete after download: {delete_after}")

    csv_data = []
    in_data = False
    line_count = 0
    start_time = time.time()
    timeout = 30

    while time.time() - start_time < timeout:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            if not in_data or line.startswith(('FILE_', 'DOWNLOAD_', 'COMMAND_', 'CSV file')):
                log(f"RX: {line}")

            if line.startswith("DOWNLOAD_DELETE_START") or line.startswith("DOWNLOAD_START"):
                in_data = True
                log("Started receiving data from ESP32...")
                continue
            elif line.startswith("DOWNLOAD_DELETE_END") or line.startswith("DOWNLOAD_END"):
                log(f"Download complete: {line_count} lines received")
                in_data = False
                break
            elif line.startswith("FILE_NAME:"):
                log(f"File: {line.split(':', 1)[1]}")
                continue
            elif line.startswith("FILE_SIZE:"):
                log(f"Size: {line.split(':', 1)[1]} bytes")
                continue
            elif line.startswith("FILE_DELETED:"):
                log(f"ESP32 deleted: {line.split(':', 1)[1]}")
                continue
            elif line.startswith("NEW_FILE_CREATED:"):
                log(f"ESP32 created new file: {line.split(':', 1)[1]}")
                continue
            elif line.startswith("COMMAND_COMPLETE"):
                log("ESP32 command completed")
                break
            elif line.startswith("CSV file initialized"):
                continue
            elif in_data:
                csv_data.append(line)
                line_count += 1
                if line_count % 100 == 0:
                    log(f"Progress: {line_count} lines received...")

        time.sleep(0.01)

    if not csv_data:
        log("WARNING: No CSV data received!")
        return False

    os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)
    with open(output_file, 'w') as f:
        f.write('\n'.join(csv_data))
        f.write('\n')

    elapsed = time.time() - download_start
    file_size_bytes = os.path.getsize(output_file)
    log(f"Saved {line_count} lines to {output_file}")
    log(f"File size: {file_size_bytes} bytes")
    log(f"Download time: {elapsed:.1f} seconds")

    # Verify timestamp format of first data row (skip header)
    data_rows = [r for r in csv_data if not r.startswith("Timestamp")]
    if data_rows:
        first_ts = data_rows[0].split(',')[0]
        if first_ts.startswith(('2024', '2025', '2026')):
            log(f"✓ Timestamp format verified (wall clock): {first_ts}")
        elif first_ts.isdigit() and len(first_ts) > 9:
            log(f"✓ Timestamp format verified (Unix epoch): {first_ts}")
        else:
            log(f"⚠ Warning: Timestamp may be boot-relative: {first_ts}")

    return True


def interactive_mode(ser):
    """Simple interactive terminal for direct ESP32 control"""
    log("Entering interactive mode. Commands: status, download, download_delete, clear, help, quit")
    while True:
        try:
            cmd = input("esp32> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == 'quit' or cmd == 'exit':
            break
        elif cmd == 'status':
            get_status(ser)
        elif cmd.startswith('download_delete'):
            parts = cmd.split()
            filename = parts[1] if len(parts) > 1 else generate_filename()
            download_csv_data(ser, filename, delete_after=True)
        elif cmd.startswith('download'):
            parts = cmd.split()
            filename = parts[1] if len(parts) > 1 else generate_filename()
            download_csv_data(ser, filename, delete_after=False)
        elif cmd == 'clear':
            ser.write(b"CLEAR\n")
            time.sleep(1)
            while ser.in_waiting:
                log(f"RX: {ser.readline().decode('utf-8', errors='ignore').strip()}")
        elif cmd == 'help':
            ser.write(b"HELP\n")
            time.sleep(1)
            while ser.in_waiting:
                log(f"RX: {ser.readline().decode('utf-8', errors='ignore').strip()}")
        else:
            log(f"Unknown command: {cmd}")
            log("Commands: status, download, download_delete, clear, help, quit")


def generate_filename():
    """Generate a timestamped filename"""
    dt_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    return f"/home/al/esp32_data/esp32_data_{dt_str}.csv"


def main():
    script_start = time.time()
    args = sys.argv[1:]  # All arguments after script name
    command = args[0].lower() if args else None
    custom_filename = args[1] if len(args) > 1 else None

    log("=== ESP32 Downloader Started ===")

    # Validate command
    valid_commands = ['status', 'download', 'download_delete', None]
    if command not in valid_commands:
        log(f"ERROR: Unknown command '{command}'")
        log("Usage: esp32_downloader.py [status|download|download_delete] [optional_filename]")
        sys.exit(1)

    ser = connect_esp32()
    if not ser:
        sys.exit(1)

    try:
        # Always sync time first (fast, and ensures good timestamps)
        sync_time(ser)
        time.sleep(0.5)

        if command == 'status':
            success = get_status(ser)

        elif command == 'download':
            output_file = custom_filename or generate_filename()
            success = download_csv_data(ser, output_file, delete_after=False)
            if success:
                log("SUCCESS: Download completed")

        elif command == 'download_delete':
            output_file = custom_filename or generate_filename()
            success = download_csv_data(ser, output_file, delete_after=True)
            if success:
                log("SUCCESS: Download and delete completed")

        else:
            # No command = interactive mode
            interactive_mode(ser)
            success = True

        sys.exit(0 if success else 1)

    except Exception as e:
        log(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        ser.close()
        log("Connection closed")
        log(f"Total time: {time.time() - script_start:.1f} seconds")
        log("=== ESP32 Downloader Finished ===")


if __name__ == "__main__":
    main()
