#!/usr/bin/env python3
"""
Automated ESP32 data download for cron jobs
Downloads and optionally deletes CSV data from ESP32
Now includes time synchronization before download
"""

import serial
import time
import sys
from datetime import datetime
import os

ESP32_PORT = "/dev/esp32"  # Use the persistent symlink
BAUDRATE = 115200
TIMEOUT = 5

def log(message):
    """Print message with timestamp"""
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(f"[{timestamp}] {message}")

def connect_esp32():
    """Connect to ESP32 with proper settings to avoid reset"""
    try:
        ser = serial.Serial(
            port=ESP32_PORT,
            baudrate=BAUDRATE,
            timeout=TIMEOUT,
            dsrdtr=False,  # Don't toggle DTR (prevents reset)
            rtscts=False   # No hardware flow control
        )
        time.sleep(2)  # Wait for ESP32 to stabilize
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        log(f"Connected to ESP32 on {ESP32_PORT}")
        return ser
    except serial.SerialException as e:
        log(f"ERROR: Could not connect to {ESP32_PORT}: {e}")
        log("Is another program (like 'pio monitor') using the port?")
        return None

def send_command(ser, command):
    """Send command and wait for response"""
    log(f"Sending: {command}")
    ser.write(f"{command}\n".encode('utf-8'))
    time.sleep(0.5)

def sync_time(ser):
    """Synchronize ESP32 time with RPi current time"""
    
    # Wait for ESP32 to be ready (either booting or already running)
    log("Waiting for ESP32 to be ready...")
    boot_complete = False
    esp32_alive = False
    start_wait = time.time()
    boot_timeout = 8  # Reduced from 15 to 8 seconds
    
    while time.time() - start_wait < boot_timeout:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                log(f"Boot: {line}")
                esp32_alive = True  # ESP32 is responding
                
            # Look for completion messages (boot or already running)
            if any(phrase in line for phrase in [
                "Setup complete! Starting data logging",
                "Data written to",
                "Reading Renogy data",
                "Temperature reading"
            ]):
                boot_complete = True
                log("ESP32 ready for commands")
                time.sleep(0.5)  # Brief stabilization
                break
        time.sleep(0.1)
    
    if not boot_complete and esp32_alive:
        log("ESP32 detected but boot not confirmed - proceeding anyway")
    elif not esp32_alive:
        log("WARNING: No response from ESP32, attempting sync anyway...")
    
    # Clear any remaining messages
    time.sleep(0.3)
    ser.reset_input_buffer()
    
    # Get current time in local timezone
    # Calculate offset from UTC (handles EST/EDT automatically)
    from datetime import datetime, timezone
    
    now_local = datetime.now()
    now_utc = datetime.now(timezone.utc).replace(tzinfo=None)
    offset_seconds = int((now_local - now_utc).total_seconds())
    
    # Get UTC epoch and adjust for local timezone
    current_epoch_utc = int(time.time())
    current_epoch_local = current_epoch_utc + offset_seconds
    
    log(f"Local time: {now_local.strftime('%Y-%m-%d %H:%M:%S')}")
    log(f"Timezone: UTC{offset_seconds/3600:+.1f} hours")
    
    # Send SETTIME command with timezone-adjusted timestamp
    command = f"SETTIME {current_epoch_local}"
    log(f"Syncing time: {command}")
    ser.write(f"{command}\n".encode('utf-8'))
    time.sleep(0.5)
    
    # Wait for confirmation
    start_time = time.time()
    timeout = 5
    
    while time.time() - start_time < timeout:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("TIME_SYNCED:"):
                log(f"Time synced successfully: {line}")
                # Read and display current timestamp from ESP32
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

def download_csv_data(ser, output_file, delete_after=False):
    """Download CSV data from ESP32"""
    
    download_start = time.time()
    
    # Request download
    command = "DOWNLOAD_DELETE" if delete_after else "DOWNLOAD"
    send_command(ser, command)
    
    log(f"Downloading to: {output_file}")
    log(f"Delete after download: {delete_after}")
    
    csv_data = []
    in_data = False
    line_count = 0
    file_name = None
    file_size = None
    
    start_time = time.time()
    timeout = 30  # 30 second timeout for complete download
    
    while time.time() - start_time < timeout:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if not line:
                continue
            
            # Log non-data lines for debugging
            if not in_data or line.startswith(('FILE_', 'DOWNLOAD_', 'COMMAND_', 'CSV file')):
                log(f"RX: {line}")
            
            # Check for protocol markers
            if line.startswith("DOWNLOAD_DELETE_START") or line.startswith("DOWNLOAD_START"):
                in_data = True
                log("Started receiving data from ESP32...")
                continue
            elif line.startswith("DOWNLOAD_DELETE_END") or line.startswith("DOWNLOAD_END"):
                log(f"Download complete: {line_count} lines received")
                in_data = False
                break
            elif line.startswith("FILE_NAME:"):
                file_name = line.split(":", 1)[1]
                log(f"File: {file_name}")
                continue
            elif line.startswith("FILE_SIZE:"):
                file_size = line.split(":", 1)[1]
                log(f"Size: {file_size} bytes")
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
                # Ignore ESP32 status messages
                continue
            elif in_data:
                # This is actual CSV data - collect it
                csv_data.append(line)
                line_count += 1
                # Log progress every 100 lines
                if line_count % 100 == 0:
                    log(f"Progress: {line_count} lines received...")
            
        time.sleep(0.01)
    
    if not csv_data:
        log("WARNING: No CSV data received!")
        return False
    
    # Save to file
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'w') as f:
        f.write('\n'.join(csv_data))
        f.write('\n')  # Add final newline
    
    log(f"Saved {line_count} lines to {output_file}")
    
    # Verify file was written
    if os.path.exists(output_file):
        file_size_bytes = os.path.getsize(output_file)
        elapsed = time.time() - download_start
        log(f"File size: {file_size_bytes} bytes")
        log(f"Download time: {elapsed:.1f} seconds")
        
        # Quick timestamp verification
        if line_count > 0:
            first_line = csv_data[0]
            # Check if timestamp looks reasonable (starts with 20xx-xx-xx)
            if first_line.startswith(('2024', '2025', '2026')):
                log(f"✓ Timestamp format verified: {first_line.split(',')[0]}")
            else:
                log(f"⚠ Warning: Unexpected timestamp format: {first_line.split(',')[0]}")
    
    return True

def main():
    script_start = time.time()
    delete_after = "--delete" in sys.argv
    
    log(f"=== ESP32 Auto Download Started ===")
    log(f"Delete after download: {delete_after}")
    
    # Connect to ESP32
    ser = connect_esp32()
    if not ser:
        sys.exit(1)
    
    try:
        # Synchronize time first
        log("Synchronizing ESP32 time...")
        sync_time(ser)
        time.sleep(1)  # Give ESP32 a moment to process
        
        # Generate filename with date
        date_str = datetime.now().strftime('%Y%m%d')
        output_file = f"/home/al/esp32_data/esp32_data_{date_str}.csv"
        
        # Download data
        success = download_csv_data(ser, output_file, delete_after)
        
        if success:
            log("SUCCESS: Download completed")
            sys.exit(0)
        else:
            log("ERROR: Download failed")
            sys.exit(1)
            
    except Exception as e:
        log(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        ser.close()
        log("Connection closed")
        total_elapsed = time.time() - script_start
        log(f"Total time: {total_elapsed:.1f} seconds")
        log("=== ESP32 Auto Download Finished ===")

if __name__ == "__main__":
    main()
