#!/usr/bin/env python3
"""
ESP32 Data Downloader
Downloads CSV data from ESP32 via serial connection
"""

import serial
import time
import sys
import os
from datetime import datetime

def find_esp32_port():
    """Try to find the ESP32 USB port automatically"""
    common_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
    for port in common_ports:
        if os.path.exists(port):
            return port
    return None

def connect_to_esp32(port='/dev/ttyUSB0', baudrate=115200, timeout=5):
    """Connect to ESP32 via serial"""
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(1)  # Give ESP32 time to respond
        
        # Clear any existing data in buffers
        ser.flushInput()
        ser.flushOutput()
        
        print(f"Connected to ESP32 on {port}")
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to {port}: {e}")
        return None

def send_command(ser, command):
    """Send a command to ESP32 and wait for response"""
    print(f"Sending command: {command}")
    
    # Clear input buffer before sending command
    ser.flushInput()
    
    # Send command
    ser.write(f"{command}\n".encode())
    ser.flush()
    
    # Give ESP32 time to process
    time.sleep(0.5)

def download_csv_data(ser, output_file=None, delete_after=False):
    """Download CSV data from ESP32"""
    if output_file is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"esp32_data_{timestamp}.csv"
    
    command = "DOWNLOAD_DELETE" if delete_after else "DOWNLOAD"
    send_command(ser, command)
    
    # Wait for download start confirmation
    start_marker = "DOWNLOAD_DELETE_START" if delete_after else "DOWNLOAD_START"
    start_found = False
    timeout_count = 0
    max_timeouts = 50
    
    while not start_found and timeout_count < max_timeouts:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line == start_marker:
                print("Download started...")
                start_found = True
                break
            elif line.startswith("ERROR"):
                print(f"Error from ESP32: {line}")
                return False
            elif line and not any(x in line.lower() for x in ['temperature', 'renogy', 'reading', 'data written']):
                # Ignore normal logging, but show other messages
                print(f"ESP32: {line}")
            timeout_count += 1
            time.sleep(0.1)
        except Exception as e:
            print(f"Error waiting for download start: {e}")
            return False
    
    if not start_found:
        print("No download start confirmation received")
        return False
    
    # Read file info
    filename = None
    file_size = None
    
    # Read filename and file size
    for _ in range(3):
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("FILE_NAME:"):
                filename = line.split(':', 1)[1]
                print(f"Downloading file: {filename}")
            elif line.startswith("FILE_SIZE:"):
                file_size = int(line.split(':')[1])
                print(f"File size: {file_size} bytes")
        except:
            continue
    
    # Read the CSV data
    csv_data = []
    bytes_received = 0
    end_marker = "DOWNLOAD_DELETE_END" if delete_after else "DOWNLOAD_END"
    
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line == end_marker:
                print("Download completed!")
                break
            elif line.startswith("FILE_DELETED:"):
                deleted_file = line.split(':', 1)[1]
                print(f"File deleted from ESP32: {deleted_file}")
            elif line.startswith("NEW_FILE_CREATED:"):
                new_file = line.split(':', 1)[1]
                print(f"New file created on ESP32: {new_file}")
            elif line.startswith("ERROR:"):
                print(f"ESP32 Error: {line}")
            elif line and ',' in line and not any(x in line.lower() for x in ['file', 'spiffs', 'command', 'error']) and not line.startswith("Timestamp,"):
                # This looks like CSV data (contains commas, isn't a status message, and isn't a header)
                csv_data.append(line)
                bytes_received += len(line) + 1  # +1 for newline
        except Exception as e:
            print(f"Error during download: {e}")
            break
    
    # Write to file
    try:
        with open(output_file, 'w') as f:
            # Always write the header first
            f.write("Timestamp,Avg_Temp_C,Avg_Temp_F,Battery_Voltage,Battery_SOC,Battery_Charging_Amps,Solar_Panel_Voltage,Solar_Panel_Amps,Solar_Panel_Watts,Controller_Temp_C,Battery_Temp_C,Load_Voltage,Load_Amps,Load_Watts\n")
            
            # Then write the data
            for line in csv_data:
                f.write(line + '\n')
        
        print(f"Data saved to: {output_file}")
        print(f"Data lines written: {len(csv_data)}")
        print(f"Total lines (including header): {len(csv_data) + 1}")
        print(f"File size: {os.path.getsize(output_file)} bytes")
        
        if delete_after:
            print("File was deleted from ESP32 after download")
        
        return True
        
    except Exception as e:
        print(f"Error writing file: {e}")
        return False

def get_status(ser):
    """Get file status from ESP32"""
    send_command(ser, "STATUS")
    
    print("ESP32 Status:")
    status_lines = []
    command_complete = False
    timeout_count = 0
    max_timeouts = 50  # Increased timeout
    
    while not command_complete and timeout_count < max_timeouts:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                # Check for command completion marker
                if line == "COMMAND_COMPLETE":
                    command_complete = True
                    break
                # Filter out normal logging messages
                elif not any(x in line.lower() for x in ['temperature', 'renogy', 'reading', 'data written']):
                    status_lines.append(line)
                    print(f"  {line}")
                timeout_count = 0  # Reset timeout counter on successful read
            else:
                timeout_count += 1
                time.sleep(0.1)
        except Exception as e:
            print(f"Error reading status: {e}")
            break
    
    if not command_complete:
        print("Status command may not have completed properly")
    
    return status_lines

def clear_data(ser):
    """Clear CSV data on ESP32"""
    response = input("Are you sure you want to clear all data on ESP32? (y/N): ")
    if response.lower() == 'y':
        send_command(ser, "CLEAR")
        
        for _ in range(5):
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"ESP32: {line}")

def interactive_mode(ser):
    """Interactive mode for sending commands"""
    print("\nInteractive mode - Available commands:")
    print("  download [filename]        - Download CSV data (keep on ESP32)")
    print("  download_delete [filename] - Download CSV data and delete from ESP32")
    print("  status                     - Show file status")
    print("  clear                      - Clear CSV data")
    print("  help                       - Show ESP32 help")
    print("  quit                       - Exit")
    
    while True:
        try:
            command = input("\nESP32> ").strip().split()
            if not command:
                continue
                
            if command[0] == 'quit':
                break
            elif command[0] == 'download':
                filename = command[1] if len(command) > 1 else None
                download_csv_data(ser, filename, delete_after=False)
            elif command[0] == 'download_delete':
                filename = command[1] if len(command) > 1 else None
                download_csv_data(ser, filename, delete_after=True)
            elif command[0] == 'status':
                get_status(ser)
            elif command[0] == 'clear':
                clear_data(ser)
            elif command[0] == 'help':
                send_command(ser, "HELP")
                for _ in range(10):
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(line)
                    else:
                        break
            else:
                print("Unknown command. Available: download, download_delete, status, clear, help, quit")
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break

def main():
    # Try to find ESP32 port automatically
    port = find_esp32_port()
    if not port:
        port = '/dev/ttyUSB0'  # Default fallback
        print(f"Could not auto-detect ESP32 port, using default: {port}")
    
    # Connect to ESP32
    ser = connect_to_esp32(port)
    if not ser:
        print("Failed to connect to ESP32")
        sys.exit(1)
    
    try:
        # Check if command line arguments provided
        if len(sys.argv) > 1:
            command = sys.argv[1].lower()
            if command == 'download':
                filename = sys.argv[2] if len(sys.argv) > 2 else None
                success = download_csv_data(ser, filename, delete_after=False)
                sys.exit(0 if success else 1)
            elif command == 'download_delete':
                filename = sys.argv[2] if len(sys.argv) > 2 else None
                success = download_csv_data(ser, filename, delete_after=True)
                sys.exit(0 if success else 1)
            elif command == 'status':
                get_status(ser)
                sys.exit(0)
            else:
                print(f"Unknown command: {command}")
                print("Usage: python3 esp32_downloader.py [download|download_delete|status] [filename]")
                print("       python3 esp32_downloader.py download_delete  # For daily cron job")
                sys.exit(1)
        else:
            # Interactive mode
            interactive_mode(ser)
            
    finally:
        ser.close()
        print("Serial connection closed")

if __name__ == "__main__":
    main()
