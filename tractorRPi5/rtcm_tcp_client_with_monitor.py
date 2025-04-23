"""
rtcm_tcp_client_with_monitor.py
=======================
This script runs on a laptop to connect to a TCP server on a Raspberry Pi (192.168.1.233:5000),
receive RTCM data, and forward it to a rover (PX1172RDP on COM36). It monitors the rover's NMEA
output to check RTK status and logs both RTCM and NMEA data to files.

Usage:
    python rtcm_tcp_client_with_monitor.py
"""

import socket
import serial
import threading
import time
from datetime import datetime
import os

# Configuration
TCP_HOST = "192.168.1.233"  # Raspberry Pi local IP
TCP_PORT = 5000
ROVER_PORT = "COM36"  # PX1172RDP (Rover)
BAUD_RATE = 115200
STATUS_INTERVAL = 5  # Print RTK status every 5 seconds
LOG_DIR = "logs"

# Shared variables
rover = None
rover_ready = threading.Event()

def open_serial():
    """Attempts to open the serial port."""
    try:
        ser = serial.Serial(ROVER_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully opened {ROVER_PORT}.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening {ROVER_PORT}: {e}")
        return None

def forward_rtcm():
    """Connects to the TCP server, receives RTCM data, and forwards it to the rover."""
    global rover

    # Open log file for RTCM data
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
    rtcm_log = open(f"{LOG_DIR}/rtcm_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log", "wb")

    rover = open_serial()  # Open COM36
    if not rover:
        print("Failed to open rover port. Exiting.")
        rtcm_log.close()
        return

    rover_ready.set()  # Signal that rover port is ready

    while True:
        try:
            print(f"Connecting to TCP server at {TCP_HOST}:{TCP_PORT}...")
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((TCP_HOST, TCP_PORT))
            print("Connected to TCP server.")
            break
        except Exception as e:
            print(f"Failed to connect to TCP server: {e}. Retrying in 5 seconds...")
            time.sleep(5)

    print(f"Forwarding RTCM data to {ROVER_PORT}...")
    try:
        while True:
            data = client_socket.recv(1024)  # Receive RTCM data in chunks
            if not data:
                print("TCP connection closed by server. Reconnecting...")
                break
            rover.write(data)  # Send RTCM to Rover
            rtcm_log.write(data)  # Log RTCM data
            rtcm_log.flush()
    except Exception as e:
        print(f"Error in RTCM forwarding: {e}")
    finally:
        client_socket.close()
        rover.close()
        rtcm_log.close()

def monitor_nmea():
    """Reads NMEA output from Rover and prints RTK status every 5 seconds with last GGA message."""
    # Open log file for NMEA data
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
    nmea_log = open(f"{LOG_DIR}/nmea_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log", "w")

    print("Waiting for Rover (COM36) to be ready...")
    rover_ready.wait()  # Wait until `forward_rtcm` opens COM36

    print(f"Listening to NMEA data from {ROVER_PORT}...")
    last_status_time = time.time()
    last_gga_message = None
    first_rtk_fix_time = None

    try:
        while True:
            if not rover.is_open:
                print("Rover port closed unexpectedly.")
                break
            line = rover.readline().decode(errors='ignore').strip()
            if not line:
                continue

            # Log NMEA data
            nmea_log.write(f"[{datetime.now().strftime('%H:%M:%S')}] {line}\n")
            nmea_log.flush()

            if line.startswith("$GPGGA"):
                last_gga_message = line
                current_time = time.time()
                if current_time - last_status_time >= STATUS_INTERVAL:
                    fields = line.split(',')
                    fix_type = fields[6]
                    num_satellites = fields[7]
                    altitude = fields[9]

                    status = {
                        "0": "No Fix",
                        "1": "GPS Fix",
                        "2": "DGPS Fix",
                        "4": "RTK Fixed",
                        "5": "RTK Float"
                    }.get(fix_type, "Unknown")

                    print(f"[{time.strftime('%H:%M:%S')}] RTK Status: {status} | Satellites: {num_satellites} | Altitude: {altitude}m")
                    print(f"Last GGA Message: {last_gga_message}")

                    # Track time to first RTK fix
                    if fix_type in ["4", "5"] and first_rtk_fix_time is None:
                        first_rtk_fix_time = time.time()
                        elapsed = first_rtk_fix_time - start_time
                        print(f"First RTK Fix achieved after {elapsed:.1f} seconds!")

                    last_status_time = current_time

    except Exception as e:
        print(f"Error in NMEA monitoring: {e}")
    finally:
        nmea_log.close()

# Run both functions in parallel
start_time = time.time()
rtcm_thread = threading.Thread(target=forward_rtcm, daemon=True)
nmea_thread = threading.Thread(target=monitor_nmea, daemon=True)

rtcm_thread.start()
nmea_thread.start()

# Keep script running
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting...")