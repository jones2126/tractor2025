"""
rtcm_tcp_client_local.py
=======================
This script runs on a Raspberry Pi 5 to connect to a TCP server on a Raspberry Pi 3 (192.168.1.233:6001),
receive RTCM data, and forward it to a rover (PX1172RD-EVB on /dev/ttyUSB0). It monitors the rover's NMEA
output to check RTK status and logs both RTCM and NMEA data to files.

Usage:
    python rtcm_tcp_client_local.py
"""

import socket
import serial
import threading
import time
from datetime import datetime
import os
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Configuration
TCP_HOST = "192.168.1.233"  # Base station IP
TCP_PORT = 6001
ROVER_PORT = "/dev/ttyUSB0"  # PX1172RD-EVB (Rover)
BAUD_RATE = 115200
STATUS_INTERVAL = 5  # Print RTK status every 5 seconds
LOG_DIR = "logs"
RECONNECT_DELAY = 5  # Seconds to wait before reconnecting

def is_rtcm_data(data):
    """Check if data starts with RTCM 3.x preamble (0xD3)."""
    return data.startswith(b'\xD3')

def count_rtcm_messages(data, message_count):
    """Count RTCM messages in the data by looking for 0xD3 preambles."""
    count = 0
    for i in range(len(data)):
        if data[i] == 0xD3:  # RTCM 3.x preamble
            count += 1
    return message_count + count

def connect_to_server():
    """Connect to the TCP server with retry logic."""
    while True:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((TCP_HOST, TCP_PORT))
            logging.info(f"Connected to TCP server at {TCP_HOST}:{TCP_PORT}")
            return client_socket
        except Exception as e:
            logging.error(f"Failed to connect to TCP server: {e}. Retrying in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)

def handle_gnss():
    """Handle RTCM forwarding and NMEA monitoring in a single thread."""
    # Initialize log files
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
    rtcm_log = open(f"{LOG_DIR}/rtcm_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log", "wb")
    nmea_log = open(f"{LOG_DIR}/nmea_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log", "w")

    # Initialize serial port
    try:
        rover = serial.Serial(ROVER_PORT, BAUD_RATE, timeout=1)
        logging.info(f"Successfully opened {ROVER_PORT}")
    except serial.SerialException as e:
        logging.error(f"Failed to open {ROVER_PORT}: {e}")
        rtcm_log.close()
        nmea_log.close()
        return

    # Variables for RTCM rate monitoring
    rtcm_message_count = 0
    last_rate_check = time.time()
    rtcm_buffer = b''  # Buffer to handle jitter

    # Variables for NMEA monitoring
    last_status_time = time.time()
    last_gga_message = None
    first_rtk_fix_time = None
    start_time = time.time()

    while True:
        # Connect to TCP server
        client_socket = connect_to_server()

        try:
            while True:
                # Receive RTCM data
                data = client_socket.recv(1024)
                if not data:
                    logging.warning("TCP connection closed by server. Reconnecting...")
                    break

                # Validate and buffer RTCM data
                if is_rtcm_data(data):
                    rtcm_buffer += data
                    rtcm_message_count = count_rtcm_messages(data, rtcm_message_count)

                    # Send RTCM data every 0.5s to mitigate jitter
                    if time.time() - last_rate_check >= 0.5:
                        rover.write(rtcm_buffer)
                        rover.flush()
                        rtcm_log.write(rtcm_buffer)
                        rtcm_log.flush()
                        rtcm_buffer = b''

                        # Calculate and log RTCM message rate
                        elapsed = time.time() - last_rate_check
                        rate = rtcm_message_count / elapsed if elapsed > 0 else 0
                        logging.info(f"RTCM Message Rate: {rate:.2f} Hz ({rtcm_message_count} messages in {elapsed:.2f}s)")
                        rtcm_message_count = 0
                        last_rate_check = time.time()

                # Read NMEA data
                line = rover.readline().decode(errors='ignore').strip()
                if not line:
                    continue

                # Log NMEA data
                nmea_log.write(f"[{datetime.now().strftime('%H:%M:%S')}] {line}\n")
                nmea_log.flush()

                # Process $GPGGA for RTK status
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

                        logging.info(f"RTK Status: {status} | Satellites: {num_satellites} | Altitude: {altitude}m")
                        logging.info(f"Last GGA Message: {last_gga_message}")

                        # Track time to first RTK fix
                        if fix_type in ["4", "5"] and first_rtk_fix_time is None:
                            first_rtk_fix_time = time.time()
                            elapsed = first_rtk_fix_time - start_time
                            logging.info(f"First RTK Fix achieved after {elapsed:.1f} seconds!")

                        last_status_time = current_time

                time.sleep(0.1)  # Balance RTCM and NMEA at 1 Hz

        except Exception as e:
            logging.error(f"Error in GNSS handling: {e}")
            client_socket.close()
        finally:
            client_socket.close()

    # Cleanup (only reached if the loop breaks unexpectedly)
    rover.close()
    rtcm_log.close()
    nmea_log.close()

# Run the script
if __name__ == "__main__":
    try:
        handle_gnss()
    except KeyboardInterrupt:
        logging.info("Exiting...")