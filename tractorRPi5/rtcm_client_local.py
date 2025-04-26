"""
rtcm_tcp_client_local.py
=======================
This script runs on a Raspberry Pi 5 to connect to a TCP server on a Raspberry Pi 3 (192.168.1.233:6001),
receive RTCM data, and forward it to a rover (PX1172RD-EVB on /dev/ttyUSB0). It monitors the rover's NMEA
output to check RTK status and logs both RTCM and NMEA data to files. It also tracks the rate of each RTCM message type.

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
RATE_REPORT_INTERVAL = 10  # Report RTCM message rates every 10 seconds
LOG_DIR = "logs"
RECONNECT_DELAY = 5  # Seconds to wait before reconnecting

def is_rtcm_data(data):
    """Check if data starts with RTCM 3.x preamble (0xD3)."""
    return data.startswith(b'\xD3')

def parse_rtcm_message(data, pos):
    """
    Parse an RTCM message starting at pos.
    Returns (message_type, message_length, new_pos).
    """
    if len(data) < pos + 3:
        return None, None, pos  # Not enough data for header
    if data[pos] != 0xD3:
        return None, None, pos  # Not an RTCM message

    # Extract message length (10 bits) from bytes 1-2
    length = ((data[pos + 1] & 0x03) << 8) | data[pos + 2]
    if len(data) < pos + 3 + length:
        return None, None, pos  # Not enough data for full message

    # Extract message type (12 bits) from bytes 3-4
    message_type = ((data[pos + 3] & 0x0F) << 8) | data[pos + 4]
    new_pos = pos + 3 + length  # Move to end of message
    return message_type, length, new_pos

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
    rtcm_buffer = b''  # Buffer to handle jitter
    message_counts = {}  # Dictionary to track counts of each message type
    last_rate_check = time.time()

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

                rtcm_buffer += data

                # Parse RTCM messages from the buffer
                pos = 0
                while pos < len(rtcm_buffer):
                    message_type, length, new_pos = parse_rtcm_message(rtcm_buffer, pos)
                    if message_type is None:
                        break  # Incomplete message, wait for more data
                    message_counts[message_type] = message_counts.get(message_type, 0) + 1
                    pos = new_pos

                # Remove processed messages from buffer
                rtcm_buffer = rtcm_buffer[pos:]

                # Send RTCM data every 0.5s to mitigate jitter
                current_time = time.time()
                if current_time - last_rate_check >= 0.5:
                    if rtcm_buffer:
                        rover.write(rtcm_buffer)
                        rover.flush()
                        rtcm_log.write(rtcm_buffer)
                        rtcm_log.flush()

                    # Report message rates every 10 seconds
                    if current_time - last_rate_check >= RATE_REPORT_INTERVAL:
                        elapsed = current_time - last_rate_check
                        if elapsed > 0:
                            logging.info("RTCM Message Rates (messages/sec):")
                            total_count = 0
                            for msg_type, count in sorted(message_counts.items()):
                                rate = count / elapsed
                                total_count += count
                                logging.info(f"  Type {msg_type}: {rate:.2f} Hz ({count} messages)")
                            total_rate = total_count / elapsed
                            logging.info(f"  Total: {total_rate:.2f} Hz ({total_count} messages in {elapsed:.2f}s)")
                        message_counts.clear()  # Reset counts
                        last_rate_check = current_time

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