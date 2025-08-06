"""
rtcm_tcp_client_0507.py
=======================
This script runs on a Raspberry Pi 5 to connect to a TCP server on a Raspberry Pi 3 (192.168.1.233:6001),
receive RTCM data, and forward it to a rover (PX1172RD-EVB on /dev/ttyUSB0). It monitors the rover's NMEA
output to check RTK status and logs both RTCM and NMEA data with time-based rotation. It also tracks RTCM message rates
and decodes critical messages (e.g., 1005) for debugging.
"""

import socket
import serial
import threading
import time
from datetime import datetime
import os
import logging
import logging.handlers
import struct
import glob
import gzip

# Configure application logging
logger = logging.getLogger('RTCMClient')
logger.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

log_dir = "logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

app_log_handler = logging.handlers.TimedRotatingFileHandler(
    filename=os.path.join(log_dir, 'rtcm_client.log'),
    when='midnight',
    interval=1,
    backupCount=7
)
app_log_handler.setFormatter(formatter)
logger.addHandler(app_log_handler)
console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

# Configuration
TCP_HOST = "192.168.1.233"
TCP_PORT = 6001
ROVER_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
STATUS_INTERVAL = 5
RATE_REPORT_INTERVAL = 15
LOG_INTERVAL = 3600
LOG_RETENTION = 24
RTCM_LOG_DIR = os.path.join(log_dir, "rtcm")
NMEA_LOG_DIR = os.path.join(log_dir, "nmea")
RAW_LOG_DIR = os.path.join(log_dir, "raw")
RTCM_LOG_PREFIX = "rtcm"
NMEA_LOG_PREFIX = "nmea"
RAW_LOG_PREFIX = "raw"
RECONNECT_DELAY = 5
COMPRESS_LOGS = True

def calculate_rtcm_crc(data):
    """Calculate the 24-bit CRC for an RTCM message."""
    crc = 0
    for i in range(len(data)):
        crc ^= (data[i] << 16)
        for j in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        crc &= 0xFFFFFF
    return crc

def decode_1005(data):
    """Decode RTCM 1005 message to extract base station coordinates."""
    try:
        if len(data) < 19:  # Minimum length for 1005
            return None
        # Extract fields (Reference Station ID: 12 bits, ECEF X, Y, Z: 38 bits each, etc.)
        payload = data[3:-3]  # Exclude header and CRC
        ref_station_id = (struct.unpack('>H', payload[0:2])[0] & 0xFFF)  # First 12 bits
        ecef_x = struct.unpack('>i', payload[2:6])[0] / 100.0  # mm to meters
        ecef_y = struct.unpack('>i', payload[6:10])[0] / 100.0
        ecef_z = struct.unpack('>i', payload[10:14])[0] / 100.0
        return {
            "ref_station_id": ref_station_id,
            "ecef_x": ecef_x,
            "ecef_y": ecef_y,
            "ecef_z": ecef_z
        }
    except Exception as e:
        logger.error(f"Error decoding 1005: {e}")
        return None

def parse_rtcm_message(data, pos, crc_errors):
    """
    Parse an RTCM message starting at pos.
    Returns (message_type, message_length, new_pos, decoded_data) or (None, None, pos, None).
    """
    if len(data) < pos + 3:
        return None, None, pos, None
    if data[pos] != 0xD3:
        return None, None, pos, None

    length = ((data[pos + 1] & 0x03) << 8) | data[pos + 2]
    total_length = 3 + length + 3
    if len(data) < pos + total_length:
        return None, None, pos, None

    message = data[pos:pos + total_length]
    calculated_crc = calculate_rtcm_crc(message[:-3])
    crc_bytes = message[-3:]
    received_crc = struct.unpack('>I', b'\x00' + crc_bytes)[0]
    if calculated_crc != received_crc:
        logger.warning(f"CRC mismatch at buffer pos {pos}, skipping. Total CRC errors: {crc_errors}")
        return None, None, pos + 1, None

    if len(message) < 5:
        return None, None, pos + 1, None
    msg_type = struct.unpack('>H', message[3:5])[0] >> 4
    new_pos = pos + total_length

    # Decode specific messages
    decoded_data = None
    if msg_type == 1005:
        decoded_data = decode_1005(message)
        if decoded_data:
            logger.info(f"1005 Decoded: RefStationID={decoded_data['ref_station_id']}, "
                       f"ECEF_X={decoded_data['ecef_x']:.3f}m, "
                       f"ECEF_Y={decoded_data['ecef_y']:.3f}m, "
                       f"ECEF_Z={decoded_data['ecef_z']:.3f}m")

    return msg_type, length, new_pos, decoded_data

def connect_to_server():
    """Connect to the TCP server with retry logic."""
    while True:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((TCP_HOST, TCP_PORT))
            logger.info(f"Connected to TCP server at {TCP_HOST}:{TCP_PORT}")
            return client_socket
        except Exception as e:
            logger.error(f"Failed to connect to TCP server: {e}. Retrying in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)

def manage_logs(log_dir, prefix, retention):
    """Delete old log files to enforce retention policy."""
    log_files = sorted(glob.glob(os.path.join(log_dir, f"{prefix}_*.log*")))
    while len(log_files) > retention:
        oldest_file = log_files.pop(0)
        try:
            os.remove(oldest_file)
            logger.info(f"Deleted old log: {oldest_file}")
        except OSError as e:
            logger.error(f"Error deleting {oldest_file}: {e}")

def compress_log(filename):
    """Compress a log file using gzip."""
    if not COMPRESS_LOGS:
        return filename
    compressed_file = f"{filename}.gz"
    try:
        with open(filename, 'rb') as f_in:
            with gzip.open(compressed_file, 'wb') as f_out:
                f_out.writelines(f_in)
        os.remove(filename)
        logger.info(f"Compressed log: {compressed_file}")
        return compressed_file
    except Exception as e:
        logger.error(f"Error compressing {filename}: {e}")
        return filename

def handle_gnss():
    """Handle RTCM forwarding and NMEA monitoring."""
    # Initialize log files
    if not os.path.exists(RTCM_LOG_DIR):
        os.makedirs(RTCM_LOG_DIR)
    if not os.path.exists(NMEA_LOG_DIR):
        os.makedirs(NMEA_LOG_DIR)
    if not os.path.exists(RAW_LOG_DIR):
        os.makedirs(RAW_LOG_DIR)
    rtcm_log_filename = os.path.join(RTCM_LOG_DIR, f"{RTCM_LOG_PREFIX}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
    nmea_log_filename = os.path.join(NMEA_LOG_DIR, f"{NMEA_LOG_PREFIX}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
    raw_log_filename = os.path.join(RAW_LOG_DIR, f"{RAW_LOG_PREFIX}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
    rtcm_log = open(rtcm_log_filename, "wb")
    nmea_log = open(nmea_log_filename, "w")
    raw_log = open(raw_log_filename, "wb")
    last_rotation = time.time()

    # Initialize serial port
    try:
        rover = serial.Serial(ROVER_PORT, BAUD_RATE, timeout=1)
        logger.info(f"Successfully opened {ROVER_PORT}")
    except serial.SerialException as e:
        logger.error(f"Failed to open {ROVER_PORT}: {e}")
        rtcm_log.close()
        nmea_log.close()
        raw_log.close()
        return

    # Variables for RTCM rate monitoring
    rtcm_buffer = b''
    message_counts = {}
    crc_errors = 0
    last_rate_check = time.time()

    # Variables for NMEA monitoring
    last_status_time = time.time()
    last_gga_message = None
    first_rtk_fix_time = None
    start_time = time.time()

    while True:
        client_socket = connect_to_server()

        try:
            while True:
                # Receive RTCM data
                data = client_socket.recv(2048)
                if not data:
                    logger.warning("TCP connection closed by server. Reconnecting...")
                    break

                # Log raw data
                raw_log.write(data)
                raw_log.flush()

                rtcm_buffer += data

                # Parse RTCM messages
                pos = 0
                while pos < len(rtcm_buffer):
                    message_type, length, new_pos, decoded_data = parse_rtcm_message(rtcm_buffer, pos, crc_errors)
                    if message_type is None:
                        if new_pos > pos:
                            crc_errors += 1
                        pos = new_pos if new_pos > pos else pos + 1
                        continue
                    message_counts[message_type] = message_counts.get(message_type, 0) + 1
                    pos = new_pos

                # Forward RTCM data to rover
                if rtcm_buffer:
                    rover.write(rtcm_buffer)
                    rover.flush()
                    rtcm_log.write(rtcm_buffer)
                    rtcm_log.flush()

                # Remove processed messages
                rtcm_buffer = rtcm_buffer[pos:]

                # Rotate logs
                current_time = time.time()
                if current_time - last_rotation >= LOG_INTERVAL:
                    rtcm_log.close()
                    compress_log(rtcm_log_filename)
                    manage_logs(RTCM_LOG_DIR, RTCM_LOG_PREFIX, LOG_RETENTION)
                    rtcm_log_filename = os.path.join(RTCM_LOG_DIR, f"{RTCM_LOG_PREFIX}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
                    rtcm_log = open(rtcm_log_filename, "wb")
                    logger.info(f"Rotated RTCM log to: {rtcm_log_filename}")

                    nmea_log.close()
                    compress_log(nmea_log_filename)
                    manage_logs(NMEA_LOG_DIR, NMEA_LOG_PREFIX, LOG_RETENTION)
                    nmea_log_filename = os.path.join(NMEA_LOG_DIR, f"{NMEA_LOG_PREFIX}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
                    nmea_log = open(nmea_log_filename, "w")
                    logger.info(f"Rotated NMEA log to: {nmea_log_filename}")

                    raw_log.close()
                    compress_log(raw_log_filename)
                    manage_logs(RAW_LOG_DIR, RAW_LOG_PREFIX, LOG_RETENTION)
                    raw_log_filename = os.path.join(RAW_LOG_DIR, f"{RAW_LOG_PREFIX}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
                    raw_log = open(raw_log_filename, "wb")
                    logger.info(f"Rotated raw log to: {raw_log_filename}")

                    last_rotation = current_time

                # Report RTCM message rates
                if current_time - last_rate_check >= RATE_REPORT_INTERVAL:
                    elapsed = current_time - last_rate_check
                    if elapsed > 0:
                        logger.info("RTCM Message Rates (messages/sec):")
                        total_count = 0
                        for msg_type, count in sorted(message_counts.items()):
                            rate = count / elapsed
                            total_count += count
                            logger.info(f"  Type {msg_type}: {rate:.2f} Hz ({count} messages)")
                        total_rate = total_count / elapsed
                        logger.info(f"  Total: {total_rate:.2f} Hz ({total_count} messages in {elapsed:.2f}s)")
                        logger.info(f"  CRC Errors: {crc_errors / elapsed:.2f} Hz ({crc_errors} errors)")
                        crc_errors = 0
                    message_counts.clear()
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

                        logger.info(f"RTK Status: {status} | Satellites: {num_satellites} | Altitude: {altitude}m")
                        logger.info(f"Last GGA Message: {last_gga_message}")

                        if fix_type in ["4", "5"] and first_rtk_fix_time is None:
                            first_rtk_fix_time = current_time
                            elapsed = first_rtk_fix_time - start_time
                            logger.info(f"First RTK Fix achieved after {elapsed:.1f} seconds!")
                        elif fix_type == "4" and first_rtk_fix_time is not None:
                            logger.info(f"RTK Fixed achieved after {current_time - start_time:.1f} seconds!")

                        last_status_time = current_time

                time.sleep(0.1)

        except Exception as e:
            logger.error(f"Error in GNSS handling: {e}")
            client_socket.close()
        finally:
            client_socket.close()

    # Cleanup
    rover.close()
    rtcm_log.close()
    nmea_log.close()
    raw_log.close()
    compress_log(rtcm_log_filename)
    compress_log(nmea_log_filename)
    compress_log(raw_log_filename)
    manage_logs(RTCM_LOG_DIR, RTCM_LOG_PREFIX, LOG_RETENTION)
    manage_logs(NMEA_LOG_DIR, NMEA_LOG_PREFIX, LOG_RETENTION)
    manage_logs(RAW_LOG_DIR, RAW_LOG_PREFIX, LOG_RETENTION)

if __name__ == "__main__":
    try:
        handle_gnss()
    except KeyboardInterrupt:
        logger.info("Exiting...")