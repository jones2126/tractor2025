"""
rtcm_tcp_server.py
=======================
This script runs on a Raspberry Pi 3 Model B to read RTCM data from a base station (PX1125R on /dev/ttyUSB0)
and stream it over TCP to connected clients. It logs RTCM data to a file with time-based rotation and reports 
RTCM message type rates.
"""

import socket
import serial
import threading
import time
from datetime import datetime
import os
import logging
import struct
import glob
import gzip

# Configure application logging
logger = logging.getLogger('RTCMServer')
logger.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

# Use TimedRotatingFileHandler for application logs
log_dir = "logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
app_log_handler = logging.handlers.TimedRotatingFileHandler(
    filename=os.path.join(log_dir, 'rtcm_server.log'),
    when='midnight',  # Rotate at midnight daily
    interval=1,
    backupCount=7  # Keep 7 days of logs
)
app_log_handler.setFormatter(formatter)
logger.addHandler(app_log_handler)
console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

# Configuration
BASE_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
TCP_HOST = "0.0.0.0"
TCP_PORT = 6001
RTCM_LOG_DIR = os.path.join(log_dir, "rtcm")
RTCM_LOG_INTERVAL = 3600  # Rotate RTCM logs every hour (3600 seconds)
RTCM_LOG_RETENTION = 24  # Keep 24 hours of RTCM logs (24 files)
RTCM_LOG_PREFIX = "rtcm"
RATE_REPORT_INTERVAL = 10  # Report RTCM message rates every 10 seconds

# Shared variables
base = None
clients = []

def calculate_rtcm_crc(data):
    """Calculate the 24-bit CRC for an RTCM message."""
    crc = 0
    for i in range(len(data)):
        crc ^= (data[i] << 16)
        for j in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB  # CRC-24Q polynomial
        crc &= 0xFFFFFF
    return crc

def open_serial():
    """Attempts to open the serial port."""
    try:
        ser = serial.Serial(BASE_PORT, BAUD_RATE, timeout=1)
        logger.info(f"Successfully opened {BASE_PORT}")
        return ser
    except serial.SerialException as e:
        logger.error(f"Error opening {BASE_PORT}: {e}")
        return None

def parse_rtcm_message(data, pos):
    """
    Parse an RTCM message starting at pos.
    Returns (message_type, message_length, new_pos) or (None, None, pos) if invalid/incomplete.
    """
    if len(data) < pos + 3:
        return None, None, pos
    if data[pos] != 0xD3:
        return None, None, pos

    length = ((data[pos + 1] & 0x03) << 8) | data[pos + 2]
    total_length = 3 + length + 3
    if len(data) < pos + total_length:
        return None, None, pos

    message = data[pos:pos + total_length]
    calculated_crc = calculate_rtcm_crc(message[:-3])
    crc_bytes = message[-3:]
    received_crc = struct.unpack('>I', b'\x00' + crc_bytes)[0]
    if calculated_crc != received_crc:
        logger.warning(f"CRC mismatch at buffer pos {pos}, skipping.")
        return None, None, pos + 1

    if len(message) < 5:
        return None, None, pos + 1
    msg_type = struct.unpack('>H', message[3:5])[0] >> 4
    new_pos = pos + total_length
    return msg_type, length, new_pos

def handle_client(client_socket, address):
    """Handles a single TCP client connection."""
    logger.info(f"Client connected: {address}")
    clients.append(client_socket)
    try:
        while True:
            client_socket.sendall(b"")
            threading.Event().wait(1)
    except Exception as e:
        logger.info(f"Client {address} disconnected: {e}")
    finally:
        clients.remove(client_socket)
        client_socket.close()

def manage_rtcm_logs():
    """Delete old RTCM log files to enforce retention policy."""
    log_files = sorted(glob.glob(os.path.join(RTCM_LOG_DIR, f"{RTCM_LOG_PREFIX}_*.log*")))
    while len(log_files) > RTCM_LOG_RETENTION:
        oldest_file = log_files.pop(0)
        try:
            os.remove(oldest_file)
            logger.info(f"Deleted old RTCM log: {oldest_file}")
        except OSError as e:
            logger.error(f"Error deleting {oldest_file}: {e}")

def compress_rtcm_log(filename):
    """Compress a log file using gzip."""
    compressed_file = f"{filename}.gz"
    try:
        with open(filename, 'rb') as f_in:
            with gzip.open(compressed_file, 'wb') as f_out:
                f_out.writelines(f_in)
        os.remove(filename)  # Remove uncompressed file
        logger.info(f"Compressed RTCM log: {compressed_file}")
        return compressed_file
    except Exception as e:
        logger.error(f"Error compressing {filename}: {e}")
        return filename

def broadcast_rtcm():
    """Reads RTCM data from the base station and broadcasts it to all clients."""
    global base

    # Initialize RTCM log
    if not os.path.exists(RTCM_LOG_DIR):
        os.makedirs(RTCM_LOG_DIR)
    rtcm_log_filename = os.path.join(RTCM_LOG_DIR, f"{RTCM_LOG_PREFIX}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
    rtcm_log = open(rtcm_log_filename, "wb")
    last_rotation = time.time()

    base = open_serial()
    if not base:
        logger.error("Failed to open base port. Exiting.")
        rtcm_log.close()
        return

    logger.info(f"Reading RTCM data from {BASE_PORT} and broadcasting to clients...")

    rtcm_buffer = b''
    message_counts = {}
    last_rate_check = time.time()

    try:
        while True:
            data = base.read(1024)
            if data:
                rtcm_buffer += data
                rtcm_log.write(data)
                rtcm_log.flush()

                # Rotate RTCM log if interval has passed
                current_time = time.time()
                if current_time - last_rotation >= RTCM_LOG_INTERVAL:
                    rtcm_log.close()
                    # Compress the old log
                    compress_rtcm_log(rtcm_log_filename)
                    # Enforce retention
                    manage_rtcm_logs()
                    # Open new log file
                    rtcm_log_filename = os.path.join(RTCM_LOG_DIR, f"{RTCM_LOG_PREFIX}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
                    rtcm_log = open(rtcm_log_filename, "wb")
                    last_rotation = current_time
                    logger.info(f"Rotated RTCM log to: {rtcm_log_filename}")

                # Parse RTCM messages
                pos = 0
                while pos < len(rtcm_buffer):
                    message_type, length, new_pos = parse_rtcm_message(rtcm_buffer, pos)
                    if message_type is None:
                        pos += 1
                        continue
                    message_counts[message_type] = message_counts.get(message_type, 0) + 1
                    pos = new_pos

                if pos > 0:
                    rtcm_buffer = rtcm_buffer[pos:]

                for client in clients[:]:
                    try:
                        client.sendall(data)
                    except Exception as e:
                        logger.error(f"Error sending to client: {e}")
                        clients.remove(client)
                        client.close()

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
                    message_counts.clear()
                    last_rate_check = current_time

    except Exception as e:
        logger.error(f"Error in RTCM broadcasting: {e}")
    finally:
        base.close()
        rtcm_log.close()
        compress_rtcm_log(rtcm_log_filename)
        manage_rtcm_logs()

def start_tcp_server():
    """Starts the TCP server to accept client connections."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(5)
    logger.info(f"TCP server started on {TCP_HOST}:{TCP_PORT}")

    try:
        while True:
            client_socket, address = server.accept()
            client_thread = threading.Thread(target=handle_client, args=(client_socket, address), daemon=True)
            client_thread.start()
    except Exception as e:
        logger.error(f"TCP server error: {e}")
    finally:
        server.close()

# Run both TCP server and RTCM broadcasting in parallel
tcp_thread = threading.Thread(target=start_tcp_server, daemon=True)
rtcm_thread = threading.Thread(target=broadcast_rtcm, daemon=True)

tcp_thread.start()
rtcm_thread.start()

# Keep script running
try:
    while True:
        threading.Event().wait(1)
except KeyboardInterrupt:
    logger.info("Exiting...")