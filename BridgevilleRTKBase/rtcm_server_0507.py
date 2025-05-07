"""
rtcm_tcp_server.py
=======================
This script runs on a Raspberry Pi 3 to read RTCM data from a base station (PX1125R on /dev/ttyUSB0)
and stream it over TCP to connected clients. It logs RTCM data with time-based rotation and reports message rates.
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
logger = logging.getLogger('RTCMServer')
logger.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

log_dir = "logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

app_log_handler = logging.handlers.TimedRotatingFileHandler(
    filename=os.path.join(log_dir, 'rtcm_server.log'),
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
BASE_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
TCP_HOST = "0.0.0.0"
TCP_PORT = 6001
RTCM_LOG_DIR = os.path.join(log_dir, "rtcm")
RTCM_LOG_INTERVAL = 3600
RTCM_LOG_RETENTION = 24
RTCM_LOG_PREFIX = "rtcm"
RATE_REPORT_INTERVAL = 15

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
                crc ^= 0x1864CFB
        crc &= 0xFFFFFF
    return crc

def decode_1005(data):
    """Decode RTCM 1005 message to extract base station coordinates."""
    try:
        if len(data) < 19:
            return None
        payload = data[3:-3]
        ref_station_id = (struct.unpack('>H', payload[0:2])[0] & 0xFFF)
        ecef_x = struct.unpack('>i', payload[2:6])[0] / 100.0
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

def open_serial():
    """Attempts to open the serial port."""
    try:
        ser = serial.Serial(BASE_PORT, BAUD_RATE, timeout=1)
        logger.info(f"Successfully opened {BASE_PORT}")
        return ser
    except serial.SerialException as e:
        logger.error(f"Error opening {BASE_PORT}: {e}")
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

    decoded_data = None
    if msg_type == 1005:
        decoded_data = decode_1005(message)
        if decoded_data:
            logger.info(f"1005 Decoded: RefStationID={decoded_data['ref_station_id']}, "
                       f"ECEF_X={decoded_data['ecef_x']:.3f}m, "
                       f"ECEF_Y={decoded_data['ecef_y']:.3f}m, "
                       f"ECEF_Z={decoded_data['ecef_z']:.3f}m")

    return msg_type, length, new_pos, decoded_data

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
        os.remove(filename)
        logger.info(f"Compressed RTCM log: {compressed_file}")
        return compressed_file
    except Exception as e:
        logger.error(f"Error compressing {filename}: {e}")
        return filename

def broadcast_rtcm():
    """Reads RTCM data from the base station and broadcasts it to all clients."""
    global base

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
    crc_errors = 0
    last_rate_check = time.time()

    try:
        while True:
            data = base.read(2048)
            if data:
                rtcm_buffer += data
                rtcm_log.write(data)
                rtcm_log.flush()

                current_time = time.time()
                if current_time - last_rotation >= RTCM_LOG_INTERVAL:
                    rtcm_log.close()
                    compress_rtcm_log(rtcm_log_filename)
                    manage_rtcm_logs()
                    rtcm_log_filename = os.path.join(RTCM_LOG_DIR, f"{RTCM_LOG_PREFIX}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
                    rtcm_log = open(rtcm_log_filename, "wb")
                    last_rotation = current_time
                    logger.info(f"Rotated RTCM log to: {rtcm_log_filename}")

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
                        logger.info(f"  CRC Errors: {crc_errors / elapsed:.2f} Hz ({crc_errors} errors)")
                        crc_errors = 0
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

try:
    while True:
        threading.Event().wait(1)
except KeyboardInterrupt:
    logger.info("Exiting...")