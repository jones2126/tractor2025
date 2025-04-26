"""
rtcm_tcp_server.py
=======================
This script runs on a Raspberry Pi 3 Model B to read RTCM data from a base station (PX1125R on /dev/ttyUSB0)
and stream it over TCP to connected clients. It logs the RTCM data to a file and reports RTCM message type rates.

Usage:
    python3 rtcm_tcp_server.py
"""

import socket
import serial
import threading
from datetime import datetime
import os
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Configuration
BASE_PORT = "/dev/ttyUSB0"  # Confirmed by your output
BAUD_RATE = 115200
TCP_HOST = "0.0.0.0"  # Listen on all interfaces
TCP_PORT = 6001  # Port for TCP server
LOG_DIR = "logs"
RATE_REPORT_INTERVAL = 10  # Report RTCM message rates every 10 seconds

# Shared variables
base = None
clients = []

def open_serial():
    """Attempts to open the serial port."""
    try:
        ser = serial.Serial(BASE_PORT, BAUD_RATE, timeout=1)
        logging.info(f"Successfully opened {BASE_PORT}")
        return ser
    except serial.SerialException as e:
        logging.error(f"Error opening {BASE_PORT}: {e}")
        return None

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

def handle_client(client_socket, address):
    """Handles a single TCP client connection."""
    logging.info(f"Client connected: {address}")
    clients.append(client_socket)
    try:
        while True:
            # Keep the connection alive, data is broadcast from the main thread
            client_socket.sendall(b"")  # Keep-alive
            threading.Event().wait(1)  # Sleep to avoid busy loop
    except Exception as e:
        logging.info(f"Client {address} disconnected: {e}")
    finally:
        clients.remove(client_socket)
        client_socket.close()

def broadcast_rtcm():
    """Reads RTCM data from the base station and broadcasts it to all clients."""
    global base

    # Open log file for RTCM data
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
    rtcm_log = open(f"{LOG_DIR}/rtcm_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log", "wb")

    base = open_serial()
    if not base:
        logging.error("Failed to open base port. Exiting.")
        rtcm_log.close()
        return

    logging.info(f"Reading RTCM data from {BASE_PORT} and broadcasting to clients...")

    # Variables for RTCM rate monitoring
    rtcm_buffer = b''
    message_counts = {}
    last_rate_check = time.time()

    try:
        while True:
            data = base.read(1024)  # Read RTCM data in chunks
            if data:
                rtcm_buffer += data
                rtcm_log.write(data)  # Log RTCM data
                rtcm_log.flush()

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

                # Broadcast to all connected clients
                for client in clients[:]:  # Copy list to avoid issues if modified
                    try:
                        client.sendall(data)
                    except Exception as e:
                        logging.error(f"Error sending to client: {e}")
                        clients.remove(client)
                        client.close()

                # Report message rates every 10 seconds
                current_time = time.time()
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

    except Exception as e:
        logging.error(f"Error in RTCM broadcasting: {e}")
    finally:
        base.close()
        rtcm_log.close()

def start_tcp_server():
    """Starts the TCP server to accept client connections."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(5)
    logging.info(f"TCP server started on {TCP_HOST}:{TCP_PORT}")

    try:
        while True:
            client_socket, address = server.accept()
            client_thread = threading.Thread(target=handle_client, args=(client_socket, address), daemon=True)
            client_thread.start()
    except Exception as e:
        logging.error(f"TCP server error: {e}")
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
    logging.info("Exiting...")