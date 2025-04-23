"""
rtcm_tcp_server.py
=======================
This script runs on a Raspberry Pi 3 Model B to read RTCM data from a base station (PX1125R on /dev/ttyUSB0)
and stream it over TCP to connected clients. It logs the RTCM data to a file.

Usage:
    python3 rtcm_tcp_server.py
"""

import socket
import serial
import threading
from datetime import datetime
import os

# Configuration
BASE_PORT = "/dev/ttyUSB0"  # Confirmed by your output
BAUD_RATE = 115200
TCP_HOST = "0.0.0.0"  # Listen on all interfaces
TCP_PORT = 5000  # Port for TCP server
LOG_DIR = "logs"

# Shared variables
base = None
clients = []

def open_serial():
    """Attempts to open the serial port."""
    try:
        ser = serial.Serial(BASE_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully opened {BASE_PORT}.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening {BASE_PORT}: {e}")
        return None

def handle_client(client_socket, address):
    """Handles a single TCP client connection."""
    print(f"Client connected: {address}")
    clients.append(client_socket)
    try:
        while True:
            # Keep the connection alive, data is broadcast from the main thread
            client_socket.sendall(b"")  # Keep-alive
            threading.Event().wait(1)  # Sleep to avoid busy loop
    except Exception as e:
        print(f"Client {address} disconnected: {e}")
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
        print("Failed to open base port. Exiting.")
        rtcm_log.close()
        return

    print(f"Reading RTCM data from {BASE_PORT} and broadcasting to clients...")
    try:
        while True:
            data = base.read(1024)  # Read RTCM data in chunks
            if data:
                rtcm_log.write(data)  # Log RTCM data
                rtcm_log.flush()
                # Broadcast to all connected clients
                for client in clients[:]:  # Copy list to avoid issues if modified
                    try:
                        client.sendall(data)
                    except Exception as e:
                        print(f"Error sending to client: {e}")
                        clients.remove(client)
                        client.close()
    except Exception as e:
        print(f"Error in RTCM broadcasting: {e}")
    finally:
        base.close()
        rtcm_log.close()

def start_tcp_server():
    """Starts the TCP server to accept client connections."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(5)
    print(f"TCP server started on {TCP_HOST}:{TCP_PORT}")

    try:
        while True:
            client_socket, address = server.accept()
            client_thread = threading.Thread(target=handle_client, args=(client_socket, address), daemon=True)
            client_thread.start()
    except Exception as e:
        print(f"TCP server error: {e}")
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
    print("\nExiting...")