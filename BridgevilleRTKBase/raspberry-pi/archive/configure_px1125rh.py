"""
configure_px1125rh.py
=======================
This script configures a PX1125R base station (on /dev/ttyUSB0) with specified coordinates
and RTCM3 MSM4 message output for RTK operation.

Usage:
    python3 configure_px1125rh.py [--latitude LAT] [--longitude LON] [--altitude ALT] [--ephemeris-interval SEC]
"""

import serial
import time
import struct
import logging
import argparse
from datetime import datetime
import os

# Configure logging
LOG_DIR = "logs"
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(f"{LOG_DIR}/configure_px1125r_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"),
        logging.StreamHandler()
    ]
)

# Configuration
PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

def compute_checksum(payload):
    """Computes the checksum for a SkyTraq command payload."""
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return checksum

def configure_base_station(ser, latitude=40.7249028, longitude=-80.7283178, altitude=325.553, ephemeris_interval=80):
    """
    Configures the PX1125R as an RTK base station.

    Args:
        ser: Serial port object.
        latitude (float): Latitude in degrees (default: 40.7249028).
        longitude (float): Longitude in degrees (default: -80.7283178).
        altitude (float): Altitude in meters (default: 325.553).
        ephemeris_interval (int): Ephemeris interval in seconds (default: 80).
    """
    try:
        # Command 1: Configure RTK mode (base station)
        cmd1 = bytes.fromhex("a0 a1 00 02 6a 07 6d 0d 0a")
        ser.write(cmd1)
        ser.flush()
        time.sleep(1)
        logging.info("Sent RTK mode configuration command")

        # Command 2: Set surveyed position
        lat_bytes = struct.pack('<d', latitude)
        lon_bytes = struct.pack('<d', longitude)
        alt_bytes = struct.pack('<f', altitude)

        payload = bytearray([
            0x01, 0x01, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x1e
        ])
        payload.extend(lat_bytes)
        payload.extend(lon_bytes)
        payload.extend(alt_bytes)
        payload.extend([0x00, 0x00, 0x00, 0x00, 0x01])

        cmd2 = bytearray([0xa0, 0xa1])
        length = len(payload) + 2
        cmd2.extend([0x00, length])
        cmd2.extend([0x6a, 0x06])
        cmd2.extend(payload)
        checksum = compute_checksum(cmd2[4:])
        cmd2.extend([checksum, 0x0d, 0x0a])

        ser.write(cmd2)
        ser.flush()
        time.sleep(1)
        logging.info(f"Sent RTK base station position command: {cmd2.hex()}")

        # Command 3: Configure RTCM messages (MSM4 to match current setup)
        payload = bytearray([
            0x03,  # RTCM v3
            0x01,  # Output rate (1 Hz)
            0x02,  # MSM type (MSM4)
            0x00,  # Reserved
            0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x01,  # Enable: 1005, 1074, 1084, 1094, 1104 (off), 1114 (off), 1124
            0x00,  # Reserved
            ephemeris_interval & 0xFF,  # GPS ephemeris
            ephemeris_interval & 0xFF,  # GLONASS ephemeris
            ephemeris_interval & 0xFF,  # Galileo ephemeris
            0x00,  # Reserved
            0x00,  # NavIC ephemeris (disabled)
            ephemeris_interval & 0xFF,  # BeiDou ephemeris
            0x00,  # Reserved
            0x01   # Update to SRAM+FLASH
        ])

        cmd3 = bytearray([0xa0, 0xa1])
        length = len(payload) + 2
        cmd3.extend([0x00, length])
        cmd3.extend([0x69, 0x05])
        cmd3.extend(payload)
        checksum = compute_checksum(cmd3[4:])
        cmd3.extend([checksum, 0x0d, 0x0a])

        ser.write(cmd3)
        ser.flush()
        time.sleep(1)
        logging.info(f"Sent RTCM message configuration command: {cmd3.hex()}")

        # Read response
        response = ser.read(1024)
        if response:
            logging.info(f"Received response: {response.hex()}")
    except Exception as e:
        logging.error(f"Error configuring base station: {e}")

def main():
    parser = argparse.ArgumentParser(description="Configure PX1125R base station.")
    parser.add_argument("--latitude", type=float, default=40.7249028, help="Latitude in degrees")
    parser.add_argument("--longitude", type=float, default=-80.7283178, help="Longitude in degrees")
    parser.add_argument("--altitude", type=float, default=325.553, help="Altitude in meters")
    parser.add_argument("--ephemeris-interval", type=int, default=80, help="Ephemeris interval in seconds")
    args = parser.parse_args()

    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        logging.info(f"Connected to {PORT}")
        configure_base_station(ser, args.latitude, args.longitude, args.altitude, args.ephemeris_interval)
        ser.close()
    except Exception as e:
        logging.error(f"Error: {e}")

if __name__ == "__main__":
    main()