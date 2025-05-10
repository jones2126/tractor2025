"""
fetch_rtk2go_rpi.py
===================
This script fetches RTCM correction data from an NTRIP server (RTK2GO) and sends it to a rover (PX1172RDP on /dev/ttyUSB1),
monitoring NMEA output to track RTK status. Optimized for Raspberry Pi (Linux).

**Important**: Ensure the rover is connected to /dev/ttyUSB1 and powered on.
Run with sudo if serial port access is denied (e.g., sudo python3 fetch_rtk2go_rpi.py).

Usage:
    python3 fetch_rtk2go_rpi.py
"""

import socket
import base64
import serial
import threading
import time
from datetime import datetime
import os
import logging
import signal
import sys

# Configure logging
LOG_DIR = "logs"
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)
log_file = f"{LOG_DIR}/fetch_rtcm_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)

# NTRIP server details
NTRIP_SERVER = "rtk2go.com"
NTRIP_PORT = 2101
MOUNTPOINT = "cmuairlab01"
USERNAME = "aej2126l@protonmail.com"
PASSWORD = "none"

# Serial port details
SERIAL_PORT = "/dev/ttyUSB1"  # PX1172RDP Rover
BAUDRATE = 115200

# Shared variables
start_time = 0
gps_serial = None
fix_stats = {
    "fixed_time": 0,
    "float_time": 0,
    "no_fix_time": 0,
    "last_fix_type": None,
    "last_change_time": time.time(),
    "drop_count": 0,
    "fixed_periods": [],
    "float_periods": [],
    "avg_satellites_fixed": [],
    "avg_hdop_fixed": [],
    "first_float_time": None,
    "first_fixed_time": None
}

def signal_handler(sig, frame):
    """Handle Ctrl+C and log fix statistics."""
    logging.info("Exiting...")
    log_fix_statistics()
    sys.exit(0)

def log_fix_statistics():
    """Log RTK fix consistency statistics."""
    total_time = time.time() - start_time
    if total_time <= 0:
        logging.info("No runtime data available.")
        return

    fixed_percent = (fix_stats["fixed_time"] / total_time) * 100
    float_percent = (fix_stats["float_time"] / total_time) * 100
    no_fix_percent = (fix_stats["no_fix_time"] / total_time) * 100
    avg_satellites = sum(fix_stats["avg_satellites_fixed"]) / len(fix_stats["avg_satellites_fixed"]) if fix_stats["avg_satellites_fixed"] else 0
    avg_hdop = sum(fix_stats["avg_hdop_fixed"]) / len(fix_stats["avg_hdop_fixed"]) if fix_stats["avg_hdop_fixed"] else 0

    logging.info("=== RTK Fix Consistency Summary ===")
    logging.info(f"Total Runtime: {total_time:.1f}s")
    if fix_stats["first_float_time"]:
        logging.info(f"First RTK Float: {fix_stats['first_float_time']:.1f}s")
    if fix_stats["first_fixed_time"]:
        logging.info(f"First RTK Fixed: {fix_stats['first_fixed_time']:.1f}s")
    logging.info(f"RTK Fixed: {fix_stats['fixed_time']:.1f}s ({fixed_percent:.2f}%)")
    logging.info(f"RTK Float: {fix_stats['float_time']:.1f}s ({float_percent:.2f}%)")
    logging.info(f"No Fix: {fix_stats['no_fix_time']:.1f}s ({no_fix_percent:.2f}%)")
    logging.info(f"Number of Drops to Float/No Fix: {fix_stats['drop_count']}")
    logging.info(f"Fixed Periods: {len(fix_stats['fixed_periods'])}")
    for i, period in enumerate(fix_stats['fixed_periods'], 1):
        logging.info(f"  Period {i}: {period['start']} to {period['end']} ({period['duration']:.1f}s)")
    logging.info(f"Float Periods: {len(fix_stats['float_periods'])}")
    for i, period in enumerate(fix_stats['float_periods'], 1):
        logging.info(f"  Period {i}: {period['start']} to {period['end']} ({period['duration']:.1f}s)")
    logging.info(f"Average Satellites (Fixed): {avg_satellites:.1f}")
    logging.info(f"Average HDOP (Fixed): {avg_hdop:.2f}")

def check_source_table():
    """Get the NTRIP source table to verify mountpoint."""
    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(10)
        logging.info(f"Connecting to {NTRIP_SERVER}:{NTRIP_PORT} for source table...")
        client.connect((NTRIP_SERVER, NTRIP_PORT))
        logging.info(f"Connected to {NTRIP_SERVER}:{NTRIP_PORT}")

        auth_string = f"{USERNAME}:{PASSWORD}"
        auth = base64.b64encode(auth_string.encode()).decode()
        headers = (
            f"GET / HTTP/1.0\r\n"
            f"User-Agent: NTRIP GNSSViewer/1.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            f"\r\n"
        )
        client.send(headers.encode())
        logging.info("Sent source table request")

        response = b""
        while True:
            try:
                chunk = client.recv(4096)
                if not chunk:
                    break
                response += chunk
            except socket.timeout:
                break

        response_text = response.decode('ascii', errors='ignore')
        if MOUNTPOINT in response_text:
            logging.info(f"Mountpoint '{MOUNTPOINT}' found in source table!")
            return True
        else:
            logging.error(f"Mountpoint '{MOUNTPOINT}' not found in the source table.")
            return False
    except Exception as e:
        logging.error(f"Error checking source table: {e}")
        return False
    finally:
        try:
            client.close()
        except:
            pass

def parse_gpgga(line):
    """Parse GPGGA sentence and update fix_stats."""
    try:
        fields = line.split(',')
        if len(fields) < 10:
            logging.warning(f"Invalid GPGGA: {line}")
            return
        fix_type = fields[6]
        current_time = time.time()

        # Update fix statistics
        if fix_stats["last_fix_type"] != fix_type:
            duration = current_time - fix_stats["last_change_time"]
            fix_time_str = datetime.fromtimestamp(fix_stats["last_change_time"]).strftime('%Y-%m-%d %H:%M:%S')
            curr_time_str = datetime.fromtimestamp(current_time).strftime('%Y-%m-%d %H:%M:%S')
            if fix_stats["last_fix_type"] == "4":
                fix_stats["fixed_time"] += duration
                fix_stats["fixed_periods"].append({
                    "start": fix_time_str,
                    "end": curr_time_str,
                    "duration": duration
                })
            elif fix_stats["last_fix_type"] == "5":
                fix_stats["float_time"] += duration
                fix_stats["float_periods"].append({
                    "start": fix_time_str,
                    "end": curr_time_str,
                    "duration": duration
                })
            elif fix_stats["last_fix_type"] in ["0", "1", "2"]:
                fix_stats["no_fix_time"] += duration
            if fix_stats["last_fix_type"] == "4" and fix_type in ["5", "0", "1", "2"]:
                fix_stats["drop_count"] += 1
            fix_stats["last_fix_type"] = fix_type
            fix_stats["last_change_time"] = current_time

            # Log first Float and Fixed times
            if fix_type == "5" and fix_stats["first_float_time"] is None:
                fix_stats["first_float_time"] = current_time - start_time
                logging.info(f"First RTK Float achieved after {fix_stats['first_float_time']:.1f}s")
            if fix_type == "4" and fix_stats["first_fixed_time"] is None:
                fix_stats["first_fixed_time"] = current_time - start_time
                logging.info(f"First RTK Fixed achieved after {fix_stats['first_fixed_time']:.1f}s")

        # Store satellites and HDOP for Fixed periods
        if fix_type == "4":
            try:
                fix_stats["avg_satellites_fixed"].append(int(fields[7]))
                fix_stats["avg_hdop_fixed"].append(float(fields[8]))
            except ValueError:
                pass
    except Exception as e:
        logging.error(f"Error parsing GPGGA: {e}")

def read_nmea():
    """Read NMEA sentences from the GPS, parse RTK status, and log periodically."""
    global gps_serial
    last_print_time = 0

    try:
        while True:
            line = gps_serial.readline().decode('ascii', errors='ignore').strip()
            if not line:
                logging.debug("No NMEA data received")
                continue
            if line.startswith("$GPGGA"):
                logging.debug(f"Raw GPGGA: {line}")
                parse_gpgga(line)
                parts = line.split(',')
                if len(parts) >= 7:
                    try:
                        fix_type = int(parts[6]) if parts[6] else 0
                        current_time = time.time()
                        if current_time - last_print_time >= 5:
                            last_print_time = current_time
                            rtk_status = {
                                0: "No Fix",
                                1: "GPS Fix (Standard)",
                                2: "DGPS Fix",
                                4: "RTK Fixed",
                                5: "RTK Float"
                            }.get(fix_type, "Unknown")
                            logging.info(f"RTK Status: {rtk_status} | Satellites: {parts[7]} | Position: {parts[2]},{parts[3]} {parts[4]},{parts[5]} | Altitude: {parts[9]}m")
                            logging.info(f"Last GGA Message: {line}")
                    except (ValueError, IndexError) as e:
                        logging.error(f"Error parsing fix type: {e}")
    except Exception as e:
        logging.error(f"Error reading NMEA: {e}")
    finally:
        if gps_serial:
            gps_serial.close()
            logging.info("Serial port closed")

def fetch_rtcm():
    """Fetch RTCM data from NTRIP server and send to GPS."""
    logging.info("Verifying mountpoint availability...")
    if not check_source_table():
        logging.error("Exiting due to mountpoint unavailability.")
        return

    max_retries = 3
    retry_count = 0

    while retry_count < max_retries:
        client = None
        global gps_serial
        try:
            logging.info("Waiting 2 seconds before connecting...")
            time.sleep(2)
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(10)
            logging.info(f"Connecting to {NTRIP_SERVER}:{NTRIP_PORT}...")
            client.connect((NTRIP_SERVER, NTRIP_PORT))
            logging.info(f"Connected to {NTRIP_SERVER}:{NTRIP_PORT}")

            auth_string = f"{USERNAME}:{PASSWORD}"
            auth = base64.b64encode(auth_string.encode()).decode()
            headers = (
                f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
                f"User-Agent: NTRIP GNSSViewer/1.0\r\n"
                f"Authorization: Basic {auth}\r\n"
                f"\r\n"
            )
            logging.info("Sending NTRIP request headers...")
            client.send(headers.encode())

            response_buffer = b""
            while b"\r\n\r\n" not in response_buffer and len(response_buffer) < 4096:
                chunk = client.recv(1024)
                if not chunk:
                    break
                response_buffer += chunk

            header_end = response_buffer.find(b"\r\n\r\n")
            if header_end >= 0:
                header = response_buffer[:header_end+4]
                initial_data = response_buffer[header_end+4:]
                first_line = header.split(b'\r\n')[0].decode('ascii', errors='ignore')
                logging.info(f"Server response: {first_line}")
            else:
                first_line = response_buffer.split(b'\r\n')[0].decode('ascii', errors='ignore')
                logging.info(f"Server response: {first_line}")
                initial_data = b""
            if "200 OK" not in response_buffer.decode('ascii', errors='ignore'):
                response_text = response_buffer.decode('ascii', errors='ignore')
                if "401" in response_text:
                    logging.error("Authentication failed. Check username and password.")
                elif "403" in response_text:
                    logging.error("Access forbidden. No permission for this mountpoint.")
                elif "404" in response_text:
                    logging.error("Mountpoint not found. Check mountpoint name.")
                else:
                    logging.error("Unknown error. Check connection settings.")
                retry_count += 1
                if retry_count < max_retries:
                    logging.info(f"Retrying ({retry_count}/{max_retries})...")
                    time.sleep(5)
                continue

            logging.info("NTRIP server connected successfully")

            # GPGGA matching Windows (40.345354°N, -80.128727°W, 362.0m)
            gpgga = "$GPGGA,092750.000,4020.7212606,N,08007.7235976,W,1,8,1.03,362.0,M,-33.184,M,,*52\r\n"

            try:
                gps_serial = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=3)
                logging.info(f"Opened serial port {SERIAL_PORT} at {BAUDRATE} baud")
                nmea_thread = threading.Thread(target=read_nmea, args=(), daemon=True)
                nmea_thread.start()
                if initial_data:
                    gps_serial.write(initial_data)
                    logging.info(f"Sent {len(initial_data)} bytes of RTCM data to GPS")
            except Exception as e:
                logging.error(f"Error opening serial port: {e}")
                logging.error("Check GPS connection and port settings (/dev/ttyUSB1 expected).")
                break

            client.send(gpgga.encode())
            logging.info(f"Sent initial GPGGA position: {gpgga.strip()}")

            last_gpgga_time = time.time()
            last_rtcm_time = time.time()
            data_received = 0

            while True:
                current_time = time.time()
                if current_time - last_gpgga_time >= 10:
                    client.send(gpgga.encode())
                    logging.info(f"Sent GPGGA update: {gpgga.strip()}")
                    last_gpgga_time = current_time

                client.settimeout(1.0)
                try:
                    data = client.recv(1024)
                    if not data:
                        logging.error("Connection closed by server")
                        break
                    gps_serial.write(data)
                    data_received += len(data)
                    if current_time - last_rtcm_time >= 5:
                        logging.info(f"RTCM data received: {len(data)} bytes (Total: {data_received} bytes)")
                        last_rtcm_time = current_time
                except socket.timeout:
                    continue
                except Exception as e:
                    logging.error(f"Error receiving data: {e}")
                    break

            retry_count = 0
            break

        except Exception as e:
            logging.error(f"Error: {e}")
            retry_count += 1
            if retry_count < max_retries:
                logging.info(f"Retrying ({retry_count}/{max_retries})...")
                time.sleep(5)
        finally:
            if client:
                client.close()
            if gps_serial:
                gps_serial.close()
            logging.info("NTRIP connection closed")

    if retry_count >= max_retries:
        logging.error(f"Failed after {max_retries} attempts. Check your connection settings.")

if __name__ == "__main__":
    try:
        start_time = time.time()
        signal.signal(signal.SIGINT, signal_handler)
        while True:
            try:
                fetch_rtcm()
                logging.info("Connection closed. Reconnecting in 5 seconds...")
                time.sleep(5)
            except KeyboardInterrupt:
                logging.info("Exiting...")
                log_fix_statistics()
                sys.exit(0)
            except Exception as e:
                logging.error(f"Error: {e}")
                logging.info("Reconnecting in 10 seconds...")
                time.sleep(10)
    except KeyboardInterrupt:
        logging.info("Exiting...")
        log_fix_statistics()
        sys.exit(0)
    except Exception as e:
        logging.error(f"Unhandled exception: {e}")