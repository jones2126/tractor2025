"""
forward_rtcm_rpi.py
===================
This script forwards RTCM data from a pre-configured base station (PX1125R on /dev/ttyUSB0) to a rover (PX1172RDP on /dev/ttyUSB1)
and monitors NMEA output to check RTK status. Optimized for Raspberry Pi (Linux).

**Important**: Power cycle the PX1125R base station and run enable_base_mode.py to configure it before running this script.
The base should be set to output RTCM MSM4 with surveyed position (40.3453810°N, -80.1287566°W, 323.882m).
Run with sudo if serial port access is denied (e.g., sudo python3 forward_rtcm_rpi.py).

Usage:
    python3 forward_rtcm_rpi.py
"""

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
log_file = f"{LOG_DIR}/forward_rtcm_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)

# Assigning serial ports
BASE_PORT = "/dev/ttyUSB0"  # PX1125R (Base station)
ROVER_PORT = "/dev/ttyUSB1"  # PX1172RDP (Rover)
BAUD_RATE = 115200
STATUS_INTERVAL = 5
GGA_INTERVAL = 1

# Shared variables
rover = None
rover_ready = threading.Event()
start_time = 0
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
    "avg_hdop_fixed": []
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

def open_serial(port):
    """Attempts to open a serial port."""
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        logging.info(f"Successfully opened {port}.")
        return ser
    except serial.SerialException as e:
        logging.error(f"Error opening {port}: {e}")
        logging.error("Ensure ports are correct and run with sudo if needed (e.g., sudo python3 forward_rtcm_rpi.py).")
        return None

def forward_rtcm():
    """Reads RTCM data from Base and forwards it to Rover."""
    global rover

    rtcm_log = open(f"{LOG_DIR}/rtcm_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log", "wb")

    rover = open_serial(ROVER_PORT)
    if not rover:
        logging.error("Failed to open rover port. Exiting.")
        rtcm_log.close()
        return

    base = open_serial(BASE_PORT)
    if not base:
        logging.error("Failed to open base port. Exiting.")
        rover.close()
        rtcm_log.close()
        return

    rover_ready.set()
    logging.info(f"Forwarding RTCM data from {BASE_PORT} to {ROVER_PORT}...")
    try:
        while True:
            data = base.read(4096)
            if data:
                rover.write(data)
                rtcm_log.write(data)
                rtcm_log.flush()
    except Exception as e:
        logging.error(f"Error in RTCM forwarding: {e}")
    finally:
        base.close()
        rover.close()
        rtcm_log.close()

def monitor_nmea():
    """Reads NMEA output from Rover and monitors RTK status."""
    nmea_log = open(f"{LOG_DIR}/nmea_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log", "w")

    logging.info(f"Waiting for Rover ({ROVER_PORT}) to be ready...")
    rover_ready.wait()

    logging.info(f"Listening to NMEA data from {ROVER_PORT}...")
    last_status_time = time.time()
    last_gga_time = time.time()
    last_gga_message = None
    first_rtk_fix_time = None

    try:
        while True:
            if not rover.is_open:
                logging.error("Rover port closed unexpectedly.")
                break
            line = rover.readline().decode(errors='ignore').strip()
            if not line:
                continue

            nmea_log.write(f"[{datetime.now().strftime('%H:%M:%S')}] {line}\n")
            nmea_log.flush()

            if line.startswith("$GPGGA"):
                last_gga_message = line
                fields = line.split(',')
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

                if fix_type != "4" and current_time - last_gga_time >= GGA_INTERVAL:
                    logging.info(f"GGA (Status not RTK Fixed): {last_gga_message}")
                    last_gga_time = current_time

                if current_time - last_status_time >= STATUS_INTERVAL:
                    num_satellites = fields[7]
                    altitude = fields[9]
                    hdop = fields[8]
                    status = {
                        "0": "No Fix",
                        "1": "GPS Fix",
                        "2": "DGPS Fix",
                        "4": "RTK Fixed",
                        "5": "RTK Float"
                    }.get(fix_type, "Unknown")
                    logging.info(f"RTK Status: {status} | Satellites: {num_satellites} | Altitude: {altitude}m")
                    logging.info(f"Last GGA Message: {last_gga_message}")

                    if fix_type == "4":
                        try:
                            fix_stats["avg_satellites_fixed"].append(int(num_satellites))
                            fix_stats["avg_hdop_fixed"].append(float(hdop))
                        except ValueError:
                            pass

                    if fix_type in ["4", "5"] and first_rtk_fix_time is None:
                        first_rtk_fix_time = time.time()
                        elapsed = first_rtk_fix_time - start_time
                        logging.info(f"First RTK Fix achieved after {elapsed:.1f} seconds!")

                    last_status_time = current_time

    except Exception as e:
        logging.error(f"Error in NMEA monitoring: {e}")
    finally:
        nmea_log.close()

if __name__ == "__main__":
    start_time = time.time()
    signal.signal(signal.SIGINT, signal_handler)
    rtcm_thread = threading.Thread(target=forward_rtcm, daemon=True)
    nmea_thread = threading.Thread(target=monitor_nmea, daemon=True)

    rtcm_thread.start()
    nmea_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Exiting...")
        log_fix_statistics()