#!/usr/bin/env python3
"""
field_test_logger_20260622.py
==============================
Field test data logger for tractor manual drive sessions.

Listens on:
  UDP 6002 - GPS/RTK state (from rtcm_server)
  UDP 6003 - Teensy/system status (from teensy_serial_bridge)

Merges latest values from both sources by wall-clock timestamp.
Writes one CSV row per 6003 broadcast (~5 Hz) when Teensy bridge is running.

CHANGED 20260622: GPS-only mode — if UDP 6003 is silent for GPS_ONLY_TIMEOUT
seconds (e.g. no Teensy connected), rows are driven by UDP 6002 instead.
Teensy/steering/radio columns will be empty in GPS-only rows.

CSV is TimescaleDB-ready:
  - 'time' column is ISO-8601 UTC (hypertable partition key)
  - All other columns are plain numerics or short strings
  - Ingest with: \\COPY field_test FROM 'file.csv' CSV HEADER

Usage:
  python3 field_test_logger_20260622.py
  python3 field_test_logger_20260622.py --output /home/al/field_logs/run2.csv

Output file auto-named by datetime if --output not specified:
  /home/al/field_logs/field_test_20260622_143022.csv
"""

import argparse
import csv
import json
import os
import select
import signal
import socket
import sys
import threading
import time
from datetime import datetime, timezone

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
UDP_GPS_PORT     = 6002   # from rtcm_server
UDP_STATUS_PORT  = 6003   # from teensy_serial_bridge
LOG_DIR          = "/home/al/field_logs"
LOG_HZ           = 5      # rows per second (driven by 6003 or 6002 in GPS-only mode)

# CHANGED 20260622: if 6003 is silent longer than this, switch to GPS-only mode
GPS_ONLY_TIMEOUT = 2.0    # seconds

# CSV column order - 'time' first for TimescaleDB hypertable key
CSV_COLUMNS = [
    "time",               # ISO-8601 UTC wall clock
    "elapsed_sec",        # seconds since logging started
    # --- Transmission ---
    "bucket",             # 0-9 transmission bucket
    "jrk_target",         # JRK commanded position
    "jrk_current",        # JRK feedback position
    "trans_mode",         # 0=pause 1=manual 2=auto
    # --- Steering ---
    "steer_setpoint",     # pot counts commanded
    "steer_current",      # pot counts actual
    "steer_error",        # setpoint - current
    "steer_pwm",          # IBT-2 PWM value
    "steer_mode",         # steering mode (mirrors trans_mode)
    # --- GPS ---
    "lat",                # decimal degrees
    "lon",                # decimal degrees
    "fix_quality",        # e.g. "RTK Fixed"
    "heading_deg",        # degrees from north
    "head_valid",         # bool: heading valid flag
    "carrier",            # "fixed" / "float" / "none"
    "speed_mps",          # ground speed m/s (from VTG)
    # --- Radio ---
    "radio_signal",       # "GOOD" / "UNKNOWN"
    "ack_rate",           # ACK packets/sec
    # --- Notes ---
    "notes",              # empty; fill in post-processing if needed
]

# ---------------------------------------------------------------------------
# Shared state (updated by listener threads, read by logger thread)
# ---------------------------------------------------------------------------
latest_gps    = {}
latest_status = {}
state_lock    = threading.Lock()
running       = True
last_6003_time = 0.0      # CHANGED 20260622: track when 6003 was last received

# ---------------------------------------------------------------------------
# UDP listener threads
# ---------------------------------------------------------------------------

def gps_listener():
    """Listen on 6002, update latest_gps.

    CHANGED 20260622: also sets _new_row flag to drive logger when 6003 is
    silent (GPS-only mode).  No separate thread needed — single socket handles both.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.bind(('', UDP_GPS_PORT))
    sock.setblocking(False)
    print(f"[GPS listener] bound to UDP {UDP_GPS_PORT}")
    while running:
        ready = select.select([sock], [], [], 0.2)
        if ready[0]:
            try:
                data, _ = sock.recvfrom(2048)
                parsed = json.loads(data.decode())
                with state_lock:
                    latest_gps.update(parsed)
                    # CHANGED 20260622: drive logger from GPS when Teensy bridge absent
                    if time.time() - last_6003_time > GPS_ONLY_TIMEOUT:
                        latest_status['_new_row'] = True
            except Exception as e:
                print(f"[GPS listener] error: {e}")
    sock.close()


def status_listener():
    """Listen on 6003, update latest_status. Also signals the logger thread."""
    global last_6003_time    # CHANGED 20260622
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', UDP_STATUS_PORT))
    sock.setblocking(False)
    print(f"[Status listener] bound to UDP {UDP_STATUS_PORT}")
    while running:
        ready = select.select([sock], [], [], 0.2)
        if ready[0]:
            try:
                data, _ = sock.recvfrom(4096)
                parsed = json.loads(data.decode())
                with state_lock:
                    latest_status.update(parsed)
                    latest_status['_new_row'] = True
                    last_6003_time = time.time()   # CHANGED 20260622
            except Exception as e:
                print(f"[Status listener] error: {e}")
    sock.close()

# ---------------------------------------------------------------------------
# Row builder
# ---------------------------------------------------------------------------

def build_row(start_time: float) -> dict:
    """Merge latest GPS + status into one flat CSV row."""
    now = datetime.now(timezone.utc)
    elapsed = time.time() - start_time

    with state_lock:
        gps    = dict(latest_gps)
        status = dict(latest_status)

    steer = status.get('steering', {})
    trans = status.get('transmission', {})
    radio = status.get('radio', {})

    row = {
        "time":          now.isoformat(timespec='milliseconds'),
        "elapsed_sec":   round(elapsed, 2),

        # Transmission
        "bucket":        trans.get('bucket', ''),
        "jrk_target":    trans.get('target', ''),
        "jrk_current":   trans.get('current', ''),
        "trans_mode":    trans.get('mode', ''),

        # Steering
        "steer_setpoint": steer.get('setpoint', ''),
        "steer_current":  steer.get('current', ''),
        "steer_error":    steer.get('error', ''),
        "steer_pwm":      steer.get('pwm', ''),
        "steer_mode":     steer.get('mode', ''),

        # GPS
        "lat":           gps.get('lat', ''),
        "lon":           gps.get('lon', ''),
        "fix_quality":   gps.get('fix_quality', ''),
        "heading_deg":   gps.get('heading_deg', ''),
        "head_valid":    gps.get('headValid', ''),
        "carrier":       gps.get('carrier', ''),
        "speed_mps":     gps.get('speed_mps', ''),

        # Radio
        "radio_signal":  radio.get('signal', ''),
        "ack_rate":      radio.get('ack_rate', ''),

        "notes": '',
    }
    return row

# ---------------------------------------------------------------------------
# Main logger loop
# ---------------------------------------------------------------------------

def run_logger(output_path: str):
    global running

    os.makedirs(LOG_DIR, exist_ok=True)

    print(f"\n{'='*55}")
    print(f"  Field Test Logger  |  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Output: {output_path}")
    print(f"  Rate:   ~{LOG_HZ} Hz (6003 when Teensy present, else 6002 GPS-only)")  # CHANGED
    print(f"  Ctrl+C to stop and close file cleanly")
    print(f"{'='*55}\n")

    start_time = time.time()
    row_count  = 0
    last_print = time.time()

    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        csvfile.flush()

        try:
            while running:
                # Wait for a new packet (_new_row set by either listener)
                time.sleep(1.0 / (LOG_HZ * 4))   # poll at 4x log rate

                with state_lock:
                    new_row = latest_status.pop('_new_row', False)

                if not new_row:
                    continue

                row = build_row(start_time)
                writer.writerow(row)
                row_count += 1

                # Flush every 10 rows so data survives a crash
                if row_count % 10 == 0:
                    csvfile.flush()

                # Console status every 5 seconds
                now = time.time()
                if now - last_print >= 5.0:
                    fix   = row.get('fix_quality', 'N/A')
                    speed = row.get('speed_mps', 'N/A')
                    bkt   = row.get('bucket', 'N/A')
                    sig   = row.get('radio_signal', 'N/A')
                    # CHANGED 20260622: show GPS-only vs full mode
                    mode_str = "GPS-only" if time.time() - last_6003_time > GPS_ONLY_TIMEOUT else "full"
                    print(f"  [{row['elapsed_sec']:7.1f}s]  rows={row_count:5d}  "
                          f"fix={fix:<12}  speed={speed} m/s  "
                          f"bucket={bkt}  radio={sig}  mode={mode_str}")

                    last_print = now

        except KeyboardInterrupt:
            pass

    print(f"\nLogger stopped. {row_count} rows written to:")
    print(f"  {output_path}")
    print(f"\nTimescaleDB ingest (once table exists):")
    print(f"  \\COPY field_test FROM '{output_path}' CSV HEADER;")

# ---------------------------------------------------------------------------
# Signal handler
# ---------------------------------------------------------------------------

def handle_signal(sig, frame):
    global running
    print("\n[Logger] Caught signal - shutting down cleanly...")
    running = False

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    global running

    parser = argparse.ArgumentParser(description="Tractor field test data logger")
    parser.add_argument(
        '--output', '-o',
        default=None,
        help='Output CSV path (default: auto-named in /home/al/field_logs/)'
    )
    args = parser.parse_args()

    if args.output:
        output_path = args.output
    else:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_path = os.path.join(LOG_DIR, f"field_test_{ts}.csv")

    signal.signal(signal.SIGINT,  handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    # Start listener threads
    t_gps    = threading.Thread(target=gps_listener,    daemon=True)
    t_status = threading.Thread(target=status_listener, daemon=True)
    t_gps.start()
    t_status.start()

    # CHANGED 20260622: announce GPS-only mode if 6003 silent at startup
    time.sleep(0.5)
    print("[Logger] Waiting for data... (GPS-only mode active until Teensy bridge seen)")

    run_logger(output_path)

    running = False
    t_gps.join(timeout=1)
    t_status.join(timeout=1)


if __name__ == "__main__":
    main()
