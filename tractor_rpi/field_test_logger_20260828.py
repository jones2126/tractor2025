#!/usr/bin/env python3
"""
field_test_logger_20260828.py
==============================
Field test data logger for tractor manual and autonomous drive sessions.

Listens on:
  UDP 6009 - dedicated GPS/RTK logging feed (from rtcm_server)
  UDP 6003 - Teensy/system status (from teensy_serial_bridge)
  TCP 6005 - GL router WiFi signal strength (from wifi_publish.sh on router)

Merges latest values from all three sources by wall-clock timestamp.
Writes one CSV row for each new 20 Hz Teensy steering sequence received in
the bridge's UDP 6003 status broadcast. Duplicate UDP copies and repeated
cached steering sequences are ignored.

CHANGED 20260622: GPS-only mode — if UDP 6003 is silent for GPS_ONLY_TIMEOUT
seconds (e.g. no Teensy connected), rows are driven by UDP 6009 instead.
Teensy/steering/radio columns will be empty in GPS-only rows.

CHANGED 20260710: Added TCP listener on port 6005 for GL router upstream WiFi
signal strength (router -> field hotspot link). This is separate from the
wlan0 wifi_ssid/rssi columns which reflect the RPi's own wlan0 interface
(RPi -> Mofi link). Both signal paths are now logged:
  wifi_ssid/rssi_dbm/signal_label    = RPi wlan0 -> Mofi
  router_wifi_ssid/rssi_dbm/signal_label = GL router -> upstream hotspot

CHANGED 20260728:
  - Original full 10 Hz steering telemetry path.
  - One row per unique Teensy steering sequence; duplicate UDP copies and
    repeated bridge cache values are suppressed.
  - Added low-level PID, PWM-channel, clamp, saturation, state, and timing
    columns from teensy_main_20260728.cpp via teensy_serial_bridge_20260728.py.

CSV is TimescaleDB-ready:
  - 'time' column is ISO-8601 UTC (hypertable partition key)
  - All other columns are plain numerics or short strings
  - Ingest with: \\COPY field_test FROM 'file.csv' CSV HEADER

Usage:
CHANGED 20260804:
  - Expect and preserve the new 20 Hz steering/PID sequence from
    teensy_main_20260804.cpp.
  - Poll the new-row handoff at 80 Hz so each 50 ms source record is consumed
    before the next one arrives.

CHANGED 20260828:
  - Added symmetric, receiver-specific satellite diagnostics for Base-Link
    and Heading F9Ps. Removed the ambiguous legacy numSV column.

  python3 field_test_logger_20260828.py
  python3 field_test_logger_20260828.py --output /home/al/field_logs/run2.csv

Output file auto-named by datetime if --output not specified:
  /home/al/field_logs/field_test_20260828_143022.csv
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
UDP_GPS_PORT     = 6009   # from rtcm_server -- dedicated logging feed, no longer shares 6002 with the live controller
UDP_STATUS_PORT  = 6003   # from teensy_serial_bridge
TCP_ROUTER_PORT  = 6005   # NEW 20260710: GL router upstream WiFi signal (from wifi_publish.sh)
LOG_DIR          = "/home/al/field_logs"
LOG_HZ           = 20     # rows per second (driven by new steering sequences on UDP 6003)

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
    "jrk_actual_target",          # target read back from the JRK
    "jrk_scaled_feedback",        # JRK feedback after configured scaling
    "jrk_integral",               # JRK PID integral accumulator
    "jrk_duty_cycle_target",      # signed desired motor duty (-600..600 nominal)
    "jrk_duty_cycle",             # signed applied motor duty (-600..600)
    "jrk_errors_halting",         # active error bitmask
    "jrk_errors_occurred",        # latched error bitmask
    "jrk_sequence",               # successful diagnostic snapshot counter
    "jrk_valid",                  # 1 when latest diagnostic read succeeded
    "jrk_read_latency_ms",        # blocking JRK diagnostic read duration
    "jrk_timeouts",               # cumulative JRK read timeouts
    "jrk_discarded_bytes",        # cumulative stale bytes discarded before reads
    "radio_transmission_raw",     # handheld transmission potentiometer value
    "trans_cmd_vel_mps",          # speed value most recently received by Teensy
    "trans_cmd_vel_age_ms",       # Teensy age of current speed command
    "trans_mode",         # 0=auto 1=manual 2=pause 9=radio loss
    # --- Steering ---
    "steer_setpoint",     # pot counts commanded
    "steer_current",      # pot counts actual
    "steer_error",        # setpoint - current
    "steer_pwm",          # IBT-2 PWM value
    "steer_mode",         # steering mode (mirrors trans_mode)
    "steer_sequence",             # Teensy steering control-cycle sequence
    "steer_teensy_timestamp_ms",  # Teensy millis() when control cycle ran
    "steer_state",                # OK / NO_CMD / NO_SIG / PAUSE / MODE_ERR
    "steer_direction",            # AL/AR etc.; direction associated with applied PWM
    "steer_left_pwm",             # actual LPWM channel value written
    "steer_right_pwm",            # actual RPWM channel value written
    "steer_normalized_command",   # Pure Pursuit normalized steer command
    "steer_pid_active",           # 1 when PID calculated during this cycle
    "steer_deadband_active",      # 1 when output suppressed by deadband
    "steer_min_pwm_clamped",      # 1 when raw output was raised to minimum PWM
    "steer_pwm_saturated",        # 1 when raw PID magnitude exceeded 255
    "steer_pid_dt_s",             # PID calculation interval, seconds
    "steer_integral_sum",         # accumulated error before Ki multiplication
    "steer_error_derivative",     # change in pot-count error per second
    "steer_p_term",               # proportional contribution
    "steer_i_term",               # integral contribution
    "steer_d_term",               # derivative contribution
    "steer_pid_output",           # signed raw PID output before PWM clamps
    "steer_cmd_age_ms",           # age of most recent cmd_vel on Teensy
    "steer_bridge_age_s",         # age of cached STEER record in bridge
    # --- GPS ---
    "lat",                # decimal degrees
    "lon",                # decimal degrees
    "fix_quality",        # e.g. "RTK Fixed"
    "heading_deg",        # degrees from north
    "head_valid",         # bool: heading valid flag
    "carrier",            # "fixed" / "float" / "none"
    "speed_mps",          # ground speed m/s (from VTG)
    "base_numSV_used",          # Base-Link NAV-PVT/NAV-SAT satellites used
    "base_numSV_visible",       # Base-Link NAV-SAT satellite records
    "base_cno_mean_dbhz",       # Base-Link mean positive NAV-SAT C/N0
    "base_cno_min_dbhz",
    "base_cno_max_dbhz",
    "base_numSV_used_gga",      # independent GGA cross-check
    "base_satellite_timestamp", # time of latest Base-Link PVT/SAT response
    "heading_numSV_used",       # Heading NAV-PVT/NAV-SAT satellites used
    "heading_numSV_visible",    # Heading NAV-SAT satellite records
    "heading_cno_mean_dbhz",    # Heading mean positive NAV-SAT C/N0
    "heading_cno_min_dbhz",
    "heading_cno_max_dbhz",
    "heading_satellite_timestamp", # time of latest Heading PVT/SAT response
    "hdop",                # NEW 20260717: horizontal dilution of precision          # <- new line
    "diff_age",            # NEW 20260717: age of RTCM corrections, seconds          # <- new line
    "wifi_ssid",           # RPi wlan0 SSID (RPi -> Mofi link)
    "wifi_rssi_dbm",       # RPi wlan0 signal strength dBm
    "wifi_signal_label",   # "strong"/"medium"/"weak"/"unknown"
    # --- GL Router upstream WiFi (router -> field hotspot link) ---  NEW 20260710
    "router_wifi_ssid",         # NEW 20260710: SSID GL router is connected to
    "router_wifi_rssi_dbm",     # NEW 20260710: GL router upstream signal dBm
    "router_wifi_signal_label", # NEW 20260710: "strong"/"medium"/"weak"
    # --- Radio ---
    "radio_signal",       # "GOOD" / "UNKNOWN"
    "ack_rate",           # ACK packets/sec
    # --- Notes ---
    "notes",              # empty; fill in post-processing if needed
]

# ---------------------------------------------------------------------------
# Shared state (updated by listener threads, read by logger thread)
# ---------------------------------------------------------------------------
latest_gps         = {}
latest_status      = {}
latest_router_wifi = {}   # NEW 20260710: data from GL router TCP port 6005
state_lock    = threading.Lock()
running       = True
last_6003_time = 0.0      # CHANGED 20260622: track when 6003 was last received
last_steering_sequence = None  # NEW 20260728: suppress duplicate/cached UDP rows

# ---------------------------------------------------------------------------
# Status merge / deduplication
# ---------------------------------------------------------------------------

def merge_status_packet(parsed: dict) -> bool:
    """Merge one bridge packet and return True only for a loggable new row.

    The bridge transmits each UDP status packet twice. A July 28 steering
    sequence identifies both copies and also identifies a cached sequence
    repeated on a later bridge cycle. Older packets without a sequence retain
    the legacy packet-driven behavior.
    """
    global last_6003_time, last_steering_sequence

    with state_lock:
        steering = parsed.get('steering', {})
        sequence = steering.get('sequence')
        latest_status.update(parsed)

        is_new_row = (
            sequence is None or
            sequence != last_steering_sequence
        )

        if is_new_row:
            latest_status['_new_row'] = True
            if sequence is not None:
                last_steering_sequence = sequence

        last_6003_time = time.time()

    return is_new_row

# ---------------------------------------------------------------------------
# UDP listener threads
# ---------------------------------------------------------------------------

def gps_listener():
    """Listen on the dedicated UDP 6009 logging feed, update latest_gps.

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
    """Listen on 6003 and signal one row per new steering sequence."""
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
                merge_status_packet(parsed)
            except Exception as e:
                print(f"[Status listener] error: {e}")
    sock.close()

# NEW 20260710: TCP listener for GL router upstream WiFi signal strength
def router_wifi_listener():
    """Listen on TCP 6005 for WiFi signal data pushed by GL router wifi_publish.sh.

    The GL router connects to the field hotspot (upstream) and reports that
    link's SSID and RSSI here. This is distinct from the RPi's own wlan0
    signal which reflects the RPi->Mofi link.
    """
    while running:
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('', TCP_ROUTER_PORT))
            server.listen(1)
            server.settimeout(2.0)
            print(f"[Router WiFi listener] bound to TCP {TCP_ROUTER_PORT}")
            while running:
                try:
                    conn, addr = server.accept()
                    conn.settimeout(2.0)
                    with conn:
                        data = conn.recv(1024)
                        if data:
                            parsed = json.loads(data.decode())
                            with state_lock:
                                latest_router_wifi.update(parsed)
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"[Router WiFi listener] connection error: {e}")
        except Exception as e:
            print(f"[Router WiFi listener] socket error: {e}, retrying in 5s")
            time.sleep(5)
        finally:
            try:
                server.close()
            except Exception:
                pass

# ---------------------------------------------------------------------------
# Row builder
# ---------------------------------------------------------------------------

def build_row(start_time: float) -> dict:
    """Merge latest GPS + status into one flat CSV row."""
    now = datetime.now(timezone.utc)
    elapsed = time.time() - start_time

    with state_lock:
        gps         = dict(latest_gps)
        status      = dict(latest_status)
        router_wifi = dict(latest_router_wifi)   # NEW 20260710

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
        "jrk_actual_target":       trans.get('actual_target', ''),
        "jrk_scaled_feedback":     trans.get('scaled_feedback', ''),
        "jrk_integral":            trans.get('integral', ''),
        "jrk_duty_cycle_target":   trans.get('duty_cycle_target', ''),
        "jrk_duty_cycle":          trans.get('duty_cycle', ''),
        "jrk_errors_halting":      trans.get('errors_halting', ''),
        "jrk_errors_occurred":     trans.get('errors_occurred', ''),
        "jrk_sequence":            trans.get('jrk_sequence', ''),
        "jrk_valid":               trans.get('jrk_valid', ''),
        "jrk_read_latency_ms":     trans.get('jrk_read_latency_ms', ''),
        "jrk_timeouts":            trans.get('jrk_timeouts', ''),
        "jrk_discarded_bytes":     trans.get('jrk_discarded_bytes', ''),
        "radio_transmission_raw":  trans.get('radio_transmission_raw', ''),
        "trans_cmd_vel_mps":       trans.get('cmd_vel_mps', ''),
        "trans_cmd_vel_age_ms":    trans.get('cmd_vel_age_ms', ''),
        "trans_mode":    trans.get('mode', ''),

        # Steering
        "steer_setpoint": steer.get('setpoint', ''),
        "steer_current":  steer.get('current', ''),
        "steer_error":    steer.get('error', ''),
        "steer_pwm":      steer.get('pwm', ''),
        "steer_mode":     steer.get('mode', ''),
        "steer_sequence":             steer.get('sequence', ''),
        "steer_teensy_timestamp_ms":  steer.get('teensy_timestamp_ms', ''),
        "steer_state":                steer.get('state', ''),
        "steer_direction":            steer.get('direction', ''),
        "steer_left_pwm":             steer.get('left_pwm', ''),
        "steer_right_pwm":            steer.get('right_pwm', ''),
        "steer_normalized_command":   steer.get('normalized_command', ''),
        "steer_pid_active":           steer.get('pid_active', ''),
        "steer_deadband_active":      steer.get('deadband_active', ''),
        "steer_min_pwm_clamped":      steer.get('min_pwm_clamped', ''),
        "steer_pwm_saturated":        steer.get('pwm_saturated', ''),
        "steer_pid_dt_s":             steer.get('pid_dt_s', ''),
        "steer_integral_sum":         steer.get('integral_sum', ''),
        "steer_error_derivative":     steer.get('error_derivative', ''),
        "steer_p_term":               steer.get('p_term', ''),
        "steer_i_term":               steer.get('i_term', ''),
        "steer_d_term":               steer.get('d_term', ''),
        "steer_pid_output":           steer.get('pid_output', ''),
        "steer_cmd_age_ms":           steer.get('cmd_age_ms', ''),
        "steer_bridge_age_s":         steer.get('age', ''),

        # GPS
        "lat":           gps.get('lat', ''),
        "lon":           gps.get('lon', ''),
        "fix_quality":   gps.get('fix_quality', ''),
        "heading_deg":   gps.get('heading_deg', ''),
        "head_valid":    gps.get('headValid', ''),
        "carrier":       gps.get('carrier', ''),
        "speed_mps":     gps.get('speed_mps', ''),
        "base_numSV_used": gps.get('base_numSV_used', ''),
        "base_numSV_visible": gps.get('base_numSV_visible', ''),
        "base_cno_mean_dbhz": gps.get('base_cno_mean_dbhz', ''),
        "base_cno_min_dbhz": gps.get('base_cno_min_dbhz', ''),
        "base_cno_max_dbhz": gps.get('base_cno_max_dbhz', ''),
        "base_numSV_used_gga": gps.get('base_numSV_used_gga', ''),
        "base_satellite_timestamp": gps.get('base_satellite_timestamp', ''),
        "heading_numSV_used": gps.get('heading_numSV_used', ''),
        "heading_numSV_visible": gps.get('heading_numSV_visible', ''),
        "heading_cno_mean_dbhz": gps.get('heading_cno_mean_dbhz', ''),
        "heading_cno_min_dbhz": gps.get('heading_cno_min_dbhz', ''),
        "heading_cno_max_dbhz": gps.get('heading_cno_max_dbhz', ''),
        "heading_satellite_timestamp": gps.get('heading_satellite_timestamp', ''),
        "hdop":          gps.get('hdop', ''),          # <- new line
        "diff_age":      gps.get('diff_age', ''),      # <- new line
        "wifi_ssid":     gps.get('wifi_ssid', ''),
        "wifi_rssi_dbm":     gps.get('wifi_rssi_dbm', ''),
        "wifi_signal_label": gps.get('wifi_signal_label', ''),

        # GL router upstream WiFi (router -> field hotspot)  NEW 20260710
        "router_wifi_ssid":         router_wifi.get('wifi_ssid', ''),
        "router_wifi_rssi_dbm":     router_wifi.get('wifi_rssi_dbm', ''),
        "router_wifi_signal_label": router_wifi.get('wifi_signal_label', ''),

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
    print(f"  Rate:   ~{LOG_HZ} Hz (6003 when Teensy present, else 6009 GPS-only)")  # CHANGED
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
    t_gps    = threading.Thread(target=gps_listener,         daemon=True)
    t_status = threading.Thread(target=status_listener,      daemon=True)
    t_router = threading.Thread(target=router_wifi_listener, daemon=True)  # NEW 20260710
    t_gps.start()
    t_status.start()
    t_router.start()   # NEW 20260710

    # CHANGED 20260622: announce GPS-only mode if 6003 silent at startup
    time.sleep(0.5)
    print("[Logger] Waiting for data... (GPS-only mode active until Teensy bridge seen)")

    run_logger(output_path)

    running = False
    t_gps.join(timeout=1)
    t_status.join(timeout=1)
    t_router.join(timeout=1)   # NEW 20260710


if __name__ == "__main__":
    main()
