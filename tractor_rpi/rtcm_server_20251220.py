#!/usr/bin/env python3
"""RTCM forwarder and GPS data server.

This script forwards RTCM correction data from a TCP source to the base F9P
receiver and simultaneously parses NMEA (GNGGA) from the base receiver to
obtain latitude, longitude and fix quality.  It also listens to the heading
F9P receiver for UBX-NAV-RELPOSNED messages to obtain heading information.

The latest navigation state is broadcast as JSON over UDP so that a separate
navigation program can consume the data without needing ROS2.  The UDP
broadcast runs at a fixed rate and contains the following fields:

    {
        "timestamp": ISO-8601 string,
        "lat": decimal degrees,
        "lon": decimal degrees,
        "fix_quality": string,            # e.g. "RTK Fixed"
        "heading_deg": degrees from north,
        "headValid": bool,
        "carrier": string,                # e.g. "float"/"fixed"
        "expectedErrDeg": float          # expected heading error (1-sigma)
    }

Adjust the configuration section below to match your hardware setup.

11/26/25 - Added code for fatal errors when the GPS units are not detected
"""

import json
import math
import re
import socket
import struct
import threading
import time
from datetime import datetime, timezone
import serial

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
RTCM_TCP_IP = "192.168.1.95"      # IP of RTCM source
RTCM_TCP_PORT = 6001               # Port of RTCM source
BASE_SERIAL = "/dev/gps-base-link"  # serial device of base F9P
HEADING_SERIAL = "/dev/gps-heading" # serial device of heading F9P
SERIAL_BAUD = 115200

# UDP publication target – by default use localhost so navigation program on
# same machine can listen on UDP_PORT.
UDP_TARGET_IP = "127.0.0.1"
UDP_TARGET_PORT = 6002
UDP_PUBLISH_HZ = 20                 # broadcast rate

# Regex to extract fields from GNGGA
GGA_PATTERN = re.compile(
    rb"\$GNGGA,([^,]*),([^,]*),([NS]),([^,]*),([EW]),(\d),"
)
FIX_QUALITY = {
    0: "Invalid",
    1: "GPS Fix",
    2: "DGPS",
    4: "RTK Fixed",
    5: "RTK Float",
}

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
state = {
    "lat": None,
    "lon": None,
    "fix_quality": "Unknown",
    "heading_deg": None,
    "headValid": None,
    "carrier": None,
    "expectedErrDeg": None,
    "timestamp": None,
    # Fatal connection info
    "fatal_error": False,
    "fatal_base_reason": None,    # /dev/gps-base-link issues
    "fatal_heading_reason": None, # /dev/gps-heading issues

}
state_lock = threading.Lock()

# ---------------------------------------------------------------------------
# Utility functions for UBX parsing (RELPOSNED)
# ---------------------------------------------------------------------------
SYNC1, SYNC2 = 0xB5, 0x62
CLS_NAV, ID_RELPOSNED = 0x01, 0x3C

def ubx_checksum(data: bytes):
    a = b = 0
    for x in data:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b

def cm_hp_to_m(cm: int, hp_0p1mm: int) -> float:
    return (cm + hp_0p1mm * 0.01) / 100.0

def parse_relposned(payload: bytes):
    if len(payload) not in (40, 64):
        return None
    iTOW, relPosN, relPosE, relPosD = struct.unpack_from("<Iiii", payload, 0x04)
    off = 0x14
    relPosLen = relPosHead = None
    if len(payload) == 64:
        relPosLen, relPosHead = struct.unpack_from("<ii", payload, off)
        off += 8
        off += 4  # reserved
    relPosHPN, relPosHPE, relPosHPD = struct.unpack_from("<bbb", payload, off)
    off += 3
    relPosHPLen = None
    if len(payload) == 64:
        relPosHPLen = struct.unpack_from("<b", payload, off)[0]
        off += 1
    else:
        off += 1
    accN_0p1mm, accE_0p1mm, accD_0p1mm = struct.unpack_from("<III", payload, off)
    off += 12
    accLen_0p1mm = accHead_1e5deg = None
    if len(payload) == 64:
        accLen_0p1mm, accHead_1e5deg = struct.unpack_from("<II", payload, off)
        off += 8
        off += 4  # reserved3
    flags = struct.unpack_from("<I", payload, off)[0]

    # Convert to metric units where needed
    length = cm_hp_to_m(relPosLen, relPosHPLen) if relPosLen is not None else None
    heading = (relPosHead * 1e-5) if relPosHead is not None else None

    accN_m, accE_m = accN_0p1mm/10000.0, accE_0p1mm/10000.0
    accHead_d = (accHead_1e5deg*1e-5) if accHead_1e5deg is not None else None

    carrier = {0: "none", 1: "float", 2: "fixed"}.get((flags >> 3) & 0x3, "unknown")
    headValid = bool(flags & (1 << 8))
    return {
        "length_m": length,
        "heading_deg": heading,
        "accN_m": accN_m,
        "accE_m": accE_m,
        "accHead_deg": accHead_d,
        "carrier": carrier,
        "headValid": headValid,
    }


def expected_heading_error_deg(d):
    """Return expected 1-sigma heading error in degrees."""
    if d["accHead_deg"] is not None and d["headValid"]:
        return d["accHead_deg"]
    L = d["length_m"] or 0.0
    if L <= 1e-6:
        return float("nan")
    sigma_perp = math.hypot(d["accN_m"] or 0.0, d["accE_m"] or 0.0)
    return math.degrees(math.atan2(sigma_perp, L))

# ---------------------------------------------------------------------------
# Threads
# ---------------------------------------------------------------------------

def forward_rtcm(ser):
    forwarded_total = 0
    last_log_time = time.time()
    reconnect_delay = 5

    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(15)
            print(f"[RTCM Forwarder] Attempting connection to {RTCM_TCP_IP}:{RTCM_TCP_PORT}...")
            sock.connect((RTCM_TCP_IP, RTCM_TCP_PORT))
            print(f"[RTCM Forwarder] CONNECTED successfully to ESP32 RTCM source")

            interval_bytes = 0
            interval_start = time.time()

            while True:
                data = sock.recv(4096)
                if not data:
                    print("[RTCM Forwarder] Connection closed by ESP32 (empty recv)")
                    break

                written = ser.write(data)
                ser.flush()
                if written != len(data):
                    print(f"[RTCM Forwarder] WARNING: Serial write incomplete: {written}/{len(data)} bytes")

                interval_bytes += len(data)
                forwarded_total += len(data)

                now = time.time()
                if now - last_log_time >= 10:
                    elapsed = now - interval_start if interval_start else 1
                    rate = interval_bytes / elapsed if elapsed > 0 else 0
                    print(f"[RTCM Forwarder] Forwarded {interval_bytes} bytes (~{rate:.0f} B/s) in last {elapsed:.1f}s | Total forwarded: {forwarded_total}")
                    interval_bytes = 0
                    interval_start = now
                    last_log_time = now

        except socket.timeout:
            print(f"[RTCM Forwarder] TCP timeout after {sock.gettimeout()}s")
        except ConnectionRefusedError:
            print("[RTCM Forwarder] Connection refused by ESP32")
        except Exception as e:
            print(f"[RTCM Forwarder] Unexpected error: {type(e).__name__}: {e}")
        finally:
            try:
                sock.close()
            except:
                pass
            print(f"[RTCM Forwarder] Reconnecting in {reconnect_delay} seconds...")
            time.sleep(reconnect_delay)

def monitor_gga(serial_conn):
    buf = b""
    while True:
        try:
            b = serial_conn.read(1)
            if not b:
                # No data available right now - brief pause to avoid tight loop
                time.sleep(0.01)
                continue

            buf += b

            # Prevent unbounded growth if no newline
            if len(buf) > 512:
                print("[GGA Monitor] Buffer overflow prevention: trimming old data")
                buf = buf[-256:]

            if b == b'\n':
                line = buf.strip()
                buf = b""

                if line.startswith(b'$GNGGA') or line.startswith(b'$GPGGA'):
                    match = GGA_PATTERN.match(line)
                    if match:
                        time_str, lat_str, lat_dir, lon_str, lon_dir, fix = match.groups()
                        if fix in (b'4', b'5'):
                            try:
                                # Parse latitude
                                lat = float(lat_str)
                                lat_deg = int(lat / 100)
                                lat_min = lat - lat_deg * 100
                                lat = lat_deg + lat_min / 60
                                if lat_dir == b'S':
                                    lat = -lat

                                # Parse longitude
                                lon = float(lon_str)
                                lon_deg = int(lon / 100)
                                lon_min = lon - lon_deg * 100
                                lon = lon_deg + lon_min / 60
                                if lon_dir == b'W':
                                    lon = -lon

                                with state_lock:
                                    state["lat"] = lat
                                    state["lon"] = lon
                                    state["fix_quality"] = FIX_QUALITY.get(int(fix), "Unknown")
                                    state["timestamp"] = datetime.now(timezone.utc).isoformat()
                            except (ValueError, ZeroDivisionError) as parse_err:
                                print(f"[GGA Monitor] Parse error on line: {line.decode(errors='ignore')} - {parse_err}")

        except serial.SerialException as e:
            print(f"[GGA Monitor] Serial error: {e} - attempting recovery in 5s...")
            time.sleep(5)
            # Continue trying on existing connection - often recovers automatically
        except Exception as e:
            print(f"[GGA Monitor] Unexpected error: {type(e).__name__}: {e}")
            time.sleep(1)  # Prevent spam on repeated errors

def _parse_deg(raw: str, direction: str) -> float:
    """Convert NMEA latitude/longitude component to decimal degrees."""
    if not raw:
        return float("nan")
    # latitude has two degree digits; longitude has three
    deg_digits = 2 if direction in ("N", "S") else 3
    deg = float(raw[:deg_digits])
    minutes = float(raw[deg_digits:])
    value = deg + minutes / 60.0
    if direction in ("S", "W"):
        value = -value
    return value

def monitor_relposned(serial_conn):
    buf = bytearray()
    while True:
        try:
            b = serial_conn.read(1)
            if not b:
                # No data available right now - brief pause to avoid tight loop
                time.sleep(0.01)
                continue

            buf.append(b[0])

            # Prevent unbounded growth if sync never found
            if len(buf) > 512:
                print("[RELPOSNED Monitor] Buffer overflow prevention: trimming old data")
                buf = buf[-256:]

            # Need at least header to check sync
            if len(buf) < 2:
                continue

            # Search for sync chars
            if buf[0] != SYNC1 or buf[1] != SYNC2:
                buf.pop(0)
                continue

            # Need full header for length
            if len(buf) < 6:
                continue

            cls_ = buf[2]
            id_ = buf[3]
            length = buf[4] | (buf[5] << 8)
            need = length + 8  # header (6) + payload + checksum (2)

            if len(buf) < need:
                continue

            # Extract frame
            frame = buf[:need]
            buf = buf[need:]  # Remove processed frame

            # Verify checksum
            ck_a, ck_b = ubx_checksum(frame[2:6 + length])
            if ck_a != frame[6 + length] or ck_b != frame[7 + length]:
                print("[RELPOSNED Monitor] Checksum failed - discarding frame")
                continue

            payload = frame[6:6 + length]

            if cls_ == CLS_NAV and id_ == ID_RELPOSNED:
                d = parse_relposned(payload)
                if d:
                    err_deg = expected_heading_error_deg(d)  # Assuming this function exists in your script
                    with state_lock:
                        state["headValid"] = d["headValid"]
                        state["carrier"] = d["carrier"]
                        state["expectedErrDeg"] = err_deg
                        if d["headValid"]:
                            state["heading_deg"] = d["heading_deg"]
                        state["timestamp"] = datetime.now(timezone.utc).isoformat()

        except serial.SerialException as e:
            print(f"[RELPOSNED Monitor] Serial error: {e} - attempting recovery in 5s...")
            time.sleep(5)
            # Optional: fully reopen serial_conn here if you add reconnect logic
            # For now, continue trying on existing (often recovers)
        except Exception as e:
            print(f"[RELPOSNED Monitor] Unexpected error: {type(e).__name__}: {e}")
            time.sleep(1)  # Prevent spam on repeated errors

def udp_publisher():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        time.sleep(1.0 / UDP_PUBLISH_HZ)
        with state_lock:
            payload = json.dumps(state).encode()
        sock.sendto(payload, (UDP_TARGET_IP, UDP_TARGET_PORT))

# ---------------------------------------------------------------------------

def main():
    global state

    base_ser = None
    heading_ser = None

    fatal_error = False
    fatal_base_reason = None
    fatal_heading_reason = None

    # Try to open base serial
    try:
        base_ser = serial.Serial(BASE_SERIAL, SERIAL_BAUD, timeout=1)
    except (serial.SerialException, FileNotFoundError) as e:
        fatal_error = True
        fatal_base_reason = f"could not open {BASE_SERIAL}: {e}"

    # Try to open heading serial
    try:
        heading_ser = serial.Serial(HEADING_SERIAL, SERIAL_BAUD, timeout=1)
    except (serial.SerialException, FileNotFoundError) as e:
        fatal_error = True
        fatal_heading_reason = f"could not open {HEADING_SERIAL}: {e}"

    # Update shared state with fatal info so LED/status consumers can see it
    with state_lock:
        state["fatal_error"] = fatal_error
        state["fatal_base_reason"] = fatal_base_reason
        state["fatal_heading_reason"] = fatal_heading_reason
        state["timestamp"] = datetime.now(timezone.utc).isoformat()

    # Always start UDP publisher so we broadcast either normal nav data
    # or a fatal_error + reasons.
    threading.Thread(target=udp_publisher, daemon=True).start()

    if fatal_error:
        # Log clear messages to journalctl
        print("[rtcm-server] FATAL GPS CONNECTION ERROR(S) DETECTED")
        if fatal_base_reason:
            print(f"[rtcm-server]   BASE:    {fatal_base_reason}")
        if fatal_heading_reason:
            print(f"[rtcm-server]   HEADING: {fatal_heading_reason}")
        print("[rtcm-server] Service will remain running and continue to publish "
              "fatal_error status over UDP for LED/monitoring clients.")

        # Sit in a loop so systemd sees the service as 'running' and the LED controller can showing the fatal condition.
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        return

    # If we get here, both serial ports opened successfully: normal behavior
    threading.Thread(target=forward_rtcm, args=(base_ser,), daemon=True).start()
    threading.Thread(target=monitor_gga, args=(base_ser,), daemon=True).start()
    threading.Thread(target=monitor_relposned, args=(heading_ser,), daemon=True).start()

    print("RTCM server running. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
