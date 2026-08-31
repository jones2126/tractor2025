#!/usr/bin/env python3
"""RTCM forwarder and GPS data server.

This script forwards RTCM correction data from a TCP source to the base F9P
receiver and simultaneously parses NMEA (GNGGA) from the base receiver to
obtain latitude, longitude and fix quality.  It also listens to the heading
F9P receiver for UBX-NAV-RELPOSNED messages to obtain heading information.
Each F9P serial reader polls UBX-NAV-PVT and UBX-NAV-SAT at 1 Hz to publish
directly comparable, receiver-specific satellite diagnostics.

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
        "expectedErrDeg": float,          # expected heading error (1-sigma)
        "relposned_count": int,           # decoded UBX-NAV-RELPOSNED frames seen
        "relposned_timestamp": ISO-8601 string,
        "relpos_length_m": float,         # moving-baseline length
        "relpos_valid": bool,
        "relpos_moving": bool,
        "relpos_ref_pos_miss": bool,
        "relpos_ref_obs_miss": bool,
        "base_numSV_used": int,           # Base-Link NAV-PVT/NAV-SAT used
        "base_numSV_visible": int,        # Base-Link NAV-SAT records
        "heading_numSV_used": int,        # Heading NAV-PVT/NAV-SAT used
        "heading_numSV_visible": int,     # Heading NAV-SAT records reported
        "heading_cno_mean_dbhz": float,   # mean positive Heading C/N0
        "base_cno_mean_dbhz": float       # mean positive Base-Link C/N0
    }

Adjust the configuration section below to match your hardware setup.

11/26/25 - Added code for fatal errors when the GPS units are not detected
07/27/26 - Added dedicated UDP 6010 navigation feed so Pure Pursuit does not
           compete with the Teensy bridge on 6002 or field logger on 6009.
08/28/26 - Added symmetric Base-Link and Heading-F9P satellite diagnostics.
"""

import json
import math
import re
import socket
import struct
import subprocess
import threading
import time
from datetime import datetime, timezone
import serial

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
#RTCM_TCP_IP = "192.168.1.95"      # IP of RTCM source - Brenham
RTCM_TCP_IP = "192.168.193.88"      # IP of RTCM source - Bridgeville

RTCM_TCP_PORT = 6001               # Port of RTCM source
BASE_SERIAL = "/dev/gps-base-link"  # serial device of base F9P
HEADING_SERIAL = "/dev/gps-heading" # serial device of heading F9P
SERIAL_BAUD = 115200

# UDP publication target â€“ by default use localhost so navigation program on
# same machine can listen on UDP_PORT.
UDP_TARGET_IP = "127.0.0.1"
UDP_TARGET_PORT = 6002
UDP_NAV_PORT = 6010
UDP_LOG_PORT = 6009          # NEW line — dedicated feed for field_test_logger
UDP_PUBLISH_HZ = 20                 # broadcast rate
SATELLITE_DIAGNOSTIC_POLL_HZ = 1    # NAV-PVT/NAV-SAT, both receivers

# NEW (7/2/26): WiFi RSSI monitoring, mirrors RTKBase/Bridgeville/wifi_monitor_20260624.py
WIFI_INTERFACE  = "wlan0"
WIFI_POLL_HZ    = 2                 # polling rate for iwconfig (2 Hz requested; cheap on RPi)
WIFI_WEAK_DBM   = -80               # dBm threshold used for the "weak" label

# Regex to extract fields from GNGGA
# Fields: time, lat, N/S, lon, E/W, fix, numSV, HDOP, alt, M, geoidSep, M, diffAge
# CHANGED: Expanded from 6 capture groups to 8 (added numSV, HDOP)
# diffAge is parsed separately via split since fields 9-13 have optional empty values

# new pattern below captures numSV and HDOP as well, which are important diagnostics even when fix is degraded
GGA_PATTERN = re.compile(
    rb"\$G[NP]GGA,([^,]*),([^,]*),([NS]?),([^,]*),([EW]?),(\d),(\d*),([^,]*),"
)

# NEW (6/17/26) VTG pattern for ground speed
VTG_PATTERN = re.compile(
    rb"\$G[NP]VTG,[^,]*,[TM]?,[^,]*,[TM]?,([0-9]*\.?[0-9]+),N,([0-9]*\.?[0-9]+),K,"
)

# NEW (7/14/26) RMC pattern for ground speed - fallback since F9P outputs
# RMC by default but VTG is not enabled. Field 7 = speed over ground (knots).
RMC_PATTERN = re.compile(
    rb"\$G[NP]RMC,[^,]*,([AV]),[^,]*,[NS]?,[^,]*,[EW]?,([0-9]*\.?[0-9]+),"
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
    "base_numSV_used": None,       # Base-Link NAV-PVT/NAV-SAT used count
    "base_numSV_visible": None,    # Base-Link NAV-SAT record count
    "base_cno_mean_dbhz": None,
    "base_cno_min_dbhz": None,
    "base_cno_max_dbhz": None,
    "base_numSV_used_gga": None,   # Independent GGA cross-check
    "base_satellite_timestamp": None,
    "hdop": None,            # Horizontal dilution of precision (GGA field 8)
    "diff_age": None,        # Age of differential corrections in seconds (GGA field 13)
    "speed_mps": None,       # NEW 6/17/26: Ground speed m/s (from VTG)    
    "heading_deg": None,
    "headValid": None,
    "carrier": None,
    "expectedErrDeg": None,
    "relposned_count": 0,
    "relposned_timestamp": None,
    "relposned_itow_ms": None,
    "relpos_length_m": None,
    "relpos_heading_accuracy_deg": None,
    "relpos_gnss_fix_ok": None,
    "relpos_diff_solution": None,
    "relpos_valid": None,
    "relpos_moving": None,
    "relpos_ref_pos_miss": None,
    "relpos_ref_obs_miss": None,
    "relpos_normalized": None,
    "heading_numSV_used": None,       # Heading NAV-PVT/NAV-SAT satellites used
    "heading_numSV_visible": None,    # Heading NAV-SAT records reported
    "heading_cno_mean_dbhz": None,    # Mean positive NAV-SAT C/N0
    "heading_cno_min_dbhz": None,
    "heading_cno_max_dbhz": None,
    "heading_satellite_timestamp": None,
    "timestamp": None,
    # Fatal connection info
    "fatal_error": False,
    "fatal_base_reason": None,    # /dev/gps-base-link issues
    "fatal_heading_reason": None, # /dev/gps-heading issues
    # NEW (7/2/26): WiFi status, polled independently of GPS state
    "wifi_ssid": None,
    "wifi_rssi_dbm": None,
    "wifi_signal_label": "unknown",
}
state_lock = threading.Lock()
base_serial_write_lock = threading.Lock()

# ---------------------------------------------------------------------------
# NEW (7/2/26): WiFi monitoring helpers
# (adapted from RTKBase/Bridgeville/wifi_monitor_20260624.py)
# ---------------------------------------------------------------------------

def wifi_signal_label(rssi):
    if rssi is None:
        return "unknown"
    if rssi >= -65:
        return "strong"
    if rssi >= WIFI_WEAK_DBM:
        return "medium"
    return "weak"


def get_wifi_info():
    """Return (ssid, rssi_dBm) for current WIFI_INTERFACE connection, or (None, None)."""
    try:
        result = subprocess.run(
            ["iwconfig", WIFI_INTERFACE],
            capture_output=True, text=True, timeout=5
        )
        out = result.stdout
        ssid, rssi = None, None
        for line in out.splitlines():
            if 'ESSID:"' in line:
                ssid = line.split('ESSID:"')[1].split('"')[0]
                if ssid == "":
                    ssid = None
            if "Signal level=" in line:
                raw = line.split("Signal level=")[1].split()[0]
                try:
                    rssi = int(raw.replace("dBm", "").strip())
                except ValueError:
                    pass
        return ssid, rssi
    except Exception:
        return None, None

# ---------------------------------------------------------------------------
# Utility functions for UBX parsing
# ---------------------------------------------------------------------------
SYNC1, SYNC2 = 0xB5, 0x62
CLS_NAV = 0x01
ID_PVT = 0x07
ID_SAT = 0x35
ID_RELPOSNED = 0x3C

def ubx_checksum(data: bytes):
    a = b = 0
    for x in data:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def ubx_frame(message_class: int, message_id: int, payload: bytes = b"") -> bytes:
    """Build a checksum-valid UBX frame, including zero-length poll requests."""
    body = struct.pack("<BBH", message_class, message_id, len(payload)) + payload
    return bytes((SYNC1, SYNC2)) + body + bytes(ubx_checksum(body))


def parse_nav_pvt_satellites(payload: bytes):
    """Return satellites used from a UBX-NAV-PVT payload."""
    if len(payload) < 24:
        return None
    return {"numSV_used": payload[23]}


def parse_nav_sat(payload: bytes):
    """Summarize UBX-NAV-SAT records and their positive C/N0 values."""
    if len(payload) < 8:
        return None

    num_svs = payload[5]
    expected_length = 8 + num_svs * 12
    if len(payload) < expected_length:
        return None

    cno_values = []
    num_svs_used = 0
    for index in range(num_svs):
        offset = 8 + index * 12
        cno_dbhz = payload[offset + 2]
        flags = struct.unpack_from("<I", payload, offset + 8)[0]
        if flags & (1 << 3):  # svUsed
            num_svs_used += 1
        if cno_dbhz > 0:
            cno_values.append(cno_dbhz)

    return {
        "numSV_visible": num_svs,
        "numSV_used": num_svs_used,
        "cno_mean_dbhz": (
            round(sum(cno_values) / len(cno_values), 1)
            if cno_values else None
        ),
        "cno_min_dbhz": min(cno_values) if cno_values else None,
        "cno_max_dbhz": max(cno_values) if cno_values else None,
    }


def update_satellite_state(receiver: str, message_id: int, payload: bytes):
    """Apply a NAV-PVT or NAV-SAT summary for base or heading receiver."""
    if receiver not in ("base", "heading"):
        raise ValueError(f"unsupported receiver: {receiver}")

    parsed = (
        parse_nav_pvt_satellites(payload)
        if message_id == ID_PVT
        else parse_nav_sat(payload)
        if message_id == ID_SAT
        else None
    )
    if not parsed:
        return False

    prefix = f"{receiver}_"
    with state_lock:
        state[prefix + "numSV_used"] = parsed["numSV_used"]
        if message_id == ID_SAT:
            state[prefix + "numSV_visible"] = parsed["numSV_visible"]
            state[prefix + "cno_mean_dbhz"] = parsed["cno_mean_dbhz"]
            state[prefix + "cno_min_dbhz"] = parsed["cno_min_dbhz"]
            state[prefix + "cno_max_dbhz"] = parsed["cno_max_dbhz"]
        state[prefix + "satellite_timestamp"] = (
            datetime.now(timezone.utc).isoformat()
        )
    return True

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
        "iTOW_ms": iTOW,
        "length_m": length,
        "heading_deg": heading,
        "accN_m": accN_m,
        "accE_m": accE_m,
        "accHead_deg": accHead_d,
        "carrier": carrier,
        "headValid": headValid,
        "gnssFixOK": bool(flags & (1 << 0)),
        "diffSoln": bool(flags & (1 << 1)),
        "relPosValid": bool(flags & (1 << 2)),
        "isMoving": bool(flags & (1 << 5)),
        "refPosMiss": bool(flags & (1 << 6)),
        "refObsMiss": bool(flags & (1 << 7)),
        "relPosNormalized": bool(flags & (1 << 9)),
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

                # Keep complete RTCM and UBX poll frames from interleaving.
                with base_serial_write_lock:
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

def process_base_nmea_line(line: bytes):
    """Update position, fix, speed, and the independent GGA satellite count."""
    if line.startswith(b'$GNGGA') or line.startswith(b'$GPGGA'):
        match = GGA_PATTERN.match(line)
        if not match:
            return
        time_str, lat_str, lat_dir, lon_str, lon_dir, fix, num_sv_raw, hdop_raw = match.groups()
        try:
            fix_int = int(fix)
            fix_str = FIX_QUALITY.get(fix_int, "Unknown")
            num_sv = int(num_sv_raw) if num_sv_raw else None
            hdop = float(hdop_raw) if hdop_raw else None
            diff_age = None
            fields = line.split(b',')
            if len(fields) > 13 and fields[13]:
                try:
                    diff_age = float(fields[13])
                except ValueError:
                    pass

            with state_lock:
                state["fix_quality"] = fix_str
                state["base_numSV_used_gga"] = num_sv
                state["hdop"] = hdop
                state["diff_age"] = diff_age
                state["timestamp"] = datetime.now(timezone.utc).isoformat()

                if fix_int in (1, 2, 4, 5) and lat_str and lon_str:
                    lat = float(lat_str)
                    lat_deg = int(lat / 100)
                    lat = lat_deg + (lat - lat_deg * 100) / 60
                    if lat_dir == b'S':
                        lat = -lat

                    lon = float(lon_str)
                    lon_deg = int(lon / 100)
                    lon = lon_deg + (lon - lon_deg * 100) / 60
                    if lon_dir == b'W':
                        lon = -lon
                    state["lat"] = lat
                    state["lon"] = lon
        except (ValueError, ZeroDivisionError) as parse_err:
            print(f"[Base Monitor] GGA parse error: {parse_err}")

    elif line.startswith(b'$GNVTG') or line.startswith(b'$GPVTG'):
        match = VTG_PATTERN.match(line)
        if match:
            try:
                with state_lock:
                    state["speed_mps"] = round(float(match.group(2)) / 3.6, 3)
            except ValueError:
                pass

    elif line.startswith(b'$GNRMC') or line.startswith(b'$GPRMC'):
        match = RMC_PATTERN.match(line)
        if match:
            status, speed_knots_raw = match.groups()
            if status == b'A':
                try:
                    with state_lock:
                        state["speed_mps"] = round(float(speed_knots_raw) * 0.514444, 3)
                except ValueError:
                    pass


def monitor_base_receiver(serial_conn):
    """Read Base-Link NMEA plus polled UBX satellite diagnostics."""
    buf = bytearray()
    last_diagnostic_poll = 0.0
    while True:
        try:
            now = time.monotonic()
            if now - last_diagnostic_poll >= 1.0 / SATELLITE_DIAGNOSTIC_POLL_HZ:
                polls = ubx_frame(CLS_NAV, ID_PVT) + ubx_frame(CLS_NAV, ID_SAT)
                with base_serial_write_lock:
                    serial_conn.write(polls)
                    serial_conn.flush()
                last_diagnostic_poll = now

            b = serial_conn.read(1)
            if not b:
                time.sleep(0.01)
                continue
            buf.append(b[0])

            if buf.startswith(bytes((SYNC1, SYNC2))):
                if len(buf) < 6:
                    continue
                length = struct.unpack_from("<H", buf, 4)[0]
                frame_length = 8 + length
                if len(buf) < frame_length:
                    continue
                frame = bytes(buf[:frame_length])
                del buf[:frame_length]
                ck_a, ck_b = ubx_checksum(frame[2:6 + length])
                if (ck_a, ck_b) != (frame[6 + length], frame[7 + length]):
                    print("[Base Monitor] UBX checksum failed")
                    continue
                cls_, id_ = frame[2], frame[3]
                if cls_ == CLS_NAV and id_ in (ID_PVT, ID_SAT):
                    update_satellite_state("base", id_, frame[6:6 + length])
                continue

            if b == b'\n':
                process_base_nmea_line(bytes(buf).strip())
                buf.clear()

            if len(buf) > 4096:
                print("[Base Monitor] Buffer overflow prevention: clearing data")
                buf.clear()
        except serial.SerialException as e:
            print(f"[Base Monitor] Serial error: {e} - attempting recovery in 5s...")
            time.sleep(5)
        except Exception as e:
            print(f"[Base Monitor] Unexpected error: {type(e).__name__}: {e}")
            time.sleep(1)

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

def monitor_heading_ubx(serial_conn):
    """Read heading plus polled satellite diagnostics from one serial owner."""
    buf = bytearray()
    last_diagnostic_poll = 0.0
    while True:
        try:
            now = time.monotonic()
            if now - last_diagnostic_poll >= 1.0 / SATELLITE_DIAGNOSTIC_POLL_HZ:
                serial_conn.write(
                    ubx_frame(CLS_NAV, ID_PVT) + ubx_frame(CLS_NAV, ID_SAT)
                )
                serial_conn.flush()
                last_diagnostic_poll = now

            b = serial_conn.read(1)
            if not b:
                # No data available right now - brief pause to avoid tight loop
                time.sleep(0.01)
                continue

            buf.append(b[0])

            # Prevent unbounded growth if sync never found
            if len(buf) > 4096:
                print("[Heading UBX Monitor] Buffer overflow prevention: trimming old data")
                buf = buf[-2048:]

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
                print("[Heading UBX Monitor] Checksum failed - discarding frame")
                continue

            payload = frame[6:6 + length]

            if cls_ == CLS_NAV and id_ == ID_RELPOSNED:
                d = parse_relposned(payload)
                if d:
                    err_deg = expected_heading_error_deg(d)  # Assuming this function exists in your script
                    observed_at = datetime.now(timezone.utc).isoformat()
                    with state_lock:
                        state["relposned_count"] += 1
                        state["relposned_timestamp"] = observed_at
                        state["relposned_itow_ms"] = d["iTOW_ms"]
                        state["relpos_length_m"] = d["length_m"]
                        state["relpos_heading_accuracy_deg"] = d["accHead_deg"]
                        state["relpos_gnss_fix_ok"] = d["gnssFixOK"]
                        state["relpos_diff_solution"] = d["diffSoln"]
                        state["relpos_valid"] = d["relPosValid"]
                        state["relpos_moving"] = d["isMoving"]
                        state["relpos_ref_pos_miss"] = d["refPosMiss"]
                        state["relpos_ref_obs_miss"] = d["refObsMiss"]
                        state["relpos_normalized"] = d["relPosNormalized"]
                        state["headValid"] = d["headValid"]
                        state["carrier"] = d["carrier"]
                        state["expectedErrDeg"] = err_deg
                        if d["headValid"]:
                            state["heading_deg"] = d["heading_deg"]
                        state["timestamp"] = observed_at

            elif cls_ == CLS_NAV and id_ == ID_PVT:
                update_satellite_state("heading", id_, payload)

            elif cls_ == CLS_NAV and id_ == ID_SAT:
                update_satellite_state("heading", id_, payload)

        except serial.SerialException as e:
            print(f"[Heading UBX Monitor] Serial error: {e} - attempting recovery in 5s...")
            time.sleep(5)
            # Optional: fully reopen serial_conn here if you add reconnect logic
            # For now, continue trying on existing (often recovers)
        except Exception as e:
            print(f"[Heading UBX Monitor] Unexpected error: {type(e).__name__}: {e}")
            time.sleep(1)  # Prevent spam on repeated errors

# NEW (7/2/26): polls WiFi RSSI at WIFI_POLL_HZ and writes into shared state,
# which udp_publisher() below broadcasts on UDP 6002 along with GPS data.
def monitor_wifi():
    while True:
        try:
            ssid, rssi = get_wifi_info()
            label = wifi_signal_label(rssi)
            with state_lock:
                state["wifi_ssid"] = ssid
                state["wifi_rssi_dbm"] = rssi
                state["wifi_signal_label"] = label
        except Exception as e:
            print(f"[WiFi Monitor] Unexpected error: {type(e).__name__}: {e}")
        time.sleep(1.0 / WIFI_POLL_HZ)


def udp_publisher():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        time.sleep(1.0 / UDP_PUBLISH_HZ)
        with state_lock:
            payload = json.dumps(state).encode()
        sock.sendto(payload, (UDP_TARGET_IP, UDP_TARGET_PORT))
        sock.sendto(payload, (UDP_TARGET_IP, UDP_NAV_PORT))
        sock.sendto(payload, (UDP_TARGET_IP, UDP_LOG_PORT))  # NEW line — separate flow, avoids SO_REUSEPORT contention with the controller on 6002

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

    # NEW (7/2/26): Always start WiFi monitor too - independent of GPS fatal
    # state, so signal strength is visible even when the GPS units fail.
    threading.Thread(target=monitor_wifi, daemon=True).start()

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
    threading.Thread(target=monitor_base_receiver, args=(base_ser,), daemon=True).start()
    threading.Thread(target=monitor_heading_ubx, args=(heading_ser,), daemon=True).start()

    print("RTCM server running. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
