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
RTCM_TCP_IP = "192.168.1.180"      # IP of RTCM source
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

def forward_rtcm(serial_conn):
    """Forward RTCM data from TCP stream to base receiver."""
    while True:
        try:
            sock = socket.create_connection((RTCM_TCP_IP, RTCM_TCP_PORT), timeout=10)
            while True:
                data = sock.recv(1024)
                if not data:
                    raise ConnectionError("RTCM stream closed")
                serial_conn.write(data)
        except Exception:
            time.sleep(5)


def monitor_gga(serial_conn):
    """Monitor GNGGA sentences for lat/lon/fix info."""
    buffer = b""
    while True:
        b = serial_conn.read(1)
        if not b:
            continue
        buffer += b
        if b == b"\n":
            if b"GNGGA" in buffer:
                m = GGA_PATTERN.search(buffer)
                if m:
                    time_raw, lat_raw, lat_dir, lon_raw, lon_dir, fix_code = m.groups()
                    lat = _parse_deg(lat_raw.decode(), lat_dir.decode())
                    lon = _parse_deg(lon_raw.decode(), lon_dir.decode())
                    fix = FIX_QUALITY.get(int(fix_code), "Unknown")
                    with state_lock:
                        state.update({
                            "lat": lat,
                            "lon": lon,
                            "fix_quality": fix,
                            "timestamp": datetime.now(timezone.utc).isoformat(),
                        })
            buffer = b""

'''
2. Change both `datetime.utcnow().isoformat()` calls (lines 192 and 247) to\
   `datetime.now(timezone.utc).isoformat()`.
'''


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
    """Parse UBX-NAV-RELPOSNED for heading."""
    buf = bytearray()
    while True:
        b = serial_conn.read(1)
        if not b:
            continue
        buf += b
        if len(buf) == 1 and buf[0] != SYNC1:
            buf.clear(); continue
        if len(buf) == 2 and buf[1] != SYNC2:
            buf = bytearray([buf[1]]) if buf[1] == SYNC1 else bytearray(); continue
        if len(buf) < 6:
            continue
        cls_, id_, length = struct.unpack_from("<BBH", buf, 2)
        need = 6 + length + 2
        while len(buf) < need:
            chunk = serial_conn.read(need - len(buf))
            if not chunk:
                break
            buf += chunk
        if len(buf) < need:
            buf.clear(); continue
        ck_a, ck_b = ubx_checksum(buf[2:6+length])
        if (ck_a, ck_b) != (buf[6+length], buf[7+length]):
            buf.clear(); continue
        payload = bytes(buf[6:6+length])
        buf.clear()
        if cls_ == CLS_NAV and id_ == ID_RELPOSNED:
            d = parse_relposned(payload)
            if d:
                err_deg = expected_heading_error_deg(d)
                with state_lock:
                    state["headValid"] = d["headValid"]
                    state["carrier"] = d["carrier"]
                    state["expectedErrDeg"] = err_deg
                    if d["headValid"]:
                        state["heading_deg"] = d["heading_deg"]
                    state["timestamp"] = datetime.now(timezone.utc).isoformat()

def udp_publisher():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        time.sleep(1.0 / UDP_PUBLISH_HZ)
        with state_lock:
            payload = json.dumps(state).encode()
        sock.sendto(payload, (UDP_TARGET_IP, UDP_TARGET_PORT))

# ---------------------------------------------------------------------------

def main():
    base_ser = serial.Serial(BASE_SERIAL, SERIAL_BAUD, timeout=1)
    heading_ser = serial.Serial(HEADING_SERIAL, SERIAL_BAUD, timeout=1)

    threading.Thread(target=forward_rtcm, args=(base_ser,), daemon=True).start()
    threading.Thread(target=monitor_gga, args=(base_ser,), daemon=True).start()
    threading.Thread(target=monitor_relposned, args=(heading_ser,), daemon=True).start()
    threading.Thread(target=udp_publisher, daemon=True).start()

    print("RTCM server running. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
