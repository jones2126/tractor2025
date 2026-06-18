#!/usr/bin/env python3
"""RTCM forwarder and GPS data server — ZED-X20D single-module variant.

Hardware: ArduSimple simpleRTK 4 Dual (u-blox ZED-X20D chipset)
          Two antennas on one module; produces position + heading on one USB port.

Differences from rtcm_server_20260617.py (two-F9P version):
  - Single serial port (/dev/gps-heading) replaces two F9P ports.
  - UBX-NAV-DAHEADING (0x01 0x45, 64-byte payload) replaces RELPOSNED (0x01 0x3C).
  - One combined reader thread (monitor_gga_and_daheading) replaces the two
    separate monitor_gga / monitor_relposned threads.
  - RTCM corrections are still written to the same serial port (Linux full-duplex;
    one thread reads, forward_rtcm writes — no lock needed).
  - fatal_base_reason key removed from state dict (only one port to open).

UDP output schema is IDENTICAL to the F9P version so all downstream consumers
(teensy_serial_bridge.py, led_status_controller.py, nav stack) need no changes.

2026-06-15  Initial version derived from rtcm_server_20260306.py + parseDAHEADING.py
2026-06-17  Synced with rtcm_server_20260617.py (F9P):
              - NEW: VTG_PATTERN regex for ground speed parsing
              - NEW: speed_mps field added to state dict
              - NEW: VTG parsing block added to _parse_gga_line()
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
# CHANGED: Single port replaces two F9P serial entries.
# REMOVED: BASE_SERIAL = "/dev/gps-base-link"
# REMOVED: HEADING_SERIAL = "/dev/gps-heading"  (was heading F9P)
# ---------------------------------------------------------------------------
# Line ~30 — select active base station IP for your location
#RTCM_TCP_IP = "192.168.1.95"       # Brenham TX base
RTCM_TCP_IP = "192.168.193.88"      # Bridgeville PA base (ZeroTier)

RTCM_TCP_PORT = 6001

# Single ZED-X20D port (replaces BASE_SERIAL + HEADING_SERIAL)
X20D_SERIAL = "/dev/gps-heading"    # udev symlink for ZED-X20D (VID=1546, PID=01ab)
SERIAL_BAUD  = 115200

UDP_TARGET_IP   = "127.0.0.1"
UDP_TARGET_PORT = 6002
UDP_PUBLISH_HZ  = 20

# Regex: parse $GNGGA / $GPGGA.  Captures time, lat, N/S, lon, E/W, fix, numSV, hdop.
GGA_PATTERN = re.compile(
    rb"\$G[NP]GGA,([^,]*),([^,]*),([NS]?),([^,]*),([EW]?),(\d),(\d*),([^,]*),"
)

# NEW (synced from rtcm_server_20260617.py): VTG pattern for ground speed
# Line ~50
VTG_PATTERN = re.compile(
    rb"\$G[NP]VTG,[^,]*,[TM]?,[^,]*,[TM]?,([0-9]*\.?[0-9]+),N,([0-9]*\.?[0-9]+),K,"
)

FIX_QUALITY = {
    0: "Invalid",
    1: "GPS Fix",
    2: "DGPS",
    4: "RTK Fixed",
    5: "RTK Float",
}

# ---------------------------------------------------------------------------
# Shared state  — schema IDENTICAL to F9P version; downstream consumers unchanged.
# CHANGED: removed fatal_base_reason (only one port now).
# NEW (line ~70): speed_mps field added to match rtcm_server_20260617.py
# ---------------------------------------------------------------------------
state = {
    "lat":              None,
    "lon":              None,
    "fix_quality":      "Unknown",
    "numSV":            None,
    "hdop":             None,
    "diff_age":         None,
    "speed_mps":        None,       # NEW: Ground speed m/s (from VTG sentence)
    "heading_deg":      None,
    "headValid":        None,
    "carrier":          None,
    "expectedErrDeg":   None,
    "timestamp":        None,
    # Fatal connection info
    "fatal_error":          False,
    "fatal_heading_reason": None,   # single port — was fatal_base_reason + fatal_heading_reason
}
state_lock = threading.Lock()

# ---------------------------------------------------------------------------
# UBX framing constants
# ID_DAHEADING = 0x45  (replaces ID_RELPOSNED = 0x3C from F9P version)
# ---------------------------------------------------------------------------
SYNC1, SYNC2            = 0xB5, 0x62
CLS_NAV                 = 0x01
ID_DAHEADING            = 0x45      # ZED-X20D native heading message
DAHEADING_PAYLOAD_LEN   = 64        # confirmed by bench test 2026-05-29

def ubx_checksum(data: bytes):
    """Fletcher-8 checksum over bytes between sync and checksum fields."""
    a = b = 0
    for x in data:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b

# ---------------------------------------------------------------------------
# parse_daheading() — replaces parse_relposned() from F9P version.
# Derived from tractor_rpi/testing/parseDAHEADING.py (bench-tested 2026-05-29).
# Returns dict with same keys used downstream as parse_relposned() did,
# so expected_heading_error_deg() and state update code remain compatible.
# ---------------------------------------------------------------------------
def parse_daheading(payload: bytes):
    """Parse UBX-NAV-DAHEADING payload (64 bytes).

    Field layout (confirmed by live bench test):
      Offset  Type  Field
      0       U1    version (= 1)
      1-3     —     reserved
      4-7     U4    iTOW (ms)
      8-11    I4    relPosN (cm)
      12-15   I4    relPosE (cm)
      16-19   I4    relPosD (cm)
      20-23   U4    relPosLength (cm)      <-- U4, not I4
      24-27   U4    relPosHeading (1e-5 deg, True North CW)
      28-31   —     reserved
      32      I1    relPosHPN (0.1 mm HP)
      33      I1    relPosHPE (0.1 mm HP)
      34      I1    relPosHPD (0.1 mm HP)
      35      I1    relPosHPLength (0.1 mm HP)
      36-39   U4    accN (0.1 mm)
      40-43   U4    accE (0.1 mm)
      44-47   U4    accD (0.1 mm)
      48-51   U4    accLength (0.1 mm)
      52-55   U4    accHeading (1e-5 deg)
      56-59   —     reserved
      60-63   U4    flags
                      bit 0  gnssFixOK
                      bit 1  diffSoln
                      bit 2  relPosValid
                      bits 3-4  carrSoln (0=none,1=float,2=fixed)
                      bit 8  relPosHeadingValid
    """
    if len(payload) != DAHEADING_PAYLOAD_LEN:
        return None

    iTOW                          = struct.unpack_from("<I",    payload,  4)[0]
    relPosN_cm, relPosE_cm, \
        relPosD_cm                = struct.unpack_from("<iii",  payload,  8)
    relPosLen_cm                  = struct.unpack_from("<I",    payload, 20)[0]   # U4
    relPosHead_1e5deg             = struct.unpack_from("<I",    payload, 24)[0]   # U4
    hpN, hpE, hpD, hpLen         = struct.unpack_from("<bbbb", payload, 32)
    accN_0p1mm                    = struct.unpack_from("<I",    payload, 36)[0]
    accE_0p1mm                    = struct.unpack_from("<I",    payload, 40)[0]
    accD_0p1mm                    = struct.unpack_from("<I",    payload, 44)[0]
    accLen_0p1mm                  = struct.unpack_from("<I",    payload, 48)[0]
    accHead_1e5deg                = struct.unpack_from("<I",    payload, 52)[0]
    flags                         = struct.unpack_from("<I",    payload, 60)[0]

    heading_deg = relPosHead_1e5deg * 1e-5
    accHead_deg = accHead_1e5deg    * 1e-5

    # High-precision metric positions
    N_m      = (relPosN_cm + hpN * 0.01) / 100.0
    E_m      = (relPosE_cm + hpE * 0.01) / 100.0
    length_m = (relPosLen_cm + hpLen * 0.01) / 100.0

    head_valid = bool(flags & (1 << 8))
    carr_soln  = {0: "none", 1: "float", 2: "fixed"}.get((flags >> 3) & 0x3, "unknown")

    # Return dict with same keys as parse_relposned() for downstream compatibility
    return {
        "length_m":     length_m,
        "heading_deg":  heading_deg,
        "accN_m":       accN_0p1mm  / 10000.0,
        "accE_m":       accE_0p1mm  / 10000.0,
        "accHead_deg":  accHead_deg,
        "carrier":      carr_soln,
        "headValid":    head_valid,
    }


def expected_heading_error_deg(d):
    """Return expected 1-sigma heading error in degrees.
    UNCHANGED from F9P version — dict keys are identical.
    """
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
    """Forward RTCM corrections from TCP base station to X20D serial port.
    UNCHANGED from F9P version — still writes to the one serial port.
    Linux full-duplex: this thread writes while monitor_gga_and_daheading reads.
    """
    forwarded_total = 0
    last_log_time   = time.time()
    reconnect_delay = 5

    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(15)
            print(f"[RTCM Forwarder] Connecting to {RTCM_TCP_IP}:{RTCM_TCP_PORT}...")
            sock.connect((RTCM_TCP_IP, RTCM_TCP_PORT))
            print("[RTCM Forwarder] CONNECTED to RTK base")

            interval_bytes = 0
            interval_start = time.time()

            while True:
                data = sock.recv(4096)
                if not data:
                    print("[RTCM Forwarder] Connection closed by base (empty recv)")
                    break

                written = ser.write(data)
                ser.flush()
                if written != len(data):
                    print(f"[RTCM Forwarder] WARNING: incomplete write {written}/{len(data)} bytes")

                interval_bytes  += len(data)
                forwarded_total += len(data)

                now = time.time()
                if now - last_log_time >= 10:
                    elapsed = now - interval_start if interval_start else 1
                    rate = interval_bytes / elapsed if elapsed > 0 else 0
                    print(f"[RTCM Forwarder] {interval_bytes}B (~{rate:.0f} B/s) "
                          f"| total {forwarded_total}B")
                    interval_bytes = 0
                    interval_start = now
                    last_log_time  = now

        except socket.timeout:
            print(f"[RTCM Forwarder] TCP timeout")
        except ConnectionRefusedError:
            print("[RTCM Forwarder] Connection refused")
        except Exception as e:
            print(f"[RTCM Forwarder] Error: {type(e).__name__}: {e}")
        finally:
            try:
                sock.close()
            except Exception:
                pass
            print(f"[RTCM Forwarder] Reconnecting in {reconnect_delay}s...")
            time.sleep(reconnect_delay)


# Combined GGA + DAHEADING reader — replaces monitor_gga() and monitor_relposned()
# from the F9P version.
#
# The X20D sends both NMEA sentences (GGA for position/fix) and UBX binary
# (DAHEADING for heading) on the same USB serial port, interleaved.
# This thread reads one byte at a time and dispatches based on framing:
#   '$'       → accumulate NMEA line until '\n', then parse GGA/VTG
#   0xB5 0x62 → accumulate UBX frame, verify checksum, parse DAHEADING
#   anything else → discard (e.g. other NMEA sentences, other UBX IDs)
def monitor_gga_and_daheading(serial_conn):
    """Single reader thread for the ZED-X20D serial port.
    Dispatches to GGA/VTG parser (position/fix/speed) or DAHEADING parser (heading).
    """
    nmea_buf = b""
    ubx_buf  = bytearray()
    IN_NMEA  = 1
    IN_UBX   = 2
    IDLE     = 0
    mode     = IDLE

    while True:
        try:
            raw = serial_conn.read(1)
            if not raw:
                time.sleep(0.01)
                continue

            byte = raw[0]

            # ---- UBX sync detection (highest priority — check before NMEA '$') ----
            if mode == IN_UBX:
                ubx_buf.append(byte)

                # Need at least 6 bytes for header
                if len(ubx_buf) < 6:
                    continue

                cls_  = ubx_buf[2]
                id_   = ubx_buf[3]
                plen  = ubx_buf[4] | (ubx_buf[5] << 8)
                need  = 6 + plen + 2   # header + payload + checksum

                if len(ubx_buf) < need:
                    continue

                # Full frame received — verify checksum
                frame = bytes(ubx_buf[:need])
                ubx_buf = bytearray()
                mode = IDLE

                ck_a, ck_b = ubx_checksum(frame[2:6 + plen])
                if ck_a != frame[6 + plen] or ck_b != frame[7 + plen]:
                    print("[X20D Monitor] UBX checksum failed — discarding frame")
                    continue

                # Only care about DAHEADING
                if cls_ != CLS_NAV or id_ != ID_DAHEADING:
                    continue

                payload = frame[6:6 + plen]
                d = parse_daheading(payload)
                if not d:
                    continue

                err_deg = expected_heading_error_deg(d)
                with state_lock:
                    state["headValid"]      = d["headValid"]
                    state["carrier"]        = d["carrier"]
                    state["expectedErrDeg"] = err_deg
                    if d["headValid"]:
                        state["heading_deg"] = d["heading_deg"]
                    state["timestamp"] = datetime.now(timezone.utc).isoformat()

            elif mode == IN_NMEA:
                if byte == ord('\n'):
                    line = nmea_buf.strip()
                    nmea_buf = b""
                    mode = IDLE
                    _parse_gga_line(line)   # handles both GGA and VTG
                else:
                    nmea_buf += raw
                    if len(nmea_buf) > 512:
                        print("[X20D Monitor] NMEA buffer overflow — resetting")
                        nmea_buf = b""
                        mode = IDLE

            else:  # IDLE — look for frame start
                if byte == ord('$'):
                    nmea_buf = b"$"
                    mode = IN_NMEA
                elif byte == SYNC1:
                    ubx_buf = bytearray([SYNC1])
                    mode = IN_UBX

            # Confirm second sync byte when we just entered UBX mode with one byte
            if mode == IN_UBX and len(ubx_buf) == 1:
                pass  # wait for next byte to be appended above
            elif mode == IN_UBX and len(ubx_buf) == 2 and ubx_buf[1] != SYNC2:
                # Not a valid UBX frame start — discard and reset
                ubx_buf = bytearray()
                mode = IDLE

        except serial.SerialException as e:
            print(f"[X20D Monitor] Serial error: {e} — retrying in 5s...")
            time.sleep(5)
        except Exception as e:
            print(f"[X20D Monitor] Unexpected error: {type(e).__name__}: {e}")
            time.sleep(1)


def _parse_gga_line(line: bytes):
    """Parse a single NMEA sentence and update shared state.
    Handles GGA (position/fix/diagnostics) and VTG (ground speed).

    CHANGED (20260617): Added VTG parsing block (synced from rtcm_server_20260617.py).
    """
    # ---- GGA ----
    if line.startswith(b'$GNGGA') or line.startswith(b'$GPGGA'):
        match = GGA_PATTERN.match(line)
        if not match:
            return

        time_str, lat_str, lat_dir, lon_str, lon_dir, fix, num_sv_raw, hdop_raw = match.groups()

        try:
            fix_int = int(fix)
            fix_str = FIX_QUALITY.get(fix_int, "Unknown")
            num_sv  = int(num_sv_raw)  if num_sv_raw  else None
            hdop    = float(hdop_raw)  if hdop_raw    else None

            # diff_age is GGA field index 13 (0-based)
            diff_age = None
            fields = line.split(b',')
            if len(fields) > 13 and fields[13]:
                try:
                    diff_age = float(fields[13])
                except ValueError:
                    pass

            with state_lock:
                state["fix_quality"] = fix_str
                state["numSV"]       = num_sv
                state["hdop"]        = hdop
                state["diff_age"]    = diff_age
                state["timestamp"]   = datetime.now(timezone.utc).isoformat()

                # Parse lat/lon for any valid fix (GPS, DGPS, RTK Float, RTK Fixed)
                if fix_int in (1, 2, 4, 5) and lat_str and lon_str:
                    lat     = float(lat_str)
                    lat_deg = int(lat / 100)
                    lat     = lat_deg + (lat - lat_deg * 100) / 60.0
                    if lat_dir == b'S':
                        lat = -lat

                    lon     = float(lon_str)
                    lon_deg = int(lon / 100)
                    lon     = lon_deg + (lon - lon_deg * 100) / 60.0
                    if lon_dir == b'W':
                        lon = -lon

                    state["lat"] = lat
                    state["lon"] = lon

        except (ValueError, ZeroDivisionError) as e:
            print(f"[X20D Monitor] GGA parse error: {line.decode(errors='ignore')} — {e}")

    # ---- VTG — ground speed (NEW, synced from rtcm_server_20260617.py) ----
    elif line.startswith(b'$GNVTG') or line.startswith(b'$GPVTG'):
        match = VTG_PATTERN.match(line)
        if match:
            try:
                speed_kmh = float(match.group(2))
                with state_lock:
                    state["speed_mps"] = round(speed_kmh / 3.6, 3)
            except ValueError:
                pass


def udp_publisher():
    """Broadcast state dict at UDP_PUBLISH_HZ.  UNCHANGED from F9P version."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        time.sleep(1.0 / UDP_PUBLISH_HZ)
        with state_lock:
            payload = json.dumps(state).encode()
        sock.sendto(payload, (UDP_TARGET_IP, UDP_TARGET_PORT))


# ---------------------------------------------------------------------------
# Main
# CHANGED: Open one serial port instead of two.
# REMOVED: base_ser open + fatal_base_reason block
# REMOVED: heading_ser open + fatal_heading_reason block
# NEW:     x20d_ser open + fatal_heading_reason (reused key for single-port error)
# ---------------------------------------------------------------------------
def main():
    x20d_ser     = None
    fatal_error  = False

    try:
        x20d_ser = serial.Serial(X20D_SERIAL, SERIAL_BAUD, timeout=1)
        print(f"[main] Opened {X20D_SERIAL} at {SERIAL_BAUD} baud")
    except (serial.SerialException, FileNotFoundError) as e:
        fatal_error = True
        with state_lock:
            state["fatal_error"]          = True
            state["fatal_heading_reason"] = f"could not open {X20D_SERIAL}: {e}"
            state["timestamp"]            = datetime.now(timezone.utc).isoformat()

    # Always start UDP publisher — broadcasts fatal_error state if port open failed
    threading.Thread(target=udp_publisher, daemon=True).start()

    if fatal_error:
        print("[rtcm-server-x20d] FATAL: could not open X20D serial port")
        with state_lock:
            print(f"[rtcm-server-x20d]   {state['fatal_heading_reason']}")
        print("[rtcm-server-x20d] Remaining running to broadcast fatal_error over UDP.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        return

    # Normal operation — two threads: RTCM forwarder + combined GGA/DAHEADING reader
    threading.Thread(target=forward_rtcm,              args=(x20d_ser,), daemon=True).start()
    threading.Thread(target=monitor_gga_and_daheading, args=(x20d_ser,), daemon=True).start()

    print("rtcm_server_x20d_20260617 running (ZED-X20D single-port mode). Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
