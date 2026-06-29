#!/usr/bin/env python3
"""
PX1125R Commit Surveyed Coordinate as Fixed Static RTK Base — JSON Edition

This script reads the coordinate and supporting survey record from a JSON
file instead of embedding latitude, longitude, or ellipsoid height in Python.

Default configuration file:
    px1125r_base_config.json
in the same folder as this script.

Modes:
  Preview:
      python px1125r_commit_static_base_json.py

  Commit the JSON coordinate to SRAM + Flash:
      python px1125r_commit_static_base_json.py --commit

  Verify after a manual power-cycle:
      python px1125r_commit_static_base_json.py --verify

  Use a different survey-candidate/config record:
      python px1125r_commit_static_base_json.py --config my_survey.json --commit

The script:
  - logs raw receiver output to a .bin capture
  - prints raw ACK/NACK and 6A 83 RTK status messages
  - verifies repeat CRC-valid RTCM 1005 frames
  - appends successful commit/verification evidence to the JSON audit_log
  - creates a .bak copy before changing the JSON file

Important:
  - height must be WGS-84 ellipsoid height, not MSL elevation
  - close GNSS Viewer and stop any process that owns the serial port
  - do not forward a raw mixed serial stream to a rover during this operation
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import struct
import sys
import time
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from functools import reduce
from pathlib import Path
from typing import Any, Optional

import serial


# =============================================================================
# USER SETTINGS
# =============================================================================

# PORT = "COM6"        # Windows: "COM6"; Raspberry Pi: "/dev/gps" or "/dev/ttyUSB0"
PORT = "/dev/skytraq"        # Windows: "COM6"; Raspberry Pi: "/dev/gps" or "/dev/ttyUSB0"
BAUD = 115200

# Default file is next to this script. Override it with --config PATH.
DEFAULT_CONFIG_PATH = Path(__file__).with_name("px1125r_base_config.json")

# Static-mode command fields. The receiver may retain legacy survey values in
# its status response even after saved operation becomes Static; this script
# does not interpret those retained fields as a new survey.
STATIC_SURVEY_LENGTH_SECONDS = 0
STATIC_STANDARD_DEVIATION_M = 0
STATIC_BASELINE_CONSTRAINT_M = 0.0
COMMIT_ATTRIBUTES = 1   # 1 = SRAM + Flash

SERIAL_TIMEOUT_SECONDS = 0.20
COMMAND_RESPONSE_SECONDS = 3.0
COMMAND_GAP_SECONDS = 1.0
POST_COMMIT_1005_TIMEOUT_SECONDS = 20
REQUIRED_1005_FRAMES = 3

# Before writing Flash, the live run-time survey coordinate must agree tightly
# with the JSON target. These are deliberately strict guardrails.
RUNTIME_LAT_LON_TOLERANCE_DEG = 0.00000005   # roughly 5.6 mm latitude
RUNTIME_HEIGHT_TOLERANCE_M = 0.05            # 5 cm

# After commit/reboot, permit only expected LLA <-> ECEF <-> RTCM quantization.
VERIFY_LAT_LON_TOLERANCE_DEG = 0.00000020    # roughly 2.2 cm latitude
VERIFY_HEIGHT_TOLERANCE_M = 0.10             # 10 cm


# =============================================================================
# RUNTIME CONFIGURATION, LOADED FROM JSON
# =============================================================================

CONFIG_PATH: Optional[Path] = None
BASE_CONFIG: dict[str, Any] = {}
TARGET_LAT_DEG = 0.0
TARGET_LON_DEG = 0.0
TARGET_ELLIPSOID_HEIGHT_M = 0.0
TARGET_STATION_ID: Optional[int] = None
CAPTURE_FILENAME = ""


def local_timestamp() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def require_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"JSON field '{label}' must be a number.")
    return float(value)


def load_base_config(path: Path) -> dict[str, Any]:
    global BASE_CONFIG, CONFIG_PATH
    global TARGET_LAT_DEG, TARGET_LON_DEG, TARGET_ELLIPSOID_HEIGHT_M
    global TARGET_STATION_ID, CAPTURE_FILENAME

    if not path.is_file():
        raise FileNotFoundError(f"JSON configuration file not found: {path}")

    with path.open("r", encoding="utf-8") as fh:
        loaded = json.load(fh)

    if not isinstance(loaded, dict):
        raise ValueError("JSON root must be an object.")

    coordinate = loaded.get("coordinate")
    if not isinstance(coordinate, dict):
        raise ValueError("JSON must contain a 'coordinate' object.")

    datum = str(coordinate.get("datum", "")).strip()
    if datum.upper() not in {"WGS 84", "WGS84", "WGS-84"}:
        raise ValueError(
            "JSON coordinate.datum must identify WGS 84 / WGS84 / WGS-84."
        )

    TARGET_LAT_DEG = require_number(coordinate.get("latitude_deg"), "coordinate.latitude_deg")
    TARGET_LON_DEG = require_number(coordinate.get("longitude_deg"), "coordinate.longitude_deg")
    TARGET_ELLIPSOID_HEIGHT_M = require_number(
        coordinate.get("ellipsoid_height_m"),
        "coordinate.ellipsoid_height_m",
    )

    if not -90.0 <= TARGET_LAT_DEG <= 90.0:
        raise ValueError("coordinate.latitude_deg must be between -90 and +90.")
    if not -180.0 <= TARGET_LON_DEG <= 180.0:
        raise ValueError("coordinate.longitude_deg must be between -180 and +180.")
    if not -1000.0 <= TARGET_ELLIPSOID_HEIGHT_M <= 10000.0:
        raise ValueError("coordinate.ellipsoid_height_m is outside a plausible range.")

    base = loaded.get("base", {})
    if base is not None and not isinstance(base, dict):
        raise ValueError("JSON field 'base' must be an object when supplied.")

    station_id = base.get("reference_station_id") if isinstance(base, dict) else None
    if station_id is not None:
        if isinstance(station_id, bool) or not isinstance(station_id, int):
            raise ValueError("base.reference_station_id must be an integer or null.")
        if not 0 <= station_id <= 4095:
            raise ValueError("base.reference_station_id must be between 0 and 4095.")
        TARGET_STATION_ID = station_id
    else:
        TARGET_STATION_ID = None

    BASE_CONFIG = loaded
    CONFIG_PATH = path.resolve()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    CAPTURE_FILENAME = str(
        CONFIG_PATH.with_name(f"px1125r_commit_static_{stamp}.bin")
    )
    return loaded


def write_config_with_audit(event: str, details: dict[str, Any]) -> None:
    """
    Append a structured audit record only after a successful commit/verify.

    The original file is copied to <filename>.bak before the JSON update.
    A temporary file is atomically replaced where supported by the OS.
    """
    if CONFIG_PATH is None:
        raise RuntimeError("Configuration has not been loaded.")

    BASE_CONFIG.setdefault("audit_log", [])
    if not isinstance(BASE_CONFIG["audit_log"], list):
        raise ValueError("JSON audit_log exists but is not an array.")

    BASE_CONFIG["audit_log"].append(
        {
            "timestamp_local": local_timestamp(),
            "event": event,
            "details": details,
        }
    )
    BASE_CONFIG["last_updated_local"] = local_timestamp()

    backup_path = CONFIG_PATH.with_name(CONFIG_PATH.name + ".bak")
    shutil.copy2(CONFIG_PATH, backup_path)

    temp_path = CONFIG_PATH.with_name(CONFIG_PATH.name + ".tmp")
    with temp_path.open("w", encoding="utf-8", newline="\n") as fh:
        json.dump(BASE_CONFIG, fh, indent=2)
        fh.write("\n")

    os.replace(temp_path, CONFIG_PATH)
    print(f"\nJSON audit updated: {CONFIG_PATH}")
    print(f"JSON backup created: {backup_path}")


# =============================================================================
# GEODESY / RTCM 1005
# =============================================================================

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


def ecef_to_llh(x_m: float, y_m: float, z_m: float) -> tuple[float, float, float]:
    p = math.hypot(x_m, y_m)
    lon = math.atan2(y_m, x_m)
    lat = math.atan2(z_m, p * (1 - WGS84_E2))

    for _ in range(30):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
        h = p / math.cos(lat) - n
        updated = math.atan2(z_m, p * (1 - WGS84_E2 * n / (n + h)))
        if abs(updated - lat) < 1e-14:
            lat = updated
            break
        lat = updated

    sin_lat = math.sin(lat)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
    h = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), h


def crc24q(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def get_unsigned_bits(data: bytes, bit_offset: int, bit_length: int) -> int:
    value = int.from_bytes(data, "big")
    shift = len(data) * 8 - bit_offset - bit_length
    return (value >> shift) & ((1 << bit_length) - 1)


def sign_extend(value: int, bits: int) -> int:
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


@dataclass(frozen=True)
class RTCM1005:
    station_id: int
    x_m: float
    y_m: float
    z_m: float
    lat_deg: float
    lon_deg: float
    ellipsoid_height_m: float


def decode_rtcm_1005(frame: bytes) -> RTCM1005:
    payload_length = ((frame[1] & 0x03) << 8) | frame[2]
    payload = frame[3:3 + payload_length]

    if len(payload) != 19:
        raise ValueError(f"Expected 19-byte RTCM 1005 payload; got {len(payload)}.")
    if get_unsigned_bits(payload, 0, 12) != 1005:
        raise ValueError("Frame is not RTCM 1005.")

    station_id = get_unsigned_bits(payload, 12, 12)
    x_m = sign_extend(get_unsigned_bits(payload, 34, 38), 38) * 0.0001
    y_m = sign_extend(get_unsigned_bits(payload, 74, 38), 38) * 0.0001
    z_m = sign_extend(get_unsigned_bits(payload, 114, 38), 38) * 0.0001
    lat_deg, lon_deg, h_m = ecef_to_llh(x_m, y_m, z_m)
    return RTCM1005(station_id, x_m, y_m, z_m, lat_deg, lon_deg, h_m)


# =============================================================================
# SKYTRAQ PACKETS / RTK STATUS
# =============================================================================

def hex_bytes(data: bytes, max_bytes: int = 200) -> str:
    if len(data) <= max_bytes:
        return data.hex(" ").upper()
    return data[:max_bytes].hex(" ").upper() + f" ... ({len(data) - max_bytes} more byte(s))"


def xor_checksum(payload: bytes) -> int:
    return reduce(lambda total, value: total ^ value, payload, 0)


def build_skytraq_packet(payload: bytes) -> bytes:
    if not payload:
        raise ValueError("SkyTraq payload may not be empty.")
    return (
        b"\xA0\xA1"
        + len(payload).to_bytes(2, "big")
        + payload
        + bytes([xor_checksum(payload)])
        + b"\x0D\x0A"
    )


@dataclass(frozen=True)
class RTKStatus:
    rtk_mode: int
    saved_operation: int
    saved_survey_seconds: int
    saved_stddev_m: int
    latitude_deg: float
    longitude_deg: float
    ellipsoid_height_m: float
    runtime_operation: int
    runtime_survey_seconds: int

    @property
    def rtk_mode_name(self) -> str:
        return {
            0: "RTK rover",
            1: "RTK base",
            2: "RTK precisely-kinematic base",
        }.get(self.rtk_mode, f"unknown ({self.rtk_mode})")

    @property
    def saved_operation_name(self) -> str:
        return {0: "Kinematic", 1: "Survey", 2: "Static"}.get(
            self.saved_operation, f"unknown ({self.saved_operation})"
        )

    @property
    def runtime_operation_name(self) -> str:
        return {0: "Kinematic/Normal", 1: "Survey", 2: "Static"}.get(
            self.runtime_operation, f"unknown ({self.runtime_operation})"
        )


def parse_rtk_status(payload: bytes) -> RTKStatus:
    if len(payload) != 41 or payload[:2] != b"\x6A\x83":
        raise ValueError("Not a valid SkyTraq 6A 83 RTK status payload.")

    return RTKStatus(
        rtk_mode=payload[2],
        saved_operation=payload[3],
        saved_survey_seconds=struct.unpack(">I", payload[4:8])[0],
        saved_stddev_m=struct.unpack(">I", payload[8:12])[0],
        latitude_deg=struct.unpack(">d", payload[12:20])[0],
        longitude_deg=struct.unpack(">d", payload[20:28])[0],
        ellipsoid_height_m=struct.unpack(">f", payload[28:32])[0],
        runtime_operation=payload[32],
        runtime_survey_seconds=struct.unpack(">I", payload[33:37])[0],
    )


def make_static_commit_payload() -> bytes:
    return (
        bytes([0x6A, 0x06, 0x01, 0x02])  # command, subcommand, base, static
        + struct.pack(">I", STATIC_SURVEY_LENGTH_SECONDS)
        + struct.pack(">I", STATIC_STANDARD_DEVIATION_M)
        + struct.pack(">d", TARGET_LAT_DEG)
        + struct.pack(">d", TARGET_LON_DEG)
        + struct.pack(">f", TARGET_ELLIPSOID_HEIGHT_M)
        + struct.pack(">f", STATIC_BASELINE_CONSTRAINT_M)
        + bytes([COMMIT_ATTRIBUTES])    # SRAM + Flash
    )


# =============================================================================
# MIXED STREAM PARSER
# =============================================================================

class MixedStream:
    def __init__(self) -> None:
        self.sky_buffer = bytearray()
        self.rtcm_buffer = bytearray()
        self.latest_status: Optional[RTKStatus] = None
        self.latest_status_time = 0.0
        self.ack_count = 0
        self.nack_count = 0
        self.rtcm_counts: Counter[int] = Counter()
        self.collect_1005 = False
        self.positions_1005: list[RTCM1005] = []

    def feed(self, data: bytes, context: str) -> None:
        if data:
            self._feed_skytraq(data, context)
            self._feed_rtcm(data, context)

    def _feed_skytraq(self, data: bytes, context: str) -> None:
        self.sky_buffer.extend(data)
        while True:
            start = self.sky_buffer.find(b"\xA0\xA1")
            if start < 0:
                self.sky_buffer[:] = (
                    self.sky_buffer[-1:] if self.sky_buffer[-1:] == b"\xA0" else b""
                )
                return

            if start > 0:
                del self.sky_buffer[:start]

            if len(self.sky_buffer) < 7:
                return

            payload_len = (self.sky_buffer[2] << 8) | self.sky_buffer[3]
            frame_len = 2 + 2 + payload_len + 1 + 2
            if payload_len == 0 or payload_len > 4096:
                del self.sky_buffer[0]
                continue
            if len(self.sky_buffer) < frame_len:
                return

            frame = bytes(self.sky_buffer[:frame_len])
            payload = frame[4:4 + payload_len]
            packet_checksum = frame[4 + payload_len]

            if frame[-2:] != b"\x0D\x0A" or packet_checksum != xor_checksum(payload):
                del self.sky_buffer[0]
                continue

            del self.sky_buffer[:frame_len]
            self._report_skytraq(frame, payload, context)

    def _report_skytraq(self, frame: bytes, payload: bytes, context: str) -> None:
        print("\n" + "=" * 78)
        print(f"[SkyTraq response during {context}]")
        print(f"Raw:     {hex_bytes(frame)}")
        print(f"Payload: {hex_bytes(payload)}")

        if payload[0] == 0x83:
            self.ack_count += 1
            print("Classification: ACK")
            if len(payload) >= 3:
                print(f"ACK for command ID 0x{payload[1]:02X}, sub-ID 0x{payload[2]:02X}.")
        elif payload[0] == 0x84:
            self.nack_count += 1
            print("Classification: NACK — receiver rejected the command.")
            if len(payload) >= 3:
                print(f"NACK for command ID 0x{payload[1]:02X}, sub-ID 0x{payload[2]:02X}.")
        elif len(payload) == 41 and payload[:2] == b"\x6A\x83":
            self.latest_status = parse_rtk_status(payload)
            self.latest_status_time = time.monotonic()
            self.print_status(self.latest_status)
        else:
            print(f"Classification: other valid SkyTraq packet 0x{payload[0]:02X}.")

        print("=" * 78)

    @staticmethod
    def print_status(status: RTKStatus) -> None:
        print("Classification: RTK mode and operational-function status (6A 83)")
        print(f"RTK mode:                {status.rtk_mode_name}")
        print(f"Saved operational state: {status.saved_operation_name}")
        print(f"Saved survey length:     {status.saved_survey_seconds} second(s)")
        print(f"Saved std. deviation:    {status.saved_stddev_m} m")
        print(f"Run-time operational:    {status.runtime_operation_name}")
        print(f"Run-time survey length:  {status.runtime_survey_seconds} second(s)")
        print("Coordinate contained in status reply:")
        print(f"  Lat:                   {status.latitude_deg:.12f} deg")
        print(f"  Lon:                   {status.longitude_deg:.12f} deg")
        print(f"  Ellipsoid height:      {status.ellipsoid_height_m:.6f} m")

    def _feed_rtcm(self, data: bytes, context: str) -> None:
        self.rtcm_buffer.extend(data)

        while True:
            start = self.rtcm_buffer.find(b"\xD3")
            if start < 0:
                self.rtcm_buffer.clear()
                return
            if start > 0:
                del self.rtcm_buffer[:start]
            if len(self.rtcm_buffer) < 6:
                return

            if self.rtcm_buffer[1] & 0xFC:
                del self.rtcm_buffer[0]
                continue

            payload_len = ((self.rtcm_buffer[1] & 0x03) << 8) | self.rtcm_buffer[2]
            frame_len = 3 + payload_len + 3
            if len(self.rtcm_buffer) < frame_len:
                return

            frame = bytes(self.rtcm_buffer[:frame_len])
            if crc24q(frame[:-3]) != int.from_bytes(frame[-3:], "big"):
                del self.rtcm_buffer[0]
                continue

            del self.rtcm_buffer[:frame_len]
            if payload_len < 2:
                continue

            msg_type = ((frame[3] << 4) | (frame[4] >> 4)) & 0x0FFF
            self.rtcm_counts[msg_type] += 1

            if self.collect_1005 and msg_type == 1005:
                position = decode_rtcm_1005(frame)
                self.positions_1005.append(position)
                print(
                    f"[Verified RTCM 1005 #{len(self.positions_1005)}] "
                    f"lat={position.lat_deg:.12f}, "
                    f"lon={position.lon_deg:.12f}, "
                    f"h={position.ellipsoid_height_m:.4f} m"
                )


# =============================================================================
# SERIAL CONTROL / VALIDATION
# =============================================================================

def read_for(
    ser: serial.Serial,
    stream: MixedStream,
    seconds: float,
    context: str,
    raw_file,
) -> int:
    end = time.monotonic() + seconds
    total = 0
    while time.monotonic() < end:
        waiting = ser.in_waiting
        data = ser.read(waiting if waiting else 1)
        if not data:
            continue
        total += len(data)
        raw_file.write(data)
        raw_file.flush()
        stream.feed(data, context)
    return total


def send_command(
    ser: serial.Serial,
    stream: MixedStream,
    raw_file,
    label: str,
    payload: bytes,
    response_seconds: float = COMMAND_RESPONSE_SECONDS,
) -> None:
    packet = build_skytraq_packet(payload)
    print("\n" + "#" * 78)
    print(f"COMMAND: {label}")
    print(f"Payload: {hex_bytes(payload)}")
    print(f"TX raw:  {hex_bytes(packet)}")
    print("#" * 78)

    ser.write(packet)
    ser.flush()
    print(f"Observing raw replies for {response_seconds:.1f} seconds...")
    received = read_for(ser, stream, response_seconds, f"response to {label}", raw_file)
    print(f"Response window complete; received {received} byte(s).")


def query_status(
    ser: serial.Serial,
    stream: MixedStream,
    raw_file,
) -> Optional[RTKStatus]:
    prior_time = stream.latest_status_time
    send_command(
        ser,
        stream,
        raw_file,
        "Query RTK mode and operational function [0x6A / 0x07]",
        bytes([0x6A, 0x07]),
    )
    if stream.latest_status is not None and stream.latest_status_time > prior_time:
        return stream.latest_status
    print("WARNING: No valid 6A 83 RTK status response was received.")
    return None


def close_to_target(lat: float, lon: float, h: float, *, precommit: bool) -> bool:
    lat_lon_tol = (
        RUNTIME_LAT_LON_TOLERANCE_DEG if precommit else VERIFY_LAT_LON_TOLERANCE_DEG
    )
    h_tol = RUNTIME_HEIGHT_TOLERANCE_M if precommit else VERIFY_HEIGHT_TOLERANCE_M
    return (
        abs(lat - TARGET_LAT_DEG) <= lat_lon_tol
        and abs(lon - TARGET_LON_DEG) <= lat_lon_tol
        and abs(h - TARGET_ELLIPSOID_HEIGHT_M) <= h_tol
    )


def print_target() -> None:
    print("\nJSON TARGET FIXED BASE COORDINATE")
    print(f"  File:              {CONFIG_PATH}")
    print(f"  Latitude:          {TARGET_LAT_DEG:.12f} deg")
    print(f"  Longitude:         {TARGET_LON_DEG:.12f} deg")
    print(f"  Ellipsoid height:  {TARGET_ELLIPSOID_HEIGHT_M:.7f} m")
    print(f"  Station ID:        {TARGET_STATION_ID if TARGET_STATION_ID is not None else 'not checked'}")
    print("  Datum:             WGS 84")
    print("  Height type:       WGS-84 ellipsoid height, not MSL")


def verify_1005(
    ser: serial.Serial,
    stream: MixedStream,
    raw_file,
) -> tuple[bool, Optional[RTCM1005]]:
    stream.positions_1005.clear()
    stream.collect_1005 = True

    print("\n" + "*" * 78)
    print(f"COLLECTING {REQUIRED_1005_FRAMES} VALID RTCM 1005 FRAMES FOR VERIFICATION")
    print("*" * 78)

    deadline = time.monotonic() + POST_COMMIT_1005_TIMEOUT_SECONDS
    while (
        time.monotonic() < deadline
        and len(stream.positions_1005) < REQUIRED_1005_FRAMES
    ):
        read_for(
            ser,
            stream,
            min(1.0, deadline - time.monotonic()),
            "RTCM 1005 verification",
            raw_file,
        )

    stream.collect_1005 = False

    if len(stream.positions_1005) < REQUIRED_1005_FRAMES:
        print(
            f"\nFAIL: Received only {len(stream.positions_1005)} valid 1005 frame(s); "
            f"needed {REQUIRED_1005_FRAMES}."
        )
        return False, None

    first = stream.positions_1005[0]
    stable = all(
        abs(item.x_m - first.x_m) <= 0.001
        and abs(item.y_m - first.y_m) <= 0.001
        and abs(item.z_m - first.z_m) <= 0.001
        for item in stream.positions_1005[1:]
    )
    if not stable:
        print("\nFAIL: Repeat RTCM 1005 coordinates were not stable.")
        return False, first

    station_ok = TARGET_STATION_ID is None or first.station_id == TARGET_STATION_ID
    coordinate_ok = close_to_target(
        first.lat_deg, first.lon_deg, first.ellipsoid_height_m, precommit=False
    )

    print("\nRTCM 1005 REPRESENTATIVE POSITION")
    print(f"  Station ID:        {first.station_id}")
    print(f"  ECEF X:            {first.x_m:.4f} m")
    print(f"  ECEF Y:            {first.y_m:.4f} m")
    print(f"  ECEF Z:            {first.z_m:.4f} m")
    print(f"  Latitude:          {first.lat_deg:.12f} deg")
    print(f"  Longitude:         {first.lon_deg:.12f} deg")
    print(f"  Ellipsoid height:  {first.ellipsoid_height_m:.4f} m")

    if not station_ok:
        print(
            f"\nFAIL: RTCM 1005 station ID {first.station_id} does not match "
            f"JSON base.reference_station_id {TARGET_STATION_ID}."
        )
        return False, first

    if not coordinate_ok:
        print(
            "\nFAIL: RTCM 1005 coordinate does not agree with the JSON target.\n"
            f"  latitude difference: {(first.lat_deg - TARGET_LAT_DEG):+.12f} deg\n"
            f"  longitude difference:{(first.lon_deg - TARGET_LON_DEG):+.12f} deg\n"
            f"  height difference:   {(first.ellipsoid_height_m - TARGET_ELLIPSOID_HEIGHT_M):+.4f} m"
        )
        return False, first

    print("\nPASS: Repeat 1005 coordinate is stable and agrees with JSON.")
    return True, first


def status_to_dict(status: RTKStatus) -> dict[str, Any]:
    return {
        "rtk_mode": status.rtk_mode_name,
        "saved_operational_state": status.saved_operation_name,
        "runtime_operational_state": status.runtime_operation_name,
        "saved_survey_length_seconds": status.saved_survey_seconds,
        "saved_standard_deviation_m": status.saved_stddev_m,
        "runtime_survey_length_seconds": status.runtime_survey_seconds,
        "latitude_deg": status.latitude_deg,
        "longitude_deg": status.longitude_deg,
        "ellipsoid_height_m": status.ellipsoid_height_m,
    }


def position_to_dict(position: RTCM1005) -> dict[str, Any]:
    return {
        "reference_station_id": position.station_id,
        "latitude_deg": position.lat_deg,
        "longitude_deg": position.lon_deg,
        "ellipsoid_height_m": position.ellipsoid_height_m,
        "ecef_m": {
            "x": position.x_m,
            "y": position.y_m,
            "z": position.z_m,
        },
    }


# =============================================================================
# MAIN
# =============================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Commit or verify a PX1125R fixed static base from JSON."
    )
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--commit", action="store_true", help="Write target to SRAM + Flash.")
    modes.add_argument("--verify", action="store_true", help="Verify saved target after manual power-cycle.")
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="Path to base-coordinate JSON. Defaults to px1125r_base_config.json next to this script.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    mode = "commit" if args.commit else "verify" if args.verify else "preview"

    try:
        load_base_config(Path(args.config).expanduser())
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"CONFIGURATION ERROR: {exc}")
        return 2

    print("=" * 78)
    print("PX1125R COMMIT SURVEYED COORDINATE AS FIXED STATIC RTK BASE")
    print("=" * 78)
    print(f"Mode:               {mode.upper()}")
    print(f"Port:               {PORT}")
    print(f"Baud:               {BAUD}")
    print(f"Raw capture:        {CAPTURE_FILENAME}")
    print_target()
    print("=" * 78)

    if mode == "preview":
        print(
            "\nPREVIEW ONLY: No receiver command will be sent.\n"
            "Review the JSON record. To commit its coordinate to SRAM + Flash:\n\n"
            f"  python {Path(__file__).name} --commit --config \"{CONFIG_PATH}\"\n\n"
            "After a manual power-cycle, verify persistence:\n\n"
            f"  python {Path(__file__).name} --verify --config \"{CONFIG_PATH}\""
        )
        return 0

    if mode == "commit":
        print(
            "\nCOMMIT MODE WARNING:\n"
            "This writes RTK Base / Static configuration and the JSON coordinate\n"
            "to both SRAM and Flash. It replaces the receiver's saved static-base\n"
            "coordinate and saved RTK operational mode."
        )
        phrase = input("Type exactly: COMMIT STATIC BASE  -> ").strip()
        if phrase != "COMMIT STATIC BASE":
            print("Cancelled. No receiver command was sent.")
            return 0

    stream = MixedStream()
    ser: Optional[serial.Serial] = None

    try:
        print(f"\nOpening {PORT} at {BAUD} baud...")
        ser = serial.Serial(
            PORT,
            BAUD,
            timeout=SERIAL_TIMEOUT_SECONDS,
            write_timeout=2,
        )
        print("Serial port opened. Waiting 1 second for link stabilization...")
        time.sleep(1.0)
        ser.reset_input_buffer()

        with open(CAPTURE_FILENAME, "wb") as raw_file:
            status_before = query_status(ser, stream, raw_file)
            if status_before is None:
                print("\nFAIL: Cannot safely continue without a 6A 83 RTK status response.")
                return 10

            if mode == "verify":
                status_ok = (
                    status_before.rtk_mode == 1
                    and status_before.saved_operation == 2
                    and status_before.runtime_operation == 2
                    and close_to_target(
                        status_before.latitude_deg,
                        status_before.longitude_deg,
                        status_before.ellipsoid_height_m,
                        precommit=False,
                    )
                )
                if not status_ok:
                    print(
                        "\nFAIL: Receiver status does not show expected RTK Base / Static "
                        "with the JSON target coordinate."
                    )
                    return 11

                print("\nPASS: 6A 83 status confirms saved RTK Base / Static JSON target.")
                time.sleep(COMMAND_GAP_SECONDS)
                success, position = verify_1005(ser, stream, raw_file)
                if not success or position is None:
                    return 12

                write_config_with_audit(
                    "flash_persistence_verified",
                    {
                        "capture_file": CAPTURE_FILENAME,
                        "status": status_to_dict(status_before),
                        "rtcm_1005": position_to_dict(position),
                        "rtcm_1005_frames_checked": REQUIRED_1005_FRAMES,
                    },
                )

                print("\n" + "=" * 78)
                print("PERSISTENCE VERIFICATION PASSED")
                print("The PX1125R retained the JSON fixed static base coordinate after power-cycle.")
                print("=" * 78)
                return 0

            # Commit-mode guardrails:
            completed_survey_active = (
                status_before.rtk_mode == 1
                and status_before.saved_operation == 1
                and status_before.runtime_operation == 2
            )
            runtime_matches_json = close_to_target(
                status_before.latitude_deg,
                status_before.longitude_deg,
                status_before.ellipsoid_height_m,
                precommit=True,
            )

            if not completed_survey_active:
                print(
                    "\nFAIL: Safety precondition is not met.\n"
                    "Expected current state: RTK Base, saved Survey, run-time Static.\n"
                    "This prevents committing JSON coordinates unless the receiver is\n"
                    "currently holding a completed survey result."
                )
                return 20

            if not runtime_matches_json:
                print(
                    "\nFAIL: Current run-time survey coordinate does not agree closely "
                    "enough with the JSON target. No Flash write was sent.\n"
                    f"  latitude difference: {(status_before.latitude_deg - TARGET_LAT_DEG):+.12f} deg\n"
                    f"  longitude difference:{(status_before.longitude_deg - TARGET_LON_DEG):+.12f} deg\n"
                    f"  height difference:   {(status_before.ellipsoid_height_m - TARGET_ELLIPSOID_HEIGHT_M):+.6f} m"
                )
                return 21

            print(
                "\nPRE-COMMIT SAFETY CHECK PASSED:\n"
                "  • Receiver is RTK Base.\n"
                "  • Saved mode is Survey.\n"
                "  • Run-time survey has completed and is Static.\n"
                "  • Current run-time coordinate agrees with JSON.\n"
            )

            print(f"Intentional delay before Flash write: {COMMAND_GAP_SECONDS:.1f} seconds...")
            time.sleep(COMMAND_GAP_SECONDS)

            send_command(
                ser,
                stream,
                raw_file,
                "COMMIT RTK Base / Static JSON coordinate to SRAM + Flash [0x6A / 0x06]",
                make_static_commit_payload(),
            )
            if stream.nack_count:
                print("\nFAIL: Receiver returned a NACK. Commit did not complete.")
                return 22

            print(f"\nWaiting {COMMAND_GAP_SECONDS:.1f} seconds before re-querying...")
            time.sleep(COMMAND_GAP_SECONDS)
            status_after = query_status(ser, stream, raw_file)
            if status_after is None:
                print("\nFAIL: No post-commit RTK status response.")
                return 23

            status_ok = (
                status_after.rtk_mode == 1
                and status_after.saved_operation == 2
                and status_after.runtime_operation == 2
                and close_to_target(
                    status_after.latitude_deg,
                    status_after.longitude_deg,
                    status_after.ellipsoid_height_m,
                    precommit=False,
                )
            )
            if not status_ok:
                print(
                    "\nFAIL: Post-commit status does not confirm RTK Base / Static "
                    "with the JSON target."
                )
                return 24

            print("\nPASS: 6A 83 status confirms RTK Base / Static JSON target.")
            time.sleep(COMMAND_GAP_SECONDS)

            success, position = verify_1005(ser, stream, raw_file)
            if not success or position is None:
                return 25

            write_config_with_audit(
                "static_coordinate_committed_to_flash",
                {
                    "capture_file": CAPTURE_FILENAME,
                    "status_before_commit": status_to_dict(status_before),
                    "status_after_commit": status_to_dict(status_after),
                    "rtcm_1005": position_to_dict(position),
                    "rtcm_1005_frames_checked": REQUIRED_1005_FRAMES,
                },
            )

            print("\n" + "=" * 78)
            print("COMMIT AND LIVE VERIFICATION PASSED")
            print("=" * 78)
            print(
                "The receiver reports RTK Base / Static and RTCM 1005 agrees with JSON.\n"
                "For final persistence proof, power-cycle it manually, then run:\n\n"
                f"  python {Path(__file__).name} --verify --config \"{CONFIG_PATH}\""
            )
            return 0

    except serial.SerialException as exc:
        print(f"\nSERIAL ERROR: {exc}")
        print("Close GNSS Viewer and other serial consumers; then confirm PORT / BAUD.")
        return 30
    except KeyboardInterrupt:
        print(
            "\nInterrupted. If this happened after the commit command was ACKed, "
            "power-cycle the receiver and run --verify before trusting its state."
        )
        return 130
    except OSError as exc:
        print(f"\nFILE ERROR: {exc}")
        return 31
    finally:
        if ser is not None and ser.is_open:
            ser.close()
            print("\nSerial port closed.")

        print("\nRTCM message counts observed in this run:")
        if stream.rtcm_counts:
            for msg_type, count in sorted(stream.rtcm_counts.items()):
                print(f"  Type {msg_type:4d}: {count}")
        else:
            print("  No valid RTCM frames decoded.")

        if CAPTURE_FILENAME:
            print(f"\nRaw capture saved to: {CAPTURE_FILENAME}")


if __name__ == "__main__":
    sys.exit(main())
