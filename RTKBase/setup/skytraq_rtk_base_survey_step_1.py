#!/usr/bin/env python3
"""
SkyTraq RTK Base Survey -> RTCM 1005 -> JSON Candidate Record

Progress-monitor revision: polls run-time survey length and aborts on no progress.

What this script does
---------------------
1. Queries and displays the receiver's current RTK base state.
2. Changes the SkyTraq RTK receiver to RTK BASE + SURVEY mode for SURVEY_SECONDS.
3. Logs every byte received from the receiver to a raw .bin capture.
4. Polls the receiver's RTK run-time state until the survey completes.
5. After the receiver reports run-time STATIC, collects repeat RTCM 1005
   messages and reports the surveyed position as:
       - latitude / longitude
       - WGS-84 ellipsoid height
       - ECEF X / Y / Z

Important behavior
------------------
- This test uses SRAM ONLY. It does NOT write configuration to Flash.
- When the survey completes, the receiver should hold the derived coordinate
  in run-time Static mode for the current powered session.
- After a power-cycle, it may resume its saved configuration rather than
  retain this surveyed coordinate. That is intentional for the testing phase.
- Do NOT run this while another process owns the serial port.
- Stop forwarding raw serial bytes to a rover during this test. The Pi-side
  forwarding service should later forward only CRC-valid RTCM frames.

Protocol used
-------------
SkyTraq Phoenix:
  Configure RTK Mode and Operational Function: 0x6A / 0x06
  Query RTK Mode and Operational Function:     0x6A / 0x07
  Query response:                              0x6A / 0x83

Usage
-----
Windows:
    python px1125r_base_survey.py

Raspberry Pi:
    Set PORT = "/dev/gps" or "/dev/ttyUSB0", then:
    python3 px1125r_base_survey.py
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import struct
import sys
import time
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from functools import reduce
from typing import Callable, Optional

import serial


# =============================================================================
# USER SETTINGS
# =============================================================================

PORT = "COM6"                  # Example Windows port; Pi example: "/dev/gps"
BAUD = 115200

# SkyTraq supports 60 through 1,209,600 seconds. Start at 60 for testing.
# For a 24-hour survey later, set this to 86400.
SURVEY_SECONDS = 60

# SkyTraq protocol: allowed range 3 through 100 m. The vendor example uses 30.
# This is a survey-mode parameter, not a claim of final RTK accuracy.
STANDARD_DEVIATION_METERS = 30

# 0 = SRAM only. Keep this at 0 for testing.
# Do NOT change to 1 until you explicitly want to persist configuration.
ATTRIBUTES = 0

# Default survey-monitor settings.  The maximum is calculated as
# max(--duration + --post-survey-grace-seconds, --minimum-monitor-timeout-seconds)
# so a short functional test receives a minutes-scale timeout while a long survey
# remains allowed to run for its requested duration.
DEFAULT_STATUS_POLL_SECONDS = 20.0
DEFAULT_POST_SURVEY_GRACE_SECONDS = 240
DEFAULT_MINIMUM_MONITOR_TIMEOUT_SECONDS = 300
DEFAULT_MAX_STALLED_POLLS = 3
DEFAULT_MAX_MISSING_STATUS_POLLS = 3
COMMAND_RESPONSE_SECONDS = 2.5
SERIAL_TIMEOUT_SECONDS = 0.20

# After survey completion, collect repeat 1005 messages to ensure the
# advertised coordinates are stable.
REQUIRED_1005_FRAMES = 3
POST_STATIC_CAPTURE_TIMEOUT_SECONDS = 20

# The raw .bin filename and JSON candidate filename are generated in main()
# from the command-line label and timestamp.


# =============================================================================
# GEODESY / RTCM HELPERS
# =============================================================================

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


def ecef_to_llh(x_m: float, y_m: float, z_m: float) -> tuple[float, float, float]:
    """Convert WGS-84 ECEF meters to latitude degrees, longitude degrees, ellipsoid height."""
    p = math.hypot(x_m, y_m)
    lon = math.atan2(y_m, x_m)
    lat = math.atan2(z_m, p * (1 - WGS84_E2))

    for _ in range(20):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
        h = p / math.cos(lat) - n
        new_lat = math.atan2(z_m, p * (1 - WGS84_E2 * n / (n + h)))
        if abs(new_lat - lat) < 1e-14:
            lat = new_lat
            break
        lat = new_lat

    sin_lat = math.sin(lat)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat * sin_lat)
    h = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), h


def crc24q(data: bytes) -> int:
    """RTCM 3 CRC-24Q."""
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def get_unsigned_bits(data: bytes, bit_offset: int, bit_length: int) -> int:
    """Extract a big-endian RTCM bit field."""
    value = int.from_bytes(data, "big")
    total_bits = len(data) * 8
    shift = total_bits - bit_offset - bit_length
    return (value >> shift) & ((1 << bit_length) - 1)


def sign_extend(value: int, bits: int) -> int:
    """Interpret unsigned value as signed two's complement."""
    if value & (1 << (bits - 1)):
        return value - (1 << bits)
    return value


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
    """Decode a CRC-valid RTCM 1005 frame."""
    payload_length = ((frame[1] & 0x03) << 8) | frame[2]
    payload = frame[3:3 + payload_length]

    if len(payload) != 19:
        raise ValueError(f"RTCM 1005 payload must be 19 bytes; got {len(payload)}.")

    msg_type = get_unsigned_bits(payload, 0, 12)
    if msg_type != 1005:
        raise ValueError(f"Expected RTCM 1005, got {msg_type}.")

    station_id = get_unsigned_bits(payload, 12, 12)
    x_raw = sign_extend(get_unsigned_bits(payload, 34, 38), 38)
    y_raw = sign_extend(get_unsigned_bits(payload, 74, 38), 38)
    z_raw = sign_extend(get_unsigned_bits(payload, 114, 38), 38)

    x_m = x_raw * 0.0001
    y_m = y_raw * 0.0001
    z_m = z_raw * 0.0001
    lat_deg, lon_deg, h_m = ecef_to_llh(x_m, y_m, z_m)

    return RTCM1005(
        station_id=station_id,
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        ellipsoid_height_m=h_m,
    )


# =============================================================================
# SKYTRAQ PACKET HELPERS
# =============================================================================

def hex_bytes(data: bytes, limit: int = 200) -> str:
    """Readable hex, capped to keep console output manageable."""
    if len(data) <= limit:
        return data.hex(" ").upper()
    return data[:limit].hex(" ").upper() + f" ... ({len(data) - limit} more byte(s))"


def xor_checksum(payload: bytes) -> int:
    return reduce(lambda accum, byte: accum ^ byte, payload, 0)


def skytraq_packet(payload: bytes) -> bytes:
    """Build A0 A1 ... checksum ... 0D 0A packet."""
    return (
        b"\xA0\xA1"
        + len(payload).to_bytes(2, "big")
        + payload
        + bytes([xor_checksum(payload)])
        + b"\x0D\x0A"
    )


@dataclass
class RTKStatus:
    rtk_mode: int
    saved_operational_function: int
    saved_survey_seconds: int
    standard_deviation_m: int
    latitude_deg: float
    longitude_deg: float
    ellipsoid_height_m: float
    runtime_operational_function: int
    runtime_survey_seconds: int

    @property
    def rtk_mode_name(self) -> str:
        return {
            0: "RTK rover",
            1: "RTK base",
            2: "RTK precisely-kinematic base",
        }.get(self.rtk_mode, f"unknown ({self.rtk_mode})")

    @property
    def saved_function_name(self) -> str:
        return {0: "Kinematic", 1: "Survey", 2: "Static"}.get(
            self.saved_operational_function,
            f"unknown ({self.saved_operational_function})",
        )

    @property
    def runtime_function_name(self) -> str:
        return {0: "Kinematic/Normal", 1: "Survey", 2: "Static"}.get(
            self.runtime_operational_function,
            f"unknown ({self.runtime_operational_function})",
        )


def parse_rtk_status(payload: bytes) -> RTKStatus:
    """
    Parse SkyTraq RTK mode response:
      6A 83 [rtk mode] [saved op] [survey seconds] [stddev]
      [lat double] [lon double] [height float] [baseline float]
      [runtime op] [runtime survey seconds] [reserved]
    """
    if len(payload) != 41 or payload[0] != 0x6A or payload[1] != 0x83:
        raise ValueError(
            "Unexpected RTK status payload; expected 41 bytes beginning 6A 83."
        )

    return RTKStatus(
        rtk_mode=payload[2],
        saved_operational_function=payload[3],
        saved_survey_seconds=struct.unpack(">I", payload[4:8])[0],
        standard_deviation_m=struct.unpack(">I", payload[8:12])[0],
        latitude_deg=struct.unpack(">d", payload[12:20])[0],
        longitude_deg=struct.unpack(">d", payload[20:28])[0],
        ellipsoid_height_m=struct.unpack(">f", payload[28:32])[0],
        # The run-time fields begin immediately after the 4-byte baseline
        # constraint at payload offset 32.  In a 6A 83 response:
        #   payload[32]    = run-time operational function
        #   payload[33:37] = run-time survey duration (uint32, big-endian)
        #   payload[37:41] = reserved
        runtime_operational_function=payload[32],
        runtime_survey_seconds=struct.unpack(">I", payload[33:37])[0],
    )


# =============================================================================
# MIXED-STREAM PARSER
# =============================================================================

class MixedStreamParser:
    """
    Parses SkyTraq packets and CRC-valid RTCM frames from one serial stream.

    The receiver may interleave proprietary command replies and RTCM output.
    The raw capture remains untouched; this parser is only for observation.
    """

    def __init__(self) -> None:
        self.sky_buffer = bytearray()
        self.rtcm_buffer = bytearray()
        self.latest_rtk_status: Optional[RTKStatus] = None
        self.latest_rtk_status_time: float = 0.0
        self.rtcm_counter: Counter[int] = Counter()
        self.rtcm_1005_after_static: list[RTCM1005] = []
        self.collect_1005 = False
        self.sky_ack_count = 0
        self.sky_nack_count = 0

    def feed(self, data: bytes, context: str) -> None:
        if not data:
            return
        self._feed_skytraq(data, context)
        self._feed_rtcm(data, context)

    def _feed_skytraq(self, data: bytes, context: str) -> None:
        self.sky_buffer.extend(data)

        while True:
            start = self.sky_buffer.find(b"\xA0\xA1")
            if start < 0:
                self.sky_buffer[:] = self.sky_buffer[-1:] if self.sky_buffer[-1:] == b"\xA0" else b""
                return

            if start > 0:
                del self.sky_buffer[:start]

            if len(self.sky_buffer) < 7:
                return

            payload_length = (self.sky_buffer[2] << 8) | self.sky_buffer[3]
            total_length = 2 + 2 + payload_length + 1 + 2

            if payload_length == 0 or payload_length > 4096:
                del self.sky_buffer[0]
                continue

            if len(self.sky_buffer) < total_length:
                return

            raw = bytes(self.sky_buffer[:total_length])
            payload = raw[4:4 + payload_length]
            checksum = raw[4 + payload_length]

            if raw[-2:] != b"\x0D\x0A" or checksum != xor_checksum(payload):
                del self.sky_buffer[0]
                continue

            del self.sky_buffer[:total_length]
            self._on_skytraq_packet(raw, payload, context)

    def _on_skytraq_packet(self, raw: bytes, payload: bytes, context: str) -> None:
        print("\n" + "=" * 78)
        print(f"[SkyTraq response during {context}]")
        print(f"Raw:     {hex_bytes(raw)}")
        print(f"Payload: {hex_bytes(payload)}")

        if payload[0] == 0x83:
            self.sky_ack_count += 1
            print("Classification: ACK")
            if len(payload) >= 3:
                print(f"ACK for command ID 0x{payload[1]:02X}, sub-ID 0x{payload[2]:02X}.")
        elif payload[0] == 0x84:
            self.sky_nack_count += 1
            print("Classification: NACK — receiver rejected the command.")
            if len(payload) >= 3:
                print(f"NACK for command ID 0x{payload[1]:02X}, sub-ID 0x{payload[2]:02X}.")
        elif len(payload) == 41 and payload[:2] == b"\x6A\x83":
            try:
                self.latest_rtk_status = parse_rtk_status(payload)
                self.latest_rtk_status_time = time.monotonic()
                self.print_rtk_status(self.latest_rtk_status)
            except Exception as exc:
                print(f"Classification: RTK status response, but parsing failed: {exc}")
        else:
            print(f"Classification: other valid SkyTraq packet (message ID 0x{payload[0]:02X}).")

        print("=" * 78)

    @staticmethod
    def print_rtk_status(status: RTKStatus) -> None:
        print("Classification: RTK mode and operational-function status (6A 83)")
        print(f"RTK mode:                {status.rtk_mode_name}")
        print(f"Saved operational state: {status.saved_function_name}")
        print(f"Saved survey length:     {status.saved_survey_seconds} second(s)")
        print(f"Survey std. deviation:   {status.standard_deviation_m} m")
        print(f"Run-time operational:    {status.runtime_function_name}")
        print(f"Run-time survey length:  {status.runtime_survey_seconds} second(s)")
        print("Saved/run-time coordinate fields reported by receiver:")
        print(f"  Lat:                   {status.latitude_deg:.9f} deg")
        print(f"  Lon:                   {status.longitude_deg:.9f} deg")
        print(f"  Ellipsoid height:      {status.ellipsoid_height_m:.3f} m")

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

            payload_length = ((self.rtcm_buffer[1] & 0x03) << 8) | self.rtcm_buffer[2]
            total_length = 3 + payload_length + 3

            if len(self.rtcm_buffer) < total_length:
                return

            frame = bytes(self.rtcm_buffer[:total_length])
            if crc24q(frame[:-3]) != int.from_bytes(frame[-3:], "big"):
                del self.rtcm_buffer[0]
                continue

            del self.rtcm_buffer[:total_length]

            if payload_length < 2:
                continue

            message_type = ((frame[3] << 4) | (frame[4] >> 4)) & 0x0FFF
            self.rtcm_counter[message_type] += 1

            if self.collect_1005 and message_type == 1005:
                try:
                    position = decode_rtcm_1005(frame)
                    self.rtcm_1005_after_static.append(position)
                    n = len(self.rtcm_1005_after_static)
                    print(
                        f"[RTCM 1005 after reported Static state #{n}] "
                        f"lat={position.lat_deg:.9f}, lon={position.lon_deg:.9f}, "
                        f"h={position.ellipsoid_height_m:.3f} m"
                    )
                except Exception as exc:
                    print(f"[RTCM 1005] decode failure: {exc}")


# =============================================================================
# SERIAL / COMMAND FUNCTIONS
# =============================================================================

def read_for(
    ser: serial.Serial,
    parser: MixedStreamParser,
    seconds: float,
    context: str,
    raw_file,
) -> int:
    """Read serial bytes for a given period, log them raw, and parse passively."""
    end = time.monotonic() + seconds
    received = 0

    while time.monotonic() < end:
        waiting = ser.in_waiting
        data = ser.read(waiting if waiting else 1)
        if not data:
            continue

        received += len(data)
        raw_file.write(data)
        raw_file.flush()
        parser.feed(data, context)

    return received


def send_command(
    ser: serial.Serial,
    parser: MixedStreamParser,
    raw_file,
    label: str,
    payload: bytes,
    response_seconds: float = COMMAND_RESPONSE_SECONDS,
) -> None:
    """Send one command and record/disclose all observed replies."""
    packet = skytraq_packet(payload)

    print("\n" + "#" * 78)
    print(f"COMMAND: {label}")
    print(f"Payload: {hex_bytes(payload)}")
    print(f"TX raw:  {hex_bytes(packet)}")
    print("#" * 78)

    ser.write(packet)
    ser.flush()
    received = read_for(ser, parser, response_seconds, f"response to {label}", raw_file)
    print(f"Command response window complete; received {received} byte(s).")


def query_rtk_status(
    ser: serial.Serial,
    parser: MixedStreamParser,
    raw_file,
) -> Optional[RTKStatus]:
    """Send 6A 07, then return a fresh RTK status response if received."""
    previous_time = parser.latest_rtk_status_time
    send_command(
        ser,
        parser,
        raw_file,
        "Query RTK mode and operational function [0x6A / 0x07]",
        bytes([0x6A, 0x07]),
    )
    if parser.latest_rtk_status is not None and parser.latest_rtk_status_time > previous_time:
        return parser.latest_rtk_status

    print("WARNING: No parseable 6A 83 RTK status response was observed.")
    return None


def survey_seed_from_prior_status(
    status: Optional[RTKStatus],
) -> tuple[float, float, float, str]:
    """
    Return the coordinate fields to carry into a new Base/Survey command.

    SkyTraq's protocol documentation marks latitude, longitude, and height
    as "not used" in Survey mode.  GNSS Viewer nevertheless preserves the
    last run-time Static coordinate in fields 13–32 when it starts a new
    PX1172RH survey.  On a PX1172RH this behavior empirically completed a
    60-second survey, whereas an all-zero payload did not.

    To reproduce the viewer's actual command, carry the coordinate only when
    the initial 6A 83 status says the receiver is currently RTK Base +
    run-time Static and the coordinate is valid.  Otherwise use zero fields,
    which remains protocol-compliant and preserves the earlier script behavior.
    """
    if status is None:
        return 0.0, 0.0, 0.0, "No initial RTK status; using zero coordinate fields."

    valid = (
        status.rtk_mode == 1
        and status.runtime_operational_function == 2
        and math.isfinite(status.latitude_deg)
        and math.isfinite(status.longitude_deg)
        and math.isfinite(status.ellipsoid_height_m)
        and not (
            status.latitude_deg == 0.0
            and status.longitude_deg == 0.0
            and status.ellipsoid_height_m == 0.0
        )
    )
    if valid:
        return (
            status.latitude_deg,
            status.longitude_deg,
            status.ellipsoid_height_m,
            "Copied prior run-time Static coordinate to match GNSS Viewer behavior.",
        )

    return (
        0.0,
        0.0,
        0.0,
        "Initial status was not a valid run-time Static coordinate; using zero coordinate fields.",
    )


def make_rtk_base_survey_payload(
    survey_seconds: int,
    standard_deviation_m: int,
    attributes: int,
    seed_latitude_deg: float,
    seed_longitude_deg: float,
    seed_ellipsoid_height_m: float,
) -> bytes:
    """
    Build 6A 06 payload:
      6A 06
      RTK mode = 1 (base)
      operation = 1 (survey)
      survey seconds (uint32)
      standard deviation meters (uint32)
      latitude / longitude / height copied from a prior run-time Static state
      baseline constraint = ignored in base survey, zero
      attributes = 0 (SRAM only)

    The official protocol says fields 13–32 are not used in Survey mode.
    Retaining a prior Static coordinate is therefore harmless on compliant
    firmware and reproduces the actual GNSS Viewer command observed on the
    PX1172RH.
    """
    if not 60 <= survey_seconds <= 1_209_600:
        raise ValueError("SURVEY_SECONDS must be between 60 and 1,209,600.")
    if not 3 <= standard_deviation_m <= 100:
        raise ValueError("STANDARD_DEVIATION_METERS must be between 3 and 100.")
    if attributes != 0:
        raise ValueError(
            "Safety check: ATTRIBUTES must remain 0 for this test (SRAM only)."
        )

    return (
        bytes([0x6A, 0x06, 0x01, 0x01])  # message, sub-ID, base, survey
        + struct.pack(">I", survey_seconds)
        + struct.pack(">I", standard_deviation_m)
        + struct.pack(">d", seed_latitude_deg)
        + struct.pack(">d", seed_longitude_deg)
        + struct.pack(">f", seed_ellipsoid_height_m)
        + struct.pack(">f", 0.0)         # baseline constraint unused in base survey
        + bytes([attributes])
    )


def stable_1005_position(frames: list[RTCM1005]) -> Optional[RTCM1005]:
    """
    Return representative position only if repeat 1005 ECEF values agree.

    Since the receiver has transitioned to Static, repeat 1005 values should
    be identical at 0.1 mm resolution. A 1 mm tolerance permits harmless
    packet / representation differences while rejecting a moving value.
    """
    if len(frames) < REQUIRED_1005_FRAMES:
        return None

    first = frames[0]
    tolerance_m = 0.001
    for item in frames[1:]:
        if (
            abs(item.x_m - first.x_m) > tolerance_m
            or abs(item.y_m - first.y_m) > tolerance_m
            or abs(item.z_m - first.z_m) > tolerance_m
        ):
            return None
    return first


# =============================================================================
# MAIN
# =============================================================================


# =============================================================================
# JSON CANDIDATE RECORD HELPERS
# =============================================================================

def local_timestamp() -> str:
    """Return an ISO-8601 local timestamp with UTC offset."""
    return datetime.now().astimezone().isoformat(timespec="seconds")


def safe_label(text: str) -> str:
    """
    Turn a user-provided label into a safe, readable filename component.
    Examples:
      'Current Location' -> 'current-location'
      'Moved ~50 feet'   -> 'moved-50-feet'
    """
    clean = re.sub(r"[^A-Za-z0-9]+", "-", text.strip()).strip("-").lower()
    return clean or "candidate"


def load_candidate_coordinate(path_text: str) -> tuple[dict, RTCM1005]:
    """Read a prior candidate JSON record for optional distance comparison."""
    path = os.path.abspath(os.path.expanduser(path_text))
    with open(path, "r", encoding="utf-8") as fh:
        record = json.load(fh)

    coordinate = record.get("coordinate")
    if not isinstance(coordinate, dict):
        raise ValueError("Reference JSON has no 'coordinate' object.")

    ecef = coordinate.get("ecef_m")
    if not isinstance(ecef, dict):
        raise ValueError("Reference JSON has no 'coordinate.ecef_m' object.")

    def numeric(value, name: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"Reference JSON field {name} must be numeric.")
        return float(value)

    base = record.get("base", {})
    station_id = 0
    if isinstance(base, dict) and isinstance(base.get("reference_station_id"), int):
        station_id = base["reference_station_id"]

    position = RTCM1005(
        station_id=station_id,
        x_m=numeric(ecef.get("x"), "coordinate.ecef_m.x"),
        y_m=numeric(ecef.get("y"), "coordinate.ecef_m.y"),
        z_m=numeric(ecef.get("z"), "coordinate.ecef_m.z"),
        lat_deg=numeric(coordinate.get("latitude_deg"), "coordinate.latitude_deg"),
        lon_deg=numeric(coordinate.get("longitude_deg"), "coordinate.longitude_deg"),
        ellipsoid_height_m=numeric(
            coordinate.get("ellipsoid_height_m"),
            "coordinate.ellipsoid_height_m",
        ),
    )
    return record, position


def enu_difference_m(reference: RTCM1005, candidate: RTCM1005) -> dict:
    """
    Express the candidate-minus-reference ECEF vector in the reference's
    local East/North/Up axes. This gives a useful horizontal distance for
    comparing, for example, the same location versus a point moved ~50 feet.
    """
    lat = math.radians(reference.lat_deg)
    lon = math.radians(reference.lon_deg)

    dx = candidate.x_m - reference.x_m
    dy = candidate.y_m - reference.y_m
    dz = candidate.z_m - reference.z_m

    east = -math.sin(lon) * dx + math.cos(lon) * dy
    north = (
        -math.sin(lat) * math.cos(lon) * dx
        - math.sin(lat) * math.sin(lon) * dy
        + math.cos(lat) * dz
    )
    up = (
        math.cos(lat) * math.cos(lon) * dx
        + math.cos(lat) * math.sin(lon) * dy
        + math.sin(lat) * dz
    )

    horizontal = math.hypot(east, north)
    three_dimensional = math.sqrt(east * east + north * north + up * up)

    return {
        "east_m": east,
        "north_m": north,
        "up_m": up,
        "horizontal_distance_m": horizontal,
        "horizontal_distance_ft": horizontal * 3.280839895,
        "three_dimensional_distance_m": three_dimensional,
        "three_dimensional_distance_ft": three_dimensional * 3.280839895,
    }


def make_candidate_record(
    *,
    label: str,
    raw_capture_file: str,
    survey_started_local: str,
    static_detected_local: str,
    position: RTCM1005,
    rtcm_counts: Counter,
    comparison: Optional[dict],
) -> dict:
    """
    Create a JSON record directly usable by px1125r_commit_static_base_json.py.

    This is a survey candidate only. It is intentionally NOT marked as
    committed. The commit script later performs its own live guardrails before
    any Flash write.
    """
    record = {
        "schema_version": 1,
        "record_type": "px1125r_static_base_survey_candidate",
        "candidate_status": "not_committed",
        "base": {
            "name": label,
            "receiver_model": "SkyTraq RTK receiver",
            "reference_station_id": position.station_id,
        },
        "coordinate": {
            "datum": "WGS 84",
            "latitude_deg": position.lat_deg,
            "longitude_deg": position.lon_deg,
            "ellipsoid_height_m": position.ellipsoid_height_m,
            "mean_sea_level_height_m": None,
            "geoid_model": None,
            "ecef_m": {
                "x": position.x_m,
                "y": position.y_m,
                "z": position.z_m,
            },
        },
        "coordinate_source": {
            "method": (
                "SkyTraq RTK Base Survey followed by confirmed run-time Static "
                "state and three identical CRC-valid RTCM 1005 messages"
            ),
            "survey_duration_seconds": SURVEY_SECONDS,
            "survey_standard_deviation_parameter_m": STANDARD_DEVIATION_METERS,
            "configuration_persistence": "SRAM only; not written to Flash",
            "survey_started_local": survey_started_local,
            "static_detected_local": static_detected_local,
            "raw_capture_file": os.path.basename(raw_capture_file),
            "raw_capture_path": os.path.abspath(raw_capture_file),
            "post_static_1005_frames_confirmed": REQUIRED_1005_FRAMES,
            "post_static_1005_identical_within_m": 0.001,
        },
        "receiver_configuration": {
            "rtk_mode": "base",
            "saved_operational_state": "survey",
            "runtime_operational_state": "static",
            "serial_port": PORT,
            "baud": BAUD,
        },
        "rtcm_output": {
            "message_types_observed": sorted(int(msg) for msg in rtcm_counts),
            "message_counts_observed": {
                str(msg): int(count) for msg, count in sorted(rtcm_counts.items())
            },
            "required_message_types_for_rover": [1005, 1074, 1094, 1124],
            "expected_1005_rate_hz": 1.0,
        },
        "notes": [
            (
                "The static-base commit command must use coordinate.ellipsoid_height_m. "
                "It must not use a generic altitude or MSL elevation."
            ),
            (
                "This file is a candidate record, not authorization to write Flash. "
                "Review the coordinate, source details, and optional comparison before "
                "calling the JSON-driven commit script with --commit."
            ),
            (
                "For a serious survey, change SURVEY_SECONDS to 86400 and preserve "
                "this candidate file as an immutable record of the derived coordinate."
            ),
        ],
        "audit_log": [
            {
                "timestamp_local": static_detected_local,
                "event": "survey_candidate_created",
                "details": (
                    "Survey completed; SkyTraq receiver reported run-time Static; "
                    "three repeat RTCM 1005 frames matched."
                ),
            }
        ],
    }

    if comparison is not None:
        record["comparison_to_reference"] = comparison

    return record


def write_json_record(path: str, record: dict) -> None:
    """Write a new candidate JSON file. Refuse to overwrite an existing file."""
    if os.path.exists(path):
        raise FileExistsError(
            f"Refusing to overwrite existing candidate JSON: {os.path.abspath(path)}"
        )

    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(record, fh, indent=2)
        fh.write("\n")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a SkyTraq RTK Base Survey, confirm Static mode, decode RTCM 1005, "
            "and write a JSON candidate record for the commit script."
        )
    )
    parser.add_argument(
        "--label",
        default="survey-candidate",
        help=(
            "Human-readable label used in the candidate JSON and filename. "
            "Examples: current-location-test or moved-50-feet."
        ),
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=SURVEY_SECONDS,
        help=(
            "Survey duration in seconds. Default is 60. "
            "Use 86400 for a 24-hour survey."
        ),
    )
    parser.add_argument(
        "--output",
        help=(
            "Optional JSON output filename. By default, a timestamped "
            "skytraq_survey_candidate_*.json file is created."
        ),
    )
    parser.add_argument(
        "--compare",
        help=(
            "Optional prior candidate/config JSON. The new record and console "
            "output will include East/North/Up and horizontal/3D distance from it."
        ),
    )
    parser.add_argument(
        "--status-poll-seconds",
        type=float,
        default=DEFAULT_STATUS_POLL_SECONDS,
        help=(
            "Seconds between RTK-status checks in normal monitor mode. "
            "Default: 20."
        ),
    )
    parser.add_argument(
        "--post-survey-grace-seconds",
        type=int,
        default=DEFAULT_POST_SURVEY_GRACE_SECONDS,
        help=(
            "Additional seconds permitted after the requested survey duration "
            "before normal monitor mode times out. Default: 240."
        ),
    )
    parser.add_argument(
        "--minimum-monitor-timeout-seconds",
        type=int,
        default=DEFAULT_MINIMUM_MONITOR_TIMEOUT_SECONDS,
        help=(
            "Minimum total monitor time in normal mode. The effective timeout is "
            "max(duration + post-survey grace, this value). Default: 300."
        ),
    )
    parser.add_argument(
        "--max-stalled-polls",
        type=int,
        default=DEFAULT_MAX_STALLED_POLLS,
        help=(
            "Abort normal monitor mode after this many consecutive valid Survey "
            "status replies show no reduction in run-time survey length. "
            "Default: 3."
        ),
    )
    parser.add_argument(
        "--max-missing-status-polls",
        type=int,
        default=DEFAULT_MAX_MISSING_STATUS_POLLS,
        help=(
            "Abort normal monitor mode after this many consecutive status polls "
            "produce no parseable 6A 83 reply. Default: 3."
        ),
    )
    parser.add_argument(
        "--px1172rh-diagnostic",
        action="store_true",
        help=(
            "PX1172RH diagnostic mode: after configuring Survey, send NO status "
            "queries during the survey. The script passively logs data for "
            "duration + --diagnostic-extra-wait seconds, then sends one final "
            "0x6A/0x07 status query and exits."
        ),
    )
    parser.add_argument(
        "--diagnostic-extra-wait",
        type=int,
        default=15,
        help=(
            "Additional passive seconds to wait after --duration in PX1172RH "
            "diagnostic mode. Default: 15."
        ),
    )
    return parser.parse_args()


# =============================================================================
# MAIN
# =============================================================================

def main() -> int:
    args = parse_arguments()

    if ATTRIBUTES != 0:
        print("ERROR: This survey script must use ATTRIBUTES = 0 (SRAM only).")
        return 2

    if not 60 <= args.duration <= 1_209_600:
        print("ERROR: --duration must be between 60 and 1,209,600 seconds.")
        return 2

    if args.diagnostic_extra_wait < 0:
        print("ERROR: --diagnostic-extra-wait must be zero or greater.")
        return 2
    if args.status_poll_seconds <= 0:
        print("ERROR: --status-poll-seconds must be greater than zero.")
        return 2
    if args.post_survey_grace_seconds < 0:
        print("ERROR: --post-survey-grace-seconds must be zero or greater.")
        return 2
    if args.minimum_monitor_timeout_seconds < 60:
        print("ERROR: --minimum-monitor-timeout-seconds must be at least 60.")
        return 2
    if args.max_stalled_polls < 1:
        print("ERROR: --max-stalled-polls must be at least 1.")
        return 2
    if args.max_missing_status_polls < 1:
        print("ERROR: --max-missing-status-polls must be at least 1.")
        return 2

    label = args.label.strip() or "survey-candidate"
    filename_label = safe_label(label)
    timestamp_for_files = datetime.now().strftime("%Y%m%d_%H%M%S")

    raw_capture_filename = (
        f"skytraq_survey_{timestamp_for_files}_{filename_label}.bin"
    )
    json_filename = args.output or (
        f"skytraq_survey_candidate_{timestamp_for_files}_{filename_label}.json"
    )

    # Use the command-line duration during this run without permanently changing
    # the default constant for the next invocation.
    global SURVEY_SECONDS
    SURVEY_SECONDS = args.duration

    reference_record = None
    reference_position = None
    if args.compare:
        try:
            reference_record, reference_position = load_candidate_coordinate(args.compare)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            print(f"ERROR: Could not load --compare JSON: {exc}")
            return 2

    print("=" * 78)
    print("SKYTRAQ RTK BASE SURVEY -> RTCM 1005 -> JSON CANDIDATE RECORD")
    print("=" * 78)
    print(f"Port:                          {PORT}")
    print(f"Baud:                          {BAUD}")
    print(f"Candidate label:               {label}")
    print(f"Requested survey duration:     {SURVEY_SECONDS} second(s)")
    print(f"Survey standard deviation:     {STANDARD_DEVIATION_METERS} m")
    print("Configuration persistence:     SRAM only (not Flash)")
    print(f"Raw capture file:              {os.path.abspath(raw_capture_filename)}")
    print(f"Candidate JSON file:           {os.path.abspath(json_filename)}")
    if args.px1172rh_diagnostic:
        print("PX1172RH diagnostic mode:     ENABLED")
        print(
            "Passive wait:                  "
            f"{SURVEY_SECONDS + args.diagnostic_extra_wait} second(s) "
            "(no status queries during survey)"
        )
    else:
        monitor_timeout_seconds = max(
            SURVEY_SECONDS + args.post_survey_grace_seconds,
            args.minimum_monitor_timeout_seconds,
        )
        print("Progress-monitor mode:         ENABLED")
        print(f"Status poll interval:          {args.status_poll_seconds:g} second(s)")
        print(f"Maximum monitor time:          {monitor_timeout_seconds} second(s)")
        print(
            "Abort condition:                "
            f"{args.max_stalled_polls} consecutive valid Survey replies with no "
            "reduction in run-time survey length"
        )
    if reference_position is not None:
        print(f"Comparison reference JSON:     {os.path.abspath(args.compare)}")
    print("=" * 78)

    print(
        "\nSAFETY CHECK:\n"
        "  • Close GNSS Viewer.\n"
        "  • Stop any service that owns or forwards this serial port.\n"
        "  • Do not forward the raw serial stream to a rover during the survey.\n"
        "  • This changes the SkyTraq receiver into RTK BASE SURVEY mode for this\n"
        "    powered session only. It does not write configuration to Flash.\n"
        "  • The JSON file written at the end is a survey candidate. It is not\n"
        "    automatically committed and does not overwrite another JSON record."
    )
    answer = input(
        f"\nType SURVEY to begin the {SURVEY_SECONDS}-second test: "
    ).strip()
    if answer != "SURVEY":
        print("Cancelled. No receiver command was sent.")
        return 0

    parser = MixedStreamParser()
    ser: Optional[serial.Serial] = None
    survey_started_local = local_timestamp()

    try:
        ser = serial.Serial(
            PORT,
            BAUD,
            timeout=SERIAL_TIMEOUT_SECONDS,
            write_timeout=2,
        )
        print(f"\nOpened {PORT} successfully. Settling for 1 second...")
        time.sleep(1.0)
        ser.reset_input_buffer()

        with open(raw_capture_filename, "wb") as raw_file:
            before = query_rtk_status(ser, parser, raw_file)
            if before is not None:
                print("\nInitial RTK status was received successfully.")
            else:
                print(
                    "\nWARNING: Initial RTK status query did not yield a parseable response.\n"
                    "The script will still attempt the SRAM-only survey, but it will\n"
                    "write no candidate JSON unless final validation succeeds."
                )

            print("\nWaiting 1 second before configuring Base Survey mode...")
            time.sleep(1.0)

            (
                seed_latitude_deg,
                seed_longitude_deg,
                seed_ellipsoid_height_m,
                seed_note,
            ) = survey_seed_from_prior_status(before)

            print("\nSurvey command coordinate-field handling:")
            print(f"  {seed_note}")
            print(f"  Latitude field:         {seed_latitude_deg:.12f} deg")
            print(f"  Longitude field:        {seed_longitude_deg:.12f} deg")
            print(f"  Ellipsoid-height field: {seed_ellipsoid_height_m:.6f} m")

            survey_payload = make_rtk_base_survey_payload(
                SURVEY_SECONDS,
                STANDARD_DEVIATION_METERS,
                ATTRIBUTES,
                seed_latitude_deg,
                seed_longitude_deg,
                seed_ellipsoid_height_m,
            )
            send_command(
                ser,
                parser,
                raw_file,
                (
                    f"Configure RTK BASE SURVEY for {SURVEY_SECONDS} second(s) "
                    "[0x6A / 0x06, SRAM only]"
                ),
                survey_payload,
            )

            print("\nWaiting 1 second after configuration ACK window...")
            time.sleep(1.0)

            configured = query_rtk_status(ser, parser, raw_file)
            if configured is None:
                print(
                    "\nWARNING: Could not verify Survey mode from status response.\n"
                    "Continuing to observe, but no JSON will be written unless run-time\n"
                    "Static and repeat CRC-valid 1005 messages are confirmed."
                )
            elif not (
                configured.rtk_mode == 1
                and configured.saved_operational_function == 1
            ):
                print(
                    "\nERROR: Receiver did not report RTK BASE + saved SURVEY mode.\n"
                    "No candidate JSON will be written."
                )
                return 3

            print("\n" + "*" * 78)
            print(
                f"SURVEY OBSERVATION STARTED. The receiver is expected to survey for "
                f"{SURVEY_SECONDS} seconds, then change its RUN-TIME operation to Static."
            )
            print("*" * 78)

            survey_start = time.monotonic()
            completed = False
            static_detected_local = ""

            if args.px1172rh_diagnostic:
                # This branch deliberately sends no 0x6A/0x07 status queries
                # while the survey is in progress. It keeps draining and logging
                # the serial port so the OS/USB buffer cannot fill, but it does
                # not transmit any further SkyTraq command until one final query.
                passive_wait = SURVEY_SECONDS + args.diagnostic_extra_wait
                passive_end = survey_start + passive_wait

                print(
                    "\nPX1172RH DIAGNOSTIC MODE:\n"
                    f"  Passive capture duration: {passive_wait} second(s)\n"
                    "  Status queries during this interval: NONE\n"
                    "  The receiver will receive no commands until the single final "
                    "RTK-status query after the passive interval."
                )

                while time.monotonic() < passive_end:
                    remaining = passive_end - time.monotonic()
                    interval = min(10.0, remaining)
                    received = read_for(
                        ser,
                        parser,
                        interval,
                        "PX1172RH passive survey capture (no command sent)",
                        raw_file,
                    )
                    elapsed = time.monotonic() - survey_start
                    print(
                        f"[Passive capture] {elapsed:.1f}/{passive_wait} s elapsed; "
                        f"{received} byte(s) received; no status query sent."
                    )

                print(
                    "\nPassive interval complete. Sending ONE final 0x6A/0x07 "
                    "RTK-status query now."
                )
                status = query_rtk_status(ser, parser, raw_file)
                elapsed = time.monotonic() - survey_start

                if status is None:
                    print(
                        "\nDIAGNOSTIC RESULT: No parseable 6A 83 response after "
                        "the one final status query."
                    )
                else:
                    print(
                        f"\nDiagnostic elapsed {elapsed:.1f}/{passive_wait} s | "
                        f"final run-time state: {status.runtime_function_name}"
                    )
                    if (
                        status.rtk_mode == 1
                        and status.saved_operational_function == 1
                        and status.runtime_operational_function == 2
                    ):
                        static_detected_local = local_timestamp()
                        completed = True
                        print(
                            "\nDIAGNOSTIC RESULT: Receiver reached run-time Static "
                            "without repeated 0x6A/0x07 status queries."
                        )
                    else:
                        print(
                            "\nDIAGNOSTIC RESULT: Receiver is still not in run-time "
                            "Static after the passive survey interval."
                        )

                if not completed:
                    print(
                        "\nNo JSON candidate was written because a completed "
                        "run-time Static survey was not confirmed."
                    )
                    print(
                        "This diagnostic run sent one initial status query, one "
                        "survey-configure command, one post-configure status query, "
                        "and one final status query. It sent no status queries "
                        "during the passive survey interval."
                    )
                    print("\nRTCM frames observed during this diagnostic run:")
                    if parser.rtcm_counter:
                        for msg_type, count in sorted(parser.rtcm_counter.items()):
                            print(f"  Type {msg_type:4d}: {count}")
                    else:
                        print("  No CRC-valid RTCM frames were observed.")
                    return 4

            else:
                # Progress-monitor mode intentionally polls more slowly than the
                # original script.  It makes a decision from the receiver's actual
                # run-time countdown, not simply wall-clock time.  That matters for
                # the PX1172RH, which can take longer than the requested duration to
                # transition from Survey to Static in marginal test conditions.
                monitor_timeout_seconds = max(
                    SURVEY_SECONDS + args.post_survey_grace_seconds,
                    args.minimum_monitor_timeout_seconds,
                )
                deadline = survey_start + monitor_timeout_seconds
                poll_interval = args.status_poll_seconds

                previous_remaining: Optional[int] = None
                if (
                    configured is not None
                    and configured.rtk_mode == 1
                    and configured.saved_operational_function == 1
                    and configured.runtime_operational_function == 1
                ):
                    previous_remaining = configured.runtime_survey_seconds

                stalled_polls = 0
                missing_status_polls = 0
                next_poll = survey_start + poll_interval

                print(
                    "Progress-monitor mode: checks RTK state every "
                    f"{poll_interval:g} seconds, allows up to "
                    f"{monitor_timeout_seconds} seconds total, and aborts after "
                    f"{args.max_stalled_polls} consecutive non-decreasing Survey "
                    "countdown readings."
                )
                if previous_remaining is not None:
                    print(
                        "Initial run-time survey length from post-configure status: "
                        f"{previous_remaining} second(s)."
                    )
                else:
                    print(
                        "Post-configure status did not provide a usable Survey "
                        "countdown; the first valid Survey status will establish "
                        "the baseline."
                    )

                while time.monotonic() < deadline:
                    sleep_seconds = max(0.0, min(next_poll - time.monotonic(), deadline - time.monotonic()))
                    if sleep_seconds > 0:
                        # Keep draining/logging raw serial input while waiting so a
                        # USB serial buffer cannot fill during high-rate RTCM output.
                        read_for(
                            ser,
                            parser,
                            sleep_seconds,
                            "progress-monitor capture between RTK-status polls",
                            raw_file,
                        )

                    if time.monotonic() >= deadline:
                        break

                    elapsed = time.monotonic() - survey_start
                    status = query_rtk_status(ser, parser, raw_file)
                    next_poll += poll_interval

                    if status is None:
                        missing_status_polls += 1
                        print(
                            f"\nSurvey elapsed {elapsed:.1f}/{monitor_timeout_seconds} s | "
                            "no parseable RTK-status reply "
                            f"({missing_status_polls}/{args.max_missing_status_polls})."
                        )
                        if missing_status_polls >= args.max_missing_status_polls:
                            print(
                                "\nERROR: Survey monitor aborted because repeated "
                                "0x6A/0x07 status queries did not return a parseable "
                                "0x6A/0x83 reply."
                            )
                            print("No JSON candidate was written.")
                            return 4
                        continue

                    missing_status_polls = 0
                    print(
                        f"\nSurvey elapsed {elapsed:.1f}/{monitor_timeout_seconds} s | "
                        f"run-time state: {status.runtime_function_name}"
                    )

                    if not (
                        status.rtk_mode == 1
                        and status.saved_operational_function == 1
                    ):
                        print(
                            "\nERROR: Survey monitor aborted because the receiver no "
                            "longer reports RTK Base + saved Survey configuration."
                        )
                        print("No JSON candidate was written.")
                        return 4

                    if status.runtime_operational_function == 2:
                        static_detected_local = local_timestamp()
                        print(
                            "\nReceiver confirms RTK Base Survey configuration and "
                            "run-time transition to Static."
                        )
                        completed = True
                        break

                    if status.runtime_operational_function != 1:
                        print(
                            "\nERROR: Survey monitor aborted because the receiver "
                            "reported an unexpected run-time operational state: "
                            f"{status.runtime_function_name}."
                        )
                        print("No JSON candidate was written.")
                        return 4

                    current_remaining = status.runtime_survey_seconds
                    if previous_remaining is None:
                        previous_remaining = current_remaining
                        print(
                            "Run-time survey countdown baseline established at "
                            f"{current_remaining} second(s)."
                        )
                        continue

                    if current_remaining < previous_remaining:
                        reduced_by = previous_remaining - current_remaining
                        print(
                            "Run-time survey countdown is progressing: "
                            f"{previous_remaining} -> {current_remaining} second(s) "
                            f"(reduced by {reduced_by})."
                        )
                        previous_remaining = current_remaining
                        stalled_polls = 0
                    else:
                        stalled_polls += 1
                        relation = "unchanged" if current_remaining == previous_remaining else "increased"
                        print(
                            "WARNING: Run-time survey countdown did not reduce: "
                            f"{previous_remaining} -> {current_remaining} second(s) "
                            f"({relation}); non-progress poll "
                            f"{stalled_polls}/{args.max_stalled_polls}."
                        )
                        # Use the newest reply as the comparison baseline. This lets
                        # a real restart be visible and still allows later decreases
                        # to clear the non-progress counter.
                        previous_remaining = current_remaining

                        if stalled_polls >= args.max_stalled_polls:
                            print(
                                "\nERROR: Survey monitor aborted because the run-time "
                                "survey length did not reduce across the configured "
                                "number of status polls."
                            )
                            print("No JSON candidate was written.")
                            return 4

                if not completed:
                    print(
                        "\nERROR: Survey did not reach confirmed run-time Static "
                        f"state within the {monitor_timeout_seconds}-second monitor timeout."
                    )
                    print("No JSON candidate was written.")
                    return 4

            print("\n" + "*" * 78)
            print(
                f"COLLECTING {REQUIRED_1005_FRAMES} REPEAT RTCM 1005 MESSAGES AFTER "
                "CONFIRMED STATIC STATE"
            )
            print("*" * 78)

            parser.collect_1005 = True
            parser.rtcm_1005_after_static.clear()
            capture_deadline = time.monotonic() + POST_STATIC_CAPTURE_TIMEOUT_SECONDS

            while (
                time.monotonic() < capture_deadline
                and len(parser.rtcm_1005_after_static) < REQUIRED_1005_FRAMES
            ):
                read_for(
                    ser,
                    parser,
                    min(1.0, capture_deadline - time.monotonic()),
                    "post-survey 1005 collection",
                    raw_file,
                )

            parser.collect_1005 = False
            final_position = stable_1005_position(parser.rtcm_1005_after_static)

            if final_position is None:
                print(
                    "\nERROR: Did not obtain enough stable 1005 frames after the receiver "
                    "reported run-time Static."
                )
                print(
                    f"1005 frames collected: {len(parser.rtcm_1005_after_static)} "
                    f"(required: {REQUIRED_1005_FRAMES})"
                )
                print("No JSON candidate was written.")
                return 5

            comparison_result = None
            if reference_position is not None:
                delta = enu_difference_m(reference_position, final_position)
                comparison_result = {
                    "reference_json": os.path.abspath(args.compare),
                    "reference_label": (
                        reference_record.get("base", {}).get("name")
                        if isinstance(reference_record, dict)
                        else None
                    ),
                    "reference_coordinate": {
                        "latitude_deg": reference_position.lat_deg,
                        "longitude_deg": reference_position.lon_deg,
                        "ellipsoid_height_m": reference_position.ellipsoid_height_m,
                        "ecef_m": {
                            "x": reference_position.x_m,
                            "y": reference_position.y_m,
                            "z": reference_position.z_m,
                        },
                    },
                    "candidate_minus_reference_enu_m": {
                        "east": delta["east_m"],
                        "north": delta["north_m"],
                        "up": delta["up_m"],
                    },
                    "horizontal_distance_m": delta["horizontal_distance_m"],
                    "horizontal_distance_ft": delta["horizontal_distance_ft"],
                    "three_dimensional_distance_m": delta["three_dimensional_distance_m"],
                    "three_dimensional_distance_ft": delta["three_dimensional_distance_ft"],
                }

            candidate = make_candidate_record(
                label=label,
                raw_capture_file=raw_capture_filename,
                survey_started_local=survey_started_local,
                static_detected_local=static_detected_local,
                position=final_position,
                rtcm_counts=parser.rtcm_counter,
                comparison=comparison_result,
            )
            write_json_record(json_filename, candidate)

            print("\n" + "=" * 78)
            print("SURVEY RESULT — DERIVED FROM POST-SURVEY RTCM MESSAGE 1005")
            print("=" * 78)
            print(f"Reference Station ID:   {final_position.station_id}")
            print(f"ECEF X:                 {final_position.x_m:.4f} m")
            print(f"ECEF Y:                 {final_position.y_m:.4f} m")
            print(f"ECEF Z:                 {final_position.z_m:.4f} m")
            print(f"Latitude:               {final_position.lat_deg:.12f} deg")
            print(f"Longitude:              {final_position.lon_deg:.12f} deg")
            print(f"Ellipsoid height:       {final_position.ellipsoid_height_m:.4f} m")
            print()
            print(
                "Altitude note: RTCM 1005 contains ECEF coordinates. The recorded\n"
                "height is WGS-84 ellipsoid height, NOT MSL elevation."
            )
            print("=" * 78)

            if comparison_result is not None:
                enu = comparison_result["candidate_minus_reference_enu_m"]
                print("\nCOMPARISON TO REFERENCE JSON")
                print(f"Reference:              {comparison_result['reference_json']}")
                print(f"East / North / Up:      {enu['east']:+.3f} / "
                      f"{enu['north']:+.3f} / {enu['up']:+.3f} m")
                print(f"Horizontal separation:  "
                      f"{comparison_result['horizontal_distance_m']:.3f} m "
                      f"({comparison_result['horizontal_distance_ft']:.2f} ft)")
                print(f"3D separation:          "
                      f"{comparison_result['three_dimensional_distance_m']:.3f} m "
                      f"({comparison_result['three_dimensional_distance_ft']:.2f} ft)")

            print(f"\nJSON survey candidate written: {os.path.abspath(json_filename)}")
            print(
                "This JSON is directly compatible with px1125r_commit_static_base_json.py.\n"
                "Review it first. To commit this candidate later, use:\n"
                f'  python px1125r_commit_static_base_json.py --config "{json_filename}" --commit'
            )

            print("\nRTCM frames observed throughout this run:")
            for msg_type, count in sorted(parser.rtcm_counter.items()):
                print(f"  Type {msg_type:4d}: {count}")

            print(
                "\nTEST COMPLETE.\n"
                "The receiver remains in run-time Static using this survey-derived\n"
                "coordinate for the current powered session. This survey script used\n"
                "SRAM only and did not write Flash."
            )

    except serial.SerialException as exc:
        print(f"\nSERIAL ERROR: {exc}")
        print("Confirm that no other program owns the port and that PORT/BAUD are correct.")
        return 10
    except FileExistsError as exc:
        print(f"\nOUTPUT ERROR: {exc}")
        print("Choose a different --output filename or leave --output off for an automatic name.")
        return 11
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"\nFILE / JSON ERROR: {exc}")
        return 12
    except KeyboardInterrupt:
        print("\nInterrupted by user. No JSON candidate was written by this interrupted run.")
        print("The receiver may remain in its current survey/static state.")
        return 130
    finally:
        if ser is not None and ser.is_open:
            ser.close()
            print("\nSerial port closed.")

    print(f"\nRaw capture saved to: {os.path.abspath(raw_capture_filename)}")
    print(f"JSON candidate saved to: {os.path.abspath(json_filename)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
