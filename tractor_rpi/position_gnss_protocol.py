# Production-folder placement reviewed 2026-11-20; runtime logic unchanged.
"""Reviewed GNSS wire parsing extracted from rtcm_server_20260828.py.
No device ownership, Wi-Fi polling, threads or hardware access at import.
The original production file remains unchanged.
"""
import copy
import math
import struct

SYNC1, SYNC2 = 0xB5, 0x62
CLS_NAV, ID_PVT, ID_SAT, ID_RELPOSNED = 0x01, 0x07, 0x35, 0x3C
FIX_QUALITY = {0: "Invalid", 1: "GPS Fix", 2: "DGPS", 4: "RTK Fixed", 5: "RTK Float"}
INITIAL_GNSS_STATE = {'lat': None,
 'lon': None,
 'fix_quality': 'Unknown',
 'base_numSV_used': None,
 'base_numSV_visible': None,
 'base_cno_mean_dbhz': None,
 'base_cno_min_dbhz': None,
 'base_cno_max_dbhz': None,
 'base_numSV_used_gga': None,
 'base_satellite_timestamp': None,
 'hdop': None,
 'diff_age': None,
 'speed_mps': None,
 'speed_update_count': 0,
 'speed_timestamp': None,
 'speed_source': None,
 'heading_deg': None,
 'headValid': None,
 'carrier': None,
 'expectedErrDeg': None,
 'relposned_count': 0,
 'relposned_timestamp': None,
 'relposned_itow_ms': None,
 'relpos_length_m': None,
 'relpos_heading_accuracy_deg': None,
 'relpos_gnss_fix_ok': None,
 'relpos_diff_solution': None,
 'relpos_valid': None,
 'relpos_moving': None,
 'relpos_ref_pos_miss': None,
 'relpos_ref_obs_miss': None,
 'relpos_normalized': None,
 'heading_numSV_used': None,
 'heading_numSV_visible': None,
 'heading_cno_mean_dbhz': None,
 'heading_cno_min_dbhz': None,
 'heading_cno_max_dbhz': None,
 'heading_satellite_timestamp': None,
 'heading_nmea_count': 0,
 'heading_nmea_last_type': None,
 'heading_nmea_timestamp': None,
 'heading_nmea_types': {},
 'timestamp': None,
 'fatal_error': False,
 'fatal_base_reason': None,
 'fatal_heading_reason': None}


def initial_state():
    return copy.deepcopy(INITIAL_GNSS_STATE)


def parse_checksum_valid_nmea_type(line: bytes):
    """Return the NMEA sentence identifier, or None for invalid/non-NMEA data."""
    candidate = line.strip()
    if not candidate.startswith(b"$") or b"*" not in candidate:
        return None
    body, checksum_text = candidate[1:].rsplit(b"*", 1)
    if len(checksum_text) != 2:
        return None
    try:
        expected = int(checksum_text, 16)
    except ValueError:
        return None
    actual = 0
    for value in body:
        actual ^= value
    if actual != expected:
        return None
    sentence_id = body.split(b",", 1)[0]
    if not 3 <= len(sentence_id) <= 10 or not sentence_id.isalnum():
        return None
    try:
        return sentence_id.decode("ascii")
    except UnicodeDecodeError:
        return None


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
