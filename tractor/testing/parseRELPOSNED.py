#!/usr/bin/env python3
import serial, struct, sys, math
from datetime import datetime

PORT = "COM41"
BAUD = 115200

# Output toggles
PRINT_HEX = False   # set True if you want to see raw payload hex 

SYNC1, SYNC2 = 0xB5, 0x62
CLS_NAV, ID_RELPOSNED = 0x01, 0x3C

def ubx_checksum(data: bytes):
    a = b = 0
    for x in data:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b

def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)

def cm_hp_to_m(cm: int, hp_0p1mm: int) -> float:
    return (cm + hp_0p1mm * 0.01) / 100.0  # cm + 0.01*hp(cm) -> m

def wrap_deg(x):
    x %= 360.0
    return x if x >= 0 else x + 360.0

def parse_relposned(payload: bytes):
    n = len(payload)
    if n not in (40, 64):
        return None

    # 0x00..0x03: version, reserved1, refStationId
    version, res1, refStationId = struct.unpack_from("<BBH", payload, 0x00)
    # 0x04..0x13: iTOW (U4), relPosN/E/D (I4 each) in cm
    iTOW, relPosN_cm, relPosE_cm, relPosD_cm = struct.unpack_from("<Iiii", payload, 0x04)

    # Next field starts at 0x14
    off = 0x14

    relPosLen_cm = relPosHead_1e5deg = None
    if n == 64:
        # 0x14..0x1B: relPosLen (I4), relPosHead (I4, 1e-5 deg)
        relPosLen_cm, relPosHead_1e5deg = struct.unpack_from("<ii", payload, off)
        off += 8
        off += 4  # reserved2[4]

    # 0x20..0x23: HP N/E/D/Len (I1 each; 0.1 mm units)
    relPosHPN, relPosHPE, relPosHPD = struct.unpack_from("<bbb", payload, off); off += 3
    relPosHPLen = None
    if n == 64:
        relPosHPLen = struct.unpack_from("<b", payload, off)[0]; off += 1
    else:
        off += 1

    # 0x24..0x2F: accN/E/D (U4, 0.1 mm)
    accN_0p1mm, accE_0p1mm, accD_0p1mm = struct.unpack_from("<III", payload, off); off += 12

    # 0x30..0x37: accLen (U4, 0.1 mm), accHead (U4, 1e-5 deg) [F9P only]
    accLen_0p1mm = accHead_1e5deg = None
    if n == 64:
        accLen_0p1mm, accHead_1e5deg = struct.unpack_from("<II", payload, off); off += 8
        off += 4  # reserved3[4]

    # 0x3C..0x3F: flags
    (flags,) = struct.unpack_from("<I", payload, off)

    # Compose metric values
    N = cm_hp_to_m(relPosN_cm, relPosHPN)
    E = cm_hp_to_m(relPosE_cm, relPosHPE)
    D = cm_hp_to_m(relPosD_cm, relPosHPD)
    length_m  = cm_hp_to_m(relPosLen_cm, relPosHPLen) if relPosLen_cm is not None else None
    heading_d = (relPosHead_1e5deg * 1e-5) if relPosHead_1e5deg is not None else None

    # Accuracies
    accN_m, accE_m, accD_m = accN_0p1mm/10000.0, accE_0p1mm/10000.0, accD_0p1mm/10000.0
    accLen_m   = (accLen_0p1mm/10000.0) if accLen_0p1mm is not None else None
    accHead_d  = (accHead_1e5deg*1e-5)  if accHead_1e5deg is not None else None

    # Flags decoded
    carrier = {0:"none", 1:"float", 2:"fixed"}.get((flags>>3)&0x3, "unknown")
    headValid = bool(flags & (1<<8))

    return {
        "iTOW_ms": iTOW,
        "N_m": N, "E_m": E, "D_m": D,
        "length_m": length_m,
        "heading_deg": heading_d,
        "yaw_enu_deg": wrap_deg(90.0 - heading_d) if heading_d is not None else None,
        "accN_m": accN_m, "accE_m": accE_m, "accD_m": accD_m,
        "accLen_m": accLen_m, "accHead_deg": accHead_d,
        "flags": {
            "headValid": headValid,
            "carrier": carrier,
        }
    }

def expected_heading_error_deg(d):
    """
    Return expected 1-sigma heading error in degrees.
    Prefer accHead if present, else use small-angle approx: atan2(sigma_perp, L).
    """
    # Best: receiver's own estimate
    if d["accHead_deg"] is not None and d["flags"]["headValid"]:
        return d["accHead_deg"]

    L = d["length_m"] or 0.0
    if L <= 1e-6:
        return float("nan")

    # Conservative perpendicular sigma (no covariance available)
    sigma_perp = math.hypot(d["accN_m"] or 0.0, d["accE_m"] or 0.0)
    # radians -> degrees
    return math.degrees(math.atan2(sigma_perp, L))

def read_stream():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print(f"[{datetime.now().isoformat(timespec='seconds')}] Listening on {PORT} @ {BAUD} for UBX-NAV-RELPOSNED (0x01 0x3C)")
        buf = bytearray()
        while True:
            b = ser.read(1)
            if not b: continue
            buf += b
            if len(buf)==1 and buf[0]!=SYNC1: buf.clear(); continue
            if len(buf)==2 and buf[1]!=SYNC2:
                buf = bytearray([buf[1]]) if buf[1]==SYNC1 else bytearray(); continue
            if len(buf)<6: continue

            cls_, id_, length = struct.unpack_from("<BBH", buf, 2)
            need = 6 + length + 2
            while len(buf) < need:
                chunk = ser.read(need - len(buf))
                if not chunk: break
                buf += chunk
            if len(buf) < need:
                buf.clear(); continue

            ck_a, ck_b = ubx_checksum(buf[2:6+length])
            if (ck_a, ck_b) != (buf[6+length], buf[7+length]):
                buf.clear(); continue

            payload = bytes(buf[6:6+length]); buf.clear()

            if cls_ == CLS_NAV and id_ == ID_RELPOSNED:
                if PRINT_HEX:
                    print(f"RAW ({length}B): {hexdump(payload)}")

                d = parse_relposned(payload)
                if not d: continue

                err_deg = expected_heading_error_deg(d)

                # print: Length, Heading, ENU Heading, headValid, carrier, ExpectedErrDeg
                print(
                    f"Len={d['length_m']:.4f} m | "
                    f"Head={d['heading_deg']:.5f}° | "
                    f"Yaw_ENU={d['yaw_enu_deg']:.5f}° | "
                    f"headValid={d['flags']['headValid']} | "
                    f"carrier={d['flags']['carrier']} | "
                    f"ExpectedErrDeg={err_deg:.5f}°"
                )

if __name__ == "__main__":
    try:
        read_stream()
    except KeyboardInterrupt:
        sys.exit(0)
