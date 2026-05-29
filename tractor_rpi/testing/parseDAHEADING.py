#!/usr/bin/env python3
"""
parseDAHEADING.py

Parses UBX-NAV-DAHEADING (0x01 0x45) messages from the ArduSimple ZED-X20D
dual-antenna heading module and prints heading, baseline, and accuracy.

The heading is the clockwise angle from True North to the baseline running
from the master antenna (GPS1) to the slave antenna (GPS2). Mount GPS1 at
the rear of the tractor and GPS2 at the front so that heading = vehicle
forward direction with no offset correction needed.

Message structure (64-byte payload, confirmed by reverse engineering):
  Offset  Type   Field
  0       U1     version (= 1)
  1-3     U1×3   reserved
  4-7     U4     iTOW (ms, GPS time of week)
  8-11    I4     relPosN (cm)
  12-15   I4     relPosE (cm)
  16-19   I4     relPosD (cm)
  20-23   U4     relPosLength (cm)
  24-27   U4     relPosHeading (1e-5 deg, True North clockwise)
  28-31   —      reserved
  32      I1     relPosHPN (0.1 mm HP correction)
  33      I1     relPosHPE (0.1 mm HP correction)
  34      I1     relPosHPD (0.1 mm HP correction)
  35      I1     relPosHPLength (0.1 mm HP correction)
  36-39   U4     accN (0.1 mm)
  40-43   U4     accE (0.1 mm)
  44-47   U4     accD (0.1 mm)
  48-51   U4     accLength (0.1 mm)
  52-55   U4     accHeading (1e-5 deg)
  56-59   —      reserved
  60-63   U4     flags

Flags (relevant bits):
  bit 0   gnssFixOK
  bit 1   diffSoln
  bit 2   relPosValid
  bits 3-4 carrSoln (0=none, 1=float, 2=fixed)
  bit 8   relPosHeadingValid

USB device will appear as /dev/ttyACM1 (or ACM0) on the RPi.

Usage:
    python3 parseDAHEADING.py [port] [baud]

Defaults:
    port = /dev/ttyACM1
    baud = 115200
"""

import serial, struct, sys, math
from datetime import datetime

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

PRINT_HEX = False  # set True to see raw payload bytes

SYNC1, SYNC2 = 0xB5, 0x62
CLS_NAV, ID_DAHEADING = 0x01, 0x45
PAYLOAD_LEN = 64

def ubx_checksum(data: bytes):
    a = b = 0
    for x in data:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return a, b

def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)

def parse_daheading(payload: bytes):
    if len(payload) != PAYLOAD_LEN:
        return None

    version = payload[0]
    iTOW                      = struct.unpack_from("<I", payload, 4)[0]
    relPosN_cm, relPosE_cm, relPosD_cm = struct.unpack_from("<iii", payload, 8)
    relPosLen_cm              = struct.unpack_from("<I", payload, 20)[0]
    relPosHead_1e5deg         = struct.unpack_from("<I", payload, 24)[0]
    hpN, hpE, hpD, hpLen     = struct.unpack_from("<bbbb", payload, 32)
    accN_0p1mm                = struct.unpack_from("<I", payload, 36)[0]
    accE_0p1mm                = struct.unpack_from("<I", payload, 40)[0]
    accD_0p1mm                = struct.unpack_from("<I", payload, 44)[0]
    accLen_0p1mm              = struct.unpack_from("<I", payload, 48)[0]
    accHead_1e5deg            = struct.unpack_from("<I", payload, 52)[0]
    flags                     = struct.unpack_from("<I", payload, 60)[0]

    heading_deg  = relPosHead_1e5deg * 1e-5
    accHead_deg  = accHead_1e5deg * 1e-5

    # Full high-precision positions
    N_m = (relPosN_cm + hpN * 0.01) / 100.0
    E_m = (relPosE_cm + hpE * 0.01) / 100.0
    D_m = (relPosD_cm + hpD * 0.01) / 100.0
    length_m = (relPosLen_cm + hpLen * 0.01) / 100.0

    head_valid  = bool(flags & (1 << 8))
    carr_soln   = {0: "none", 1: "float", 2: "fixed"}.get((flags >> 3) & 0x3, "?")
    gnss_fix_ok = bool(flags & (1 << 0))
    diff_soln   = bool(flags & (1 << 1))
    rel_valid   = bool(flags & (1 << 2))

    return {
        "iTOW_ms":      iTOW,
        "N_m":          N_m,
        "E_m":          E_m,
        "D_m":          D_m,
        "length_m":     length_m,
        "heading_deg":  heading_deg,
        "accN_mm":      accN_0p1mm * 0.1,
        "accE_mm":      accE_0p1mm * 0.1,
        "accD_mm":      accD_0p1mm * 0.1,
        "accLen_mm":    accLen_0p1mm * 0.1,
        "accHead_deg":  accHead_deg,
        "flags": {
            "headValid":  head_valid,
            "carrSoln":   carr_soln,
            "gnssFixOK":  gnss_fix_ok,
            "diffSoln":   diff_soln,
            "relValid":   rel_valid,
        },
    }

def read_stream():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print(f"[{datetime.now().isoformat(timespec='seconds')}] "
              f"Listening on {PORT} @ {BAUD} for UBX-NAV-DAHEADING (0x01 0x45)")
        print(f"{'iTOW':>12}  {'Heading':>10}  {'Baseline':>10}  {'AccHead':>9}  "
              f"{'Valid':>5}  {'Carr':>5}  {'N_m':>8}  {'E_m':>8}")
        print("-" * 90)

        buf = bytearray()
        while True:
            b = ser.read(1)
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
                chunk = ser.read(need - len(buf))
                if not chunk:
                    break
                buf += chunk
            if len(buf) < need:
                buf.clear(); continue

            ck_a, ck_b = ubx_checksum(buf[2:6 + length])
            if (ck_a, ck_b) != (buf[6 + length], buf[7 + length]):
                buf.clear(); continue

            payload = bytes(buf[6:6 + length])
            buf.clear()

            if cls_ != CLS_NAV or id_ != ID_DAHEADING:
                continue

            if PRINT_HEX:
                print(f"RAW ({length}B): {hexdump(payload)}")

            d = parse_daheading(payload)
            if not d:
                continue

            print(
                f"{d['iTOW_ms']:>12}  "
                f"{d['heading_deg']:>9.4f}°  "
                f"{d['length_m']:>9.4f}m  "
                f"{d['accHead_deg']:>8.4f}°  "
                f"{'Y' if d['flags']['headValid'] else 'N':>5}  "
                f"{d['flags']['carrSoln']:>5}  "
                f"{d['N_m']:>8.4f}  "
                f"{d['E_m']:>8.4f}"
            )

if __name__ == "__main__":
    try:
        read_stream()
    except KeyboardInterrupt:
        print("\nDone.")
        sys.exit(0)
