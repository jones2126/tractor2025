#!/usr/bin/env python3
"""
F9P Survey-In for a New Location (trip-ready)
=============================================
Quick, light-touch base-station setup for field testing at an unknown spot.
Uses the modern u-blox CFG-VALSET / CFG-VALGET config interface, mirroring the
proven Bridgeville production base (firmware HPG 1.51, survey-in mode).

What it does, in order:
  1. (Optional 3D-fix check) confirms the receiver has a fix.
  2. Disables any existing base mode (CFG_TMODE_MODE=0). Touches only the
     time-mode setting -- it does NOT factory-reset or wipe other config.
  3. Sets the navigation dynamic model to "stationary".
  4. Starts a Survey-In (CFG_TMODE_MODE=1): averages its own position until BOTH
     the minimum duration AND the accuracy limit are met.
  5. Monitors NAV-SVIN until the survey completes.
  6. Enables the RTCM3 messages a rover needs (1005/1074/1084/1094/1230 -- the
     same set the Bridgeville base broadcasts) on USB and UART1.

Persistence (5th arg):
  flash (default) -> writes to RAM+BBR+Flash, so the config survives a power
                     cycle in the field. On reboot it re-surveys (same as the
                     production base) -- fine for relative RTK.
  ram             -> writes to RAM only. Nothing is persisted. Use this when
                     TESTING on a live/production base so you don't overwrite
                     its flashed config. NOTE: RAM changes still take effect
                     immediately and only revert when the F9P is POWER-CYCLED
                     (restarting the rtcm server does NOT reset the F9P).

Usage:
    python3 f9p-survey-new-location.py [port] [baud] [min_dur_s] [acc_limit_m] [flash|ram]

Examples:
    python3 f9p-survey-new-location.py                       # field defaults, saved to flash
    python3 f9p-survey-new-location.py /dev/f9p 115200 120 1.0
    python3 f9p-survey-new-location.py /dev/f9p 115200 90 2.0 ram   # safe test, no persist

Note on accuracy: a quick survey's absolute position is only as good as the
averaged standalone fix (often 1-2 m off true). That is fine -- RTK still gives
cm-level *relative* accuracy between this base and the rover; the base's
absolute error just shifts the whole frame by a constant. (Confirmed in the
field: the Bridgeville base surveyed at ~9 m yet rovers still get RTK Fix.)
"""

import sys
import time
import serial
from pyubx2 import UBXMessage, UBXReader

# ---------- Defaults (override via command line) ----------
DEFAULT_PORT = "/dev/f9p"
DEFAULT_BAUD = 115200
SURVEY_MIN_DURATION = 90        # seconds: survey runs at least this long
SURVEY_ACC_LIMIT_M = 2.0        # meters: survey ends once mean accuracy <= this
SURVEY_MAX_WAIT = 600           # seconds: give up monitoring after 10 min
FIX_CHECK_ATTEMPTS = 8

# RTCM3 message types to broadcast (mirrors the Bridgeville production base)
RTCM_TYPES = ["1005", "1074", "1084", "1094", "1230"]

# CFG-VALSET layer bits
LAYER_RAM = 1
LAYER_BBR = 2
LAYER_FLASH = 4


# ---------- robust serial read (handles the mixed RTCM/NMEA/UBX stream) ----------

def read_raw(ser, seconds):
    buf = bytearray()
    end = time.time() + seconds
    while time.time() < end:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        else:
            time.sleep(0.01)
    return bytes(buf)


def find_ubx(buf, cls_id, msg_id):
    """Return the LAST valid (checksum-verified) UBX frame matching cls/id."""
    i, n = 0, len(buf)
    last = None
    while i < n - 8:
        if buf[i] == 0xB5 and buf[i + 1] == 0x62:
            length = buf[i + 4] | (buf[i + 5] << 8)
            end = i + 6 + length + 2
            if end <= n:
                frame = buf[i:end]
                ck_a = ck_b = 0
                for x in frame[2:-2]:
                    ck_a = (ck_a + x) & 0xFF
                    ck_b = (ck_b + ck_a) & 0xFF
                if ck_a == frame[-2] and ck_b == frame[-1]:
                    if frame[2] == cls_id and frame[3] == msg_id:
                        last = bytes(frame)
                    i = end
                    continue
        i += 1
    return last


def any_ubx(buf):
    """Return list of (cls,id) tuples for every checksum-valid UBX frame."""
    out = []
    i, n = 0, len(buf)
    while i < n - 7:
        if buf[i] == 0xB5 and buf[i + 1] == 0x62:
            length = buf[i + 4] | (buf[i + 5] << 8)
            end = i + 6 + length + 2
            if end <= n:
                frame = buf[i:end]
                ck_a = ck_b = 0
                for x in frame[2:-2]:
                    ck_a = (ck_a + x) & 0xFF
                    ck_b = (ck_b + ck_a) & 0xFF
                if ck_a == frame[-2] and ck_b == frame[-1]:
                    out.append((frame[2], frame[3]))
                    i = end
                    continue
        i += 1
    return out


# The F9P's replies arrive intermittently, so single-shot reads miss them.
# Re-send and re-read several times until the reply is captured.
RETRIES = 8
TRY_BUDGET = 0.6


def send_and_ack(ser, msg, debug=False):
    """Write a UBX message (retrying), return True (ACK) / False (NAK) / None."""
    seen = []
    for _ in range(RETRIES):
        ser.reset_input_buffer()
        ser.write(msg.serialize())
        ser.flush()
        frames = any_ubx(read_raw(ser, TRY_BUDGET))
        seen += frames
        if (0x05, 0x01) in frames:
            return True
        if (0x05, 0x00) in frames:
            return False
    if debug:
        uniq = sorted(set(seen))
        print(f"      [debug] no ACK after {RETRIES} tries; UBX frames seen: "
              f"{uniq if uniq else 'NONE (receiver sent nothing back)'}")
    return None


# ---------- config helpers (CFG-VALSET / CFG-VALGET) ----------

def valset(ser, cfgdata, layers, label, debug=False):
    msg = UBXMessage.config_set(layers, 0, cfgdata)
    ack = send_and_ack(ser, msg, debug=debug)
    print(f"  {label}: {'OK' if ack else ('NAK' if ack is False else 'no ACK')}")
    return ack


def get_tmode_mode(ser):
    """Read CFG_TMODE_MODE from RAM (0=disabled, 1=survey-in, 2=fixed)."""
    poll = UBXMessage.config_poll(0, 0, ["CFG_TMODE_MODE"])
    for _ in range(RETRIES):
        ser.reset_input_buffer()
        ser.write(poll.serialize())
        ser.flush()
        frame = find_ubx(read_raw(ser, TRY_BUDGET), 0x06, 0x8B)
        if frame:
            try:
                return UBXReader.parse(frame).__dict__.get("CFG_TMODE_MODE")
            except Exception:
                pass
    return None


# ---------- steps ----------

def check_3d_fix(ser):
    print("Checking for a 3D GNSS fix...")
    poll = UBXMessage("NAV", "NAV-PVT", msgmode=0)
    for _ in range(FIX_CHECK_ATTEMPTS):
        ser.reset_input_buffer()
        ser.write(poll.serialize())
        ser.flush()
        frame = find_ubx(read_raw(ser, 1.0), 0x01, 0x07)
        if frame:
            try:
                p = UBXReader.parse(frame)
                ft = p.__dict__.get("fixType", 0)
                nsv = p.__dict__.get("numSV", 0)
                print(f"  fixType={ft}, numSV={nsv}")
                if ft >= 3:
                    print("  3D fix confirmed.")
                    return True
            except Exception:
                pass
    print("  WARNING: no confirmed 3D fix. Survey may stall until the sky clears.")
    return False


def start_survey_in(ser, min_dur, acc_01mm, layers):
    print(f"Starting Survey-In (min {min_dur}s, accuracy <= {acc_01mm/10000.0:.2f} m)...")
    valset(ser, [("CFG_TMODE_MODE", 0)], layers, "disable old base mode")
    time.sleep(0.5)
    valset(ser, [("CFG_NAVSPG_DYNMODEL", 2)], layers, "dynamic model = stationary")
    time.sleep(0.5)
    cfg = [
        ("CFG_TMODE_MODE", 1),                 # 1 = survey-in
        ("CFG_TMODE_SVIN_MIN_DUR", min_dur),   # seconds
        ("CFG_TMODE_SVIN_ACC_LIMIT", acc_01mm),  # 0.1 mm units
    ]
    for attempt in range(3):
        valset(ser, cfg, layers, f"enable survey-in (try {attempt + 1})", debug=True)
        time.sleep(2)
        mode = get_tmode_mode(ser)
        print(f"  CFG_TMODE_MODE now = {mode} (1 = survey-in)")
        if mode == 1:
            print("  Survey-In active.")
            return True
    print("  ERROR: could not activate Survey-In.")
    return False


def monitor_survey(ser):
    print("Monitoring Survey-In (checking every 5 s)...")
    poll = UBXMessage("NAV", "NAV-SVIN", msgmode=0)
    start = time.time()
    while time.time() - start < SURVEY_MAX_WAIT:
        ser.reset_input_buffer()
        ser.write(poll.serialize())
        ser.flush()
        frame = find_ubx(read_raw(ser, 1.0), 0x01, 0x3B)
        if frame:
            try:
                p = UBXReader.parse(frame)
                active = p.__dict__.get("active", 0)
                obs = p.__dict__.get("obs", 0)
                acc = p.__dict__.get("meanAcc", 0) / 10000.0
                elapsed = int(time.time() - start)
                print(f"  [{elapsed:3d}s] obs={obs}, meanAcc={acc:.2f} m, active={active}")
                if not active and obs > 0:
                    print("  Survey-In complete.")
                    return True
            except Exception:
                pass
        time.sleep(5)
    print("  WARNING: survey monitoring timed out (it may still be running).")
    return False


def set_outputs(ser, rate, layers, label):
    """Set the RTCM types + NMEA GGA to `rate` (0 = off) on USB and UART1."""
    cfg = []
    for t in RTCM_TYPES:
        cfg.append((f"CFG_MSGOUT_RTCM_3X_TYPE{t}_USB", rate))
        cfg.append((f"CFG_MSGOUT_RTCM_3X_TYPE{t}_UART1", rate))
    cfg.append(("CFG_MSGOUT_NMEA_ID_GGA_USB", rate))
    cfg.append(("CFG_MSGOUT_NMEA_ID_GGA_UART1", rate))
    return valset(ser, cfg, layers, label)


def quiet_port(ser, layers):
    """Silence the F9P's periodic output so config writes/reads are reliable.

    Configuring while the receiver floods the port with RTCM makes the small
    ACK / VALGET replies easy to miss. Silencing first removes that noise.
    """
    print("Quieting the port for reliable configuration...")
    set_outputs(ser, 0, layers, "silence RTCM + GGA output")
    time.sleep(1.0)
    ser.reset_input_buffer()
    leftover = read_raw(ser, 1.0)
    print(f"  stream now ~{len(leftover)} bytes/s (was ~thousands)")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD
    min_dur = int(sys.argv[3]) if len(sys.argv) > 3 else SURVEY_MIN_DURATION
    acc_m = float(sys.argv[4]) if len(sys.argv) > 4 else SURVEY_ACC_LIMIT_M
    persist = sys.argv[5].lower() if len(sys.argv) > 5 else "flash"
    acc_01mm = int(acc_m * 10000)

    if persist == "ram":
        layers = LAYER_RAM
        persist_desc = "RAM only (NOT persisted; reverts on F9P power-cycle)"
    else:
        layers = LAYER_RAM | LAYER_BBR | LAYER_FLASH
        persist_desc = "RAM+BBR+Flash (persisted across power cycles)"

    print("F9P Survey-In for a New Location")
    print("=" * 40)
    print(f"Port: {port}  Baud: {baud}")
    print(f"Survey: min {min_dur}s, accuracy limit {acc_m:.2f} m")
    print(f"Persist: {persist_desc}")
    print("No factory reset will be performed.\n")

    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            time.sleep(0.5)
            print("Connected.\n")

            check_3d_fix(ser)
            print()
            quiet_port(ser, layers)
            print()
            if not start_survey_in(ser, min_dur, acc_01mm, layers):
                print("\nCould not start the survey. Aborting before RTCM enable.")
                return
            print()
            if monitor_survey(ser):
                print("\nSurvey done -- re-enabling RTCM output.\n")
                set_outputs(ser, 1, layers, f"enable RTCM {', '.join(RTCM_TYPES)} + GGA")
                print("\nBase station is configured"
                      + (" and persisted to flash." if layers & LAYER_FLASH else " in RAM only."))
            else:
                print("\nSurvey did not confirm complete -- not enabling RTCM.")
                print("Re-run, or check the antenna's sky view.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as e:
        print(f"\nError: {e}")


if __name__ == "__main__":
    main()
