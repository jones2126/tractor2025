#!/usr/bin/env python3
"""
pyubx2_base_survey.py — Robust Survey-In Base Configurator
=========================================================
Configures an Ardusimple F9P as a Survey-In base station.

Flow:
  0) Reset receiver to factory defaults
  1) Wait for 3D fix (up to 60s)
  2) Configure TMODE3 for Survey-In
  3) Poll CFG-TMODE3 until active (retry if needed)
  4) Monitor NAV-SVIN until survey completes or 10-min timeout
  5) Enable RTCM messages (1005,1074,1084,1094,1230 @ 1Hz)
  6) Save configuration
  7) Wait up to 5 minutes for 1005 frames
"""

import sys, time, serial
from pyubx2 import UBXMessage, UBXReader, SET

SURVEY_MIN_DURATION = 120      # seconds
SURVEY_ACC_LIMIT = 50000       # 0.1 mm units (5 m)
SURVEY_MAX_WAIT = 600          # seconds (10 min)
FIX_WAIT_TIMEOUT = 60          # wait up to 60s for 3D fix

WAIT_FOR_1005_TIMEOUT = 300    # seconds
WAIT_FOR_1005_INTERVAL = 20    # print every 20s

# ---------- UBX Helpers ----------

def build_cfg_cfg(clearMask, saveMask, loadMask, deviceMask):
    sync = b"\xB5\x62"
    msg_class = b"\x06"
    msg_id = b"\x09"
    payload = (
        int(clearMask).to_bytes(4, "little")
        + int(saveMask).to_bytes(4, "little")
        + int(loadMask).to_bytes(4, "little")
        + int(deviceMask).to_bytes(1, "little")
    )
    length = len(payload).to_bytes(2, "little")
    ck_a = ck_b = 0
    for byte in msg_class + msg_id + length + payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return sync + msg_class + msg_id + length + payload + bytes([ck_a, ck_b])

def send_and_wait_ack(ser, msg, timeout=1.0):
    ser.reset_input_buffer()
    ser.write(msg.serialize())
    ser.flush()
    ubr = UBXReader(ser, protfilter=2)
    end = time.time() + timeout
    while time.time() < end:
        if ser.in_waiting:
            raw, parsed = ubr.read()
            if parsed:
                if parsed.identity == "ACK-ACK":
                    return True
                if parsed.identity == "ACK-NAK":
                    return False
        time.sleep(0.05)
    return None

# ---------- Configuration Steps ----------

def reset_device(ser):
    print("Resetting device to factory defaults...")
    frame = build_cfg_cfg(0xFFFFFFFF, 0x00000000, 0x00000000, 0x17)
    ser.write(frame)
    ser.flush()
    time.sleep(2)
    print("  ✓ Reset command sent")

def wait_for_fix(ser):
    print(f"Waiting up to {FIX_WAIT_TIMEOUT}s for 3D GNSS fix...")
    poll = UBXMessage("NAV", "NAV-PVT", msgmode=0)
    ubr = UBXReader(ser, protfilter=2)
    start = time.time()
    while time.time() - start < FIX_WAIT_TIMEOUT:
        ser.reset_input_buffer()
        ser.write(poll.serialize())
        ser.flush()
        time.sleep(0.2)
        if ser.in_waiting:
            raw, parsed = ubr.read()
            if parsed and parsed.identity == "NAV-PVT":
                fixType = parsed.__dict__.get("fixType", 0)
                numSV = parsed.__dict__.get("numSV", 0)
                print(f"  ➤ fixType={fixType}, numSV={numSV}")
                if fixType >= 3:
                    print("  ✓ 3D fix acquired.")
                    return True
        time.sleep(1)
    print("  ✗ Timed out waiting for 3D fix.")
    return False

def configure_survey_in(ser):
    msg = UBXMessage(
        "CFG", "CFG-TMODE3", msgmode=SET,
        version=0, reserved1=0, flags=0x0001,  # Survey-In
        ecefX=0, ecefY=0, ecefZ=0,
        lat=0, lon=0, height=0,
        latHP=0, lonHP=0, heightHP=0,
        reserved2=0, fixedPosAcc=0,
        svinMinDur=SURVEY_MIN_DURATION,
        svinAccLimit=SURVEY_ACC_LIMIT
    )
    return send_and_wait_ack(ser, msg)

def poll_tmode3(ser):
    poll = UBXMessage("CFG", "CFG-TMODE3", msgmode=0)
    ser.write(poll.serialize())
    time.sleep(0.2)
    ubr = UBXReader(ser, protfilter=2)
    if ser.in_waiting:
        raw, parsed = ubr.read()
        if parsed:
            flags = parsed.__dict__.get("flags")
            return flags
    return None

def ensure_survey_in_started(ser, retries=5):
    for attempt in range(retries):
        ack = configure_survey_in(ser)
        if ack:
            print(f"  ✓ Survey-In command sent (attempt {attempt+1})")
        else:
            print(f"  ⚠ No ACK for Survey-In (attempt {attempt+1})")
        time.sleep(2)  # allow F9P to commit mode change
        flags = poll_tmode3(ser)
        if flags is not None and (flags & 0x0001):
            print("  ✓ Survey-In mode confirmed active.")
            return True
        print("  ⚠ TMODE3 not active yet, retrying...")
    print("  ✗ Unable to activate Survey-In after retries.")
    return False

def poll_survey_status(ser):
    poll = UBXMessage("NAV", "NAV-SVIN", msgmode=0)
    ser.write(poll.serialize())
    ser.flush()
    time.sleep(0.1)
    ubr = UBXReader(ser, protfilter=2)
    if ser.in_waiting:
        raw, parsed = ubr.read()
        if parsed and parsed.identity == "NAV-SVIN":
            return parsed
    return None

def wait_for_survey_completion(ser):
    print(f"Waiting for Survey-In to complete (timeout {SURVEY_MAX_WAIT}s)...")
    start = time.time()
    while time.time() - start < SURVEY_MAX_WAIT:
        status = poll_survey_status(ser)
        if status:
            active = status.__dict__.get("active", 0)
            obs = status.__dict__.get("obs", 0)
            meanAcc = status.__dict__.get("meanAcc", 0) / 10000.0
            print(f"  ➤ SurveyIn: active={active}, obs={obs}, meanAcc={meanAcc:.3f} m")
            if not active and obs > 0:
                print("  ✓ Survey-In completed successfully.")
                return True
        time.sleep(1)
    print("  ✗ Survey-In timed out after 10 min.")
    return False

def disable_rtcm_set(ser, ids):
    for msgID in ids:
        msg = UBXMessage("CFG", "CFG-MSG", msgmode=SET,
                         msgClass=0xF5, msgID=msgID,
                         rateDDC=0, rateUART1=0, rateUART2=0, rateUSB=0,
                         rateSPI=0, rateI2C=0)
        send_and_wait_ack(ser, msg, timeout=0.5)

def enable_rtcm(ser, msgID, label):
    msg = UBXMessage("CFG", "CFG-MSG", msgmode=SET,
                     msgClass=0xF5, msgID=msgID,
                     rateDDC=0, rateUART1=1, rateUART2=0, rateUSB=1,
                     rateSPI=0, rateI2C=0)
    ack = send_and_wait_ack(ser, msg)
    if ack:
        print(f"  ✓ Enabled RTCM {label} @ 1 Hz")
    else:
        print(f"  ⚠ No ACK for RTCM {label}")

def configure_rtcm_messages(ser):
    print("Disabling common RTCM messages...")
    disable_rtcm_set(ser, [0x05, 0x06, 0x4A, 0x54, 0x5E, 0x74, 0xE6])
    print("Enabling selected RTCM @ 1 Hz...")
    enable_rtcm(ser, 0x05, "1005 - Base ARP")
    enable_rtcm(ser, 0x4A, "1074 - GPS MSM4")
    enable_rtcm(ser, 0x54, "1084 - GLONASS MSM4")
    enable_rtcm(ser, 0x5E, "1094 - Galileo MSM4")
    enable_rtcm(ser, 0xE6, "1230 - GLONASS Biases")

def save_configuration(ser):
    print("Saving configuration to flash...")
    frame = build_cfg_cfg(0x00000000, 0x0000FFFF, 0x00000000, 0x17)
    ser.write(frame)
    ser.flush()
    print("  ✓ Configuration saved")

def monitor_for_1005(ser):
    print(f"Waiting up to {WAIT_FOR_1005_TIMEOUT}s for 1005 frames...")
    start = time.time()
    buf = bytearray()
    while time.time() - start < WAIT_FOR_1005_TIMEOUT:
        if ser.in_waiting:
            buf.extend(ser.read(ser.in_waiting))
            i = 0
            while i + 6 < len(buf):
                if buf[i] != 0xD3:
                    i += 1
                    continue
                length = ((buf[i+1] & 0x03) << 8) | buf[i+2]
                frame_len = 3 + length + 3
                if i + frame_len > len(buf): break
                payload = buf[i+3:i+3+length]
                if len(payload) >= 2:
                    msg_type = ((payload[0] << 4) | (payload[1] >> 4)) & 0x0FFF
                    if msg_type == 1005:
                        print(f"  ✓ 1005 detected after {int(time.time()-start)}s")
                        return True
                i += frame_len
            if i:
                del buf[:i]
        if int(time.time() - start) % WAIT_FOR_1005_INTERVAL == 0:
            print(f"  ...still no 1005 after {int(time.time() - start)}s")
        time.sleep(0.1)
    print("  ✗ Timed out waiting for 1005.")
    return False

# ---------- Main ----------

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/f9p"
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print("F9P Base Station Config (Survey-In w/ Fix Wait)")
    print("=" * 55)
    print(f"Port: {port}\nBaudrate: {baudrate}")

    try:
        with serial.Serial(port, baudrate, timeout=2) as ser:
            time.sleep(1)
            print("✓ Serial connection established")

            reset_device(ser)
            if not wait_for_fix(ser):
                print("❌ No 3D fix acquired, Survey-In cannot start.")
                return
            print(f"Configuring TMODE3 for Survey-In (minDur={SURVEY_MIN_DURATION}s, acc={SURVEY_ACC_LIMIT/10000:.2f}m)...")
            if not ensure_survey_in_started(ser):
                print("❌ Survey-In could not be activated. Exiting.")
                return
            if wait_for_survey_completion(ser):
                configure_rtcm_messages(ser)
                save_configuration(ser)
                monitor_for_1005(ser)
            else:
                print("⚠ Survey-In did not complete. Skipping RTCM configuration.")

    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
