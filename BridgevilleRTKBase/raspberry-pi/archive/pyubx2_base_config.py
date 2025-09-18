#!/usr/bin/env python3
"""
pyubx2_base_config.py
=====================
Configures an Ardusimple F9P as a fixed-position RTK base station.

Changes in this version:
- fixedPosAcc tightened to 10 mm (matches u-center defaults)
- 1005 explicitly re-enabled after all other RTCM config (final step)
- Polls CFG-MSG to confirm 1005 is enabled before monitoring
- 5-minute loop checking every 20s for live 1005 frames
"""

import sys
import time
import serial
from collections import defaultdict
from pyubx2 import UBXMessage, UBXReader, SET

DEFAULT_WAIT_TIME = 15
WAIT_FOR_1005_TIMEOUT = 300
WAIT_FOR_1005_INTERVAL = 20

LAT = 40.34525533
LON = -80.12876667
ALT_M = 327.100  # meters

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
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    ser.write(msg.serialize())
    ser.flush()
    ubr = UBXReader(ser, protfilter=2)
    end = time.time() + timeout
    while time.time() < end:
        if ser.in_waiting:
            try:
                raw, parsed = ubr.read()
            except Exception:
                break
            if parsed:
                if parsed.identity == "ACK-ACK":
                    return True
                if parsed.identity == "ACK-NAK":
                    return False
        time.sleep(0.05)
    return None

def reset_device(ser):
    print("Resetting device to factory defaults...")
    frame = build_cfg_cfg(0xFFFFFFFF, 0x00000000, 0x00000000, 0x17)
    ser.write(frame)
    ser.flush()
    time.sleep(2)
    print("  ✓ Reset command sent")

def save_configuration(ser):
    print("Saving configuration to flash...")
    frame = build_cfg_cfg(0x00000000, 0x0000FFFF, 0x00000000, 0x17)
    ser.write(frame)
    ser.flush()
    print("  ✓ Configuration saved")

def _to_hp_byte(val):
    val = int(val)
    return val if val <= 127 else val - 256

def configure_tmode3(ser):
    print("Configuring TMODE3 (fixed position LLA)...")
    lat_scaled = int(LAT * 1e7)
    lon_scaled = int(LON * 1e7)
    alt_scaled = int(ALT_M * 100)

    lat_hp = _to_hp_byte(round((LAT * 1e7 - lat_scaled) * 100))
    lon_hp = _to_hp_byte(round((LON * 1e7 - lon_scaled) * 100))
    alt_hp = _to_hp_byte(round((ALT_M * 100 - alt_scaled) * 100))

    msg = UBXMessage(
        "CFG", "CFG-TMODE3", msgmode=SET,
        version=0, reserved1=0, flags=0x0201,
        ecefX=0, ecefY=0, ecefZ=0,
        lat=lat_scaled, lon=lon_scaled, height=alt_scaled,
        latHP=lat_hp, lonHP=lon_hp, heightHP=alt_hp,
        reserved2=0, fixedPosAcc=10,  # tighter accuracy in mm
        svinMinDur=0, svinAccLimit=0
    )
    ack = send_and_wait_ack(ser, msg)
    if ack is True:
        print("  ✓ TMODE3 applied")
    elif ack is False:
        print("  ✗ TMODE3 rejected")
    else:
        print("  ⚠ TMODE3 not acknowledged (continuing anyway)")

def configure_nav_rate_1hz(ser):
    print("Setting measurement/nav rate to 1 Hz (before TMODE3)...")
    msg = UBXMessage("CFG", "CFG-RATE", msgmode=SET, measRate=1000, navRate=1, timeRef=0)
    ack = send_and_wait_ack(ser, msg)
    if ack is True:
        print("  ✓ CFG-RATE set to 1 Hz")
    elif ack is False:
        print("  ✗ CFG-RATE rejected")
    else:
        print("  ⚠ No ACK for CFG-RATE (continuing...)")

def disable_rtcm_set(ser, ids):
    for msgID in ids:
        msg = UBXMessage("CFG", "CFG-MSG", msgmode=SET,
                         msgClass=0xF5, msgID=msgID,
                         rateDDC=0, rateUART1=0, rateUART2=0, rateUSB=0, rateSPI=0, rateI2C=0)
        send_and_wait_ack(ser, msg, timeout=0.5)

def enable_rtcm(ser, msgID, label):
    msg = UBXMessage("CFG", "CFG-MSG", msgmode=SET,
                     msgClass=0xF5, msgID=msgID,
                     rateDDC=0, rateUART1=1, rateUART2=0, rateUSB=1, rateSPI=0, rateI2C=0)
    ack = send_and_wait_ack(ser, msg)
    if ack:
        print(f"  ✓ Enabled RTCM {label} @ 1 Hz")
    else:
        print(f"  ⚠ No ACK for RTCM {label}")

def disable_all_common_rtcm(ser):
    print("Disabling common RTCM messages...")
    disable_rtcm_set(ser, [0x05, 0x06, 0x4A, 0x54, 0x5E, 0x74, 0xE6])

def enable_selected_rtcm(ser):
    print("Enabling selected RTCM @ 1 Hz...")
    enable_rtcm(ser, 0x05, "1005 - Base ARP")
    enable_rtcm(ser, 0x4A, "1074 - GPS MSM4")
    enable_rtcm(ser, 0x54, "1084 - GLONASS MSM4")
    enable_rtcm(ser, 0x5E, "1094 - Galileo MSM4")
    enable_rtcm(ser, 0xE6, "1230 - GLONASS Biases")

def poll_cfg_msg_status(ser, msgID):
    poll = UBXMessage("CFG", "CFG-MSG", msgmode=0, msgClass=0xF5, msgID=msgID)
    ser.write(poll.serialize())
    time.sleep(0.2)
    ubr = UBXReader(ser, protfilter=2)
    if ser.in_waiting:
        raw, parsed = ubr.read()
        if parsed:
            d = parsed.__dict__
            uart1 = d.get("rateUART1")
            usb = d.get("rateUSB")
            return (uart1, usb, parsed)
    return None

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
                        print(f"  ✓ 1005 detected after {int(time.time() - start)}s")
                        return True
                i += frame_len
            if i:
                del buf[:i]
        if (time.time() - start) % WAIT_FOR_1005_INTERVAL < 0.1:
            print(f"  ...still no 1005 after {int(time.time() - start)}s")
        time.sleep(0.1)
    print("  ✗ Timed out waiting for 1005.")
    return False

def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else "/dev/f9p"
    baudrate = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200
    wait_time = int(sys.argv[3]) if len(sys.argv) >= 4 else DEFAULT_WAIT_TIME

    print("F9P Base Station Config (pyubx2)")
    print("=" * 45)
    print(f"Port: {port}\nBaudrate: {baudrate}\nWait time: {wait_time}s")

    try:
        with serial.Serial(port, baudrate, timeout=2) as ser:
            time.sleep(1)
            print("✓ Serial connection established")

            reset_device(ser)
            configure_nav_rate_1hz(ser)
            configure_tmode3(ser)
            print(f"Waiting {wait_time}s to settle...")
            time.sleep(wait_time)

            disable_all_common_rtcm(ser)
            enable_selected_rtcm(ser)

            print("Re-disabling 1124 (BeiDou MSM4) after TMODE3...")
            disable_rtcm_set(ser, [0x74])

            print("Re-enabling 1005 after all other config...")
            enable_rtcm(ser, 0x05, "1005 - Base ARP (final)")

            status = poll_cfg_msg_status(ser, 0x05)
            if status:
                uart1, usb, parsed = status
                print(f"  ↪ 1005 poll response: UART1={uart1}, USB={usb}")
            else:
                print("  ⚠ Unable to poll 1005 status.")

            save_configuration(ser)
            monitor_for_1005(ser)

    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
