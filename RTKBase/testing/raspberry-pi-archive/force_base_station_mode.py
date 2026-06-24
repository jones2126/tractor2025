#!/usr/bin/env python3
"""
Force F9P Base Station Mode (Adaptive + Explicit 1Hz RTCM)
=========================================================
Configures an Ardusimple F9P as a fixed-position base station.

- Waits for receiver to settle into TIME mode
- Tries enabling RTCM 1005 (retries once, falls back to 1006)
- Enables RTCM 1074, 1084, 1094, 1230 explicitly at 1 Hz
- Adaptive monitoring: exits early once enough RTCM traffic is seen
"""

import serial
import struct
import time
import sys

DEFAULT_WAIT_TIME = 120       # seconds to wait after TMODE3
DEFAULT_MONITOR_TIME = 120    # seconds to monitor max
MIN_RTCM_MESSAGES = 50        # exit monitoring early after this many

class UBXMessage:
    def __init__(self):
        self.SYNC_CHAR1 = 0xB5
        self.SYNC_CHAR2 = 0x62

    def calculate_checksum(self, msg_class, msg_id, payload):
        ck_a, ck_b = 0, 0
        ck_a = (ck_a + msg_class) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
        ck_a = (ck_a + msg_id) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
        length = len(payload)
        ck_a = (ck_a + (length & 0xFF)) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
        ck_a = (ck_a + ((length >> 8) & 0xFF)) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
        for byte in payload:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return ck_a, ck_b

    def create_message(self, msg_class, msg_id, payload):
        length = len(payload)
        ck_a, ck_b = self.calculate_checksum(msg_class, msg_id, payload)
        message = bytearray()
        message.append(self.SYNC_CHAR1)
        message.append(self.SYNC_CHAR2)
        message.append(msg_class)
        message.append(msg_id)
        message.extend(struct.pack('<H', length))
        message.extend(payload)
        message.append(ck_a)
        message.append(ck_b)
        return bytes(message)

def wait_for_ack(ser, timeout=2):
    start_time = time.time()
    buffer = b''
    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            buffer += data
            for i in range(len(buffer) - 9):
                if buffer[i:i+2] == b'\xB5\x62' and len(buffer) >= i + 10:
                    msg_class = buffer[i+2]
                    msg_id = buffer[i+3]
                    if msg_class == 0x05 and msg_id == 0x01:
                        return "ACK"
                    elif msg_class == 0x05 and msg_id == 0x00:
                        return "NACK"
        time.sleep(0.05)
    return "TIMEOUT"

def reset_device(ser):
    ubx = UBXMessage()
    payload = struct.pack('<III', 0xFFFFFFFF, 0x00000000, 0x00000000)
    ser.write(ubx.create_message(0x06, 0x09, payload))
    ser.flush()
    print("  ✓ Device reset command sent")
    time.sleep(2)

def configure_base_station_mode(ser):
    ubx = UBXMessage()
    print("Configuring base station mode (fixed position)...")
    lat, lon, alt = 40.34525533, -80.12876667, 327.100
    lat_scaled = int(lat * 1e7)
    lon_scaled = int(lon * 1e7)
    alt_scaled = int(alt * 100)

    lat_hp = int(round((lat * 1e7 - lat_scaled) * 100))
    lon_hp = int(round((lon * 1e7 - lon_scaled) * 100))
    alt_hp = int(round((alt * 100 - alt_scaled) * 100))
    if lat_hp > 127: lat_hp -= 256
    if lon_hp > 127: lon_hp -= 256
    if alt_hp > 127: alt_hp -= 256

    version, reserved1, flags, reserved2 = 0, 0, 0x0001 | 0x0200, 0
    fixedPosAcc, svinMinDur, svinAccLimit = 100, 0, 0

    payload = struct.pack('<BBHiiibbbBIII',
        version, reserved1, flags,
        lat_scaled, lon_scaled, alt_scaled,
        lat_hp, lon_hp, alt_hp,
        reserved2, fixedPosAcc, svinMinDur, svinAccLimit
    )
    ser.write(ubx.create_message(0x06, 0x71, payload))
    ser.flush()
    if wait_for_ack(ser, 3) == "ACK":
        print("  ✓ Base station mode acknowledged")
        return True
    print("  ✗ Base station mode rejected")
    return False

def send_rtcm_message(ser, msg_class, msg_id, description, rate_uart1=1, rate_usb=1):
    ubx = UBXMessage()
    payload = struct.pack('BBBBBBBB', msg_class, msg_id, 0, rate_uart1, 0, rate_usb, 0, 0)
    ser.write(ubx.create_message(0x06, 0x01, payload))
    ser.flush()
    response = wait_for_ack(ser, 2)
    if response == "ACK":
        print(f"  ✓ RTCM {description} enabled @ 1 Hz")
        return True
    elif response == "NACK":
        print(f"  ✗ RTCM {description} - REJECTED")
    else:
        print(f"  → RTCM {description} - no ACK")
    return False

def enable_rtcm_messages(ser):
    print("Enabling RTCM 1005 (Base Station ARP)...")
    if not send_rtcm_message(ser, 0xF5, 0x05, "1005 - Base Station ARP"):
        print("Waiting 30s and retrying 1005...")
        time.sleep(30)
        if not send_rtcm_message(ser, 0xF5, 0x05, "1005 - Base Station ARP (retry)"):
            print("Falling back to RTCM 1006...")
            if not send_rtcm_message(ser, 0xF5, 0x06, "1006 - Base Station ARP w/ Height"):
                print("✗ Neither 1005 nor 1006 accepted.")
                return False
    # Enable the rest, all explicitly at 1 Hz
    print("Enabling remaining RTCM MSM messages...")
    send_rtcm_message(ser, 0xF5, 0x4A, "1074 - GPS MSM4")
    send_rtcm_message(ser, 0xF5, 0x54, "1084 - GLONASS MSM4")
    send_rtcm_message(ser, 0xF5, 0x5E, "1094 - Galileo MSM4")
    send_rtcm_message(ser, 0xF5, 0xE6, "1230 - GLONASS Biases")
    return True

def save_configuration(ser):
    ubx = UBXMessage()
    payload = struct.pack('<III', 0x00000000, 0x0000061F, 0x00000000)
    ser.write(ubx.create_message(0x06, 0x09, payload))
    ser.flush()
    print("  ✓ Save command sent")

def monitor_for_rtcm(ser, duration=DEFAULT_MONITOR_TIME):
    print(f"Monitoring for RTCM messages (up to {duration}s)...")
    start = time.time()
    rtcm_count, total_bytes = 0, 0
    while time.time() - start < duration:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            total_bytes += len(data)
            rtcm_count += data.count(b'\xD3')
        if rtcm_count >= MIN_RTCM_MESSAGES:
            print(f"  ✓ PASS: {rtcm_count} RTCM messages in {(time.time()-start):.1f}s")
            return True
        time.sleep(0.1)
    print(f"  ✗ FAIL: Only {rtcm_count} RTCM messages detected in {duration}s")
    return False

def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else '/dev/f9p'
    baudrate = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200
    wait_time = int(sys.argv[3]) if len(sys.argv) >= 4 else DEFAULT_WAIT_TIME

    print("Force F9P Base Station Mode (Explicit 1 Hz)")
    print("=" * 45)
    print(f"Port: {port}\nBaudrate: {baudrate}\nWait time: {wait_time}s")

    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(1)
        print("✓ Serial connection established")

        print("\n0. Resetting device to factory settings...")
        reset_device(ser)

        print("\n1. Setting base station mode...")
        if configure_base_station_mode(ser):
            print(f"Waiting {wait_time}s for base mode to settle...")
            time.sleep(wait_time)

        print("\n2. Enabling RTCM messages...")
        if enable_rtcm_messages(ser):
            print("✓ RTCM messages enabled @ 1 Hz")
        else:
            print("⚠ RTCM setup incomplete")

        print("\n3. Saving configuration...")
        save_configuration(ser)

        print("\n4. Waiting for configuration to take effect...")
        time.sleep(10)

        print("\n5. Monitoring for RTCM output...")
        monitor_for_rtcm(ser)

    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == "__main__":
    main()
