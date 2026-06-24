#!/usr/bin/env python3
"""
Diagnose F9P Status
===================
Check what's happening with the F9P and restore basic functionality if needed.
"""

import serial
import struct
import time
import sys

class UBXMessage:
    def __init__(self):
        self.SYNC_CHAR1 = 0xB5
        self.SYNC_CHAR2 = 0x62
    
    def calculate_checksum(self, msg_class, msg_id, payload):
        ck_a = 0
        ck_b = 0
        
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

def check_raw_output(ser, duration=10):
    """Check what raw data is coming from the device"""
    print(f"Checking raw output for {duration} seconds...")
    
    start_time = time.time()
    total_bytes = 0
    has_ubx = False
    has_nmea = False
    has_rtcm = False
    
    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            total_bytes += len(data)
            
            # Check for different message types
            if b'\xB5\x62' in data:
                has_ubx = True
            if b'$' in data:
                has_nmea = True
            if b'\xD3' in data:
                has_rtcm = True
            
            # Print first few bytes as hex for diagnosis
            if total_bytes <= 50:
                hex_data = ' '.join(f'{b:02X}' for b in data[:20])
                print(f"Raw data: {hex_data}")
        
        time.sleep(0.1)
    
    print(f"\nRaw output analysis:")
    print(f"  Total bytes received: {total_bytes}")
    print(f"  UBX messages detected: {'Yes' if has_ubx else 'No'}")
    print(f"  NMEA messages detected: {'Yes' if has_nmea else 'No'}")
    print(f"  RTCM messages detected: {'Yes' if has_rtcm else 'No'}")
    
    return total_bytes > 0, has_ubx, has_nmea, has_rtcm

def request_device_status(ser):
    """Request device version and configuration status"""
    ubx = UBXMessage()
    
    print("Requesting device status...")
    
    # Request version information
    version_msg = ubx.create_message(0x0A, 0x04, b'')  # MON-VER
    
    # Request timing mode status  
    tmode_msg = ubx.create_message(0x06, 0x71, b'')    # CFG-TMODE3 poll
    
    try:
        # Send version request
        ser.write(version_msg)
        ser.flush()
        time.sleep(0.5)
        
        # Send timing mode request
        ser.write(tmode_msg)
        ser.flush()
        time.sleep(0.5)
        
        # Read responses
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"Received {len(response)} bytes of response data")
            
            # Look for version info
            if b'FWVER=' in response:
                fw_start = response.find(b'FWVER=')
                fw_end = response.find(b'\x00', fw_start)
                if fw_end > fw_start:
                    fw_version = response[fw_start:fw_end].decode('ascii', errors='ignore')
                    print(f"Firmware: {fw_version}")
            
            return True
        else:
            print("No response received - device may not be responding")
            return False
            
    except Exception as e:
        print(f"Error requesting status: {e}")
        return False

def restore_basic_nmea(ser):
    """Restore basic NMEA output to get the device working"""
    ubx = UBXMessage()
    
    print("Restoring basic NMEA output...")
    
    # Enable basic NMEA messages
    nmea_configs = [
        (0xF0, 0x00, 1),  # GGA at 1Hz
        (0xF0, 0x04, 1),  # RMC at 1Hz
        (0xF0, 0x05, 1),  # VTG at 1Hz
    ]
    
    success_count = 0
    
    for msg_class, msg_id, rate in nmea_configs:
        # Enable on UART1
        payload = struct.pack('BBBBBBBB', msg_class, msg_id, 0, rate, 0, 0, 0, 0)
        message = ubx.create_message(0x06, 0x01, payload)
        
        try:
            ser.write(message)
            ser.flush()
            time.sleep(0.2)
            success_count += 1
        except Exception as e:
            print(f"Error enabling NMEA {msg_class:02X}-{msg_id:02X}: {e}")
    
    # Disable timing mode (return to rover mode)
    print("Disabling base station mode...")
    try:
        # CFG-TMODE3 with mode = 0 (disable)
        tmode_payload = struct.pack('<BBHHIIIIIIIII',
            0, 0, 0x0000, 0,  # version, reserved, flags=disabled, reserved
            0, 0, 0,          # position (unused)
            0, 0, 0,          # high precision (unused)
            0, 0, 0           # accuracy, survey params (unused)
        )
        
        tmode_message = ubx.create_message(0x06, 0x71, tmode_payload)
        ser.write(tmode_message)
        ser.flush()
        time.sleep(0.5)
        print("Base station mode disabled")
        
    except Exception as e:
        print(f"Error disabling base station mode: {e}")
    
    return success_count

def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else '/dev/f9p'
    baudrate = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200
    
    print("F9P Status Diagnosis and Recovery")
    print("=" * 40)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(1)
        print("Serial connection established")
        
        # Step 1: Check raw output
        print("\n1. Checking raw device output...")
        has_data, has_ubx, has_nmea, has_rtcm = check_raw_output(ser, 5)
        
        if not has_data:
            print("\nNo data detected - possible issues:")
            print("  - Device not powered")
            print("  - Wrong serial port")
            print("  - Wrong baud rate")
            print("  - Device in unusable state")
            
        elif has_rtcm and not has_nmea:
            print("\nDevice is outputting RTCM only")
            print("This means the RTCM-only configuration worked!")
            print("The issue might be with your rtcm_server_0714.py parsing")
            
        elif has_nmea and not has_rtcm:
            print("\nDevice is in rover mode (NMEA only)")
            print("Base station configuration didn't take effect")
            
        elif has_ubx:
            print("\nDevice is outputting UBX binary data")
            print("May need to restore NMEA configuration")
        
        # Step 2: Request status if device is responding
        if has_data:
            print("\n2. Requesting device status...")
            request_device_status(ser)
        
        # Step 3: Offer recovery options
        print("\n3. Recovery options:")
        
        if not has_data:
            print("Try:")
            print("  - Check physical connections")
            print("  - Try different baud rates (9600, 38400, 115200)")
            print("  - Power cycle the device")
            
        elif has_rtcm:
            print("RTCM is working! Try:")
            print("  - Test with: timeout 10 cat /dev/f9p | hexdump -C")
            print("  - Check rtcm_server_0714.py RTCM parsing logic")
            print("  - Verify RTCM message format")
            
        else:
            print("Restoring basic NMEA output...")
            restore_count = restore_basic_nmea(ser)
            print(f"Restored {restore_count} NMEA message types")
            
            print("\nWait 10 seconds then check output again...")
            time.sleep(10)
            
            print("Checking output after restoration...")
            has_data2, has_ubx2, has_nmea2, has_rtcm2 = check_raw_output(ser, 5)
            
            if has_nmea2:
                print("NMEA restored successfully!")
                print("Device is back in working rover mode")
            else:
                print("Restoration failed - device may need factory reset")
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == "__main__":
    main()