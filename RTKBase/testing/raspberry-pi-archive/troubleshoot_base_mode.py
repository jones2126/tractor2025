#!/usr/bin/env python3
"""
Troubleshoot Base Station Mode
==============================
Diagnose why base station mode is being rejected and try alternatives.
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

def wait_for_response(ser, timeout=3):
    """Wait for any UBX response and return full details"""
    start_time = time.time()
    buffer = b''
    
    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            buffer += data
            
            # Look for UBX messages
            for i in range(len(buffer) - 7):
                if (buffer[i:i+2] == b'\xB5\x62' and 
                    len(buffer) >= i + 8):
                    
                    msg_class = buffer[i+2]
                    msg_id = buffer[i+3]
                    length = struct.unpack('<H', buffer[i+4:i+6])[0]
                    
                    if len(buffer) >= i + 8 + length:
                        payload = buffer[i+6:i+6+length]
                        
                        if msg_class == 0x05 and msg_id == 0x01:
                            return "ACK", payload
                        elif msg_class == 0x05 and msg_id == 0x00:
                            return "NACK", payload
                        else:
                            return f"MSG-{msg_class:02X}{msg_id:02X}", payload
        
        time.sleep(0.1)
    
    return "TIMEOUT", b''

def check_current_timing_mode(ser):
    """Check current timing mode configuration"""
    ubx = UBXMessage()
    
    print("Checking current timing mode...")
    
    # Poll CFG-TMODE3
    poll_message = ubx.create_message(0x06, 0x71, b'')
    
    try:
        ser.write(poll_message)
        ser.flush()
        
        response_type, payload = wait_for_response(ser, 3)
        
        if response_type.startswith("MSG-0671") and len(payload) >= 40:
            version = payload[0]
            flags = struct.unpack('<H', payload[2:4])[0]
            
            mode = flags & 0x0003
            mode_names = {0: "Disabled", 1: "Survey-In", 2: "Fixed", 3: "Reserved"}
            
            print(f"  Current mode: {mode_names.get(mode, 'Unknown')} (flags=0x{flags:04X})")
            
            if mode == 1:  # Survey-in
                svin_min_dur = struct.unpack('<I', payload[32:36])[0]
                svin_acc_limit = struct.unpack('<I', payload[36:40])[0]
                print(f"  Survey-in duration: {svin_min_dur}s, accuracy: {svin_acc_limit/10000:.1f}m")
            
            return mode
        else:
            print(f"  Unexpected response: {response_type}")
            return None
            
    except Exception as e:
        print(f"  Error checking timing mode: {e}")
        return None

def check_navigation_status(ser):
    """Check navigation engine status"""
    ubx = UBXMessage()
    
    print("Checking navigation status...")
    
    # Request NAV-STATUS
    status_message = ubx.create_message(0x01, 0x03, b'')
    
    try:
        ser.write(status_message)
        ser.flush()
        
        response_type, payload = wait_for_response(ser, 3)
        
        if response_type.startswith("MSG-0103") and len(payload) >= 16:
            flags = payload[4]
            fix_stat = payload[5]
            flags2 = payload[6]
            
            print(f"  GPS fix status: 0x{fix_stat:02X}")
            print(f"  Flags: 0x{flags:02X}")
            print(f"  Power safe mode: {'Yes' if (flags2 & 0x01) else 'No'}")
            
            # Check if we have a valid fix
            gps_fix_ok = bool(flags & 0x01)
            diff_soln = bool(flags & 0x02)
            
            print(f"  GPS fix OK: {gps_fix_ok}")
            print(f"  Differential solution: {diff_soln}")
            
            return gps_fix_ok
        else:
            print(f"  Unexpected response: {response_type}")
            return None
            
    except Exception as e:
        print(f"  Error checking navigation status: {e}")
        return None

def try_simple_base_mode(ser):
    """Try a very simple base station configuration"""
    ubx = UBXMessage()
    
    print("Trying minimal base station configuration...")
    
    # Minimal CFG-TMODE3 for survey-in with very relaxed parameters
    version = 0
    reserved1 = 0
    flags = 0x0001  # Survey-in mode only
    reserved2 = 0
    
    # All position fields = 0 (not used in survey-in)
    ecef_fields = [0] * 6
    
    # Very relaxed survey parameters
    fixedPosAcc = 50000    # 5m accuracy 
    svinMinDur = 60        # 1 minute
    svinAccLimit = 100000  # 10m accuracy limit
    
    payload = struct.pack('<BBHHIIIIIIIII',
        version, reserved1, flags, reserved2,
        *ecef_fields,
        fixedPosAcc, svinMinDur, svinAccLimit
    )
    
    message = ubx.create_message(0x06, 0x71, payload)
    
    try:
        ser.write(message)
        ser.flush()
        
        response_type, response_payload = wait_for_response(ser, 3)
        
        if response_type == "ACK":
            print("  Minimal base mode ACCEPTED")
            return True
        elif response_type == "NACK":
            print("  Minimal base mode REJECTED")
            if len(response_payload) >= 4:
                nack_class = response_payload[0]
                nack_id = response_payload[1]
                print(f"    NACK for: 0x{nack_class:02X} 0x{nack_id:02X}")
            return False
        else:
            print(f"  Unexpected response: {response_type}")
            return False
            
    except Exception as e:
        print(f"  Error trying base mode: {e}")
        return False

def try_alternative_commands(ser):
    """Try alternative approaches to enable base station"""
    print("Trying alternative base station methods...")
    
    # Method 1: NMEA command
    print("1. Trying NMEA command...")
    try:
        nmea_cmd = "$PUBX,41,1,0003,0001,115200,0\r\n"
        ser.write(nmea_cmd.encode('ascii'))
        ser.flush()
        time.sleep(1)
        print("   NMEA command sent")
    except Exception as e:
        print(f"   NMEA error: {e}")
    
    # Method 2: Try disabling first, then enabling
    print("2. Trying disable-then-enable...")
    ubx = UBXMessage()
    
    try:
        # Disable first
        disable_payload = struct.pack('<BBHHIIIIIIIII',
            0, 0, 0x0000, 0,  # Disabled mode
            0, 0, 0, 0, 0, 0,
            0, 0, 0
        )
        disable_msg = ubx.create_message(0x06, 0x71, disable_payload)
        ser.write(disable_msg)
        ser.flush()
        time.sleep(1)
        
        # Then enable survey-in
        enable_payload = struct.pack('<BBHHIIIIIIIII',
            0, 0, 0x0001, 0,  # Survey-in mode
            0, 0, 0, 0, 0, 0,
            10000, 60, 50000  # 1m acc, 60s, 5m limit
        )
        enable_msg = ubx.create_message(0x06, 0x71, enable_payload)
        ser.write(enable_msg)
        ser.flush()
        
        response_type, _ = wait_for_response(ser, 3)
        if response_type == "ACK":
            print("   Disable-then-enable WORKED")
            return True
        else:
            print(f"   Disable-then-enable: {response_type}")
    except Exception as e:
        print(f"   Disable-then-enable error: {e}")
    
    return False

def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else '/dev/f9p'
    baudrate = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200
    
    print("F9P Base Station Mode Troubleshooting")
    print("=" * 45)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(1)
        print("Serial connection established")
        
        # Step 1: Check current status
        print("\n=== DIAGNOSTIC PHASE ===")
        current_mode = check_current_timing_mode(ser)
        gps_fix = check_navigation_status(ser)
        
        print(f"\nDiagnostic summary:")
        print(f"  Current timing mode: {current_mode}")
        print(f"  GPS fix available: {gps_fix}")
        
        # Step 2: Try different approaches
        print("\n=== SOLUTION ATTEMPTS ===")
        
        if current_mode == 1:
            print("Already in survey-in mode - checking for RTCM...")
            # Monitor briefly for RTCM
            time.sleep(5)
            buffer = b''
            if ser.in_waiting > 0:
                buffer = ser.read(ser.in_waiting)
            
            rtcm_count = buffer.count(b'\xD3')
            if rtcm_count > 0:
                print(f"RTCM messages detected: {rtcm_count}")
                print("Base station is working!")
            else:
                print("No RTCM detected despite being in survey-in mode")
        
        elif gps_fix is False:
            print("No GPS fix - base station mode requires GPS lock")
            print("Wait for GPS fix and try again")
        
        else:
            # Try various methods
            success = try_simple_base_mode(ser)
            
            if not success:
                success = try_alternative_commands(ser)
            
            if success:
                print("\nBase station mode enabled - waiting for survey...")
                time.sleep(10)
                print("Check rtcm_server_0714.py for RTCM messages")
            else:
                print("\nAll methods failed. Possible issues:")
                print("  - Firmware doesn't support base station mode")
                print("  - Hardware limitation")
                print("  - License/feature not enabled")
                print("  - Device needs GPS lock first")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == "__main__":
    main()