#!/usr/bin/env python3
"""
Check F9P Survey-In Status
==========================
This script checks the current survey-in status and can force enable base station mode.
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

def request_survey_status(ser):
    """Request UBX-NAV-SVIN message to check survey-in status"""
    ubx = UBXMessage()
    
    # UBX-NAV-SVIN request (no payload)
    message = ubx.create_message(0x01, 0x3B, b'')
    
    try:
        ser.write(message)
        ser.flush()
        print("📡 Requesting survey-in status...")
        return True
    except Exception as e:
        print(f"❌ Error requesting survey status: {e}")
        return False

def parse_svin_response(data):
    """Parse UBX-NAV-SVIN response"""
    # Look for UBX-NAV-SVIN response (0x01 0x3B)
    for i in range(len(data) - 40):
        if (data[i:i+2] == b'\xB5\x62' and 
            len(data) >= i + 40 and
            data[i+2] == 0x01 and data[i+3] == 0x3B):
            
            # Parse the SVIN message (40 bytes payload)
            payload = data[i+6:i+46]
            if len(payload) >= 40:
                
                # Parse fields from payload
                version = payload[0]
                reserved1 = payload[1:4]
                iTOW = struct.unpack('<I', payload[4:8])[0]
                dur = struct.unpack('<I', payload[8:12])[0]  # Duration in seconds
                meanX = struct.unpack('<i', payload[12:16])[0]  # ECEF X in cm
                meanY = struct.unpack('<i', payload[16:20])[0]  # ECEF Y in cm  
                meanZ = struct.unpack('<i', payload[20:24])[0]  # ECEF Z in cm
                meanXHP = struct.unpack('<b', payload[24:25])[0]  # High precision X in 0.1mm
                meanYHP = struct.unpack('<b', payload[25:26])[0]  # High precision Y in 0.1mm
                meanZHP = struct.unpack('<b', payload[26:27])[0]  # High precision Z in 0.1mm
                reserved2 = payload[27]
                meanAcc = struct.unpack('<I', payload[28:32])[0]  # Mean accuracy in 0.1mm
                obs = struct.unpack('<I', payload[32:36])[0]  # Number of observations
                valid = payload[36]  # Survey-in position validity flags
                active = payload[37]  # Survey-in in progress flag
                reserved3 = payload[38:40]
                
                return {
                    'iTOW': iTOW,
                    'duration': dur,
                    'meanX_cm': meanX,
                    'meanY_cm': meanY, 
                    'meanZ_cm': meanZ,
                    'meanAcc_mm': meanAcc / 10.0,  # Convert 0.1mm to mm
                    'observations': obs,
                    'valid': bool(valid & 0x01),
                    'active': bool(active & 0x01),
                }
    
    return None

def create_cfg_valset_message(key_id, value, value_size):
    ubx = UBXMessage()
    
    version = 0x01
    layer = 0x01  # RAM
    reserved = 0x0000
    
    payload = bytearray()
    payload.append(version)
    payload.append(layer)
    payload.extend(struct.pack('<H', reserved))
    payload.extend(struct.pack('<I', key_id))
    
    if value_size == 1:
        payload.append(value)
    elif value_size == 2:
        payload.extend(struct.pack('<H', value))
    elif value_size == 4:
        payload.extend(struct.pack('<I', value))
    
    return ubx.create_message(0x06, 0x8A, payload)

def force_enable_rtcm(ser):
    """Force enable RTCM messages regardless of survey-in status"""
    print("\n🔧 Force enabling RTCM messages...")
    
    rtcm_configs = [
        (0x209102fe, 1000, 2),       # RTCM 1005 - Base station position
        (0x209102fd, 1000, 2),       # RTCM 1074 - GPS MSM4
        (0x20910300, 1000, 2),       # RTCM 1084 - GLONASS MSM4  
        (0x20910302, 1000, 2),       # RTCM 1124 - BeiDou MSM4
        (0x20910303, 10000, 2),      # RTCM 1230 - GLONASS bias
    ]
    
    success_count = 0
    for key_id, value, size in rtcm_configs:
        message = create_cfg_valset_message(key_id, value, size)
        
        try:
            ser.write(message)
            ser.flush()
            time.sleep(0.2)
            
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                if b'\xb5\x62\x05\x01' in response:
                    success_count += 1
                    rtcm_type = {
                        0x209102fe: "1005",
                        0x209102fd: "1074", 
                        0x20910300: "1084",
                        0x20910302: "1124",
                        0x20910303: "1230"
                    }.get(key_id, "unknown")
                    print(f"  ✓ RTCM {rtcm_type} enabled")
                    
        except Exception as e:
            print(f"  ❌ Error enabling RTCM: {e}")
    
    return success_count

def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else '/dev/f9p'
    baudrate = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200
    
    print("F9P Survey-In Status Checker")
    print("=" * 35)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(1)
        print("✓ Serial connection established")
        
        # Request survey-in status
        if request_survey_status(ser):
            
            # Wait for response and parse
            print("⏳ Waiting for survey-in status response...")
            time.sleep(2)
            
            if ser.in_waiting > 0:
                response_data = ser.read(ser.in_waiting)
                svin_status = parse_svin_response(response_data)
                
                if svin_status:
                    print(f"\n📊 Survey-In Status:")
                    print(f"  Duration: {svin_status['duration']} seconds")
                    print(f"  Observations: {svin_status['observations']}")
                    print(f"  Mean Accuracy: {svin_status['meanAcc_mm']:.1f} mm")
                    print(f"  Active: {'YES' if svin_status['active'] else 'NO'}")
                    print(f"  Valid: {'YES' if svin_status['valid'] else 'NO'}")
                    
                    if svin_status['active']:
                        print(f"\n🔄 Survey-in is RUNNING")
                        remaining = max(0, 300 - svin_status['duration'])
                        print(f"   Time remaining: ~{remaining} seconds")
                        print(f"   Target accuracy: 1000.0 mm (current: {svin_status['meanAcc_mm']:.1f} mm)")
                        
                    elif svin_status['valid']:
                        print(f"\n✅ Survey-in COMPLETED successfully!")
                        print(f"   Position is valid and should be broadcasting RTCM")
                        print(f"   If no RTCM, trying to force enable...")
                        force_enable_rtcm(ser)
                        
                    else:
                        print(f"\n❌ Survey-in NOT active or completed")
                        print(f"   Trying to restart survey-in and force RTCM...")
                        
                        # Restart survey-in
                        restart_message = create_cfg_valset_message(0x40030001, 1, 1)
                        ser.write(restart_message)
                        time.sleep(0.5)
                        
                        # Force enable RTCM
                        force_enable_rtcm(ser)
                        
                else:
                    print("\n❌ Could not parse survey-in status")
                    print("   Forcing RTCM enable anyway...")
                    force_enable_rtcm(ser)
            else:
                print("\n❌ No response received")
                print("   Forcing RTCM enable anyway...")
                force_enable_rtcm(ser)
        
        print(f"\n🎯 Summary:")
        print(f"  • Check your rtcm_server_0714.py output now")
        print(f"  • RTCM messages should appear within 30 seconds")
        print(f"  • If still no RTCM, the device may need a restart")
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == "__main__":
    main()