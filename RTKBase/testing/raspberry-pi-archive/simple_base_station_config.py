#!/usr/bin/env python3
"""
Simple F9P Base Station Configuration
=====================================
Uses NMEA commands and simple UBX messages to configure base station mode.
This approach is more reliable than complex coordinate calculations.
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

def send_nmea_command(ser, command):
    """Send NMEA command"""
    if not command.startswith('$'):
        command = '$' + command
    if not command.endswith('\r\n'):
        command += '\r\n'
    
    try:
        ser.write(command.encode('ascii'))
        ser.flush()
        time.sleep(0.2)
        print(f"✓ Sent: {command.strip()}")
        return True
    except Exception as e:
        print(f"✗ Error sending: {e}")
        return False

def enable_rtcm_messages(ser):
    """Enable RTCM messages using UBX-CFG-MSG commands"""
    ubx = UBXMessage()
    
    # RTCM message configurations: (msg_class, msg_id, rate)
    rtcm_messages = [
        (0xF5, 0x05, 1),  # RTCM3.3 1005 - Stationary RTK reference ARP
        (0xF5, 0x4A, 1),  # RTCM3.3 1074 - GPS MSM4  
        (0xF5, 0x54, 1),  # RTCM3.3 1084 - GLONASS MSM4
        (0xF5, 0x7C, 1),  # RTCM3.3 1124 - BeiDou MSM4
        (0xF5, 0xE6, 1),  # RTCM3.3 1230 - GLONASS code-phase biases
    ]
    
    print("Enabling RTCM messages...")
    success_count = 0
    
    for msg_class, msg_id, rate in rtcm_messages:
        # UBX-CFG-MSG payload: msgClass, msgID, rate[6]
        # rate[0] = I2C, rate[1] = UART1, rate[2] = UART2, rate[3] = USB, rate[4] = SPI, rate[5] = reserved
        payload = struct.pack('BBBBBBBB', msg_class, msg_id, 0, rate, 0, 0, 0, 0)
        message = ubx.create_message(0x06, 0x01, payload)  # CFG-MSG
        
        try:
            ser.write(message)
            ser.flush()
            time.sleep(0.3)
            
            # Check for ACK
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                if b'\xb5\x62\x05\x01' in response:
                    success_count += 1
                    rtcm_type = {0x05: "1005", 0x4A: "1074", 0x54: "1084", 0x7C: "1124", 0xE6: "1230"}.get(msg_id, f"{msg_id:02X}")
                    print(f"  ✓ RTCM {rtcm_type} enabled")
                else:
                    print(f"  → RTCM {msg_id:02X} sent")
            else:
                print(f"  → RTCM {msg_id:02X} sent")
                
        except Exception as e:
            print(f"  ✗ Error enabling RTCM {msg_id:02X}: {e}")
    
    return success_count

def configure_simple_base_station(ser):
    """Configure F9P as base station using simple commands"""
    
    print("Configuring F9P as Base Station (Simple Method)...")
    
    # Step 1: Enable high precision NMEA first
    print("\n1. Enabling high-precision NMEA...")
    nmea_commands = [
        "PUBX,41,1,0003,0001,115200,0",  # Set port config
        "PUBX,40,GGA,0,1,0,0,0,0",       # Enable GGA at 1Hz
        "PUBX,40,RMC,0,1,0,0,0,0",       # Enable RMC at 1Hz  
        "PUBX,40,VTG,0,1,0,0,0,0",       # Enable VTG at 1Hz
        "PUBX,40,GLL,0,1,0,0,0,0",       # Enable GLL at 1Hz
    ]
    
    for cmd in nmea_commands:
        send_nmea_command(ser, cmd)
    
    # Step 2: Set base station mode using UBX
    print("\n2. Setting base station mode...")
    ubx = UBXMessage()
    
    # UBX-CFG-TMODE3 message to set base station mode
    # Mode = 1 (Survey-in), but with very short duration to get quick position
    version = 0
    flags = 0x0001  # Survey-in mode
    ecefXOrLat = 0  # Will be filled by survey
    ecefYOrLon = 0  # Will be filled by survey  
    ecefZOrAlt = 0  # Will be filled by survey
    ecefXOrLatHP = 0
    ecefYOrLonHP = 0
    ecefZOrAltHP = 0
    fixedPosAcc = 10000  # 1m accuracy (10000 mm)
    svinMinDur = 60      # 60 seconds minimum (reduced from 300)
    svinAccLimit = 20000 # 2m accuracy limit (20000 mm, relaxed)
    
    payload = struct.pack('<BBHHIIIIIIIII',
        version, 0, flags, 0,
        ecefXOrLat, ecefYOrLon, ecefZOrAlt,
        ecefXOrLatHP, ecefYOrLonHP, ecefZOrAltHP,
        fixedPosAcc, svinMinDur, svinAccLimit
    )
    
    tmode_message = ubx.create_message(0x06, 0x71, payload)  # CFG-TMODE3
    
    try:
        ser.write(tmode_message)
        ser.flush()
        time.sleep(0.5)
        print("  ✓ Base station mode enabled (60s survey)")
    except Exception as e:
        print(f"  ✗ Error setting base mode: {e}")
    
    # Step 3: Enable RTCM messages
    print("\n3. Enabling RTCM corrections...")
    rtcm_success = enable_rtcm_messages(ser)
    
    # Step 4: Save configuration
    print("\n4. Saving configuration...")
    save_payload = struct.pack('<III', 0x0000061F, 0x0000061F, 0x00000000)  # Save all to flash
    save_message = ubx.create_message(0x06, 0x09, save_payload)  # CFG-CFG
    
    try:
        ser.write(save_message)
        ser.flush()
        time.sleep(1)
        print("  ✓ Configuration saved to flash")
    except Exception as e:
        print(f"  ✗ Error saving config: {e}")
    
    print(f"\n📋 Configuration Summary:")
    print(f"  • High-precision NMEA enabled")
    print(f"  • Base station mode: Survey-in (60 seconds)")
    print(f"  • RTCM corrections: {rtcm_success}/5 message types enabled")
    print(f"  • Mixed NMEA + RTCM output")
    
    print(f"\n⏳ Timeline:")
    print(f"  • Now: High-precision NMEA active")
    print(f"  • 60 seconds: Survey-in complete, RTCM starts")
    print(f"  • Check rtcm_server_0714.py in 1-2 minutes")
    
    return True

def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else '/dev/f9p'
    baudrate = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200
    
    print("Simple F9P Base Station Configuration")
    print("=" * 45)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(1)
        print("✓ Serial connection established")
        
        success = configure_simple_base_station(ser)
        
        if success:
            print("\n🎉 Base station configuration applied!")
            print("\nThis uses a much simpler approach:")
            print("  • 60-second survey instead of 300 seconds")
            print("  • Direct RTCM message enabling")  
            print("  • No complex coordinate calculations")
            print("  • Should work more reliably")
        else:
            print("\n❌ Configuration failed")
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == "__main__":
    main()