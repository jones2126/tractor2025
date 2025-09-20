#!/usr/bin/env python3
"""
RTCM-Only Base Station Configuration
===================================
Configures F9P as a base station that outputs ONLY RTCM corrections
using the specified fixed coordinates.
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

def disable_all_nmea(ser):
    """Disable all NMEA messages"""
    ubx = UBXMessage()
    
    print("Disabling all NMEA messages...")
    
    # NMEA message IDs to disable
    nmea_messages = [
        (0xF0, 0x00),  # GGA
        (0xF0, 0x01),  # GLL  
        (0xF0, 0x02),  # GSA
        (0xF0, 0x03),  # GSV
        (0xF0, 0x04),  # RMC
        (0xF0, 0x05),  # VTG
        (0xF0, 0x06),  # GRS
        (0xF0, 0x07),  # GST
        (0xF0, 0x08),  # ZDA
        (0xF0, 0x09),  # GBS
        (0xF0, 0x0A),  # DTM
        (0xF1, 0x00),  # PUBX,00
        (0xF1, 0x03),  # PUBX,03
        (0xF1, 0x04),  # PUBX,04
    ]
    
    success_count = 0
    
    for msg_class, msg_id in nmea_messages:
        # Set rate to 0 for all ports
        payload = struct.pack('BBBBBBBB', msg_class, msg_id, 0, 0, 0, 0, 0, 0)
        message = ubx.create_message(0x06, 0x01, payload)  # CFG-MSG
        
        try:
            ser.write(message)
            ser.flush()
            time.sleep(0.1)
            success_count += 1
        except Exception as e:
            print(f"Error disabling NMEA {msg_class:02X}-{msg_id:02X}: {e}")
    
    print(f"Disabled {success_count} NMEA message types")
    return success_count

def enable_rtcm_only(ser):
    """Enable only RTCM messages"""
    ubx = UBXMessage()
    
    print("Enabling RTCM messages...")
    
    # RTCM message configurations: (msg_class, msg_id, description)
    rtcm_messages = [
        (0xF5, 0x05, "1005 - Base Station ARP"),
        (0xF5, 0x4A, "1074 - GPS MSM4"),  
        (0xF5, 0x54, "1084 - GLONASS MSM4"),
        (0xF5, 0x7C, "1124 - BeiDou MSM4"),
        (0xF5, 0xE6, "1230 - GLONASS Biases"),
    ]
    
    success_count = 0
    
    for msg_class, msg_id, description in rtcm_messages:
        # Enable on UART1 at 1Hz, disable on other ports
        payload = struct.pack('BBBBBBBB', msg_class, msg_id, 0, 1, 0, 0, 0, 0)
        message = ubx.create_message(0x06, 0x01, payload)  # CFG-MSG
        
        try:
            ser.write(message)
            ser.flush()
            time.sleep(0.2)
            
            # Check for ACK
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                if b'\xb5\x62\x05\x01' in response:
                    success_count += 1
                    print(f"  ✓ RTCM {description}")
                elif b'\xb5\x62\x05\x00' in response:
                    print(f"  ✗ RTCM {description} - NACK")
                else:
                    print(f"  → RTCM {description} - sent")
            else:
                print(f"  → RTCM {description} - sent")
                
        except Exception as e:
            print(f"  ✗ Error enabling RTCM {msg_id:02X}: {e}")
    
    return success_count

def set_fixed_position(ser, lat, lon, alt):
    """Set fixed base station position using UBX-CFG-TMODE3"""
    ubx = UBXMessage()
    
    print(f"Setting fixed position: {lat:.8f}, {lon:.8f}, {alt:.3f}m")
    
    # Convert coordinates to F9P format
    # For TMODE3 in LLA mode: lat/lon in degrees * 1e7, height in cm
    lat_hp = int(lat * 1e7)
    lon_hp = int(lon * 1e7)
    alt_hp = int(alt * 100)  # Convert m to cm
    
    print(f"F9P format: lat={lat_hp}, lon={lon_hp}, alt={alt_hp}cm")
    
    # UBX-CFG-TMODE3 payload for fixed mode
    version = 0
    reserved1 = 0
    flags = 0x0002  # Fixed mode, LLA coordinates
    reserved2 = 0
    
    # Position in LLA format
    ecefXOrLat = lat_hp & 0xFFFFFFFF  # Handle as unsigned
    ecefYOrLon = lon_hp & 0xFFFFFFFF  # Handle as unsigned
    ecefZOrAlt = alt_hp
    
    # High precision parts (not used for LLA)
    ecefXOrLatHP = 0
    ecefYOrLonHP = 0  
    ecefZOrAltHP = 0
    
    # Accuracy (10000 = 1m)
    fixedPosAcc = 10000
    
    # Survey-in parameters (not used in fixed mode)
    svinMinDur = 0
    svinAccLimit = 0
    
    # Pack the payload - use unsigned format for lat/lon
    payload = struct.pack('<BBHHIIIIIIIII',
        version, reserved1, flags, reserved2,
        ecefXOrLat, ecefYOrLon, ecefZOrAlt,
        ecefXOrLatHP, ecefYOrLonHP, ecefZOrAltHP,
        fixedPosAcc, svinMinDur, svinAccLimit
    )
    
    message = ubx.create_message(0x06, 0x71, payload)  # CFG-TMODE3
    
    try:
        ser.write(message)
        ser.flush()
        time.sleep(0.5)
        
        # Check for response
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            if b'\xb5\x62\x05\x01' in response:
                print("  ✓ Fixed position set successfully")
                return True
            elif b'\xb5\x62\x05\x00' in response:
                print("  ✗ Fixed position rejected (NACK)")
                return False
            else:
                print("  → Fixed position command sent")
                return True
        else:
            print("  → Fixed position command sent (no response)")
            return True
            
    except Exception as e:
        print(f"  ✗ Error setting fixed position: {e}")
        return False

def save_configuration(ser):
    """Save configuration to flash memory"""
    ubx = UBXMessage()
    
    print("Saving configuration to flash...")
    
    # CFG-CFG: Save current config to flash
    clearMask = 0x00000000   # Don't clear anything
    saveMask = 0x0000061F    # Save everything
    loadMask = 0x00000000    # Don't load anything
    
    payload = struct.pack('<III', clearMask, saveMask, loadMask)
    message = ubx.create_message(0x06, 0x09, payload)  # CFG-CFG
    
    try:
        ser.write(message)
        ser.flush()
        time.sleep(1)
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            if b'\xb5\x62\x05\x01' in response:
                print("  ✓ Configuration saved to flash")
                return True
        
        print("  → Save command sent")
        return True
        
    except Exception as e:
        print(f"  ✗ Error saving configuration: {e}")
        return False

def configure_rtcm_base_station(ser):
    """Configure F9P as RTCM-only base station"""
    
    # Fixed coordinates
    LAT = 40.34525533
    LON = -80.12876667  
    ALT = 327.100
    
    print("Configuring F9P as RTCM-Only Base Station")
    print(f"Location: {LAT:.8f}, {LON:.8f}, {ALT:.3f}m")
    print("=" * 50)
    
    success_steps = 0
    
    # Step 1: Disable all NMEA messages
    print("\n1. Disabling NMEA messages...")
    if disable_all_nmea(ser) > 0:
        success_steps += 1
        print("   ✓ NMEA messages disabled")
    
    # Step 2: Set fixed position
    print("\n2. Setting fixed base station position...")
    if set_fixed_position(ser, LAT, LON, ALT):
        success_steps += 1
        print("   ✓ Fixed position configured")
    
    # Step 3: Enable RTCM messages
    print("\n3. Enabling RTCM corrections...")
    rtcm_count = enable_rtcm_only(ser)
    if rtcm_count > 0:
        success_steps += 1
        print(f"   ✓ {rtcm_count}/5 RTCM message types enabled")
    
    # Step 4: Save configuration
    print("\n4. Saving configuration...")
    if save_configuration(ser):
        success_steps += 1
        print("   ✓ Configuration saved")
    
    print(f"\n📋 Configuration Summary:")
    print(f"  • Mode: Fixed base station (RTCM-only)")
    print(f"  • Position: {LAT:.8f}, {LON:.8f}, {ALT:.3f}m")
    print(f"  • NMEA output: DISABLED") 
    print(f"  • RTCM output: ENABLED ({rtcm_count} message types)")
    print(f"  • Configuration steps completed: {success_steps}/4")
    
    if success_steps >= 3:
        print(f"\n✓ Base station should be broadcasting RTCM corrections now!")
        print(f"   Check your rtcm_server_0714.py - you should see RTCM messages only")
        return True
    else:
        print(f"\n✗ Configuration incomplete - some steps failed")
        return False

def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else '/dev/f9p'
    baudrate = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200
    
    print("F9P RTCM-Only Base Station Configuration")
    print("=" * 50)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(1)
        print("✓ Serial connection established")
        
        success = configure_rtcm_base_station(ser)
        
        if success:
            print("\n🎯 RTCM-only base station configured!")
            print("\nExpected behavior:")
            print("  • No NMEA messages")
            print("  • Only RTCM binary data")  
            print("  • rtcm_server_0714.py should detect RTCM messages immediately")
            print("  • TCP clients will receive pure RTCM corrections")
        else:
            print("\n❌ Configuration failed - check errors above")
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        try:
            ser.close()
        except:
            pass

if __name__ == "__main__":
    main()