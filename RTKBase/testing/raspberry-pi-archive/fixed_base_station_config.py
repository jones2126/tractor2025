#!/usr/bin/env python3
"""
Configure F9P as Fixed Base Station
===================================
This script configures the F9P as a fixed base station using current position,
bypassing survey-in issues.
"""

import serial
import struct
import time
import sys
import math

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
    elif value_size == 8:
        payload.extend(struct.pack('<Q', value))
    
    return ubx.create_message(0x06, 0x8A, payload)

def get_current_position(ser):
    """Get current position from NMEA GGA messages"""
    print("Getting current position from NMEA...")
    
    buffer = b''
    start_time = time.time()
    positions = []
    
    while time.time() - start_time < 10 and len(positions) < 5:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            buffer += data
            
            while b'\n' in buffer:
                line_end = buffer.find(b'\n')
                line = buffer[:line_end].decode('ascii', errors='ignore').strip()
                buffer = buffer[line_end + 1:]
                
                if line.startswith('$') and 'GGA' in line:
                    parts = line.split(',')
                    if len(parts) >= 10 and parts[2] and parts[4] and parts[9]:
                        try:
                            # Parse latitude
                            lat_str = parts[2]
                            lat_dir = parts[3]
                            lat_deg = float(lat_str[:2])
                            lat_min = float(lat_str[2:])
                            lat = lat_deg + lat_min / 60.0
                            if lat_dir == 'S':
                                lat = -lat
                                
                            # Parse longitude  
                            lon_str = parts[4]
                            lon_dir = parts[5]
                            lon_deg = float(lon_str[:3])
                            lon_min = float(lon_str[3:])
                            lon = lon_deg + lon_min / 60.0
                            if lon_dir == 'W':
                                lon = -lon
                                
                            # Parse altitude
                            alt = float(parts[9])
                            
                            positions.append((lat, lon, alt))
                            print(f"Position sample {len(positions)}: {lat:.8f}, {lon:.8f}, {alt:.3f}m")
                            
                        except ValueError:
                            continue
        time.sleep(0.1)
    
    if positions:
        # Average the positions
        avg_lat = sum(p[0] for p in positions) / len(positions)
        avg_lon = sum(p[1] for p in positions) / len(positions)
        avg_alt = sum(p[2] for p in positions) / len(positions)
        return avg_lat, avg_lon, avg_alt
    else:
        return None, None, None

def lat_lon_alt_to_ecef(lat, lon, alt):
    """Convert WGS84 lat/lon/alt to ECEF coordinates"""
    # WGS84 constants
    a = 6378137.0  # Semi-major axis
    f = 1/298.257223563  # Flattening
    e2 = 2*f - f*f  # First eccentricity squared
    
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    
    N = a / math.sqrt(1 - e2 * sin_lat * sin_lat)
    
    X = (N + alt) * cos_lat * cos_lon
    Y = (N + alt) * cos_lat * sin_lon
    Z = (N * (1 - e2) + alt) * sin_lat
    
    # Convert to cm (F9P uses cm)
    return int(X * 100), int(Y * 100), int(Z * 100)

def configure_fixed_base_station(ser):
    """Configure F9P as fixed base station using current position"""
    
    print("Configuring F9P as Fixed Base Station...")
    
    # Get current position
    lat, lon, alt = get_current_position(ser)
    
    if lat is None:
        print("ERROR: Could not get current position from NMEA")
        return False
    
    print(f"\nUsing position: {lat:.8f}, {lon:.8f}, {alt:.3f}m")
    
    # Convert to F9P format (degrees * 1e7, altitude in cm)
    lat_f9p = int(lat * 1e7)  # degrees * 10^7
    lon_f9p = int(lon * 1e7)  # degrees * 10^7  
    alt_f9p = int(alt * 100)  # meters to cm
    
    print(f"F9P format: Lat={lat_f9p}, Lon={lon_f9p}, Alt={alt_f9p}cm")
    
    configs = [
        # Disable survey-in mode first
        (0x40030001, 0, 1),          # CFG-TMODE-MODE = Disabled
        
        # Set fixed mode with lat/lon coordinates
        (0x40030001, 2, 1),          # CFG-TMODE-MODE = Fixed mode  
        (0x40030002, 1, 1),          # CFG-TMODE-POS_TYPE = 1 (LLH - Lat/Lon/Height)
        (0x4003000B, lat_f9p, 4),    # CFG-TMODE-LAT (degrees * 1e7)
        (0x4003000C, lon_f9p, 4),    # CFG-TMODE-LON (degrees * 1e7)  
        (0x4003000D, alt_f9p, 4),    # CFG-TMODE-HEIGHT (cm)
        (0x4003000A, 10000, 4),      # CFG-TMODE-FIXED_POS_ACC = 1m (10000 mm)
        
        # High-precision NMEA
        (0x10930006, 1, 1),          # CFG-NMEA-HIGHPREC = 1
        (0x10930001, 0, 1),          # CFG-NMEA-COMPAT = 0
        (0x20930007, 0x41, 1),       # CFG-NMEA-MAINVER = 4.1
        
        # NMEA message rates
        (0x209100bb, 1000, 2),       # GGA = 1Hz
        (0x209100ac, 1000, 2),       # RMC = 1Hz  
        (0x209100b1, 1000, 2),       # VTG = 1Hz
        (0x209100b0, 1000, 2),       # GLL = 1Hz
        
        # RTCM message rates
        (0x209102fe, 1000, 2),       # RTCM 1005 = 1Hz (base position)
        (0x209102fd, 1000, 2),       # RTCM 1074 = 1Hz (GPS MSM4)
        (0x20910300, 1000, 2),       # RTCM 1084 = 1Hz (GLONASS MSM4)  
        (0x20910302, 1000, 2),       # RTCM 1124 = 1Hz (BeiDou MSM4)
        (0x20910303, 10000, 2),      # RTCM 1230 = 0.1Hz (GLONASS bias)
    ]
    
    success_count = 0
    
    for key_id, value, size in configs:
        message = create_cfg_valset_message(key_id, value, size)
        
        try:
            ser.write(message)
            ser.flush()
            time.sleep(0.2)
            
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                if b'\xb5\x62\x05\x01' in response:
                    success_count += 1
                    print(f"✓ Config 0x{key_id:08X} acknowledged")
                else:
                    print(f"→ Config 0x{key_id:08X} sent")
            else:
                print(f"→ Config 0x{key_id:08X} sent")
                
        except Exception as e:
            print(f"✗ Error sending config 0x{key_id:08X}: {e}")
    
    print(f"\n✓ Fixed base station configuration completed ({success_count} confirmations)")
    print("📡 RTCM messages should start immediately")
    print("🎯 No survey-in delay - base station is active now")
    
    return success_count > 0

def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else '/dev/f9p'
    baudrate = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200
    
    print("F9P Fixed Base Station Configuration")
    print("=" * 45)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(1)
        print("✓ Serial connection established")
        
        success = configure_fixed_base_station(ser)
        
        if success:
            print("\n🎉 Fixed base station configured!")
            print("\nWhat to expect:")
            print("  • RTCM messages start immediately (no wait)")  
            print("  • High-precision NMEA continues")
            print("  • Mixed NMEA + RTCM output")
            print("  • Check rtcm_server_0714.py now")
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