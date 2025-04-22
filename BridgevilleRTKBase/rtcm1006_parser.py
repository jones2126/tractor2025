#!/usr/bin/env python3
"""
RTCM1006B Parser for K706 GPS
This script connects to a K706 GPS unit and parses RTCM1006B messages.
RTCM1006B contains base station coordinates (ECEF X,Y,Z) and antenna height.

Usage:
    - Connect K706 GPS to Raspberry Pi via USB
    - Ensure proper permissions for the serial port
    - Run the script
"""

import serial
import time
import struct
import binascii
import math
import argparse

# CRC24Q calculation for RTCM3 messages
def crc24q(data):
    """
    Calculate CRC24Q checksum used in RTCM3 messages
    
    Args:
        data: bytes or bytearray to calculate CRC for
        
    Returns:
        int: 24-bit CRC value
    """
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ CRC24Q_TABLE[(crc >> 16) ^ byte]
    return crc

# CRC24Q table (precomputed for performance)
# This is the standard CRC-24Q polynomial used in RTCM3
CRC24Q_TABLE = [
    0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
    0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
    0x420E70, 0xC4428B, 0xC8DB7D, 0x4E9786, 0xD1E891, 0x57A46A, 0x5B3D9C, 0xDD7167,
    0xE37949, 0x6535B2, 0x69AC44, 0xEFE0BF, 0x709FA8, 0xF6D353, 0xFA4AA5, 0x7C065E,
    0x0428A0, 0x82645B, 0x8EFDAD, 0x08B156, 0x97CE41, 0x1182BA, 0x1D1B4C, 0x9B57B7,
    0xA5A999, 0x23E562, 0x2F7C94, 0xA9306F, 0x364F78, 0xB00383, 0xBC9A75, 0x3AD68E,
    0x462610, 0xC06AEB, 0xCCF31D, 0x4ABFE6, 0xD5C0F1, 0x538C0A, 0x5F15FC, 0xD95907,
    0xE7A729, 0x61EBD2, 0x6D7224, 0xEB3EDF, 0x7441C8, 0xF20D33, 0xFE94C5, 0x78D83E,
    0x0840E0, 0x8E0C1B, 0x8295ED, 0x04D916, 0x9BA601, 0x1DEAFA, 0x11730C, 0x973FF7,
    0xA9C1D9, 0x2F8D22, 0x2314D4, 0xA5582F, 0x3A2738, 0xBC6BC3, 0xB0F235, 0x36BECE,
    0x4A6E50, 0xCC22AB, 0xC0BB5D, 0x46F7A6, 0xD988B1, 0x5FC44A, 0x535DBC, 0xD51147,
    0xEBEF69, 0x6DA392, 0x613A64, 0xE7769F, 0x78098A, 0xFE4571, 0xF2DC87, 0x74907C,
    0x0CC920, 0x8A85DB, 0x861C2D, 0x0050D6, 0x9F2FC1, 0x19633A, 0x15FACC, 0x93B637,
    0xAD4819, 0x2B04E2, 0x279D14, 0xA1D1EF, 0x3EAEF8, 0xB8E203, 0xB47BF5, 0x32370E,
    0x4EC790, 0xC88B6B, 0xC4129D, 0x425E66, 0xDD2171, 0x5B6D8A, 0x57F47C, 0xD1B887,
    0xEF46A9, 0x690A52, 0x6593A4, 0xE3DF5F, 0x7CA048, 0xFAECB3, 0xF67545, 0x7039BE,
    0x10C9E0, 0x96851B, 0x9A1CED, 0x1C5016, 0x832F01, 0x0563FA, 0x09FA0C, 0x8FB6F7,
    0xB148D9, 0x370422, 0x3B9DD4, 0xBDD12F, 0x22AE38, 0xA4E2C3, 0xA87B35, 0x2E37CE,
    0x52E750, 0xD4ABAB, 0xD8325D, 0x5E7EA6, 0xC101B1, 0x474D4A, 0x4BD4BC, 0xCD9847,
    0xF3C669, 0x758A92, 0x79E364, 0xFFAF9F, 0x60208A, 0xE66C71, 0xEAF587, 0x6CB97C,
    0x148140, 0x92CDBB, 0x9E544D, 0x1818B6, 0x8767A1, 0x012B5A, 0x0DB2AC, 0x8BFE57,
    0xB50079, 0x334C82, 0x3FD574, 0xB9998F, 0x26E698, 0xA0AA63, 0xAC3395, 0x2A7F6E,
    0x56AFD0, 0xD0E32B, 0xDC7ADD, 0x5A3626, 0xC54931, 0x4305CA, 0x4F9C3C, 0xC9D0C7,
    0xF72EE9, 0x716212, 0x7DFBE4, 0xFBB71F, 0x64C808, 0xE284F3, 0xEE1D05, 0x6851FE,
    0x1400A0, 0x924C5B, 0x9ED5AD, 0x18D956, 0x87E641, 0x01AABA, 0x0D334C, 0x8B7FB7,
    0xB58199, 0x33CD62, 0x3F5494, 0xB9186F, 0x26677E, 0xA02B85, 0xACB273, 0x2AFE88,
    0x562E30, 0xD062CB, 0xDCFB3D, 0x5AB7C6, 0xC5C8D1, 0x43842A, 0x4F1DDC, 0xC95127,
    0xF7AF09, 0x71E3F2, 0x7D7A04, 0xFB36FF, 0x6449E8, 0xE20513, 0xEE9CE5, 0x68D01E,
    0x1808C0, 0x9E443B, 0x92DDCD, 0x14A136, 0x8BEE21, 0x0DA2DA, 0x013B2C, 0x8777D7,
    0xB989F9, 0x3FC502, 0x335CF4, 0xB5100F, 0x2A6F18, 0xAC23E3, 0xA0BA15, 0x26F6EE,
    0x5A2650, 0xDC6AAB, 0xD0F35D, 0x56BFA6, 0xC9C0B1, 0x4F8C4A, 0x4315BC, 0xC55947,
    0xFBA769, 0x7DEB92, 0x717264, 0xF73E9F, 0x68418A, 0xEE0D71, 0xE29487, 0x64D87C,
    0x200580, 0xA6497B, 0xAAD08D, 0x2C9C76, 0xB3E361, 0x35AF9A, 0x39366C, 0xBF7A97,
    0x8184B9, 0x07C842, 0x0B51B4, 0x8D1D4F, 0x12625A, 0x942EA1, 0x98B757, 0x1EFBAC,
    0x620B10, 0xE447EB, 0xE8DE1D, 0x6E92E6, 0xF1EDF1, 0x77A10A, 0x7B38FC, 0xFD7407,
    0xC38A29, 0x45C6D2, 0x495F24, 0xCF13DF, 0x506CC8, 0xD62033, 0xDAB9C5, 0x5CF53E,
    0x243D20, 0xA271DB, 0xAEE82D, 0x28A4D6, 0xB7DBC1, 0x31973A, 0x3D0ECC, 0xBB4237,
    0x85BC19, 0x03F0E2, 0x0F6914, 0x8925EF, 0x165AF8, 0x901603, 0x9C8FF5, 0x1AC30E,
    0x663390, 0xE07F6B, 0xECE69D, 0x6AAA66, 0xF5D571, 0x73998A, 0x7F007C, 0xF94C87,
    0xC7B2A9, 0x41FE52, 0x4D67A4, 0xCB2B5F, 0x54544E, 0xD218B5, 0xDE8143, 0x58CDB8,
    0x284C40, 0xAE00BB, 0xA2994D, 0x24D5B6, 0xBBAAA1, 0x3DE65A, 0x317FAC, 0xB73357,
    0x89CD79, 0x0F8182, 0x031874, 0x85548F, 0x1A2B9A, 0x9C6761, 0x90FE97, 0x16B26C,
    0x6A62D0, 0xEC2E2B, 0xE0B7DD, 0x66FB26, 0xF98431, 0x7FC8CA, 0x73513C, 0xF51DC7,
    0xCBE3E9, 0x4DAF12, 0x4136E4, 0xC77A1F, 0x58050A, 0xDE49F1, 0xD2D007, 0x549CFC
]

def parse_rtcm3_header(data):
    """Parse the RTCM3 message header"""
    if len(data) < 3:
        return None
    
    # First 3 bytes of RTCM3 message
    # Bits 0-7: Preamble (always 0xD3)
    # Bits 8-13: Reserved (6 bits)
    # Bits 14-23: Message length (10 bits)
    if data[0] != 0xD3:
        return None
    
    msg_length = ((data[1] & 0x03) << 8) | data[2]
    return {
        'length': msg_length,
        'total_length': msg_length + 3 + 3  # header (3) + CRC (3)
    }

def parse_rtcm1006(message):
    """
    Parse RTCM1006 message (Station Coordinates with Antenna Height)
    
    Returns:
        dict: with keys for station_id, ECEF coordinates (x,y,z) and antenna height
    """
    if len(message) < 20:  # Minimum expected length
        return None
    
    # First 2 bytes after header contain message number (12 bits) and station ID (12 bits)
    msg_number = (message[0] << 4) | ((message[1] & 0xF0) >> 4)
    station_id = ((message[1] & 0x0F) << 8) | message[2]
    
    if msg_number != 1006:
        return None  # Not a 1006 message
    
    # Extract ITRF realization year, GPS and GLONASS indicators, etc.
    # Bits 24-30 contain various indicators
    indicators = message[3]
    itrf_year = indicators >> 6  # 0 means ITRF2008 or earlier
    gps_ind = (indicators >> 5) & 0x01  # 1 means GPS is supported
    glo_ind = (indicators >> 4) & 0x01  # 1 means GLONASS is supported
    galileo_ind = (indicators >> 3) & 0x01  # 1 means Galileo is supported
    ref_station_ind = (indicators >> 2) & 0x01  # 1 means reference station
    single_receiver_osc_ind = (indicators >> 1) & 0x01  # 1 means single receiver oscilator
    
    # ECEF coordinates are 38-bit signed integers with 0.1mm resolution
    # They start at byte 4 and use 5 bytes each (with the MSB partial)
    x_scaled = ((message[4] & 0x3F) << 32) | (message[5] << 24) | \
               (message[6] << 16) | (message[7] << 8) | message[8]
    # Convert from 38-bit signed value
    if x_scaled & 0x2000000000:  # Check sign bit
        x_scaled = x_scaled - 0x4000000000
    x_meters = x_scaled * 0.0001  # Scale from 0.1mm to meters
    
    y_scaled = ((message[9] & 0x3F) << 32) | (message[10] << 24) | \
               (message[11] << 16) | (message[12] << 8) | message[13]
    if y_scaled & 0x2000000000:  # Check sign bit
        y_scaled = y_scaled - 0x4000000000
    y_meters = y_scaled * 0.0001  # Scale from 0.1mm to meters
    
    z_scaled = ((message[14] & 0x3F) << 32) | (message[15] << 24) | \
               (message[16] << 16) | (message[17] << 8) | message[18]
    if z_scaled & 0x2000000000:  # Check sign bit
        z_scaled = z_scaled - 0x4000000000
    z_meters = z_scaled * 0.0001  # Scale from 0.1mm to meters
    
    # Antenna height (in 1006, not in 1005) is 16-bit unsigned with 0.1mm resolution
    antenna_height = ((message[19] << 8) | message[20]) * 0.0001  # meters
    
    return {
        'message_type': 1006,
        'station_id': station_id,
        'itrf_year': itrf_year,
        'gps_supported': bool(gps_ind),
        'glonass_supported': bool(glo_ind),
        'galileo_supported': bool(galileo_ind),
        'is_reference_station': bool(ref_station_ind),
        'single_receiver_oscillator': bool(single_receiver_osc_ind),
        'x_ecef': x_meters,
        'y_ecef': y_meters,
        'z_ecef': z_meters,
        'antenna_height': antenna_height
    }

def convert_ecef_to_lla(x, y, z):
    """
    Convert ECEF coordinates to Latitude, Longitude, Altitude
    
    Args:
        x, y, z: ECEF coordinates in meters
        
    Returns:
        tuple: (latitude in degrees, longitude in degrees, altitude in meters)
    """
    # WGS84 ellipsoid parameters
    a = 6378137.0  # semi-major axis in meters
    f = 1/298.257223563  # flattening
    b = a * (1 - f)  # semi-minor axis
    e_sq = 1 - (b**2) / (a**2)  # eccentricity squared
    
    # Calculate longitude
    longitude = math.atan2(y, x)
    
    # Initial value for latitude
    p = math.sqrt(x**2 + y**2)
    latitude = math.atan2(z, p * (1 - e_sq))
    
    # Iterative solution for latitude (iteration improves accuracy)
    for _ in range(5):  # Usually converges in 2-3 iterations
        N = a / math.sqrt(1 - e_sq * math.sin(latitude)**2)
        h = p / math.cos(latitude) - N
        latitude = math.atan2(z, p * (1 - e_sq * N / (N + h)))
    
    # Calculate height
    N = a / math.sqrt(1 - e_sq * math.sin(latitude)**2)
    h = p / math.cos(latitude) - N
    
    # Convert to degrees
    latitude_deg = math.degrees(latitude)
    longitude_deg = math.degrees(longitude)
    
    return (latitude_deg, longitude_deg, h)

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371000  # Radius of earth in meters
    return c * r  # Return distance in meters

def main(port='/dev/ttyUSB_com2', baud=115200, known_lat=40.34536010088, known_lon=-80.12878619119, known_alt=326.5974):
    """
    Main function to read and parse RTCM1006B messages
    
    Args:
        port: Serial port for K706 GPS
        baud: Baud rate for serial port
        known_lat: Known latitude of base station
        known_lon: Known longitude of base station
        known_alt: Known altitude of base station
    """
    # Serial port configuration
    port_com2 = port
    baud_rate = baud
    
    # Store known base station coordinates
    known_lat = known_lat
    known_lon = known_lon
    known_alt = known_alt
    
    try:
        print(f"Opening serial port {port_com2} at {baud_rate} baud...")
        ser = serial.Serial(port_com2, baud_rate, timeout=1)
        
        # Buffer for incoming data
        buffer = bytearray()
        
        print("Waiting for RTCM1006B messages...")
        print(f"Will compare with known position: Lat: {known_lat}°, Lon: {known_lon}°, Alt: {known_alt}m")
        print("Press Ctrl+C to exit")
        
        # Debugging flag - set to True to see raw buffer data
        # This will be set by command line args
        
        while True:
            try:
                # Read available data
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue
                
                # Add to buffer
                buffer.extend(data)
                
                if debug_mode and len(buffer) > 0:
                    print(f"Buffer size: {len(buffer)} bytes")
                    print(f"Buffer starts with: {' '.join([f'{b:02X}' for b in buffer[:min(10, len(buffer))]])}")
                
                # Look for RTCM3 preamble (0xD3)
                while len(buffer) > 6 and buffer[0] != 0xD3:
                    buffer.pop(0)
            
            # If buffer has at least 3 bytes, we can check the header
            if len(buffer) >= 3:
                try:
                    header = parse_rtcm3_header(buffer)
                    
                    if header and len(buffer) >= header['total_length']:
                        # We have a complete message
                        msg_data = buffer[3:3+header['length']]
                        
                        # Extract received CRC (last 3 bytes of message)
                        crc_received = (buffer[3+header['length']] << 16) | \
                                      (buffer[3+header['length']+1] << 8) | \
                                      buffer[3+header['length']+2]
                        
                        # Calculate CRC
                        crc_calc = crc24q(buffer[:3+header['length']])
                        
                        if debug_mode:
                            print(f"Message length: {header['length']} bytes")
                            print(f"CRC received: 0x{crc_received:06X}")
                            print(f"CRC calculated: 0x{crc_calc:06X}")
                        
                        if crc_calc == crc_received:
                            # Extract message type (first 12 bits after header)
                            msg_type = (msg_data[0] << 4) | ((msg_data[1] & 0xF0) >> 4)
                            
                            if debug_mode:
                                print(f"Message type: {msg_type}")
                                
                except Exception as e:
                    if debug_mode:
                        print(f"Error parsing message: {e}")
                    # Clear the first byte and continue
                    buffer.pop(0)
                    continue
                        
                        if msg_type == 1006:
                            # Parse 1006 message
                            result = parse_rtcm1006(msg_data)
                            if result:
                                # Convert ECEF to lat/lon/alt
                                lat, lon, alt = convert_ecef_to_lla(
                                    result['x_ecef'],
                                    result['y_ecef'],
                                    result['z_ecef']
                                )
                                
                                # Calculate differences from known position
                                lat_diff = abs(lat - known_lat)
                                lon_diff = abs(lon - known_lon)
                                alt_diff = abs(alt - known_alt)
                                
                                # Calculate horizontal distance using Haversine formula
                                horizontal_dist = haversine_distance(lat, lon, known_lat, known_lon)
                                
                                # Calculate 3D distance
                                distance_3d = math.sqrt(horizontal_dist**2 + alt_diff**2)
                                
                                print("\nRTCM1006B Message Received:")
                                print(f"Station ID: {result['station_id']}")
                                print(f"ECEF Coordinates: X={result['x_ecef']:.4f}, "
                                      f"Y={result['y_ecef']:.4f}, Z={result['z_ecef']:.4f} meters")
                                print(f"Antenna Height: {result['antenna_height']:.4f} meters")
                                print(f"Calculated Position: Lat: {lat:.8f}°, Lon: {lon:.8f}°, Alt: {alt:.4f}m")
                                print(f"Known Position:     Lat: {known_lat:.8f}°, Lon: {known_lon:.8f}°, Alt: {known_alt:.4f}m")
                                print("\nPosition Differences:")
                                print(f"Latitude Diff:  {lat_diff:.8f}° ({lat_diff * 111319:.2f} meters)")
                                print(f"Longitude Diff: {lon_diff:.8f}° ({lon_diff * 111319 * math.cos(math.radians(lat)):.2f} meters)")
                                print(f"Altitude Diff:  {alt_diff:.4f} meters")
                                print(f"Horizontal Distance: {horizontal_dist:.2f} meters")
                                print(f"3D Distance:        {distance_3d:.2f} meters")
                                print(f"Supports GPS: {result['gps_supported']}, "
                                      f"GLONASS: {result['glonass_supported']}, "
                                      f"Galileo: {result['galileo_supported']}")
                                print("-" * 70)
                    
                    # Remove processed message from buffer
                    buffer = buffer[header['total_length']:]
            
            # Sleep to prevent CPU usage from spiking
            time.sleep(0.01)
            
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except KeyboardInterrupt:
        print("\nScript terminated by user")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Parse RTCM1006B messages from K706 GPS')
    parser.add_argument('--port', default='/dev/ttyUSB_com2', help='Serial port for K706 GPS')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial port')
    parser.add_argument('--debug', action='store_true', help='Enable debug output')
    parser.add_argument('--lat', type=float, default=40.34536010088, help='Known latitude')
    parser.add_argument('--lon', type=float, default=-80.12878619119, help='Known longitude')
    parser.add_argument('--alt', type=float, default=326.5974, help='Known altitude')
    
    args = parser.parse_args()
    
    # Set global debug flag
    global debug_mode
    debug_mode = args.debug
    
    try:
        main(port=args.port, baud=args.baud, known_lat=args.lat, known_lon=args.lon, known_alt=args.alt)
    except KeyboardInterrupt:
        print("\nScript terminated by user")
    except Exception as e:
        print(f"Error: {e}")
        if debug_mode:
            import traceback
            traceback.print_exc()