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

# CRC24Q calculation for RTCM3 messages
def crc24q(data):
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ CRC24Q_TABLE[(crc >> 16) ^ byte]
    return crc

# CRC24Q table (precomputed for performance)
# This is the standard CRC-24Q polynomial used in RTCM3
CRC24Q_TABLE = [
    0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
    0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
    # ... truncated for brevity - you would need the full table in production
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

def main():
    # Serial port configuration for K706 on RPi 3
    port_com2 = "/dev/ttyUSB_com2"
    baud_rate = 115200  # Default baud rate for K706
    
    # Known base station coordinates
    known_lat = 40.34536010088
    known_lon = -80.12878619119
    known_alt = 326.5974
    
    try:
        print(f"Opening serial port {port_com2} at {baud_rate} baud...")
        ser = serial.Serial(port_com2, baud_rate, timeout=1)
        
        # Buffer for incoming data
        buffer = bytearray()
        
        print("Waiting for RTCM1006B messages...")
        print(f"Will compare with known position: Lat: {known_lat}°, Lon: {known_lon}°, Alt: {known_alt}m")
        
        while True:
            # Read available data
            data = ser.read(ser.in_waiting or 1)
            if not data:
                continue
            
            # Add to buffer
            buffer.extend(data)
            
            # Look for RTCM3 preamble (0xD3)
            while len(buffer) > 6 and buffer[0] != 0xD3:
                buffer.pop(0)
            
            # If buffer has at least 3 bytes, we can check the header
            if len(buffer) >= 3:
                header = parse_rtcm3_header(buffer)
                
                if header and len(buffer) >= header['total_length']:
                    # We have a complete message
                    msg_data = buffer[3:3+header['length']]
                    crc_received = (buffer[3+header['length']] << 16) | \
                                  (buffer[3+header['length']+1] << 8) | \
                                  buffer[3+header['length']+2]
                    
                    # Calculate CRC
                    crc_calc = crc24q(buffer[:3+header['length']])
                    
                    if crc_calc == crc_received:
                        # Extract message type (first 12 bits after header)
                        msg_type = (msg_data[0] << 4) | ((msg_data[1] & 0xF0) >> 4)
                        
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
    main()
