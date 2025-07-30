#!/usr/bin/env python3
"""
RTCM1006B Parser for K706 GPS (Bypass CRC Check)
This script connects to a K706 GPS unit and parses RTCM1006B messages.
RTCM1006B contains base station coordinates (ECEF X,Y,Z) and antenna height.
This version assumes data is correct if it starts with 0xD3 preamble.

Usage:
    - Connect K706 GPS to Raspberry Pi via USB
    - Ensure proper permissions for the serial port
    - Run the script
"""

import serial
import time
import math
import argparse

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

def crc24q(data):
    """
    Calculate CRC24Q checksum used in RTCM3 messages
    
    Args:
        data: bytes or bytearray to calculate CRC for
        
    Returns:
        int: 24-bit CRC value
    """
    CRC24Q_TABLE = [
        0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
        0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
        0x420E70, 0xC4428B, 0xC8DB7D, 0x4E9786, 0xD1E891, 0x57A46A, 0x5B3D9C, 0xDD7167,
        0xE38F49, 0x65C3B2, 0x695A44, 0xEF16BF, 0x7069A8, 0xF62553, 0xFABCA5, 0x7CF05E,
        0x8421E0, 0x026D1B, 0x0EF4ED, 0x88B816, 0x17C701, 0x918BFA, 0x9D120C, 0x1B5EF7,
        0x25A0D9, 0xA3EC22, 0xAF75D4, 0x29392F, 0xB64638, 0x300AC3, 0x3C9335, 0xBADFCE,
        0xC62F90, 0x40636B, 0x4CFA9D, 0xCAB666, 0x55C971, 0xD3858A, 0xDF1C7C, 0x595087,
        0x67AEA9, 0xE1E252, 0xED7BA4, 0x6B375F, 0xF44848, 0x7204B3, 0x7E9D45, 0xF8D1BE,
        0x8843C0, 0x0E0F3B, 0x0296CD, 0x84DA36, 0x1BA521, 0x9DE9DA, 0x91702C, 0x173CD7,
        0x29C2F9, 0xAF8E02, 0xA317F4, 0x255B0F, 0xBA2418, 0x3C68E3, 0x30F115, 0xB6BDEE,
        0xCA4DB0, 0x4C014B, 0x4098BD, 0xC6D446, 0x59AB51, 0xDFE7AA, 0xD37E5C, 0x5532A7,
        0x6BCC89, 0xED8072, 0xE11984, 0x67557F, 0xF82A68, 0x7E6693, 0x72FF65, 0xF4B39E,
        0x8C6220, 0x0A2EDB, 0x06B72D, 0x80FBD6, 0x1F84C1, 0x99C83A, 0x9551CC, 0x131D37,
        0x2DE319, 0xABAFE2, 0xA73614, 0x217AEF, 0xBE05F8, 0x384903, 0x34D0F5, 0xB29C0E,
        0xCE6C50, 0x4820AB, 0x44B95D, 0xC2F5A6, 0x5D8AB1, 0xDBC64A, 0xD75FBC, 0x511347,
        0x6FED69, 0xE9A192, 0xE53864, 0x63749F, 0xFC0B88, 0x7A4773, 0x76DE85, 0xF0927E,
        0x908720, 0x16CBDB, 0x1A522D, 0x9C1ED6, 0x0361C1, 0x852D3A, 0x89B4CC, 0x0FF837,
        0x310619, 0xB74AE2, 0xBBD314, 0x3D9FEF, 0xA2E0F8, 0x24AC03, 0x2835F5, 0xAE790E,
        0xD28950, 0x54C5AB, 0x585C5D, 0xDE10A6, 0x416FB1, 0xC7234A, 0xCBBABC, 0x4DF647,
        0x730869, 0xF54492, 0xF9DD64, 0x7F919F, 0xE0EE88, 0x66A273, 0x6A3B85, 0xEC777E,
        0x94A6C0, 0x12EA3B, 0x1E73CD, 0x983F36, 0x074021, 0x810CDA, 0x8D952C, 0x0BD9D7,
        0x3527F9, 0xB36B02, 0xBFF2F4, 0x39BE0F, 0xA6C118, 0x208DE3, 0x2C1415, 0xAA58EE,
        0xD6A8B0, 0x50E44B, 0x5C7DBD, 0xDA3146, 0x454E51, 0xC302AA, 0xCF9B5C, 0x49D7A7,
        0x772989, 0xF16572, 0xFDFC84, 0x7BB07F, 0xE4CF68, 0x628393, 0x6E1A65, 0xE8569E,
        0x98C4E0, 0x1E881B, 0x1211ED, 0x945D16, 0x0B2201, 0x8D6EFA, 0x81F70C, 0x07BBF7,
        0x3945D9, 0xBF0922, 0xB390D4, 0x35DC2F, 0xAAA338, 0x2CEFC3, 0x207635, 0xA63ACE,
        0xDACA90, 0x5C866B, 0x501F9D, 0xD65366, 0x492C71, 0xCF608A, 0xC3F97C, 0x45B587,
        0x7B4BA9, 0xFD0752, 0xF19EA4, 0x77D25F, 0xE8AD48, 0x6EE1B3, 0x627845, 0xE434BE,
        0x9CE500, 0x1AA9FB, 0x16300D, 0x907CF6, 0x0F03E1, 0x894F1A, 0x85D6EC, 0x039A17,
        0x3D6439, 0xBB28C2, 0xB7B134, 0x31FDCF, 0xAE82D8, 0x28CE23, 0x2457D5, 0xA21B2E,
        0xDEEB70, 0x58A78B, 0x543E7D, 0xD27286, 0x4D0D91, 0xCB416A, 0xC7D89C, 0x419467,
        0x7F6A49, 0xF926B2, 0xF5BF44, 0x73F3BF, 0xEC8CA8, 0x6AC053, 0x6659A5, 0xE0155E,
    ]
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ CRC24Q_TABLE[(crc >> 16) ^ byte]
    return crc

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
    single_receiver_osc_ind = (indicators >> 1) & 0x01  # 1 means single receiver oscillator
    
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

def main(port='/dev/ttyUSB_com2', baud=115200, known_lat=40.34536010088, known_lon=-80.12878619119, known_alt=326.5974, debug=False):
    """
    Main function to read and parse RTCM1006B messages, assuming data is correct if it starts with 0xD3
    
    Args:
        port: Serial port for K706 GPS
        baud: Baud rate for serial port
        known_lat: Known latitude of base station
        known_lon: Known longitude of base station
        known_alt: Known altitude of base station
        debug: Enable debug output
    """
    try:
        print(f"Opening serial port {port} at {baud} baud...")
        ser = serial.Serial(port, baud, timeout=1)
        
        # Buffer for incoming data
        buffer = bytearray()
        
        print("Waiting for RTCM1006B messages (assuming data correct if starts with 0xD3)...")
        print(f"Will compare with known position: Lat: {known_lat}°, Lon: {known_lon}°, Alt: {known_alt}m")
        print("Press Ctrl+C to exit")
        
        while True:
            try:
                # Read available data
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue
                
                # Add to buffer
                buffer.extend(data)
                
                # Log raw data to a file for offline analysis
                with open("rtcm_log.bin", "ab") as f:
                    f.write(data)
                
                if debug:
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
                            
                            # Calculate CRC for debugging (not used for validation)
                            crc_calc = crc24q(buffer[:3+header['length']])
                            
                            if debug:
                                print(f"Message length: {header['length']} bytes")
                                print(f"CRC calculated: 0x{crc_calc:06X}")
                                print(f"CRC received: 0x{crc_received:06X}")
                            
                            # Extract message type (first 12 bits after header)
                            msg_type = (msg_data[0] << 4) | ((msg_data[1] & 0xF0) >> 4)
                            
                            # Print message type regardless of CRC result
                            if debug:
                                print(f"Message type: {msg_type}")
                            
                            # Parse RTCM 1006 messages regardless of CRC
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
                                    
                                    print("\nRTCM1006B Message Received (CRC Ignored):")
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
                            else:
                                if debug:
                                    print(f"Skipping non-1006 message type: {msg_type}")
                            
                            # Log CRC status for debugging
                            if crc_calc == crc_received:
                                print("CRC Valid")
                            else:
                                print(f"CRC mismatch: calculated 0x{crc_calc:06X}, received 0x{crc_received:06X}")
                            
                            # Remove processed message from buffer
                            buffer = buffer[header['total_length']:]
                        elif debug and header:
                            print(f"Incomplete message: have {len(buffer)} bytes, need {header['total_length']} bytes")
                    
                    except Exception as e:
                        if debug:
                            print(f"Error parsing message: {e}")
                        # Clear the first byte and continue
                        if len(buffer) > 0:
                            buffer.pop(0)
            
            except Exception as e:
                if debug:
                    print(f"Error in main loop: {e}")
                # Continue on error
                time.sleep(0.1)
            
            # Sleep to prevent CPU usage from spiking
            time.sleep(0.01)
            
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Parse RTCM1006B messages from K706 GPS (Bypass CRC)')
    parser.add_argument('--port', default='/dev/ttyUSB_com2', help='Serial port for K706 GPS')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial port')
    parser.add_argument('--debug', action='store_true', help='Enable debug output')
    parser.add_argument('--lat', type=float, default=40.34536010088, help='Known latitude')
    parser.add_argument('--lon', type=float, default=-80.12878619119, help='Known longitude')
    parser.add_argument('--alt', type=float, default=326.5974, help='Known altitude')
    
    args = parser.parse_args()
    
    try:
        main(port=args.port, baud=args.baud, known_lat=args.lat, known_lon=args.lon, known_alt=args.alt, debug=args.debug)
    except KeyboardInterrupt:
        print("\nScript terminated by user")
    except Exception as e:
        print(f"Error: {e}")
        if args.debug:
            import traceback
            traceback.print_exc()