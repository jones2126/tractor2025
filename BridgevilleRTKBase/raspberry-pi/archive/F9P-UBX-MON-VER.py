#!/usr/bin/env python3
"""
F9P Firmware Version Query Script
=================================
Simple script to query the firmware version of a u-blox F9P GPS module
connected to /dev/ttyACM0 on Raspberry Pi.
"""

import serial
import struct
import time

def calculate_ubx_checksum(msg_class, msg_id, payload):
    """Calculate UBX checksum for the given message class, ID, and payload."""
    ck_a = 0
    ck_b = 0
    
    # Add message class and ID to checksum
    ck_a = (ck_a + msg_class) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    ck_a = (ck_a + msg_id) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    
    # Add payload length to checksum
    payload_len = len(payload)
    ck_a = (ck_a + (payload_len & 0xFF)) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    ck_a = (ck_a + ((payload_len >> 8) & 0xFF)) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    
    # Add payload bytes to checksum
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    
    return ck_a, ck_b

def create_ubx_message(msg_class, msg_id, payload=b''):
    """Create a complete UBX message with header, length, payload, and checksum."""
    # UBX header
    message = b'\xB5\x62'
    
    # Message class and ID
    message += struct.pack('BB', msg_class, msg_id)
    
    # Payload length (little-endian)
    message += struct.pack('<H', len(payload))
    
    # Payload
    message += payload
    
    # Calculate and add checksum
    ck_a, ck_b = calculate_ubx_checksum(msg_class, msg_id, payload)
    message += struct.pack('BB', ck_a, ck_b)
    
    return message

def parse_ubx_response(data):
    """Parse UBX response and look for MON-VER messages."""
    pos = 0
    while pos < len(data):
        # Look for UBX header
        if pos + 1 < len(data) and data[pos:pos+2] == b'\xB5\x62':
            if pos + 6 <= len(data):
                msg_class = data[pos + 2]
                msg_id = data[pos + 3]
                length = struct.unpack('<H', data[pos + 4:pos + 6])[0]
                
                # Check if we have the complete message
                if pos + 6 + length + 2 <= len(data):
                    if msg_class == 0x0A and msg_id == 0x04:  # MON-VER
                        payload = data[pos + 6:pos + 6 + length]
                        return parse_mon_ver(payload)
                    pos += 6 + length + 2
                else:
                    break
            else:
                break
        else:
            pos += 1
    
    return None

def parse_mon_ver(payload):
    """Parse MON-VER payload to extract version information."""
    if len(payload) < 40:
        return None
    
    # Extract software version (30 bytes, null-terminated)
    sw_version = payload[0:30].rstrip(b'\x00').decode('ascii', errors='ignore')
    
    # Extract hardware version (10 bytes, null-terminated)
    hw_version = payload[30:40].rstrip(b'\x00').decode('ascii', errors='ignore')
    
    # Extract extensions (if any)
    extensions = []
    pos = 40
    while pos + 30 <= len(payload):
        ext = payload[pos:pos+30].rstrip(b'\x00').decode('ascii', errors='ignore')
        if ext:
            extensions.append(ext)
        pos += 30
    
    return {
        'software_version': sw_version,
        'hardware_version': hw_version,
        'extensions': extensions
    }

def query_f9p_version():
    """Query F9P firmware version."""
    try:
        # Open serial connection
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=5)
        print("Connected to F9P on /dev/ttyACM0")
        
        # Clear any existing data
        ser.reset_input_buffer()
        time.sleep(0.1)
        
        # Create MON-VER request message (Class: 0x0A, ID: 0x04, no payload)
        mon_ver_request = create_ubx_message(0x0A, 0x04)
        
        print("Sending version query...")
        ser.write(mon_ver_request)
        ser.flush()
        
        # Read response with timeout
        response_data = b''
        start_time = time.time()
        
        while time.time() - start_time < 3:  # 3 second timeout
            if ser.in_waiting > 0:
                new_data = ser.read(ser.in_waiting)
                response_data += new_data
                
                # Try to parse response
                version_info = parse_ubx_response(response_data)
                if version_info:
                    print("\n" + "="*50)
                    print("F9P FIRMWARE VERSION INFORMATION")
                    print("="*50)
                    print(f"Software Version: {version_info['software_version']}")
                    print(f"Hardware Version: {version_info['hardware_version']}")
                    
                    if version_info['extensions']:
                        print("\nExtensions:")
                        for ext in version_info['extensions']:
                            print(f"  {ext}")
                    
                    print("="*50)
                    ser.close()
                    return True
            
            time.sleep(0.1)
        
        print("Timeout: No version response received")
        ser.close()
        return False
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    print("F9P Firmware Version Query")
    print("-" * 30)
    
    success = query_f9p_version()
    
    if not success:
        print("\nTroubleshooting tips:")
        print("1. Check that F9P is connected to /dev/ttyACM0")
        print("2. Ensure no other processes are using the serial port")
        print("3. Verify the F9P is powered on and responding")
        print("4. Try running: ls -la /dev/ttyACM*")
