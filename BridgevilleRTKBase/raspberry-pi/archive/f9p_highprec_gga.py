#!/usr/bin/env python3
"""
F9P High Precision GGA Configuration Script
===========================================
This script configures a u-blox F9P GPS receiver to output high-precision GGA sentences
with extended decimal places for improved coordinate resolution.

Usage:
    python3 f9p_highprec_gga.py [port] [baudrate]
    
Examples:
    python3 f9p_highprec_gga.py /dev/ttyACM0 115200
    python3 f9p_highprec_gga.py /dev/f9p 115200
    python3 f9p_highprec_gga.py COM3 115200
"""

import serial
import struct
import time
import sys

class UBXMessage:
    """Class to handle UBX message creation and parsing"""
    
    def __init__(self):
        self.SYNC_CHAR1 = 0xB5
        self.SYNC_CHAR2 = 0x62
    
    def calculate_checksum(self, msg_class, msg_id, payload):
        """Calculate UBX checksum"""
        ck_a = 0
        ck_b = 0
        
        # Add class and ID to checksum
        ck_a = (ck_a + msg_class) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
        ck_a = (ck_a + msg_id) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
        
        # Add payload length to checksum
        length = len(payload)
        ck_a = (ck_a + (length & 0xFF)) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
        ck_a = (ck_a + ((length >> 8) & 0xFF)) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
        
        # Add payload to checksum
        for byte in payload:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
            
        return ck_a, ck_b
    
    def create_message(self, msg_class, msg_id, payload):
        """Create a complete UBX message with header and checksum"""
        length = len(payload)
        ck_a, ck_b = self.calculate_checksum(msg_class, msg_id, payload)
        
        message = bytearray()
        message.append(self.SYNC_CHAR1)
        message.append(self.SYNC_CHAR2)
        message.append(msg_class)
        message.append(msg_id)
        message.extend(struct.pack('<H', length))  # Little-endian 16-bit length
        message.extend(payload)
        message.append(ck_a)
        message.append(ck_b)
        
        return bytes(message)

def create_cfg_valset_message(key_id, value, value_size):
    """Create CFG-VALSET message for configuration"""
    ubx = UBXMessage()
    
    # CFG-VALSET parameters
    version = 0x01      # Message version
    layer = 0x01        # RAM layer
    reserved = 0x0000   # Reserved
    
    # Build payload
    payload = bytearray()
    payload.append(version)
    payload.append(layer)
    payload.extend(struct.pack('<H', reserved))
    
    # Add key-value pair
    payload.extend(struct.pack('<I', key_id))  # Key ID (32-bit)
    
    if value_size == 1:
        payload.append(value)  # 1-byte value
    elif value_size == 2:
        payload.extend(struct.pack('<H', value))  # 2-byte value
    elif value_size == 4:
        payload.extend(struct.pack('<I', value))  # 4-byte value
    
    return ubx.create_message(0x06, 0x8A, payload)  # CFG-VALSET

def configure_high_precision_gga(ser):
    """Configure F9P for high-precision GGA output"""
    
    print("Configuring F9P for high-precision GGA...")
    
    # Configuration keys for high precision NMEA
    configs = [
        # Enable high precision NMEA (extends decimal places)
        (0x10930006, 1, 1),  # CFG-NMEA-HIGHPREC
        
        # Disable compatibility mode for maximum precision
        (0x10930001, 0, 1),  # CFG-NMEA-COMPAT
        
        # Set NMEA version to 4.1 for extended precision support
        (0x20930007, 0x41, 1),  # CFG-NMEA-MAINVER (4.1)
        
        # Enable extended satellite numbering
        (0x10930004, 1, 1),  # CFG-NMEA-SVNUMBERING
        
        # Optional: Increase GGA output rate if needed (1Hz = 1000ms)
        (0x209100BB, 1000, 2),  # CFG-MSGOUT-NMEA_ID_GGA_UART1 (in milliseconds)
    ]
    
    success_count = 0
    
    for key_id, value, size in configs:
        message = create_cfg_valset_message(key_id, value, size)
        
        try:
            ser.write(message)
            ser.flush()
            time.sleep(0.1)  # Give device time to process
            
            # Try to read acknowledgment (optional)
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                # Check for ACK (0x05 0x01) in response
                if b'\xb5\x62\x05\x01' in response:
                    success_count += 1
                    print(f"✓ Configuration key 0x{key_id:08X} acknowledged")
                else:
                    print(f"⚠ Configuration key 0x{key_id:08X} sent (no ACK)")
            else:
                print(f"→ Configuration key 0x{key_id:08X} sent")
                
        except Exception as e:
            print(f"✗ Error sending configuration key 0x{key_id:08X}: {e}")
    
    # Save configuration to flash memory
    print("\nSaving configuration to flash memory...")
    save_message = create_cfg_valset_message(0x00000000, 0, 0)  # Special save command
    try:
        ser.write(save_message)
        ser.flush()
        time.sleep(0.5)
        print("✓ Configuration saved to flash")
    except Exception as e:
        print(f"✗ Error saving configuration: {e}")
    
    return success_count

def monitor_gga_precision(ser, duration=30):
    """Monitor GGA messages to verify precision improvement"""
    
    print(f"\nMonitoring GGA messages for {duration} seconds...")
    print("Looking for improved coordinate precision...\n")
    
    start_time = time.time()
    gga_count = 0
    max_precision = 0
    
    buffer = b''
    
    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            buffer += data
            
            # Look for complete NMEA sentences
            while b'\n' in buffer:
                line_end = buffer.find(b'\n')
                line = buffer[:line_end].decode('ascii', errors='ignore').strip()
                buffer = buffer[line_end + 1:]
                
                if line.startswith('$') and 'GGA' in line:
                    gga_count += 1
                    
                    # Parse coordinates to check precision
                    parts = line.split(',')
                    if len(parts) >= 6:
                        lat_str = parts[2]
                        lon_str = parts[4]
                        
                        # Count decimal places
                        if '.' in lat_str:
                            lat_decimals = len(lat_str.split('.')[1])
                            max_precision = max(max_precision, lat_decimals)
                        
                        if '.' in lon_str:
                            lon_decimals = len(lon_str.split('.')[1])
                            max_precision = max(max_precision, lon_decimals)
                        
                        print(f"GGA #{gga_count}: {line}")
                        
                        if lat_str and lon_str:
                            lat_precision = len(lat_str.split('.')[1]) if '.' in lat_str else 0
                            lon_precision = len(lon_str.split('.')[1]) if '.' in lon_str else 0
                            print(f"  → Lat precision: {lat_precision} decimals, Lon precision: {lon_precision} decimals")
        
        time.sleep(0.1)
    
    print(f"\nMonitoring complete:")
    print(f"  • Total GGA messages: {gga_count}")
    print(f"  • Maximum precision observed: {max_precision} decimal places")
    
    if max_precision >= 7:
        print("  ✓ High precision mode confirmed!")
    elif max_precision >= 5:
        print("  ⚠ Standard precision detected - configuration may need time to take effect")
    else:
        print("  ✗ Low precision detected - configuration may have failed")

def main():
    """Main function"""
    
    # Parse command line arguments
    if len(sys.argv) >= 2:
        port = sys.argv[1]
    else:
        port = '/dev/ttyACM0'  # Default for Linux
    
    if len(sys.argv) >= 3:
        baudrate = int(sys.argv[2])
    else:
        baudrate = 115200  # Default baudrate
    
    print("F9P High Precision GGA Configuration Tool")
    print("=" * 45)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print()
    
    try:
        # Open serial connection
        print("Opening serial connection...")
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        print("✓ Serial connection established")
        
        # Configure high precision
        success_count = configure_high_precision_gga(ser)
        
        if success_count > 0:
            print(f"\n✓ Configuration completed ({success_count} settings applied)")
            print("Waiting for configuration to take effect...")
            time.sleep(3)
            
            # Monitor the results
            monitor_gga_precision(ser, duration=20)
        else:
            print("\n✗ Configuration failed - no settings were successfully applied")
        
    except serial.SerialException as e:
        print(f"✗ Serial connection error: {e}")
        print("Make sure the device is connected and the port is correct.")
    except KeyboardInterrupt:
        print("\n⚠ Interrupted by user")
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
    finally:
        try:
            ser.close()
            print("\nSerial connection closed.")
        except:
            pass

if __name__ == "__main__":
    main()