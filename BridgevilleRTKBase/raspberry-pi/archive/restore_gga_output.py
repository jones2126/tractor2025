#!/usr/bin/env python3
"""
Restore GGA Output Script
========================
This script restores GGA message output on the F9P and ensures it's configured 
for high precision with proper output rate.

Usage:
    python3 restore_gga_output.py [port] [baudrate]
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
    layer = 0x01        # RAM layer only for immediate effect
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

def send_nmea_command(ser, command):
    """Send NMEA command to configure message rates"""
    if not command.startswith('$'):
        command = '$' + command
    if not command.endswith('\r\n'):
        command += '\r\n'
    
    try:
        ser.write(command.encode('ascii'))
        ser.flush()
        time.sleep(0.2)
        print(f"✓ Sent NMEA command: {command.strip()}")
        return True
    except Exception as e:
        print(f"✗ Error sending NMEA command: {e}")
        return False

def restore_gga_output(ser):
    """Restore GGA output using multiple methods"""
    
    print("Attempting to restore GGA output...")
    print("=" * 50)
    
    # Method 1: Use NMEA commands (more reliable for message enable/disable)
    print("\nMethod 1: Using NMEA commands")
    nmea_commands = [
        "PUBX,40,GGA,0,1,0,0,0,0",  # Enable GGA on UART1 at 1Hz
        "PUBX,40,RMC,0,1,0,0,0,0",  # Ensure RMC stays enabled  
        "PUBX,40,VTG,0,1,0,0,0,0",  # Ensure VTG stays enabled
    ]
    
    for cmd in nmea_commands:
        send_nmea_command(ser, cmd)
    
    # Method 2: Use UBX CFG-VALSET commands
    print("\nMethod 2: Using UBX configuration")
    
    configs = [
        # Enable high precision NMEA first
        (0x10930006, 1, 1),     # CFG-NMEA-HIGHPREC = 1
        
        # Set GGA output rate to 1Hz (1000ms) on UART1
        (0x209100BB, 1000, 2),  # CFG-MSGOUT-NMEA_ID_GGA_UART1
        
        # Ensure other common messages stay enabled
        (0x209100AC, 1000, 2),  # CFG-MSGOUT-NMEA_ID_RMC_UART1 
        (0x209100B1, 1000, 2),  # CFG-MSGOUT-NMEA_ID_VTG_UART1
        
        # Set NMEA version for best compatibility
        (0x20930007, 0x41, 1),  # CFG-NMEA-MAINVER = 4.1
    ]
    
    for key_id, value, size in configs:
        message = create_cfg_valset_message(key_id, value, size)
        
        try:
            ser.write(message)
            ser.flush()
            time.sleep(0.2)
            
            # Check for response
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                if b'\xb5\x62\x05\x01' in response:
                    print(f"✓ UBX config 0x{key_id:08X} = {value} acknowledged")
                elif b'\xb5\x62\x05\x00' in response:
                    print(f"✗ UBX config 0x{key_id:08X} = {value} NACK")
                else:
                    print(f"→ UBX config 0x{key_id:08X} = {value} sent")
            else:
                print(f"→ UBX config 0x{key_id:08X} = {value} sent")
                
        except Exception as e:
            print(f"✗ Error sending UBX config 0x{key_id:08X}: {e}")
    
    # Method 3: Reset to factory defaults if needed (use with caution)
    print(f"\nMethod 3: Available if needed")
    print("To reset all message configurations to factory defaults:")
    print("Send NMEA command: $PUBX,41,1,0007,0003,19200,0*13")
    print("(This will reset baud rate to 19200 and all message rates)")

def monitor_nmea_output(ser, duration=15):
    """Monitor NMEA output to verify GGA is present"""
    
    print(f"\nMonitoring NMEA output for {duration} seconds...")
    print("Looking for GGA messages...\n")
    
    start_time = time.time()
    message_counts = {}
    gga_found = False
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
                
                if line.startswith('$') and len(line) > 6:
                    # Extract message type
                    msg_type = line[1:6] if len(line) >= 6 else line[1:]
                    message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
                    
                    if 'GGA' in msg_type:
                        gga_found = True
                        print(f"✓ GGA FOUND: {line}")
                        
                        # Check precision
                        parts = line.split(',')
                        if len(parts) >= 6:
                            lat_str = parts[2]
                            lon_str = parts[4]
                            if lat_str and '.' in lat_str:
                                precision = len(lat_str.split('.')[1])
                                print(f"  → Coordinate precision: {precision} decimal places")
                    else:
                        print(f"  {msg_type}: {line}")
        
        time.sleep(0.1)
    
    print(f"\n" + "=" * 50)
    print("NMEA Message Summary:")
    for msg_type, count in sorted(message_counts.items()):
        print(f"  {msg_type}: {count} messages")
    
    if gga_found:
        print(f"\n✓ SUCCESS: GGA messages are being output!")
    else:
        print(f"\n✗ PROBLEM: No GGA messages detected")
        print("Try the following:")
        print("1. Check if device is in binary mode instead of NMEA")
        print("2. Reset to factory defaults")
        print("3. Check physical connections")
    
    return gga_found

def main():
    """Main function"""
    
    # Parse command line arguments
    if len(sys.argv) >= 2:
        port = sys.argv[1]
    else:
        port = '/dev/f9p'  # Default
    
    if len(sys.argv) >= 3:
        baudrate = int(sys.argv[2])
    else:
        baudrate = 115200  # Default baudrate
    
    print("F9P GGA Output Restoration Tool")
    print("=" * 40)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    
    try:
        # Open serial connection
        print("\nOpening serial connection...")
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        print("✓ Serial connection established")
        
        # Restore GGA output
        restore_gga_output(ser)
        
        print("\nWaiting for configuration to take effect...")
        time.sleep(3)
        
        # Monitor the results
        gga_restored = monitor_nmea_output(ser, duration=15)
        
        if gga_restored:
            print("\n🎉 GGA output successfully restored!")
        else:
            print("\n⚠ GGA output not detected. Manual intervention may be required.")
        
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