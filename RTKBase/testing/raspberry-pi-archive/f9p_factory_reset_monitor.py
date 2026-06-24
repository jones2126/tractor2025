#!/usr/bin/env python3
"""
F9P Factory Reset and Message Monitor
====================================
This script performs a factory reset on the F9P and then monitors all message types
(NMEA, UBX, RTCM) to see what's actually being output.

Usage:
    python3 f9p_factory_reset_monitor.py [port] [baudrate]
"""

import serial
import struct
import time
import sys
from datetime import datetime

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

def factory_reset_f9p(ser):
    """Perform a factory reset on the F9P"""
    print("Performing factory reset...")
    print("⚠ This will restore ALL settings to factory defaults!")
    
    ubx = UBXMessage()
    
    # CFG-CFG message to reset to factory defaults
    # clearMask: Clear all config sections
    # saveMask: Save nothing (use defaults)  
    # loadMask: Load everything from default
    clear_mask = 0x1F1F  # Clear all sections
    save_mask = 0x0000   # Don't save anything
    load_mask = 0x1F1F   # Load all sections from default
    
    payload = struct.pack('<III', clear_mask, save_mask, load_mask)
    reset_message = ubx.create_message(0x06, 0x09, payload)  # CFG-CFG
    
    try:
        ser.write(reset_message)
        ser.flush()
        print("✓ Factory reset command sent")
        
        # Wait for device to reset and restart
        print("Waiting for device to restart...")
        time.sleep(5)
        
        # Clear any pending data
        if ser.in_waiting > 0:
            ser.read(ser.in_waiting)
            
        print("✓ Factory reset completed")
        return True
        
    except Exception as e:
        print(f"✗ Error during factory reset: {e}")
        return False

def parse_nmea_message_type(data, pos):
    """Extract NMEA message type and sentence starting at pos (from rtcm_server_0714.py)"""
    try:
        # Find the end of the NMEA sentence (CR/LF)
        end_pos = pos
        while end_pos < len(data) and data[end_pos:end_pos + 1] not in [b'\r', b'\n']:
            end_pos += 1

        if end_pos >= len(data):
            return None, pos, None  # Incomplete message

        # Extract the message type (first 6 characters after $)
        nmea_sentence = data[pos:end_pos].decode('ascii', errors='ignore')
        if len(nmea_sentence) >= 6 and nmea_sentence.startswith('$'):
            # Extract message type, handling potential commas
            parts = nmea_sentence.split(',')
            if len(parts) > 0 and len(parts[0]) >= 6:
                message_type = parts[0][1:6]  # e.g., "GPGGA", "GPRMC", etc.
                # Validate it's a proper NMEA message type (letters and numbers only)
                if message_type.isalnum() and message_type.isupper():
                    return message_type, end_pos + 1, nmea_sentence
        return None, end_pos + 1, nmea_sentence
    except:
        return None, pos + 1, None

def parse_ubx_message_type(data, pos):
    """Extract UBX message type from data starting at pos (from rtcm_server_0714.py)"""
    try:
        if len(data) < pos + 6:
            return None, pos  # Not enough data for UBX header
            
        if data[pos:pos+2] != b'\xB5\x62':
            return None, pos  # Not a UBX message
            
        msg_class = data[pos + 2]
        msg_id = data[pos + 3]
        length = struct.unpack('<H', data[pos + 4:pos + 6])[0]
        
        # Check if we have the complete message (header + payload + checksum)
        total_length = 6 + length + 2
        if len(data) < pos + total_length:
            return None, pos  # Incomplete message
            
        message_type = f"{msg_class:02X}-{msg_id:02X}"
        return message_type, pos + total_length
    except:
        return None, pos + 1

def parse_rtcm_message_type(data, pos):
    """Extract RTCM message type from data starting at pos (from rtcm_server_0714.py)"""
    try:
        if len(data) < pos + 6:  # Need at least header + 2 bytes for message type
            return None, pos, None  # Not enough data for RTCM header
            
        if data[pos] != 0xD3:
            return None, pos, None  # Not an RTCM message
            
        # Extract message length (10 bits) from bytes 1-2
        length = ((data[pos + 1] & 0x03) << 8) | data[pos + 2]
        total_length = 3 + length + 3  # header + payload + CRC
        
        if len(data) < pos + total_length:
            return None, pos, None  # Incomplete message
            
        # Extract the complete RTCM message
        rtcm_message = data[pos:pos + total_length]
        
        # Extract message type (12 bits) from the first 12 bits of the payload
        # The message type is stored in bits 0-11 of the payload
        byte3 = data[pos + 3]
        byte4 = data[pos + 4] if pos + 4 < len(data) else 0
        
        # Message type is the first 12 bits of the payload
        message_type = (byte3 << 4) | (byte4 >> 4)
        
        return message_type, pos + total_length, rtcm_message
    except Exception as e:
        return None, pos + 1, None

def monitor_all_messages(ser, duration=30):
    """Monitor all message types coming from the F9P after reset"""
    print(f"\nMonitoring all message types for {duration} seconds...")
    print("Checking for NMEA, UBX, and RTCM messages...\n")
    
    start_time = time.time()
    message_counts = {}
    data_buffer = b''
    gga_found = False
    sample_messages = {}  # Store one sample of each message type
    
    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            new_data = ser.read(ser.in_waiting)
            data_buffer += new_data
            
            # Parse messages from the buffer (using same logic as rtcm_server_0714.py)
            pos = 0
            while pos < len(data_buffer):
                original_pos = pos
                message_detected = False

                # Check for NMEA message
                if pos < len(data_buffer) and data_buffer[pos:pos+1] == b'$':
                    msg_type, new_pos, sentence = parse_nmea_message_type(data_buffer, pos)
                    if msg_type:
                        key = f"NMEA_{msg_type}"
                        message_counts[key] = message_counts.get(key, 0) + 1
                        
                        # Store sample message
                        if key not in sample_messages:
                            sample_messages[key] = sentence
                        
                        # Check for GGA specifically
                        if msg_type.endswith("GGA"):
                            gga_found = True
                            print(f"✓ GGA FOUND: {sentence}")
                        
                        pos = new_pos
                        message_detected = True

                # Check for UBX message
                elif pos < len(data_buffer) - 1 and data_buffer[pos:pos+2] == b'\xB5\x62':
                    msg_type, new_pos = parse_ubx_message_type(data_buffer, pos)
                    if msg_type:
                        key = f"UBX_{msg_type}"
                        message_counts[key] = message_counts.get(key, 0) + 1
                        
                        # Store sample message info
                        if key not in sample_messages:
                            sample_messages[key] = f"UBX message class/ID: {msg_type}"
                        
                        pos = new_pos
                        message_detected = True

                # Check for RTCM message
                elif pos < len(data_buffer) and data_buffer[pos:pos+1] == b'\xD3':
                    msg_type, new_pos, rtcm_message = parse_rtcm_message_type(data_buffer, pos)
                    if msg_type and rtcm_message:
                        key = f"RTCM_{msg_type}"
                        message_counts[key] = message_counts.get(key, 0) + 1
                        
                        # Store sample message info
                        if key not in sample_messages:
                            sample_messages[key] = f"RTCM Type {msg_type}, Length: {len(rtcm_message)} bytes"
                        
                        pos = new_pos
                        message_detected = True
                    elif new_pos > pos:
                        pos = new_pos

                # If no message detected, advance by 1 byte
                if not message_detected:
                    pos += 1

                # Prevent infinite loops
                if pos <= original_pos:
                    pos = original_pos + 1

            # Keep only the last 10KB in buffer to prevent memory issues
            if len(data_buffer) > 10240:
                data_buffer = data_buffer[-5120:]  # Keep last 5KB
        
        time.sleep(0.01)
    
    # Report results
    elapsed = time.time() - start_time
    print(f"\n" + "=" * 60)
    print(f"MESSAGE ANALYSIS AFTER FACTORY RESET ({elapsed:.1f}s)")
    print("=" * 60)
    
    # Group messages by type
    nmea_messages = {k: v for k, v in message_counts.items() if k.startswith('NMEA_')}
    ubx_messages = {k: v for k, v in message_counts.items() if k.startswith('UBX_')}
    rtcm_messages = {k: v for k, v in message_counts.items() if k.startswith('RTCM_')}
    
    total_count = 0
    
    if nmea_messages:
        print("\n📡 NMEA Messages:")
        for msg_key, count in sorted(nmea_messages.items()):
            msg_type = msg_key.replace('NMEA_', '')
            rate = count / elapsed
            total_count += count
            print(f"  {msg_type}: {rate:.2f} Hz ({count} messages)")
            if msg_key in sample_messages:
                print(f"    Sample: {sample_messages[msg_key][:100]}...")
    
    if ubx_messages:
        print("\n🛰️  UBX Messages:")
        for msg_key, count in sorted(ubx_messages.items()):
            msg_type = msg_key.replace('UBX_', '')
            rate = count / elapsed
            total_count += count
            print(f"  {msg_type}: {rate:.2f} Hz ({count} messages)")
    
    if rtcm_messages:
        print("\n📶 RTCM Messages:")
        for msg_key, count in sorted(rtcm_messages.items()):
            msg_type = msg_key.replace('RTCM_', '')
            rate = count / elapsed
            total_count += count
            print(f"  Type {msg_type}: {rate:.2f} Hz ({count} messages)")
    
    if total_count > 0:
        total_rate = total_count / elapsed
        print(f"\n📊 Total: {total_rate:.2f} Hz ({total_count} messages)")
    else:
        print("\n❌ No messages detected!")
    
    # GGA Status
    print(f"\n🎯 GGA Status:")
    if gga_found:
        print("  ✅ GGA messages ARE being output!")
    else:
        print("  ❌ NO GGA messages detected")
    
    # Recommendations
    print(f"\n💡 Recommendations:")
    if not nmea_messages and not ubx_messages and not rtcm_messages:
        print("  • Check physical connections")
        print("  • Verify correct serial port")
        print("  • Check if device is powered on")
    elif ubx_messages and not nmea_messages:
        print("  • Device is in binary mode - NMEA is disabled")
        print("  • Use u-center or UBX commands to enable NMEA")
    elif nmea_messages and not gga_found:
        print("  • NMEA is working but GGA is disabled")
        print("  • Use PUBX commands to enable GGA")
    elif gga_found:
        print("  • Everything looks good!")
        print("  • GGA is working normally")
    
    return gga_found, message_counts

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
    
    print("F9P Factory Reset and Message Monitor")
    print("=" * 45)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    try:
        # Open serial connection
        print("\nOpening serial connection...")
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        print("✓ Serial connection established")
        
        # Perform factory reset
        reset_success = factory_reset_f9p(ser)
        
        if reset_success:
            # Monitor messages after reset
            gga_found, message_counts = monitor_all_messages(ser, duration=30)
            
            if gga_found:
                print("\n🎉 Success! GGA messages are working after factory reset.")
            else:
                print("\n⚠️  GGA still not detected. Manual configuration may be needed.")
        else:
            print("\n❌ Factory reset failed. Cannot proceed with monitoring.")
        
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