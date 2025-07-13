import serial
import time
import struct
from datetime import datetime
import re

# Configure the serial port
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# Message rate monitoring configuration
RATE_REPORT_INTERVAL = 10  # Report message rates every 10 seconds

def parse_nmea_message_type(data, pos):
    """
    Extract NMEA message type from data starting at pos.
    Returns (message_type, message_end_pos) or (None, pos)
    """
    try:
        # Find the end of the NMEA sentence (CR/LF)
        end_pos = pos
        while end_pos < len(data) and data[end_pos:end_pos+1] not in [b'\r', b'\n']:
            end_pos += 1
        
        if end_pos >= len(data):
            return None, pos  # Incomplete message
            
        # Extract the message type (first 6 characters after $)
        nmea_sentence = data[pos:end_pos].decode('ascii', errors='ignore')
        if len(nmea_sentence) >= 6 and nmea_sentence.startswith('

def parse_ubx_message_type(data, pos):
    """
    Extract UBX message type from data starting at pos.
    Returns (message_type, message_end_pos) or (None, pos)
    """
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
    """
    Extract RTCM message type from data starting at pos.
    Returns (message_type, message_end_pos) or (None, pos)
    """
    try:
        if len(data) < pos + 6:  # Need at least header + 2 bytes for message type
            return None, pos  # Not enough data for RTCM header
            
        if data[pos] != 0xD3:
            return None, pos  # Not an RTCM message
            
        # Extract message length (10 bits) from bytes 1-2
        length = ((data[pos + 1] & 0x03) << 8) | data[pos + 2]
        total_length = 3 + length + 3  # header + payload + CRC
        
        if len(data) < pos + total_length:
            return None, pos  # Incomplete message
            
        # Extract message type (12 bits) from the first 12 bits of the payload
        # The message type is stored in bits 0-11 of the payload
        byte3 = data[pos + 3]
        byte4 = data[pos + 4] if pos + 4 < len(data) else 0
        
        # Message type is the first 12 bits of the payload
        message_type = (byte3 << 4) | (byte4 >> 4)
        
        return message_type, pos + total_length
    except Exception as e:
        return None, pos + 1

def report_message_rates(message_counts, elapsed_time):
    """Report message rates for all detected message types."""
    if elapsed_time <= 0:
        return
        
    print(f"\n=== Message Rates Report (over {elapsed_time:.1f}s) ===")
    
    # Group messages by type
    nmea_messages = {k.replace('NMEA_', ''): v for k, v in message_counts.items() if k.startswith('NMEA_')}
    ubx_messages = {k.replace('UBX_', ''): v for k, v in message_counts.items() if k.startswith('UBX_')}
    rtcm_messages = {k.replace('RTCM_', ''): v for k, v in message_counts.items() if k.startswith('RTCM_')}
    
    total_count = 0
    
    if nmea_messages:
        print("NMEA Messages:")
        for msg_type, count in sorted(nmea_messages.items()):
            rate = count / elapsed_time
            total_count += count
            print(f"  {msg_type}: {rate:.2f} Hz ({count} messages)")
    
    if ubx_messages:
        print("UBX Messages:")
        for msg_type, count in sorted(ubx_messages.items()):
            rate = count / elapsed_time
            total_count += count
            print(f"  {msg_type}: {rate:.2f} Hz ({count} messages)")
    
    if rtcm_messages:
        print("RTCM Messages:")
        for msg_type, count in sorted(rtcm_messages.items(), key=lambda x: int(x[0]) if x[0].isdigit() else 0):
            rate = count / elapsed_time
            total_count += count
            print(f"  Type {msg_type}: {rate:.2f} Hz ({count} messages)")
    
    if total_count > 0:
        total_rate = total_count / elapsed_time
        print(f"Total: {total_rate:.2f} Hz ({total_count} messages)")
    else:
        print("No messages detected in this interval")
    print("=" * 50)

print("Listening for GPS data on /dev/ttyACM0 with message rate monitoring...")
print("Press Ctrl+C to stop")

# Message rate monitoring variables
message_counts = {}
last_rate_check = time.time()
data_buffer = b''

try:
    with open('gps_log.txt', 'wb') as log_file:
        while True:
            if ser.in_waiting > 0:
                new_data = ser.read(ser.in_waiting)
                log_file.write(new_data)
                log_file.flush()
                
                # Add new data to buffer for parsing
                data_buffer += new_data
                
                # Parse messages from the buffer
                pos = 0
                while pos < len(data_buffer):
                    original_pos = pos
                    message_detected = False
                    
                    # Check for NMEA message
                    if pos < len(data_buffer) and data_buffer[pos:pos+1] == b'
                
                # Keep only the last 10KB in buffer to prevent memory issues
                if len(data_buffer) > 10240:
                    data_buffer = data_buffer[-5120:]  # Keep last 5KB
                
                # Report message rates periodically
                current_time = time.time()
                if current_time - last_rate_check >= RATE_REPORT_INTERVAL:
                    elapsed = current_time - last_rate_check
                    report_message_rates(message_counts, elapsed)
                    message_counts.clear()
                    last_rate_check = current_time
                
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage

except KeyboardInterrupt:
    print("\nStopped by user")
    
    # Final report
    current_time = time.time()
    elapsed = current_time - last_rate_check
    if elapsed > 0 and message_counts:
        print("\nFinal message rates:")
        report_message_rates(message_counts, elapsed)
    
    ser.close()

except serial.SerialException as e:
    print(f"Serial error: {e}")
    ser.close()):
            # Extract message type, handling potential commas
            parts = nmea_sentence.split(',')
            if len(parts) > 0 and len(parts[0]) >= 6:
                message_type = parts[0][1:6]  # e.g., "GPGGA", "GPRMC", etc.
                # Validate it's a proper NMEA message type (letters and numbers only)
                if message_type.isalnum() and message_type.isupper():
                    return message_type, end_pos + 1
        return None, end_pos + 1
    except:
        return None, pos + 1

def parse_ubx_message_type(data, pos):
    """
    Extract UBX message type from data starting at pos.
    Returns (message_type, message_end_pos) or (None, pos)
    """
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
    """
    Extract RTCM message type from data starting at pos.
    Returns (message_type, message_end_pos) or (None, pos)
    """
    try:
        if len(data) < pos + 3:
            return None, pos  # Not enough data for RTCM header
            
        if data[pos] != 0xD3:
            return None, pos  # Not an RTCM message
            
        # Extract message length (10 bits) from bytes 1-2
        length = ((data[pos + 1] & 0x03) << 8) | data[pos + 2]
        total_length = 3 + length + 3  # header + payload + CRC
        
        if len(data) < pos + total_length:
            return None, pos  # Incomplete message
            
        if len(data) < pos + 5:
            return None, pos + 1  # Not enough data for message type
            
        # Extract message type (12 bits) from bytes 3-4
        message_type = ((data[pos + 3] & 0x0F) << 8) | data[pos + 4]
        return message_type, pos + total_length
    except:
        return None, pos + 1

def report_message_rates(message_counts, elapsed_time):
    """Report message rates for all detected message types."""
    if elapsed_time <= 0:
        return
        
    print(f"\n=== Message Rates Report (over {elapsed_time:.1f}s) ===")
    
    # Group messages by type
    nmea_messages = {k: v for k, v in message_counts.items() if isinstance(k, str) and len(k) == 5}
    ubx_messages = {k: v for k, v in message_counts.items() if isinstance(k, str) and '-' in k}
    rtcm_messages = {k: v for k, v in message_counts.items() if isinstance(k, int)}
    
    total_count = 0
    
    if nmea_messages:
        print("NMEA Messages:")
        for msg_type, count in sorted(nmea_messages.items()):
            rate = count / elapsed_time
            total_count += count
            print(f"  {msg_type}: {rate:.2f} Hz ({count} messages)")
    
    if ubx_messages:
        print("UBX Messages:")
        for msg_type, count in sorted(ubx_messages.items()):
            rate = count / elapsed_time
            total_count += count
            print(f"  {msg_type}: {rate:.2f} Hz ({count} messages)")
    
    if rtcm_messages:
        print("RTCM Messages:")
        for msg_type, count in sorted(rtcm_messages.items()):
            rate = count / elapsed_time
            total_count += count
            print(f"  Type {msg_type}: {rate:.2f} Hz ({count} messages)")
    
    if total_count > 0:
        total_rate = total_count / elapsed_time
        print(f"Total: {total_rate:.2f} Hz ({total_count} messages)")
    else:
        print("No messages detected in this interval")
    print("=" * 50)

print("Listening for GPS data on /dev/ttyACM0 with message rate monitoring...")
print("Press Ctrl+C to stop")

# Message rate monitoring variables
message_counts = {}
last_rate_check = time.time()
data_buffer = b''

try:
    with open('gps_log.txt', 'wb') as log_file:
        while True:
            if ser.in_waiting > 0:
                new_data = ser.read(ser.in_waiting)
                log_file.write(new_data)
                log_file.flush()
                
                # Add new data to buffer for parsing
                data_buffer += new_data
                
                # Parse messages from the buffer
                pos = 0
                while pos < len(data_buffer):
                    original_pos = pos
                    message_detected = False
                    
                    # Check for NMEA message
                    if pos < len(data_buffer) and data_buffer[pos:pos+1] == b'$':
                        msg_type, new_pos = parse_nmea_message_type(data_buffer, pos)
                        if msg_type:
                            message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
                            pos = new_pos
                            message_detected = True
                    
                    # Check for UBX message
                    elif pos < len(data_buffer) - 1 and data_buffer[pos:pos+2] == b'\xB5\x62':
                        msg_type, new_pos = parse_ubx_message_type(data_buffer, pos)
                        if msg_type:
                            message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
                            pos = new_pos
                            message_detected = True
                    
                    # Check for RTCM message
                    elif pos < len(data_buffer) and data_buffer[pos:pos+1] == b'\xD3':
                        msg_type, new_pos = parse_rtcm_message_type(data_buffer, pos)
                        if msg_type:
                            message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
                            pos = new_pos
                            message_detected = True
                    
                    # If no message detected, advance by 1 byte
                    if not message_detected:
                        pos += 1
                    
                    # Prevent infinite loops
                    if pos <= original_pos:
                        pos = original_pos + 1
                
                # Keep only the last 10KB in buffer to prevent memory issues
                if len(data_buffer) > 10240:
                    data_buffer = data_buffer[-5120:]  # Keep last 5KB
                
                # Report message rates periodically
                current_time = time.time()
                if current_time - last_rate_check >= RATE_REPORT_INTERVAL:
                    elapsed = current_time - last_rate_check
                    report_message_rates(message_counts, elapsed)
                    message_counts.clear()
                    last_rate_check = current_time
                
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage

except KeyboardInterrupt:
    print("\nStopped by user")
    
    # Final report
    current_time = time.time()
    elapsed = current_time - last_rate_check
    if elapsed > 0 and message_counts:
        print("\nFinal message rates:")
        report_message_rates(message_counts, elapsed)
    
    ser.close()

except serial.SerialException as e:
    print(f"Serial error: {e}")
    ser.close():
                        msg_type, new_pos = parse_nmea_message_type(data_buffer, pos)
                        if msg_type:
                            message_counts[f"NMEA_{msg_type}"] = message_counts.get(f"NMEA_{msg_type}", 0) + 1
                            pos = new_pos
                            message_detected = True
                    
                    # Check for UBX message
                    elif pos < len(data_buffer) - 1 and data_buffer[pos:pos+2] == b'\xB5\x62':
                        msg_type, new_pos = parse_ubx_message_type(data_buffer, pos)
                        if msg_type:
                            message_counts[f"UBX_{msg_type}"] = message_counts.get(f"UBX_{msg_type}", 0) + 1
                            pos = new_pos
                            message_detected = True
                    
                    # Check for RTCM message
                    elif pos < len(data_buffer) and data_buffer[pos:pos+1] == b'\xD3':
                        msg_type, new_pos = parse_rtcm_message_type(data_buffer, pos)
                        if msg_type:
                            message_counts[f"RTCM_{msg_type}"] = message_counts.get(f"RTCM_{msg_type}", 0) + 1
                            pos = new_pos
                            message_detected = True
                    
                    # If no message detected, advance by 1 byte
                    if not message_detected:
                        pos += 1
                    
                    # Prevent infinite loops
                    if pos <= original_pos:
                        pos = original_pos + 1
                
                # Keep only the last 10KB in buffer to prevent memory issues
                if len(data_buffer) > 10240:
                    data_buffer = data_buffer[-5120:]  # Keep last 5KB
                
                # Report message rates periodically
                current_time = time.time()
                if current_time - last_rate_check >= RATE_REPORT_INTERVAL:
                    elapsed = current_time - last_rate_check
                    report_message_rates(message_counts, elapsed)
                    message_counts.clear()
                    last_rate_check = current_time
                
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage

except KeyboardInterrupt:
    print("\nStopped by user")
    
    # Final report
    current_time = time.time()
    elapsed = current_time - last_rate_check
    if elapsed > 0 and message_counts:
        print("\nFinal message rates:")
        report_message_rates(message_counts, elapsed)
    
    ser.close()

except serial.SerialException as e:
    print(f"Serial error: {e}")
    ser.close()):
            # Extract message type, handling potential commas
            parts = nmea_sentence.split(',')
            if len(parts) > 0 and len(parts[0]) >= 6:
                message_type = parts[0][1:6]  # e.g., "GPGGA", "GPRMC", etc.
                # Validate it's a proper NMEA message type (letters and numbers only)
                if message_type.replace('A', '').replace('B', '').replace('C', '').replace('D', '').replace('E', '').replace('F', '').replace('G', '').replace('H', '').replace('I', '').replace('J', '').replace('K', '').replace('L', '').replace('M', '').replace('N', '').replace('O', '').replace('P', '').replace('Q', '').replace('R', '').replace('S', '').replace('T', '').replace('U', '').replace('V', '').replace('W', '').replace('X', '').replace('Y', '').replace('Z', '').replace('0', '').replace('1', '').replace('2', '').replace('3', '').replace('4', '').replace('5', '').replace('6', '').replace('7', '').replace('8', '').replace('9', '') == '':
                    return message_type, end_pos + 1
        return None, end_pos + 1
    except:
        return None, pos + 1

def parse_ubx_message_type(data, pos):
    """
    Extract UBX message type from data starting at pos.
    Returns (message_type, message_end_pos) or (None, pos)
    """
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
    """
    Extract RTCM message type from data starting at pos.
    Returns (message_type, message_end_pos) or (None, pos)
    """
    try:
        if len(data) < pos + 3:
            return None, pos  # Not enough data for RTCM header
            
        if data[pos] != 0xD3:
            return None, pos  # Not an RTCM message
            
        # Extract message length (10 bits) from bytes 1-2
        length = ((data[pos + 1] & 0x03) << 8) | data[pos + 2]
        total_length = 3 + length + 3  # header + payload + CRC
        
        if len(data) < pos + total_length:
            return None, pos  # Incomplete message
            
        if len(data) < pos + 5:
            return None, pos + 1  # Not enough data for message type
            
        # Extract message type (12 bits) from bytes 3-4
        message_type = ((data[pos + 3] & 0x0F) << 8) | data[pos + 4]
        return message_type, pos + total_length
    except:
        return None, pos + 1

def report_message_rates(message_counts, elapsed_time):
    """Report message rates for all detected message types."""
    if elapsed_time <= 0:
        return
        
    print(f"\n=== Message Rates Report (over {elapsed_time:.1f}s) ===")
    
    # Group messages by type
    nmea_messages = {k: v for k, v in message_counts.items() if isinstance(k, str) and len(k) == 5}
    ubx_messages = {k: v for k, v in message_counts.items() if isinstance(k, str) and '-' in k}
    rtcm_messages = {k: v for k, v in message_counts.items() if isinstance(k, int)}
    
    total_count = 0
    
    if nmea_messages:
        print("NMEA Messages:")
        for msg_type, count in sorted(nmea_messages.items()):
            rate = count / elapsed_time
            total_count += count
            print(f"  {msg_type}: {rate:.2f} Hz ({count} messages)")
    
    if ubx_messages:
        print("UBX Messages:")
        for msg_type, count in sorted(ubx_messages.items()):
            rate = count / elapsed_time
            total_count += count
            print(f"  {msg_type}: {rate:.2f} Hz ({count} messages)")
    
    if rtcm_messages:
        print("RTCM Messages:")
        for msg_type, count in sorted(rtcm_messages.items()):
            rate = count / elapsed_time
            total_count += count
            print(f"  Type {msg_type}: {rate:.2f} Hz ({count} messages)")
    
    if total_count > 0:
        total_rate = total_count / elapsed_time
        print(f"Total: {total_rate:.2f} Hz ({total_count} messages)")
    else:
        print("No messages detected in this interval")
    print("=" * 50)

print("Listening for GPS data on /dev/ttyACM0 with message rate monitoring...")
print("Press Ctrl+C to stop")

# Message rate monitoring variables
message_counts = {}
last_rate_check = time.time()
data_buffer = b''

try:
    with open('gps_log.txt', 'wb') as log_file:
        while True:
            if ser.in_waiting > 0:
                new_data = ser.read(ser.in_waiting)
                log_file.write(new_data)
                log_file.flush()
                
                # Add new data to buffer for parsing
                data_buffer += new_data
                
                # Parse messages from the buffer
                pos = 0
                while pos < len(data_buffer):
                    original_pos = pos
                    message_detected = False
                    
                    # Check for NMEA message
                    if pos < len(data_buffer) and data_buffer[pos:pos+1] == b'$':
                        msg_type, new_pos = parse_nmea_message_type(data_buffer, pos)
                        if msg_type:
                            message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
                            pos = new_pos
                            message_detected = True
                    
                    # Check for UBX message
                    elif pos < len(data_buffer) - 1 and data_buffer[pos:pos+2] == b'\xB5\x62':
                        msg_type, new_pos = parse_ubx_message_type(data_buffer, pos)
                        if msg_type:
                            message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
                            pos = new_pos
                            message_detected = True
                    
                    # Check for RTCM message
                    elif pos < len(data_buffer) and data_buffer[pos:pos+1] == b'\xD3':
                        msg_type, new_pos = parse_rtcm_message_type(data_buffer, pos)
                        if msg_type:
                            message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
                            pos = new_pos
                            message_detected = True
                    
                    # If no message detected, advance by 1 byte
                    if not message_detected:
                        pos += 1
                    
                    # Prevent infinite loops
                    if pos <= original_pos:
                        pos = original_pos + 1
                
                # Keep only the last 10KB in buffer to prevent memory issues
                if len(data_buffer) > 10240:
                    data_buffer = data_buffer[-5120:]  # Keep last 5KB
                
                # Report message rates periodically
                current_time = time.time()
                if current_time - last_rate_check >= RATE_REPORT_INTERVAL:
                    elapsed = current_time - last_rate_check
                    report_message_rates(message_counts, elapsed)
                    message_counts.clear()
                    last_rate_check = current_time
                
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage

except KeyboardInterrupt:
    print("\nStopped by user")
    
    # Final report
    current_time = time.time()
    elapsed = current_time - last_rate_check
    if elapsed > 0 and message_counts:
        print("\nFinal message rates:")
        report_message_rates(message_counts, elapsed)
    
    ser.close()

except serial.SerialException as e:
    print(f"Serial error: {e}")
    ser.close()