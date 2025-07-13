import serial
import time
import struct
from datetime import datetime
import asyncio
import websockets
import threading
import json
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Configure the serial port
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# WebSocket configuration
WEBSOCKET_HOST = "0.0.0.0"
WEBSOCKET_PORT = 8765
RTCM_PUBLISH_RATE = 1.0  # 1 Hz

# Message rate monitoring configuration
RATE_REPORT_INTERVAL = 10  # Report message rates every 10 seconds

# Shared variables for WebSocket
connected_clients = set()
rtcm_message_buffer = []
rtcm_buffer_lock = threading.Lock()

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
        if len(nmea_sentence) >= 6 and nmea_sentence.startswith('$'):
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

def calculate_rtcm_crc(data):
    """Calculate the 24-bit CRC for an RTCM message."""
    crc = 0
    for i in range(len(data)):
        crc ^= (data[i] << 16)
        for j in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        crc &= 0xFFFFFF
    return crc

def parse_rtcm_message_type(data, pos):
    """
    Extract RTCM message type from data starting at pos.
    Returns (message_type, message_end_pos, rtcm_message) or (None, pos, None)
    """
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
        
        # Verify CRC
        calculated_crc = calculate_rtcm_crc(rtcm_message[:-3])
        crc_bytes = rtcm_message[-3:]
        received_crc = struct.unpack('>I', b'\x00' + crc_bytes)[0]
        
        if calculated_crc != received_crc:
            logger.warning(f"RTCM CRC mismatch at pos {pos}")
            return None, pos + 1, None
            
        # Extract message type (12 bits) from the first 12 bits of the payload
        # The message type is stored in bits 0-11 of the payload
        byte3 = data[pos + 3]
        byte4 = data[pos + 4] if pos + 4 < len(data) else 0
        
        # Message type is the first 12 bits of the payload
        message_type = (byte3 << 4) | (byte4 >> 4)
        
        return message_type, pos + total_length, rtcm_message
    except Exception as e:
        logger.error(f"Error parsing RTCM message: {e}")
        return None, pos + 1, None

def add_rtcm_to_buffer(rtcm_message):
    """Add RTCM message to buffer for WebSocket publishing."""
    with rtcm_buffer_lock:
        rtcm_message_buffer.append({
            'timestamp': time.time(),
            'data': rtcm_message.hex(),
            'length': len(rtcm_message)
        })
        # Keep only the last 100 messages to prevent memory issues
        if len(rtcm_message_buffer) > 100:
            rtcm_message_buffer.pop(0)

async def websocket_handler(websocket, path):
    """Handle WebSocket connections."""
    logger.info(f"WebSocket client connected from {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        await websocket.wait_closed()
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected_clients.discard(websocket)
        logger.info(f"WebSocket client disconnected from {websocket.remote_address}")

async def publish_rtcm_messages():
    """Publish RTCM messages to WebSocket clients at specified rate."""
    last_publish_time = 0
    
    while True:
        current_time = time.time()
        
        # Check if it's time to publish (1 Hz)
        if current_time - last_publish_time >= (1.0 / RTCM_PUBLISH_RATE):
            if connected_clients and rtcm_message_buffer:
                with rtcm_buffer_lock:
                    # Get the most recent RTCM messages
                    messages_to_send = rtcm_message_buffer.copy()
                    rtcm_message_buffer.clear()
                
                if messages_to_send:
                    # Create WebSocket message
                    ws_message = {
                        'type': 'rtcm_data',
                        'timestamp': current_time,
                        'message_count': len(messages_to_send),
                        'messages': messages_to_send
                    }
                    
                    # Send to all connected clients
                    disconnected_clients = set()
                    for client in connected_clients:
                        try:
                            await client.send(json.dumps(ws_message))
                        except websockets.exceptions.ConnectionClosed:
                            disconnected_clients.add(client)
                        except Exception as e:
                            logger.error(f"Error sending to WebSocket client: {e}")
                            disconnected_clients.add(client)
                    
                    # Remove disconnected clients
                    connected_clients.difference_update(disconnected_clients)
                    
                    if connected_clients:
                        logger.info(f"Published {len(messages_to_send)} RTCM messages to {len(connected_clients)} clients")
                
                last_publish_time = current_time
        
        await asyncio.sleep(0.1)  # Small delay to prevent busy loop

async def start_websocket_server():
    """Start the WebSocket server."""
    logger.info(f"Starting WebSocket server on {WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
    
    # Start the RTCM publishing task
    publish_task = asyncio.create_task(publish_rtcm_messages())
    
    # Start the WebSocket server
    async with websockets.serve(websocket_handler, WEBSOCKET_HOST, WEBSOCKET_PORT):
        logger.info(f"WebSocket server listening on ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
        await publish_task

def run_websocket_server():
    """Run the WebSocket server in a separate thread."""
    asyncio.run(start_websocket_server())

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
    
    # Report WebSocket status
    print(f"WebSocket clients connected: {len(connected_clients)}")
    print("=" * 50)

print("Starting GPS data logger with WebSocket RTCM publishing...")
print(f"WebSocket server will be available at ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
print("Press Ctrl+C to stop")

# Start WebSocket server in a separate thread
websocket_thread = threading.Thread(target=run_websocket_server, daemon=True)
websocket_thread.start()

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
                        msg_type, new_pos, rtcm_message = parse_rtcm_message_type(data_buffer, pos)
                        if msg_type and rtcm_message:
                            message_counts[f"RTCM_{msg_type}"] = message_counts.get(f"RTCM_{msg_type}", 0) + 1
                            # Add RTCM message to WebSocket buffer
                            add_rtcm_to_buffer(rtcm_message)
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
