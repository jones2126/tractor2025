#!/usr/bin/env python3
"""
f9p_message_rates_query.py
==========================
Query the F9P to see which specific messages are configured and their output rates
for different ports (UART1, UART2, USB, SPI, I2C, Reserved).

Purpose: "What specific messages are being sent?"
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
    message = b'\xB5\x62'
    message += struct.pack('BB', msg_class, msg_id)
    message += struct.pack('<H', len(payload))
    message += payload
    ck_a, ck_b = calculate_ubx_checksum(msg_class, msg_id, payload)
    message += struct.pack('BB', ck_a, ck_b)
    return message

def parse_ubx_response(data, expected_class, expected_id):
    """Parse UBX response looking for specific message class/ID."""
    pos = 0
    while pos < len(data):
        if pos + 1 < len(data) and data[pos:pos+2] == b'\xB5\x62':
            if pos + 6 <= len(data):
                msg_class = data[pos + 2]
                msg_id = data[pos + 3]
                length = struct.unpack('<H', data[pos + 4:pos + 6])[0]
                
                if pos + 6 + length + 2 <= len(data):
                    if msg_class == expected_class and msg_id == expected_id:
                        payload = data[pos + 6:pos + 6 + length]
                        return payload
                    pos += 6 + length + 2
                else:
                    break
            else:
                break
        else:
            pos += 1
    return None

def query_message_config(ser, msg_class, msg_id, timeout=2):
    """Query configuration for a specific message type."""
    # Create CFG-MSG poll request
    payload = struct.pack('BB', msg_class, msg_id)
    cfg_msg_poll = create_ubx_message(0x06, 0x01, payload)
    
    # Clear buffer and send request
    ser.reset_input_buffer()
    ser.write(cfg_msg_poll)
    ser.flush()
    
    # Wait for response
    response_data = b''
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            new_data = ser.read(ser.in_waiting)
            response_data += new_data
            
            # Look for CFG-MSG response (0x06, 0x01)
            response_payload = parse_ubx_response(response_data, 0x06, 0x01)
            if response_payload and len(response_payload) >= 8:
                # Parse response: msgClass, msgID, rate[6 ports]
                resp_class = response_payload[0]
                resp_id = response_payload[1]
                rates = list(response_payload[2:8])  # UART1, UART2, USB, SPI, I2C, Reserved
                
                if resp_class == msg_class and resp_id == msg_id:
                    return rates
        
        time.sleep(0.01)
    
    return None

# Define message types to query
MESSAGE_TYPES = {
    # NMEA Messages
    'NMEA': {
        (0xF0, 0x00): 'GGA - Global positioning system fix data',
        (0xF0, 0x01): 'GLL - Geographic position',
        (0xF0, 0x02): 'GSA - GNSS DOP and active satellites',
        (0xF0, 0x03): 'GSV - GNSS satellites in view',
        (0xF0, 0x04): 'RMC - Recommended minimum',
        (0xF0, 0x05): 'VTG - Track made good and ground speed',
        (0xF0, 0x08): 'ZDA - Time and date',
        (0xF0, 0x09): 'GBS - GNSS satellite fault detection',
        (0xF0, 0x0A): 'DTM - Datum reference',
        (0xF0, 0x0D): 'GNS - GNSS fix data',
        (0xF0, 0x0E): 'THS - True heading and status',
        (0xF0, 0x0F): 'VLW - Dual ground/water distance',
    },
    # UBX Navigation Messages
    'UBX-NAV': {
        (0x01, 0x07): 'NAV-PVT - Navigation position velocity time solution',
        (0x01, 0x14): 'NAV-HPPOSECEF - High precision position ECEF',
        (0x01, 0x15): 'NAV-HPPOSLLH - High precision position LLH',
        (0x01, 0x3B): 'NAV-RELPOSNED - Relative positioning information in NED',
        (0x01, 0x3C): 'NAV-RELPOSNED9 - Relative positioning information in NED',
        (0x01, 0x35): 'NAV-SAT - Satellite information',
        (0x01, 0x03): 'NAV-STATUS - Receiver navigation status',
        (0x01, 0x21): 'NAV-TIMEUTC - UTC time solution',
        (0x01, 0x20): 'NAV-TIMEGPS - GPS time solution',
        (0x01, 0x23): 'NAV-TIMEGLO - GLONASS time solution',
        (0x01, 0x25): 'NAV-TIMEGAL - Galileo time solution',
        (0x01, 0x24): 'NAV-TIMEBDS - BeiDou time solution',
    },
    # UBX Raw Measurement Messages
    'UBX-RXM': {
        (0x02, 0x15): 'RXM-RAWX - Multi-GNSS raw measurement data',
        (0x02, 0x13): 'RXM-SFRBX - Broadcast navigation data subframe',
        (0x02, 0x59): 'RXM-RLM - Galileo SAR short-RLM report',
        (0x02, 0x14): 'RXM-MEASX - Satellite measurements for RRLP',
    },
    # RTCM Messages
    'RTCM3': {
        (0xF5, 0x05): 'RTCM3.3 - Type 1005 - Stationary RTK reference ARP',
        (0xF5, 0x4A): 'RTCM3.3 - Type 1074 - GPS MSM4',
        (0xF5, 0x54): 'RTCM3.3 - Type 1084 - GLONASS MSM4',
        (0xF5, 0x5E): 'RTCM3.3 - Type 1094 - Galileo MSM4',
        (0xF5, 0x68): 'RTCM3.3 - Type 1104 - BeiDou MSM4',
        (0xF5, 0x7C): 'RTCM3.3 - Type 1124 - BeiDou MSM4',
        (0xF5, 0xE6): 'RTCM3.3 - Type 1230 - GLONASS code-phase biases',
    }
}

PORT_NAMES = ['UART1', 'UART2', 'USB', 'SPI', 'I2C', 'Reserved']

def query_f9p_message_config():
    """Query F9P message configuration."""
    try:
        ser = serial.Serial('/dev/ttyACM2', 115200, timeout=5)
        print("Connected to F9P on /dev/ttyACM2")
        print("\nQuerying message configurations...\n")
        
        active_messages = []
        
        for category, messages in MESSAGE_TYPES.items():
            print(f"\n{'='*60}")
            print(f"{category} MESSAGES")
            print('='*60)
            
            category_has_active = False
            
            for (msg_class, msg_id), description in messages.items():
                rates = query_message_config(ser, msg_class, msg_id)
                
                if rates is not None:
                    # Check if any port has a non-zero rate
                    if any(rate > 0 for rate in rates):
                        category_has_active = True
                        active_messages.append({
                            'category': category,
                            'class': msg_class,
                            'id': msg_id,
                            'description': description,
                            'rates': rates
                        })
                        
                        print(f"\n✓ {description}")
                        print(f"   Class: 0x{msg_class:02X}, ID: 0x{msg_id:02X}")
                        
                        rate_info = []
                        for i, rate in enumerate(rates):
                            if rate > 0:
                                if rate == 1:
                                    rate_info.append(f"{PORT_NAMES[i]}: Every solution")
                                else:
                                    rate_info.append(f"{PORT_NAMES[i]}: Every {rate} solutions")
                        
                        if rate_info:
                            print(f"   Rates: {', '.join(rate_info)}")
                else:
                    print(f"✗ {description} - No response")
            
            if not category_has_active:
                print(f"   No active {category} messages configured")
        
        # Summary
        print(f"\n{'='*60}")
        print("SUMMARY")
        print('='*60)
        
        if active_messages:
            print(f"Total active messages: {len(active_messages)}")
            
            # Group by port
            port_summary = {port: [] for port in PORT_NAMES}
            for msg in active_messages:
                for i, rate in enumerate(msg['rates']):
                    if rate > 0:
                        port_summary[PORT_NAMES[i]].append({
                            'description': msg['description'],
                            'rate': rate
                        })
            
            for port, messages in port_summary.items():
                if messages:
                    print(f"\n{port}:")
                    for msg in messages:
                        rate_text = "every solution" if msg['rate'] == 1 else f"every {msg['rate']} solutions"
                        print(f"  • {msg['description']} ({rate_text})")
        else:
            print("No active messages found")
        
        ser.close()
        return True
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    print("F9P Message Rates Query")
    print("What specific messages are being sent?")
    print("-" * 40)
    
    success = query_f9p_message_config()
    
    if not success:
        print("\nTroubleshooting tips:")
        print("1. Check that F9P is connected to /dev/ttyACM?")
        print("2. Ensure no other processes are using the serial port")
        print("3. Verify the F9P is powered on and responding")
        print("4. Some message queries may timeout - this is normal")
