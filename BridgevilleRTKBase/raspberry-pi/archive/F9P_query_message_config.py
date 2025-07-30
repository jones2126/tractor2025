#!/usr/bin/env python3
"""
F9P Port Configuration Investigation
===================================
Query additional port settings to understand the routing
"""

import serial
import struct
import time

def calculate_ubx_checksum(msg_class, msg_id, payload):
    ck_a = 0
    ck_b = 0
    ck_a = (ck_a + msg_class) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    ck_a = (ck_a + msg_id) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    
    payload_len = len(payload)
    ck_a = (ck_a + (payload_len & 0xFF)) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    ck_a = (ck_a + ((payload_len >> 8) & 0xFF)) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    
    return ck_a, ck_b

def create_ubx_message(msg_class, msg_id, payload=b''):
    message = b'\xB5\x62'
    message += struct.pack('BB', msg_class, msg_id)
    message += struct.pack('<H', len(payload))
    message += payload
    ck_a, ck_b = calculate_ubx_checksum(msg_class, msg_id, payload)
    message += struct.pack('BB', ck_a, ck_b)
    return message

def parse_ubx_response(data, expected_class, expected_id):
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

def query_port_config(ser, port_id, timeout=3):
    """Query CFG-PRT for a specific port."""
    payload = struct.pack('B', port_id)
    cfg_prt_poll = create_ubx_message(0x06, 0x00, payload)
    
    ser.reset_input_buffer()
    ser.write(cfg_prt_poll)
    ser.flush()
    
    response_data = b''
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            new_data = ser.read(ser.in_waiting)
            response_data += new_data
            
            response_payload = parse_ubx_response(response_data, 0x06, 0x00)
            if response_payload and len(response_payload) >= 20:
                return response_payload
        
        time.sleep(0.01)
    
    return None

def parse_port_config(payload):
    """Parse CFG-PRT response payload."""
    if len(payload) < 20:
        return None
    
    port_id = payload[0]
    reserved = payload[1]
    tx_ready = struct.unpack('<H', payload[2:4])[0]
    mode = struct.unpack('<I', payload[4:8])[0]
    baud_rate = struct.unpack('<I', payload[8:12])[0]
    in_proto_mask = struct.unpack('<H', payload[12:14])[0]
    out_proto_mask = struct.unpack('<H', payload[14:16])[0]
    flags = struct.unpack('<H', payload[16:18])[0]
    reserved2 = struct.unpack('<H', payload[18:20])[0]
    
    return {
        'port_id': port_id,
        'baud_rate': baud_rate,
        'in_proto_mask': in_proto_mask,
        'out_proto_mask': out_proto_mask,
        'flags': flags
    }

def investigate_f9p_ports():
    """Investigate F9P port configurations."""
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=5)
        print("Connected to F9P on /dev/ttyACM0")
        print("\nInvestigating port configurations...\n")
        
        # Port definitions
        ports = {
            0: 'I2C',
            1: 'UART1', 
            2: 'UART2',
            3: 'USB',
            4: 'SPI'
        }
        
        protocol_masks = {
            0x01: 'UBX',
            0x02: 'NMEA',
            0x04: 'RTCM2',
            0x20: 'RTCM3'
        }
        
        print("="*70)
        print("PORT CONFIGURATION DETAILS")
        print("="*70)
        
        for port_id, port_name in ports.items():
            config = query_port_config(ser, port_id)
            
            if config:
                parsed = parse_port_config(config)
                if parsed:
                    print(f"\n{port_name} (Port {port_id}):")
                    print(f"  Baud Rate: {parsed['baud_rate']}")
                    
                    # Decode input protocols
                    in_protocols = []
                    for mask, name in protocol_masks.items():
                        if parsed['in_proto_mask'] & mask:
                            in_protocols.append(name)
                    print(f"  Input Protocols: {', '.join(in_protocols) if in_protocols else 'None'}")
                    
                    # Decode output protocols  
                    out_protocols = []
                    for mask, name in protocol_masks.items():
                        if parsed['out_proto_mask'] & mask:
                            out_protocols.append(name)
                    print(f"  Output Protocols: {', '.join(out_protocols) if out_protocols else 'None'}")
                    
                    print(f"  Input Mask: 0x{parsed['in_proto_mask']:04X}")
                    print(f"  Output Mask: 0x{parsed['out_proto_mask']:04X}")
                    print(f"  Flags: 0x{parsed['flags']:04X}")
            else:
                print(f"\n{port_name} (Port {port_id}): No response")
        
        # Query CFG-USB specifically
        print(f"\n{'='*70}")
        print("USB-SPECIFIC CONFIGURATION")
        print("="*70)
        
        cfg_usb_poll = create_ubx_message(0x06, 0x1B)
        ser.reset_input_buffer()
        ser.write(cfg_usb_poll)
        ser.flush()
        
        response_data = b''
        start_time = time.time()
        
        while time.time() - start_time < 3:
            if ser.in_waiting > 0:
                new_data = ser.read(ser.in_waiting)
                response_data += new_data
                
                usb_payload = parse_ubx_response(response_data, 0x06, 0x1B)
                if usb_payload and len(usb_payload) >= 108:
                    vendor_id = struct.unpack('<H', usb_payload[0:2])[0]
                    product_id = struct.unpack('<H', usb_payload[2:4])[0]
                    power_consumption = struct.unpack('<H', usb_payload[6:8])[0]
                    flags = struct.unpack('<H', usb_payload[8:10])[0]
                    
                    print(f"USB Vendor ID: 0x{vendor_id:04X}")
                    print(f"USB Product ID: 0x{product_id:04X}")
                    print(f"Power Consumption: {power_consumption}mA")
                    print(f"USB Flags: 0x{flags:04X}")
                    
                    # Extract strings
                    vendor_str = usb_payload[10:42].rstrip(b'\x00').decode('ascii', errors='ignore')
                    product_str = usb_payload[42:74].rstrip(b'\x00').decode('ascii', errors='ignore')
                    serial_str = usb_payload[74:106].rstrip(b'\x00').decode('ascii', errors='ignore')
                    
                    print(f"Vendor String: '{vendor_str}'")
                    print(f"Product String: '{product_str}'")
                    print(f"Serial String: '{serial_str}'")
                    break
            time.sleep(0.01)
        
        ser.close()
        return True
        
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    print("F9P Port Configuration Investigation")
    print("-" * 40)
    
    investigate_f9p_ports()
    
    print(f"\n{'='*70}")
    print("ANALYSIS")
    print("="*70)
    print("This investigation should help explain why you're seeing")
    print("RTCM messages on USB when they're not configured for USB.")
    print("Look for:")
    print("• Protocol routing between ports")
    print("• Output protocol masks")
    print("• Any port multiplexing configuration")