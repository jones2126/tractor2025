import serial
import logging
import time
from pyubx2 import UBXMessage

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

PORT = "/dev/f9p"
BAUDRATE = 115200
TIMEOUT = 1
MONITOR_DURATION = 20  # seconds - increased to capture all message types

# RTCM messages needed for RTK base station
RTCM_MESSAGES = [
    {"msg_id": 1005, "ubx_id": 0x05, "description": "Stationary RTK reference station ARP"},
    {"msg_id": 1074, "ubx_id": 0x4A, "description": "GPS MSM4"},
    {"msg_id": 1084, "ubx_id": 0x54, "description": "GLONASS MSM4"},
    {"msg_id": 1094, "ubx_id": 0x5E, "description": "Galileo MSM4"}, 
    {"msg_id": 1230, "ubx_id": 0xE6, "description": "GLONASS L1 and L2 code-phase biases"},
]

def enable_rtcm_messages(ser):
    """Enable all required RTCM messages for RTK base station"""
    
    for msg in RTCM_MESSAGES:
        try:
            payload_bytes = bytes([
                0xF5,           # msgClass = RTCM-3X (0xF5)
                msg["ubx_id"],  # msgID (UBX mapping for RTCM message)
                0,              # rateDDC (I2C)
                0,              # rateUART1
                0,              # rateUART2
                1,              # rateUSB (enable on USB with rate 1)
                0,              # rateSPI
                0               # reserved
            ])
            
            ubx_msg = UBXMessage("CFG", "CFG-MSG", msgmode=0, payload=payload_bytes)
            ser.write(ubx_msg.serialize())
            logging.info(f"Enabled RTCM {msg['msg_id']} (ID=0x{msg['ubx_id']:02X}) - {msg['description']}")
            time.sleep(0.1)
            
        except Exception as e:
            logging.error(f"Failed to enable RTCM {msg['msg_id']}: {e}")

def enable_nmea_gga(ser):
    """Enable NMEA GGA message for base station position tracking"""
    try:
        payload_bytes = bytes([
            0xF0,  # msgClass = NMEA (0xF0)
            0x00,  # msgID = GGA (0x00)
            0,     # rateDDC (I2C)
            0,     # rateUART1
            0,     # rateUART2
            1,     # rateUSB (enable on USB)
            0,     # rateSPI
            0      # reserved
        ])
        
        ubx_msg = UBXMessage("CFG", "CFG-MSG", msgmode=0, payload=payload_bytes)
        ser.write(ubx_msg.serialize())
        logging.info("Enabled NMEA GGA message on USB")
        time.sleep(0.1)
        
    except Exception as e:
        logging.error(f"Failed to enable NMEA GGA: {e}")

def disable_nav_svin(ser):
    """Disable NAV-SVIN messages to clean up the data stream"""
    try:
        payload_bytes = bytes([
            0x01,  # msgClass = NAV (0x01)
            0x3B,  # msgID = SVIN (0x3B)
            0, 0, 0, 0, 0, 0  # disable on all ports
        ])
        
        ubx_msg = UBXMessage("CFG", "CFG-MSG", msgmode=0, payload=payload_bytes)
        ser.write(ubx_msg.serialize())
        logging.info("Disabled NAV-SVIN messages to clean data stream")
        time.sleep(0.1)
        
    except Exception as e:
        logging.error(f"Failed to disable NAV-SVIN: {e}")

def monitor_output(ser, duration):
    """Monitor both RTCM and NMEA output with proper separation"""
    logging.info(f"Monitoring RTCM and NMEA output for {duration}s...")
    start_time = time.time()
    
    rtcm_found = set()
    nmea_found = set()
    ubx_found = set()
    rtcm_frame_count = 0
    nmea_line_count = 0
    ubx_frame_count = 0
    total_bytes = 0

    buffer = bytearray()
    
    while time.time() - start_time < duration:
        data = ser.read(1024)
        if data:
            total_bytes += len(data)
            buffer.extend(data)
            
            # Process buffer for different message types
            i = 0
            while i < len(buffer):
                
                # Check for NMEA message (starts with $)
                if buffer[i] == 36:  # ASCII value for '$'
                    # Look for end of line
                    end_idx = i + 1
                    while end_idx < len(buffer) and buffer[end_idx] not in [13, 10]:  # \r, \n
                        end_idx += 1
                    
                    if end_idx < len(buffer):  # Found complete line
                        try:
                            line = buffer[i:end_idx].decode('ascii')
                            if 'GGA' in line:
                                nmea_found.add('GGA')
                                nmea_line_count += 1
                                if nmea_line_count <= 3:  # Only log first few
                                    logging.info(f"Detected NMEA GGA (line #{nmea_line_count})")
                        except:
                            pass
                        
                        # Skip past line ending
                        while end_idx < len(buffer) and buffer[end_idx] in [13, 10]:
                            end_idx += 1
                        i = end_idx
                        continue
                
                # Check for UBX message (0xB5, 0x62)
                elif buffer[i] == 0xB5 and i + 1 < len(buffer) and buffer[i + 1] == 0x62:
                    if i + 6 <= len(buffer):  # Minimum UBX header
                        length = (buffer[i + 5] << 8) | buffer[i + 4]
                        frame_len = 6 + length + 2  # header + payload + checksum
                        
                        if i + frame_len <= len(buffer):  # Complete UBX frame
                            ubx_found.add("UBX")
                            ubx_frame_count += 1
                            i += frame_len
                            continue
                
                # Check for RTCM message (0xD3)
                elif buffer[i] == 0xD3:
                    if i + 3 <= len(buffer):
                        # Extract length (10 bits from bytes 1-2)
                        length = ((buffer[i + 1] & 0x03) << 8) | buffer[i + 2]
                        frame_len = 3 + length + 3  # header + payload + CRC
                        
                        if i + frame_len <= len(buffer):  # Complete RTCM frame
                            # Extract message number from payload
                            if length >= 2:
                                payload_start = i + 3
                                # Message number is in first 12 bits of payload
                                byte0 = buffer[payload_start]
                                byte1 = buffer[payload_start + 1]
                                msg_num = ((byte0 << 4) | (byte1 >> 4)) & 0xFFF  # 12 bits
                                
                                if msg_num not in rtcm_found:  # Only log first occurrence
                                    logging.info(f"First RTCM message {msg_num} detected")
                                
                                rtcm_found.add(msg_num)
                                rtcm_frame_count += 1
                            
                            i += frame_len
                            continue
                
                # If no message type matched, advance by 1
                i += 1

            # Keep unprocessed data in buffer
            buffer = buffer[i:]

    # Summary
    logging.info(f"\n{'='*50}")
    logging.info(f"MONITORING SUMMARY")
    logging.info(f"{'='*50}")
    logging.info(f"Total bytes received: {total_bytes}")
    logging.info(f"RTCM frames processed: {rtcm_frame_count}")
    logging.info(f"NMEA lines processed: {nmea_line_count}")
    logging.info(f"UBX frames processed: {ubx_frame_count}")
    
    if rtcm_found:
        logging.info(f"RTCM messages detected: {', '.join(str(m) for m in sorted(rtcm_found))}")
        
        # Check if we got all required RTCM messages
        required_rtcm = {msg["msg_id"] for msg in RTCM_MESSAGES}
        missing_rtcm = required_rtcm - rtcm_found
        if missing_rtcm:
            logging.warning(f"Missing RTCM messages: {', '.join(str(m) for m in sorted(missing_rtcm))}")
        else:
            logging.info("All required RTCM messages are being output!")
    else:
        logging.warning("No RTCM messages detected")
        
    if nmea_found:
        logging.info(f"NMEA messages detected: {', '.join(sorted(nmea_found))}")
    else:
        logging.warning("No NMEA messages detected")
    
    if ubx_found:
        logging.info(f"UBX messages also detected (Survey-In status)")

def main():
    logging.info(f"Connecting to F9P on {PORT} @ {BAUDRATE}")
    with serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT) as ser:
        # Clear any existing data
        ser.flushInput()
        
        # Disable NAV-SVIN messages first to clean up data stream
        logging.info("Disabling NAV-SVIN messages...")
        disable_nav_svin(ser)
        
        logging.info("Enabling RTCM messages for RTK base station...")
        enable_rtcm_messages(ser)
        
        logging.info("Enabling NMEA GGA message...")
        enable_nmea_gga(ser)
        
        time.sleep(3)  # Give device time to start outputting
        monitor_output(ser, MONITOR_DURATION)

if __name__ == "__main__":
    main()