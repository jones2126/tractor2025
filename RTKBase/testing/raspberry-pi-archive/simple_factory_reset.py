#!/usr/bin/env python3
"""
Simple F9P Factory Reset
========================
Performs a complete factory reset using multiple methods and baud rates.
"""

import serial
import struct
import time
import sys

class UBXMessage:
    def __init__(self):
        self.SYNC_CHAR1 = 0xB5
        self.SYNC_CHAR2 = 0x62
    
    def calculate_checksum(self, msg_class, msg_id, payload):
        ck_a = 0
        ck_b = 0
        
        ck_a = (ck_a + msg_class) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
        ck_a = (ck_a + msg_id) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
        
        length = len(payload)
        ck_a = (ck_a + (length & 0xFF)) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
        ck_a = (ck_a + ((length >> 8) & 0xFF)) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
        
        for byte in payload:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
            
        return ck_a, ck_b
    
    def create_message(self, msg_class, msg_id, payload):
        length = len(payload)
        ck_a, ck_b = self.calculate_checksum(msg_class, msg_id, payload)
        
        message = bytearray()
        message.append(self.SYNC_CHAR1)
        message.append(self.SYNC_CHAR2)
        message.append(msg_class)
        message.append(msg_id)
        message.extend(struct.pack('<H', length))
        message.extend(payload)
        message.append(ck_a)
        message.append(ck_b)
        
        return bytes(message)

def send_nmea_command(ser, command):
    """Send NMEA command"""
    if not command.startswith('$'):
        command = '$' + command
    if not command.endswith('\r\n'):
        command += '\r\n'
    
    try:
        ser.write(command.encode('ascii'))
        ser.flush()
        time.sleep(0.5)
        return True
    except:
        return False

def factory_reset_at_baudrate(port, baudrate):
    """Attempt factory reset at specific baud rate"""
    print(f"Attempting reset at {baudrate} baud...")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(1)
        
        ubx = UBXMessage()
        reset_attempted = False
        
        # Method 1: UBX CFG-CFG factory reset
        try:
            clear_mask = 0x1F1F  # Clear all sections
            save_mask = 0x0000   # Don't save anything
            load_mask = 0x1F1F   # Load all sections from default
            
            payload = struct.pack('<III', clear_mask, save_mask, load_mask)
            reset_message = ubx.create_message(0x06, 0x09, payload)
            
            ser.write(reset_message)
            ser.flush()
            print(f"  UBX factory reset sent at {baudrate}")
            reset_attempted = True
            time.sleep(2)
        except:
            pass
        
        # Method 2: NMEA factory reset
        try:
            send_nmea_command(ser, "PUBX,41,1,0007,0003,115200,0")
            print(f"  NMEA factory reset sent at {baudrate}")
            reset_attempted = True
            time.sleep(2)
        except:
            pass
        
        # Method 3: Alternative NMEA reset
        try:
            send_nmea_command(ser, "PUBX,41,1,0003,0001,115200,0")
            print(f"  Alternative NMEA reset sent at {baudrate}")
            reset_attempted = True
            time.sleep(2)
        except:
            pass
        
        ser.close()
        return reset_attempted
        
    except Exception as e:
        print(f"  Error at {baudrate}: {e}")
        return False

def test_for_response(port, baudrate, duration=5):
    """Test if device responds at given baud rate"""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(1)
        
        start_time = time.time()
        byte_count = 0
        
        while time.time() - start_time < duration:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                byte_count += len(data)
            time.sleep(0.1)
        
        ser.close()
        return byte_count > 0
        
    except:
        return False

def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else '/dev/f9p'
    
    print("F9P Simple Factory Reset")
    print("=" * 30)
    print(f"Port: {port}")
    
    # Common baud rates to try
    baud_rates = [115200, 38400, 9600, 19200, 57600]
    
    print("\nStep 1: Attempting factory reset at multiple baud rates...")
    
    reset_attempts = 0
    for baudrate in baud_rates:
        if factory_reset_at_baudrate(port, baudrate):
            reset_attempts += 1
    
    print(f"\nFactory reset attempts: {reset_attempts}")
    
    if reset_attempts > 0:
        print("\nStep 2: Waiting for device to restart...")
        time.sleep(10)  # Give device time to restart
        
        print("\nStep 3: Testing for device response...")
        
        # Test at common baud rates
        for baudrate in baud_rates:
            print(f"Testing {baudrate} baud...", end=' ')
            if test_for_response(port, baudrate, 3):
                print(f"RESPONSE DETECTED at {baudrate}!")
                print(f"\nSUCCESS: Device is responding at {baudrate} baud")
                print(f"Run diagnostic script with: python3 diagnose_f9p_status.py {port} {baudrate}")
                return
            else:
                print("no response")
        
        print("\nNo response detected at any baud rate.")
        print("The device may need more time or manual intervention.")
        
    else:
        print("\nCould not send reset commands.")
        print("Check:")
        print("  - Device is powered on")
        print("  - Correct serial port")
        print("  - Physical connections")
    
    print(f"\nNext steps:")
    print(f"1. Wait 1-2 minutes for device to fully restart")
    print(f"2. Try: python3 diagnose_f9p_status.py {port} 115200") 
    print(f"3. If still no response, try other baud rates: 9600, 38400")
    print(f"4. Power cycle the device if necessary")

if __name__ == "__main__":
    main()