#!/usr/bin/env python3
"""
Quick F9P Measurement Period Check
Simply checks if the F9P is configured for a specific measurement rate.
"""

import serial
import struct
import sys

def check_measurement_period(port='/dev/gps-base-link', baudrate=115200):
    """Quick check of F9P measurement period."""
    
    try:
        # Connect to F9P
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port}")
        
        # Build UBX-CFG-RATE poll message
        # Sync(2) + Class(1) + ID(1) + Length(2) + Checksum(2)
        msg = bytearray([0xB5, 0x62,  # Sync
                        0x06, 0x08,   # CFG-RATE
                        0x00, 0x00])  # Length = 0 (poll)
        
        # Calculate checksum
        ck_a = ck_b = 0
        for byte in msg[2:]:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        msg.extend([ck_a, ck_b])
        
        # Send poll request
        ser.write(msg)
        
        # Read response (up to 256 bytes should be enough)
        response = ser.read(256)
        
        # Find UBX-CFG-RATE response in the data
        for i in range(len(response) - 7):
            if (response[i:i+4] == b'\xb5\x62\x06\x08' and 
                len(response) >= i + 14):  # Need full message
                
                # Extract measurement rate (first 2 bytes of payload)
                payload_start = i + 6
                meas_rate_ms = struct.unpack('<H', response[payload_start:payload_start+2])[0]
                nav_rate = struct.unpack('<H', response[payload_start+2:payload_start+4])[0]
                
                # Calculate frequency
                freq_hz = 1000.0 / meas_rate_ms if meas_rate_ms > 0 else 0
                
                # Display results
                print("\n" + "="*50)
                print(f"MEASUREMENT PERIOD: {meas_rate_ms} ms")
                print(f"FREQUENCY: {freq_hz:.1f} Hz")
                print(f"Navigation Rate: {nav_rate}")
                print("="*50)
                
                # Check common configurations
                if meas_rate_ms == 50:
                    print("✓ Configured for 20 Hz")
                elif meas_rate_ms == 100:
                    print("✓ Configured for 10 Hz")
                elif meas_rate_ms == 200:
                    print("✓ Configured for 5 Hz")
                elif meas_rate_ms == 1000:
                    print("✓ Configured for 1 Hz")
                else:
                    print(f"⚠ Non-standard configuration")
                
                if nav_rate != 1:
                    print(f"⚠ Navigation rate is {nav_rate} (not 1:1)")
                    print(f"  Actual output rate: {freq_hz/nav_rate:.1f} Hz")
                
                ser.close()
                return meas_rate_ms
        
        print("ERROR: No valid CFG-RATE response received")
        print("Possible causes:")
        print("  - F9P not responding on this port")
        print("  - Different baud rate needed")
        print("  - Port permissions issue")
        ser.close()
        return None
        
    except serial.SerialException as e:
        print(f"ERROR: Could not open port {port}")
        print(f"  {e}")
        print("\nTry:")
        print(f"  - Check if port exists: ls -la {port}")
        print(f"  - Check permissions: sudo chmod 666 {port}")
        print(f"  - Verify it's not in use: lsof {port}")
        return None
    except Exception as e:
        print(f"ERROR: {e}")
        return None


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Check F9P measurement period')
    parser.add_argument('--port', default='/dev/gps-base-link',
                       help='Serial port (default: /dev/gps-base-link)')
    parser.add_argument('--baud', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    
    args = parser.parse_args()
    
    result = check_measurement_period(args.port, args.baud)
    
    # Exit with status code
    sys.exit(0 if result else 1)
