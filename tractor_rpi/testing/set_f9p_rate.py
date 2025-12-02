#!/usr/bin/env python3
"""
Set F9P Measurement Period
Configure the measurement rate on a u-blox F9P receiver.
"""

import serial
import struct
import time
import sys
import argparse


def set_measurement_period(port='/dev/gps-base-link', baudrate=115200, 
                          period_ms=100, nav_rate=1, save_to_flash=False):
    """Set the F9P measurement period.
    
    Args:
        port: Serial port path
        baudrate: Serial baud rate
        period_ms: Measurement period in milliseconds (50=20Hz, 100=10Hz, 200=5Hz)
        nav_rate: Navigation rate (1 = one solution per measurement)
        save_to_flash: If True, save configuration permanently
    """
    
    try:
        # Connect to F9P
        ser = serial.Serial(port, baudrate, timeout=2)
        print(f"Connected to {port}")
        time.sleep(0.5)  # Give device time to stabilize
        
        # Calculate frequency
        freq_hz = 1000.0 / period_ms
        print(f"Setting measurement period to {period_ms} ms ({freq_hz:.1f} Hz)")
        
        # Build UBX-CFG-RATE set message
        # Payload: measRate(2) + navRate(2) + timeRef(2)
        time_ref = 1  # 1 = GPS time
        payload = struct.pack('<HHH', period_ms, nav_rate, time_ref)
        
        # Build complete message
        msg = bytearray([0xB5, 0x62,  # Sync
                        0x06, 0x08,   # CFG-RATE
                        0x06, 0x00])  # Length = 6
        msg.extend(payload)
        
        # Calculate checksum
        ck_a = ck_b = 0
        for byte in msg[2:]:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        msg.extend([ck_a, ck_b])
        
        # Send configuration
        ser.write(msg)
        print("Configuration sent")
        time.sleep(0.5)
        
        # Verify by polling CFG-RATE

        # === VERIFY CONFIGURATION (WITH RETRY) ===
        print("\nVerifying configuration...")
        verified = False
        poll_msg = bytearray([0xB5, 0x62, 0x06, 0x08, 0x00, 0x00])
        ck_a = ck_b = 0
        for byte in poll_msg[2:]:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        poll_msg.extend([ck_a, ck_b])

        for attempt in range(3):
            ser.write(poll_msg)
            time.sleep(0.8)  # Give F9P time to respond
            response = ser.read(512)
            
            for i in range(len(response) - 7):
                if (response[i:i+4] == b'\xb5\x62\x06\x08' and 
                    len(response) >= i + 14):
                    
                    payload_start = i + 6
                    actual_period = struct.unpack('<H', response[payload_start:payload_start+2])[0]
                    actual_nav = struct.unpack('<H', response[payload_start+2:payload_start+4])[0]
                    
                    if actual_period == period_ms and actual_nav == nav_rate:
                        print(f"Configuration verified: {actual_period} ms @ {1000.0/actual_period:.1f} Hz")
                        verified = True
                    else:
                        print(f"Configuration mismatch on attempt {attempt + 1}!")
                        print(f"  Requested: {period_ms} ms, nav_rate={nav_rate}")
                        print(f"  Actual: {actual_period} ms, nav_rate={actual_nav}")
                    break  # Exit inner loop
            if verified:
                break
            else:
                print(f"  Retry {attempt + 2}/3...")
        
        if not verified:
            print("Could not verify configuration — but it may still have applied.")
            print("  Run check_f9p_rate.py to confirm.")

        # === SAVE TO FLASH (CORRECTED & SINGLE SEND) ===
        if save_to_flash and verified:
            print("\nSaving configuration to flash memory...")
            
            save_payload = struct.pack('<III', 0x0000061F, 0x00000000, 0x00000000)
            save_msg = bytearray([
                0xB5, 0x62,        # Sync
                0x06, 0x09,        # CFG-CFG
                0x0D, 0x00         # Length = 13
            ])
            save_msg.extend(save_payload)
            save_msg.append(0x04)  # Flash device

            # Checksum
            ck_a = ck_b = 0
            for byte in save_msg[2:]:
                ck_a = (ck_a + byte) & 0xFF
                ck_b = (ck_b + ck_a) & 0xFF
            save_msg.extend([ck_a, ck_b])

            ser.write(save_msg)
            time.sleep(2.0)  # Flash write takes time
            print("Configuration saved to flash")
            print("  Settings will persist after power cycle")
        
        ser.close()
        return verified
        
    except serial.SerialException as e:
        print(f"ERROR: Could not open port {port}")
        print(f"  {e}")
        return False
    except Exception as e:
        print(f"ERROR: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Set F9P measurement period',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Common configurations:
  20 Hz:  --period 50
  10 Hz:  --period 100  
  5 Hz:   --period 200
  1 Hz:   --period 1000
  
Examples:
  %(prog)s --period 100           # Set to 10 Hz
  %(prog)s --period 50 --save     # Set to 20 Hz and save permanently
  %(prog)s --port /dev/ttyUSB0 --period 200  # Set specific port to 5 Hz
        """
    )
    
    parser.add_argument('--port', default='/dev/gps-base-link',
                       help='Serial port (default: /dev/gps-base-link)')
    parser.add_argument('--baud', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('--period', type=int, required=True,
                       help='Measurement period in ms (50=20Hz, 100=10Hz)')
    parser.add_argument('--nav-rate', type=int, default=1,
                       help='Navigation rate (default: 1)')
    parser.add_argument('--save', action='store_true',
                       help='Save configuration to flash memory')
    
    args = parser.parse_args()
    
    # Validate period
    freq = 1000.0 / args.period
    if freq > 40:
        print(f"WARNING: {freq:.1f} Hz is very high. F9P max is typically 20-25 Hz")
        response = input("Continue anyway? (y/n): ")
        if response.lower() != 'y':
            sys.exit(0)
    
    # Apply configuration
    success = set_measurement_period(
        args.port,
        args.baud, 
        args.period,
        args.nav_rate,
        args.save
    )
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
