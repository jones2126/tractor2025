import serial
import time
from collections import defaultdict
import threading
import re

class F9PConfigurator:
    def __init__(self, port='/dev/f9p', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.message_stats = defaultdict(int)
        self.monitoring = False
        
    def connect(self):
        """Connect to the F9P device"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False
    
    def send_ubx_command(self, msg_class, msg_id, payload=b''):
        """Send a UBX command with proper framing and checksum"""
        # UBX header
        header = b'\xb5\x62'
        
        # Message class and ID
        msg_class_id = bytes([msg_class, msg_id])
        
        # Length (little endian)
        length = len(payload).to_bytes(2, 'little')
        
        # Build message without checksum
        message = msg_class_id + length + payload
        
        # Calculate checksum
        ck_a, ck_b = 0, 0
        for byte in message:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        
        # Full message with header and checksum
        full_message = header + message + bytes([ck_a, ck_b])
        
        # Send message
        self.ser.write(full_message)
        print(f"Sent UBX command: class=0x{msg_class:02X}, id=0x{msg_id:02X}")
    
    def factory_reset(self):
        """Perform factory reset using UBX-CFG-CFG command"""
        # Payload: clearMask, saveMask, loadMask, deviceMask
        # Clear all configurations, save to persistent storage
        payload = b'\x1f\x00\x00\x00'  # Clear all
        payload += b'\x1f\x00\x00\x00'  # Save all
        payload += b'\x00\x00\x00\x00'  # Load nothing
        payload += b'\x07\x00\x00\x00'  # Apply to all devices (BBR, Flash, I2C-EEPROM)
        
        self.send_ubx_command(0x06, 0x09, payload)
        print("Factory reset command sent")
    
    def is_valid_nmea_message(self, data):
        """Check if data represents a valid NMEA message"""
        try:
            # Check if it starts with $ and ends with \n
            if not data.startswith(b'$') or not data.endswith(b'\n'):
                return False
            
            # Convert to string and check for valid NMEA format
            message_str = data.decode('ascii', errors='ignore').strip()
            if not re.match(r'^\$[A-Z]{2}[A-Z]{3},', message_str):
                return False
                
            # Basic checksum validation (optional)
            return True
            
        except:
            return False
    
    def get_nmea_message_type(self, data):
        """Extract NMEA message type from valid message"""
        try:
            message_str = data.decode('ascii', errors='ignore').strip()
            # Extract the 5-character message type (e.g., "$GNGSA" -> "GNGSA")
            return message_str[1:6] if len(message_str) >= 6 else "UNKNOWN"
        except:
            return "UNKNOWN"
    
    def monitor_messages(self, duration=60):
        """Monitor and count message types for specified duration"""
        self.message_stats.clear()
        self.monitoring = True
        start_time = time.time()
        buffer = bytearray()
        
        print(f"Monitoring messages for {duration} seconds...")
        
        while time.time() - start_time < duration and self.monitoring:
            try:
                # Read available data
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)
                    
                    # Process buffer to find complete messages
                    while len(buffer) > 0:
                        # Look for UBX messages first (binary, starts with 0xB5 0x62)
                        if len(buffer) >= 8 and buffer[0] == 0xB5 and buffer[1] == 0x62:
                            # UBX message found
                            length = int.from_bytes(buffer[4:6], 'little')
                            total_length = 8 + length  # Header(6) + payload + checksum(2)
                            
                            if len(buffer) >= total_length:
                                # Complete UBX message
                                msg_class = buffer[2]
                                msg_id = buffer[3]
                                msg_type = f"UBX-CLS-{msg_class:02X}-ID-{msg_id:02X}"
                                self.message_stats[msg_type] += 1
                                
                                # Remove processed message from buffer
                                buffer = buffer[total_length:]
                                continue
                            else:
                                # Incomplete UBX message, wait for more data
                                break
                        
                        # Look for NMEA messages (ASCII, starts with '$')
                        nmea_start = -1
                        for i in range(len(buffer)):
                            if buffer[i] == ord('$'):
                                nmea_start = i
                                break
                        
                        if nmea_start >= 0:
                            # Found start of potential NMEA message
                            # Look for end of message (newline)
                            nmea_end = -1
                            for i in range(nmea_start + 1, len(buffer)):
                                if buffer[i] == ord('\n'):
                                    nmea_end = i
                                    break
                            
                            if nmea_end >= 0:
                                # Complete potential NMEA message found
                                nmea_data = bytes(buffer[nmea_start:nmea_end + 1])
                                
                                if self.is_valid_nmea_message(nmea_data):
                                    # Valid NMEA message
                                    msg_type = self.get_nmea_message_type(nmea_data)
                                    self.message_stats[f"NMEA-{msg_type}"] += 1
                                else:
                                    # Invalid NMEA-like data, probably binary corruption
                                    self.message_stats["INVALID-NMEA-LIKE"] += 1
                                
                                # Remove processed data from buffer
                                buffer = buffer[nmea_end + 1:]
                                continue
                            else:
                                # Incomplete NMEA message, wait for more data
                                break
                        
                        # If we get here, no valid message start found, clean buffer
                        if nmea_start == -1 and (len(buffer) < 2 or buffer[0] != 0xB5 or buffer[1] != 0x62):
                            # No valid message start, remove first byte and continue
                            buffer.pop(0)
                        else:
                            # Wait for more data
                            break
                
                time.sleep(0.01)  # Small delay to prevent CPU overload
                
            except serial.SerialException as e:
                print(f"Serial error during monitoring: {e}")
                break
            except Exception as e:
                print(f"Error during message parsing: {e}")
                # Clear buffer on error to recover
                buffer.clear()
        
        self.monitoring = False
        print("Monitoring completed")
    
    def run_factory_reset_and_monitor(self):
        """Main function to perform factory reset and monitor output"""
        if not self.connect():
            return
        
        try:
            # Step 1: Perform factory reset
            print("Performing factory reset...")
            self.factory_reset()
            
            # Step 2: Wait 60 seconds for reset to complete
            print("Waiting 60 seconds for reset to complete...")
            time.sleep(60)
            
            # Clear any buffered data
            self.ser.reset_input_buffer()
            
            # Step 3: Monitor output for 60 seconds
            print("Monitoring messages for 60 seconds...")
            self.monitor_messages(60)
            
            # Step 4: Report message statistics
            print("\n=== MESSAGE STATISTICS ===")
            print("Message Type                    Count")
            print("-" * 40)
            
            total_messages = sum(self.message_stats.values())
            for msg_type, count in sorted(self.message_stats.items()):
                print(f"{msg_type:<30} {count}")
            
            print(f"\nTotal messages: {total_messages}")
            
        except Exception as e:
            print(f"Error during operation: {e}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("Serial connection closed")

# Run the configuration
if __name__ == "__main__":
    configurator = F9PConfigurator('/dev/f9p')
    configurator.run_factory_reset_and_monitor()