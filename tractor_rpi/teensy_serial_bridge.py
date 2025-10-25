#!/usr/bin/env python3
"""
teensy_serial_bridge.py
=======================
Bridges Teensy serial data to UDP broadcasts for monitoring and control.

Input Data:
1. Teensy data from Serial.print commands (Type 1 and Type 2 messages)
2. (FUTURE) Navigation instructions from Pure Pursuit module

Data Published:
1. UDP Broadcast @ 5 Hz on Port 6003 - Steering, Transmission, and NRF24 Status
2. (FUTURE) Write cmd_vel (linear_x, angular_z) to Teensy for autonomous commands
"""

import serial
import socket
import json
import time
from collections import defaultdict
from datetime import datetime
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('TeensyBridge')

# Configuration
# SERIAL_PORT = '/dev/ttyACM0'
SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 115200
UDP_BROADCAST_IP = '255.255.255.255'
UDP_PORT = 6003
BROADCAST_RATE = 5  # Hz

class TeensySerialBridge:
    def __init__(self, serial_port=SERIAL_PORT, baud_rate=BAUD_RATE):
        """Initialize the Teensy serial bridge"""
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.sock = None
        
        # Store latest data from each subsystem
        self.latest_data = defaultdict(dict)
        
        # Timing
        self.last_broadcast = 0
        self.broadcast_interval = 1.0 / BROADCAST_RATE
        
        # Statistics
        self.stats = {
            'messages_received': 0,
            'messages_parsed': 0,
            'broadcasts_sent': 0,
            'errors': 0
        }
        
        self.setup()
    
    def setup(self):
        """Setup serial and UDP connections"""
        # Setup serial connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            logger.info(f"Serial connected to {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port: {e}")
            raise
        
        # Setup UDP socket
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            logger.info(f"UDP broadcast configured on port {UDP_PORT}")
        except socket.error as e:
            logger.error(f"Failed to create UDP socket: {e}")
            raise
    
    def parse_message(self, line):
        """
        Parse Teensy message format: <msg_type>,<timestamp>,<subsystem>,<data>
        
        Returns dict with parsed data or None if parse fails
        """
        try:
            parts = line.strip().split(',', 3)
            
            if len(parts) < 3:
                return None
            
            msg_type = int(parts[0])
            timestamp = int(parts[1])
            subsystem = parts[2]
            
            data = {'msg_type': msg_type, 'timestamp': timestamp}
            
            # Parse data section (key=value pairs or simple text)
            if len(parts) > 3:
                data_str = parts[3]
                
                if '=' in data_str:
                    # Parse key=value pairs
                    pairs = data_str.split(',')
                    for pair in pairs:
                        if '=' in pair:
                            key, value = pair.split('=', 1)
                            try:
                                # Try to convert to float
                                data[key] = float(value)
                            except ValueError:
                                # Keep as string
                                data[key] = value
                else:
                    # Simple text message
                    data['message'] = data_str
            
            return {
                'subsystem': subsystem,
                'data': data
            }
            
        except (ValueError, IndexError) as e:
            logger.debug(f"Parse error: {e} for line: {line}")
            return None
    
    def update_latest_data(self, parsed):
        """Update latest data for each subsystem"""
        if not parsed:
            return
        
        subsystem = parsed['subsystem']
        data = parsed['data']
        
        # Store latest data for this subsystem
        self.latest_data[subsystem].update(data)
        self.latest_data[subsystem]['last_update'] = time.time()
    
    def create_broadcast_message(self):
        """
        Create UDP broadcast message with key system data
        
        Message contains:
        - RADIO: signal status, ack rate
        - STEER: mode, setpoint, current, error, pwm
        - TRANS: mode, bucket, target
        - SYSTEM: heartbeat timestamp
        """
        current_time = time.time()
        
        # Build message with latest data from key subsystems
        message = {
            'timestamp': current_time,
            'source': 'teensy_bridge',
            'radio': {},
            'steering': {},
            'transmission': {},
            'system': {}
        }
        
        # Add RADIO data
        if 'RADIO' in self.latest_data:
            radio_data = self.latest_data['RADIO']
            message['radio'] = {
                'signal': radio_data.get('signal', 'UNKNOWN'),
                'ack_rate': radio_data.get('ack_rate', 0.0),
                'current_rate': radio_data.get('current_rate', 0.0),
                'age': current_time - radio_data.get('last_update', current_time)
            }
        
        # Add STEER data
        if 'STEER' in self.latest_data:
            steer_data = self.latest_data['STEER']
            message['steering'] = {
                'mode': int(steer_data.get('mode', 0)),
                'setpoint': steer_data.get('setpt', 0.0),
                'current': steer_data.get('current', 0.0),
                'error': steer_data.get('error', 0.0),
                'direction': steer_data.get('dir', 'UNKNOWN'),
                'pwm': steer_data.get('pwm', 0.0),
                'age': current_time - steer_data.get('last_update', current_time)
            }
        
        # Add TRANS data
        if 'TRANS' in self.latest_data:
            trans_data = self.latest_data['TRANS']
            message['transmission'] = {
                'mode': int(trans_data.get('mode', 0)),
                'bucket': int(trans_data.get('bucket', 5)),
                'target': trans_data.get('target', 2985),
                'current': trans_data.get('current', 2985),
                'age': current_time - trans_data.get('last_update', current_time)
            }
        
        # Add SYSTEM data
        if 'SYSTEM' in self.latest_data:
            sys_data = self.latest_data['SYSTEM']
            message['system'] = {
                'heartbeat_age': current_time - sys_data.get('last_update', current_time)
            }
        
        return message
    
    def broadcast_status(self):
        """Broadcast system status via UDP at specified rate"""
        current_time = time.time()
        
        if current_time - self.last_broadcast < self.broadcast_interval:
            return
        
        try:
            message = self.create_broadcast_message()
            json_data = json.dumps(message)
            
            self.sock.sendto(json_data.encode(), (UDP_BROADCAST_IP, UDP_PORT))
            
            self.stats['broadcasts_sent'] += 1
            self.last_broadcast = current_time
            
            # Log occasional broadcasts for monitoring
            if self.stats['broadcasts_sent'] % 25 == 0:  # Every 5 seconds at 5Hz
                logger.info(f"Broadcast #{self.stats['broadcasts_sent']}: "
                          f"Radio={message['radio'].get('signal', 'N/A')}, "
                          f"Steer_mode={message['steering'].get('mode', 'N/A')}, "
                          f"Trans_mode={message['transmission'].get('mode', 'N/A')}")
        
        except Exception as e:
            logger.error(f"Broadcast error: {e}")
            self.stats['errors'] += 1
    
    def process_serial_line(self, line):
        """Process one line of serial data"""
        self.stats['messages_received'] += 1
        
        # Parse the message
        parsed = self.parse_message(line)

        # log the first 10 messages for debugging
        if self.stats['messages_received'] <= 10:  # Log first 10 only
            logger.info(f"Raw received line: '{line}'")        
        
        if parsed:
            self.stats['messages_parsed'] += 1
            
            # Update latest data storage
            self.update_latest_data(parsed)
            
            # Log Type 1 messages occasionally
            if parsed['data'].get('msg_type') == 1 and self.stats['messages_parsed'] % 100 == 0:
                logger.debug(f"Parsed: {parsed['subsystem']}: {parsed['data']}")
    
    def print_statistics(self):
        """Print statistics periodically"""
        logger.info(f"Statistics - Received: {self.stats['messages_received']}, "
                   f"Parsed: {self.stats['messages_parsed']}, "
                   f"Broadcasts: {self.stats['broadcasts_sent']}, "
                   f"Errors: {self.stats['errors']}")
    
    def run(self):
        """Main loop"""
        logger.info("Teensy Serial Bridge starting...")
        logger.info(f"Broadcasting on UDP port {UDP_PORT} at {BROADCAST_RATE} Hz")
        
        last_stats_print = time.time()
        stats_interval = 30.0  # Print stats every 30 seconds
        
        try:
            while True:
                # Read serial data
                if self.ser.in_waiting:
                    try:
                        line = self.ser.readline().decode('utf-8').strip()
                        if line:
                            self.process_serial_line(line)
                    except UnicodeDecodeError:
                        self.stats['errors'] += 1
                        pass  # Skip malformed data
                
                # Broadcast status at specified rate
                self.broadcast_status()
                
                # Print statistics periodically
                if time.time() - last_stats_print >= stats_interval:
                    self.print_statistics()
                    last_stats_print = time.time()
                
                # Small sleep to prevent CPU spinning
                time.sleep(0.001)  # 1ms
        
        except KeyboardInterrupt:
            logger.info("Shutting down...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Serial port closed")
        
        if self.sock:
            self.sock.close()
            logger.info("UDP socket closed")
        
        self.print_statistics()


def main():
    """Main entry point"""
    try:
        bridge = TeensySerialBridge()
        bridge.run()
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        raise


if __name__ == "__main__":
    main()
