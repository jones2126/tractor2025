#!/usr/bin/env python3
"""
teensy_serial_bridge.py v2.1
=============================
Bidirectional bridge between Teensy and ROS/monitoring systems.
"""

import serial
import socket
import json
import time
import threading
from collections import defaultdict
from datetime import datetime
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('TeensyBridge')

SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 115200
UDP_BROADCAST_IP = '255.255.255.255'
UDP_STATUS_PORT = 6003
UDP_COMMAND_PORT = 6004
BROADCAST_RATE = 5

class TeensySerialBridge:
    def __init__(self, serial_port=SERIAL_PORT, baud_rate=BAUD_RATE):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.status_sock = None
        self.command_sock = None
        
        self.latest_data = defaultdict(dict)
        
        self.last_cmd_vel = {'linear_x': 0.0, 'angular_z': 0.0, 'timestamp': 0}
        self.cmd_vel_received_count = 0
        self.cmd_vel_sent_count = 0
        self.cmd_vel_echo_count = 0
        
        self.last_broadcast = 0
        self.broadcast_interval = 1.0 / BROADCAST_RATE
        
        self.running = True
        
        self.stats = {
            'messages_received': 0,
            'messages_parsed': 0,
            'broadcasts_sent': 0,
            'broadcasts_failed': 0,
            'commands_received': 0,
            'commands_sent': 0,
            'command_bursts': 0,
            'max_burst_size': 0,
            'errors': 0
        }
        
        self.setup()
    
    def setup(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            logger.info(f"Serial connected to {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port: {e}")
            raise
        
        try:
            self.status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.status_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.status_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.status_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
            logger.info(f"UDP status broadcast configured on port {UDP_STATUS_PORT}")
        except socket.error as e:
            logger.error(f"Failed to create status UDP socket: {e}")
            raise
        
        try:
            self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.command_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.command_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
            self.command_sock.bind(('', UDP_COMMAND_PORT))
            self.command_sock.setblocking(False)
            logger.info(f"UDP command listener configured on port {UDP_COMMAND_PORT}")
        except socket.error as e:
            logger.error(f"Failed to create command UDP socket: {e}")
            raise
    
    def parse_message(self, line):
        try:
            parts = line.strip().split(',', 3)
            
            if len(parts) < 3:
                return None
            
            msg_type = int(parts[0])
            timestamp = int(parts[1])
            subsystem = parts[2]
            
            data = {'msg_type': msg_type, 'timestamp': timestamp}
            
            if len(parts) > 3:
                data_str = parts[3]
                
                if '=' in data_str:
                    pairs = data_str.split(',')
                    for pair in pairs:
                        if '=' in pair:
                            key, value = pair.split('=', 1)
                            try:
                                data[key] = float(value)
                            except ValueError:
                                data[key] = value
                else:
                    data['message'] = data_str
            
            return {
                'subsystem': subsystem,
                'data': data
            }
            
        except (ValueError, IndexError) as e:
            logger.debug(f"Parse error: {e} for line: {line}")
            return None
    
    def update_latest_data(self, parsed):
        if not parsed:
            return
        
        subsystem = parsed['subsystem']
        data = parsed['data']
        
        if subsystem == 'CMD_ECHO':
            self.cmd_vel_echo_count += 1
            logger.info(f"CMD ECHO #{self.cmd_vel_echo_count}: "
                       f"linear_x={data.get('linear_x', 0):.3f}, "
                       f"angular_z={data.get('angular_z', 0):.3f}")
        
        self.latest_data[subsystem].update(data)
        self.latest_data[subsystem]['last_update'] = time.time()
    
    def send_cmd_vel_to_teensy(self, linear_x, angular_z):
        try:
            if self.ser.out_waiting < 256:
                command = f"CMD,{linear_x:.4f},{angular_z:.4f}\n"
                self.ser.write(command.encode('utf-8'))
                self.ser.flush()
                
                self.cmd_vel_sent_count += 1
                self.stats['commands_sent'] += 1
                
                self.last_cmd_vel = {
                    'linear_x': linear_x,
                    'angular_z': angular_z,
                    'timestamp': time.time()
                }
                
                if self.cmd_vel_sent_count % 100 == 0:
                    logger.info(f"Sent CMD #{self.cmd_vel_sent_count}: "
                               f"linear_x={linear_x:.3f}, angular_z={angular_z:.3f}")
                return True
            else:
                logger.warning("Serial output buffer full - skipping command")
                return False
        except Exception as e:
            logger.error(f"Failed to send cmd_vel: {e}")
            self.stats['errors'] += 1
            return False
    
    def listen_for_commands(self):
        import select
        logger.info("Command listener thread started")
        
        while self.running:
            try:
                ready = select.select([self.command_sock], [], [], 0.1)
                if ready[0]:
                    packets_read = 0
                    while packets_read < 100:
                        try:
                            data, addr = self.command_sock.recvfrom(1024)
                            packets_read += 1
                            command = json.loads(data.decode())
                            linear_x = command.get('linear_x', 0.0)
                            angular_z = command.get('angular_z', 0.0)
                            self.cmd_vel_received_count += 1
                            self.stats['commands_received'] += 1
                            self.send_cmd_vel_to_teensy(linear_x, angular_z)
                        except socket.error:
                            break
                        except json.JSONDecodeError as e:
                            logger.error(f"Invalid JSON command: {e}")
                            self.stats['errors'] += 1
                    if packets_read > 0:
                        self.stats['command_bursts'] += 1
                        if packets_read > self.stats['max_burst_size']:
                            self.stats['max_burst_size'] = packets_read
                        if packets_read > 10:
                            logger.debug(f"Burst: Read {packets_read} packets")
            except Exception as e:
                if self.running:
                    logger.error(f"Command listener error: {e}")
                    self.stats['errors'] += 1
        logger.info("Command listener thread stopped")
    
    def create_broadcast_message(self):
        current_time = time.time()
        
        message = {
            'timestamp': current_time,
            'source': 'teensy_bridge',
            'version': '2.1',
            'radio': {},
            'steering': {},
            'transmission': {},
            'system': {},
            'cmd_vel': {}
        }
        
        if 'RADIO' in self.latest_data:
            radio_data = self.latest_data['RADIO']
            message['radio'] = {
                'signal': radio_data.get('signal', 'UNKNOWN'),
                'ack_rate': radio_data.get('ack_rate', 0.0),
                'current_rate': radio_data.get('current_rate', 0.0),
                'age': current_time - radio_data.get('last_update', current_time)
            }
        
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
        
        if 'TRANS' in self.latest_data:
            trans_data = self.latest_data['TRANS']
            message['transmission'] = {
                'mode': int(trans_data.get('mode', 0)),
                'bucket': int(trans_data.get('bucket', 5)),
                'target': trans_data.get('target', 2985),
                'current': trans_data.get('current', 2985),
                'age': current_time - trans_data.get('last_update', current_time)
            }
        
        if 'SYSTEM' in self.latest_data:
            sys_data = self.latest_data['SYSTEM']
            message['system'] = {
                'heartbeat_age': current_time - sys_data.get('last_update', current_time)
            }
        
        message['cmd_vel'] = {
            'last_linear_x': self.last_cmd_vel['linear_x'],
            'last_angular_z': self.last_cmd_vel['angular_z'],
            'commands_received': self.cmd_vel_received_count,
            'commands_sent': self.cmd_vel_sent_count,
            'commands_echoed': self.cmd_vel_echo_count,
            'pending_echoes': self.cmd_vel_sent_count - self.cmd_vel_echo_count,
            'age': current_time - self.last_cmd_vel['timestamp'],
            'active': (current_time - self.last_cmd_vel['timestamp']) < 2.0
        }
        
        return message
    
    def broadcast_status(self):
        current_time = time.time()
        
        if current_time - self.last_broadcast < self.broadcast_interval:
            return
        
        try:
            message = self.create_broadcast_message()
            json_data = json.dumps(message)
            
            bytes_sent = self.status_sock.sendto(
                json_data.encode(), 
                (UDP_BROADCAST_IP, UDP_STATUS_PORT)
            )
            
            if bytes_sent != len(json_data):
                logger.warning(f"Incomplete broadcast: sent {bytes_sent}/{len(json_data)} bytes")
                self.stats['broadcasts_failed'] += 1
            else:
                self.stats['broadcasts_sent'] += 1
            
            self.last_broadcast = current_time
            
            if self.stats['broadcasts_sent'] % 25 == 0:
                logger.info(f"Broadcast #{self.stats['broadcasts_sent']}: "
                          f"Radio={message['radio'].get('signal', 'N/A')}, "
                          f"Steer_mode={message['steering'].get('mode', 'N/A')}, "
                          f"Trans_mode={message['transmission'].get('mode', 'N/A')}, "
                          f"CMD_echoes={message['cmd_vel']['commands_echoed']}, "
                          f"Size={len(json_data)} bytes")
        
        except Exception as e:
            logger.error(f"Broadcast error: {e}")
            self.stats['errors'] += 1
            self.stats['broadcasts_failed'] += 1
    
    def process_serial_line(self, line):
        self.stats['messages_received'] += 1
        
        parsed = self.parse_message(line)
        
        if parsed:
            self.stats['messages_parsed'] += 1
            self.update_latest_data(parsed)
            
            if parsed['data'].get('msg_type') == 1 and self.stats['messages_parsed'] % 100 == 0:
                logger.debug(f"Parsed: {parsed['subsystem']}: {parsed['data']}")
    
    def print_statistics(self):
        logger.info(f"Statistics - Received: {self.stats['messages_received']}, "
                   f"Parsed: {self.stats['messages_parsed']}, "
                   f"Broadcasts: {self.stats['broadcasts_sent']} "
                   f"({self.stats['broadcasts_failed']} failed), "
                   f"Errors: {self.stats['errors']}")
        logger.info(f"Commands - Received: {self.stats['commands_received']}, "
                   f"Sent: {self.stats['commands_sent']}, "
                   f"Echoed: {self.cmd_vel_echo_count}, "
                   f"Pending: {self.cmd_vel_sent_count - self.cmd_vel_echo_count}")
        logger.info(f"Command Bursts - Total: {self.stats['command_bursts']}, "
                   f"Max Size: {self.stats['max_burst_size']}, "
                   f"Avg Size: {self.stats['commands_received'] / max(1, self.stats['command_bursts']):.1f}")
    
    def run(self):
        logger.info("Teensy Serial Bridge v2.1 starting...")
        logger.info(f"Status broadcasts on UDP port {UDP_STATUS_PORT} at {BROADCAST_RATE} Hz")
        logger.info(f"Command listener on UDP port {UDP_COMMAND_PORT}")
        
        command_thread = threading.Thread(target=self.listen_for_commands, daemon=True)
        command_thread.start()
        
        last_stats_print = time.time()
        stats_interval = 30.0
        
        try:
            while True:
                if self.ser.in_waiting:
                    try:
                        line = self.ser.readline().decode('utf-8').strip()
                        if line:
                            self.process_serial_line(line)
                    except UnicodeDecodeError:
                        self.stats['errors'] += 1
                        pass
                
                self.broadcast_status()
                
                if time.time() - last_stats_print >= stats_interval:
                    self.print_statistics()
                    last_stats_print = time.time()
                
                time.sleep(0.001)
        
        except KeyboardInterrupt:
            logger.info("Shutting down...")
        finally:
            self.running = False
            command_thread.join(timeout=2)
            self.cleanup()
    
    def cleanup(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Serial port closed")
        
        if self.status_sock:
            self.status_sock.close()
            logger.info("Status UDP socket closed")
        
        if self.command_sock:
            self.command_sock.close()
            logger.info("Command UDP socket closed")
        
        self.print_statistics()

def main():
    try:
        bridge = TeensySerialBridge()
        bridge.run()
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        raise

if __name__ == "__main__":
    main()