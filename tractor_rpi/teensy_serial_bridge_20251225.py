#!/usr/bin/env python3
"""
teensy_serial_bridge_20251225.py
=============================
Bidirectional bridge between Teensy and ROS/monitoring systems.
UPDATED: GPS status listener on UDP 6002 (from RTCM server), maps to enum, sends to Teensy at 2Hz.
"""

import serial
import socket
import json
import time
import threading
import select
from collections import defaultdict
from datetime import datetime
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('TeensyBridge')

SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 460800  # Match Teensy
UDP_BROADCAST_IP = '255.255.255.255'
UDP_STATUS_PORT = 6003
UDP_COMMAND_PORT = 6004
UDP_GPS_PORT = 6002  # From rtcm_server.py
BROADCAST_RATE = 5

class TeensySerialBridge:
    def __init__(self, serial_port=SERIAL_PORT, baud_rate=BAUD_RATE):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.status_sock = None
        self.command_sock = None
        self.gps_sock = None  # For GPS UDP
        
        self.latest_data = defaultdict(dict)
        
        self.last_cmd_vel = {'linear_x': 0.0, 'angular_z': 0.0, 'timestamp': 0}
        self.cmd_vel_received_count = 0
        self.cmd_vel_sent_count = 0
        self.cmd_vel_echo_count = 0
        
        # GPS state
        self.current_gps_status = 1  # Default: no fix
        self.last_gps_send = 0
        self.gps_send_interval = 0.5  # 2Hz
        self.gps_last_status = 1  # Track changes
        
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
            'gps_packets_received': 0,
            'gps_status_sent': 0,
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
        
        # GPS UDP listener
        try:
            self.gps_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.gps_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.gps_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
            self.gps_sock.bind(('', UDP_GPS_PORT))
            self.gps_sock.setblocking(False)
            logger.info(f"UDP GPS listener configured on port {UDP_GPS_PORT}")
        except socket.error as e:
            logger.error(f"Failed to create GPS UDP socket: {e}")
            raise
    
    # GPS status mapping (0=unset,1=noNMEA,2=noRTK,3=RTK Fix)
    # def map_gps_status(self, fix_quality, headValid, carrier):
    #     if fix_quality == "Invalid":
    #         return 0
    #     elif fix_quality == "GPS Fix":
    #         return 1
    #     elif fix_quality == "DGPS":
    #         return 2
    #     elif fix_quality == "RTK Fixed" and headValid and carrier == "fixed":
    #         return 3
    #     elif fix_quality in ["RTK Fixed", "RTK Float"] and headValid:
    #         return 2  # RTK but not full fix
    #     else:
    #         return 1  # Default GPS/no fix
    def map_gps_status(self, fix_quality, headValid, carrier):
        # Normalize strings to handle case variations
        fix_quality = str(fix_quality).strip()
        carrier = str(carrier).strip().lower()
        
        if fix_quality == "Invalid":
            return 0
        elif fix_quality == "RTK Fixed" and headValid and carrier == "fixed":
            return 3  # RTK Fixed with valid heading
        elif "RTK" in fix_quality: 
            return 2  # RTK Float or Fixed without header and carrier
        elif fix_quality in ["GPS Fix", "DGPS"]:
            return 1  # GPS but no RTK
        else:
            return 0  # Invalid/Unknown



    # def send_gps_to_teensy(self, status):
    #     now = time.time()
        
    #     # Debug entry
    #     logger.info(f"send_gps_to_teensy called: status={status}, last_send={self.last_gps_send:.2f}, last_status={self.gps_last_status}")
        
    #     # Rate limit check (skip for first send)
    #     if self.last_gps_send > 0 and now - self.last_gps_send < self.gps_send_interval:
    #         logger.info(f"GPS send SKIPPED (rate limited: {now - self.last_gps_send:.2f}s < {self.gps_send_interval}s)")
    #         return
        
    #     # No-change check (skip for first send)
    #     if status == self.gps_last_status and self.last_gps_send > 0:
    #         logger.info(f"GPS send SKIPPED (no change: {status} == {self.gps_last_status})")
    #         return
        
    #     try:
    #         cmd = f"GPS,{status}\n".encode('utf-8')
    #         self.ser.write(cmd)
    #         self.ser.flush()
    #         self.stats['gps_status_sent'] += 1
    #         logger.info(f"Sent GPS status {status} to Teensy (prev={self.gps_last_status})")
    #         self.last_gps_send = now
    #         self.gps_last_status = status
    #     except Exception as e:
    #         logger.error(f"Failed to send GPS status: {e}")
    #         self.stats['errors'] += 1
    
    # GPS listener thread
    # def gps_listener_thread(self):
    #     logger.info("GPS listener thread started")
    #     while self.running:
    #         try:
    #             ready = select.select([self.gps_sock], [], [], 0.1)
    #             if ready[0]:
    #                 data, addr = self.gps_sock.recvfrom(1024)
    #                 self.stats['gps_packets_received'] += 1
    #                 parsed = json.loads(data.decode())

    #                 # ========== ADD DEBUG OUTPUT ==========
    #                 fix = parsed.get('fix_quality', 'Unknown')
    #                 head_valid = parsed.get('headValid', False)
    #                 carr = parsed.get('carrier', 'none')
    #                 logger.info(f"GPS Raw: fix={fix}, headValid={head_valid}, carrier={carr}")
    #                 # ========== END DEBUG ==========

    #                 status = self.map_gps_status(
    #                     parsed.get('fix_quality', 'Unknown'),
    #                     parsed.get('headValid', False),
    #                     parsed.get('carrier', 'none')
    #                 )

    #                 # ========== ADD THIS DEBUG LINE ==========
    #                 logger.info(f"GPS Mapped Status: {status} (from fix={fix}, head={head_valid}, carr={carr})")
    #                 # ========== END DEBUG ==========

    #                 self.current_gps_status = status
    #                 self.send_gps_to_teensy(status)
    #         except json.JSONDecodeError as e:
    #             logger.warning(f"Invalid GPS JSON: {e}")
    #             self.stats['errors'] += 1
    #         except Exception as e:
    #             if self.running:
    #                 logger.error(f"GPS listener error: {e}")
    #                 self.stats['errors'] += 1
    #         time.sleep(0.05)  # ~20Hz poll, but non-blocking
    #     logger.info("GPS listener thread stopped")
    
    # GPS listener thread
    def gps_listener_thread(self):
        logger.info("GPS listener thread started")
        while self.running:
            try:
                ready = select.select([self.gps_sock], [], [], 0.1)
                if ready[0]:
                    data, addr = self.gps_sock.recvfrom(1024)
                    self.stats['gps_packets_received'] += 1
                    parsed = json.loads(data.decode())
                    
                    status = self.map_gps_status(
                        parsed.get('fix_quality', 'Unknown'),
                        parsed.get('headValid', False),
                        parsed.get('carrier', 'none')
                    )
                    
                    # ========== SIMPLIFIED: Just update the status ==========
                    old_status = self.current_gps_status
                    self.current_gps_status = status
                    
                    # Log only on status changes
                    if status != old_status:
                        logger.info(f"GPS status changed: {old_status} → {status}")
                    # ========== END SIMPLIFIED ==========
                    
            except json.JSONDecodeError as e:
                logger.warning(f"Invalid GPS JSON: {e}")
                self.stats['errors'] += 1
            except Exception as e:
                if self.running:
                    logger.error(f"GPS listener error: {e}")
                    self.stats['errors'] += 1
            time.sleep(0.05)  # ~20Hz poll
        logger.info("GPS listener thread stopped")

    def parse_message(self, line):
        """Parse a line from Teensy and update latest_data"""
        try:
            parts = line.strip().split(',', 3)
            
            if len(parts) < 3:
                return None
            
            # Handle occasional missing msg_type (rare bad line)
            if not parts[0].isdigit():
                if parts[0] == '' and parts[1].isdigit():
                    parts = ['1'] + parts  # Assume type 1
                else:
                    return None
            
            msg_type = int(parts[0])
            timestamp = int(parts[1])
            subsystem = parts[2]
            
            current_time = time.time()
            
            # Parse kv data if present
            data_str = parts[3] if len(parts) > 3 else ''
            kv_dict = {}
            for kv in data_str.split(','):
                if '=' in kv:
                    k, v = kv.split('=', 1)
                    try:
                        kv_dict[k] = float(v) if '.' in v else int(v)
                    except ValueError:
                        kv_dict[k] = v
            
            parsed = {
                'msg_type': msg_type,
                'timestamp': timestamp,
                'subsystem': subsystem,
                'data': kv_dict,
                'last_update': current_time
            }
            
            if subsystem == 'RADIO':
                # Handle both stats and detailed lines
                radio_dict = self.latest_data.get('RADIO', {})
                radio_dict.update(kv_dict)  # Merge new keys
                if kv_dict.get('sg') == 1:
                    radio_dict['signal'] = 'GOOD'
                elif 'signal' not in radio_dict:
                    radio_dict['signal'] = 'UNKNOWN'
                radio_dict['last_update'] = current_time
                self.latest_data['RADIO'] = radio_dict
            
            elif subsystem == 'STEER':
                steer_dict = self.latest_data.get('STEER', {})
                steer_dict.update(kv_dict)
                steer_dict['last_update'] = current_time
                self.latest_data['STEER'] = steer_dict
            
            elif subsystem == 'JRK' or subsystem == 'TRANS':
                trans_dict = self.latest_data.get('TRANS', {})
                trans_dict.update(kv_dict)  # tgt, current, etc.
                trans_dict['last_update'] = current_time
                self.latest_data['TRANS'] = trans_dict
            
            elif subsystem == 'SYS':
                sys_dict = self.latest_data.get('SYSTEM', {})
                sys_dict.update(kv_dict)  # hb, etc.
                sys_dict['last_update'] = current_time
                self.latest_data['SYSTEM'] = sys_dict
            
            # Add other subsystems as needed (GPS_ECHO, ACK_PREP if useful)
            
            return parsed
        
        except Exception as e:
            logger.warning(f"Failed to parse line: {line} | {e}")
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
            'version': '2.2',
            'radio': {},
            'steering': {},
            'transmission': {},
            'system': {},
            'cmd_vel': {},
            'gps': {}
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
                'mode': int(steer_data.get('m', 0)),
                'setpoint': steer_data.get('sp', 0.0),
                'current': steer_data.get('c', 0.0),
                'error': steer_data.get('e', 0.0),
                'direction': steer_data.get('d', 'UNKNOWN'),
                'pwm': steer_data.get('p', 0.0),
                'age': current_time - steer_data.get('last_update', current_time)
            }
        
        if 'TRANS' in self.latest_data:
            trans_data = self.latest_data['TRANS']
            message['transmission'] = {
                'mode': int(trans_data.get('m', 0)),
                'bucket': int(trans_data.get('b', 5)),
                'target': trans_data.get('tgt', 2985),
                'current': trans_data.get('cur', 2985),
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
        
        # GPS in broadcast
        message['gps'] = {
            'status': self.current_gps_status,  # 0-3 enum
            'last_update': current_time - self.last_gps_send if self.last_gps_send else current_time,
            'packets_received': self.stats['gps_packets_received']
        }
        
        return message

    def broadcast_status(self):
        now = time.time()
        if now - self.last_broadcast < self.broadcast_interval:
            return

        # ========== ADD TIMING DEBUG ==========
        if hasattr(self, '_last_gps_send_time'):
            gps_send_gap = now - self._last_gps_send_time
            if gps_send_gap > 0.3:  # Warn if gap > 300ms
                logger.warning(f"GPS send gap: {gps_send_gap:.3f}s (expected 0.2s)")
        self._last_gps_send_time = now
        # ========== END DEBUG ==========

        status = self.create_broadcast_message()

        # ========== ADD GPS SERIAL SEND HERE (at 5Hz with broadcast) ==========
        try:
            gps_cmd = f"GPS,{self.current_gps_status}\n".encode('utf-8')
            self.ser.write(gps_cmd)
            self.ser.flush()
        except Exception as e:
            logger.error(f"Failed to send GPS to Teensy: {e}")
        # ========== END GPS SEND ==========

        try:
            data = json.dumps(status).encode('utf-8')
            self.status_sock.sendto(data, (UDP_BROADCAST_IP, UDP_STATUS_PORT))
            self.stats['broadcasts_sent'] += 1
            logger.debug(f"Broadcast #{self.stats['broadcasts_sent']}: "
                        f"GPS_status={status['gps']['status']}, "
                        f"CMD_echoes={status['cmd_vel']['commands_echoed']}, "
                        f"Size={len(data)} bytes")
        except Exception as e:
            logger.error(f"Broadcast error: {e}")
            self.stats['broadcasts_failed'] += 1
            self.stats['errors'] += 1
        
        self.last_broadcast = now

    def process_serial_line(self, line):
        self.stats['messages_received'] += 1
        parsed = self.parse_message(line)
        if parsed:
            self.stats['messages_parsed'] += 1
            self.update_latest_data(parsed)

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
        logger.info(f"GPS - Packets: {self.stats['gps_packets_received']}, "
                   f"Status Sent: {self.stats['gps_status_sent']}, "
                   f"Current: {self.current_gps_status}")

    def run(self):
        """Main loop - robust version with serial recovery and line filtering"""
        logger.info("Teensy Serial Bridge v20251225 starting...")
        logger.info(f"Status broadcasts on UDP port {UDP_STATUS_PORT} at {BROADCAST_RATE} Hz")
        logger.info(f"Command listener on UDP port {UDP_COMMAND_PORT}")
        logger.info(f"GPS listener on UDP port {UDP_GPS_PORT} → Serial at 2Hz")
        
        # Start background threads
        command_thread = threading.Thread(target=self.listen_for_commands, daemon=True)
        command_thread.start()
        
        gps_thread = threading.Thread(target=self.gps_listener_thread, daemon=True)
        gps_thread.start()
        
        last_stats_print = time.time()
        stats_interval = 30.0
        
        try:
            while True:
                try:
                    # Robust serial read
                    if self.ser.in_waiting > 0:
                        raw = self.ser.readline()
                        if not raw:
                            continue  # False positive readiness
                        
                        # Decode safely, ignore errors
                        try:
                            line = raw.decode('utf-8', errors='ignore').strip()
                        except:
                            continue
                        
                        if not line:
                            continue
                        
                        # Skip non-standard lines (debug, garbage, etc.)
                        if not line[0].isdigit():
                            # Optional quiet debug: logger.debug(f"Skipped line: {line}")
                            continue
                        
                        # Safe parsing
                        self.process_serial_line(line)
                    
                    # Normal operations
                    self.broadcast_status()
                    
                    if time.time() - last_stats_print >= stats_interval:
                        self.print_statistics()
                        last_stats_print = time.time()
                    
                    time.sleep(0.001)  # Low CPU, responsive
                    
                except serial.SerialException as e:
                    logger.error(f"Serial read error: {e} - attempting reconnect in 5s...")
                    try:
                        self.ser.close()
                    except:
                        pass
                    time.sleep(5)
                    try:
                        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=2)
                        self.ser.flushInput()
                        self.ser.flushOutput()
                        logger.info("Serial reconnected successfully")
                    except Exception as reconnect_e:
                        logger.error(f"Serial reconnect failed: {reconnect_e}")
                
                except Exception as e:
                    logger.error(f"Unexpected error in main loop: {e}")
                    time.sleep(1)
        
        except KeyboardInterrupt:
            logger.info("Shutting down...")
        finally:
            self.running = False
            command_thread.join(timeout=2)
            gps_thread.join(timeout=2)
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
        
        if self.gps_sock:
            self.gps_sock.close()
            logger.info("GPS UDP socket closed")
        
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