#!/usr/bin/env python3
"""
teensy_serial_bridge_20260625-final.py
Full original functionality + proven reliable broadcast thread.
"""

import serial
import socket
import json
import time
import threading
import select
from collections import defaultdict
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('TeensyBridge')

SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 460800

# ==================== CONFIG ====================
# Change these when switching interfaces
# UDP_BROADCAST_IP = '192.168.1.255'   # wlan0
# UDP_BIND_IP = '192.168.1.213'        # wlan0
UDP_BROADCAST_IP = '192.168.193.255'
#UDP_BIND_IP = '192.168.193.48'         # 192.168.193.48 (tractor02's IP)
UDP_BIND_IP = '192.168.193.76'         # 192.168.193.48 (tractor2025/tractor01's IP)

UDP_STATUS_PORT = 6003
UDP_COMMAND_PORT = 6004
UDP_GPS_PORT = 6002
BROADCAST_RATE = 5
# ================================================

class TeensySerialBridge:
    def __init__(self):
        self.ser = None
        self.status_sock = None
        self.command_sock = None
        self.gps_sock = None

        self.latest_data = defaultdict(dict)
        self.last_cmd_vel = {'linear_x': 0.0, 'angular_z': 0.0, 'timestamp': 0}
        self.cmd_vel_received_count = 0
        self.cmd_vel_sent_count = 0
        self.cmd_vel_echo_count = 0

        self.current_gps_status = 1
        self.running = True

        self.stats = {k: 0 for k in ['messages_received', 'messages_parsed', 'broadcasts_sent',
                                    'broadcasts_failed', 'commands_received', 'commands_sent',
                                    'gps_packets_received', 'errors']}

        self.setup()

    def create_broadcast_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        sock.bind((UDP_BIND_IP, 0))
        return sock

    def setup(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        logger.info(f"Serial connected to {SERIAL_PORT}")

        self.status_sock = self.create_broadcast_socket()
        logger.info("UDP broadcast socket ready")

        self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.command_sock.bind(('', UDP_COMMAND_PORT))
        self.command_sock.setblocking(False)

        self.gps_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gps_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.gps_sock.bind(('', UDP_GPS_PORT))
        self.gps_sock.setblocking(False)

    # ==================== Your original methods ====================

    def map_gps_status(self, fix_quality):
        fix_quality = str(fix_quality).strip()
        if fix_quality == "RTK Fixed": return 3
        if fix_quality == "RTK Float": return 2
        if fix_quality in ["GPS Fix", "DGPS"]: return 1
        return 0

    def gps_listener_thread(self):
        logger.info("GPS listener started")
        while self.running:
            try:
                ready = select.select([self.gps_sock], [], [], 0.1)
                if ready[0]:
                    data, _ = self.gps_sock.recvfrom(1024)
                    self.stats['gps_packets_received'] += 1
                    parsed = json.loads(data.decode())
                    status = self.map_gps_status(parsed.get('fix_quality', 'Unknown'))
                    if status != self.current_gps_status:
                        logger.info(f"GPS: {self.current_gps_status} → {status}")
                    self.current_gps_status = status
            except:
                pass
            time.sleep(0.05)

    def parse_message(self, line):
        try:
            parts = line.strip().split(',', 3)
            if len(parts) < 3:
                return None
            if not parts[0].isdigit():
                if parts[0] == '' and parts[1].isdigit():
                    parts = ['1'] + parts
                else:
                    return None

            msg_type = int(parts[0])
            timestamp = int(parts[1])
            subsystem = parts[2]
            current_time = time.time()

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
                radio_dict = self.latest_data.get('RADIO', {})
                radio_dict.update(kv_dict)
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
            elif subsystem in ('JRK', 'TRANS'):
                trans_dict = self.latest_data.get('TRANS', {})
                trans_dict.update(kv_dict)
                trans_dict['last_update'] = current_time
                self.latest_data['TRANS'] = trans_dict
            elif subsystem == 'SYS':
                sys_dict = self.latest_data.get('SYSTEM', {})
                sys_dict.update(kv_dict)
                sys_dict['last_update'] = current_time
                self.latest_data['SYSTEM'] = sys_dict

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
                self.last_cmd_vel = {'linear_x': linear_x, 'angular_z': angular_z, 'timestamp': time.time()}
                return True
        except Exception as e:
            logger.error(f"Failed to send cmd_vel: {e}")
        return False

    def listen_for_commands(self):
        logger.info("Command listener started")
        while self.running:
            try:
                ready = select.select([self.command_sock], [], [], 0.1)
                if ready[0]:
                    packets_read = 0
                    while packets_read < 100:
                        try:
                            data, _ = self.command_sock.recvfrom(1024)
                            packets_read += 1
                            command = json.loads(data.decode())
                            linear_x = command.get('linear_x', 0.0)
                            angular_z = command.get('angular_z', 0.0)
                            self.cmd_vel_received_count += 1
                            self.stats['commands_received'] += 1
                            self.send_cmd_vel_to_teensy(linear_x, angular_z)
                        except socket.error:
                            break
                        except json.JSONDecodeError:
                            logger.error("Invalid JSON command")
            except Exception as e:
                if self.running:
                    logger.error(f"Command listener error: {e}")
        logger.info("Command listener stopped")

    def create_broadcast_message(self):
        current_time = time.time()
        message = {
            'timestamp': current_time,
            'source': 'teensy_bridge',
            'version': '2.2',
            'radio': {}, 'steering': {}, 'transmission': {}, 'system': {},
            'cmd_vel': {}, 'gps': {}
        }

        if 'RADIO' in self.latest_data:
            d = self.latest_data['RADIO']
            message['radio'] = {
                'signal': d.get('signal', 'UNKNOWN'),
                'ack_rate': d.get('ack_rate', 0.0),
                'current_rate': d.get('current_rate', 0.0),
                'age': current_time - d.get('last_update', current_time)
            }
        if 'STEER' in self.latest_data:
            d = self.latest_data['STEER']
            message['steering'] = {
                'mode': int(d.get('m', 0)),
                'setpoint': d.get('sp', 0.0),
                'current': d.get('c', 0.0),
                'error': d.get('e', 0.0),
                'direction': d.get('d', 'UNKNOWN'),
                'pwm': d.get('p', 0.0),
                'age': current_time - d.get('last_update', current_time)
            }
        if 'TRANS' in self.latest_data:
            d = self.latest_data['TRANS']
            message['transmission'] = {
                'mode': int(d.get('m', 0)),
                'bucket': int(d.get('b', 5)),
                'target': d.get('tgt', 2985),
                'current': d.get('cur', 2985),
                'age': current_time - d.get('last_update', current_time)
            }
        if 'SYSTEM' in self.latest_data:
            d = self.latest_data['SYSTEM']
            message['system'] = {
                'heartbeat_age': current_time - d.get('last_update', current_time)
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

        message['gps'] = {'status': self.current_gps_status}

        return message

    def broadcast_thread_func(self):
        logger.info("Broadcast thread started")
        counter = 0
        while self.running:
            try:
                status = self.create_broadcast_message()
                data = json.dumps(status).encode()

                self.status_sock.sendto(data, (UDP_BROADCAST_IP, UDP_STATUS_PORT))
                time.sleep(0.0005)
                self.status_sock.sendto(data, (UDP_BROADCAST_IP, UDP_STATUS_PORT))

                self.stats['broadcasts_sent'] += 1
                if counter % 10 == 0:
                    logger.info(f"✓ BROADCAST SENT ({len(data)} bytes, GPS={status['gps']['status']})")

                counter += 1
                time.sleep(1.0 / BROADCAST_RATE)

            except Exception as e:
                logger.error(f"Broadcast error: {e}")
                time.sleep(1)

    def run(self):
        logger.info("Teensy Serial Bridge starting...")

        threading.Thread(target=self.gps_listener_thread, daemon=True).start()
        threading.Thread(target=self.listen_for_commands, daemon=True).start()
        threading.Thread(target=self.broadcast_thread_func, daemon=True).start()

        try:
            while True:
                if self.ser.in_waiting > 0:
                    try:
                        raw = self.ser.readline()
                        if raw:
                            line = raw.decode('utf-8', errors='ignore').strip()
                            if line and line[0].isdigit():
                                self.stats['messages_received'] += 1
                                parsed = self.parse_message(line)
                                if parsed:
                                    self.stats['messages_parsed'] += 1
                                    self.update_latest_data(parsed)
                    except:
                        pass
                time.sleep(0.01)

        except KeyboardInterrupt:
            logger.info("Shutdown requested")
        finally:
            self.running = False
            if self.ser and self.ser.is_open:
                self.ser.close()
            if self.status_sock:
                self.status_sock.close()

def main():
    try:
        bridge = TeensySerialBridge()
        bridge.run()
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)


if __name__ == "__main__":
    main()