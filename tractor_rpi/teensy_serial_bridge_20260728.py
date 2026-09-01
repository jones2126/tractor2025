#!/usr/bin/env python3
"""
teensy_serial_bridge_20260728.py
Full original functionality + source-synchronized reliable broadcasting.

CHANGED 20260728:
  - Forward each source-timed low-level steering diagnostic. This supports
    both the original 10 Hz teensy_main_20260728.cpp stream and the 20 Hz
    teensy_main_20260804.cpp stream without timer-based resampling.
  - Broadcast consolidated UDP status once for every fresh source STEER
    record, rather than sampling a cache on an independent timer. This avoids
    timer-phase repeats/skips and preserves the source sequence end to end.
  - Preserve the Teensy source timestamp and steering sequence number so
    consumers can distinguish fresh control samples from cached UDP values.
  - Retain the original steering keys for backward compatibility.
"""

import serial
import socket
import json
import time
import threading
import select
import os
import queue
import urllib.request
from collections import defaultdict
import logging
import socket as socket_lib

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('TeensyBridge')

SERIAL_PORT = '/dev/teensy'
BAUD_RATE = 460800

# ==================== CONFIG ====================
#
UDP_BROADCAST_IP = '192.168.193.255'
#
# UDP_BIND_IP is selected based the machine's hostname when moved between tractor01 and tractor02.
_HOSTNAME_BIND_IP = {
    'tractor':   '192.168.193.76',   # tractor01
    'tractor02': '192.168.193.48',   # tractor02
}
_hostname = socket.gethostname()
UDP_BIND_IP = _HOSTNAME_BIND_IP.get(_hostname, '192.168.193.76')  # falls back to tractor01's IP
if _hostname not in _HOSTNAME_BIND_IP:
    print(f"[WARNING] Unrecognized hostname '{_hostname}' - defaulting UDP_BIND_IP to tractor01 ({UDP_BIND_IP})")

UDP_STATUS_PORT = 6003
UDP_COMMAND_PORT = 6004
UDP_GPS_PORT = 6002
STATUS_LOG_EVERY = 20  # log once per second when the source is running at 20 Hz
NTFY_TOPIC = os.environ.get('TRACTOR_NTFY_TOPIC', 'rpi-tractor01-jones2126')
NTFY_URL = f'https://ntfy.sh/{NTFY_TOPIC}'
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
        self.broadcast_counter = 0
        self.last_seen_steering_fault_count = 0
        self.notification_queue = queue.Queue(maxsize=8)

        self.current_gps_status = 1
        self.running = True

        self.stats = {k: 0 for k in ['messages_received', 'messages_parsed', 'broadcasts_sent',
                                    'broadcasts_failed', 'commands_received', 'commands_sent',
                                    'gps_packets_received', 'errors']}

        self.setup()

    def queue_steering_fault_notification(self, steering):
        """Queue one ntfy event for each new Teensy steering-fault count."""
        fault_count = int(steering.get('fc', 0))
        fault_latched = int(steering.get('sf', 0)) == 1

        # A Teensy reboot resets the counter, so allow a future count=1 event.
        if fault_count == 0:
            self.last_seen_steering_fault_count = 0
            return
        if not fault_latched or fault_count == self.last_seen_steering_fault_count:
            return

        self.last_seen_steering_fault_count = fault_count
        event = {
            'fault_count': fault_count,
            'mode': int(steering.get('m', 0)),
            'setpoint': steering.get('sp', 0),
            'current': steering.get('c', 0),
            'error': steering.get('e', 0),
            'response_elapsed_ms': int(steering.get('re', 0)),
            'movement_counts': int(steering.get('rm', 0)),
        }
        try:
            self.notification_queue.put_nowait(event)
        except queue.Full:
            logger.error("ntfy queue full; steering-fault notification dropped")

    def notification_worker(self):
        """Send ntfy requests away from the timing-sensitive serial loop."""
        logger.info(f"Steering-fault notifications enabled: {NTFY_TOPIC}")
        while self.running:
            try:
                event = self.notification_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            message = (
                f"Steering control was lost; transmission forced to neutral. "
                f"fault={event['fault_count']} mode={event['mode']} "
                f"target={event['setpoint']} pot={event['current']} "
                f"error={event['error']} no_progress={event['response_elapsed_ms']}ms "
                f"movement={event['movement_counts']} counts. "
                "Use Pause, reset/debug the IBT-2, then verify steering in Manual."
            )
            request = urllib.request.Request(
                NTFY_URL,
                data=message.encode('utf-8'),
                headers={
                    'Title': f'{_hostname}: steering control lost',
                    'Priority': 'high',
                    'Tags': 'warning,tractor',
                },
                method='POST',
            )
            try:
                with urllib.request.urlopen(request, timeout=5) as response:
                    response.read(64)
                logger.warning(f"Steering-fault notification sent: fault={event['fault_count']}")
            except Exception as exc:
                logger.error(f"Steering-fault ntfy request failed: {exc}")
            finally:
                self.notification_queue.task_done()

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
        last_gps_sent_time = 0                        # NEW
        GPS_SEND_INTERVAL = 1.0  # seconds - resend periodically so Teensy's 5s timeout never trips  # NEW
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

                now = time.time()                                          # NEW
                if now - last_gps_sent_time >= GPS_SEND_INTERVAL:          # NEW
                    try:                                                   # NEW
                        if self.ser and self.ser.out_waiting < 256:        # NEW
                            command = f"GPS,{self.current_gps_status}\n"   # NEW
                            self.ser.write(command.encode('utf-8'))       # NEW
                            self.ser.flush()                               # NEW
                            last_gps_sent_time = now                       # NEW
                    except Exception as e:                                 # NEW
                        logger.error(f"Failed to send GPS status to Teensy: {e}")  # NEW
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
                steer_dict['teensy_timestamp_ms'] = timestamp
                steer_dict['last_update'] = current_time
                self.latest_data['STEER'] = steer_dict
                self.queue_steering_fault_notification(kv_dict)
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
            'version': '2.5',
            'radio': {}, 'steering': {}, 'transmission': {}, 'system': {},
            'cmd_vel': {}, 'gps': {}
        }

        if 'RADIO' in self.latest_data:
            d = self.latest_data['RADIO']
            message['radio'] = {
                'signal': d.get('signal', 'UNKNOWN'),
                'ack_rate': d.get('ar', 0.0),        # CHANGED: Teensy sends key 'ar', not 'ack_rate'
                'current_rate': d.get('cr', 0.0),    # CHANGED: Teensy sends key 'cr', not 'current_rate'
                'age': current_time - d.get('last_update', current_time)
            }
        if 'STEER' in self.latest_data:
            d = self.latest_data['STEER']
            message['steering'] = {
                'mode': int(d.get('m', 0)),
                'sequence': int(d.get('q', 0)),
                'teensy_timestamp_ms': int(d.get('teensy_timestamp_ms', 0)),
                'state': d.get('st', 'UNKNOWN'),
                'setpoint': d.get('sp', 0.0),
                'current': d.get('c', 0.0),
                'error': d.get('e', 0.0),
                'direction': d.get('d', 'UNKNOWN'),
                'pwm': d.get('p', 0.0),
                'left_pwm': d.get('lp', 0.0),
                'right_pwm': d.get('rp', 0.0),
                'normalized_command': d.get('z', 0.0),
                'pid_active': int(d.get('pa', 0)),
                'deadband_active': int(d.get('db', 0)),
                'min_pwm_clamped': int(d.get('mc', 0)),
                'pwm_saturated': int(d.get('sat', 0)),
                'pid_dt_s': d.get('pdt', 0.0),
                'integral_sum': d.get('es', 0.0),
                'error_derivative': d.get('de', 0.0),
                'p_term': d.get('pt', 0.0),
                'i_term': d.get('it', 0.0),
                'd_term': d.get('dt', 0.0),
                'pid_output': d.get('out', 0.0),
                'cmd_age_ms': d.get('ca', -1),
                'response_attempt': int(d.get('ra', 0)),
                'response_state': int(d.get('rs', 0)),
                'response_elapsed_ms': int(d.get('re', 0)),
                'response_time_ms': int(d.get('rt', -1)),
                'response_movement_counts': int(d.get('rm', 0)),
                'fault_latched': int(d.get('sf', 0)),
                'fault_count': int(d.get('fc', 0)),
                'recovery_pause_seen': int(d.get('ps', 0)),
                'drive_blocked': int(d.get('rb', 0)),
                'age': current_time - d.get('last_update', current_time)
            }
        if 'TRANS' in self.latest_data:
            d = self.latest_data['TRANS']
            message['transmission'] = {
                'mode': int(d.get('m', 0)),
                'bucket': int(d.get('b', 5)),
                'target': d.get('tgt', 2836),
                'current': d.get('cur', 2836),
                'actual_target': d.get('at', 2836),
                'scaled_feedback': d.get('sfb', 2836),
                'integral': d.get('it', 0),
                'duty_cycle_target': d.get('dtt', 0),
                'duty_cycle': d.get('dc', 0),
                'errors_halting': d.get('eh', 0),
                'errors_occurred': d.get('eo', 0),
                'jrk_sequence': d.get('jq', 0),
                'jrk_valid': int(d.get('jv', 0)),
                'jrk_read_latency_ms': d.get('jl', 0),
                'jrk_timeouts': d.get('jto', 0),
                'jrk_discarded_bytes': d.get('jdb', 0),
                'radio_transmission_raw': d.get('rv', 0),
                'cmd_vel_mps': d.get('x', 0.0),
                'cmd_vel_age_ms': d.get('ca', -1),
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

    def broadcast_latest_status(self):
        """Broadcast one consolidated status packet for one fresh STEER row."""
        try:
            status = self.create_broadcast_message()
            data = json.dumps(status).encode()

            # Preserve the proven duplicate-send behavior. The field logger
            # deduplicates these copies by the Teensy steering sequence.
            self.status_sock.sendto(data, (UDP_BROADCAST_IP, UDP_STATUS_PORT))
            time.sleep(0.0005)
            self.status_sock.sendto(data, (UDP_BROADCAST_IP, UDP_STATUS_PORT))

            self.stats['broadcasts_sent'] += 1
            self.broadcast_counter += 1
            if self.broadcast_counter % STATUS_LOG_EVERY == 0:
                logger.info(
                    f"✓ STEER-SYNC BROADCAST ({len(data)} bytes, "
                    f"GPS={status['gps']['status']}, "
                    f"q={status['steering'].get('sequence', 0)})"
                )
        except Exception as e:
            logger.error(f"Broadcast error: {e}")

    def run(self):
        logger.info("Teensy Serial Bridge starting...")

        threading.Thread(target=self.gps_listener_thread, daemon=True).start()
        threading.Thread(target=self.listen_for_commands, daemon=True).start()
        threading.Thread(target=self.notification_worker, daemon=True).start()

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
                                    if parsed['subsystem'] == 'STEER':
                                        self.broadcast_latest_status()
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
