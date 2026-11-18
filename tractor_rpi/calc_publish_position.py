"""Boot position/sensor producer; experimental pose never grants control."""
from __future__ import annotations
import argparse
from contextlib import nullcontext
import hashlib
import json
from pathlib import Path
import queue
import signal
import socket
import subprocess
import threading
import time
import datetime as dt
from tractor_rpi import position_gnss_protocol as protocol
from tractor_rpi.position_model import LEFT_SERIAL, RIGHT_SERIAL, PORTS, PoseModel, parse_base_nmea, parse_encoder
from tractor_rpi.imu_sensor import Sensor
from tractor_rpi.telemetry_ipc import Publisher, request

def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec='microseconds')

def event(source, kind, data):
    return dict(source=source, kind=kind, data=data, mono=time.monotonic(), utc=utc_now())

def require_free_device(path):
    result = subprocess.run(['fuser', str(path)], capture_output=True, text=True, timeout=5)
    if result.returncode == 0:
        raise RuntimeError(f'Device already owned: {path}; do not run beside the original server/logger')
    if result.returncode != 1:
        raise RuntimeError(f'Cannot establish device ownership: {path}: {result.stderr}')

def encoder_device(ports, serial_number):
    matches = [p.device for p in ports if p.serial_number == serial_number]
    if len(matches) != 1:
        raise RuntimeError(f'Expected exactly one encoder with USB serial {serial_number}; found {matches}')
    return matches[0]

class Framer:
    """Bounded mixed NMEA/UBX serial decoder; handles partial and joined reads."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, block):
        self.buffer.extend(block)
        if len(self.buffer) > 8192:
            raise ValueError('GNSS framing buffer overflow')
        frames = []
        while self.buffer:
            if self.buffer[0] == 36:
                end = self.buffer.find(b'\n')
                if end < 0:
                    if len(self.buffer) > 512:
                        del self.buffer[0]
                        continue
                    break
                frames.append(('nmea', bytes(self.buffer[:end + 1]).strip()))
                del self.buffer[:end + 1]
            elif self.buffer[0] == protocol.SYNC1:
                if len(self.buffer) < 2:
                    break
                if self.buffer[1] != protocol.SYNC2:
                    del self.buffer[0]
                    continue
                if len(self.buffer) < 6:
                    break
                length = self.buffer[4] | self.buffer[5] << 8
                if length > 4096:
                    del self.buffer[0]
                    continue
                if len(self.buffer) < length + 8:
                    break
                frame = bytes(self.buffer[:length + 8])
                del self.buffer[:length + 8]
                if tuple(frame[-2:]) == protocol.ubx_checksum(frame[2:-2]):
                    frames.append(('ubx', frame))
            else:
                del self.buffer[0]
        return frames

class PositionApp:

    def __init__(self, devices, bus, recorder, stop, scales=None, calibration_id=None, udp=None):
        self.devices = devices
        self.bus = bus
        self.recorder = recorder
        self.stop = stop
        self.queues = {name: queue.Queue(maxsize=4096) for name in ('gps', 'imu', 'encoders')}
        self.pending = []
        self.serial_write_lock = threading.Lock()
        self.model = PoseModel(recorder.emit, scales, calibration_id)
        self.threads = []
        self.fault = None
        self.udp = udp
        self.sequence = 0
        self.last_publish = None
        self.last_diagnostics = None
        self.udp_sent = {port: 0 for port in PORTS}

    def emit_sensor(self, family, value):
        try:
            self.queues[family].put_nowait(value)
        except queue.Full:
            raise RuntimeError(f'{family} measurement queue overflow')

    def worker(self, name, function, *args):
        try:
            function(*args)
        except Exception as exc:
            self.fault = f'{name}: {exc}'
            self.recorder.emit('events', dict(utc=utc_now(), event='worker_failed', worker=name, error=str(exc)))
            self.stop.set()

    def read_gps(self, side):
        device = self.devices[side]
        framer = Framer()
        last_poll = 0.0
        while not self.stop.is_set():
            now = time.monotonic()
            if now - last_poll >= 1:
                with self.serial_write_lock if side == 'base' else nullcontext():
                    polls = protocol.ubx_frame(protocol.CLS_NAV, protocol.ID_PVT) + protocol.ubx_frame(protocol.CLS_NAV, protocol.ID_SAT)
                    if device.write(polls) != len(polls):
                        raise RuntimeError('Incomplete diagnostic poll write')
                last_poll = now
            block = device.read(max(1, min(512, device.in_waiting)))
            for kind, data in framer.feed(block):
                result = None
                if kind == 'nmea':
                    if side == 'base':
                        result = parse_base_nmea(data)
                    else:
                        sentence = protocol.parse_checksum_valid_nmea_type(data)
                        if sentence:
                            result = ('heading_nmea', sentence)
                elif data[2] == protocol.CLS_NAV:
                    message, payload = (data[3], data[6:-2])
                    if side == 'heading' and message == protocol.ID_RELPOSNED:
                        decoded = protocol.parse_relposned(payload)
                        if decoded:
                            result = ('heading', decoded)
                    elif message in (protocol.ID_PVT, protocol.ID_SAT):
                        decoded = protocol.parse_nav_pvt_satellites(payload) if message == protocol.ID_PVT else protocol.parse_nav_sat(payload)
                        if decoded:
                            result = ('satellites', decoded)
                if result:
                    self.emit_sensor('gps', event(side, *result))

    def read_gps_base(self):
        self.read_gps('base')

    def read_gps_heading(self):
        self.read_gps('heading')

    def read_imu(self):
        sensor = Sensor(self.bus)
        sensor.configure()
        last = time.monotonic()
        while not self.stop.is_set():
            reading = sensor.read()
            if reading is not None:
                last = time.monotonic()
                self.emit_sensor('imu', event('imu', 'sample', reading))
            else:
                if time.monotonic() - last > 1:
                    raise RuntimeError('No fresh IMU sample for one second')
                self.stop.wait(0.001)

    def read_encoder(self, side):
        device = self.devices[side]
        buffer = bytearray()
        last = time.monotonic()
        while not self.stop.is_set():
            block = device.read(max(1, min(512, device.in_waiting)))
            if block:
                last = time.monotonic()
                buffer.extend(block)
            if len(buffer) > 2048:
                raise RuntimeError('Encoder framing overflow')
            while b'\n' in buffer:
                line, _, remaining = buffer.partition(b'\n')
                buffer = bytearray(remaining)
                try:
                    kind, data = parse_encoder(line, side)
                except (ValueError, UnicodeDecodeError) as exc:
                    kind, data = ('parse_error', dict(error=str(exc), raw=line.decode('ascii', errors='replace')))
                self.emit_sensor('encoders', event(side, kind, data))
            if time.monotonic() - last > 5:
                raise RuntimeError(f'Encoder {side} has produced no serial data for 5 seconds')

    def forward_rtcm_corrections(self):
        total = 0
        interval_bytes = 0
        last_log = time.monotonic()
        while not self.stop.is_set():
            try:
                with socket.create_connection(('192.168.193.88', 6001), timeout=3) as connection:
                    connection.settimeout(1)
                    self.recorder.emit('console', '[RTCM Forwarder] CONNECTED to 192.168.193.88:6001')
                    while not self.stop.is_set():
                        try:
                            data = connection.recv(4096)
                        except socket.timeout:
                            continue
                        if not data:
                            raise ConnectionError('Correction source closed connection')
                        with self.serial_write_lock:
                            written = self.devices['base'].write(data)
                        if written != len(data):
                            raise RuntimeError('Incomplete RTCM serial write')
                        total += written
                        interval_bytes += written
                        elapsed = time.monotonic() - last_log
                        if elapsed >= 10:
                            self.recorder.emit('console', f'[RTCM Forwarder] Forwarded {interval_bytes} bytes (~{interval_bytes / elapsed:.0f} B/s) in last {elapsed:.1f}s | Total forwarded: {total}')
                            self.recorder.emit('events', dict(event='rtcm_forwarding', utc=utc_now(), bytes=interval_bytes, elapsed_s=elapsed, total_bytes=total))
                            interval_bytes = 0
                            last_log = time.monotonic()
            except OSError as exc:
                self.recorder.emit('console', f'[RTCM Forwarder] Connection error: {exc}; reconnecting in 5 seconds')
                self.recorder.emit('events', dict(event='rtcm_connection_error', utc=utc_now(), error=str(exc)))
                self.stop.wait(5)

    def start(self):
        tasks = [('gps-base', self.read_gps_base, ()), ('gps-heading', self.read_gps_heading, ()), ('imu', self.read_imu, ()), ('encoder-L', self.read_encoder, ('L',)), ('encoder-R', self.read_encoder, ('R',)), ('rtcm', self.forward_rtcm_corrections, ())]
        for name, function, args in tasks:
            thread = threading.Thread(target=self.worker, args=(name, function, *args), name=name, daemon=True)
            thread.start()
            self.threads.append(thread)

    def drain(self, family):
        channel = self.queues[family]
        for _ in range(channel.maxsize):
            try:
                self.pending.append(channel.get_nowait())
            except queue.Empty:
                break

    def ingest_gps(self):
        self.drain('gps')

    def ingest_imu(self):
        self.drain('imu')

    def ingest_encoders(self):
        self.drain('encoders')

    def update_pose(self):
        self.pending.sort(key=lambda item: item['mono'])
        for item in self.pending:
            if item['source'] in ('base', 'heading'):
                self.model.apply_gps(item)
            elif item['source'] == 'imu':
                self.model.apply_imu(item)
            else:
                self.model.apply_encoder(item)
        self.pending.clear()

    def assess_pose_quality(self, now):
        return self.model.quality(now)

    def publish_navigation_if_due(self, now):
        if self.last_publish is not None and now - self.last_publish < 0.049:
            return
        if self.udp is None:
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.setblocking(False)
        payload = json.dumps(self.model.gps, allow_nan=False, separators=(',', ':')).encode()
        if len(payload) > 2048:
            raise RuntimeError('Legacy GPS snapshot exceeds field logger receive size')
        for port in PORTS:
            if self.udp.sendto(payload, ('127.0.0.1', port)) != len(payload):
                raise RuntimeError('Incomplete UDP datagram send')
            self.udp_sent[port] += 1
        record = dict(utc=utc_now(), monotonic_s=now, sequence=self.sequence)
        # GNSS is recorded once by the recorder listening on UDP 6009.
        self.recorder.emit('pose', {**record, 'pose': self.model.snapshot(now), 'calibration': self.model.calibration})
        self.sequence += 1
        self.last_publish = now

    def queue_diagnostics_if_due(self, now):
        if self.last_diagnostics is not None and now - self.last_diagnostics < 1:
            return
        status = self.model.quality(now)
        ready = status['position_healthy'] and status['heading_healthy'] and status['imu_calibrated'] and status['imu_fresh'] and all((v['fresh'] and v['valid'] for v in status['encoders'].values()))
        self.recorder.emit('console', f'{('READY FOR DATA REVIEW' if ready else 'WAITING FOR SENSOR READINESS')} · IMU {self.model.imu_count} · wheels L/R {self.model.wheels['L']['samples']}/{self.model.wheels['R']['samples']} · GNSS position/heading {status['position_healthy']}/{status['heading_healthy']} · shadow only')
        self.recorder.emit('events', dict(event='diagnostics', utc=utc_now(), quality=status, queue_depths={key: q.qsize() for key, q in self.queues.items()}))
        self.last_diagnostics = now

    def run(self, duration=0):
        start = time.monotonic()
        next_tick = start
        while not self.stop.is_set():
            self.ingest_gps()
            self.ingest_imu()
            self.ingest_encoders()
            self.update_pose()
            now = time.monotonic()
            self.assess_pose_quality(now)
            self.publish_navigation_if_due(now)
            self.queue_diagnostics_if_due(now)
            if duration and now - start >= duration:
                break
            next_tick += 0.05
            if now - next_tick > 0.25:
                raise RuntimeError('Pose loop fell over 250 ms behind schedule')
            self.stop.wait(max(0, next_tick - time.monotonic()))

def load_scales(path):
    if path is None:
        return ({'L': None, 'R': None}, None)
    data = json.loads(path.read_text())
    if data.get('schema') != 1 or data.get('reviewed_calibration') is not True or (not data.get('evidence_reference')):
        raise ValueError('Encoder calibration needs schema=1, reviewed_calibration=true and an evidence_reference')
    scales = {}
    for side, serial_number in (('L', LEFT_SERIAL), ('R', RIGHT_SERIAL)):
        item = data[side]
        value = item['metres_per_sensor_revolution']
        if item['usb_serial'] != serial_number or isinstance(value, bool) or (not isinstance(value, (int, float))) or (not 0 < value < 20):
            raise ValueError('Invalid or wrong-device encoder calibration')
        scales[side] = value
    return (scales, hashlib.sha256(path.read_bytes()).hexdigest())


class BootPositionApp(PositionApp):
    """Calibration requires fresh stationary evidence; readers never block on IPC."""

    def __init__(self, *args, boot_calibration=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.armed = boot_calibration
        self.still_since = None
        self.status = None
        self.last_confirmation = None

    def monitor_readiness(self):
        while not self.stop.is_set():
            try:
                since = time.monotonic() - 1.5
                status = request(dict(op="window", channel="status", since=since))
                events = request(dict(op="window", channel="events", since=since))
                self.emit_sensor("gps", event("readiness", "status", status["records"][-1:] ))
                for record in events["records"]:
                    if record["payload"].get("event") == "engine_off_confirmation":
                        self.emit_sensor("gps", event("readiness", "confirmation", record["received_mono"]))
            except (OSError, RuntimeError, KeyError, ValueError):
                self.emit_sensor("gps", event("readiness", "status", []))
            self.stop.wait(.5)

    def start(self):
        super().start()
        thread = threading.Thread(target=self.worker, args=("readiness", self.monitor_readiness), daemon=True)
        thread.start()
        self.threads.append(thread)

    def update_pose(self):
        self.pending.sort(key=lambda item: item["mono"])
        for item in self.pending:
            if item["source"] == "readiness":
                if item["kind"] == "status":
                    self.status = item["data"][-1] if item["data"] else None
                elif item["data"] != self.last_confirmation:
                    self.last_confirmation = item["data"]
                    if self.model.bias is None:
                        self.armed = True
                continue
            if item["source"] == "imu":
                now = item["mono"]
                status = self.status["payload"] if self.status else {}
                steer, trans = status.get("steering", {}), status.get("transmission", {})
                quality = self.model.quality(now)
                still = (self.armed and self.status is not None and
                         0 <= now - self.status["received_mono"] <= 1 and
                         steer.get("state") == "PAUSE" and steer.get("pwm") == 0 and
                         trans.get("target") == 2836 and trans.get("actual_target") == 2836 and
                         quality["position_healthy"] and
                         isinstance(self.model.gps.get("speed_mps"), (float, int)) and
                         0 <= self.model.gps["speed_mps"] <= .05 and
                         all(w["fresh"] and w["valid"] for w in quality["encoders"].values()) and
                         all(w["last_ticks"] == 0 for w in self.model.wheels.values()))
                if self.model.bias is None:
                    if not still:
                        self.still_since = None
                        self.model.calibration_allowed = False
                        self.model.cal_gyro.clear()
                        self.model.cal_accel.clear()
                    else:
                        self.still_since = self.still_since or now
                        if now - self.still_since >= 3 and not self.model.calibration_allowed:
                            self.model.imu_start = now
                            self.model.calibration_allowed = True
                    previous_imu = self.model.last_imu
                    try:
                        self.model.apply_imu(item)
                    except RuntimeError as exc:
                        self.armed = False
                        self.model.calibration_allowed = False
                        self.model.last_imu = previous_imu
                        # Preserve the raw sample that triggered calibration
                        # rejection, with its real interval and waiting phase.
                        self.model.apply_imu(item)
                        self.recorder.emit("events", dict(event="calibration_failed", error=str(exc)))
                else:
                    self.model.apply_imu(item)
            elif item["source"] in ("base", "heading"):
                self.model.apply_gps(item)
            else:
                self.model.apply_encoder(item)
        self.pending.clear()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--encoder-calibration", type=Path)
    parser.add_argument("--runtime-dir", type=Path, default=Path("/run/tractor-position"))
    args = parser.parse_args(argv)
    stop = threading.Event()
    devices = {}
    bus = None
    app = None
    publisher = Publisher(stop)
    code = 0
    try:
        other_imu = subprocess.run(["pgrep", "-af", "[i]sm330dhcx_field_logger_20261116.py"],
                                   capture_output=True, text=True, timeout=5)
        if other_imu.returncode == 0:
            raise RuntimeError("Standalone IMU logger is running; do not configure one sensor from two programs")
        import serial
        from serial.tools import list_ports
        from smbus2 import SMBus
        scales, calibration_id = load_scales(args.encoder_calibration)
        ports = list(list_ports.comports())
        paths = dict(base="/dev/gps-base-link", heading="/dev/gps-heading",
                     L=encoder_device(ports, LEFT_SERIAL), R=encoder_device(ports, RIGHT_SERIAL))
        resolved = [Path(p).resolve() for p in paths.values()]
        if len(set(resolved)) != 4 or Path("/dev/teensy").resolve() in resolved:
            raise RuntimeError("Serial ownership collision")
        for path in paths.values():
            require_free_device(path)
        for name, path in paths.items():
            devices[name] = serial.Serial(path, 115200, timeout=.05, write_timeout=.5, exclusive=True)
        bus = SMBus(1)
        args.runtime_dir.mkdir(parents=True, exist_ok=True)
        token = args.runtime_dir / "calibration-started"
        # RuntimeDirectoryPreserve=yes retains this through service restarts,
        # while /run is cleared by an actual boot.
        boot_calibration = not token.exists()
        if boot_calibration:
            token.write_text(utc_now(), encoding="utf-8")
        publisher.thread.start()
        publisher.emit("events", dict(event="producer_start", devices=paths,
                       left_serial=LEFT_SERIAL, right_serial=RIGHT_SERIAL, scales=scales,
                       calibration_id=calibration_id, boot_engine_off_assumed=boot_calibration,
                       note="Stationarity is measured; engine-off is an operating prerequisite, not detected"))
        app = BootPositionApp(devices, bus, publisher, stop, scales, calibration_id,
                              boot_calibration=boot_calibration)
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stop.set())
        app.start()
        app.run()
        if app.fault or publisher.error:
            raise RuntimeError(app.fault or publisher.error)
    except Exception as exc:
        code = 1
        print(f"Position producer failed: {exc}", flush=True)
    finally:
        stop.set()
        if app:
            for thread in app.threads:
                thread.join(4)
            app.ingest_gps()
            app.ingest_imu()
            app.ingest_encoders()
            app.update_pose()
            if app.udp:
                app.udp.close()
        publisher.finishing.set()
        if publisher.thread.ident:
            publisher.thread.join(5)
        if publisher.error or publisher.thread.is_alive():
            code = 1
        for device in devices.values():
            device.close()
        if bus is not None:
            bus.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
