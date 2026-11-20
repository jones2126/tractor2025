"""DRAFT field data-flow replacement for the RTCM server; shadow pose only."""
from __future__ import annotations

import argparse
import copy
from contextlib import nullcontext
import csv
import datetime as dt
import hashlib
import json
from pathlib import Path
import queue
import signal
import socket
import subprocess
import threading
import time

from tractor_rpi.archive.pose_producer_20261117 import position_gnss_protocol_20261117 as protocol
from tractor_rpi.archive.pose_producer_20261117.position_model_20261117 import (
    LEFT_SERIAL, RIGHT_SERIAL, PORTS, POSITION_IMU_FIELDS, PROVISIONAL_METRES_PER_SENSOR_REV,
    PoseModel, parse_base_nmea, parse_encoder,
)
from tractor_rpi.testing.imu.ism330dhcx_field_logger_20261116 import Sensor

ENCODER_FIELDS=("utc","monotonic_s","side","device_ms","position","total_rot","delta_ticks",
                "reported_speed_mps","reported_delta_m","computed_delta_m","scale_calibrated","contiguous","raw")


def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="microseconds")


def event(source,kind,data):
    return dict(source=source,kind=kind,data=data,mono=time.monotonic(),utc=utc_now())


def require_free_device(path):
    result=subprocess.run(["fuser",str(path)],capture_output=True,text=True,timeout=5)
    if result.returncode==0:raise RuntimeError(f"Device already owned: {path}; do not run beside the original server/logger")
    if result.returncode!=1:raise RuntimeError(f"Cannot establish device ownership: {path}: {result.stderr}")


def encoder_device(ports,serial_number):
    matches=[p.device for p in ports if p.serial_number==serial_number]
    if len(matches)!=1:raise RuntimeError(f"Expected exactly one encoder with USB serial {serial_number}; found {matches}")
    return matches[0]


class Framer:
    """Bounded mixed NMEA/UBX serial decoder; handles partial and joined reads."""
    def __init__(self):self.buffer=bytearray()

    def feed(self,block):
        self.buffer.extend(block)
        if len(self.buffer)>8192:raise ValueError("GNSS framing buffer overflow")
        frames=[]
        while self.buffer:
            if self.buffer[0]==36:
                end=self.buffer.find(b"\n")
                if end<0:
                    if len(self.buffer)>512:del self.buffer[0];continue
                    break
                frames.append(("nmea",bytes(self.buffer[:end+1]).strip()));del self.buffer[:end+1]
            elif self.buffer[0]==protocol.SYNC1:
                if len(self.buffer)<2:break
                if self.buffer[1]!=protocol.SYNC2:del self.buffer[0];continue
                if len(self.buffer)<6:break
                length=self.buffer[4]|self.buffer[5]<<8
                if length>4096:del self.buffer[0];continue
                if len(self.buffer)<length+8:break
                frame=bytes(self.buffer[:length+8]);del self.buffer[:length+8]
                if tuple(frame[-2:])==protocol.ubx_checksum(frame[2:-2]):frames.append(("ubx",frame))
            else:del self.buffer[0]
        return frames


class Recorder:
    """One file-owning thread. Any log overflow/write failure stops capture."""
    def __init__(self,directory,stop):
        self.directory=directory;self.stop=stop
        self.queue=queue.Queue(maxsize=32768)
        self.finishing=threading.Event();self.error=None
        self.rows={key:0 for key in ("imu","encoders","gnss","pose","events")}

    def emit(self,channel,payload):
        try:self.queue.put_nowait((channel,copy.deepcopy(payload)))
        except queue.Full:
            self.error="Diagnostic evidence queue overflow; no silent record loss permitted"
            self.stop.set()

    def run(self):
        handles={}
        try:
            for channel,name in (("imu","samples.csv"),("encoders","encoders.csv"),("gnss","gnss.jsonl"),
                                 ("pose","shadow_pose.jsonl"),("events","events.jsonl")):
                handles[channel]=(self.directory/name).open("x",newline="",encoding="utf-8")
            writers={"imu":csv.DictWriter(handles["imu"],POSITION_IMU_FIELDS),
                     "encoders":csv.DictWriter(handles["encoders"],ENCODER_FIELDS)}
            for writer in writers.values():writer.writeheader()
            last_flush=time.monotonic()
            while not self.finishing.is_set() or not self.queue.empty():
                try:channel,payload=self.queue.get(timeout=.1)
                except queue.Empty:channel=None
                if channel:
                    if channel=="console":print(payload,flush=True)
                    else:
                        if channel in writers:writers[channel].writerow(payload)
                        else:handles[channel].write(json.dumps(payload,allow_nan=False,separators=(",",":"))+"\n")
                        self.rows[channel]+=1
                if time.monotonic()-last_flush>=1:
                    for handle in handles.values():handle.flush()
                    last_flush=time.monotonic()
        except Exception as exc:
            self.error=f"Evidence write failure: {exc}";self.stop.set()
        finally:
            for handle in handles.values():handle.close()


class PositionApp:
    def __init__(self,devices,bus,recorder,stop,scales=None,calibration_id=None,udp=None):
        self.devices=devices;self.bus=bus;self.recorder=recorder;self.stop=stop
        self.queues={name:queue.Queue(maxsize=4096) for name in ("gps","imu","encoders")}
        self.pending=[];self.serial_write_lock=threading.Lock()
        self.model=PoseModel(recorder.emit,scales,calibration_id)
        self.threads=[];self.fault=None
        self.udp=udp
        self.sequence=0;self.last_publish=None;self.last_diagnostics=None
        self.udp_sent={port:0 for port in PORTS}

    def emit_sensor(self,family,value):
        try:self.queues[family].put_nowait(value)
        except queue.Full:raise RuntimeError(f"{family} measurement queue overflow")

    def worker(self,name,function,*args):
        try:function(*args)
        except Exception as exc:
            self.fault=f"{name}: {exc}"
            self.recorder.emit("events",dict(utc=utc_now(),event="worker_failed",worker=name,error=str(exc)))
            self.stop.set()

    def read_gps(self,side):
        device=self.devices[side];framer=Framer();last_poll=0.
        while not self.stop.is_set():
            now=time.monotonic()
            if now-last_poll>=1:
                with self.serial_write_lock if side=="base" else nullcontext():
                    polls=protocol.ubx_frame(protocol.CLS_NAV,protocol.ID_PVT)+protocol.ubx_frame(protocol.CLS_NAV,protocol.ID_SAT)
                    if device.write(polls)!=len(polls):raise RuntimeError("Incomplete diagnostic poll write")
                last_poll=now
            block=device.read(max(1,min(512,device.in_waiting)))
            for kind,data in framer.feed(block):
                result=None
                if kind=="nmea":
                    if side=="base":result=parse_base_nmea(data)
                    else:
                        sentence=protocol.parse_checksum_valid_nmea_type(data)
                        if sentence:result=("heading_nmea",sentence)
                elif data[2]==protocol.CLS_NAV:
                    message,payload=data[3],data[6:-2]
                    if side=="heading" and message==protocol.ID_RELPOSNED:
                        decoded=protocol.parse_relposned(payload)
                        if decoded:result=("heading",decoded)
                    elif message in (protocol.ID_PVT,protocol.ID_SAT):
                        decoded=protocol.parse_nav_pvt_satellites(payload) if message==protocol.ID_PVT else protocol.parse_nav_sat(payload)
                        if decoded:result=("satellites",decoded)
                if result:self.emit_sensor("gps",event(side,*result))

    def read_gps_base(self):self.read_gps("base")
    def read_gps_heading(self):self.read_gps("heading")

    def read_imu(self):
        sensor=Sensor(self.bus);sensor.configure();last=time.monotonic()
        while not self.stop.is_set():
            reading=sensor.read()
            if reading is not None:
                last=time.monotonic();self.emit_sensor("imu",event("imu","sample",reading))
            else:
                if time.monotonic()-last>1:raise RuntimeError("No fresh IMU sample for one second")
                self.stop.wait(.001)

    def read_encoder(self,side):
        device=self.devices[side];buffer=bytearray();last=time.monotonic()
        while not self.stop.is_set():
            block=device.read(max(1,min(512,device.in_waiting)))
            if block:last=time.monotonic();buffer.extend(block)
            if len(buffer)>2048:raise RuntimeError("Encoder framing overflow")
            while b"\n" in buffer:
                line,_,remaining=buffer.partition(b"\n");buffer=bytearray(remaining)
                try:kind,data=parse_encoder(line,side)
                except (ValueError,UnicodeDecodeError) as exc:
                    kind,data="parse_error",dict(error=str(exc),raw=line.decode("ascii",errors="replace"))
                self.emit_sensor("encoders",event(side,kind,data))
            if time.monotonic()-last>5:raise RuntimeError(f"Encoder {side} has produced no serial data for 5 seconds")

    def forward_rtcm_corrections(self):
        total=0;interval_bytes=0;last_log=time.monotonic()
        while not self.stop.is_set():
            try:
                with socket.create_connection(("192.168.193.88",6001),timeout=3) as connection:
                    connection.settimeout(1)
                    self.recorder.emit("console","[RTCM Forwarder] CONNECTED to 192.168.193.88:6001")
                    while not self.stop.is_set():
                        try:data=connection.recv(4096)
                        except socket.timeout:continue
                        if not data:raise ConnectionError("Correction source closed connection")
                        with self.serial_write_lock:
                            written=self.devices["base"].write(data)
                        if written!=len(data):raise RuntimeError("Incomplete RTCM serial write")
                        total+=written;interval_bytes+=written
                        elapsed=time.monotonic()-last_log
                        if elapsed>=10:
                            self.recorder.emit("console",f"[RTCM Forwarder] Forwarded {interval_bytes} bytes (~{interval_bytes/elapsed:.0f} B/s) in last {elapsed:.1f}s | Total forwarded: {total}")
                            self.recorder.emit("events",dict(event="rtcm_forwarding",utc=utc_now(),bytes=interval_bytes,elapsed_s=elapsed,total_bytes=total))
                            interval_bytes=0;last_log=time.monotonic()
            except OSError as exc:
                self.recorder.emit("console",f"[RTCM Forwarder] Connection error: {exc}; reconnecting in 5 seconds")
                self.recorder.emit("events",dict(event="rtcm_connection_error",utc=utc_now(),error=str(exc)))
                self.stop.wait(5)

    def start(self):
        tasks=[("gps-base",self.read_gps_base,()),("gps-heading",self.read_gps_heading,()),
               ("imu",self.read_imu,()),("encoder-L",self.read_encoder,("L",)),
               ("encoder-R",self.read_encoder,("R",)),("rtcm",self.forward_rtcm_corrections,())]
        for name,function,args in tasks:
            thread=threading.Thread(target=self.worker,args=(name,function,*args),name=name,daemon=True)
            thread.start();self.threads.append(thread)

    def drain(self,family):
        channel=self.queues[family]
        for _ in range(channel.maxsize):
            try:self.pending.append(channel.get_nowait())
            except queue.Empty:break

    def ingest_gps(self):self.drain("gps")
    def ingest_imu(self):self.drain("imu")
    def ingest_encoders(self):self.drain("encoders")

    def update_pose(self):
        self.pending.sort(key=lambda item:item["mono"])
        for item in self.pending:
            if item["source"] in ("base","heading"):self.model.apply_gps(item)
            elif item["source"]=="imu":self.model.apply_imu(item)
            else:self.model.apply_encoder(item)
        self.pending.clear()

    def assess_pose_quality(self,now):return self.model.quality(now)

    def publish_navigation_if_due(self,now):
        if self.last_publish is not None and now-self.last_publish<.049:return
        if self.udp is None:self.udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.udp.setblocking(False)
        payload=json.dumps(self.model.gps,allow_nan=False,separators=(",",":")).encode()
        # Existing field logger reads only 2048 bytes. Sensor/pose records must
        # never be bolted onto the legacy packets and silently truncated.
        if len(payload)>2048:raise RuntimeError("Legacy GPS snapshot exceeds field logger receive size")
        for port in PORTS:
            if self.udp.sendto(payload,("127.0.0.1",port))!=len(payload):raise RuntimeError("Incomplete UDP datagram send")
            self.udp_sent[port]+=1
        record=dict(utc=utc_now(),monotonic_s=now,sequence=self.sequence)
        self.recorder.emit("gnss",{**record,"message":self.model.gps})
        self.recorder.emit("pose",{**record,"pose":self.model.snapshot(now)})
        self.sequence+=1;self.last_publish=now

    def queue_diagnostics_if_due(self,now):
        if self.last_diagnostics is not None and now-self.last_diagnostics<1:return
        status=self.model.quality(now)
        ready=status["position_healthy"] and status["heading_healthy"] and status["imu_calibrated"] and status["imu_fresh"] and all(v["fresh"] and v["valid"] for v in status["encoders"].values())
        self.recorder.emit("console",f"{'READY FOR DATA REVIEW' if ready else 'WAITING FOR SENSOR READINESS'} · IMU {self.model.imu_count} · wheels L/R {self.model.wheels['L']['samples']}/{self.model.wheels['R']['samples']} · GNSS position/heading {status['position_healthy']}/{status['heading_healthy']} · shadow only")
        self.recorder.emit("events",dict(event="diagnostics",utc=utc_now(),quality=status,
            queue_depths={key:q.qsize() for key,q in self.queues.items()}))
        self.last_diagnostics=now

    def run(self,duration=0):
        start=time.monotonic();next_tick=start
        while not self.stop.is_set():
            self.ingest_gps()
            self.ingest_imu()
            self.ingest_encoders()
            self.update_pose()
            now=time.monotonic()
            self.assess_pose_quality(now)
            self.publish_navigation_if_due(now)
            self.queue_diagnostics_if_due(now)
            if duration and now-start>=duration:break
            next_tick+=.05
            if now-next_tick>.25:raise RuntimeError("Pose loop fell over 250 ms behind schedule")
            self.stop.wait(max(0,next_tick-time.monotonic()))


def repo_metadata():
    root=Path(__file__).resolve().parents[3]
    git=lambda *args:subprocess.check_output(["git","-C",str(root),*args],text=True,timeout=5).strip()
    if git("status","--porcelain"):raise RuntimeError("Use a clean committed checkout for field capture")
    files=[Path(__file__),Path(__file__).with_name("position_model_20261117.py"),Path(__file__).with_name("position_gnss_protocol_20261117.py"),
           Path(__file__).with_name("calc-publish-position-20261117.py"),root/"tractor_rpi/testing/imu/ism330dhcx_field_logger_20261116.py"]
    return dict(repository=str(root),commit=git("rev-parse","HEAD"),source_hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})


def save_metadata(path,data):
    temporary=path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    temporary.replace(path)


def load_scales(path):
    if path is None:return {"L":None,"R":None},None
    data=json.loads(path.read_text())
    if data.get("schema")!=1 or data.get("reviewed_calibration") is not True or not data.get("evidence_reference"):
        raise ValueError("Encoder calibration needs schema=1, reviewed_calibration=true and an evidence_reference")
    scales={}
    for side,serial_number in (("L",LEFT_SERIAL),("R",RIGHT_SERIAL)):
        item=data[side]
        value=item["metres_per_sensor_revolution"]
        if item["usb_serial"]!=serial_number or isinstance(value,bool) or not isinstance(value,(int,float)) or not 0<value<20:
            raise ValueError("Invalid or wrong-device encoder calibration")
        scales[side]=value
    return scales,hashlib.sha256(path.read_bytes()).hexdigest()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stationary",action="store_true",help="Confirm engine-off stationary IMU calibration startup")
    parser.add_argument("--output-dir",type=Path,default=Path("/home/al/field_logs/position"))
    parser.add_argument("--encoder-calibration",type=Path)
    parser.add_argument("--duration",type=float,default=0,help="Total capture seconds; zero runs until stopped")
    args=parser.parse_args(argv)
    if not args.stationary:parser.error("--stationary requires operator-confirmed engine-off stillness at startup")
    if not 0<=args.duration<86400:parser.error("Invalid duration")
    stop=threading.Event();devices={};bus=None;recorder=None;writer=None;app=None;directory=None
    metadata={};exit_code=1
    try:
        metadata=repo_metadata()
        scales,calibration_id=load_scales(args.encoder_calibration)
        import serial
        from serial.tools import list_ports
        from smbus2 import SMBus
        ports=list(list_ports.comports())
        paths={"base":"/dev/gps-base-link","heading":"/dev/gps-heading",
               "L":encoder_device(ports,LEFT_SERIAL),"R":encoder_device(ports,RIGHT_SERIAL)}
        resolved=[Path(path).resolve() for path in paths.values()]
        if len(set(resolved))!=4 or Path("/dev/teensy").resolve() in resolved:
            raise RuntimeError("Device collision or attempted ownership of control Teensy")
        for path in paths.values():require_free_device(path)
        directory=args.output_dir.expanduser()/dt.datetime.now(dt.timezone.utc).strftime("position_%Y%m%dT%H%M%S_%fZ")
        directory.mkdir(parents=True,exist_ok=False)
        metadata.update(schema=1,status="STARTING",started_utc=utc_now(),devices=paths,
            left_serial=LEFT_SERIAL,right_serial=RIGHT_SERIAL,encoder_scales=scales,encoder_calibration_id=calibration_id,
            odometry_scale_verified=all(scales.values()),gnss_udp_ports=list(PORTS),publish_hz=20,
            provisional_metres_per_sensor_rev=PROVISIONAL_METRES_PER_SENSOR_REV,
            imu_settings={"bus":1,"address":"0x6b","sample_hz":104,"calibration_settle_s":1,"calibration_s":10},
            pose_policy="SHADOW ONLY; 5-second healthy heading anchors; legacy ports remain GNSS observations",
            note="No degraded-motion authorization, no watchdog changes; unquantified pose uncertainty")
        if args.encoder_calibration:metadata["encoder_calibration_document"]=json.loads(args.encoder_calibration.read_text())
        save_metadata(directory/"metadata.json",metadata)
        print(f"Evidence directory: {directory}",flush=True)
        for name,path in paths.items():devices[name]=serial.Serial(path,115200,timeout=.05,write_timeout=.5,exclusive=True)
        bus=SMBus(1)
        recorder=Recorder(directory,stop)
        writer=threading.Thread(target=recorder.run,name="diagnostics",daemon=True);writer.start()
        app=PositionApp(devices,bus,recorder,stop,scales,calibration_id)
        def request_stop(*_):stop.set()
        old_signals={sig:signal.signal(sig,request_stop) for sig in (signal.SIGINT,signal.SIGTERM)}
        try:
            app.start();metadata["status"]="RECORDING"
            app.run(args.duration)
            if app.fault or recorder.error:raise RuntimeError(app.fault or recorder.error)
            if app.model.bias is None or any(w["samples"]<2 for w in app.model.wheels.values()):
                raise RuntimeError("Capture ended before IMU calibration and both encoder streams were established")
            metadata["status"]="STOPPED";exit_code=0
        finally:
            for sig,handler in old_signals.items():signal.signal(sig,handler)
    except Exception as exc:
        metadata.update(status="FAILED",error=str(exc));print(f"POSITION CAPTURE FAILED: {exc}",flush=True)
    finally:
        stop.set()
        if app:
            deadline=time.monotonic()+8
            for thread in app.threads:thread.join(max(0,deadline-time.monotonic()))
            if app.fault:metadata.update(status="FAILED",error=app.fault);exit_code=1
            if any(t.is_alive() for t in app.threads):metadata.update(status="INCOMPLETE",error="Sensor worker did not stop");exit_code=1
            try:
                app.ingest_gps();app.ingest_imu();app.ingest_encoders();app.update_pose()
            except Exception as exc:
                metadata.update(status="INCOMPLETE",error=f"Final measurement drain failed: {exc}");exit_code=1
            metadata.update(imu_calibration=app.model.calibration,final_quality=app.model.quality(time.monotonic()),
                            imu_rows=app.model.imu_count,yaw_segments=app.model.yaw.segment+1,udp_datagrams_sent=app.udp_sent)
            if app.udp:app.udp.close()
        if recorder:
            recorder.finishing.set()
            if writer:writer.join(10)
            if writer and writer.is_alive() or recorder.error:
                metadata.update(status="INCOMPLETE",error=recorder.error or "Evidence writer did not stop");exit_code=1
            metadata["record_counts"]=recorder.rows
        for device in devices.values():device.close()
        if bus is not None:bus.close()
        if directory:
            metadata["ended_utc"]=utc_now();save_metadata(directory/"metadata.json",metadata)
            if not writer or not writer.is_alive():
                hashes=[]
                for path in sorted(directory.iterdir()):
                    if path.is_file() and path.name!="SHA256SUMS":
                        hasher=hashlib.sha256()
                        with path.open("rb") as handle:
                            for block in iter(lambda:handle.read(1048576),b""):hasher.update(block)
                        hashes.append(hasher.hexdigest()+"  "+path.name)
                (directory/"SHA256SUMS").write_text("\n".join(hashes)+"\n",encoding="utf-8")
            print(f"Saved {metadata['status']}: {directory}",flush=True)
    return exit_code
