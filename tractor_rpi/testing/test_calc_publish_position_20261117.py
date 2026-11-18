"""Hardware-free tests for the candidate pose producer and compatibility feeds."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import queue
import struct
import tempfile
import threading
import unittest
from unittest.mock import patch

from tractor_rpi import calc_publish_position_20261117 as runtime
from tractor_rpi import position_gnss_protocol_20261117 as protocol
from tractor_rpi.position_model_20261117 import PoseModel, parse_base_nmea, parse_encoder, PORTS, LEFT_SERIAL
from tractor_rpi.testing.verify_position_capture_20261117 import verify


def nmea(body):
    checksum=0
    for char in body.encode():checksum^=char
    return f"${body}*{checksum:02X}\r\n".encode()


def sample(source,kind,data,t):
    return dict(source=source,kind=kind,data=data,mono=t,utc=f"2026-11-18T12:00:{t%60:09.6f}+00:00")


def healthy(model,t,fix="RTK Fixed",heading=350.):
    model.apply_gps(sample("base","gga",dict(lat=40.,lon=-80.,fix_quality=fix,hdop=.7,diff_age=1.2,base_numSV_used_gga=31),t))
    model.apply_gps(sample("heading","heading",dict(headValid=True,carrier="fixed",iTOW_ms=int(t*1000),
        length_m=1.05,accHead_deg=.5,gnssFixOK=True,diffSoln=True,relPosValid=True,isMoving=True,
        refPosMiss=False,refObsMiss=False,relPosNormalized=False,heading_deg=heading,accN_m=.01,accE_m=.01),t))


def calibrate(model):
    for i in range(1120):
        t=i/100
        if i%20==0:healthy(model,t)
        model.apply_imu(sample("imu","sample",((1,2,3,0,0,16000),(.1,-.5,.03),(0.,0.,1.)),t))


def enc(side,t,ticks=100,device_ms=None):
    return sample(side,"sample",dict(device_ms=int(t*1000) if device_ms is None else device_ms,
        position=4000,total_rot=1.,delta_ticks=ticks,reported_speed_mps=.1,reported_delta_m=.01,raw="ENC fixture"),t)


class MemoryRecorder:
    def __init__(self):self.records=[]
    def emit(self,channel,payload):self.records.append((channel,copy.deepcopy(payload)))


class FakeUdp:
    def __init__(self):self.sent=[]
    def sendto(self,data,address):self.sent.append((data,address));return len(data)


class CandidateTests(unittest.TestCase):
    def test_legacy_schema_is_preserved_except_requested_wifi_removal(self):
        root=Path(__file__).resolve().parents[2]
        tree=ast.parse((root/"tractor_rpi/rtcm_server_20260828.py").read_text(encoding="utf-8"))
        state=next(n.value for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(x,ast.Name) and x.id=="state" for x in n.targets))
        old={key.value for key in state.keys}
        self.assertEqual(set(protocol.initial_state()),old-{"wifi_ssid","wifi_rssi_dbm","wifi_signal_label"})

    def test_checked_gga_speed_and_invalid_sentence(self):
        line=nmea("GNGGA,123519,4029.14828,N,08019.93709,W,4,31,0.7,212.0,M,-30.0,M,1.2,0000")
        kind,data=parse_base_nmea(line)
        self.assertEqual(kind,"gga");self.assertEqual(data["fix_quality"],"RTK Fixed")
        self.assertAlmostEqual(data["lat"],40.4858046666667)
        self.assertAlmostEqual(data["lon"],-80.3322848333333)
        self.assertIsNone(parse_base_nmea(line.replace(b"*",b"*FF")))
        self.assertEqual(parse_base_nmea(nmea("GNVTG,0,T,0,M,1.0,N,3.6,K,A"))[1]["speed_mps"],1.)
        self.assertIsNone(parse_base_nmea(nmea("GNRMC,123519,V,4029,N,08019,W,1.0")))
        self.assertIsNone(parse_base_nmea(nmea("GNGGA,é")))

    def test_mixed_framer_partial_and_bad_checksum(self):
        framer=runtime.Framer();n=nmea("GNGGA,123519,4029,N,08019,W,4,31,0.7,0,M,0,M,1,0")
        frame=protocol.ubx_frame(1,7,b"payload")
        self.assertEqual(framer.feed(frame[:3]),[])
        result=framer.feed(frame[3:]+n)
        self.assertEqual([kind for kind,_ in result],["ubx","nmea"])
        bad=frame[:-1]+bytes([frame[-1]^1])
        self.assertEqual(framer.feed(bad),[])

    def test_encoder_side_numbers_and_warning_preserved(self):
        kind,data=parse_encoder(b"ENC,L,100,5000,0.1234,-100,-0.010,-0.0010", "L")
        self.assertEqual(kind,"sample");self.assertEqual(data["delta_ticks"],-100)
        for line in (b"ENC,R,100,5000,0,0,0,0",b"ENC,L,100,5000,nan,0,0,0",b"ENC,L,100,5000,0,99999,0,0"):
            with self.assertRaises(ValueError):parse_encoder(line,"L")
        self.assertEqual(parse_encoder(b"WARN,L,200,gforce>2 pos=5000","L")[0],"warn")

    def test_calibration_full_rate_logging_and_shadow_only(self):
        log=MemoryRecorder();model=PoseModel(log.emit);calibrate(model)
        self.assertAlmostEqual(model.bias[2],.03)
        self.assertEqual(len([r for c,r in log.records if c=="imu"]),1120)
        row=[r for c,r in log.records if c=="imu"][-1]
        self.assertEqual(row["phase"],"observation")
        self.assertAlmostEqual(row["gyro_z_corrected_dps"],0)
        self.assertEqual(row["gnss_input_transport"],"direct_serial")
        self.assertIsNone(row["gnss_socket_available"])
        self.assertFalse(model.snapshot(11.19)["valid_for_control"])

    def test_motion_during_calibration_rejected(self):
        model=PoseModel(lambda *_:None)
        with self.assertRaisesRegex(RuntimeError,"calibration failed"):
            for i in range(1110):model.apply_imu(sample("imu","sample",((0,)*6,(0.,0.,2. if i%2 else -2.),(0.,0.,1.)),i/100))

    def test_five_second_anchors_are_separate_from_relative_yaw(self):
        log=MemoryRecorder();model=PoseModel(log.emit);calibrate(model)
        first=model.heading_anchor
        for i in range(1120,1720):
            t=i/100
            if i%20==0:healthy(model,t,heading=100.)
            model.apply_imu(sample("imu","sample",((0,)*6,(.1,-.5,1.03),(0.,0.,1.)),t))
        anchors=[r for c,r in log.records if c=="events" and r.get("event")=="shadow_heading_anchor"]
        self.assertEqual(len(anchors),2)
        self.assertGreaterEqual(anchors[1]["mono"]-first,5.)
        self.assertGreater(model.yaw.yaw,5.)
        self.assertFalse(model.snapshot(17.19)["valid_for_control"])

    def test_gps_degradation_does_not_turn_legacy_heading_into_imu(self):
        model=PoseModel(lambda *_:None);calibrate(model)
        healthy(model,11.2,fix="RTK Float",heading=350.)
        old_anchor=model.heading_anchor
        for i in range(1120,1220):model.apply_imu(sample("imu","sample",((0,)*6,(.1,-.5,10.03),(0.,0.,1.)),i/100))
        self.assertAlmostEqual(model.gps["heading_deg"],350.)
        self.assertAlmostEqual(model.heading,340.,places=1)
        self.assertEqual(model.heading_anchor,old_anchor)
        self.assertEqual(model.gps["fix_quality"],"RTK Float")

    def test_gap_invalidates_shadow_anchor(self):
        model=PoseModel(lambda *_:None);calibrate(model)
        model.apply_imu(sample("imu","sample",((0,)*6,(.1,-.5,.03),(0.,0.,1.)),13.))
        self.assertIsNone(model.heading)
        self.assertEqual(model.yaw.segment,1)

    def test_encoder_error_and_reset_invalidate_continuity(self):
        model=PoseModel(lambda *_:None);healthy(model,1.)
        for side in ("L","R"):
            model.apply_encoder(enc(side,1.));model.apply_encoder(enc(side,1.1))
        self.assertTrue(model.wheels["L"]["valid"])
        model.apply_encoder(sample("L","err",dict(device_ms=1200,raw="i2c failure"),1.2))
        model.apply_encoder(enc("L",1.3))
        self.assertFalse(model.wheels["L"]["continuous"])
        model.apply_encoder(enc("L",1.4,device_ms=100))
        self.assertFalse(model.wheels["L"]["valid"])

    def test_no_calibration_means_no_trusted_distance_fallback(self):
        model=PoseModel(lambda *_:None);healthy(model,1.);model.heading=0.;model.last_imu=1.
        for side in ("L","R"):
            model.apply_encoder(enc(side,1.));model.apply_encoder(enc(side,1.1))
        old=model.position
        for side in ("L","R"):model.apply_encoder(enc(side,1.2))
        self.assertEqual(model.position,old)
        self.assertFalse(model.snapshot(2.2)["position_supported"])

    def test_calibrated_signed_travel_uses_both_wheels_and_does_not_wait_for_expiry(self):
        model=PoseModel(lambda *_:None,{"L":1.,"R":1.},"fixture");healthy(model,1.);model.heading=0.;model.last_imu=1.
        for side in ("L","R"):model.apply_encoder(enc(side,1.,ticks=0))
        for side in ("L","R"):model.apply_encoder(enc(side,1.1,ticks=1638))
        self.assertGreater(model.position[0],40.)
        self.assertAlmostEqual((model.position[0]-40.)*110540,1638/16384,places=7)
        for side in ("L","R"):model.apply_encoder(enc(side,1.2,ticks=-1638))
        self.assertAlmostEqual(model.position[0],40.)
        self.assertAlmostEqual(model.travel_since_anchor,2*1638/16384)

    def test_four_identical_packets_and_no_extra_sensor_schema(self):
        recorder=MemoryRecorder();udp=FakeUdp()
        app=runtime.PositionApp({},None,recorder,threading.Event(),udp=udp)
        healthy(app.model,1.)
        app.publish_navigation_if_due(1.)
        self.assertEqual([a[1] for _,a in udp.sent],list(PORTS))
        self.assertEqual(len(set(data for data,_ in udp.sent)),1)
        decoded=json.loads(udp.sent[0][0])
        self.assertEqual(set(decoded),set(protocol.initial_state()))
        self.assertLess(len(udp.sent[0][0]),2048)
        app.publish_navigation_if_due(1.02);self.assertEqual(len(udp.sent),4)
        app.publish_navigation_if_due(1.05);self.assertEqual(len(udp.sent),8)

    def test_oversized_compatibility_packet_is_not_silently_truncated(self):
        app=runtime.PositionApp({},None,MemoryRecorder(),threading.Event(),udp=FakeUdp())
        app.model.gps["heading_nmea_types"]={"X"+str(i):1 for i in range(500)}
        with self.assertRaisesRegex(RuntimeError,"receive size"):app.publish_navigation_if_due(1.)

    def test_zero_inner_wheel_increment_still_pairs_with_outer_wheel(self):
        model=PoseModel(lambda *_:None,{"L":1.,"R":1.},"fixture");healthy(model,1.);model.heading=0.;model.last_imu=1.
        for side in ("L","R"):model.apply_encoder(enc(side,1.,ticks=0))
        model.apply_encoder(enc("L",1.1,ticks=0));model.apply_encoder(enc("R",1.1,ticks=1638))
        self.assertAlmostEqual((model.position[0]-40.)*110540,.5*1638/16384,places=7)

    def test_encoder_device_clock_uint32_wrap_is_not_a_reset(self):
        model=PoseModel(lambda *_:None);healthy(model,1.)
        model.apply_encoder(enc("L",1.,device_ms=0xFFFFFFCE))
        model.apply_encoder(enc("L",1.1,device_ms=50))
        self.assertTrue(model.wheels["L"]["valid"])

    def test_base_reader_polling_and_framing_with_fake_serial(self):
        stop=threading.Event();payload=nmea("GNGGA,123519,4029.14828,N,08019.93709,W,4,31,0.7,212,M,-30,M,1.2,0000")
        class Device:
            in_waiting=len(payload)
            def __init__(self):self.writes=[];self.done=False
            def write(self,data):self.writes.append(data);return len(data)
            def read(self,size):
                if self.done:stop.set();return b""
                self.done=True;return payload
        device=Device();app=runtime.PositionApp({"base":device},None,MemoryRecorder(),stop,udp=FakeUdp())
        app.read_gps_base();app.ingest_gps();app.update_pose()
        self.assertEqual(app.model.gps["fix_quality"],"RTK Fixed")
        self.assertEqual(device.writes,[protocol.ubx_frame(1,7)+protocol.ubx_frame(1,0x35)])

    def test_encoder_reader_preserves_warn_and_enc_with_fake_serial(self):
        stop=threading.Event();payload=b"WARN,L,100,gforce>2\nENC,L,100,4000,0,0,0,0\n"
        class Device:
            in_waiting=len(payload)
            done=False
            def read(self,size):
                if self.done:stop.set();return b""
                self.done=True;return payload
        app=runtime.PositionApp({"L":Device()},None,MemoryRecorder(),stop,udp=FakeUdp())
        app.read_encoder("L")
        self.assertEqual([app.queues["encoders"].get()["kind"] for _ in range(2)],["warn","sample"])

    def test_heading_reader_decodes_relposned_with_fake_serial(self):
        stop=threading.Event();payload=bytearray(64)
        struct.pack_into("<Iiii",payload,4,1000,0,105,0)
        struct.pack_into("<ii",payload,20,105,9000000)
        struct.pack_into("<III",payload,36,100,100,100)
        struct.pack_into("<II",payload,48,100,50000)
        struct.pack_into("<I",payload,60,1|2|4|(2<<3)|(1<<5)|(1<<8))
        frame=protocol.ubx_frame(1,0x3C,payload)
        class Device:
            in_waiting=len(frame)
            done=False
            def write(self,data):return len(data)
            def read(self,size):
                if self.done:stop.set();return b""
                self.done=True;return frame
        app=runtime.PositionApp({"heading":Device()},None,MemoryRecorder(),stop,udp=FakeUdp())
        app.read_gps_heading();app.ingest_gps();app.update_pose()
        self.assertAlmostEqual(app.model.gps["heading_deg"],90.)
        self.assertEqual(app.model.gps["carrier"],"fixed")
        self.assertEqual(app.model.gps["relposned_count"],1)

    def test_rtcm_worker_forwards_bytes_with_fake_tcp(self):
        stop=threading.Event();data=b"RTCM bytes fixture"
        class Base:
            def __init__(self):self.written=[]
            def write(self,block):self.written.append(block);return len(block)
        class Connection:
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def settimeout(self,value):pass
            def recv(self,size):stop.set();return data
        base=Base();app=runtime.PositionApp({"base":base},None,MemoryRecorder(),stop,udp=FakeUdp())
        with patch.object(runtime.socket,"create_connection",return_value=Connection()):app.forward_rtcm_corrections()
        self.assertEqual(base.written,[data])

    def test_ingestion_keeps_all_samples_and_orders_cross_sensor_batch(self):
        app=runtime.PositionApp({},None,MemoryRecorder(),threading.Event(),udp=FakeUdp())
        app.emit_sensor("imu",sample("imu","sample",((0,)*6,(0.,0.,0.),(0.,0.,1.)),1.))
        app.emit_sensor("imu",sample("imu","sample",((0,)*6,(0.,0.,0.),(0.,0.,1.)),1.01))
        app.ingest_gps();app.ingest_imu();app.ingest_encoders();app.update_pose()
        self.assertEqual(app.model.imu_count,2)

    def test_recorder_writes_capture_and_closes(self):
        with tempfile.TemporaryDirectory() as directory:
            stop=threading.Event();recorder=runtime.Recorder(Path(directory),stop)
            thread=threading.Thread(target=recorder.run);thread.start()
            recorder.emit("events",dict(event="fixture"));recorder.finishing.set();thread.join(3)
            self.assertFalse(thread.is_alive());self.assertIsNone(recorder.error)
            self.assertEqual(json.loads((Path(directory)/"events.jsonl").read_text())["event"],"fixture")

    def test_overflow_is_explicit_and_stops_capture(self):
        recorder=runtime.Recorder(Path("unused"),threading.Event());recorder.queue=queue.Queue(maxsize=1)
        recorder.emit("events",{});recorder.emit("events",{})
        self.assertTrue(recorder.stop.is_set());self.assertIn("overflow",recorder.error)

    def test_encoder_discovery_does_not_use_unstable_acm_number(self):
        class Port:serial_number=LEFT_SERIAL;device="/dev/ttyACM9"
        self.assertEqual(runtime.encoder_device([Port()],LEFT_SERIAL),"/dev/ttyACM9")
        with self.assertRaises(RuntimeError):runtime.encoder_device([Port(),Port()],LEFT_SERIAL)

    def test_calibration_file_requires_review_and_matching_devices(self):
        self.assertEqual(runtime.load_scales(None),({"L":None,"R":None},None))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"scales.json";path.write_text('{"schema":1}')
            with self.assertRaises(ValueError):runtime.load_scales(path)

    def test_complete_synthetic_capture_verifies_with_both_encoders(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);stop=threading.Event();recorder=runtime.Recorder(root,stop)
            writer=threading.Thread(target=recorder.run);writer.start()
            app=runtime.PositionApp({},None,recorder,stop,udp=FakeUdp())
            # Simulated 14 s; no sleeps, serial devices, I2C or network.
            for i in range(1400):
                t=i/100
                if i%20==0:healthy(app.model,t)
                app.emit_sensor("imu",sample("imu","sample",((0,)*6,(.1,-.5,.03),(0.,0.,1.)),t))
                if i%10==0:
                    for side in ("L","R"):app.emit_sensor("encoders",enc(side,t,ticks=0))
                app.ingest_gps();app.ingest_imu();app.ingest_encoders();app.update_pose()
                if i%5==0:app.publish_navigation_if_due(t)
            recorder.finishing.set();writer.join(5)
            self.assertFalse(writer.is_alive());self.assertIsNone(recorder.error)
            metadata=dict(status="STOPPED",imu_calibration=app.model.calibration,
                          imu_rows=app.model.imu_count,record_counts=recorder.rows,
                          udp_datagrams_sent=app.udp_sent,commit="synthetic_fixture",odometry_scale_verified=False)
            (root/"metadata.json").write_text(json.dumps(metadata))
            (root/"SHA256SUMS").write_text("\n".join(hashlib.sha256(p.read_bytes()).hexdigest()+"  "+p.name for p in sorted(root.iterdir()) if p.name!="SHA256SUMS"))
            result=verify(root)
            self.assertEqual(result["counts"]["imu"],1400)
            self.assertEqual(result["wheel_samples"],{"L":140,"R":140})
            (root/"encoders.csv").write_text("tampered")
            with self.assertRaisesRegex(ValueError,"Checksum mismatch"):verify(root)


if __name__=="__main__":unittest.main()
