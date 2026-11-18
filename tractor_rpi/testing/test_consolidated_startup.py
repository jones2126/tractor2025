"""Offline parity, evidence-integrity and startup tests. No Tractor01 access."""
import ast
import copy
import datetime as dt
import hashlib
import io
import importlib.util
import json
from pathlib import Path
import struct
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from tractor_rpi import tractor_preflight as new
from tractor_rpi import telemetry_recorder as recorder
from tractor_rpi import telemetry_ipc as ipc
from tractor_rpi import position_model as model
from tractor_rpi import calc_publish_position as producer
from tractor_rpi import imu_sensor
from tractor_rpi import export_telemetry_run as exporter
from tractor_rpi.testing import mission_preflight_20261002 as legacy
from tractor_rpi.testing.test_calc_publish_position_20261117 import healthy, sample, enc, MemoryRecorder, FakeUdp

ROOT = Path(__file__).resolve().parents[2]
GOLDEN_SPEC = importlib.util.spec_from_file_location("consolidation_golden_base", ROOT / "tractor_rpi/testing/mission_preflight_20260804.py")
GOLDEN = importlib.util.module_from_spec(GOLDEN_SPEC)
sys.modules[GOLDEN_SPEC.name] = GOLDEN
GOLDEN_SPEC.loader.exec_module(GOLDEN)


def gps_samples(**changes):
    result = []
    for i in range(101):
        g = dict(lat=40., lon=-80., fix_quality="RTK Fixed", fatal_error=False, diff_age=1.,
                 headValid=True, carrier="fixed", heading_deg=350., relpos_length_m=1.06,
                 relpos_heading_accuracy_deg=.5, heading_nmea_count=0, relposned_count=i // 4,
                 relpos_gnss_fix_ok=True, relpos_diff_solution=True, relpos_valid=True,
                 relpos_moving=True, relpos_ref_pos_miss=False, relpos_ref_obs_miss=False,
                 speed_mps=0., speed_update_count=i // 4, speed_source="RMC", heading_numSV_used=29)
        g.update(changes)
        result.append((i * .05, g))
    return result


def comparison(checks):
    return [(c.name, c.passed, getattr(c, "warning", False)) for c in checks]


class ParityTests(unittest.TestCase):
    def test_steering_and_firmware_predicates_match_legacy(self):
        data = []
        fields = ("current", "scaled_feedback", "duty_cycle_target", "duty_cycle", "errors_halting",
                  "jrk_read_latency_ms", "jrk_timeouts", "cmd_vel_mps")
        for i in range(101):
            trans = dict.fromkeys(fields, 0)
            trans.update(mode=2, target=2836, actual_target=2836, jrk_sequence=i // 4, jrk_valid=1)
            data.append((i * .05, dict(steering=dict(sequence=i + 1, mode=2, state="PAUSE", pwm=0),
                                      transmission=trans, system=dict(firmware="v5"))))
        for key, value in ((None, None), ("errors_halting", 1), ("jrk_valid", 0), ("target", 2800)):
            rows = copy.deepcopy(data)
            if key:
                for _, row in rows:
                    row["transmission"][key] = value
            self.assertEqual(comparison(new.steering_checks(rows, 5, 2836, "v5")),
                             comparison(GOLDEN.steering_checks(rows, 5, 2836, "v5")))
    def test_gps_all_existing_predicates_match_legacy(self):
        variations = ({}, {"fix_quality": "RTK Float"}, {"headValid": False}, {"carrier": "float"},
                      {"fatal_error": True}, {"diff_age": 8}, {"relpos_length_m": 1.9},
                      {"relpos_heading_accuracy_deg": 2}, {"speed_mps": .4}, {"speed_mps": None},
                      {"heading_numSV_used": 25}, {"heading_numSV_used": 24}, {"heading_numSV_used": 23},
                      {"heading_numSV_used": None}, {"relpos_valid": False}, {"relpos_moving": False},
                      {"relpos_ref_pos_miss": True}, {"relpos_ref_obs_miss": True},
                      {"relpos_diff_solution": False}, {"relpos_gnss_fix_ok": False},
                      {"heading_nmea_count": None}, {"relposned_count": 0}, {"speed_update_count": 0})
        for variation in variations:
            with self.subTest(variation=variation):
                data = gps_samples(**variation)
                self.assertEqual(comparison(legacy.gps_checks(data, 5, 5, 1, .8, 1.3, .2)),
                                 comparison(new.gps_checks(data, 5, 5, 1, .8, 1.3, .2)))

    def test_no_gps_failure_parity(self):
        self.assertEqual(comparison(new.gps_checks([], 5, 5, 1, .8, 1.3, .2)),
                         comparison(legacy.gps_checks([], 5, 5, 1, .8, 1.3, .2)))

    def test_network_and_startup_implementation_parity(self):
        response = (0, "configure_dual_f9p_5hz_profile_20260923.py --heading-startup code=exited status=0")
        with patch.object(new, "command_output", return_value=response):
            self.assertTrue(new.heading_startup_configuration_check().passed)
        with patch.object(new, "command_output", side_effect=[(0, response[1].replace("code=exited", "code=(null)")),
                (0, "HEADING STARTUP RECOVERY\nHeading startup: PASS independent readback\nPASS: heading startup profile is verified in RAM")]):
            self.assertTrue(new.heading_startup_configuration_check().passed)

    def test_new_production_path_has_no_dated_preflight_imports(self):
        tree = ast.parse((ROOT / "tractor_rpi/tractor_preflight.py").read_text())
        imports = [ast.unparse(n) for n in ast.walk(tree) if isinstance(n, (ast.Import, ast.ImportFrom))]
        self.assertFalse(any("2026" in line for line in imports))
        self.assertNotIn("teensy-bridge.service", new.SERVICES)

    def test_preflight_uses_recorder_not_udp_bind(self):
        with patch.object(new.time, "sleep"), patch.object(new.time, "monotonic", side_effect=[0, 0, 0, 5]), patch.object(new, "request", return_value={"records": [
                dict(received_mono=5., payload={"fix_quality": "RTK Fixed"})]}) as request:
            self.assertEqual(new.collect_json_udp(6009, 5)[0][1]["fix_quality"], "RTK Fixed")
            self.assertEqual(request.call_args.args[0]["channel"], "gnss")

    def test_preflight_missing_recorder_fails(self):
        with patch.object(new, "request", side_effect=OSError("missing")):
            self.assertFalse(new.sensor_recording_checks()[0].passed)

    def test_zero_motion_encoder_readiness_and_stale_source(self):
        now = time.monotonic()
        quality = dict(imu_fresh=True, imu_calibrated=True,
                       encoders={s: dict(valid=True) for s in ("L", "R")})
        pose = dict(payload=dict(monotonic_s=now - .02,
                                 pose=dict(quality=quality, heading_anchor_age_s=1)))
        encoders = [dict(payload=dict(side=s, device_ms=i * 100, monotonic_s=now - .1, delta_ticks=0))
                    for s in ("L", "R") for i in range(5)]
        health = dict(error=None, losses=0, last_write=now - .01, free_bytes=1000, min_free_bytes=100)
        def request(message):
            if message["op"] == "health":
                return health
            return dict(records={"pose": [pose], "imu": [{}] * 100, "encoders": encoders}[message["channel"]])
        with patch.object(new, "request", side_effect=request):
            self.assertTrue(all(c.passed for c in new.sensor_recording_checks()))
            pose["payload"]["monotonic_s"] = now - 5
            self.assertFalse(all(c.passed for c in new.sensor_recording_checks()))

    def test_notify_failure_does_not_throw_or_grant_success(self):
        with patch("urllib.request.urlopen", side_effect=OSError()), patch.object(new.time, "sleep"), patch("sys.stdout", new=io.StringIO()):
            self.assertFalse(new.notify("test", "test"))

    def test_full_check_stdout_warning_and_exit_semantics(self):
        checks = new.heading_satellites_used_check(gps_samples(heading_numSV_used=25))
        with patch("sys.stdout", new_callable=io.StringIO) as output:
            self.assertTrue(new.print_checks([checks]))
            self.assertIn("[WARN]", output.getvalue())


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.evidence = recorder.Evidence(self.temp.name, zone=dt.timezone.utc, min_free=0)

    def tearDown(self):
        if self.evidence.handles:
            self.evidence.close()
        self.evidence.hasher.shutdown(wait=True)
        self.temp.cleanup()

    def message(self, seq=0):
        return dict(op="record", schema=1, session="p1", sequence=seq, channel="imu",
                    source_mono=time.monotonic(), payload={"gyro_z_raw": 0})

    def test_daily_directory_unique_and_outside_repo(self):
        directory = self.evidence.directory
        self.assertEqual(directory.parent.name, dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d"))
        self.evidence.rotate()
        self.assertNotEqual(directory, self.evidence.directory)

    def test_duplicate_ack_does_not_duplicate_file_record(self):
        self.evidence.ingest(self.message())
        self.evidence.ingest(self.message())
        self.assertEqual(self.evidence.counts["imu"], 1)

    def test_producer_gap_recorded(self):
        self.evidence.ingest(self.message())
        self.evidence.ingest(self.message(2))
        self.assertEqual(self.evidence.losses, 1)
        self.assertEqual(self.evidence.rings["events"][-1]["payload"]["event"], "producer_gap")

    def test_recent_windows_and_limits(self):
        since = time.monotonic()
        self.evidence.ingest(self.message())
        result = self.evidence.window(since, "imu")
        self.assertEqual(len(result["records"]), 1)
        with self.assertRaises(ValueError):
            self.evidence.window(since - 20, "imu")

    def test_hashes_cover_closed_files_and_compatible_csv(self):
        self.evidence.append("gnss", dict(lat=40., lon=-80., speed_mps=0))
        self.evidence.append("status", dict(steering=dict(sequence=1), system=dict(firmware="v5")))
        directory = self.evidence.directory
        self.evidence.close()
        self.evidence.hasher.shutdown(wait=True)
        for line in (directory / "SHA256SUMS").read_text().splitlines():
            digest, name = line.split("  ", 1)
            self.assertEqual(hashlib.sha256((directory / name).read_bytes()).hexdigest(), digest)
        self.assertIn("40.0", (directory / "field.csv").read_text())

    def test_export_verified_time_window_and_reject_tampering(self):
        start = dt.datetime.now(dt.timezone.utc) - dt.timedelta(seconds=1)
        self.evidence.append("status", dict(steering=dict(sequence=1)))
        directory = self.evidence.directory
        self.evidence.close()
        self.evidence.hasher.shutdown(wait=True)
        target = Path(self.temp.name) / "export"
        result = exporter.export([directory], target, start, dt.datetime.now(dt.timezone.utc))
        self.assertEqual(result["field_rows"], 1)
        (directory / "status.jsonl").write_text("tampered")
        with self.assertRaisesRegex(ValueError, "Checksum"):
            exporter.verify(directory)

    def test_nonfinite_observations_are_rejected(self):
        with self.assertRaises(ValueError):
            self.evidence.append("imu", dict(value=float("nan")))

    def test_low_space_does_not_delete_evidence(self):
        with patch.object(recorder.shutil, "disk_usage", return_value=type("Usage", (), {"free": 0})()):
            self.evidence.min_free = 1
            self.evidence.last_flush = -100
            with self.assertRaises(OSError):
                self.evidence.flush()
        self.assertTrue(self.evidence.directory.exists())

    def test_oversized_ipc_frame_rejected(self):
        with self.assertRaises(ValueError):
            ipc.encode({"data": "x" * ipc.MAX_FRAME})

    def test_live_socket_is_not_unlinked_by_failed_second_owner(self):
        path = Path(self.temp.name) / "recorder.sock"
        path.write_text("sentinel")
        server = recorder.Server(self.evidence, path)
        with self.assertRaises(RuntimeError):
            server.listen()
        server.close()
        self.assertEqual(path.read_text(), "sentinel")

    def test_real_stream_push_ack_and_query_do_not_consume_evidence(self):
        server = recorder.Server(self.evidence, Path(self.temp.name) / "unused")
        left, right = socket.socketpair()
        right.settimeout(1)
        server.buffers[left] = bytearray()
        server.selector.register(left, recorder.selectors.EVENT_READ, "client")
        try:
            right.sendall(ipc.encode(self.message()))
            server.step()
            self.assertEqual(ipc.receive(right, bytearray())["ack"], 0)
            for _ in range(2):
                right.sendall(ipc.encode(dict(op="window", channel="imu", since=time.monotonic() - 1)))
                server.step()
                self.assertEqual(len(ipc.receive(right, bytearray())["records"]), 1)
            self.assertEqual(self.evidence.counts["imu"], 1)
        finally:
            server.close()
            right.close()

    def test_router_without_newline_retained_on_disconnect(self):
        server = recorder.Server(self.evidence, Path(self.temp.name) / "unused")
        left, right = socket.socketpair()
        server.buffers[left] = bytearray()
        server.selector.register(left, recorder.selectors.EVENT_READ, "router_client")
        right.sendall(b'{"wifi_rssi_dbm":-52}')
        server.step()
        right.close()
        server.step()
        self.assertEqual(self.evidence.rings["router"][-1]["payload"]["wifi_rssi_dbm"], -52)
        server.close()

    def test_deduplicated_status_broadcast_still_allows_zero_motion(self):
        server = recorder.Server(self.evidence, Path(self.temp.name) / "unused")
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.bind(("127.0.0.1", 0))
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server.selector.register(udp, recorder.selectors.EVENT_READ, "status")
        try:
            for seq in (1, 1, 2):
                sender.sendto(ipc.encode(dict(steering=dict(sequence=seq))), udp.getsockname())
                server.step()
            self.assertEqual(self.evidence.counts["status"], 2)
        finally:
            sender.close()
            server.close()


class SensorTests(unittest.TestCase):
    def test_stationary_tick_jitter_is_bounded_and_interval_scaled(self):
        for ticks in (-4, 0, 4, -8, 8):
            self.assertTrue(producer.encoder_stationary(dict(valid=True, last_ticks=ticks, last_interval_ms=100)))
        for wheel in (dict(valid=True, last_ticks=9, last_interval_ms=100),
                      dict(valid=True, last_ticks=8, last_interval_ms=50),
                      dict(valid=False, last_ticks=0, last_interval_ms=100),
                      dict(valid=True, last_ticks=0, last_interval_ms=0)):
            self.assertFalse(producer.encoder_stationary(wheel))

    def test_silent_encoder_marks_stale_without_stopping_gps(self):
        class Stop:
            calls = 0
            def is_set(self):
                self.calls += 1
                return self.calls > 3
        class Device:
            in_waiting = 0
            def read(self, _):
                return b""
        memory = MemoryRecorder()
        app = producer.PositionApp({"L": Device()}, None, memory, Stop())
        with patch.object(producer.time, "monotonic", side_effect=range(0, 100, 6)):
            app.read_encoder("L")
        self.assertIsNone(app.fault)
        items = list(app.queues["encoders"].queue)
        self.assertEqual([item["kind"] for item in items], ["stale"])
        app.model.apply_encoder(items[0])
        self.assertFalse(app.model.wheels["L"]["valid"])

    def test_boot_calibrates_only_with_stationary_evidence(self):
        memory = MemoryRecorder()
        app = producer.BootPositionApp({}, None, memory, threading.Event(), boot_calibration=True)
        for i in range(1700):
            t = i / 100
            if i % 20 == 0:
                healthy(app.model, t)
                app.model.gps["speed_mps"] = 0.
            if i % 10 == 0:
                for side in ("L", "R"):
                    # Replay the observed parked encoder jitter rather than
                    # unrealistically requiring exact zero from both magnets.
                    app.model.apply_encoder(enc(side, t, ticks=4 if i % 20 else -4))
            status = dict(received_mono=t, payload=dict(steering=dict(state="PAUSE", pwm=0),
                          transmission=dict(target=2836, actual_target=2836)))
            app.pending.extend([sample("readiness", "status", [status], t),
                                sample("imu", "sample", ((0,) * 6, (0.,) * 3, (0., 0., 1.)), t)])
            app.update_pose()
        self.assertIsNotNone(app.model.bias)
        self.assertIsNotNone(app.model.heading_anchor)
        self.assertEqual(sum(c == "imu" for c, _ in memory.records), 1700)

    def test_threaded_publisher_sequence_is_unique(self):
        p = ipc.Publisher(threading.Event())
        workers = [threading.Thread(target=lambda: [p.emit("imu", {"v": 0}) for _ in range(100)]) for _ in range(3)]
        for w in workers:
            w.start()
        for w in workers:
            w.join()
        numbers = [p.queue.get_nowait()[0] for _ in range(p.queue.qsize())]
        self.assertEqual(numbers, list(range(300)))
    def test_uncalibrated_raw_samples_are_retained(self):
        memory = MemoryRecorder()
        m = model.PoseModel(memory.emit)
        for i in range(1200):
            m.apply_imu(sample("imu", "sample", ((0,) * 6, (0.,) * 3, (0., 0., 1.)), i / 100))
        self.assertIsNone(m.bias)
        self.assertEqual(len(memory.records), 1200)
        self.assertEqual(memory.records[-1][1]["phase"], "waiting_stationary")

    def test_calibration_and_alignment_are_separate(self):
        memory = MemoryRecorder()
        m = model.PoseModel(memory.emit)
        m.calibration_allowed = True
        for i in range(1200):
            m.apply_imu(sample("imu", "sample", ((0,) * 6, (0.,) * 3, (0., 0., 1.)), i / 100))
        self.assertIsNotNone(m.bias)
        self.assertIsNone(m.heading_anchor)
        healthy(m, 12.)
        self.assertIsNotNone(m.heading_anchor)

    def test_imu_decode_matches_commissioned_scaling(self):
        raw = (100, -100, 200, 0, 0, 16384)
        self.assertEqual(imu_sensor.decode_sample(struct.pack("<6h", *raw))[0], raw)
        self.assertAlmostEqual(imu_sensor.decode_sample(struct.pack("<6h", *raw))[1][0], .875)

    def test_legacy_gnss_is_not_mixed_with_raw_sensors(self):
        memory, udp = MemoryRecorder(), FakeUdp()
        app = producer.BootPositionApp({}, None, memory, threading.Event(), udp=udp)
        healthy(app.model, 1.)
        app.publish_navigation_if_due(1.)
        self.assertEqual({address[1] for _, address in udp.sent}, {6002, 6009, 6010, 6013})
        self.assertEqual(len({data for data, _ in udp.sent}), 1)
        self.assertNotIn("encoders", json.loads(udp.sent[0][0]))
        self.assertFalse(any(channel == "gnss" for channel, _ in memory.records))

    def test_restart_is_not_automatically_armed(self):
        app = producer.BootPositionApp({}, None, MemoryRecorder(), threading.Event())
        self.assertFalse(app.armed)
        self.assertFalse(app.model.calibration_allowed)

    def test_zero_encoder_delta_is_valid_alive_data(self):
        m = model.PoseModel(lambda *_: None)
        healthy(m, 1.)
        for side in ("L", "R"):
            m.apply_encoder(enc(side, 1., ticks=0))
            m.apply_encoder(enc(side, 1.1, ticks=0))
        self.assertTrue(all(w["valid"] for w in m.quality(1.1)["encoders"].values()))


if __name__ == "__main__":
    unittest.main()
