"""Hardware-free tests for the observation logger."""
import argparse
import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).parent / "imu/ism330dhcx_field_logger_20261116.py"
spec = importlib.util.spec_from_file_location("imu_logger", SOURCE)
logger = importlib.util.module_from_spec(spec)
spec.loader.exec_module(logger)


class FakeBus:
    def __init__(self, identity=0x6B):
        self.registers = {0x0F: identity, 0x1E: 3}
        self.writes = []
        self.closed = False

    def read_byte_data(self, address, register):
        return self.registers[register]

    def write_byte_data(self, address, register, value):
        self.writes.append((address, register, value))
        self.registers[register] = value

    def read_i2c_block_data(self, address, register, count):
        return list(struct.pack("<6h", 10, -70, 3, 900, -1700, 16400))

    def close(self):
        self.closed = True


class ImuTests(unittest.TestCase):
    def healthy(self):
        return dict(lat=40, lon=-80, headValid=True, relposned_timestamp="2026-11-16T12:00:00Z",
                    carrier="fixed", fix_quality="RTK Fixed", relpos_valid=True,
                    relpos_gnss_fix_ok=True, relpos_ref_pos_miss=False,
                    relpos_ref_obs_miss=False, fatal_error=False,
                    heading_deg=359.0, relpos_heading_accuracy_deg=0.2)

    def test_signed_decode_and_scales(self):
        raw, gyro, accel = logger.decode_sample(struct.pack("<6h", -32768, 0, 100, 1000, -2000, 0))
        self.assertEqual(raw[0], -32768)
        self.assertAlmostEqual(gyro[2], 0.875)
        self.assertAlmostEqual(accel[1], -0.122)
        with self.assertRaises(ValueError):
            logger.decode_sample([0] * 11)

    def test_configuration_and_wrong_identity_never_writes(self):
        bus = FakeBus()
        logger.Sensor(bus).configure()
        self.assertEqual(bus.writes, [(0x6B, 0x12, 0x44), (0x6B, 0x10, 0x40), (0x6B, 0x11, 0x40)])
        bad = FakeBus(0)
        with self.assertRaises(RuntimeError):
            logger.Sensor(bad).configure()
        self.assertEqual(bad.writes, [])

    def test_data_ready_and_readback_failure(self):
        bus = FakeBus()
        bus.registers[0x1E] = 1
        self.assertIsNone(logger.Sensor(bus).read())
        bus.read_byte_data = lambda address, register: 0x6B if register == 0x0F else 0
        with self.assertRaisesRegex(RuntimeError, "readback"):
            logger.Sensor(bus).configure()

    def test_bias_and_noise(self):
        mean, std = logger.bias_statistics([(1., -2., 0.), (3., -2., 2.)])
        self.assertEqual(mean, [2, -2, 1])
        self.assertAlmostEqual(std[0], 2 ** 0.5)
        self.assertEqual(std[1], 0)
        with self.assertRaises(ValueError):
            logger.bias_statistics([(1, 2, float("nan")), (1, 2, 3)])

    def test_compass_sign_wrap_and_gap_segments(self):
        self.assertEqual(logger.compass_from_yaw(5, 10), 355)
        self.assertEqual(logger.wrap180(359 - 1), -2)
        yaw = logger.YawTracker()
        yaw.update(10, 0, 5)
        gap, compass = yaw.update(10, .01, 200)  # Later GNSS does not erase drift.
        self.assertFalse(gap)
        self.assertAlmostEqual(yaw.yaw, .1)
        self.assertAlmostEqual(compass, 4.9)
        gap, compass = yaw.update(10, .2)
        self.assertTrue(gap)
        self.assertIsNone(compass)
        self.assertEqual(yaw.segment, 1)
        self.assertEqual(yaw.yaw, 0)

    def test_gnss_parsing_and_source_freshness(self):
        healthy = self.healthy()
        parsed = logger.parse_gnss(json.dumps(healthy))
        self.assertTrue(logger.heading_usable(parsed, .05, .2))
        self.assertFalse(logger.heading_usable(parsed, .05, 5))
        self.assertFalse(logger.heading_usable(parsed, 2, .2))
        for key, value in (("headValid", "true"), ("carrier", "float"), ("heading_deg", float("nan")),
                           ("relpos_heading_accuracy_deg", None), ("relpos_ref_obs_miss", True)):
            self.assertFalse(logger.heading_usable({**parsed, key: value}, .05, .2), key)
        for payload in (b"bad", b"[]", b"{}", b"\xff"):
            self.assertIsNone(logger.parse_gnss(payload))
        self.assertEqual(logger.timestamp_age("1970-01-01T00:00:00Z", 1), 1)
        self.assertIsNone(logger.timestamp_age("1970-01-01T00:00:00", 1))
        self.assertIsNone(logger.timestamp_age("1970-01-01T00:00:02Z", 1))

    def test_busy_socket_does_not_share_port(self):
        with patch.object(logger.socket, "socket") as create:
            create.return_value.bind.side_effect = OSError("busy")
            receiver = logger.GnssReceiver()
            with contextlib.redirect_stdout(io.StringIO()):
                receiver.open()
            create.return_value.setsockopt.assert_not_called()
            create.return_value.close.assert_called_once()
            self.assertIsNone(receiver.sock)

    def test_full_recording_with_fake_hardware_and_missing_gps(self):
        bus = FakeBus()
        clock = [0.0]
        def tick():
            clock[0] += .01
            return clock[0]
        with tempfile.TemporaryDirectory() as temporary:
            args = argparse.Namespace(stationary=True, output_dir=Path(temporary),
                                      calibration_seconds=10., duration=.1)
            with patch.dict(sys.modules, smbus2=types.SimpleNamespace(SMBus=lambda number: bus)), \
                 patch.object(logger, "repository_state", return_value={"commit": "test", "working_tree": ""}), \
                 patch.object(logger.time, "monotonic", side_effect=tick), \
                 patch.object(logger.GnssReceiver, "open"), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(logger.run(args), 0)
            directory = next(Path(temporary).iterdir())
            metadata = json.loads((directory / "metadata.json").read_text())
            self.assertEqual(metadata["status"], "COMPLETED")
            self.assertTrue(bus.closed)
            with (directory / "samples.csv").open() as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(tuple(rows[0]), logger.FIELDS)
            self.assertTrue(any(r["phase"] == "calibration" for r in rows))
            observed = [r for r in rows if r["phase"] == "observation"]
            self.assertGreater(len(observed), 0)
            self.assertAlmostEqual(float(observed[-1]["gyro_z_corrected_dps"]), 0)
            self.assertEqual(observed[-1]["gnss_heading_usable"], "0")
            for line in (directory / "SHA256SUMS").read_text().splitlines():
                expected, name = line.split("  ")
                self.assertEqual(logger.hashlib.sha256((directory / name).read_bytes()).hexdigest(), expected)

    def test_failed_identity_closes_bus_and_preserves_failure_metadata(self):
        bus = FakeBus(0)
        with tempfile.TemporaryDirectory() as temporary:
            args = argparse.Namespace(stationary=True, output_dir=Path(temporary),
                                      calibration_seconds=10., duration=.1)
            with patch.dict(sys.modules, smbus2=types.SimpleNamespace(SMBus=lambda number: bus)), \
                 patch.object(logger, "repository_state", return_value={"commit": "test", "working_tree": ""}), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(logger.run(args), 1)
            metadata = json.loads(next(Path(temporary).glob("*/metadata.json")).read_text())
            self.assertEqual(metadata["status"], "FAILED")
            self.assertIn("WHO_AM_I", metadata["error"])
            self.assertTrue(bus.closed)
            self.assertEqual(bus.writes, [])

    def test_unconfirmed_stationary_or_dirty_tree_does_not_open_hardware(self):
        args = argparse.Namespace(stationary=False)
        with self.assertRaisesRegex(ValueError, "stationary"):
            logger.run(args)
        args.stationary = True
        with patch.object(logger, "repository_state", return_value={"working_tree": "modified"}):
            with self.assertRaisesRegex(ValueError, "uncommitted"):
                logger.run(args)


if __name__ == "__main__":
    unittest.main()
