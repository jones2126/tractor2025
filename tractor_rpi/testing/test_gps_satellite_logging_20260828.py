#!/usr/bin/env python3
"""Focused offline tests for per-receiver F9P satellite diagnostics."""

import importlib.util
import json
import struct
import sys
import time
import types
import unittest
from pathlib import Path


TRACTOR_RPI = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# The parser tests do not open serial ports. This stub lets them run on a
# development machine that does not have pyserial installed.
sys.modules.setdefault(
    "serial",
    types.SimpleNamespace(SerialException=Exception, Serial=None),
)

rtcm_server = load_module(
    "rtcm_server_20260828_test",
    TRACTOR_RPI / "rtcm_server_20260828.py",
)
heading_analysis = load_module(
    "analyze_heading_f9p_20260828_test",
    TRACTOR_RPI.parent / "field_testing" / "tools" / "analyze_heading_f9p_20260828.py",
)
teensy_bridge = load_module(
    "teensy_serial_bridge_20260728_test",
    TRACTOR_RPI / "teensy_serial_bridge_20260728.py",
)
mission_preflight = load_module(
    "mission_preflight_20260804_test",
    TRACTOR_RPI / "testing" / "mission_preflight_20260804.py",
)
mission_dashboard = load_module(
    "mission_dashboard_20260910_test",
    TRACTOR_RPI / "pure-pursuit" / "mission_dashboard_20260910.py",
)


class DashboardJsonTests(unittest.TestCase):
    def test_non_finite_telemetry_is_encoded_as_null(self):
        telemetry = {
            "gps": {"speed_mps": float("nan"), "diff_age": float("inf")},
            "controller": {"cross_track_err_m": 0.25},
        }
        encoded = json.dumps(
            mission_dashboard.json_safe(telemetry),
            allow_nan=False,
        )
        decoded = json.loads(encoded)
        self.assertIsNone(decoded["gps"]["speed_mps"])
        self.assertIsNone(decoded["gps"]["diff_age"])
        self.assertEqual(decoded["controller"]["cross_track_err_m"], 0.25)


class SatelliteParserTests(unittest.TestCase):
    def test_nav_pvt_satellites_used(self):
        payload = bytearray(92)
        payload[23] = 8
        self.assertEqual(
            rtcm_server.parse_nav_pvt_satellites(payload),
            {"numSV_used": 8},
        )
        self.assertIsNone(rtcm_server.parse_nav_pvt_satellites(b"\x00" * 23))

    def test_nav_sat_summary(self):
        payload = bytearray(8 + 3 * 12)
        payload[4] = 1  # NAV-SAT message version
        payload[5] = 3

        cno_and_used = [(30, True), (40, False), (0, True)]
        for index, (cno, used) in enumerate(cno_and_used):
            offset = 8 + index * 12
            payload[offset + 2] = cno
            struct.pack_into("<I", payload, offset + 8, (1 << 3) if used else 0)

        self.assertEqual(
            rtcm_server.parse_nav_sat(payload),
            {
                "numSV_visible": 3,
                "numSV_used": 2,
                "cno_mean_dbhz": 35.0,
                "cno_min_dbhz": 30,
                "cno_max_dbhz": 40,
            },
        )
        self.assertIsNone(rtcm_server.parse_nav_sat(payload[:-1]))

    def test_poll_frame_checksum(self):
        frame = rtcm_server.ubx_frame(rtcm_server.CLS_NAV, rtcm_server.ID_PVT)
        self.assertEqual(frame[:6], b"\xb5\x62\x01\x07\x00\x00")
        self.assertEqual(
            tuple(frame[-2:]),
            rtcm_server.ubx_checksum(frame[2:-2]),
        )


    def test_receiver_prefixes_update_independently(self):
        base_payload = bytearray(92)
        heading_payload = bytearray(92)
        base_payload[23] = 31
        heading_payload[23] = 8
        rtcm_server.update_satellite_state("base", rtcm_server.ID_PVT, base_payload)
        rtcm_server.update_satellite_state("heading", rtcm_server.ID_PVT, heading_payload)
        self.assertEqual(rtcm_server.state["base_numSV_used"], 31)
        self.assertEqual(rtcm_server.state["heading_numSV_used"], 8)


class LoggerFieldTests(unittest.TestCase):
    def test_clean_receiver_specific_fields(self):
        logger = load_module(
            "field_test_logger_20260828_test",
            TRACTOR_RPI / "field_test_logger_20260828.py",
        )
        logger.latest_gps.clear()
        logger.latest_gps.update(
            {
                "base_numSV_used": 31,
                "base_numSV_visible": 36,
                "base_cno_mean_dbhz": 38.5,
                "base_numSV_used_gga": 31,
                "heading_numSV_used": 8,
                "heading_numSV_visible": 11,
                "heading_cno_mean_dbhz": 34.5,
            }
        )
        logger.latest_status.clear()
        logger.latest_status.update(
            {
                "system": {"firmware": "teensy_main_20260908_1p8_test"},
                "transmission": {
                    "motor_current_mA": 1425,
                    "peak_motor_current_mA": 2085,
                    "motor_current_valid": 1,
                },
            }
        )
        row = logger.build_row(time.time())

        self.assertNotIn("numSV", row)
        self.assertEqual(row["teensy_firmware"], "teensy_main_20260908_1p8_test")
        self.assertEqual(row["jrk_motor_current_mA"], 1425)
        self.assertEqual(row["jrk_peak_motor_current_mA"], 2085)
        self.assertEqual(row["jrk_motor_current_valid"], 1)
        self.assertEqual(row["base_numSV_used"], 31)
        self.assertEqual(row["base_numSV_visible"], 36)
        self.assertEqual(row["heading_numSV_used"], 8)
        self.assertEqual(row["heading_numSV_visible"], 11)
        self.assertEqual(set(row), set(logger.CSV_COLUMNS))


class JrkCurrentTelemetryTests(unittest.TestCase):
    def test_bridge_maps_current_fields(self):
        bridge = teensy_bridge.TeensySerialBridge.__new__(
            teensy_bridge.TeensySerialBridge
        )
        now = time.time()
        bridge.latest_data = {
            "TRANS": {
                "jma": 1425,
                "jmp": 2085,
                "jmv": 1,
                "last_update": now,
            }
        }
        bridge.last_cmd_vel = {
            "linear_x": 0.0,
            "angular_z": 0.0,
            "timestamp": now,
        }
        bridge.cmd_vel_received_count = 0
        bridge.cmd_vel_sent_count = 0
        bridge.cmd_vel_echo_count = 0
        bridge.current_gps_status = "UNKNOWN"

        transmission = bridge.create_broadcast_message()["transmission"]
        self.assertEqual(transmission["motor_current_mA"], 1425)
        self.assertEqual(transmission["peak_motor_current_mA"], 2085)
        self.assertEqual(transmission["motor_current_valid"], 1)

    def test_preflight_requires_valid_current_for_1p8_firmware(self):
        samples = []
        for sequence in range(1, 102):
            samples.append(
                (
                    sequence * 0.05,
                    {
                        "steering": {
                            "sequence": sequence,
                            "mode": 2,
                            "state": "PAUSE",
                            "pwm": 0,
                        },
                        "transmission": {
                            "mode": 2,
                            "target": 2836,
                            "current": 2836,
                            "actual_target": 2836,
                            "scaled_feedback": 2836,
                            "duty_cycle_target": 0,
                            "duty_cycle": 0,
                            "errors_halting": 0,
                            "jrk_sequence": sequence // 4 + 1,
                            "jrk_valid": 1,
                            "jrk_read_latency_ms": 1,
                            "jrk_timeouts": 0,
                            "cmd_vel_mps": 0,
                            "motor_current_mA": 25,
                            "peak_motor_current_mA": 40,
                            "motor_current_valid": 1,
                        },
                        "system": {
                            "firmware": "teensy_main_20260908_1p8_test"
                        },
                    },
                )
            )

        checks = mission_preflight.steering_checks(
            samples,
            seconds=5.0,
            neutral_jrk_target=2836,
            expected_firmware="teensy_main_20260908_1p8_test",
        )
        current_check = next(
            check for check in checks if check.name == "JRK motor-current telemetry"
        )
        self.assertTrue(current_check.passed, current_check.detail)


class HeadingAnalysisTests(unittest.TestCase):
    def test_stationary_pauses_select_lap(self):
        rows = []
        for second in range(101):
            speed = 0.0 if second <= 20 or second >= 80 else 1.0
            rows.append({"elapsed_sec": str(second), "speed_mps": str(speed)})
        pauses = heading_analysis.find_stationary_pauses(rows, 0.08, 15.0)
        self.assertEqual(len(pauses), 2)
        self.assertEqual(
            heading_analysis.choose_lap(rows, pauses, None, None),
            (20.0, 80.0, "stationary pauses"),
        )

    def test_position_bearing(self):
        origin = (40.0, -80.0)
        north = (40.0001, -80.0)
        east = (40.0, -79.9999)
        self.assertAlmostEqual(heading_analysis.bearing_deg(origin, north), 0.0, places=4)
        self.assertAlmostEqual(heading_analysis.bearing_deg(origin, east), 90.0, places=4)


if __name__ == "__main__":
    unittest.main()
