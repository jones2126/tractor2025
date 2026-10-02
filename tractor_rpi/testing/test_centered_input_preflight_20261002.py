#!/usr/bin/env python3
import unittest
import time
from collections import defaultdict

from tractor_rpi import teensy_serial_bridge_20261002 as bridge_module
from tractor_rpi.testing import mission_preflight_20261002_centered_inputs as preflight


def sample(value=500, tractor=525, mode=2, state="PAUSE", age=0.1, signal=1):
    return {
        "steering": {"mode": mode, "state": state, "current": tractor},
        "transmission": {"mode": mode},
        "radio": {
            "handheld_steering_raw": value,
            "handheld_additional_raw": value,
            "handheld_transmission_raw": value,
            "handheld_pot4_raw": value,
            "handheld_inputs_signal_good": signal,
            "handheld_inputs_age": age,
        },
    }


class CenteredInputTests(unittest.TestCase):
    def samples(self, **kwargs):
        return [(index / 20.0, sample(**kwargs)) for index in range(100)]

    def test_all_centered_values_pass(self):
        checks = preflight.centered_input_checks(self.samples())
        self.assertEqual(len(checks), 5)
        self.assertTrue(all(check.passed for check in checks))

    def test_handheld_value_outside_tolerance_fails(self):
        checks = preflight.centered_input_checks(self.samples(value=701))
        self.assertFalse(all(check.passed for check in checks[1:]))

    def test_tractor_sensor_outside_tolerance_fails(self):
        checks = preflight.centered_input_checks(self.samples(tractor=600))
        self.assertFalse(checks[0].passed)

    def test_stale_handheld_telemetry_fails(self):
        checks = preflight.centered_input_checks(self.samples(age=2.0))
        self.assertTrue(checks[0].passed)
        self.assertFalse(all(check.passed for check in checks[1:]))

    def test_no_radio_signal_fails(self):
        checks = preflight.centered_input_checks(self.samples(signal=0))
        self.assertFalse(all(check.passed for check in checks[1:]))

    def test_not_paused_fails_before_reporting_values(self):
        checks = preflight.centered_input_checks(self.samples(mode=9, state="NO_SIG"))
        self.assertEqual(len(checks), 1)
        self.assertFalse(checks[0].passed)
        self.assertIn("Pause", checks[0].detail)


class CenteredInputBridgeTests(unittest.TestCase):
    def test_bridge_forwards_all_four_handheld_values(self):
        bridge = bridge_module.TeensySerialBridge.__new__(
            bridge_module.TeensySerialBridge
        )
        now = time.time()
        bridge.latest_data = defaultdict(
            dict,
            {
                "RADIO_INPUTS": {
                    "s": 501,
                    "t": 502,
                    "x": 503,
                    "p4": 504,
                    "sg": 1,
                    "a": 40,
                    "last_update": now,
                }
            },
        )
        bridge.last_cmd_vel = {
            "linear_x": 0.0,
            "angular_z": 0.0,
            "timestamp": 0.0,
        }
        bridge.cmd_vel_received_count = 0
        bridge.cmd_vel_sent_count = 0
        bridge.cmd_vel_echo_count = 0
        bridge.current_gps_status = "UNKNOWN"

        radio = bridge.create_broadcast_message()["radio"]
        self.assertEqual(radio["handheld_steering_raw"], 501)
        self.assertEqual(radio["handheld_additional_raw"], 502)
        self.assertEqual(radio["handheld_transmission_raw"], 503)
        self.assertEqual(radio["handheld_pot4_raw"], 504)
        self.assertEqual(radio["handheld_inputs_signal_good"], 1)
        self.assertLess(radio["handheld_inputs_age"], 1.0)


if __name__ == "__main__":
    unittest.main()
