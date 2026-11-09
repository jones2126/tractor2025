#!/usr/bin/env python3

import os
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from tractor_rpi.rpi_command_lease_protocol_20261031 import RpiCommandMode
from tractor_rpi.teensy_serial_bridge_rpi_arbitrator_20261104 import (
    EXPECTED_FIRMWARE,
    MOTION_GATE,
    ProductionFeedReceiver,
    require_production_environment,
    stopped_phone_pause_reset_ready,
)


ROUTE = "b47660b4c74d5e26ca5d9448f069fd7193c068cf62e3c0b62d5c083d9aed4d64"


class ProductionBridgeTests(unittest.TestCase):
    def test_environment_fails_closed(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaises(ValueError):
                require_production_environment()
        with patch.dict(
            os.environ,
            {"TRACTOR_EXPECTED_ROUTE_SHA256": ROUTE},
            clear=True,
        ):
            with self.assertRaises(RuntimeError):
                require_production_environment()

    def test_exact_gate_and_degraded_setting_are_required(self):
        with patch.dict(
            os.environ,
            {
                "TRACTOR_EXPECTED_ROUTE_SHA256": ROUTE,
                "TRACTOR_MOTION_AUTHORIZATION": MOTION_GATE,
                "TRACTOR_COMMUNICATIONS_DEGRADED": "30S_30M",
            },
            clear=True,
        ):
            route, degraded = require_production_environment()
        self.assertEqual(ROUTE, route)
        self.assertTrue(degraded)

    def test_receiver_uses_phone_mode_and_sticky_guard(self):
        receiver = ProductionFeedReceiver(ROUTE, degraded=True)
        self.assertTrue(receiver.collector.phone_replaces_nrf_mode_switch)
        self.assertTrue(receiver.collector.degraded_preauthorized)
        self.assertEqual(
            "teensy_main_20261104_rpi_command_lease_v3", EXPECTED_FIRMWARE
        )

    def test_stopped_phone_pause_can_clear_startup_latch(self):
        inputs = SimpleNamespace(
            requested_mode=RpiCommandMode.PAUSE,
            phone_link_fresh=True,
        )
        status = SimpleNamespace(
            mode=RpiCommandMode.PAUSE,
            arrival_fresh=True,
            lease_usable=True,
            expiry_latched=False,
            estop_latched=False,
        )
        sample = SimpleNamespace(
            steering_control_healthy=True,
            transmission_control_healthy=True,
        )
        self.assertTrue(
            stopped_phone_pause_reset_ready(
                inputs,
                status,
                sample,
                {"STEER": {"p": 0}, "TRANS": {"dc": 0}},
            )
        )

    def test_pause_reset_fails_closed_for_motion_or_bad_health(self):
        status = SimpleNamespace(
            mode=RpiCommandMode.PAUSE,
            arrival_fresh=True,
            lease_usable=True,
            expiry_latched=False,
            estop_latched=False,
        )
        healthy = SimpleNamespace(
            steering_control_healthy=True,
            transmission_control_healthy=True,
        )
        zero = {"STEER": {"p": 0}, "TRANS": {"dc": 0}}
        for inputs, sample, data in (
            (
                SimpleNamespace(
                    requested_mode=RpiCommandMode.MANUAL_PERCENT,
                    phone_link_fresh=True,
                ),
                healthy,
                zero,
            ),
            (
                SimpleNamespace(
                    requested_mode=RpiCommandMode.PAUSE,
                    phone_link_fresh=False,
                ),
                healthy,
                zero,
            ),
            (
                SimpleNamespace(
                    requested_mode=RpiCommandMode.PAUSE,
                    phone_link_fresh=True,
                ),
                SimpleNamespace(
                    steering_control_healthy=False,
                    transmission_control_healthy=True,
                ),
                zero,
            ),
            (
                SimpleNamespace(
                    requested_mode=RpiCommandMode.PAUSE,
                    phone_link_fresh=True,
                ),
                healthy,
                {"STEER": {"p": 1}, "TRANS": {"dc": 0}},
            ),
        ):
            with self.subTest(inputs=inputs, sample=sample, data=data):
                self.assertFalse(
                    stopped_phone_pause_reset_ready(inputs, status, sample, data)
                )


if __name__ == "__main__":
    unittest.main()
