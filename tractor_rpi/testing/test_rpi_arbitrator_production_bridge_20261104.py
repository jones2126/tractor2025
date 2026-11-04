#!/usr/bin/env python3

import os
import unittest
from unittest.mock import patch

from tractor_rpi.teensy_serial_bridge_rpi_arbitrator_20261104 import (
    EXPECTED_FIRMWARE,
    MOTION_GATE,
    ProductionFeedReceiver,
    require_production_environment,
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


if __name__ == "__main__":
    unittest.main()
