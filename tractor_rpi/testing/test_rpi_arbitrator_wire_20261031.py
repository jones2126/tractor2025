#!/usr/bin/env python3

import unittest

from tractor_rpi.rpi_arbitrator_wire_20261031 import (
    parse_phone_intent,
    parse_pure_pursuit_safety,
)
from tractor_rpi.rpi_command_lease_protocol_20261031 import RpiCommandMode


ROUTE = "a" * 64


class RpiArbitratorWireTests(unittest.TestCase):
    def test_phone_intent_keeps_auto_while_link_is_explicitly_stale(self):
        sample = parse_phone_intent({
            "schema": 1,
            "kind": "phone_intent",
            "producer_session": "server-1",
            "intent_sequence": 4,
            "requested_mode": "auto",
            "operator_link_fresh": False,
            "manual_drive_percent": 0,
            "manual_steering": 0,
        }, received_monotonic_s=12.0)
        self.assertEqual(RpiCommandMode.AUTO_MPS, sample.requested_mode)
        self.assertFalse(sample.operator_link_fresh)

    def test_pure_pursuit_feed_joins_command_health_and_route(self):
        command, health = parse_pure_pursuit_safety({
            "schema": 1,
            "kind": "pure_pursuit_safety",
            "route_sha256": ROUTE,
            "safety_sequence": 9,
            "controller_state": "RUNNING",
            "speed_cmd_mps": "1.0",
            "steer_normalized": "+0.2",
            "gps_age_s": "0.05",
            "fix_quality": "RTK Fixed",
            "head_valid": True,
            "heading_carrier": "fixed",
            "heading_accuracy_deg": "0.2",
            "path_progress_m": "12.5",
            "cross_track_err_m": "0.3",
        }, received_monotonic_s=20.0)
        self.assertEqual(ROUTE, command.route_sha256)
        self.assertEqual(1.0, command.speed_mps)
        self.assertEqual("RUNNING", health.controller_state)

    def test_wrong_schema_nonfinite_and_ambiguous_boolean_are_rejected(self):
        bad_phone = {
            "schema": 1, "kind": "phone_intent", "producer_session": "s",
            "intent_sequence": 1, "requested_mode": "auto",
            "operator_link_fresh": 1, "manual_drive_percent": 0,
            "manual_steering": 0,
        }
        with self.assertRaises(ValueError):
            parse_phone_intent(bad_phone, received_monotonic_s=1.0)

        bad_pp = {
            "schema": 1, "kind": "pure_pursuit_safety", "route_sha256": ROUTE,
            "safety_sequence": 1, "controller_state": "RUNNING",
            "speed_cmd_mps": float("nan"), "steer_normalized": 0,
            "gps_age_s": 0, "fix_quality": "RTK Fixed", "head_valid": True,
            "heading_carrier": "fixed", "heading_accuracy_deg": 0.1,
            "path_progress_m": 0, "cross_track_err_m": 0,
        }
        with self.assertRaises(ValueError):
            parse_pure_pursuit_safety(bad_pp, received_monotonic_s=1.0)


if __name__ == "__main__":
    unittest.main()
