#!/usr/bin/env python3

import unittest

from tractor_rpi.rpi_arbitrator_inputs_20261031 import TeensySafetySample
from tractor_rpi.rpi_command_lease_adapter_20261031 import parse_rpi_ctl_line
from tractor_rpi.teensy_serial_bridge_rpi_arbitrator_motor_off_20261031 import (
    ArbitratorFeedReceiver,
    PauseOnlyBridge,
    TeensySerialBridge,
    validated_route_hash,
)


ROUTE = "a" * 64


def phone(sequence=1):
    return {
        "schema": 1, "kind": "phone_intent", "producer_session": "producer",
        "intent_sequence": sequence, "requested_mode": "auto",
        "operator_link_fresh": True, "manual_drive_percent": 0,
        "manual_steering": 0,
    }


def pursuit(sequence=1):
    return {
        "schema": 1, "kind": "pure_pursuit_safety", "route_sha256": ROUTE,
        "safety_sequence": sequence, "controller_state": "RUNNING",
        "speed_cmd_mps": 1.0, "steer_normalized": 0.2, "gps_age_s": 0.05,
        "fix_quality": "RTK Fixed", "head_valid": True,
        "heading_carrier": "fixed", "heading_accuracy_deg": 0.2,
        "path_progress_m": 10.0, "cross_track_err_m": 0.2,
    }


def teensy(now=1.0):
    status = parse_rpi_ctl_line(
        "1,100,RPI_CTL,m=0,f=1,u=1,l=0,a=10,ss=42,s=1,ok=1,rj=0,rr=0"
    )
    return TeensySafetySample(now, status, False, False, True, True)


class RpiArbitratorMotorOffConsumerTests(unittest.TestCase):
    def test_serial_scheduler_is_exact_inherited_pause_only_scheduler(self):
        self.assertIs(
            PauseOnlyBridge.command_lease_thread,
            TeensySerialBridge.command_lease_thread,
        )

    def test_route_hash_is_mandatory(self):
        self.assertEqual(ROUTE, validated_route_hash(ROUTE.upper()))
        for value in ("", "abc", "g" * 64):
            with self.assertRaises(ValueError):
                validated_route_hash(value)

    def test_feeds_are_joined_but_actual_output_is_always_pause_only(self):
        receiver = ArbitratorFeedReceiver(ROUTE)
        receiver.ingest(phone(), now_monotonic_s=1.0)
        receiver.ingest(pursuit(), now_monotonic_s=1.0)
        snapshot = receiver.snapshot(
            now_monotonic_s=1.1,
            teensy_sample=teensy(),
        )
        self.assertEqual(1, snapshot["accepted_phone_samples"])
        self.assertEqual(1, snapshot["accepted_pure_pursuit_samples"])
        self.assertEqual("PAUSE_ONLY", snapshot["actual_serial_output"])
        self.assertFalse(snapshot["approved_for_motion"])
        self.assertEqual("PAUSE_LATCHED", snapshot["joined_decision"])
        self.assertIn("physical_pause_released", snapshot["exact_reason"])

    def test_malformed_and_duplicate_feeds_fail_closed_with_exact_counts(self):
        receiver = ArbitratorFeedReceiver(ROUTE)
        receiver.ingest({"kind": "unknown"}, now_monotonic_s=1.0)
        receiver.ingest(phone(), now_monotonic_s=1.0)
        receiver.ingest(phone(), now_monotonic_s=1.1)
        snapshot = receiver.snapshot(now_monotonic_s=1.1, teensy_sample=teensy())
        self.assertEqual(1, snapshot["rejected_samples"])
        self.assertIn("duplicate phone", snapshot["input_fault"])
        self.assertEqual("PAUSE_ONLY", snapshot["actual_serial_output"])


if __name__ == "__main__":
    unittest.main()
