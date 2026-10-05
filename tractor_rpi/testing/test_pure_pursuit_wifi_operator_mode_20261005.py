#!/usr/bin/env python3

import importlib.util
import math
import threading
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tractor_rpi" / "pure-pursuit" / "pure_pursuit_controller_20260915.py"
SPEC = importlib.util.spec_from_file_location("pursuit_wifi_mode", SOURCE)
controller = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(controller)


class WifiOperatorModeTests(unittest.TestCase):
    def receiver_without_socket(self):
        receiver = controller.HandheldStatusReceiver.__new__(controller.HandheldStatusReceiver)
        receiver._wifi_fresh_since = None
        return receiver

    @staticmethod
    def message(mode, fresh=1, estop=0, age_ms=20, steering_mode=0):
        return {
            "steering": {"mode": steering_mode, "state": "OK"},
            "wifi_control": {
                "mode": mode,
                "heartbeat_fresh": fresh,
                "estop_latched": estop,
                "command_age_ms": age_ms,
            },
        }

    def test_wifi_manual_overrides_low_level_auto_mode(self):
        receiver = self.receiver_without_socket()
        status = receiver._decode_message(self.message(1, steering_mode=0), 100.0)
        self.assertEqual(1, status["mode"])
        self.assertEqual("MANUAL", status["mode_name"])
        self.assertEqual("wifi", status["source"])

    def test_wifi_pause_and_auto_are_mapped_to_normalized_modes(self):
        receiver = self.receiver_without_socket()
        pause = receiver._decode_message(self.message(0), 100.0)
        auto = receiver._decode_message(self.message(2), 106.0)
        self.assertEqual(2, pause["mode"])
        self.assertEqual(0, auto["mode"])
        self.assertGreaterEqual(auto["link_stable_s"], 5.0)

    def test_stale_heartbeat_resets_stability_timer(self):
        receiver = self.receiver_without_socket()
        receiver._decode_message(self.message(0), 100.0)
        stable = receiver._decode_message(self.message(0), 106.0)
        stale = receiver._decode_message(self.message(0, fresh=0), 107.0)
        recovered = receiver._decode_message(self.message(0), 108.0)
        self.assertGreaterEqual(stable["link_stable_s"], 5.0)
        self.assertEqual(0.0, stale["link_stable_s"])
        self.assertEqual(0.0, recovered["link_stable_s"])

    def test_legacy_steering_mode_remains_supported(self):
        receiver = self.receiver_without_socket()
        status = receiver._decode_message({"steering": {"mode": 2, "state": "PAUSE"}}, 100.0)
        self.assertEqual(2, status["mode"])
        self.assertEqual("legacy_steering", status["source"])

    def test_nonfinite_command_age_is_not_fresh(self):
        receiver = self.receiver_without_socket()
        status = receiver._decode_message(self.message(2, age_ms=math.inf), 100.0)
        self.assertEqual(0.0, status["link_stable_s"])

    def test_auto_ready_requires_five_seconds_of_fresh_wifi(self):
        receiver = self.receiver_without_socket()
        receiver._lock = threading.Lock()
        receiver._latest = receiver._decode_message(self.message(2), 100.0)
        receiver._last_update = time.time()
        ready, reason, _ = receiver.auto_ready()
        self.assertFalse(ready)
        self.assertIn("stable for", reason)
        receiver._latest = receiver._decode_message(self.message(2), 106.0)
        receiver._last_update = time.time()
        ready, reason, _ = receiver.auto_ready()
        self.assertTrue(ready)
        self.assertEqual("", reason)


if __name__ == "__main__":
    unittest.main()
