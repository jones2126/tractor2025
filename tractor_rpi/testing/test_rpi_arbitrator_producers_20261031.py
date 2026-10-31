#!/usr/bin/env python3

from contextlib import nullcontext
import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class FakeSocket:
    def __init__(self):
        self.sent = []

    def sendto(self, data, address):
        self.sent.append((data, address))
        return len(data)


class RpiArbitratorProducerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        webrtc = ROOT / "tractor_rpi" / "testing" / "webrtc"
        sys.path.insert(0, str(webrtc))
        cls.phone_module = load(
            "wifi_rpi_arbitrator_control_test",
            webrtc / "wifi_rpi_arbitrator_control_20261031.py",
        )
        cls.pp_module = load(
            "pure_pursuit_arbitrator_feed_test",
            ROOT / "tractor_rpi" / "pure-pursuit"
            / "pure_pursuit_arbitrator_feed_20261031.py",
        )

    def test_phone_producer_marks_auto_link_loss_without_changing_intent(self):
        module = self.phone_module
        state = module.ArbitratorWifiState.__new__(module.ArbitratorWifiState)
        state._intent_sock = FakeSocket()
        state._intent_sequence = 0
        state._intent_producer_session = "producer"
        state._intent_lock = nullcontext()
        state.requested_mode = "auto"
        state.effective_state = "auto_link_wait"
        state.owner_session = None
        state.owner_at = 0.0
        state.phone_mode = "auto"
        state.clock = lambda: 10.0
        with mock.patch.object(
            module.base.WifiPrimaryState,
            "send_drive_command",
            autospec=True,
        ):
            state.send_drive_command(0.0, 0.0, 40, "phone_freshness_expired", "pause")
        payload = json.loads(state._intent_sock.sent[0][0])
        self.assertEqual("auto", payload["requested_mode"])
        self.assertFalse(payload["operator_link_fresh"])
        self.assertEqual("pause", payload["wire_mode"])

    def test_pure_pursuit_duplicator_preserves_dashboard_and_adds_route(self):
        sink = FakeSocket()
        wrapper = self.pp_module.SafetyDuplicatingSocket(
            sink,
            telemetry_port=6012,
            safety_port=6015,
            route_sha256="a" * 64,
        )
        original = {"controller_state": "RUNNING", "speed_cmd_mps": "1.0"}
        wrapper.sendto(json.dumps(original).encode(), ("127.0.0.1", 6012))
        self.assertEqual(2, len(sink.sent))
        self.assertEqual(original, json.loads(sink.sent[0][0]))
        safety = json.loads(sink.sent[1][0])
        self.assertEqual("pure_pursuit_safety", safety["kind"])
        self.assertEqual("a" * 64, safety["route_sha256"])
        self.assertEqual(1, safety["safety_sequence"])

    def test_normalized_route_hash_ignores_line_ending_style(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            a = Path(directory) / "a.csv"
            b = Path(directory) / "b.csv"
            a.write_bytes(b"one,two\r\n1,2\r\n")
            b.write_bytes(b"one,two\n1,2\n")
            self.assertEqual(
                self.pp_module.normalized_lf_sha256(a),
                self.pp_module.normalized_lf_sha256(b),
            )


if __name__ == "__main__":
    unittest.main()
