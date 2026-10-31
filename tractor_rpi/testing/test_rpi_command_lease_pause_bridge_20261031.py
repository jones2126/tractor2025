#!/usr/bin/env python3

import json
import sys
import tempfile
import unittest
from pathlib import Path

from tractor_rpi.rpi_command_lease_adapter_20261031 import RpiCommandLeaseAdapter
from tractor_rpi.rpi_command_lease_protocol_20261031 import (
    RpiCommandMode,
    decode_frame,
)
from tractor_rpi.rpi_command_lease_runtime_20261031 import RpiCommandLeaseRuntime
from tractor_rpi.teensy_serial_bridge_rpi_lease_pause_20261031 import (
    PauseOnlyUdpGate,
    pause_supervisor_inputs,
)


ROOT = Path(__file__).resolve().parents[2]
BRIDGE = ROOT / "tractor_rpi/teensy_serial_bridge_rpi_lease_pause_20261031.py"
SERVER = ROOT / "tractor_rpi/testing/webrtc/wifi_rpi_lease_pause_only_20261031.py"
PAGE = SERVER.with_suffix(".html")
WEBRTC = SERVER.parent
if str(WEBRTC) not in sys.path:
    sys.path.insert(0, str(WEBRTC))

import wifi_primary_control_20261022 as wifi_base
from wifi_rpi_lease_pause_only_20261031 import PauseOnlyWifiState


class FakeSerial:
    def __init__(self):
        self.frames = []

    def write(self, data):
        self.frames.append(data)
        return len(data)

    def flush(self):
        return None


class PauseOnlyBridgeTests(unittest.TestCase):
    def test_exact_zero_pause_packet_is_accepted_for_evidence(self):
        gate = PauseOnlyUdpGate()
        self.assertTrue(
            gate.ingest(
                {"phone_mode": "pause", "drive_percent": 0, "angular_z": 0},
                now_monotonic_s=10.0,
            )
        )
        self.assertEqual(1, gate.accepted_pause_packets)
        self.assertEqual(0, gate.rejected_packets)

    def test_every_motion_or_pure_pursuit_shape_is_rejected(self):
        packets = (
            {"phone_mode": "manual", "drive_percent": 0, "angular_z": 0},
            {"phone_mode": "auto", "drive_percent": 0, "angular_z": 0},
            {"phone_mode": "pause", "drive_percent": 1, "angular_z": 0},
            {"phone_mode": "pause", "drive_percent": 0, "angular_z": 0.1},
            {"linear_x": 0, "angular_z": 0},
            "not an object",
        )
        gate = PauseOnlyUdpGate()
        for index, packet in enumerate(packets):
            with self.subTest(packet=packet):
                self.assertFalse(gate.ingest(packet, now_monotonic_s=float(index)))
        self.assertEqual(len(packets), gate.rejected_packets)

    def test_runtime_can_only_emit_pause_with_all_health_inputs_false(self):
        serial = FakeSerial()
        adapter = RpiCommandLeaseAdapter(serial, session=42)
        runtime = RpiCommandLeaseRuntime(adapter)
        result = runtime.tick(
            now_monotonic_s=1.0,
            inputs=pause_supervisor_inputs(1.0),
        )
        self.assertIsNotNone(result.frame_sent)
        command = decode_frame(serial.frames[-1])
        self.assertEqual(RpiCommandMode.PAUSE, command.mode)
        self.assertEqual(0.0, command.drive)
        self.assertEqual(0.0, command.steering)

    def test_live_bridge_source_has_no_legacy_or_motion_send_call(self):
        source = BRIDGE.read_text(encoding="utf-8")
        self.assertNotIn("send_cmd_vel_to_teensy(", source)
        self.assertNotIn("send_wifi_drive_to_teensy(", source)
        self.assertNotIn("send_manual(", source)
        self.assertNotIn("send_auto(", source)
        self.assertIn("requested_mode=RpiCommandMode.PAUSE", source)

    def test_phone_wrapper_is_candidate_specific_and_pause_only(self):
        source = SERVER.read_text(encoding="utf-8")
        self.assertIn("teensy_main_20261031_rpi_command_lease_v2", source)
        self.assertIn('mode != "pause"', source)
        self.assertIn("min_drive_percent=0", source)
        self.assertIn("max_drive_percent=0", source)
        self.assertIn('result["rpi_command_lease"]', source)
        self.assertNotIn("approved_for_motion", source)
        page = PAGE.read_text(encoding="utf-8")
        self.assertIn("MOTOR-OFF TEST", page)
        self.assertIn("PAUSE ONLY", page)
        self.assertNotIn('data-mode="manual"', page)
        self.assertNotIn('data-mode="auto"', page)
        self.assertNotIn('type="range"', page)

    def test_phone_wrapper_rejects_motion_and_accepts_zero_pause(self):
        with tempfile.TemporaryDirectory() as temporary:
            state = PauseOnlyWifiState(
                "127.0.0.1",
                True,
                Path(temporary) / "server.jsonl",
                policy=wifi_base.AUTHORITY_POLICIES["onboard-router-mission"],
            )
            try:
                status, result = state.accept_command(
                    {
                        "mode": "manual",
                        "drive_percent": 1,
                        "steering_percent": 0,
                    },
                    "192.168.8.190",
                )
                self.assertEqual(409, status)
                self.assertFalse(result["accepted"])
                self.assertEqual("pause", result["phone_mode"])

                state.owner_client = "phone"
                state.owner_session = "session"
                state.highest_sequence = -1
                status, result = state.accept_command(
                    {
                        "client_id": "phone",
                        "session": "session",
                        "sequence": 1,
                        "mode": "pause",
                        "drive_percent": 0,
                        "steering_percent": 0,
                        "reason": "heartbeat",
                        "client_time_ms": 0,
                    },
                    "192.168.8.190",
                )
                self.assertEqual(200, status)
                self.assertTrue(result["accepted"])
                self.assertEqual("pause", result["decision"])
            finally:
                state.close()


if __name__ == "__main__":
    unittest.main()
