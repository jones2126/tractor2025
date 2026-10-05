#!/usr/bin/env python3

import importlib.util
import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
WEBRTC = ROOT / "tractor_rpi" / "testing" / "webrtc"
RPI = ROOT / "tractor_rpi"
for path in (str(WEBRTC), str(RPI)):
    if path not in sys.path:
        sys.path.insert(0, path)

import teensy_serial_bridge_20260728 as bridge_module
import wifi_primary_control_20261003 as primary


class FakeSerial:
    out_waiting = 0

    def __init__(self):
        self.data = b""

    def write(self, data):
        self.data += data

    def flush(self):
        return None


class WifiPrimaryTests(unittest.TestCase):
    def make_state(self):
        temporary = tempfile.TemporaryDirectory()
        state = primary.WifiPrimaryState(
            "127.0.0.1", True, Path(temporary.name) / "control.jsonl"
        )
        state.bridge = {
            "system": {"firmware": primary.EXPECTED_FIRMWARE},
            "steering": {"mode": 2, "cmd_age_ms": 0},
            "transmission": {"mode": 2},
        }
        state.bridge_at = time.monotonic()
        return temporary, state

    def test_claim_and_manual_command_do_not_require_nrf(self):
        temporary, state = self.make_state()
        try:
            claim = state.claim("phone-a")
            status, result = state.accept_command(
                {
                    "client_id": "phone-a",
                    "session": claim["session"],
                    "sequence": 1,
                    "mode": "manual",
                    "drive_percent": 20,
                    "steering_percent": -100,
                    "reason": "guarded_manual",
                    "client_time_ms": time.time() * 1000,
                    "client_rtt_ms": None,
                }
            )
            self.assertEqual(200, int(status))
            self.assertTrue(result["accepted"])
            self.assertEqual("manual", state.phone_mode)
            self.assertEqual(-100, state.last_command["steering_percent"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_guarded_auto_is_accepted_and_zeroes_slider_demands(self):
        temporary, state = self.make_state()
        try:
            claim = state.claim("phone-a")
            status, result = state.accept_command(
                {
                    "client_id": "phone-a",
                    "session": claim["session"],
                    "sequence": 1,
                    "mode": "auto",
                    "drive_percent": 75,
                    "steering_percent": -80,
                    "reason": "guarded_auto",
                    "client_time_ms": time.time() * 1000,
                    "client_rtt_ms": None,
                }
            )
            self.assertEqual(200, int(status))
            self.assertTrue(result["accepted"])
            self.assertEqual("auto", state.phone_mode)
            self.assertEqual(0.0, state.last_command["drive_percent"])
            self.assertEqual(0.0, state.last_command["steering_percent"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_estop_latches_then_guarded_second_press_resets_to_pause(self):
        temporary, state = self.make_state()
        try:
            claim = state.claim("phone-a")
            common = {
                "client_id": "phone-a",
                "session": claim["session"],
                "drive_percent": 0,
                "steering_percent": 0,
                "client_time_ms": time.time() * 1000,
                "client_rtt_ms": 20,
            }
            status, result = state.accept_command(
                {
                    **common,
                    "sequence": 1,
                    "mode": "estop",
                    "reason": "guarded_estop",
                }
            )
            self.assertEqual(200, int(status))
            self.assertEqual("estop", result["phone_mode"])

            status, result = state.accept_command(
                {
                    **common,
                    "sequence": 2,
                    "mode": "pause",
                    "reason": "guarded_estop_reset",
                }
            )
            self.assertEqual(200, int(status))
            self.assertEqual("reset_pause", result["decision"])
            self.assertEqual("pause", result["phone_mode"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_new_session_preserves_reported_teensy_estop_latch(self):
        temporary, state = self.make_state()
        try:
            state.bridge["wifi_control"] = {"estop_latched": 1}
            claim = state.claim("phone-a")
            self.assertEqual("estop", claim["phone_mode"])
            self.assertEqual("estop", state.phone_mode)
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_state_endpoint_includes_teensy_wifi_control_feedback(self):
        temporary, state = self.make_state()
        try:
            state.bridge["wifi_control"] = {
                "mode": 3,
                "heartbeat_fresh": 1,
                "estop_latched": 1,
                "command_age_ms": 80,
            }
            snapshot = state.snapshot()
            self.assertEqual(1, snapshot["wifi_control"]["estop_latched"])
            self.assertEqual(80, snapshot["wifi_control"]["command_age_ms"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_expiry_sends_pause_but_retains_manual_for_recovery(self):
        temporary, state = self.make_state()
        try:
            state.phone_mode = "manual"
            state.owner_at = time.monotonic() - 2
            worker = threading.Thread(target=state.safety_loop)
            worker.start()
            time.sleep(0.12)
            state.running = False
            worker.join(1)
            self.assertEqual("manual", state.phone_mode)
            self.assertEqual("phone_freshness_expired", state.last_stop_reason)
        finally:
            state.close()
            temporary.cleanup()

    def test_bridge_adds_explicit_phone_mode_to_serial_message(self):
        obj = bridge_module.TeensySerialBridge.__new__(
            bridge_module.TeensySerialBridge
        )
        obj.ser = FakeSerial()
        obj.cmd_vel_sent_count = 0
        obj.stats = {"commands_sent": 0}
        obj.last_cmd_vel = {}
        self.assertTrue(obj.send_wifi_drive_to_teensy(25, -1, "manual"))
        self.assertEqual(b"WIFI,25.0,-1.0000,1\n", obj.ser.data)

        obj.ser.data = b""
        self.assertTrue(obj.send_wifi_drive_to_teensy(0, 0, "auto"))
        self.assertEqual(b"WIFI,0.0,0.0000,2\n", obj.ser.data)

        obj.ser.data = b""
        self.assertTrue(obj.send_wifi_drive_to_teensy(0, 0, "estop"))
        self.assertEqual(b"WIFI,0.0,0.0000,3\n", obj.ser.data)

        obj.ser.data = b""
        self.assertTrue(obj.send_wifi_drive_to_teensy(0, 0, "reset_pause"))
        self.assertEqual(b"WIFI,0.0,0.0000,4\n", obj.ser.data)

    def make_monitor(self, local_results, zerotier_results, notices):
        temporary, state = self.make_state()
        local_iter = iter(local_results)
        zero_iter = iter(zerotier_results)

        def notify(topic, title, message, click_url, tags):
            notices.append((title, click_url))
            return True

        monitor = primary.ConnectivityMonitor(
            state,
            "test-topic",
            "https://192.168.1.151:8765/?key=test",
            "https://192.168.193.76:8765/?key=test",
            False,
            initial_window_s=90,
            stable_checks=3,
            local_check=lambda: next(local_iter),
            zerotier_check=lambda: next(zero_iter),
            notifier=notify,
            clock=lambda: 0.0,
        )
        return temporary, state, monitor

    def test_connectivity_monitor_sends_local_notice_once(self):
        notices = []
        temporary, state, monitor = self.make_monitor(
            [True, True],
            [(False, "not ready"), (False, "not ready")],
            notices,
        )
        try:
            monitor.step(0)
            monitor.step(3)
            self.assertEqual(
                ["Tractor01 local control ready"], [notice[0] for notice in notices]
            )
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_zerotier_notice_requires_three_successful_peer_checks(self):
        notices = []
        temporary, state, monitor = self.make_monitor(
            [False, False, False],
            [(True, "reachable"), (True, "reachable"), (True, "reachable")],
            notices,
        )
        try:
            monitor.step(0)
            monitor.step(3)
            self.assertEqual([], notices)
            monitor.step(6)
            self.assertEqual("Tractor01 ZeroTier control ready", notices[0][0])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_delayed_notice_does_not_stop_retries(self):
        notices = []
        temporary, state, monitor = self.make_monitor(
            [True] * 6,
            [
                (False, "unreachable"),
                (False, "unreachable"),
                (False, "unreachable"),
                (True, "reachable"),
                (True, "reachable"),
                (True, "reachable"),
            ],
            notices,
        )
        try:
            monitor.step(0)
            monitor.step(45)
            monitor.step(90)
            self.assertIn("Tractor01 ZeroTier delayed", [notice[0] for notice in notices])
            monitor.step(93)
            monitor.step(96)
            monitor.step(99)
            self.assertIn(
                "Tractor01 ZeroTier control ready",
                [notice[0] for notice in notices],
            )
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_zerotier_recovery_after_loss_uses_recovered_notice(self):
        notices = []
        temporary, state, monitor = self.make_monitor(
            [False] * 9,
            [
                (True, "reachable"),
                (True, "reachable"),
                (True, "reachable"),
                (False, "lost"),
                (False, "lost"),
                (False, "lost"),
                (True, "reachable"),
                (True, "reachable"),
                (True, "reachable"),
            ],
            notices,
        )
        try:
            for index in range(9):
                monitor.step(index * 3)
            self.assertEqual(
                [
                    "Tractor01 ZeroTier control ready",
                    "Tractor01 ZeroTier recovered",
                ],
                [notice[0] for notice in notices],
            )
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_failed_local_notice_retries_after_backoff(self):
        temporary, state = self.make_state()
        attempts = []

        def notify(topic, title, message, click_url, tags):
            attempts.append(title)
            return len(attempts) > 1

        monitor = primary.ConnectivityMonitor(
            state,
            "test-topic",
            "https://local/?key=test",
            "https://zerotier/?key=test",
            False,
            notify_retry_s=15,
            local_check=lambda: True,
            zerotier_check=lambda: (False, "not ready"),
            notifier=notify,
            clock=lambda: 0.0,
        )
        try:
            monitor.step(0)
            monitor.step(10)
            self.assertEqual(1, len(attempts))
            monitor.step(15)
            self.assertEqual(2, len(attempts))
            self.assertTrue(monitor.local_notice_sent)
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_private_local_addresses_excludes_zerotier_and_public_interfaces(self):
        completed = type(
            "Completed",
            (),
            {
                "returncode": 0,
                "stdout": (
                    "2: eth0    inet 192.168.8.20/24 brd 192.168.8.255 scope global eth0\n"
                    "3: wlan0   inet 192.168.10.120/24 brd 192.168.10.255 scope global wlan0\n"
                    "4: ztabc   inet 192.168.193.76/24 brd 192.168.193.255 scope global ztabc\n"
                    "5: usb0    inet 8.8.8.8/24 brd 8.8.8.255 scope global usb0\n"
                ),
            },
        )()
        with mock.patch.object(primary.subprocess, "run", return_value=completed):
            self.assertEqual(
                ["192.168.8.20", "192.168.10.120"],
                primary.private_local_addresses(),
            )

    def test_local_control_hostname_is_covered_by_deployed_certificate_name(self):
        self.assertEqual("raspberrypi.local", primary.LOCAL_CONTROL_HOSTNAME)


if __name__ == "__main__":
    unittest.main()
