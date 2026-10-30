#!/usr/bin/env python3

import importlib.util
import ipaddress
import json
import os
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
import wifi_primary_control_20261022 as primary


class FakeSerial:
    out_waiting = 0

    def __init__(self):
        self.data = b""

    def write(self, data):
        self.data += data

    def flush(self):
        return None


class FakeClock:
    def __init__(self, now=100.0):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class WifiPrimaryTests(unittest.TestCase):
    def test_server_phone_freshness_matches_teensy_watchdog(self):
        self.assertEqual(0.5, primary.SERVER_PHONE_FRESH_S)

    def test_recovery_stable_time_uses_shared_adjustable_setting(self):
        self.assertEqual(
            primary.AUTO_LINK_STABLE_SECONDS,
            primary.WifiPrimaryState.__init__.__defaults__[-1],
        )

    def make_state(self, clock=None, stable_s=5.0, window_s=30.0):
        temporary = tempfile.TemporaryDirectory()
        state = primary.WifiPrimaryState(
            "127.0.0.1",
            True,
            Path(temporary.name) / "control.jsonl",
            clock=clock,
            auto_link_stable_s=stable_s,
            auto_recovery_window_s=window_s,
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

    def test_dashboard_diagnostics_file_is_atomic_and_contains_no_session_secret(self):
        temporary, state = self.make_state()
        try:
            state.diagnostics_status_path = Path(temporary.name) / "diagnostics.json"
            state.owner_session = "must-not-leak"
            state.session_claim_count = 3
            state.session_invalidation_count = 2
            state.publish_diagnostics()
            text = state.diagnostics_status_path.read_text(encoding="utf-8")
            record = json.loads(text)
            self.assertNotIn("must-not-leak", text)
            self.assertEqual(3, record["session_claim_count"])
            self.assertFalse(record["communications_degraded"]["enabled"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_phone_flight_recorder_is_diagnostic_only(self):
        temporary, state = self.make_state()
        try:
            owner_before = state.owner_at
            result = state.ingest_client_events(
                {
                    "events": [
                        {
                            "time": "2026-10-19T12:00:00.000Z",
                            "event": "visibility_change",
                            "visibility": "hidden",
                            "sequence": 8,
                        }
                    ]
                },
                "192.168.10.232",
            )
            self.assertEqual(1, result["accepted"])
            self.assertEqual(owner_before, state.owner_at)
            record = json.loads(state.client_log_path.read_text(encoding="utf-8"))
            self.assertEqual("192.168.10.232", record["client_ip"])
            self.assertEqual("visibility_change", record["event"]["event"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_phone_flight_recorder_deduplicates_retried_event_ids(self):
        temporary, state = self.make_state()
        try:
            event = {
                "event_id": "stable-event-id",
                "time": "2026-10-29T12:00:00.000Z",
                "event": "send_scheduler_summary",
            }
            first = state.ingest_client_events({"events": [event]}, "192.168.10.232")
            second = state.ingest_client_events({"events": [event]}, "192.168.10.232")
            self.assertEqual({"accepted": 1, "duplicates": 0}, first)
            self.assertEqual({"accepted": 0, "duplicates": 1}, second)
            self.assertEqual(1, state.client_events_deduplicated)
            self.assertEqual(1, len(state.client_log_path.read_text().splitlines()))
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_phone_flight_recorder_rejects_unbounded_batches(self):
        temporary, state = self.make_state()
        try:
            with self.assertRaises(ValueError):
                state.ingest_client_events({"events": [{}] * 65}, "192.168.10.232")
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_phone_page_has_persistent_flight_recorder_and_wake_lock(self):
        page = primary.PAGE_PATH.read_text(encoding="utf-8")
        for marker in (
            "FLIGHT_MAX=1500",
            "localStorage.setItem(FLIGHT_KEY",
            "/api/client-log",
            "navigator.wakeLock.request('screen')",
            "connection_hint_changed",
            "session_cleared_after_visibility_restore",
            "ACK_DEADLINE_MS=500",
            "ack_watchdog_expired",
            "new AbortController()",
            "sentSession===session",
            "connection_path_changed_fail_closed",
            "AUTO WAIT — LINK RECOVERY",
            "recover_auto:recoverAuto",
            "autoRecoveryEligible=false",
            "window.addEventListener('pagehide'",
            "send_scheduler_summary",
            "clock_offset_estimate_ms",
            "api_timeout_",
            "request_id",
        ):
            self.assertIn(marker, page)

    def test_phone_page_displays_server_verified_dry_run_banner(self):
        page = primary.PAGE_PATH.read_text(encoding="utf-8")
        self.assertIn("DRY RUN — UDP 6004 DISABLED", page)
        self.assertIn("NO TRACTOR MOTION COMMANDS", page)
        self.assertIn("classList.toggle('dry-run-on',d.dry_run===true)", page)

    def test_phone_page_fails_closed_when_browser_reports_offline(self):
        page = primary.PAGE_PATH.read_text(encoding="utf-8")
        self.assertIn("failClosed('browser_offline_fail_closed')", page)
        self.assertNotIn("reason='browser_offline';render();send(true)", page)

    def test_simulated_bridge_is_explicitly_marked(self):
        temporary, state = self.make_state()
        state.bridge = {}
        worker = threading.Thread(target=primary.simulated_bridge_loop, args=(state,))
        try:
            worker.start()
            time.sleep(0.08)
            snapshot = state.snapshot()
            self.assertTrue(snapshot["system"]["simulated"])
            self.assertTrue(snapshot["wifi_control"]["simulated"])
            self.assertEqual(
                primary.EXPECTED_FIRMWARE,
                snapshot["system"]["firmware"],
            )
        finally:
            state.running = False
            worker.join(1)
            state.close()
            temporary.cleanup()

    def test_manual_expiry_latches_pause_and_requires_a_new_session(self):
        clock = FakeClock()
        temporary, state = self.make_state(clock)
        try:
            claim = state.claim("phone-a")
            old_session = claim["session"]
            state.requested_mode = "manual"
            state.phone_mode = "manual"
            state.effective_state = "manual_running"
            state.last_command = {
                "client_id": "phone-a",
                "sequence": 4,
                "mode": "manual",
            }
            state.owner_at = clock() - 2
            with mock.patch.object(state, "send_stop_burst"):
                state.safety_step()
            self.assertEqual("pause", state.phone_mode)
            self.assertEqual("pause_latched", state.effective_state)
            self.assertIsNone(state.owner_session)
            status, result = state.accept_command(
                {
                    "client_id": "phone-a",
                    "session": old_session,
                    "sequence": 5,
                    "mode": "manual",
                    "drive_percent": 20,
                    "steering_percent": 0,
                }
            )
            self.assertEqual(409, int(status))
            self.assertTrue(result["reclaim"])
            self.assertEqual("pause", result["phone_mode"])
        finally:
            state.close()
            temporary.cleanup()

    def arm_auto(self, state, client_id="phone-a"):
        claim = state.claim(client_id)
        status, result = state.accept_command(
            {
                "client_id": client_id,
                "session": claim["session"],
                "sequence": 1,
                "mode": "auto",
                "drive_percent": 0,
                "steering_percent": 0,
                "reason": "guarded_auto",
            }
        )
        self.assertEqual(200, int(status))
        self.assertEqual("auto_running", result["effective_state"])
        return claim

    def recovery_heartbeat(self, state, claim, sequence, mode="auto", reason="heartbeat"):
        return state.accept_command(
            {
                "client_id": "phone-a",
                "session": claim["session"],
                "sequence": sequence,
                "mode": mode,
                "drive_percent": 0,
                "steering_percent": 0,
                "reason": reason,
            }
        )

    def test_auto_timeout_stops_and_enters_link_wait_with_requested_auto(self):
        clock = FakeClock()
        temporary, state = self.make_state(clock)
        try:
            self.arm_auto(state)
            clock.advance(primary.SERVER_PHONE_FRESH_S + 0.001)
            with mock.patch.object(state, "send_stop_burst") as stop:
                self.assertTrue(state.safety_step())
            stop.assert_called_once()
            self.assertEqual("auto", state.requested_mode)
            self.assertEqual("auto_link_wait", state.effective_state)
            self.assertTrue(state.recovery_pending())
            self.assertIsNone(state.owner_session)
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_same_page_recovers_after_adjustable_stable_link_gate(self):
        clock = FakeClock()
        temporary, state = self.make_state(clock, stable_s=5.0)
        try:
            self.arm_auto(state)
            clock.advance(0.501)
            with mock.patch.object(state, "send_stop_burst"):
                state.safety_step()
                recovery = state.claim("phone-a", recover_auto=True)
            self.assertEqual("auto", recovery["phone_mode"])
            self.assertEqual("auto_link_stabilizing", recovery["effective_state"])
            status, result = self.recovery_heartbeat(state, recovery, 1)
            self.assertEqual(200, int(status))
            self.assertEqual("pause", result["decision"])
            clock.advance(4.999)
            _, result = self.recovery_heartbeat(state, recovery, 2)
            self.assertEqual("auto_link_stabilizing", result["effective_state"])
            clock.advance(0.001)
            _, result = self.recovery_heartbeat(state, recovery, 3)
            self.assertEqual("auto_running", result["effective_state"])
            self.assertFalse(result["recovery_pending"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_recovery_at_deadline_latches_pause(self):
        clock = FakeClock()
        temporary, state = self.make_state(clock, window_s=30.0)
        try:
            self.arm_auto(state)
            clock.advance(0.501)
            with mock.patch.object(state, "send_stop_burst"):
                state.safety_step()
                deadline = state.recovery_deadline
                clock.now = deadline
                state.safety_step()
            self.assertEqual("pause", state.requested_mode)
            self.assertEqual("pause_latched", state.effective_state)
            self.assertFalse(state.recovery_pending())
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_explicit_pause_and_manual_cancel_recovery(self):
        for selected, reason, expected_state in (
            ("pause", "guarded_pause", "pause_latched"),
            ("manual", "guarded_manual", "manual_running"),
        ):
            with self.subTest(selected=selected):
                clock = FakeClock()
                temporary, state = self.make_state(clock)
                try:
                    self.arm_auto(state)
                    clock.advance(0.501)
                    with mock.patch.object(state, "send_stop_burst"):
                        state.safety_step()
                        recovery = state.claim("phone-a", recover_auto=True)
                    _, result = self.recovery_heartbeat(
                        state, recovery, 1, mode=selected, reason=reason
                    )
                    self.assertEqual(expected_state, result["effective_state"])
                    self.assertFalse(result["recovery_pending"])
                finally:
                    state.running = False
                    state.close()
                    temporary.cleanup()

    def test_software_estop_cancels_recovery_and_remains_latched(self):
        clock = FakeClock()
        temporary, state = self.make_state(clock)
        try:
            self.arm_auto(state)
            clock.advance(0.501)
            with mock.patch.object(state, "send_stop_burst"):
                state.safety_step()
                recovery = state.claim("phone-a", recover_auto=True)
            _, result = self.recovery_heartbeat(
                state, recovery, 1, mode="estop", reason="guarded_estop"
            )
            self.assertEqual("estop", result["phone_mode"])
            self.assertEqual("estop_latched", result["effective_state"])
            self.assertFalse(result["recovery_pending"])
            status, result = self.recovery_heartbeat(
                state, recovery, 2, mode="auto", reason="guarded_auto"
            )
            self.assertEqual(409, int(status))
            self.assertIn("E-stop is latched", result["error"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_different_client_cannot_inherit_auto_recovery(self):
        clock = FakeClock()
        temporary, state = self.make_state(clock)
        try:
            self.arm_auto(state)
            clock.advance(0.501)
            with mock.patch.object(state, "send_stop_burst"):
                state.safety_step()
            clock.advance(primary.base.OWNER_EXPIRE_S)
            with mock.patch.object(state, "send_stop_burst"):
                claim = state.claim("phone-b", recover_auto=True)
            self.assertEqual("pause", claim["phone_mode"])
            self.assertEqual("pause_latched", claim["effective_state"])
            self.assertFalse(claim["automatic_link_recovery"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_repeated_failure_does_not_extend_recovery_deadline(self):
        clock = FakeClock()
        temporary, state = self.make_state(clock)
        try:
            self.arm_auto(state)
            clock.advance(0.501)
            with mock.patch.object(state, "send_stop_burst"):
                state.safety_step()
                original_deadline = state.recovery_deadline
                recovery = state.claim("phone-a", recover_auto=True)
                self.recovery_heartbeat(state, recovery, 1)
                clock.advance(0.501)
                state.safety_step()
            self.assertEqual("auto_link_wait", state.effective_state)
            self.assertEqual(original_deadline, state.recovery_deadline)
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_claim_reports_that_link_recovery_is_not_automatic(self):
        temporary, state = self.make_state()
        try:
            claim = state.claim("phone-a")
            self.assertFalse(claim["automatic_link_recovery"])
            self.assertEqual("pause", claim["phone_mode"])
        finally:
            state.running = False
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

    def test_zerotier_status_notice_requires_three_successful_peer_checks(self):
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
            self.assertEqual("Tractor01 ZeroTier status available", notices[0][0])
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
                "Tractor01 ZeroTier status available",
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
                    "Tractor01 ZeroTier status available",
                    "Tractor01 ZeroTier status recovered",
                ],
                [notice[0] for notice in notices],
            )
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_control_network_accepts_lan_and_rejects_zerotier(self):
        network = ipaddress.ip_network(primary.DEFAULT_CONTROL_SUBNET)
        self.assertTrue(
            primary.base.client_in_control_network("192.168.10.232", network)
        )
        self.assertFalse(
            primary.base.client_in_control_network("192.168.193.36", network)
        )

    def test_authority_policies_make_zerotier_the_explicit_primary(self):
        zerotier = primary.authority_policy("zerotier-primary")
        self.assertEqual("192.168.193.0/24", str(zerotier.network))
        self.assertTrue(zerotier.allow_auto)
        self.assertIn("production target", zerotier.qualification)
        mission = primary.authority_policy("tractorfield-mission")
        recovery = primary.authority_policy("onboard-router-recovery")
        self.assertTrue(mission.allow_auto)
        self.assertFalse(recovery.allow_auto)
        self.assertEqual("192.168.8.0/24", str(recovery.network))
        self.assertEqual(-100, recovery.min_drive_percent)
        self.assertEqual(100, recovery.max_drive_percent)
        with self.assertRaisesRegex(ValueError, "ZeroTier motion authority"):
            primary.authority_policy(
                "tractorfield-mission", "192.168.193.0/24"
            )
        dry_run = primary.authority_policy(
            "tractorfield-mission",
            "192.168.193.0/24",
            allow_zerotier_dry_run=True,
        )
        self.assertIn("UNQUALIFIED", dry_run.qualification)

    def test_manual_recovery_policy_rejects_auto(self):
        temporary = tempfile.TemporaryDirectory()
        state = primary.WifiPrimaryState(
            "127.0.0.1",
            True,
            Path(temporary.name) / "control.jsonl",
            primary.authority_policy("onboard-router-recovery"),
        )
        state.bridge = {
            "system": {"firmware": primary.EXPECTED_FIRMWARE},
            "wifi_control": {"estop_latched": 0},
        }
        state.bridge_at = time.monotonic()
        try:
            claim = state.claim("phone-a")
            status, result = state.accept_command(
                {
                    "client_id": "phone-a",
                    "session": claim["session"],
                    "sequence": 1,
                    "mode": "auto",
                    "drive_percent": 0,
                    "steering_percent": 0,
                    "reason": "guarded_auto",
                }
            )
            self.assertEqual(409, int(status))
            self.assertIn("Auto is disabled", result["error"])
            self.assertEqual("pause", state.phone_mode)
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_manual_recovery_policy_accepts_full_drive_range(self):
        temporary = tempfile.TemporaryDirectory()
        state = primary.WifiPrimaryState(
            "127.0.0.1",
            True,
            Path(temporary.name) / "control.jsonl",
            primary.authority_policy("onboard-router-recovery"),
        )
        state.bridge = {
            "system": {"firmware": primary.EXPECTED_FIRMWARE},
            "wifi_control": {"estop_latched": 0},
        }
        state.bridge_at = time.monotonic()
        try:
            claim = state.claim("phone-a")
            for sequence, drive, reason in (
                (1, 100, "guarded_manual"),
                (2, -100, "manual_range_test"),
            ):
                status, result = state.accept_command(
                    {
                        "client_id": "phone-a",
                        "session": claim["session"],
                        "sequence": sequence,
                        "mode": "manual",
                        "drive_percent": drive,
                        "steering_percent": 100,
                        "reason": reason,
                    }
                )
                self.assertEqual(200, int(status), result)
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

    def test_operator_key_file_is_created_once_and_reused(self):
        with tempfile.TemporaryDirectory() as temporary:
            key_path = Path(temporary) / "control" / "operator.key"
            first = primary.load_or_create_operator_key(key_path)
            second = primary.load_or_create_operator_key(key_path)
            self.assertEqual(first, second)
            self.assertGreaterEqual(len(first), 24)
            if os.name != "nt":
                self.assertEqual(0o600, key_path.stat().st_mode & 0o777)

    def test_invalid_operator_key_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            key_path = Path(temporary) / "operator.key"
            key_path.write_text("too short\n", encoding="ascii")
            with self.assertRaises(RuntimeError):
                primary.load_or_create_operator_key(key_path)


if __name__ == "__main__":
    unittest.main()
