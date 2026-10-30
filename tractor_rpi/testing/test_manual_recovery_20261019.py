#!/usr/bin/env python3

import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
WEBRTC = ROOT / "tractor_rpi" / "testing" / "webrtc"
for path in (str(WEBRTC), str(ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

import wifi_primary_control_20261022 as primary
from tractor_rpi.recovery import manual_recovery_workflow_20261019 as workflow


class ManualRecoveryTests(unittest.TestCase):
    def make_state(self):
        temporary = tempfile.TemporaryDirectory()
        state = primary.WifiPrimaryState(
            "127.0.0.1",
            True,
            Path(temporary.name) / "recovery.jsonl",
            primary.authority_policy("onboard-router-recovery"),
        )
        state.bridge = {
            "system": {"firmware": primary.EXPECTED_FIRMWARE},
            "steering": {"mode": 2, "state": "PAUSE", "current": 510},
            "transmission": {"mode": 2, "target": 2836, "current": 2834},
            "wifi_control": {"estop_latched": 0, "command_age_ms": 0},
        }
        state.bridge_at = time.monotonic()
        return temporary, state

    def test_recovery_claim_is_manual_only_and_reports_numeric_path(self):
        temporary, state = self.make_state()
        try:
            claim = state.claim("phone-a", "192.168.8.42")
            self.assertTrue(claim["recovery_only"])
            self.assertFalse(claim["auto_allowed"])
            self.assertEqual(-100, claim["min_drive_percent"])
            self.assertEqual(100, claim["max_drive_percent"])
            self.assertEqual("192.168.8.20", claim["numeric_host"])
            self.assertEqual("eth0", claim["interface_name"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_server_rejects_drive_outside_recovery_limits(self):
        temporary, state = self.make_state()
        try:
            claim = state.claim("phone-a")
            common = {
                "client_id": "phone-a",
                "session": claim["session"],
                "mode": "manual",
                "steering_percent": 0,
                "reason": "guarded_manual",
            }
            for sequence, demand in ((1, 101), (2, -101)):
                status, result = state.accept_command(
                    {**common, "sequence": sequence, "drive_percent": demand}
                )
                self.assertEqual(400, int(status))
                self.assertIn("-100 to +100", result["error"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_recovery_network_rejects_non_router_clients(self):
        network = primary.authority_policy("onboard-router-recovery").network
        self.assertTrue(primary.base.client_in_control_network("192.168.8.42", network))
        self.assertFalse(
            primary.base.client_in_control_network("192.168.193.42", network)
        )

    def test_second_phone_cannot_take_a_fresh_session(self):
        temporary, state = self.make_state()
        try:
            state.claim("phone-a", "192.168.8.42")
            with self.assertRaisesRegex(RuntimeError, "another phone"):
                state.claim("phone-b", "192.168.8.43")
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_reclaim_invalidates_the_old_session(self):
        temporary, state = self.make_state()
        try:
            first = state.claim("phone-a", "192.168.8.42")
            second = state.claim("phone-a", "192.168.8.42")
            status, result = state.accept_command(
                {
                    "client_id": "phone-a",
                    "session": first["session"],
                    "sequence": 1,
                    "mode": "manual",
                    "drive_percent": 0,
                    "steering_percent": 0,
                    "reason": "guarded_manual",
                }
            )
            self.assertNotEqual(first["session"], second["session"])
            self.assertEqual(409, int(status))
            self.assertTrue(result["reclaim"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_server_restart_state_is_pause_and_unowned(self):
        temporary, state = self.make_state()
        try:
            self.assertEqual("pause", state.phone_mode)
            self.assertIsNone(state.owner_session)
            self.assertFalse(state.snapshot()["session_active"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_command_log_correlates_source_session_and_actuators(self):
        temporary, state = self.make_state()
        try:
            claim = state.claim("phone-a", "192.168.8.42")
            state.accept_command(
                {
                    "client_id": "phone-a",
                    "session": claim["session"],
                    "sequence": 1,
                    "mode": "manual",
                    "drive_percent": 20,
                    "steering_percent": 10,
                    "reason": "guarded_manual",
                },
                "192.168.8.42",
            )
            records = [json.loads(line) for line in state.log_path.read_text().splitlines()]
            command = next(record for record in records if record["event"] == "command")
            self.assertEqual("192.168.8.42", command["client_ip"])
            self.assertEqual(claim["session"], command["session"])
            self.assertEqual(2836, command["transmission"]["target"])
            self.assertEqual(510, command["steering"]["current"])
        finally:
            state.running = False
            state.close()
            temporary.cleanup()

    def test_page_hides_auto_until_policy_and_labels_recovery(self):
        page = primary.PAGE_PATH.read_text(encoding="utf-8")
        self.assertIn(".modes [data-mode=auto]{display:none}", page)
        self.assertIn("TRACTOR01 — RECOVERY MANUAL ONLY", page)
        self.assertIn("Command gap", page)
        self.assertIn("Voice motion is disabled in recovery", page)

    def test_units_conflict_and_recovery_never_auto_starts(self):
        normal = (ROOT / "tractor_rpi/setup/tractor-wifi-control.service").read_text()
        recovery = (ROOT / "tractor_rpi/setup/tractor-wifi-recovery.service").read_text()
        direct = (ROOT / "tractor_rpi/setup/tractor-wifi-direct-mission.service").read_text()
        self.assertIn("Conflicts=tractor-wifi-recovery.service", normal)
        self.assertIn("Conflicts=tractor-wifi-control.service", recovery)
        self.assertIn("tractor-wifi-direct-mission.service", normal)
        self.assertIn("tractor-wifi-direct-mission.service", recovery)
        self.assertIn(
            "Conflicts=tractor-wifi-control.service tractor-wifi-recovery.service",
            direct,
        )
        self.assertIn("RuntimeDirectory=tractor-phone-control", normal)
        self.assertIn("RuntimeDirectoryMode=0700", normal)
        self.assertIn("--ownership-lock /run/tractor-phone-control/ownership.lock", normal)
        self.assertIn("RuntimeDirectory=tractor-phone-control", recovery)
        self.assertIn("RuntimeDirectoryMode=0700", recovery)
        self.assertIn("--ownership-lock /run/tractor-phone-control/ownership.lock", recovery)
        self.assertIn("--authority-policy onboard-router-mission", direct)
        self.assertIn("--host 192.168.8.20", direct)
        self.assertIn("--ownership-lock /run/tractor-phone-control/ownership.lock", direct)
        self.assertNotIn("WantedBy=", recovery)
        self.assertNotIn("WantedBy=", direct)

    @mock.patch.object(workflow, "show_status")
    @mock.patch.object(workflow, "service_active", return_value=True)
    @mock.patch.object(workflow, "require_root")
    def test_start_is_idempotent(self, _root, _active, show_status):
        workflow.start()
        show_status.assert_called_once_with()

    @mock.patch.object(workflow, "send_pause_and_capture", return_value=None)
    @mock.patch.object(workflow, "mission_processes", return_value=["123 controller"])
    @mock.patch.object(workflow, "interface_ready", return_value=True)
    @mock.patch.object(workflow, "service_active", return_value=False)
    @mock.patch.object(workflow, "require_root")
    def test_active_mission_must_confirm_pause_before_recovery(
        self, _root, _active, _interface, _processes, _pause
    ):
        with self.assertRaisesRegex(RuntimeError, "could not confirm software Pause"):
            workflow.start()


if __name__ == "__main__":
    unittest.main()
