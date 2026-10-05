#!/usr/bin/env python3

import importlib.util
import subprocess
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "tractor_rpi" / "pure-pursuit" / "mission_dashboard_20260910.py"
SPEC = importlib.util.spec_from_file_location("mission_dashboard_local_test", MODULE_PATH)
dashboard = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(dashboard)


class MissionDashboardLocalAddressTests(unittest.TestCase):
    def test_dashboard_has_an_explicit_guarded_auto_gate(self):
        self.assertIn('id="autoGate"', dashboard.HTML)
        self.assertIn("READY FOR GUARDED AUTO", dashboard.HTML)
        self.assertIn("waitReason.includes('mode=PAUSE')", dashboard.HTML)
        self.assertIn("phoneAuto=wifiPresent?wifiMode===2", dashboard.HTML)
        self.assertIn("CONTROL_STABLE_MS=5000", dashboard.HTML)
        self.assertIn("NETWORK UNSTABLE — REMAIN IN PAUSE", dashboard.HTML)

    @mock.patch.object(subprocess, "run")
    def test_route_to_laptop_is_first_and_zerotier_is_excluded(self, run):
        run.side_effect = [
            SimpleNamespace(
                stdout=(
                    "192.168.10.48 dev wlan0 src 192.168.10.76 uid 1000\n"
                )
            ),
            SimpleNamespace(
                stdout=(
                    "2: eth0    inet 192.168.1.151/24 brd 192.168.1.255 scope global eth0\n"
                    "3: wlan0   inet 192.168.10.76/24 brd 192.168.10.255 scope global wlan0\n"
                    "4: ztabc   inet 192.168.193.76/24 brd 192.168.193.255 scope global ztabc\n"
                    "5: docker0 inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0\n"
                )
            ),
        ]

        self.assertEqual(
            ["192.168.10.76", "192.168.1.151"],
            dashboard.discover_lan_addresses("192.168.10.48"),
        )
        run.assert_any_call(
            ["ip", "-4", "route", "get", "192.168.10.48"],
            capture_output=True,
            text=True,
            timeout=2,
            check=False,
        )

    @mock.patch.object(subprocess, "run", side_effect=OSError("ip unavailable"))
    def test_missing_ip_command_has_safe_empty_fallback(self, _run):
        self.assertEqual([], dashboard.discover_lan_addresses("192.168.10.48"))


if __name__ == "__main__":
    unittest.main()
