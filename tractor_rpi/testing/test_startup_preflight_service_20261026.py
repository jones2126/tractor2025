#!/usr/bin/env python3
"""Static regression checks for the Tractor01 startup-preflight wiring."""

from pathlib import Path
import unittest


REPO = Path(__file__).resolve().parents[2]
WRAPPER = REPO / "tractor_rpi/testing/run_startup_mission_preflight_20261026.sh"
UNIT = REPO / "tractor_rpi/setup/tractor-startup-preflight.service"
INSTALLER = REPO / "tractor_rpi/setup/install_services.sh"
LOG_COMMAND = (
    "journalctl -u tractor-startup-preflight.service -b --no-pager -o cat"
)


class StartupPreflightServiceTests(unittest.TestCase):
    def test_network_gate_precedes_notifications_and_preflight(self) -> None:
        text = WRAPPER.read_text(encoding="utf-8")
        gate = text.index("wait_for_network || exit 75")
        kicked_off = text.index('echo "Automatic Tractor01 startup preflight kicked off."')
        preflight = text.index('if python3 -u "${PREFLIGHT}"')
        self.assertLess(gate, kicked_off)
        self.assertLess(gate, preflight)
        for expected in (
            "192.168.8.20/24",
            "192.168.8.1",
            "192.168.193.88",
            "192.168.193.76",
            "tractor-wifi-control.service",
            "https://ntfy.sh/",
        ):
            self.assertIn(expected, text)

    def test_failure_notice_contains_complete_log_command(self) -> None:
        self.assertIn(LOG_COMMAND, WRAPPER.read_text(encoding="utf-8"))

    def test_unit_retries_only_temporary_network_failure(self) -> None:
        text = UNIT.read_text(encoding="utf-8")
        self.assertIn("Restart=on-failure", text)
        self.assertIn("RestartPreventExitStatus=1", text)
        self.assertIn("TimeoutStartSec=480", text)

    def test_rebuild_installer_restores_and_enables_service(self) -> None:
        text = INSTALLER.read_text(encoding="utf-8")
        self.assertIn("setup/tractor-startup-preflight.service", text)
        self.assertIn("systemctl enable tractor-startup-preflight.service", text)
        self.assertIn("configure_dual_f9p_5hz_profile_20260923.py", text)


if __name__ == "__main__":
    unittest.main()
