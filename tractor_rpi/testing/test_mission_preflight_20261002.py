#!/usr/bin/env python3
import unittest
from unittest import mock

from tractor_rpi.testing import mission_preflight_20261002 as preflight


class HeadingSatelliteGateTests(unittest.TestCase):
    @staticmethod
    def samples(*values):
        return [
            (float(index), {"heading_numSV_used": value})
            for index, value in enumerate(values)
        ]

    def test_26_passes(self):
        check = preflight.heading_satellites_used_check(self.samples(26, 26, 26))
        self.assertTrue(check.passed)
        self.assertFalse(getattr(check, "warning", False))

    def test_25_warns_without_blocking(self):
        check = preflight.heading_satellites_used_check(self.samples(25, 25, 25))
        self.assertTrue(check.passed)
        self.assertTrue(check.warning)

    def test_24_warns_without_blocking(self):
        check = preflight.heading_satellites_used_check(self.samples(24, 24, 24))
        self.assertTrue(check.passed)
        self.assertTrue(check.warning)

    def test_23_fails(self):
        check = preflight.heading_satellites_used_check(self.samples(23, 23, 23))
        self.assertFalse(check.passed)


class StartupAndNetworkGateTests(unittest.TestCase):
    @mock.patch.object(preflight.base, "command_output")
    def test_heading_startup_current_invocation_passes(self, command_output):
        command_output.return_value = (
            0,
            "configure_dual_f9p_5hz_profile_20260923.py --heading-startup "
            "code=exited status=0",
        )
        self.assertTrue(preflight.heading_startup_configuration_check().passed)

    @mock.patch.object(preflight.base, "command_output")
    def test_heading_startup_failed_invocation_blocks(self, command_output):
        command_output.side_effect = [
            (
                0,
                "configure_dual_f9p_5hz_profile_20260923.py --heading-startup "
                "code=exited status=1",
            ),
            (0, "Heading startup began but no successful readback was recorded"),
        ]
        self.assertFalse(preflight.heading_startup_configuration_check().passed)

    @mock.patch.object(preflight.base, "command_output")
    def test_heading_startup_uses_current_boot_journal_after_daemon_reload(
        self, command_output
    ):
        command_output.side_effect = [
            (
                0,
                "configure_dual_f9p_5hz_profile_20260923.py --heading-startup "
                "start_time=[n/a] code=(null) status=0/0",
            ),
            (
                0,
                "HEADING STARTUP RECOVERY\n"
                "Heading startup: PASS independent readback of 22 targeted values\n"
                "PASS: heading startup profile is verified in RAM",
            ),
        ]
        self.assertTrue(preflight.heading_startup_configuration_check().passed)

    @mock.patch.object(preflight.base, "command_output")
    def test_expected_network_path_passes(self, command_output):
        command_output.side_effect = [
            (0, "2: eth0 inet 192.168.8.20/24 scope global eth0"),
            (0, "1.1.1.1 via 192.168.8.1 dev eth0 src 192.168.8.20"),
            (0, "Not connected."),
            (0, "3 packets transmitted, 3 received, 0% packet loss"),
            (0, "192.168.193.88 dev ztuze7ml6g src 192.168.193.76"),
        ]
        self.assertTrue(all(check.passed for check in preflight.network_path_checks()))

    @mock.patch.object(preflight.base, "command_output")
    def test_direct_tractorfield_wifi_blocks(self, command_output):
        command_output.side_effect = [
            (0, "2: eth0 inet 192.168.8.20/24 scope global eth0"),
            (0, "1.1.1.1 via 192.168.8.1 dev eth0 src 192.168.8.20"),
            (0, "Connected to aa:bb:cc:dd:ee:ff\n\tSSID: TractorField"),
            (0, "3 packets transmitted, 3 received, 0% packet loss"),
            (0, "192.168.193.88 dev ztuze7ml6g src 192.168.193.76"),
        ]
        checks = preflight.network_path_checks()
        wifi = next(check for check in checks if check.name == "Pi direct TractorField Wi-Fi")
        self.assertFalse(wifi.passed)

    def test_missing_value_fails_closed(self):
        check = preflight.heading_satellites_used_check([(0.0, {}), (1.0, {})])
        self.assertFalse(check.passed)


if __name__ == "__main__":
    unittest.main()
