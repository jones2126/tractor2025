#!/usr/bin/env python3

import unittest
from unittest.mock import Mock

from tractor_rpi.testing.webrtc import wifi_rpi_arbitrator_control_20261104 as module


class ArbitratorOnlyPhoneTests(unittest.TestCase):
    def test_command_uses_only_arbitrator_feed(self):
        state = object.__new__(module.ArbitratorOnlyWifiState)
        state.emit_arbitrator_intent = Mock()
        state.send_drive_command(10.0, -20.0, 7, "test", phone_mode="manual")
        state.emit_arbitrator_intent.assert_called_once_with(
            10.0, -20.0, 7, "test", phone_mode="manual"
        )

    def test_expected_firmware_is_consolidated_v4(self):
        self.assertEqual(
            "teensy_main_20261110_rpi_command_lease_v4",
            module.EXPECTED_FIRMWARE,
        )


if __name__ == "__main__":
    unittest.main()
