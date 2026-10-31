#!/usr/bin/env python3
"""Static containment checks for the opt-in RPi command-lease firmware."""

import re
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE_PATH = REPO_ROOT / "tractor_teensy/src/teensy_main_20261031_rpi_command_lease.cpp"
BUILD_PATH = REPO_ROOT / "tractor_teensy/platformio.rpi-command-lease.ini"


class TeensyRpiCommandLeaseSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE_PATH.read_text(encoding="utf-8")
        cls.build = BUILD_PATH.read_text(encoding="utf-8")

    def test_new_firmware_reuses_frozen_base(self):
        self.assertIn('#include "teensy_main_20260914.cpp"', self.source)
        self.assertIn("loop_20260914_frozen_reference", self.source)

    def test_active_loop_uses_only_generic_rpi_parser(self):
        active_loop = self.source.split('extern "C" void loop()', 1)[1]
        self.assertIn("parseRpiCommandLeaseSerial();", active_loop)
        self.assertNotIn("parseSerialCommand();", active_loop)
        self.assertNotIn("wifiPhone", active_loop)

    def test_source_contains_no_phone_or_network_freshness_input(self):
        self.assertNotRegex(self.source, re.compile(r"phoneFresh|wifiPhoneTimestamp"))
        self.assertNotIn("ZeroTier", self.source)
        self.assertNotIn("SSID", self.source)

    def test_fixed_lease_cannot_exceed_existing_500_ms_watchdog(self):
        self.assertIn(
            "static const unsigned long RPI_COMMAND_LEASE_MS = CMD_VEL_TIMEOUT;",
            self.source,
        )
        self.assertIn(
            'static_assert(RPI_COMMAND_LEASE_MS <= 500UL',
            self.source,
        )
        self.assertNotRegex(self.source, re.compile(r"RPI_COMMAND_LEASE_MS\s*=\s*30000"))

    def test_invalid_or_replayed_frames_do_not_update_timestamp(self):
        timestamp_assignments = re.findall(
            r"rpiCommandTimestamp\s*=\s*millis\(\)", self.source
        )
        self.assertEqual(1, len(timestamp_assignments))
        self.assertIn("!rpiSequenceIsNewer", self.source)
        self.assertIn("rpiCrc16Ccitt(line, bodyLength) != suppliedCrc", self.source)

    def test_legacy_motion_frames_are_not_authority_inputs(self):
        self.assertIn("Legacy CMD and WIFI motion frames are intentionally not", self.source)
        active_parser = self.source.split("static void parseRpiCommandLeaseSerial()", 1)[1]
        active_parser = active_parser.split("static void publishRpiCommandLeaseStatus", 1)[0]
        self.assertNotIn('memcmp(buffer, "CMD,"', active_parser)
        self.assertNotIn('memcmp(buffer, "WIFI,"', active_parser)

    def test_physical_pause_and_local_fault_logic_remain_present(self):
        self.assertIn("physicalPause || !leaseUsable", self.source)
        self.assertIn("rpiLeaseExpiredLatched = true", self.source)
        self.assertIn("rpiLeaseExpiredLatched = false", self.source)
        self.assertIn("steeringFaultLatched", self.source)
        self.assertIn("estopCheck();", self.source)
        self.assertIn("controlTransmission();", self.source)
        self.assertIn("controlSteering();", self.source)

    def test_new_rpi_session_requires_physical_pause_claim(self):
        self.assertIn("physicalPauseForSessionClaim", self.source)
        self.assertIn("parsedMode != RPI_MODE_PAUSE", self.source)
        self.assertIn("RPI_REJECT_SESSION", self.source)

    def test_telemetry_exposes_session_and_expiry_latch(self):
        self.assertIn("RPI_CTL,m=%u,f=%d,u=%d,l=%d", self.source)
        self.assertIn("ss=%lu,s=%lu", self.source)

    def test_separate_build_file_selects_only_new_candidate(self):
        self.assertIn(
            "build_src_filter = +<teensy_main_20261031_rpi_command_lease.cpp>",
            self.build,
        )
        self.assertNotIn("teensy_main_20261003_wifi.cpp", self.build)


if __name__ == "__main__":
    unittest.main()
