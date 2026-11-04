#!/usr/bin/env python3

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "tractor_teensy/src/teensy_main_20261031_rpi_command_lease.cpp"
WRAPPER = ROOT / "tractor_teensy/src/teensy_main_20261104_rpi_command_lease.cpp"
CONFIG = ROOT / "tractor_teensy/platformio.rpi-command-lease-v3.ini"


class TeensyV3SourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = BASE.read_text(encoding="utf-8")
        cls.wrapper = WRAPPER.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")

    def test_wrapper_is_separate_and_not_default(self):
        self.assertIn("TRACTOR_RPI_COMMAND_LEASE_V3", self.wrapper)
        self.assertIn("teensy_main_20261104_rpi_command_lease_v3", self.wrapper)
        self.assertIn("teensy_main_20261104_rpi_command_lease.cpp", self.config)
        normal = (ROOT / "tractor_teensy/platformio.wifi-primary.ini").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("20261104_rpi_command_lease", normal)

    def test_phone_estop_is_zero_only_latched_and_reported(self):
        for required in (
            "RPI_MODE_ESTOP = 3",
            "RPI_MODE_RESET_PAUSE = 4",
            "rpiSoftwareEstopLatched = true",
            "rpiSoftwareEstopLatched = false",
            "radioData.estop || rpiSoftwareEstopLatched",
            "rr=%u,e=%d",
        ):
            self.assertIn(required, self.base)

    def test_local_lease_stays_at_or_below_500_ms(self):
        self.assertIn(
            "static_assert(RPI_COMMAND_LEASE_MS <= 500UL", self.base
        )
        self.assertIn("if (rpiCommandSelected && !arrivalFresh)", self.base)


if __name__ == "__main__":
    unittest.main()
