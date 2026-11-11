#!/usr/bin/env python3

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "tractor_teensy/src/teensy_main_20261110.cpp"
CONFIG = ROOT / "tractor_teensy/platformio.ini"


class TeensyV3SourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.firmware = FIRMWARE.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")

    def test_consolidated_source_is_the_only_selected_firmware(self):
        self.assertNotIn("#ifdef", self.firmware)
        self.assertNotIn("#ifndef", self.firmware)
        self.assertNotIn("#else", self.firmware)
        self.assertNotIn("#if ", self.firmware)
        self.assertNotIn("#endif", self.firmware)
        self.assertIn("teensy_main_20261110_rpi_command_lease_v5", self.firmware)
        self.assertIn("+<teensy_main_20261110.cpp>", self.config)
        self.assertNotIn("RF24", self.config)
        self.assertNotIn("#include <SPI.h>", self.firmware)
        self.assertNotIn("#include <RF24.h>", self.firmware)
        self.assertEqual(1, self.firmware.count('extern "C" void setup()'))
        self.assertEqual(1, self.firmware.count('extern "C" void loop()'))

    def test_phone_estop_is_zero_only_latched_and_reported(self):
        for required in (
            "RPI_MODE_ESTOP = 3",
            "RPI_MODE_RESET_PAUSE = 4",
            "rpiSoftwareEstopLatched = true",
            "rpiSoftwareEstopLatched = false",
            "rpiSoftwareEstopLatched ? LOW : HIGH",
            "rr=%u,e=%d",
        ):
            self.assertIn(required, self.firmware)

    def test_local_lease_stays_at_or_below_500_ms(self):
        self.assertIn(
            "static_assert(RPI_COMMAND_LEASE_MS <= 500UL", self.firmware
        )
        self.assertIn("if (rpiCommandSelected && !arrivalFresh)", self.firmware)

    def test_boot_and_missing_authority_are_fail_closed(self):
        self.assertIn("byte control_mode = CONTROL_PAUSE", self.firmware)
        self.assertIn("bool commandAuthorityReady = false", self.firmware)
        self.assertIn("setJrkTarget(transmissionNeutralPos);", self.firmware)
        self.assertIn("if (!commandAuthorityReady)", self.firmware)
        self.assertNotIn("handleRadio()", self.firmware)
        self.assertNotIn("radio.begin()", self.firmware)

    def test_permanent_calibration_and_rpi_owned_gps(self):
        self.assertIn("const int STEER_POT_RIGHT  = 197;", self.firmware)
        self.assertIn("const int STEER_POT_CENTER = 447;", self.firmware)
        self.assertIn("const int STEER_POT_LEFT   = 815;", self.firmware)
        self.assertNotIn("GpsStatus", self.firmware)
        self.assertNotIn('memcmp(buffer, "GPS,"', self.firmware)

    def test_jrk_vin_is_read_with_current_and_reported(self):
        self.assertIn("0x17, sizeof(powerBytes), powerBytes", self.firmware)
        self.assertIn("jrkVinVoltageMv = readU16LE(powerBytes);", self.firmware)
        self.assertIn("jvin=%u,jvv=%d", self.firmware)


if __name__ == "__main__":
    unittest.main()
