#!/usr/bin/env python3
import unittest

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

    def test_missing_value_fails_closed(self):
        check = preflight.heading_satellites_used_check([(0.0, {}), (1.0, {})])
        self.assertFalse(check.passed)


if __name__ == "__main__":
    unittest.main()
