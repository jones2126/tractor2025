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

    def test_31_passes(self):
        check = preflight.heading_satellites_used_check(self.samples(31, 31, 31))
        self.assertTrue(check.passed)
        self.assertFalse(getattr(check, "warning", False))

    def test_30_warns_without_blocking(self):
        check = preflight.heading_satellites_used_check(self.samples(30, 30, 30))
        self.assertTrue(check.passed)
        self.assertTrue(check.warning)

    def test_29_warns_without_blocking(self):
        check = preflight.heading_satellites_used_check(self.samples(29, 29, 29))
        self.assertTrue(check.passed)
        self.assertTrue(check.warning)

    def test_28_fails(self):
        check = preflight.heading_satellites_used_check(self.samples(28, 28, 28))
        self.assertFalse(check.passed)

    def test_missing_value_fails_closed(self):
        check = preflight.heading_satellites_used_check([(0.0, {}), (1.0, {})])
        self.assertFalse(check.passed)


if __name__ == "__main__":
    unittest.main()
