#!/usr/bin/env python3

import unittest

from tractor_rpi.testing.rpi_full_arbitrator_fault_simulation_20261031 import run_matrix


class RpiFullArbitratorFaultSimulationTests(unittest.TestCase):
    def test_every_fault_case_passes(self):
        rows = run_matrix()
        self.assertGreaterEqual(len(rows), 10)
        self.assertTrue(all(row["pass"] for row in rows), rows)


if __name__ == "__main__":
    unittest.main()
