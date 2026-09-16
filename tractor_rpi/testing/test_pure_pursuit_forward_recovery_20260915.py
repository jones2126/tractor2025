#!/usr/bin/env python3
"""Synthetic safety tests for guarded Pure Pursuit recovery."""

import importlib.util
import math
import pathlib
import unittest


CONTROLLER = (
    pathlib.Path(__file__).resolve().parents[1]
    / "pure-pursuit"
    / "pure_pursuit_controller_20260915.py"
)
SPEC = importlib.util.spec_from_file_location("pure_pursuit_20260915", CONTROLLER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def controller(points, **kwargs):
    pursuit = MODULE.PurePursuit(**kwargs)
    pursuit.path = [(x, y, 0.0, 2.0, 0.5) for x, y in points]
    pursuit.goal_reached = False
    pursuit.idx = 0
    pursuit.progress_s = 0.0
    pursuit._build_arc_lengths()
    return pursuit


class ForwardRecoveryTests(unittest.TestCase):
    def make_controller(self, points, **kwargs):
        pursuit = controller(points, **kwargs)
        self.addCleanup(pursuit.sock.close)
        return pursuit

    def test_reacquires_nearest_point_in_front(self):
        pursuit = self.make_controller([(0, 0), (10, 0), (20, 0)])
        self.assertTrue(pursuit.reacquire_forward(5.0, 0.2, 0.0))
        self.assertAlmostEqual(pursuit.progress_s, 5.0, places=6)
        pursuit.compute_steering(5.0, 0.2, 0.0)
        self.assertGreater(pursuit._last_target_xy[0], 5.0)

    def test_never_rewinds_to_a_path_behind_progress(self):
        pursuit = self.make_controller([(0, 0), (10, 0), (20, 0)])
        pursuit.progress_s = 12.0
        pursuit.idx = 1
        self.assertFalse(pursuit.reacquire_forward(2.0, 0.0, 0.0))
        self.assertEqual(pursuit.progress_s, 12.0)

    def test_opposite_heading_is_rejected(self):
        pursuit = self.make_controller([(0, 0), (10, 0), (20, 0)])
        self.assertFalse(pursuit.reacquire_forward(5.0, 0.0, math.pi))
        self.assertEqual(pursuit.reacquire_state, "BLOCKED")

    def test_crossing_near_tie_is_rejected_as_ambiguous(self):
        pursuit = self.make_controller(
            [(0, 0), (10, 0), (10, 5), (0, 5), (0, 0), (10, 0)],
            ambiguity_distance_m=0.35,
            ambiguity_progress_m=8.0,
        )
        self.assertFalse(pursuit.reacquire_forward(5.0, 0.0, 0.0))
        self.assertIn("ambiguous", pursuit.reacquire_detail)

    def test_tracking_progress_is_monotonic(self):
        pursuit = self.make_controller([(0, 0), (10, 0), (20, 0)])
        pursuit.progress_s = 8.0
        pursuit.idx = 0
        pursuit.reacquire_state = "TRACKING"
        pursuit.compute_steering(7.0, 0.0, 0.0)
        self.assertGreaterEqual(pursuit.progress_s, 8.0)


if __name__ == "__main__":
    unittest.main()
