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
            reacquire_max_advance_m=100.0,
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

    def test_recovery_cannot_jump_beyond_forward_window(self):
        pursuit = self.make_controller(
            [(0, 0), (10, 0), (20, 0), (30, 0), (40, 0), (50, 0), (60, 0)],
            reacquire_max_advance_m=30.0,
        )
        pursuit.progress_s = 1.0
        pursuit.idx = 0
        self.assertFalse(pursuit.reacquire_forward(50.0, 0.0, 0.0))
        self.assertEqual(pursuit.progress_s, 1.0)

    def test_recovery_accepts_point_inside_forward_window(self):
        pursuit = self.make_controller(
            [(0, 0), (10, 0), (20, 0), (30, 0), (40, 0)],
            reacquire_max_advance_m=30.0,
        )
        pursuit.progress_s = 1.0
        pursuit.idx = 0
        self.assertTrue(pursuit.reacquire_forward(25.0, 0.0, 0.0))
        self.assertAlmostEqual(pursuit.progress_s, 25.0)

    def test_recovery_cannot_cross_phase_boundary(self):
        pursuit = self.make_controller(
            [(0, 0), (10, 0), (20, 0), (0, 0), (10, 0)],
            reacquire_max_advance_m=100.0,
        )
        pursuit.path_phases = ["phase_a", "phase_a", "phase_b", "phase_b", "phase_b"]
        pursuit.progress_s = 0.0
        pursuit.idx = 0
        self.assertFalse(pursuit.reacquire_forward(15.0, 0.0, 0.0))
        self.assertIn("phase 'phase_a'", pursuit.reacquire_detail)


class HeadingHealthGateTests(unittest.TestCase):
    def receiver(self):
        receiver = MODULE.GPSReceiver.__new__(MODULE.GPSReceiver)
        receiver.min_fix = "RTK Fixed"
        receiver.require_head_valid = True
        receiver.require_carrier_fixed = True
        receiver.baseline_min_m = 0.80
        receiver.baseline_max_m = 1.30
        receiver.heading_accuracy_max_deg = 1.0
        return receiver

    @staticmethod
    def healthy_pose():
        return {
            "age": 0.1,
            "fatal_error": False,
            "fatal_base_reason": None,
            "fatal_heading_reason": None,
            "fix_quality": "RTK Fixed",
            "headValid": True,
            "carrier": "fixed",
            "relpos_length_m": 1.10,
            "heading_accuracy_deg": 0.50,
        }

    def test_healthy_fixed_heading_is_drivable(self):
        self.assertEqual(self.receiver().is_drivable(self.healthy_pose()), (True, ""))

    def test_float_false_baseline_heading_is_rejected(self):
        pose = self.healthy_pose()
        pose.update(carrier="float", relpos_length_m=3.8887, heading_accuracy_deg=18.44465)
        ok, reason = self.receiver().is_drivable(pose)
        self.assertFalse(ok)
        self.assertIn("carrier='float'", reason)


if __name__ == "__main__":
    unittest.main()
