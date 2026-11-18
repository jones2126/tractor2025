"""Synthetic numerical checks for offline IMU analysis; no field hardware."""
import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np

TOOLS=Path(__file__).resolve().parents[2]/"field_testing/tools"
if str(TOOLS) not in sys.path:sys.path.insert(0,str(TOOLS))
spec=importlib.util.spec_from_file_location("imu_manual_analysis",TOOLS/"analyze_tractor01_imu_manual_20261117.py")
analysis=importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def fixture():
    t=np.arange(0,50,.01)
    # Ten CCW degrees/second; compass crosses north repeatedly.
    d={"mono":t,"yaw":10*t,"gz":np.full(len(t),10.),"delta":np.full(len(t),.01),
       "segment":np.zeros(len(t)),"phase":np.ones(len(t),dtype=bool),
       "good":np.ones(len(t),dtype=bool)}
    nt=np.arange(.2,49.8,.2)
    n={"mono":nt,"epoch":analysis.local_epoch("11:28:00")+nt,"yaw":10*nt,
       "heading":(350-10*nt)%360,"good":np.ones(len(nt),dtype=bool),
       "speed":np.ones(len(nt)),"rate":np.full(len(nt),10.),"along_speed":np.ones(len(nt))}
    return d,n


class AnalysisTests(unittest.TestCase):
    def test_circle_recovers_known_radius_and_center(self):
        theta=np.linspace(0,4*np.pi,120)
        result=analysis.circle_fit(7+2*np.cos(theta),-3+2*np.sin(theta))
        self.assertAlmostEqual(result["radius_m"],2,places=10)
        self.assertAlmostEqual(result["center_east_m"],7,places=10)
        self.assertAlmostEqual(result["center_north_m"],-3,places=10)
        self.assertLess(result["radial_rms_m"],1e-10)

    def test_straight_points_cannot_establish_a_circle(self):
        with self.assertRaises(ValueError):analysis.circle_fit(np.arange(10),np.zeros(10))

    def test_sign_and_north_crossing(self):
        d,n=fixture()
        result=analysis.compare_interval(d,n,0,20)
        self.assertAlmostEqual(result["compass_prediction_error_deg"],0,places=10)
        self.assertAlmostEqual(result["imu_minus_f9p_turn_deg"],0,places=10)

    def test_sampling_gap_and_reset_are_rejected(self):
        d,n=fixture();d["delta"][100]=.08
        self.assertIsNone(analysis.compare_interval(d,n,0,20))
        d,n=fixture();d["segment"][100:]=1
        self.assertIsNone(analysis.compare_interval(d,n,0,20))

    def test_unhealthy_interior_not_just_endpoints(self):
        d,n=fixture();d["good"][100]=False
        result=analysis.compare_interval(d,n,0,20)
        self.assertFalse(result["reference_all_healthy"])

    def test_nonoverlap_windows_and_zero_error_on_perfect_signal(self):
        d,n=fixture();windows,summary=analysis.prediction_windows(d,n)
        for horizon in (1,3,5,10):
            rows=[r for r in windows if r["scheme"]=="nonoverlap" and r["horizon_s"]==horizon]
            self.assertGreater(len(rows),0)
            for left,right in zip(rows,rows[1:]):self.assertLessEqual(left["end_epoch"],right["start_epoch"])
            self.assertLess(max(abs(r["compass_prediction_error_deg"]) for r in rows),1e-9)
        self.assertTrue(summary)

    def test_reintegration_checks_logger_values(self):
        d,n=fixture()
        self.assertLess(analysis.integration_check(d)["max_recorded_vs_reintegrated_yaw_difference_deg"],1e-8)

    def test_wrap_and_run_boundaries(self):
        self.assertEqual(float(analysis.wrap(359)), -1)
        self.assertEqual(analysis.runs(np.array([False,True,True,False,True])),[(1,2),(4,4)])


if __name__=="__main__":unittest.main()
