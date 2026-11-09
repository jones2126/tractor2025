#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest


REPO = Path(__file__).resolve().parents[2]
DASHBOARD = (
    REPO
    / "field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans"
    / "20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED"
    / "mission_dashboard_production_20261109.py"
)
spec = importlib.util.spec_from_file_location("production_dashboard", DASHBOARD)
production = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(production)


class FakeState:
    def __init__(self, arbitrator):
        self.arbitrator = arbitrator

    def snapshot(self):
        return {
            "bridge_age_s": 0.1,
            "bridge": {
                "steering": {"mode": 2, "state": "PAUSE"},
                "transmission": {"mode": 2},
                "rpi_arbitrator": dict(self.arbitrator),
            },
            "gps_age_s": 0.1,
            "gps": {
                "fix_quality": "RTK Fixed",
                "headValid": True,
                "carrier": "fixed",
                "relpos_length_m": 1.09,
                "relpos_heading_accuracy_deg": 0.53,
            },
        }


def good_arbitrator():
    return {
        "requested_mode": "PAUSE",
        "actual_serial_output": "PAUSE",
        "phone_link_fresh": True,
        "scheduler_fault_latched": False,
        "additional_local_faults": [],
        "communications_degraded_preauthorized": True,
        "distance_envelope_source": "ROUTE_PROGRESS_ONLY",
    }


class ProductionDashboardTests(unittest.TestCase):
    def test_dashboard_uses_authoritative_phone_safety_feed(self):
        self.assertIn("arb.phone_link_fresh===true", production.dashboard.HTML)
        self.assertIn("productionSafetyReady", production.dashboard.HTML)
        self.assertIn("arbPresent?browserStableMs", production.dashboard.HTML)
        self.assertIn("d.current_exact_reason", production.dashboard.MESSAGES_HTML)

    def test_exact_safe_pause_state_can_start_controller(self):
        self.assertEqual(
            (True, ""),
            production.production_safe_to_start(FakeState(good_arbitrator())),
        )

    def test_phone_request_and_actual_output_must_both_be_pause(self):
        for field, value in (
            ("requested_mode", "AUTO_MPS"),
            ("actual_serial_output", "MANUAL_PERCENT"),
            ("phone_link_fresh", False),
            ("scheduler_fault_latched", True),
            ("additional_local_faults", ["steering_control_healthy"]),
            ("communications_degraded_preauthorized", False),
            ("distance_envelope_source", "UNMEASURED"),
        ):
            arbitrator = good_arbitrator()
            arbitrator[field] = value
            with self.subTest(field=field):
                allowed, reason = production.production_safe_to_start(
                    FakeState(arbitrator)
                )
                self.assertFalse(allowed)
                self.assertTrue(reason)


if __name__ == "__main__":
    unittest.main()
