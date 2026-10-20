#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tractor_rpi" / "pure-pursuit" / "mission_dashboard_20260910.py"
SPEC = importlib.util.spec_from_file_location("mission_dashboard_stale_test", SOURCE)
DASHBOARD = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(DASHBOARD)


class MissionDashboardStaleStateTests(unittest.TestCase):
    def test_connection_loss_replaces_any_green_auto_display(self):
        self.assertIn("function showTelemetryLost(reason)", DASHBOARD.HTML)
        self.assertIn("TELEMETRY LOST — STATE UNKNOWN", DASHBOARD.HTML)
        self.assertIn("TELEMETRY LOST — DO NOT TRUST DISPLAY", DASHBOARD.HTML)
        self.assertIn("showTelemetryLost('Dashboard connection failed: '", DASHBOARD.HTML)

    def test_stale_controller_telemetry_disables_auto(self):
        self.assertIn(
            "CONTROLLER TELEMETRY STALE — DO NOT SELECT AUTO", DASHBOARD.HTML
        )

    def test_dashboard_exposes_required_diagnostic_fields(self):
        for label in (
            "Dashboard access path",
            "Authoritative phone mode",
            "Recovery lockout",
            "Last fresh controller telemetry",
        ):
            self.assertIn(label, DASHBOARD.HTML)


if __name__ == "__main__":
    unittest.main()
