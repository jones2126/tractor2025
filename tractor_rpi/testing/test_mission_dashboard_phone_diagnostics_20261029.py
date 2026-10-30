#!/usr/bin/env python3

import importlib.util
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tractor_rpi" / "pure-pursuit" / "mission_dashboard_20260910.py"
SPEC = importlib.util.spec_from_file_location("mission_dashboard_diagnostics", SOURCE)
dashboard = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(dashboard)


class MissionDashboardPhoneDiagnosticsTests(unittest.TestCase):
    def test_snapshot_reads_atomic_phone_diagnostics(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "diagnostics.json"
            path.write_text(
                json.dumps(
                    {
                        "updated_epoch_ms": round(time.time() * 1000),
                        "session_invalidation_count": 7,
                    }
                ),
                encoding="utf-8",
            )
            with mock.patch.object(dashboard, "PHONE_DIAGNOSTICS_PATH", path):
                snapshot = dashboard.MissionState().snapshot()
            self.assertEqual(7, snapshot["phone_diagnostics"]["session_invalidation_count"])
            self.assertLess(snapshot["phone_diagnostics_age_s"], 1.0)

    def test_messages_view_names_required_diagnostic_sources(self):
        for marker in (
            "Exact stop / wait reason",
            "Last rejected command",
            "Accepted-command gap",
            "Session claims / invalidations",
            "Phone → server scheduling",
            "Server → bridge age",
            "Pi → Teensy modes",
            "Pure Pursuit command freshness",
            "ZeroTier transport",
            "ntfy queue / acknowledgement",
            "COMMUNICATIONS DEGRADED",
        ):
            self.assertIn(marker, dashboard.MESSAGES_HTML)


if __name__ == "__main__":
    unittest.main()
