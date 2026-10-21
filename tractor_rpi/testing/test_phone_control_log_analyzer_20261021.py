#!/usr/bin/env python3

import json
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
ANALYZER = ROOT / (
    "field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/"
    "20261020_two_mission_field_day/analyze_phone_control_log_20261020.py"
)


class PhoneControlLogAnalyzerTests(unittest.TestCase):
    def test_current_command_mode_field_counts_as_armed(self):
        with tempfile.TemporaryDirectory() as temporary:
            log_path = Path(temporary) / "wifi_primary_control_test.jsonl"
            start = datetime(2026, 10, 21, 16, 0, tzinfo=timezone.utc)
            records = []
            for index in range(5):
                records.append(
                    {
                        "time": (start + timedelta(milliseconds=200 * index)).isoformat(),
                        "event": "command",
                        "mode": "manual",
                        "client_reported_rtt_ms": 30.0,
                        "command_gap_ms": 200.0,
                    }
                )
            log_path.write_text(
                "".join(json.dumps(record) + "\n" for record in records),
                encoding="utf-8",
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(ANALYZER),
                    str(log_path),
                    "--require-minutes",
                    "0.01",
                ],
                cwd=ROOT,
                text=True,
                capture_output=True,
            )

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("Longest continuous armed duration: 0.0 min; command intervals: 4", result.stdout)
            self.assertIn("PASS: provisional phone timing gates met", result.stdout)


if __name__ == "__main__":
    unittest.main()
