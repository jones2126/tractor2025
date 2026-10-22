#!/usr/bin/env python3

import unittest
from datetime import datetime, timedelta, timezone

import analyze_phone_timeout_telemetry as analysis


class AnalysisHelpersTests(unittest.TestCase):
    def test_nearest_respects_maximum_delta(self):
        origin = datetime(2026, 10, 21, tzinfo=timezone.utc)
        rows = [{"_time": origin}, {"_time": origin + timedelta(seconds=10)}]
        self.assertIsNone(analysis.nearest(rows, origin + timedelta(seconds=5), 1.0))
        self.assertEqual(rows[0], analysis.nearest(rows, origin + timedelta(milliseconds=20), 1.0))

    def test_distribution_uses_interpolated_percentile(self):
        result = analysis.distribution([1.0, 2.0, 3.0, 4.0])
        self.assertEqual(4, result["n"])
        self.assertEqual(2.5, result["p50"])
        self.assertEqual(4.0, result["max"])

    def test_delayed_response_classification_requires_matching_rtt(self):
        expiry = datetime.fromisoformat("2026-10-21T18:18:14+00:00")
        browser = [{"event": "ack_watchdog_expired"}, {"event": "send_ok", "rtt_ms": 900}]
        delayed = analysis.classify(expiry, browser, {"driving": "True"}, 710.0)
        failed = analysis.classify(expiry, browser, {"driving": "True"}, 40.0)
        self.assertIn("delayed response/ack", delayed)
        self.assertIn("request/ack failure", failed)


if __name__ == "__main__":
    unittest.main()
