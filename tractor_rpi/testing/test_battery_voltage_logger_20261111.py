import csv
import datetime as dt
from pathlib import Path
import tempfile
import unittest

from tractor_rpi.battery_voltage_logger_20261111 import (
    append_row,
    apply_retention,
    extract_row,
)


class BatteryVoltageLoggerTests(unittest.TestCase):
    def test_extracts_valid_voltage_and_context(self):
        row = extract_row(
            {
                "system": {"firmware": "firmware-v5"},
                "steering": {"mode": 2},
                "transmission": {
                    "mode": 2,
                    "vin_voltage_mV": 12569,
                    "vin_voltage_valid": 1,
                    "jrk_sequence": 25,
                    "jrk_timeouts": 0,
                    "errors_halting": 0,
                },
            },
            now=dt.datetime(2026, 11, 11, 14, 24, tzinfo=dt.timezone(dt.timedelta(hours=-5))),
            telemetry_age_s=0.04,
        )
        self.assertEqual(row["vin_voltage_mV"], 12569)
        self.assertEqual(row["vin_voltage_valid"], 1)
        self.assertEqual(row["firmware"], "firmware-v5")
        self.assertEqual(row["telemetry_age_s"], "0.040")
        self.assertEqual(row["voltage_context_hint"], "no_charging_source_detected")

    def test_invalid_voltage_is_explicit(self):
        row = extract_row(
            {"transmission": {"vin_voltage_mV": None, "vin_voltage_valid": 1}},
            now=dt.datetime.now(dt.timezone.utc),
            telemetry_age_s=0.0,
        )
        self.assertEqual(row["vin_voltage_mV"], "")
        self.assertEqual(row["vin_voltage_valid"], 0)

    def test_stale_telemetry_cannot_remain_valid(self):
        row = extract_row(
            {"transmission": {"vin_voltage_mV": 13000, "vin_voltage_valid": 1}},
            now=dt.datetime.now(dt.timezone.utc),
            telemetry_age_s=2.1,
        )
        self.assertEqual(row["vin_voltage_mV"], 13000)
        self.assertEqual(row["vin_voltage_valid"], 0)
        self.assertEqual(row["voltage_context_hint"], "invalid")

    def test_daily_csv_header_is_written_once(self):
        now = dt.datetime(2026, 11, 11, 19, 0, tzinfo=dt.timezone.utc)
        row = extract_row(
            {"transmission": {"vin_voltage_mV": 13000, "vin_voltage_valid": 1}},
            now=now,
            telemetry_age_s=0.1,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = append_row(Path(directory), row)
            append_row(Path(directory), row)
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["vin_voltage_mV"], "13000")

    def test_retention_only_removes_expired_logger_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            expired = root / "tractor_battery_2026-08-01.csv"
            current = root / "tractor_battery_2026-11-11.csv"
            unrelated = root / "other_2026-08-01.csv"
            for path in (expired, current, unrelated):
                path.write_text("test", encoding="utf-8")
            removed = apply_retention(
                root, today=dt.date(2026, 11, 11), retention_days=90
            )
            self.assertEqual(removed, [expired])
            self.assertFalse(expired.exists())
            self.assertTrue(current.exists())
            self.assertTrue(unrelated.exists())


if __name__ == "__main__":
    unittest.main()
