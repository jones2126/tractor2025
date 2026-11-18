"""Hardware-free replay integrity, timing, range and health tests."""
import csv
import datetime as dt
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parents[2] / "field_testing/tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))
spec = importlib.util.spec_from_file_location("imu_replay", TOOLS / "build_tractor01_imu_replay_20261117.py")
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def row(index, seconds, **changes):
    base = dt.datetime(2026, 11, 17, 16, 0, tzinfo=dt.timezone.utc)
    result = dict(sample=str(index), utc=(base+dt.timedelta(seconds=seconds)).isoformat(),
                  monotonic_s=str(100+seconds), phase="observation", dt_s="0.01", yaw_segment="0",
                  relative_yaw_ccw_deg="1", imu_compass_once_anchored_deg="359", imu_minus_gnss_deg="0.1",
                  gnss_lat="40", gnss_lon="-80", gnss_packet_age_s="0.01", gnss_heading_age_s="0.1",
                  gnss_socket_available="1", gnss_fix_quality="RTK Fixed", gnss_headValid="True",
                  gnss_carrier="fixed", gnss_relpos_valid="True", gnss_relpos_gnss_fix_ok="True",
                  gnss_relpos_ref_pos_miss="False", gnss_relpos_ref_obs_miss="False",
                  gnss_fatal_error="False", gnss_relpos_heading_accuracy_deg="0.5", gnss_relpos_length_m="1.05",
                  gnss_heading_usable="1", gnss_heading_deg="359.1", gnss_speed_mps="0.1",
                  gyro_x_corrected_dps="0", gyro_y_corrected_dps="0", gyro_z_corrected_dps="0.1",
                  accel_x_g="0.01", accel_y_g="-0.1", accel_z_g="1")
    result.update(changes)
    return result


class ReplayTests(unittest.TestCase):
    def capture(self, root, rows):
        fields = sorted(set().union(*(r.keys() for r in rows)))
        with (root/"samples.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fields)
            writer.writeheader()
            writer.writerows(rows)
        (root/"metadata.json").write_text(json.dumps({"sample_rows": len(rows)}))
        (root/"gnss.jsonl").write_text("{}\n")
        (root/"SHA256SUMS").write_text("\n".join(replay.digest(root/name)+"  "+name for name in ("samples.csv", "gnss.jsonl", "metadata.json")))

    def test_checksum_tampering_stops_build(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.capture(root, [row(0, 0), row(1, .01)])
            (root/"samples.csv").write_text("tampered")
            with self.assertRaisesRegex(ValueError, "Checksum mismatch"):
                replay.build_data(root, {})

    def test_bin_preserves_peak_and_last_value_and_true_row_count(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.capture(root, [row(0, 0), row(1, .01, gyro_z_corrected_dps="40"), row(2, .02)])
            data = replay.build_data(root, {})
            self.assertEqual(data["summary"]["rawRows"], 3)
            self.assertEqual(len(data["samples"]), 1)
            self.assertEqual(data["samples"][0]["gz"], .1)
            self.assertEqual(data["samples"][0]["range"]["gz"], [.1, 40])

    def test_gap_segment_and_health_transitions_are_not_decimated_away(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.capture(root, [row(0, 0), row(1, .01, gnss_fix_quality="RTK Float"),
                                row(2, .08, yaw_segment="1", dt_s="0.07"), row(3, .09, yaw_segment="1")])
            data = replay.build_data(root, {})
            self.assertEqual(len(data["gaps"]), 1)
            self.assertEqual(data["gaps"][0]["dt_s"], .07)
            self.assertEqual(len(data["samples"]), 3)
            episode = data["episodes"][0]
            self.assertEqual(episode["condition"], "fixed")
            self.assertAlmostEqual(episode["duration_s"], .07)

    def test_baseline_and_heading_age_are_independent_of_logger_flag(self):
        self.assertEqual(replay.health(row(0, 0))["code"], "good")
        self.assertEqual(replay.health(row(0, 0, gnss_relpos_length_m="1.77"))["code"], "heading")
        self.assertEqual(replay.health(row(0, 0, gnss_heading_age_s="4"))["code"], "heading")
        self.assertEqual(replay.health(row(0, 0, gnss_packet_age_s="4"))["code"], "stale")

    def test_nonincreasing_time_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.capture(root, [row(0, .01), row(1, 0)])
            with self.assertRaisesRegex(ValueError, "monotonic"):
                replay.build_data(root, {})

    def test_operator_notes_align_and_retain_minute_uncertainty(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.capture(root, [row(0, 0), row(1, 70)])
            data = replay.build_data(root, {"operator_reported_events": [
                {"event": "engine_started", "local_time": "2026-11-17T11:00:10-05:00"}]})
            event = next(e for e in data["events"] if e["category"] == "operator")
            self.assertAlmostEqual(event["t"], 10)
            self.assertAlmostEqual(event["uncertainUntil"], 70)

    def test_render_reuses_shared_geometry_and_escapes_script_termination(self):
        page = replay.render({"title": "</script>"})
        self.assertNotIn("__SHARED_", page)
        self.assertIn("function viewport(s)", page)
        self.assertIn('"title":"\\u003c/script>"', page)
        self.assertNotIn('<script src=', page)
        self.assertNotIn('fetch(', page)


if __name__ == "__main__":
    unittest.main()
