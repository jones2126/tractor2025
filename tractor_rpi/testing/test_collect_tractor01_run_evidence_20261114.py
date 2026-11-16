#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path
import tarfile
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[2]
COLLECTOR_PATH = ROOT / "field_testing/tools/collect_tractor01_run_evidence_20261114.py"
spec = importlib.util.spec_from_file_location("tractor01_evidence_collector", COLLECTOR_PATH)
collector = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(collector)

CONTROLLER_PATH = ROOT / "tractor_rpi/pure-pursuit/pure_pursuit_controller_20261022.py"
controller_spec = importlib.util.spec_from_file_location("pursuit_with_exact_log", CONTROLLER_PATH)
controller = importlib.util.module_from_spec(controller_spec)
assert controller_spec.loader is not None
controller_spec.loader.exec_module(controller)


class EvidenceCollectorTests(unittest.TestCase):
    def test_pursuit_logger_honors_an_exact_output_path(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "nested" / "pursuit_log_shared_run_id.csv"
            logger = controller.PursuitLogger(output_path=str(output))
            logger.close()
            self.assertEqual(output.resolve(), Path(logger.path))
            self.assertGreater(output.stat().st_size, 0)

    def test_complete_archive_uses_exact_manifest_paths_and_preserves_sources(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            home = root / "home"
            field_logs = root / "field_logs"
            phone_logs = field_logs / "phone_arbitrator_production"
            recovery_logs = field_logs / "manual_recovery"
            pilog = root / "pilog"
            runtime = root / "run"
            source = root / "source"
            pursuit_dir = root / "field-testing-data"
            for directory in (
                home,
                phone_logs,
                recovery_logs,
                pilog,
                runtime,
                source,
                pursuit_dir,
            ):
                directory.mkdir(parents=True)

            started = time.time() - 2
            files = {
                "field_log": source / "outer_perimeter_20261114_120000.csv",
                "pursuit_log": pursuit_dir / "pursuit_log_20261114_120000.csv",
                "dashboard_log": source / "production-mission-dashboard_20261114_120000.txt",
                "mission_file": source / "mission.txt",
                "audit_file": source / "mission_audit.csv",
            }
            for label, path in files.items():
                path.write_text(f"{label}\n", encoding="utf-8")
            (phone_logs / "wifi_primary_control_boot.jsonl").write_text("{}\n", encoding="utf-8")
            (phone_logs / "phone_flight_recorder_boot.jsonl").write_text("{}\n", encoding="utf-8")
            (recovery_logs / "recovery.jsonl").write_text("{}\n", encoding="utf-8")
            (pilog / "pilog.csv").write_text("time,throttled\nnow,0x0\n", encoding="utf-8")
            diagnostics = runtime / "diagnostics.json"
            diagnostics.write_text("{}\n", encoding="utf-8")

            manifest = {
                "schema": 1,
                "run_id": "phone_outer_perimeter_20261114_120000",
                "mission_label": "phone_outer_perimeter",
                "started_at_local": "2026-11-14 12:00:00",
                "started_at_epoch": started,
                "repository": str(ROOT),
                "expected_firmware": "teensy_main_20261110_rpi_command_lease_v5",
                "archive_prefix": "phone_outer_perimeter_20261114_120000",
                **{label: str(path.resolve()) for label, path in files.items()},
            }
            manifest_path = source / "run_manifest.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            source_hashes = {path: collector.sha256_file(path) for path in files.values()}

            args = collector.build_parser().parse_args(
                [
                    "--manifest", str(manifest_path),
                    "--home-dir", str(home),
                    "--field-logs-root", str(field_logs),
                    "--phone-log-dir", str(phone_logs),
                    "--recovery-log-dir", str(recovery_logs),
                    "--pilog-dir", str(pilog),
                    "--diagnostics-json", str(diagnostics),
                    "--skip-live-diagnostics",
                ]
            )
            complete, archive, checksum, warnings = collector.collect(args)

            self.assertTrue(complete.is_dir())
            self.assertTrue((complete / "SHA256SUMS").is_file())
            collector.verify_internal_manifest(complete, complete / "SHA256SUMS")
            self.assertEqual([], warnings)
            self.assertEqual(collector.sha256_file(archive), checksum.read_text().split()[0])
            with tarfile.open(archive, "r:gz") as handle:
                archived_names = {member.name for member in handle.getmembers()}
            self.assertTrue(any(name.endswith("/mission/outer_perimeter_20261114_120000.csv") for name in archived_names))
            self.assertTrue(any(name.endswith("/diagnostics/pilog/pilog.csv") for name in archived_names))
            for path, original_hash in source_hashes.items():
                self.assertTrue(path.is_file())
                self.assertEqual(original_hash, collector.sha256_file(path))


if __name__ == "__main__":
    unittest.main()
