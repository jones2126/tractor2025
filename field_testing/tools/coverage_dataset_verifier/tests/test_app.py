import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "app.py"
SPEC = importlib.util.spec_from_file_location("coverage_dataset_verifier", MODULE_PATH)
APP = importlib.util.module_from_spec(SPEC)
assert SPEC.loader
SPEC.loader.exec_module(APP)


class CoverageDatasetVerifierTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.repo = Path(self.temp_dir.name)
        (self.repo / "data").mkdir()
        (self.repo / "data" / "track.csv").write_text(
            "time,lat,lon,speed_mps\n"
            "2026-01-01T12:00:00Z,40.1,-80.2,0.2\n"
            "2026-01-01T12:00:01Z,40.2,-80.3,0.3\n",
            encoding="utf-8",
        )

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_csv_inspection_reports_hash_columns_and_ranges(self):
        result = APP.inspect_source(self.repo, "data/track.csv")
        self.assertTrue(result["ok"])
        self.assertEqual(result["rowCount"], 2)
        self.assertEqual(result["latitudeRange"], [40.1, 40.2])
        self.assertEqual(result["longitudeRange"], [-80.3, -80.2])
        self.assertEqual(result["firstCoordinate"]["dataRow"], 1)
        self.assertEqual(result["firstCoordinate"]["latitude"], 40.1)
        self.assertEqual(len(result["sha256"]), 64)

    def test_paths_cannot_escape_repository(self):
        with self.assertRaises(ValueError):
            APP.safe_repo_path(self.repo, "../outside.csv")

    def test_approval_writes_project_and_hash_chained_ledger(self):
        digest = APP.sha256_file(self.repo / "data" / "track.csv")
        sources = [{
            "id": "raw-survey",
            "name": "raw survey",
            "sourceType": "raw_recording",
            "purposes": sorted(APP.REQUIRED_PURPOSES),
            "sourceDecision": "use_this_source",
            "required": True,
            "path": "data/track.csv",
            "expectedSha256": digest,
        }]
        project = {
            "status": APP.REVIEW_STATUS,
            "siteId": "TEST",
            "siteName": "Test yard",
            "outputDirectory": "output/test",
            "equipment": {"deckWidthM": 1.0},
            "coordinateFrame": {"sourceId": "raw-survey"},
            "sources": sources,
            "sourceApproval": {"reviewer": "Al", "reviewDate": "2026-10-20", "comments": "checked", "attestation": True},
        }
        first = APP.save_project(self.repo, project, approve=True)
        second = APP.save_project(self.repo, project, approve=True)
        self.assertTrue(first["ok"])
        ledger = json.loads((self.repo / "output" / "test" / "approval-ledger.json").read_text(encoding="utf-8"))
        self.assertEqual(len(ledger["events"]), 2)
        self.assertEqual(ledger["events"][1]["previousEventHash"], ledger["events"][0]["eventHash"])
        saved = json.loads((self.repo / "output" / "test" / "site-project.json").read_text(encoding="utf-8"))
        self.assertFalse(saved["coverageGenerationAllowed"])
        self.assertEqual(saved["gates"]["datasetApproval"], "blocked")
        self.assertEqual(saved["projectRevision"], 2)
        self.assertEqual(saved["coordinateFrame"]["resolvedOrigin"]["latitude"], 40.1)

    def test_csv_preview_links_file_to_recorded_path(self):
        result = APP.preview_source(self.repo, "data/track.csv")
        self.assertTrue(result["ok"])
        self.assertEqual(result["features"][0]["geometry"]["type"], "LineString")
        self.assertEqual(result["features"][0]["geometry"]["coordinates"][0], [-80.2, 40.1])

    def test_geojson_preview_exposes_feature_name_and_geometry(self):
        geojson = {
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "properties": {"name": "north boundary", "kind": "mowable_area", "review_only": True},
                "geometry": {"type": "Polygon", "coordinates": [[[0, 0], [1, 0], [1, 1], [0, 0]]]},
            }],
        }
        (self.repo / "data" / "boundary.geojson").write_text(json.dumps(geojson), encoding="utf-8")
        result = APP.preview_source(self.repo, "data/boundary.geojson")
        self.assertEqual(result["features"][0]["name"], "north boundary")
        self.assertEqual(result["features"][0]["geometry"]["type"], "Polygon")

    def test_incomplete_draft_can_be_saved(self):
        project = {
            "status": APP.REVIEW_STATUS,
            "siteId": "NEW_SITE",
            "siteName": "",
            "outputDirectory": "output/draft",
            "equipment": {},
            "coordinateFrame": {},
            "sources": [],
            "sourceApproval": {},
        }
        result = APP.save_project(self.repo, project, approve=False)
        self.assertTrue(result["ok"])
        self.assertEqual(result["project"]["gates"]["sources"], "draft")


if __name__ == "__main__":
    unittest.main()
