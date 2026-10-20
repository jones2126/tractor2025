import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / "field_testing/tools/build_62_collins_staged_coverage_20261019.py"
SPEC = importlib.util.spec_from_file_location("staged_coverage_stage0", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class Stage0CoverageInventoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.output = Path(cls.temp.name) / "stage0"
        cls.validation = MODULE.build(cls.output)
        cls.inventory = json.loads(
            (cls.output / "62_Collins_inventory_rev004_PROPOSED_REVIEW_ONLY.geojson").read_text(
                encoding="utf-8"
            )
        )

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_stage0_validation_passes(self):
        self.assertEqual("PASS", self.validation["result"])

    def test_o012_is_provenance_only(self):
        features = [
            feature
            for feature in self.inventory["features"]
            if feature["properties"].get("asset_id") == "OBS-NEW-012"
        ]
        self.assertEqual(1, len(features))
        self.assertEqual("Point", features[0]["geometry"]["type"])
        self.assertEqual(
            "rejected_location_reference_only",
            features[0]["properties"]["geometry_role"],
        )

    def test_exactly_one_active_polygon_per_real_obstacle(self):
        active = [
            feature
            for feature in self.inventory["features"]
            if feature["properties"].get("geometry_role")
            == "proposed_active_mowing_exclusion"
        ]
        ids = [feature["properties"]["asset_id"] for feature in active]
        self.assertEqual(15, len(ids))
        self.assertEqual(15, len(set(ids)))
        self.assertNotIn("OBS-NEW-012", ids)
        self.assertTrue(all(feature["geometry"]["type"] in {"Polygon", "MultiPolygon"} for feature in active))

    def test_no_coverage_or_mission_geometry_is_created(self):
        forbidden = {"coverage_cell", "stripe", "turn", "waypoint", "mission"}
        categories = {
            feature["properties"].get("category")
            for feature in self.inventory["features"]
        }
        self.assertTrue(forbidden.isdisjoint(categories))
        self.assertFalse(self.inventory["properties"]["mission_generated"])

    def test_directional_routes_and_access_points_are_preserved(self):
        routes = [
            feature
            for feature in self.inventory["features"]
            if feature["properties"].get("category") == "transition_route"
        ]
        access = [
            feature
            for feature in self.inventory["features"]
            if feature["properties"].get("category") == "access_point"
        ]
        self.assertEqual(5, len(routes))
        self.assertEqual(8, len(access))
        self.assertTrue(
            all(route["properties"].get("directionality") == "recorded_direction_only" for route in routes)
        )

    def test_manifest_hashes_every_declared_input(self):
        manifest = json.loads((self.output / "stage_0_input_manifest.json").read_text(encoding="utf-8"))
        self.assertGreaterEqual(len(manifest["inputs"]), 15)
        self.assertTrue(all(len(item["sha256"]) == 64 for item in manifest["inputs"]))
        self.assertFalse(
            manifest["planning_parameters_for_later_review"]["applied_to_stage_0_geometry"]
        )

    def test_reconciliation_records_o012_retirement(self):
        with (self.output / "stage_0_reconciliation.csv").open(encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
        row = next(row for row in rows if row["asset_id"] == "OBS-NEW-012")
        self.assertEqual("retire_false_obstacle_keep_id", row["action"])
        self.assertEqual("resolved_owner_rejected", row["stage_0_decision"])


if __name__ == "__main__":
    unittest.main()
