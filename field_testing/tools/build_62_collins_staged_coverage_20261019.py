#!/usr/bin/env python3
"""Build Stage 0 of the 62 Collins staged coverage-planning workflow.

Stage 0 reconciles sources and creates review-only inventory artifacts.  It
does not create cells, stripes, turns, mission waypoints, or launchers.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import subprocess
from collections import Counter, defaultdict
from datetime import date
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from shapely.geometry import Point, mapping, shape
from shapely.ops import transform


REPO = Path(__file__).resolve().parents[2]
SITE = REPO / "field_testing/sites/62_Collins_multi_boundary_20260915"
PLAN = SITE / "planning/20261019_staged_coverage"
DEFAULT_OUT = PLAN / "stage_0_source_inventory"
STATUS = "REVIEW ONLY — NOT APPROVED FOR FIELD USE"

REV2 = SITE / "site_inventory/62_Collins_site_inventory.geojson"
REV2_MANIFEST = SITE / "site_inventory/62_Collins_site_inventory_manifest.json"
REV2_VALIDATION = SITE / "site_inventory/62_Collins_site_inventory_validation.json"
REV3_DIR = SITE / "site_inventory/revisions/rev_003_20260926_REVIEW_ONLY"
REV3 = REV3_DIR / "62_Collins_site_inventory_rev003_REVIEW_ONLY.geojson"
REV3_REPORT = REV3_DIR / "62_Collins_site_inventory_rev003_report.json"
REV3_WORKSHEET = REV3_DIR / "62_Collins_site_inventory_rev003_owner_review_worksheet.csv"
TURN_REPORT = SITE / "analysis/20260923_turn_diagnosis/REPORT.md"
TURN_METRICS = SITE / "analysis/20260923_turn_diagnosis/all_ring_turn_metrics.csv"
PERIMETER_VALIDATION = SITE / (
    "mission_plans/20260929_consolidated_perimeter_field_test/generated/"
    "62_Collins_consolidated_perimeter_1mps_20260929_validation.json"
)
OCTOBER_DIR = SITE / "planning/20261005_zamboni_coverage/generated_review_20261005"
OCTOBER_MANIFEST = OCTOBER_DIR / "input_manifest.json"
OCTOBER_VALIDATION = OCTOBER_DIR / "coverage_validation_report.json"
OCTOBER_README = OCTOBER_DIR / "README.md"
WORKFLOW = REPO / (
    "tractor_rpi/pure-pursuit/mission_planning/"
    "SITE_COVERAGE_MISSION_WORKFLOW_20260724.md"
)
STAGED_PROMPT = REPO / "obsidian_vault/02-testing/20261019-staged-coverage-planning-prompt.md"

REJECTED_OBSTACLE = "OBS-NEW-012"
AREA_ORDER = ["AREA-BACKYARD-GARDENS", "AREA-FRONT-YARD", "AREA-OVER-ROAD"]
DECK_WIDTH_M = 1.0668
HISTORICAL_LANE_SPACING_M = 0.9652
HISTORICAL_OVERLAP_M = 0.1016
WEAKER_RIGHT_RADIUS_M = 1.63
SOFT_TURN_REVIEW_RADIUS_M = 1.90

ACCESS_LABELS = {
    "ACCESS-BACKYARD-EAST": "BY east",
    "ACCESS-BACKYARD-NORTHEAST": "BY northeast",
    "ACCESS-FRONT-SOUTHWEST": "Front southwest",
    "ACCESS-FRONT-WEST-RETURN": "Front west return",
    "ACCESS-FRONT-NORTH-OUTBOUND": "Front north out",
    "ACCESS-FRONT-NORTH-INBOUND": "Front north in",
    "ACCESS-OVERROAD-NORTH-OUTBOUND": "Over-road out",
    "ACCESS-OVERROAD-NORTH-INBOUND": "Over-road in",
}
ACCESS_OFFSETS = {
    "ACCESS-BACKYARD-EAST": (5, -12),
    "ACCESS-BACKYARD-NORTHEAST": (5, 7),
    "ACCESS-FRONT-SOUTHWEST": (5, 7),
    "ACCESS-FRONT-WEST-RETURN": (5, -13),
    "ACCESS-FRONT-NORTH-OUTBOUND": (5, -12),
    "ACCESS-FRONT-NORTH-INBOUND": (5, 6),
    "ACCESS-OVERROAD-NORTH-OUTBOUND": (5, 7),
    "ACCESS-OVERROAD-NORTH-INBOUND": (5, -12),
}
ROUTE_LABELS = {
    "ROUTE-BASE-TO-BACKYARD": "Base → backyard",
    "ROUTE-BACKYARD-TO-FRONT": "Backyard → front",
    "ROUTE-FRONT-TO-OVERROAD": "Front → over-road",
    "ROUTE-OVERROAD-TO-FRONT": "Over-road → front",
    "ROUTE-FRONT-TO-BASE": "Front → base",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rel(path: Path) -> str:
    return path.resolve().relative_to(REPO.resolve()).as_posix()


def git_value(*args: str) -> str | None:
    try:
        result = subprocess.run(
            ["git", *args], cwd=REPO, check=True, capture_output=True, text=True
        )
        return result.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def source_paths(rev3: dict) -> list[Path]:
    paths = {
        REV2,
        REV2_MANIFEST,
        REV2_VALIDATION,
        REV3,
        REV3_REPORT,
        REV3_WORKSHEET,
        TURN_REPORT,
        TURN_METRICS,
        PERIMETER_VALIDATION,
        OCTOBER_MANIFEST,
        OCTOBER_VALIDATION,
        OCTOBER_README,
        WORKFLOW,
        STAGED_PROMPT,
        Path(__file__),
    }
    for feature in rev3["features"]:
        for value in str(feature.get("properties", {}).get("source_file", "")).split(";"):
            if value.strip():
                paths.add(REPO / value.strip())
    missing = [path for path in paths if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing Stage 0 inputs: " + ", ".join(map(str, missing)))
    return sorted(paths, key=lambda path: rel(path))


def make_manifest(paths: list[Path]) -> dict:
    head = git_value("rev-parse", "HEAD")
    dirty = bool(git_value("status", "--porcelain"))
    return {
        "site_id": "62_COLLINS",
        "stage": 0,
        "status": STATUS,
        "created_date": str(date.today()),
        "git_head": head,
        "working_tree_dirty": dirty,
        "notes": [
            "Hashes identify the exact inputs reconciled by Stage 0.",
            "A dirty working tree is recorded because some review inputs are not committed.",
            "The 2026-09-29 perimeter package was approved for a supervised blades-off test; that is not full physical validation of every candidate boundary.",
        ],
        "inputs": [
            {"path": rel(path), "sha256": sha256(path), "bytes": path.stat().st_size}
            for path in paths
        ],
        "planning_parameters_for_later_review": {
            "deck_width_m": DECK_WIDTH_M,
            "historical_lane_spacing_m": HISTORICAL_LANE_SPACING_M,
            "historical_nominal_overlap_m": HISTORICAL_OVERLAP_M,
            "historical_weaker_right_turn_radius_m": WEAKER_RIGHT_RADIUS_M,
            "soft_turn_review_radius_m": SOFT_TURN_REVIEW_RADIUS_M,
            "applied_to_stage_0_geometry": False,
        },
    }


def proposed_inventory(rev3: dict) -> dict:
    kept: list[dict] = []
    for original in rev3["features"]:
        feature = copy.deepcopy(original)
        props = feature.get("properties", {})
        category = props.get("category")
        role = props.get("geometry_role")
        asset = props.get("asset_id")

        include = False
        if category != "obstacle":
            include = True
        elif asset == REJECTED_OBSTACLE and role == "derived_center_point":
            include = True
            props.update(
                category="retired_obstacle_record",
                geometry_role="rejected_location_reference_only",
                status="owner_rejected_false_obstacle",
                notes=(
                    "O012 / OBS-NEW-012 is retained only as provenance. "
                    "It creates no active keep-out geometry and the identifiers are not reused."
                ),
            )
        elif asset == REJECTED_OBSTACLE:
            include = False
        elif role in {
            "derived_center_point",
            "recorded_centerline_evidence",
            "tested_tractor_centerline_limit",
            "minimum_recorded_gps_loop",
        }:
            include = True
            props["stage_0_use"] = "evidence_only_not_active_keep_out"
        elif role in {"candidate_mowing_exclusion", "mowing_exclusion"}:
            include = True
            props["source_geometry_role"] = role
            props["geometry_role"] = "proposed_active_mowing_exclusion"
            props["status"] = "review_only_owner_approval_required"
            props["stage_0_use"] = "single_proposed_active_keep_out"
        elif role == "candidate_polygon_mowing_exclusion":
            include = False

        if include:
            props["inventory_revision"] = 4
            props["inventory_revision_label"] = "rev_004_PROPOSED_REVIEW_ONLY"
            props["stage_0_status"] = STATUS
            kept.append(feature)

    return {
        "type": "FeatureCollection",
        "properties": {
            "site_id": "62_COLLINS",
            "inventory_revision": 4,
            "inventory_revision_label": "rev_004_PROPOSED_REVIEW_ONLY",
            "created_date": str(date.today()),
            "status": STATUS,
            "stage": 0,
            "mission_generated": False,
            "approval_required_before_stage_1": True,
            "notes": [
                "Proposed reconciliation of revision 2 and revision 3; it does not replace the authoritative inventory.",
                "One proposed active exclusion is retained per real obstacle; alternate derived polygons remain available in revision 3 provenance.",
                "O012 / OBS-NEW-012 is retired without renumbering other obstacle identifiers.",
            ],
        },
        "features": kept,
    }


def reconciliation_rows(rev3: dict) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for feature in rev3["features"]:
        props = feature.get("properties", {})
        grouped[(str(props.get("category")), str(props.get("asset_id")))].append(props)

    rows: list[dict[str, object]] = []
    for (category, asset), props_list in sorted(grouped.items()):
        first = props_list[0]
        roles = sorted({str(props.get("geometry_role")) for props in props_list})
        source_files = sorted(
            {
                item.strip()
                for props in props_list
                for item in str(props.get("source_file", "")).split(";")
                if item.strip()
            }
        )
        if category == "obstacle" and asset == REJECTED_OBSTACLE:
            action = "retire_false_obstacle_keep_id"
            proposed_role = "rejected_location_reference_only"
            decision = "resolved_owner_rejected"
            question = "Confirm O012 remains retired and creates no keep-out."
        elif category == "obstacle":
            action = "retain_one_active_exclusion_and_evidence"
            proposed_role = "proposed_active_mowing_exclusion"
            decision = "owner_approval_required"
            question = "Confirm obstacle identity and proposed keep-out shape."
        elif category == "mowable_area":
            action = "carry_candidate_boundary"
            proposed_role = "candidate_mowable_boundary"
            decision = "owner_approval_required"
            question = "Confirm this candidate mowing boundary before section planning."
        elif category == "transition_route":
            action = "carry_recorded_direction_only"
            proposed_role = "recorded_centerline_between_areas"
            decision = "owner_approval_required"
            question = "Confirm this named route and its one-way direction."
        elif category == "access_point":
            action = "carry_entry_exit_option"
            proposed_role = "area_entry_exit_point"
            decision = "owner_approval_required"
            question = "Confirm this is a useful entry/exit point."
        else:
            action = "retain_as_provenance"
            proposed_role = ";".join(roles)
            decision = "information_only"
            question = ""
        rows.append(
            {
                "asset_id": asset,
                "asset_name": first.get("asset_name", ""),
                "category": category,
                "related_area": first.get("related_area", ""),
                "revision_3_roles": ";".join(roles),
                "proposed_role": proposed_role,
                "source_status": first.get("status", ""),
                "stage_0_decision": decision,
                "action": action,
                "source_files": ";".join(source_files),
                "owner_review_question": question,
            }
        )

    rows.extend(
        [
            {
                "asset_id": "PARAM-DECK-WIDTH",
                "asset_name": "42-inch mower deck",
                "category": "planning_parameter",
                "related_area": "all",
                "revision_3_roles": "deck_edge_inset_source",
                "proposed_role": "later_stage_constraint",
                "source_status": "documented",
                "stage_0_decision": "information_only",
                "action": "record_not_apply",
                "source_files": rel(REV3),
                "owner_review_question": f"Confirm deck width remains {DECK_WIDTH_M:.4f} m.",
            },
            {
                "asset_id": "PARAM-LANE-SPACING",
                "asset_name": "Historical 38-inch lane spacing",
                "category": "planning_parameter",
                "related_area": "all",
                "revision_3_roles": "none",
                "proposed_role": "later_stage_candidate",
                "source_status": "historical_context",
                "stage_0_decision": "defer_to_angle_and_coverage_review",
                "action": "record_not_apply",
                "source_files": rel(OCTOBER_README),
                "owner_review_question": "Confirm whether 38-inch spacing should be evaluated later.",
            },
            {
                "asset_id": "PARAM-TURN-RADIUS",
                "asset_name": "Turn-radius evidence",
                "category": "planning_parameter",
                "related_area": "all",
                "revision_3_roles": "none",
                "proposed_role": "later_stage_constraint",
                "source_status": "historical_evidence_not_production_approval",
                "stage_0_decision": "defer_to_turn_stage",
                "action": "record_1.63m_history_and_1.90m_review_target",
                "source_files": f"{rel(TURN_REPORT)};{rel(TURN_METRICS)};{rel(OCTOBER_README)}",
                "owner_review_question": "No turn radius is approved by Stage 0.",
            },
            {
                "asset_id": "EVIDENCE-PERIMETER-20260929",
                "asset_name": "Consolidated perimeter blades-off package",
                "category": "field_evidence",
                "related_area": "all",
                "revision_3_roles": "candidate_boundary_test_package",
                "proposed_role": "provenance_only",
                "source_status": "approved_for_supervised_blades_off_test_not_fully_executed",
                "stage_0_decision": "do_not_claim_full_field_validation",
                "action": "record_limit",
                "source_files": rel(PERIMETER_VALIDATION),
                "owner_review_question": "Candidate boundaries remain review-only.",
            },
            {
                "asset_id": "BASELINE-OCTOBER-20261005",
                "asset_name": "October whole-site coverage review",
                "category": "historical_baseline",
                "related_area": "all",
                "revision_3_roles": "coverage_review",
                "proposed_role": "comparison_only",
                "source_status": "review_only",
                "stage_0_decision": "preserve_unchanged",
                "action": "do_not_overwrite_or_promote",
                "source_files": f"{rel(OCTOBER_MANIFEST)};{rel(OCTOBER_VALIDATION)};{rel(OCTOBER_README)}",
                "owner_review_question": "None; this is a preserved comparison baseline.",
            },
        ]
    )
    return rows


def local_frame(features: list[dict]):
    points = []
    for feature in features:
        geom = shape(feature["geometry"])
        center = geom.centroid
        points.append((center.x, center.y))
    origin_lon = sum(point[0] for point in points) / len(points)
    origin_lat = sum(point[1] for point in points) / len(points)
    east = 111_320.0 * math.cos(math.radians(origin_lat))
    north = 110_540.0

    def to_xy(lon, lat, z=None):
        return (lon - origin_lon) * east, (lat - origin_lat) * north

    return to_xy


def plot_polygon(ax, geom, **kwargs):
    polygons = [geom] if geom.geom_type == "Polygon" else list(geom.geoms)
    for polygon in polygons:
        x, y = polygon.exterior.xy
        ax.fill(x, y, **kwargs)
        for ring in polygon.interiors:
            hx, hy = ring.xy
            ax.fill(hx, hy, color="white", zorder=kwargs.get("zorder", 1) + 0.1)


def build_map(inventory: dict, destination: Path) -> None:
    to_xy = local_frame(inventory["features"])
    fig, ax = plt.subplots(figsize=(13, 10), constrained_layout=True)
    area_colors = {
        "AREA-BACKYARD-GARDENS": "#7fb3d5",
        "AREA-FRONT-YARD": "#82c596",
        "AREA-OVER-ROAD": "#e6bd79",
    }
    rejected_point = None
    for feature in inventory["features"]:
        props = feature["properties"]
        geom = transform(to_xy, shape(feature["geometry"]))
        category = props.get("category")
        role = props.get("geometry_role")
        asset = props.get("asset_id", "")
        if category == "mowable_area" and role == "candidate_mowable_boundary":
            plot_polygon(
                ax,
                geom,
                color=area_colors.get(asset, "#cccccc"),
                alpha=0.28,
                edgecolor="#263238",
                linewidth=1.5,
                zorder=1,
            )
            point = geom.representative_point()
            ax.text(point.x, point.y, props.get("asset_name", asset), ha="center", va="center", fontsize=9, weight="bold")
        elif category == "obstacle" and role == "proposed_active_mowing_exclusion":
            plot_polygon(ax, geom, color="#d95f59", alpha=0.55, edgecolor="#8b1a1a", linewidth=1.0, zorder=4)
            point = geom.representative_point()
            ax.text(point.x, point.y, asset.replace("OBS-NEW-", "O"), ha="center", va="center", fontsize=7, zorder=6)
        elif category == "transition_route":
            x, y = geom.xy
            ax.plot(x, y, color="#4f5b62", linewidth=1.7, alpha=0.9, zorder=3)
            if len(x) > 1:
                index = max(0, len(x) // 2 - 1)
                ax.annotate("", xy=(x[index + 1], y[index + 1]), xytext=(x[index], y[index]), arrowprops={"arrowstyle": "->", "color": "#4f5b62", "lw": 1.7})
            midpoint = geom.interpolate(0.5, normalized=True)
            ax.annotate(
                ROUTE_LABELS.get(asset, asset),
                (midpoint.x, midpoint.y),
                xytext=(3, 4),
                textcoords="offset points",
                fontsize=7,
                color="#37474f",
                zorder=6,
            )
        elif category == "access_point":
            ax.scatter([geom.x], [geom.y], marker="s", s=34, color="#2455a4", edgecolor="white", linewidth=0.5, zorder=7)
            ax.annotate(
                ACCESS_LABELS.get(asset, asset),
                (geom.x, geom.y),
                xytext=ACCESS_OFFSETS.get(asset, (4, 4)),
                textcoords="offset points",
                fontsize=6.5,
                color="#173b70",
            )
        elif category == "site_reference_point":
            ax.scatter([geom.x], [geom.y], marker="*", s=95, color="#7d3c98", zorder=7)
        elif category == "retired_obstacle_record":
            rejected_point = geom

    if rejected_point is not None:
        ax.scatter([rejected_point.x], [rejected_point.y], marker="x", s=90, color="#111111", linewidth=2, zorder=8)
        ax.annotate("O012 retired\n(no keep-out)", (rejected_point.x, rejected_point.y), xytext=(8, -18), textcoords="offset points", fontsize=8, weight="bold")

    legend = [
        Line2D([0], [0], color="#263238", lw=8, alpha=0.25, label="Candidate mowable boundaries"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#d95f59", markeredgecolor="#8b1a1a", markersize=9, label="15 proposed obstacle keep-outs"),
        Line2D([0], [0], color="#4f5b62", lw=2, label="5 recorded one-way transition routes"),
        Line2D([0], [0], marker="s", color="none", markerfacecolor="#2455a4", markersize=8, label="8 entry/exit points"),
        Line2D([0], [0], marker="x", color="#111111", lw=0, markersize=8, label="O012 retired; provenance only"),
    ]
    ax.legend(handles=legend, loc="lower left", frameon=True, fontsize=9)
    ax.set_title("62 Collins — Stage 0 source and inventory reconciliation", fontsize=16, weight="bold", pad=24)
    ax.text(0.5, 1.01, STATUS, transform=ax.transAxes, ha="center", va="bottom", color="#a01818", fontsize=12, weight="bold")
    ax.text(0.99, 0.01, "No cells, stripes, turns, or mission waypoints are shown.", transform=ax.transAxes, ha="right", va="bottom", fontsize=9, color="#4f5b62")
    ax.set_xlabel("Local east (m)")
    ax.set_ylabel("Local north (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, color="#d9d9d9", linewidth=0.5, zorder=0)
    fig.savefig(destination, dpi=180)
    plt.close(fig)


def validate(inventory: dict, rows: list[dict], manifest: dict) -> dict:
    counts = Counter(
        (f["properties"].get("category"), f["properties"].get("geometry_role"))
        for f in inventory["features"]
    )
    active_ids = [
        f["properties"].get("asset_id")
        for f in inventory["features"]
        if f["properties"].get("geometry_role") == "proposed_active_mowing_exclusion"
    ]
    retired = [
        f for f in inventory["features"]
        if f["properties"].get("asset_id") == REJECTED_OBSTACLE
    ]
    geometry_valid = all(shape(f["geometry"]).is_valid for f in inventory["features"])
    checks = {
        "three_candidate_mowable_boundaries": counts[("mowable_area", "candidate_mowable_boundary")] == 3,
        "fifteen_unique_active_obstacles": len(active_ids) == 15 and len(set(active_ids)) == 15,
        "o012_has_no_active_keep_out": REJECTED_OBSTACLE not in active_ids,
        "o012_has_one_retired_provenance_point": len(retired) == 1 and retired[0]["geometry"]["type"] == "Point",
        "five_directional_transition_routes": counts[("transition_route", "recorded_centerline_between_areas")] == 5,
        "eight_access_points": counts[("access_point", "area_entry_exit_point")] == 8,
        "all_geometries_valid": geometry_valid,
        "reconciliation_table_not_empty": len(rows) > 0,
        "input_hashes_present": len(manifest["inputs"]) >= 15 and all(item["sha256"] for item in manifest["inputs"]),
        "mission_generation_disabled": inventory["properties"].get("mission_generated") is False,
    }
    return {
        "site_id": "62_COLLINS",
        "stage": 0,
        "status": STATUS,
        "result": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "counts": {f"{key[0]}::{key[1]}": value for key, value in sorted(counts.items())},
        "active_obstacle_ids": sorted(active_ids),
        "explicitly_not_generated": ["coverage cells", "stripe candidates", "turns", "waypoints", "mission file", "launcher"],
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_readme(path: Path, validation: dict, manifest: dict) -> None:
    text = f"""# 62 Collins staged coverage planning — Stage 0

> **{STATUS}**

Stage 0 reconciles the site sources before any new coverage geometry is planned.
It deliberately stops here for Al's review.

## What is in this checkpoint

- 3 candidate mowable-area boundaries from revision 3.
- 15 proposed active obstacle exclusions, with one active polygon per obstacle.
- `O012` / `OBS-NEW-012` retired as a false obstacle. Its identifier remains in
  the provenance record and is not reused, but it creates no keep-out.
- 8 entry/exit points and 5 recorded between-area routes. Route direction remains
  the recorded direction only.
- Exact SHA-256 hashes for {len(manifest['inputs'])} boundary, obstacle,
  transition, deck, turn-radius, field-evidence, workflow, and baseline inputs.

The proposed revision 4 file is a review artifact. It does **not** replace the
current authoritative inventory.

## Important evidence limits

- Revision 3 boundaries are still candidate boundaries. Approval of the
  September 29 package for a supervised blades-off test did not prove that every
  boundary was physically completed and validated.
- The 1.63 m weaker-right history and 1.90 m soft-turn review target are recorded
  for later stages only. Stage 0 approves no turn radius.
- The 1.0668 m deck width and historical 0.9652 m lane spacing are recorded, not
  applied to new coverage geometry.
- The October 5 whole-site coverage package remains unchanged as a comparison
  baseline; it is not promoted by this checkpoint.

## Review files

- `62_Collins_stage_0_inventory_map_REVIEW_ONLY.png` — human-readable map.
- `stage_0_reconciliation.csv` — one row per asset plus planning/evidence rows.
- `62_Collins_inventory_rev004_PROPOSED_REVIEW_ONLY.geojson` — proposed clean
  inventory with one active keep-out per real obstacle.
- `stage_0_input_manifest.json` — exact input provenance and hashes.
- `stage_0_validation.json` — mechanical checks (`{validation['result']}`).
- `staged_coverage_workflow.json` — the stage gates and current stop point.

## Approval gate

Please review the map and reconciliation table. Stage 1 must not begin until Al
confirms the boundaries, obstacles (especially the unclassified obstacles),
entry/exit points, and recorded route directions are suitable for section/cell
planning.

No cells, stripes, turns, executable mission, or launcher were generated.
"""
    path.write_text(text, encoding="utf-8")


def workflow_state() -> dict:
    stages = [
        (0, "Source and inventory reconciliation", "awaiting_owner_review"),
        (1, "Section/cell proposal", "blocked_by_stage_0"),
        (2, "Entry/exit and travel-path review", "blocked_by_stage_1"),
        (3, "Stripe-angle comparison", "blocked_by_stage_2"),
        (4, "Ordering alternatives", "blocked_by_stage_3"),
        (5, "Turn synthesis", "blocked_by_stage_4"),
        (6, "Combined coverage validation", "blocked_by_stage_5"),
    ]
    return {
        "site_id": "62_COLLINS",
        "status": STATUS,
        "current_stage": 0,
        "stop_reason": "owner approval required before section/cell planning",
        "stages": [
            {"number": number, "name": name, "status": status}
            for number, name, status in stages
        ],
        "field_use_approved": False,
        "executable_mission_generated": False,
    }


def build(output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    rev3 = json.loads(REV3.read_text(encoding="utf-8"))
    manifest = make_manifest(source_paths(rev3))
    inventory = proposed_inventory(rev3)
    rows = reconciliation_rows(rev3)
    validation = validate(inventory, rows, manifest)
    if validation["result"] != "PASS":
        raise RuntimeError(json.dumps(validation, indent=2))

    (output / "stage_0_input_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    (output / "62_Collins_inventory_rev004_PROPOSED_REVIEW_ONLY.geojson").write_text(json.dumps(inventory, indent=2) + "\n", encoding="utf-8")
    write_csv(output / "stage_0_reconciliation.csv", rows)
    (output / "stage_0_validation.json").write_text(json.dumps(validation, indent=2) + "\n", encoding="utf-8")
    (output / "staged_coverage_workflow.json").write_text(json.dumps(workflow_state(), indent=2) + "\n", encoding="utf-8")
    build_map(inventory, output / "62_Collins_stage_0_inventory_map_REVIEW_ONLY.png")
    write_readme(output / "README.md", validation, manifest)
    return validation


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=["stage0"], help="Only Stage 0 is implemented and permitted in this checkpoint.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    result = build(args.output_dir.resolve())
    print(f"{result['result']}: Stage 0 artifacts written to {args.output_dir.resolve()}")
    print("STOP: owner approval is required before Stage 1.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
