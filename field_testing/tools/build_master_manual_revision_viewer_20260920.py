#!/usr/bin/env python3
"""Embed the review-only revised mission and original route in one offline HTML file."""

from __future__ import annotations

import csv
import json
from pathlib import Path

from build_master_manual_revision_20260920 import OUT, main as build_revision, sha256
from build_master_target_mission_viewer_20260919 import MISSION, load


HERE = Path(__file__).resolve().parent
TEMPLATE = HERE / "master-manual-revision-viewer-template.html"
AUDIT = OUT / "62_Collins_master_manual_revision_audit_20260920.csv"
REPORT = OUT / "62_Collins_master_manual_revision_report_20260920.json"
MISSION_DRAFT = OUT / "62_Collins_master_manual_revision_DRAFT_1mps_20260920.txt"
OUTPUT = OUT / "62_Collins_master_manual_revision_INTERACTIVE_REVIEW_20260920.html"


def main() -> None:
    if not all(p.is_file() for p in (AUDIT, REPORT, MISSION_DRAFT)):
        build_revision()
    source = load()
    report = json.loads(REPORT.read_text(encoding="utf-8"))
    if report["source_mission_sha256"] != sha256(MISSION):
        raise ValueError("Original mission hash differs from review candidate")
    with AUDIT.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != report["revised_waypoints"] or len(rows) != 21_792:
        raise ValueError("Unexpected revised audit count")
    phases = list(dict.fromkeys(row["phase"] for row in rows))
    phase_id = {name: i for i, name in enumerate(phases)}
    revised = []
    for i, row in enumerate(rows, 1):
        if int(row["revision_waypoint"]) != i:
            raise ValueError(f"Bad revision index at {i}")
        revised.append([
            float(row["east_m"]), float(row["north_m"]),
            float(row["lat"]), float(row["lon"]),
            phase_id[row["phase"]], row["source_id"],
            float(row["heading_deg"]), float(row["speed_mps"]), row["gps_fix"],
        ])
    ids = {p[5]: i for i, p in enumerate(revised)}
    for marker in ("W12652", "M18", "M2860", "W12845", "W13642", "W13704", "W13907", "W13941",
                   "W14517", "W14556", "W15749", "W15807", "W15867", "W15903"):
        if marker not in ids:
            raise ValueError(f"Missing expected source marker {marker}")
    data = {
        "title": "Master mission revision · review only",
        "original": [[p[0], p[1]] for p in source["points"]],
        "revised": revised,
        "phases": phases,
        "originalCount": len(source["points"]),
        "sourceHash": report["source_mission_sha256"],
        "draftHash": sha256(MISSION_DRAFT),
        "removed": report["removed_original_ranges"],
        "markers": ["W12652", "M18", "M2860", "W12845", "W13642", "W13704", "W13907", "W13941",
                    "W14517", "W14556", "W15749", "W15807", "W15867", "W15903"],
    }
    template = TEMPLATE.read_text(encoding="utf-8")
    if template.count("__DATA__") != 1:
        raise ValueError("Expected one data placeholder")
    encoded = json.dumps(data, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    OUTPUT.write_text(template.replace("__DATA__", encoded), encoding="utf-8", newline="\n")
    print(f"{OUTPUT} | {len(revised):,} revised waypoints | {OUTPUT.stat().st_size:,} bytes")


if __name__ == "__main__":
    main()
