#!/usr/bin/env python3
"""Replay every master waypoint as a possible Manual-to-AUTO resume location."""

from __future__ import annotations

import bisect
import csv
import importlib.util
import json
from collections import Counter
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
GENERATED = HERE / "generated"
MISSION = GENERATED / "62_Collins_master_boundary_replay_REVIEW_ONLY_20260915.txt"
AUDIT = GENERATED / "62_Collins_master_boundary_replay_audit_20260915.csv"
CONTROLLER = ROOT / "tractor_rpi" / "pure-pursuit" / "pure_pursuit_controller_20260915.py"


def load_controller_module():
    spec = importlib.util.spec_from_file_location("pure_pursuit_20260915", CONTROLLER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    module = load_controller_module()
    pursuit = module.PurePursuit()
    pursuit.load_path(str(MISSION))
    with AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        audit = list(csv.DictReader(handle))

    results = []
    for index, point in enumerate(pursuit.path[:-1]):
        simulated_prior_s = max(0.0, pursuit.cumulative_s[index] - 20.0)
        pursuit.progress_s = simulated_prior_s
        pursuit.idx = max(0, bisect.bisect_right(pursuit.cumulative_s, simulated_prior_s) - 1)
        pursuit.reacquire_state = "REQUIRED"
        acquired = pursuit.reacquire_forward(point[0], point[1], point[2])
        results.append({
            "waypoint": index + 1,
            "segment": int(audit[index]["segment"]),
            "phase": audit[index]["phase"],
            "mission_progress_m": round(pursuit.cumulative_s[index], 3),
            "simulated_manual_travel_m": round(pursuit.cumulative_s[index] - simulated_prior_s, 3),
            "result": "ACQUIRED" if acquired else pursuit.reacquire_state,
            "detail": pursuit.reacquire_detail,
        })
    pursuit.sock.close()

    with (GENERATED / "forward_recovery_waypoint_audit_20260915.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)

    by_result = Counter(row["result"] for row in results)
    blocked_by_phase = Counter(
        row["phase"] for row in results if row["result"] != "ACQUIRED"
    )
    report = {
        "status": "PROTOTYPE_REVIEW_ONLY",
        "scenario": "At each waypoint, simulate 20 m of Manual travel and then request AUTO.",
        "tested_locations": len(results),
        "results": dict(sorted(by_result.items())),
        "blocked_by_phase": dict(sorted(blocked_by_phase.items())),
        "interpretation": (
            "BLOCKED locations are deliberate safe refusals where two forward path "
            "branches are almost equally close but far apart in mission progress. "
            "The operator can remain in Manual and move/align until the choice is unique."
        ),
    }
    (GENERATED / "forward_recovery_validation_report_20260915.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8", newline="\n"
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
