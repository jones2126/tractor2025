#!/usr/bin/env python3
"""Build and validate the self-contained Tractor01 2026-09-23 mission replay.

This run is a direct capture (two CSV files), not a September 18-style
collection_summary.json archive.  Shared parsing helpers and the shared HTML
template are reused without changing the September 18 builder's input path.
"""

from __future__ import annotations

import bisect
import csv
import json
import math
from collections import Counter
from pathlib import Path

from build_tractor_mission_replays_20260919 import (
    boolean,
    digest,
    integer,
    load_field,
    load_mission,
    make_sample,
    mode,
    nearest_field,
    numeric,
)


SITE = Path(__file__).resolve().parents[1] / "sites" / "62_Collins_multi_boundary_20260915"
RUN = SITE / "runs" / "20260923_124933_master_manual_5hz"
PLAN = SITE / "mission_plans" / "20260915_master_boundary_replay" / "generated_master_manual_field_20260920"
OUTPUT = SITE / "analysis" / "20260923_mission_replay"
PURSUIT = RUN / "pursuit_log_20260923_124935.csv"
FIELD = RUN / "master_manual_field_20260923_124933.csv"
MISSION = PLAN / "62_Collins_master_manual_resampled_1mps_20260920.txt"
AUDIT = PLAN / "62_Collins_master_manual_resampled_1mps_20260920_audit.csv"
TEMPLATE = Path(__file__).with_name("tractor-mission-replay-template.html")
SHARED_BUILDER = Path(__file__).with_name("build_tractor_mission_replays_20260919.py")
OUTPUT_HTML = OUTPUT / "replay_20260923_124933_master_manual_5hz.html"


def event_labels(previous, current):
    """Return searchable labels for safety, control, and mission transitions."""
    labels = []
    if previous is None:
        labels.append(("mission", "Replay begins"))
        return labels

    if current["mode"] != previous["mode"]:
        label = "Handheld radio loss" if current["mode"] == "RADIO_LOSS" else f"Handheld {current['mode'].title()}"
        labels.append(("radio" if current["mode"] == "RADIO_LOSS" else "mode", label))
    if current["fix"] != previous["fix"]:
        if current["fix"] != "RTK Fixed":
            labels.append(("rtk", f"RTK loss: {current['fix'] or 'unavailable'}"))
        elif previous["fix"] != "RTK Fixed":
            labels.append(("rtk", "RTK Fixed restored"))
    if current["headValid"] != previous["headValid"]:
        labels.append(("heading", "Heading valid" if current["headValid"] else "Invalid heading"))
    if current["driving"] != previous["driving"]:
        labels.append(("driving", "Resumed driving" if current["driving"] else "Stopped driving"))
    if current["reacquire"] != previous["reacquire"]:
        if current["reacquire"] == "TRACKING":
            labels.append(("reacquisition", "Path reacquired"))
        else:
            labels.append(("reacquisition", f"Path reacquisition {current['reacquire'].lower()}"))
    if current["phase"] != previous["phase"]:
        labels.append(("phase", f"Mission phase: {current['phase']}"))
    if current["goal"] and not previous["goal"]:
        labels.append(("mission", "Mission goal reached"))
    return labels


def compact_state(row, phase):
    return {
        "driving": boolean(row.get("driving")),
        "mode": mode(row.get("handheld_mode")),
        "fix": row.get("fix_quality", ""),
        "headValid": boolean(row.get("head_valid")),
        "pause": boolean(row.get("software_paused")),
        "reacquire": row.get("reacquire_state", ""),
        "phase": phase,
        "goal": boolean(row.get("goal_reached")),
    }


def augment_sample(sample, row, field_row, trail_index):
    sample.update({
        "trailIndex": trail_index,
        "goal": boolean(row.get("goal_reached")),
        "waypointsTotal": integer(row.get("waypoints_total")),
        "pathProgress": numeric(row.get("path_progress_m")),
        "gpsAge": numeric(row.get("gps_age_s")),
        "headingAccuracy": numeric(row.get("heading_accuracy_deg")),
        "headingSatellites": integer(row.get("heading_numSV_used")),
        "bucket": integer(field_row.get("bucket")),
        "jrkCurrent": integer(field_row.get("jrk_current")),
        "jrkDuty": integer(field_row.get("jrk_duty_cycle")),
        "jrkMotorCurrent": integer(field_row.get("jrk_motor_current_mA")),
        "jrkValid": boolean(field_row.get("jrk_valid")),
        "jrkErrors": integer(field_row.get("jrk_errors_halting")),
        "radioSignal": field_row.get("radio_signal", ""),
        "transCmd": numeric(field_row.get("trans_cmd_vel_mps")),
    })
    return sample


def build_payload():
    for path in (PURSUIT, FIELD, MISSION, AUDIT, TEMPLATE, SHARED_BUILDER):
        if not path.is_file():
            raise FileNotFoundError(path)

    mission, lat0, lon0, sx, sy = load_mission(MISSION)
    with AUDIT.open(newline="", encoding="utf-8-sig") as handle:
        audit_rows = list(csv.DictReader(handle))
    phases = [row.get("phase", "") for row in audit_rows]
    if len(phases) != len(mission):
        raise ValueError(f"mission/audit mismatch: {len(mission)} points vs {len(phases)} audit rows")

    field_rows = load_field(FIELD)
    if not field_rows:
        raise ValueError(f"no field telemetry rows: {FIELD}")
    field_times = [stamp for stamp, _ in field_rows]

    samples, trail, events = [], [], []
    last_sample_t = -math.inf
    last_state = None
    last_row = None
    pending = None
    raw_rows = 0
    with PURSUIT.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            stamp = numeric(row.get("timestamp"))
            if stamp is None:
                continue  # descriptive units row
            raw_rows += 1
            idx = max(0, min(integer(row.get("waypoint_idx"), 0), len(phases) - 1))
            phase = phases[idx]
            lat, lon = numeric(row.get("lat")), numeric(row.get("lon"))
            point = [round((lon - lon0) * sx, 3), round((lat - lat0) * sy, 3)] \
                if lat is not None and lon is not None else None
            if point is not None:
                trail.append(point)
            trail_index = len(trail) - 1
            state = compact_state(row, phase)
            labels = event_labels(last_state, state)
            changed = bool(labels) or (last_state is not None and state != last_state)
            interval = 0.5 if state["driving"] else 2.0
            if not samples or changed or stamp - last_sample_t >= interval:
                field_row = nearest_field(field_rows, field_times, stamp)
                sample = augment_sample(make_sample(row, field_row, point, phase), row, field_row, trail_index)
                samples.append(sample)
                last_sample_t = stamp
                for category, label in labels:
                    events.append({"sample": len(samples) - 1, "category": category, "label": label})
            last_state = state
            last_row = row
            pending = (row, phase, point, trail_index)

    if not samples or last_row is None:
        raise ValueError(f"no pursuit data rows: {PURSUIT}")
    if samples[-1]["t"] != numeric(last_row.get("timestamp")):
        row, phase, point, trail_index = pending
        field_row = nearest_field(field_rows, field_times, numeric(row["timestamp"]))
        samples.append(augment_sample(make_sample(row, field_row, point, phase), row, field_row, trail_index))

    used_sources = [PURSUIT, FIELD, MISSION, AUDIT, TEMPLATE, SHARED_BUILDER, Path(__file__).resolve()]
    source_files = [{"name": path.name, "sha256": digest(path)} for path in used_sources]
    return {
        "title": "September 23, 2026 · Completed Tractor01 automatic mission · HISTORICAL REPLAY",
        "run": RUN.name,
        "mission": mission,
        "trail": trail,
        "samples": samples,
        "events": events,
        "summary": {
            "rawPursuitRows": raw_rows,
            "fieldRows": len(field_rows),
            "missionWaypoints": len(mission),
            "auditRows": len(audit_rows),
            "eventCategories": dict(sorted(Counter(event["category"] for event in events).items())),
        },
        "source": {"files": source_files},
    }


def validate_output(page, payload):
    prefix, suffix = "const D=", ";\nconst samples="
    start = page.index(prefix) + len(prefix)
    end = page.index(suffix, start)
    embedded = json.loads(page[start:end])
    if embedded != payload:
        raise ValueError("embedded replay payload does not round-trip")
    if len(embedded["trail"]) < 1 or len(embedded["mission"]) != len(embedded["summary"]["auditRows"] * [None]):
        raise ValueError("embedded mission/trail validation failed")
    required = {"rtk", "heading", "mode", "radio", "driving", "reacquisition", "phase"}
    present = {event["category"] for event in embedded["events"]}
    missing = required - present
    if missing:
        raise ValueError(f"required event categories absent: {sorted(missing)}")
    for source in embedded["source"]["files"]:
        if len(source["sha256"]) != 64:
            raise ValueError(f"bad embedded SHA-256: {source['name']}")
    for marker in ("Previous event", "Next event", "eventSearch", "Full mission", "commanded speed"):
        if marker not in page:
            raise ValueError(f"HTML control/feature missing: {marker}")


def main():
    payload = build_payload()
    encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    page = TEMPLATE.read_text(encoding="utf-8").replace("__DATA__", encoded)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    OUTPUT_HTML.write_text(page, encoding="utf-8", newline="\n")
    validate_output(page, payload)
    print(
        f"{OUTPUT_HTML} | {len(payload['samples'])} replay samples | "
        f"{len(payload['events'])} detected events | {len(payload['trail'])} trail points | "
        f"{OUTPUT_HTML.stat().st_size:,} bytes"
    )
    print("event categories:", json.dumps(payload["summary"]["eventCategories"], sort_keys=True))


if __name__ == "__main__":
    main()
