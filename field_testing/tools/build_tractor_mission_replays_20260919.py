#!/usr/bin/env python3
"""Build self-contained historical replays of the two 2026-09-18 missions.

Unlike the older pursuit replay, this retains WAIT cycles and safety events.
No external map tiles, scripts, or network requests are used by the HTML files.
"""

from __future__ import annotations

import bisect
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1] / "sites" / "62_Collins_multi_boundary_20260915"
RUNS = ROOT / "runs"
OUTPUT = ROOT / "analysis" / "20260918_mission_replays"
TEMPLATE = Path(__file__).with_name("tractor-mission-replay-template.html")
PACKAGE = "20260915_master_boundary_replay"
SPECS = (
    {
        "run": "20260918_121342",
        "title": "September 18 · Main backyard mission",
        "mission": "generated_rings_only/62_Collins_rings_only_resume_wp0091_20260916.txt",
        "audit": "generated_rings_only/62_Collins_rings_only_resume_wp0091_audit_20260916.csv",
    },
    {
        "run": "20260918_134506",
        "title": "September 18 · Post-backyard continuation",
        "mission": "generated_continuation_20260918/62_Collins_continuation_after_main_backyard_1mps_20260918.txt",
        "audit": "generated_continuation_20260918/62_Collins_continuation_after_main_backyard_1mps_audit_20260918.csv",
    },
)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def numeric(value, default=None):
    try:
        result = float(value)
        return result if math.isfinite(result) else default
    except (TypeError, ValueError):
        return default


def integer(value, default=None):
    result = numeric(value)
    return int(result) if result is not None else default


def boolean(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def mode(value) -> str:
    return {0: "AUTO", 1: "MANUAL", 2: "PAUSE", 9: "RADIO_LOSS"}.get(
        integer(value), "UNKNOWN"
    )


def load_mission(path: Path):
    rows = [list(map(float, line.split())) for line in path.read_text().splitlines() if line.strip()]
    if not rows or any(len(row) != 5 for row in rows):
        raise ValueError(f"invalid five-column mission: {path}")
    lat0, lon0 = rows[0][:2]
    sx = 111_320.0 * math.cos(math.radians(lat0))
    sy = 110_540.0
    points = [[round((r[1] - lon0) * sx, 3), round((r[0] - lat0) * sy, 3)] for r in rows]
    return points, lat0, lon0, sx, sy


def load_field(path: Path):
    result = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            try:
                stamp = dt.datetime.fromisoformat(row["time"]).timestamp()
            except (KeyError, ValueError):
                continue
            result.append((stamp, row))
    return result


def nearest_field(rows, times, stamp):
    index = bisect.bisect_left(times, stamp)
    choices = [i for i in (index - 1, index) if 0 <= i < len(rows)]
    return rows[min(choices, key=lambda i: abs(times[i] - stamp))][1] if choices else {}


def make_sample(row, field, point, phase):
    return {
        "t": numeric(row.get("timestamp")),
        "elapsed": numeric(row.get("elapsed_s")),
        "x": point[0] if point else None,
        "y": point[1] if point else None,
        "lat": numeric(row.get("lat")),
        "lon": numeric(row.get("lon")),
        "heading": numeric(row.get("heading_compass_deg")),
        "idx": integer(row.get("waypoint_idx")),
        "phase": phase,
        "targetX": numeric(row.get("target_x_m")),
        "targetY": numeric(row.get("target_y_m")),
        "cte": numeric(row.get("cross_track_err_m")),
        "steer": numeric(row.get("steer_normalized")),
        "speedCmd": numeric(row.get("speed_cmd_mps")),
        "speed": numeric(row.get("actual_speed_mps")),
        "driving": boolean(row.get("driving")),
        "wait": row.get("wait_reason", ""),
        "pause": boolean(row.get("software_paused")),
        "mode": mode(row.get("handheld_mode")),
        "state": row.get("handheld_state", ""),
        "fix": row.get("fix_quality", ""),
        "headValid": boolean(row.get("head_valid")),
        "carrier": row.get("heading_carrier", ""),
        "reacquire": row.get("reacquire_state", ""),
        "reacquireDetail": row.get("reacquire_detail", ""),
        "jrkTarget": integer(field.get("jrk_target")),
        "jrkFeedback": integer(field.get("jrk_scaled_feedback")),
        "ackRate": numeric(field.get("ack_rate")),
        "transMode": mode(field.get("trans_mode")),
        "steerMode": mode(field.get("steer_mode")),
        "steerPwm": integer(field.get("steer_pwm")),
        "fieldAge": round(abs(numeric(row.get("timestamp"), 0) -
                              dt.datetime.fromisoformat(field["time"]).timestamp()), 3)
        if field.get("time") else None,
    }


def build(spec, template):
    run_dir = RUNS / spec["run"]
    summary = json.loads((run_dir / "collection_summary.json").read_text(encoding="utf-8-sig"))
    pursuit = run_dir / summary["pursuit_log"]
    field_path = run_dir / summary["field_log"]
    mission_path = run_dir / PACKAGE / spec["mission"]
    audit_path = run_dir / PACKAGE / spec["audit"]
    for path, expected in ((pursuit, summary["pursuit_sha256"]),
                           (field_path, summary.get("field_sha256", summary.get("field_log_sha256")))):
        if digest(path) != expected.upper():
            raise ValueError(f"source checksum mismatch: {path}")

    mission, lat0, lon0, sx, sy = load_mission(mission_path)
    with audit_path.open(newline="", encoding="utf-8-sig") as handle:
        phases = [row.get("phase", "") for row in csv.DictReader(handle)]
    if len(phases) != len(mission):
        raise ValueError("mission and audit row counts differ")

    field = load_field(field_path)
    times = [t for t, _ in field]
    samples = []
    events = []
    last_sample_t = -math.inf
    last_state = None
    last_row = None
    pending = None
    with pursuit.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            stamp = numeric(row.get("timestamp"))
            if stamp is None:
                continue  # CSV units row
            idx = integer(row.get("waypoint_idx"), 0)
            idx = max(0, min(idx, len(phases) - 1))
            phase = phases[idx]
            state = (
                boolean(row.get("driving")), mode(row.get("handheld_mode")),
                row.get("fix_quality", ""), boolean(row.get("head_valid")),
                boolean(row.get("software_paused")), row.get("speed_cmd_mps", ""),
                row.get("reacquire_state", ""), phase,
            )
            changed = last_state is not None and state != last_state
            interval = 0.5 if state[0] else 2.0
            should_take = not samples or changed or stamp - last_sample_t >= interval
            if should_take:
                field_row = nearest_field(field, times, stamp)
                lat, lon = numeric(row.get("lat")), numeric(row.get("lon"))
                point = [round((lon - lon0) * sx, 3), round((lat - lat0) * sy, 3)] \
                    if lat is not None and lon is not None else None
                sample = make_sample(row, field_row, point, phase)
                samples.append(sample)
                last_sample_t = stamp
                if changed:
                    labels = []
                    if state[1] != last_state[1]: labels.append("Radio " + state[1])
                    if state[2] != last_state[2]: labels.append("GPS " + (state[2] or "unavailable"))
                    if state[3] != last_state[3]: labels.append("Heading " + ("valid" if state[3] else "invalid"))
                    if state[4] != last_state[4]: labels.append("Dashboard " + ("paused" if state[4] else "unpaused"))
                    if state[5] != last_state[5] and state[5]: labels.append("Command " + state[5] + " m/s")
                    if state[0] != last_state[0]: labels.append("Driving" if state[0] else "Waiting")
                    if labels:
                        events.append({"sample": len(samples) - 1, "label": ", ".join(labels)})
            last_state = state
            last_row = row
            pending = (row, phase)
    if not samples or last_row is None:
        raise ValueError(f"no pursuit rows: {pursuit}")
    if samples[-1]["t"] != numeric(last_row["timestamp"]):
        row, phase = pending
        lat, lon = numeric(row.get("lat")), numeric(row.get("lon"))
        point = [round((lon - lon0) * sx, 3), round((lat - lat0) * sy, 3)] \
            if lat is not None and lon is not None else None
        samples.append(make_sample(row, nearest_field(field, times, numeric(row["timestamp"])), point, phase))

    payload = {
        "title": spec["title"], "run": spec["run"], "mission": mission,
        "samples": samples, "events": events,
        "source": {"pursuit": pursuit.name, "pursuitSha256": digest(pursuit),
                   "field": field_path.name, "fieldSha256": digest(field_path),
                   "mission": mission_path.name, "missionSha256": digest(mission_path)},
    }
    OUTPUT.mkdir(parents=True, exist_ok=True)
    output = OUTPUT / f"replay_{spec['run']}.html"
    encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    page = template.replace("__DATA__", encoded)
    output.write_text(page, encoding="utf-8", newline="\n")
    print(f"{output} | {len(samples)} samples | {len(events)} events | {output.stat().st_size:,} bytes")


def main():
    template = TEMPLATE.read_text(encoding="utf-8")
    for spec in SPECS:
        build(spec, template)


if __name__ == "__main__":
    main()
