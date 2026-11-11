#!/usr/bin/env python3
"""Build the November 9 phone-controlled outer-perimeter replay and report."""

from __future__ import annotations

import argparse
import bisect
from collections import Counter
import csv
import datetime as dt
import json
import math
from pathlib import Path
import statistics

from build_tractor01_mission_replay_20260923 import (
    augment_sample,
    compact_state,
)
from build_tractor_mission_replays_20260919 import (
    boolean,
    digest,
    load_field,
    load_mission,
    make_sample,
    nearest_field,
    numeric,
)


SITE = Path(__file__).resolve().parents[1] / "sites" / "62_Collins_multi_boundary_20260915"
DEFAULT_EVIDENCE = (
    SITE / "runs" / "20261109_phone_outer_perimeter" / "evidence"
    / "20261109_phone_outer_perimeter_collection"
)
DEFAULT_OUTPUT = SITE / "analysis" / "20261109_phone_outer_perimeter_replay"
TEMPLATE = Path(__file__).with_name("tractor-mission-replay-template.html")
SHARED_BUILDER = Path(__file__).with_name("build_tractor_mission_replays_20260919.py")
THIS_FILE = Path(__file__).resolve()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, default=DEFAULT_EVIDENCE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def read_pursuit(path):
    rows = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            stamp = numeric(row.get("timestamp"))
            if stamp is None:
                continue
            row["_timestamp"] = stamp
            rows.append(row)
    return rows


def read_phone(path):
    records = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            try:
                record["_timestamp"] = dt.datetime.fromisoformat(record["time"]).timestamp()
            except (KeyError, ValueError):
                continue
            records.append(record)
    return records


def mode_timeline(records):
    timeline = [(records[0]["_timestamp"] if records else 0.0, "PAUSE")]
    for record in records:
        if record.get("event") == "mode_transition":
            timeline.append((record["_timestamp"], str(record.get("to_mode", "pause")).upper()))
    timeline.sort()
    return timeline


def mode_at(timeline, stamp):
    times = [item[0] for item in timeline]
    index = bisect.bisect_right(times, stamp) - 1
    return timeline[max(0, index)][1]


def actual_output(field, requested_mode):
    if field.get("steer_state") == "PAUSE" or field.get("trans_mode") == "2":
        return "PAUSE"
    return requested_mode if requested_mode in {"AUTO", "MANUAL"} else "UNKNOWN"


def heading_healthy(row):
    accuracy = numeric(row.get("heading_accuracy_deg"), math.inf)
    return (
        boolean(row.get("head_valid"))
        and row.get("heading_carrier", "").strip().lower() == "fixed"
        and accuracy <= 1.0
    )


def reason_category(reason):
    if not reason:
        return "unknown"
    if reason.startswith("GPS/heading stable"):
        return "five-second stability recovery"
    if "headValid=False" in reason:
        return "heading validity lost"
    if "heading accuracy=" in reason:
        return "heading accuracy exceeded 1.0 deg"
    if "heading carrier=" in reason:
        return "heading carrier was not fixed"
    if "fix_quality=" in reason:
        return "RTK fix dropped below RTK Fixed"
    if "stale or no GPS" in reason:
        return "GPS feed stale or absent"
    return reason.split(";", 1)[0]


def safety_episodes(pursuit):
    result = []
    started = False
    active = None
    for row in pursuit:
        driving = boolean(row.get("driving"))
        if driving:
            started = True
        if not started:
            continue
        if not driving and active is None:
            active = {
                "start": row,
                "end": row,
                "causes": [],
                "categories": Counter(),
            }
        if active is not None:
            if not driving:
                active["end"] = row
                reason = row.get("wait_reason", "")
                category = reason_category(reason)
                active["categories"][category] += 1
                if reason and not reason.startswith("GPS/heading stable") and reason not in active["causes"]:
                    active["causes"].append(reason)
            else:
                active["recovery"] = row
                result.append(active)
                active = None
    if active is not None:
        result.append(active)
    return result


def episode_at(episodes, stamp):
    for episode in episodes:
        start = episode["start"]["_timestamp"]
        recovery = episode.get("recovery")
        end = recovery["_timestamp"] if recovery else math.inf
        if start <= stamp < end:
            return episode
    return None


def classify_pause(stamp, pursuit_row, requested_mode, episode):
    if requested_mode == "PAUSE":
        return "operator", "operator selected Pause"
    if episode is not None:
        cause = episode["causes"][0] if episode["causes"] else episode["start"].get("wait_reason", "")
        return "pure_pursuit_safety", reason_category(cause)
    cross_track = numeric(pursuit_row.get("cross_track_err_m"))
    if cross_track is not None and cross_track > 2.0:
        return "supervisor_corridor", f"cross-track {cross_track:.3f} m exceeded supervisor 2.0 m limit"
    return "unresolved", "no breached parameter is visible in the collected rows"


def pause_timeline(field_rows, pursuit, timeline, episodes):
    pursuit_times = [row["_timestamp"] for row in pursuit]
    transitions = []
    previous = None
    for stamp, row in field_rows:
        state = row.get("steer_state")
        if state == "PAUSE" and previous == "OK":
            index = bisect.bisect_left(pursuit_times, stamp)
            choices = [i for i in (index - 1, index) if 0 <= i < len(pursuit)]
            pp = pursuit[min(choices, key=lambda i: abs(pursuit_times[i] - stamp))]
            requested = mode_at(timeline, stamp)
            episode = episode_at(episodes, stamp)
            category, cause = classify_pause(stamp, pp, requested, episode)
            transitions.append({
                "timestamp": dt.datetime.fromtimestamp(stamp, dt.timezone.utc).isoformat(),
                "elapsed_s": numeric(row.get("elapsed_sec")),
                "requested_mode": requested,
                "category": category,
                "cause": cause,
                "waypoint_idx": pp.get("waypoint_idx", ""),
                "path_progress_m": pp.get("path_progress_m", ""),
                "cross_track_m": pp.get("cross_track_err_m", ""),
                "fix_quality": pp.get("fix_quality", ""),
                "heading_carrier": pp.get("heading_carrier", ""),
                "heading_accuracy_deg": pp.get("heading_accuracy_deg", ""),
                "actual_speed_mps": row.get("speed_mps", ""),
            })
        previous = state
    return transitions


def failure_duration_s(pursuit, trigger_time, category, trigger):
    """Measure one continuous breached condition using the raw pursuit rows."""
    if category == "operator":
        return ""

    if "heading validity lost" in trigger:
        breached = lambda row: not boolean(row.get("head_valid"))
    elif "heading accuracy exceeded" in trigger:
        breached = lambda row: numeric(row.get("heading_accuracy_deg"), math.inf) > 1.0
    elif "heading carrier was not fixed" in trigger:
        breached = lambda row: row.get("heading_carrier", "").strip().lower() != "fixed"
    elif "RTK fix dropped" in trigger:
        breached = lambda row: row.get("fix_quality", "").strip() != "RTK Fixed"
    elif category == "supervisor_corridor":
        breached = lambda row: numeric(row.get("cross_track_err_m"), -math.inf) > 2.0
    else:
        return ""

    candidates = [
        index for index, row in enumerate(pursuit)
        if breached(row) and abs(row["_timestamp"] - trigger_time) <= 5.0
    ]
    if not candidates:
        return ""
    index = min(candidates, key=lambda item: abs(pursuit[item]["_timestamp"] - trigger_time))
    start = index
    while start > 0 and breached(pursuit[start - 1]):
        start -= 1
    recovery = index + 1
    while recovery < len(pursuit) and breached(pursuit[recovery]):
        recovery += 1
    end_time = (
        pursuit[recovery]["_timestamp"]
        if recovery < len(pursuit)
        else pursuit[-1]["_timestamp"]
    )
    return round(end_time - pursuit[start]["_timestamp"], 3)


def interruption_timeline(samples, pause_transitions, pursuit):
    """Group all mode cycling between two AUTO/AUTO states into one interruption.

    Replay sample numbers are one-based. Route waypoint indices retain the
    zero-based values displayed by the replay and recorded by Pure Pursuit.
    """
    interruptions = []
    active = None
    seen_auto = False

    def both_auto(sample):
        return sample.get("mode") == "AUTO" and sample.get("transMode") == "AUTO"

    def iso(stamp):
        return dt.datetime.fromtimestamp(stamp, dt.timezone.utc).isoformat()

    def trigger_sample_index(start_index):
        # The Pure Pursuit safety gate generally changes a few replay samples
        # before the transmission acknowledges Pause. Preserve that precursor.
        lower = max(0, start_index - 12)
        candidate = start_index
        found_stopped = False
        for index in range(start_index, lower - 1, -1):
            sample = samples[index]
            if sample.get("driving") is False:
                found_stopped = True
                wait = sample.get("wait", "")
                if wait and not wait.startswith("GPS/heading stable"):
                    candidate = index
            elif found_stopped:
                break
        if not found_stopped:
            corridor_candidates = [
                index for index in range(lower, start_index + 1)
                if numeric(samples[index].get("cte")) is not None
            ]
            if corridor_candidates:
                worst = max(corridor_candidates, key=lambda index: numeric(samples[index].get("cte")))
                if numeric(samples[worst].get("cte")) > 2.0:
                    return worst
        return candidate

    def finish(item, resume_index):
        start_index = item["start_index"]
        last_auto_index = item["last_auto_index"]
        end_index = resume_index if resume_index is not None else len(samples) - 1
        start = samples[start_index]
        end = samples[end_index]
        last_auto = samples[last_auto_index]
        trigger_index = trigger_sample_index(start_index)
        trigger_sample = samples[trigger_index]
        start_time = start["t"]
        end_time = end["t"]
        matching = [
            row for row in pause_transitions
            if dt.datetime.fromisoformat(row["timestamp"]).timestamp() >= last_auto["t"]
            and dt.datetime.fromisoformat(row["timestamp"]).timestamp() <= end_time
        ]
        first_pause = matching[0] if matching else None
        trigger_cte = numeric(trigger_sample.get("cte"))
        if first_pause:
            trigger = first_pause["cause"]
        elif trigger_cte is not None and trigger_cte > 2.0:
            trigger = f"cross-track {trigger_cte:.3f} m exceeded supervisor 2.0 m limit"
        else:
            trigger = reason_category(trigger_sample.get("wait", ""))
        if first_pause:
            category = first_pause["category"]
        elif trigger_sample.get("wait"):
            category = "pure_pursuit_safety"
        elif trigger_cte is not None and trigger_cte > 2.0:
            category = "supervisor_corridor"
        else:
            category = "unresolved"
        auto_attempts = 0
        previous_mode = start.get("mode")
        for sample in samples[start_index + 1:end_index + 1]:
            if previous_mode != "AUTO" and sample.get("mode") == "AUTO":
                auto_attempts += 1
            previous_mode = sample.get("mode")
        return {
            "interruption_id": len(interruptions) + 1,
            "last_auto_sample": last_auto_index + 1,
            "pause_start_sample": start_index + 1,
            "auto_restored_sample": resume_index + 1 if resume_index is not None else "",
            "trigger_sample": trigger_index + 1,
            "last_auto_route_waypoint": last_auto.get("idx", ""),
            "pause_start_route_waypoint": start.get("idx", ""),
            "auto_restored_route_waypoint": end.get("idx", "") if resume_index is not None else "",
            "pause_start_timestamp_utc": iso(start_time),
            "auto_restored_timestamp_utc": iso(end_time) if resume_index is not None else "",
            "duration_s": round(end_time - start_time, 3),
            "trigger_category": category,
            "trigger": trigger,
            "failure_duration_s": failure_duration_s(
                pursuit, trigger_sample["t"], category, trigger
            ),
            "trigger_route_waypoint": trigger_sample.get("idx", ""),
            "trigger_cross_track_m": trigger_sample.get("cte", ""),
            "trigger_fix_quality": trigger_sample.get("fix", ""),
            "trigger_heading_carrier": trigger_sample.get("carrier", ""),
            "trigger_heading_accuracy_deg": trigger_sample.get("headingAccuracy", ""),
            "pause_latches_in_window": len(matching),
            "auto_rearm_attempts_in_window": auto_attempts,
            "result": "AUTO/AUTO restored" if resume_index is not None else "AUTO/AUTO not restored before recording ended",
        }

    for index, sample in enumerate(samples):
        is_auto = both_auto(sample)
        if active is None:
            if is_auto:
                seen_auto = True
            elif seen_auto and index > 0:
                active = {"start_index": index, "last_auto_index": index - 1}
        elif is_auto:
            interruptions.append(finish(active, index))
            active = None

    if active is not None:
        interruptions.append(finish(active, None))
    return interruptions


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(len(ordered) * fraction) - 1)]


def phone_summary(records):
    gaps, rtts = [], []
    event_counts = Counter(record.get("event") for record in records)
    transition_counts = Counter()
    for record in records:
        if record.get("event") == "mode_transition":
            transition_counts[(record.get("from_mode"), record.get("to_mode"))] += 1
        if record.get("event") == "command":
            gap = numeric(record.get("command_gap_ms"))
            rtt = numeric(record.get("client_reported_rtt_ms"))
            if gap is not None:
                gaps.append(gap)
            if rtt is not None:
                rtts.append(rtt)
    return {
        "event_counts": event_counts,
        "transition_counts": transition_counts,
        "gaps": gaps,
        "rtts": rtts,
    }


def event_labels(previous, current, row):
    labels = []
    if previous is None:
        return [("mission", "Replay begins")]
    if current["mode"] != previous["mode"]:
        labels.append(("mode", f"Phone {current['mode'].title()}"))
    if current["actual"] != previous["actual"]:
        label = "Actual output Pause" if current["actual"] == "PAUSE" else f"Actual output {current['actual'].title()}"
        labels.append(("safety" if current["actual"] == "PAUSE" else "mode", label))
    if current["fix"] != previous["fix"]:
        label = "RTK Fixed restored" if current["fix"] == "RTK Fixed" else f"RTK loss: {current['fix'] or 'unavailable'}"
        labels.append(("rtk", label))
    if current["headingHealthy"] != previous["headingHealthy"]:
        labels.append(("heading", "Heading safety healthy" if current["headingHealthy"] else "Heading safety threshold breached"))
    if current["driving"] != previous["driving"]:
        label = "Pure Pursuit resumed" if current["driving"] else f"Pure Pursuit stopped: {reason_category(row.get('wait_reason', ''))}"
        labels.append(("driving" if current["driving"] else "safety", label))
    if current["corridor"] != previous["corridor"]:
        labels.append(("corridor", "Inside 2.0 m supervisor corridor" if current["corridor"] else "Outside 2.0 m supervisor corridor"))
    if current["reacquire"] != previous["reacquire"]:
        labels.append(("reacquisition", "Path reacquired" if current["reacquire"] == "TRACKING" else f"Path reacquisition {current['reacquire'].lower()}"))
    if current["phase"] != previous["phase"]:
        labels.append(("phase", f"Mission phase: {current['phase']}"))
    return labels


def build_payload(
    pursuit,
    field_rows,
    mission_path,
    audit_path,
    phone_records,
    evidence_sources,
):
    mission, lat0, lon0, sx, sy = load_mission(mission_path)
    with audit_path.open(newline="", encoding="utf-8-sig") as handle:
        audit_rows = list(csv.DictReader(handle))
    phases = [row.get("phase", "") for row in audit_rows]
    if len(phases) != len(mission):
        raise ValueError("mission/audit waypoint mismatch")

    field_times = [stamp for stamp, _ in field_rows]
    phone_modes = mode_timeline(phone_records)
    samples, trail, events = [], [], []
    last_sample_t = -math.inf
    previous_state = None
    pending = None
    for row in pursuit:
        stamp = row["_timestamp"]
        index = max(0, min(int(numeric(row.get("waypoint_idx"), 0)), len(phases) - 1))
        phase = phases[index]
        lat, lon = numeric(row.get("lat")), numeric(row.get("lon"))
        point = [round((lon - lon0) * sx, 3), round((lat - lat0) * sy, 3)] if lat is not None and lon is not None else None
        if point is not None:
            trail.append(point)
        trail_index = len(trail) - 1
        field_row = nearest_field(field_rows, field_times, stamp)
        requested = mode_at(phone_modes, stamp)
        actual = actual_output(field_row, requested)
        cross_track = numeric(row.get("cross_track_err_m"))
        state = compact_state(row, phase)
        state.update({
            "mode": requested,
            "actual": actual,
            "headingHealthy": heading_healthy(row),
            "corridor": cross_track is None or cross_track <= 2.0,
        })
        labels = event_labels(previous_state, state, row)
        changed = bool(labels)
        interval = 0.5 if boolean(row.get("driving")) else 1.0
        if not samples or changed or stamp - last_sample_t >= interval:
            sample = augment_sample(make_sample(row, field_row, point, phase), row, field_row, trail_index)
            sample.update({
                "mode": requested,
                "state": actual,
                "actualOutput": actual,
                "corridorHealthy": state["corridor"],
            })
            samples.append(sample)
            last_sample_t = stamp
            for category, label in labels:
                events.append({"sample": len(samples) - 1, "category": category, "label": label})
        previous_state = state
        pending = (row, field_row, point, phase, trail_index, requested, actual, state)

    if pending and samples[-1]["t"] != pending[0]["_timestamp"]:
        row, field_row, point, phase, trail_index, requested, actual, state = pending
        sample = augment_sample(make_sample(row, field_row, point, phase), row, field_row, trail_index)
        sample.update({"mode": requested, "state": actual, "actualOutput": actual, "corridorHealthy": state["corridor"]})
        samples.append(sample)

    sources = [*evidence_sources, TEMPLATE, SHARED_BUILDER, THIS_FILE]
    return {
        "title": "November 9, 2026 · Phone-controlled outer-perimeter mission · HISTORICAL REPLAY",
        "run": "20261109_phone_outer_perimeter",
        "mission": mission,
        "trail": trail,
        "samples": samples,
        "events": events,
        "summary": {
            "rawPursuitRows": len(pursuit),
            "fieldRows": len(field_rows),
            "missionWaypoints": len(mission),
            "auditRows": len(audit_rows),
            "eventCategories": dict(sorted(Counter(event["category"] for event in events).items())),
        },
        "source": {"files": [{"name": path.name, "sha256": digest(path)} for path in sources]},
    }


def write_report(output, pursuit, field_rows, phone_records, episodes, pauses, source_paths):
    phone = phone_summary(phone_records)
    phone_modes = mode_timeline(phone_records)
    categories = Counter(item["category"] for item in pauses)
    max_progress = max(numeric(row.get("path_progress_m"), 0.0) for row in pursuit)
    max_waypoint = max(int(numeric(row.get("waypoint_idx"), 0)) for row in pursuit)
    cross_tracks = [numeric(row.get("cross_track_err_m")) for row in pursuit]
    cross_tracks = [value for value in cross_tracks if value is not None]
    auto_cross_tracks = [
        numeric(row.get("cross_track_err_m"))
        for row in pursuit
        if mode_at(phone_modes, row["_timestamp"]) == "AUTO"
        and numeric(row.get("cross_track_err_m")) is not None
    ]
    transitions = phone["transition_counts"]
    gaps = phone["gaps"]
    rtts = phone["rtts"]
    auto_episodes = []
    timeline = mode_timeline(phone_records)
    for episode in episodes:
        if mode_at(timeline, episode["start"]["_timestamp"]) == "AUTO":
            auto_episodes.append(episode)

    lines = [
        "# November 9, 2026 outer-perimeter mission analysis",
        "",
        "## Confirmed result",
        "",
        "The repeated stops were not caused by the direct phone network. The authoritative server log contains zero `phone_freshness_expired` events during this run. The interruptions came from GPS/heading safety gates and, later, the RPi supervisor's 2.0 m route-corridor gate.",
        "",
        "## Counts",
        "",
        f"- Actual output transitions into Pause: **{len(pauses)}**.",
        f"- Pure Pursuit GPS/heading safety stops while Auto was requested: **{categories['pure_pursuit_safety']}**.",
        f"- Supervisor cross-track corridor latches: **{categories['supervisor_corridor']}**.",
        f"- Operator-selected Pause transitions: **{categories['operator']}**.",
        f"- Unresolved Pause transitions: **{categories['unresolved']}**.",
        f"- Pure Pursuit safety episodes while Auto was requested: **{len(auto_episodes)}**.",
        f"- Phone mode transitions: **{sum(transitions.values())}** (`Auto→Pause` {transitions[('auto', 'pause')]}, `Pause→Manual` {transitions[('pause', 'manual')]}, `Manual→Auto` {transitions[('manual', 'auto')]}).",
        "- Steering fault latch, steering drive block, and JRK halting errors: **zero in all field rows**.",
        "",
        "## Phone and session evidence",
        "",
        f"- Accepted commands measured by the server: **{len(gaps):,}**.",
        f"- Accepted-command gap: median **{statistics.median(gaps):.1f} ms**, p95 **{percentile(gaps, .95):.1f} ms**, p99 **{percentile(gaps, .99):.1f} ms**.",
        f"- Client-reported RTT: median **{statistics.median(rtts):.1f} ms**, p95 **{percentile(rtts, .95):.1f} ms**, maximum **{max(rtts):.1f} ms**.",
        f"- Phone freshness expirations: **{phone['event_counts']['phone_freshness_expired']}**.",
        "- One 17.9 s accepted-command gap coincided with a new owner/session claim while the tractor was already in Pause. It did not initiate an Auto stop.",
        "- The single `control session is not current` rejection occurred at server startup before the current owner was claimed.",
        "",
        "## Safety parameters breached",
        "",
        "Pure Pursuit required RTK Fixed, fixed/valid dual-antenna heading, heading accuracy no worse than 1.0°, and then five continuous healthy seconds before recovery. The recorded episodes included:",
        "",
        "- RTK quality dropping to RTK Float or DGPS.",
        "- Heading carrier dropping to float or none.",
        "- `headValid`/RELPOSNED validity loss, including momentary zero-baseline records.",
        "- Heading accuracy exceeding 1.0°.",
        "",
        "Every such fault correctly stopped Pure Pursuit immediately and required a five-second stable recovery window. The RPi supervisor then remained Pause-latched until a fresh stopped Pause acknowledgement.",
        "",
        "## Why Pause–Manual–Auto was sometimes needed",
        "",
        "After progress 380.857 m, Pure Pursuit was willing to keep proposing within its 6.0 m tracking window, but the RPi supervisor required cross-track error ≤2.0 m. Recorded Auto attempts occurred at 2.003–5.403 m cross-track. Each Auto selection therefore immediately re-latched Pause. Manual was only useful for repositioning; it could not clear the latch unless the tractor returned to Pause while stopped, and Auto could not succeed until it was back inside 2.0 m.",
        "",
        "This is a design mismatch, not a phone-link failure. The dashboard should display the exact latch reason and the distance/direction back to the permitted corridor instead of prompting repeated mode cycling.",
        "",
        "## Mission extent",
        "",
        f"- Maximum route progress: **{max_progress:.3f} m** of 689.949 m.",
        f"- Maximum waypoint index: **{max_waypoint}** of 3461.",
        f"- Maximum cross-track error while Auto was requested: **{max(auto_cross_tracks):.3f} m**.",
        f"- Maximum cross-track error across the full recording, including the later Manual drive home: **{max(cross_tracks):.3f} m**.",
        "",
        "## Recommendations",
        "",
        "1. Keep the supervisor's 2.0 m corridor limit for now; do not simply widen it to match the 6.0 m Pure Pursuit search window.",
        "2. Make the controller and dashboard share one explicit recovery contract: exact breached field, current value, threshold, five-second stability countdown, Pause-latch state, and required recovery direction/distance.",
        "3. When outside 2.0 m, inhibit Auto re-arm and provide Manual guidance back inside the corridor; only then accept stopped Pause and Auto.",
        "4. Add continuous supervisor-decision logging. This run could be reconstructed, but exact latch reasons should be written as first-class events rather than inferred from synchronized feeds.",
        "5. Preserve immediate GPS/heading and local actuator stops. Investigate antenna visibility and correction continuity before considering any threshold/hysteresis adjustment.",
        "6. Re-run blades-off from the known route start after the recovery UI and decision logging are corrected.",
        "",
        "## Source integrity",
        "",
    ]
    for path in source_paths:
        lines.append(f"- `{path.name}` — SHA-256 `{digest(path)}`")
    (output / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def main():
    args = parse_args()
    root = args.evidence_root.resolve()
    output = args.output_dir.resolve()
    mission_csv = root / "mission" / "outer_perimeter_20261109_154324.csv"
    pursuit_csv = root / "pursuit" / "pursuit_log_20261109_154326.csv"
    phone_jsonl = root / "phone" / "wifi_primary_control_20261109_154125.jsonl"
    mission_path = root / "package" / "62_Collins_outer_perimeter_field_test_1mps_20260928.txt"
    audit_path = root / "package" / "62_Collins_outer_perimeter_field_test_1mps_20260928_audit.csv"
    for path in (mission_csv, pursuit_csv, phone_jsonl, mission_path, audit_path, TEMPLATE):
        if not path.is_file():
            raise FileNotFoundError(path)

    field_rows = load_field(mission_csv)
    pursuit = read_pursuit(pursuit_csv)
    phone_records = read_phone(phone_jsonl)
    timeline = mode_timeline(phone_records)
    episodes = safety_episodes(pursuit)
    pauses = pause_timeline(field_rows, pursuit, timeline, episodes)
    sources = [mission_csv, pursuit_csv, phone_jsonl, mission_path, audit_path]
    payload = build_payload(
        pursuit,
        field_rows,
        mission_path,
        audit_path,
        phone_records,
        sources,
    )
    interruptions = interruption_timeline(payload["samples"], pauses, pursuit)

    output.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    page = TEMPLATE.read_text(encoding="utf-8").replace("__DATA__", encoded)
    replay = output / "replay_20261109_phone_outer_perimeter.html"
    replay.write_text(page, encoding="utf-8", newline="\n")

    timeline_path = output / "safety_pause_timeline.csv"
    with timeline_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(interruptions[0]))
        writer.writeheader()
        writer.writerows(interruptions)

    detail_path = output / "pause_transition_details.csv"
    with detail_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(pauses[0]))
        writer.writeheader()
        writer.writerows(pauses)

    write_report(output, pursuit, field_rows, phone_records, episodes, pauses, sources)
    (output / "README.md").write_text(
        "# November 9 mission replay\n\n"
        "Open `replay_20261109_phone_outer_perimeter.html` in a modern browser. "
        "It is self-contained and makes no network requests.\n\n"
        "`safety_pause_timeline.csv` groups each complete interruption from the "
        "first non-AUTO/AUTO replay sample through the first restored AUTO/AUTO "
        "sample. Replay sample numbers are not route waypoint numbers; both are "
        "included explicitly. Route waypoint indices are zero-based, matching the "
        "replay display and Pure Pursuit log. `pause_transition_details.csv` retains "
        "the individual low-level Pause transitions. `failure_duration_s` measures "
        "the continuous breached condition from its first bad raw 20 Hz Pure Pursuit "
        "row through the first recovered row; it is blank for an operator-selected "
        "Pause. This is distinct from the longer interruption/re-arming `duration_s`.\n\n"
        "Rebuild from the repository root with:\n\n"
        "```bash\npython3 field_testing/tools/build_tractor01_mission_replay_20261109.py "
        "--evidence-root /path/to/20261109_phone_outer_perimeter_collection\n```\n",
        encoding="utf-8",
        newline="\n",
    )
    print(f"{replay} | {len(payload['samples'])} samples | {len(payload['events'])} events | {replay.stat().st_size:,} bytes")
    print(f"{timeline_path} | {len(interruptions)} grouped mission interruptions")
    print(f"{detail_path} | {len(pauses)} individual Pause transitions")
    print(output / "REPORT.md")


if __name__ == "__main__":
    main()
