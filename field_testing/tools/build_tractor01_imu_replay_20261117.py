#!/usr/bin/env python3
"""Offline, observation-only IMU/GNSS replay using the current mission replay lineage."""
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

from build_tractor_mission_replays_20260919 import boolean, digest, numeric

HERE = Path(__file__).resolve().parent
TEMPLATE = HERE / "tractor-imu-replay-template.html"
SHARED_TEMPLATE = HERE / "tractor-mission-replay-template.html"
LATEST_BUILDER = HERE / "build_tractor01_mission_replay_20261109.py"
SIGNALS = {"gz": "gyro_z_corrected_dps", "gx": "gyro_x_corrected_dps",
           "gy": "gyro_y_corrected_dps", "ax": "accel_x_g", "ay": "accel_y_g", "az": "accel_z_g"}


def epoch(value):
    stamp = dt.datetime.fromisoformat(value.replace("Z", "+00:00"))
    if stamp.tzinfo is None:
        raise ValueError("Timestamp must carry a timezone")
    return stamp.timestamp()


def verify_capture(root):
    manifest = root / "SHA256SUMS"
    entries = {}
    for line in manifest.read_text(encoding="utf-8").splitlines():
        expected, name = line.split("  ", 1)
        if name not in {"samples.csv", "gnss.jsonl", "metadata.json"} or name in entries:
            raise ValueError(f"Unexpected or duplicate manifest entry: {name}")
        path = root / name
        if digest(path).lower() != expected.lower():
            raise ValueError(f"Checksum mismatch: {name}")
        entries[name] = expected.lower()
    if set(entries) != {"samples.csv", "gnss.jsonl", "metadata.json"}:
        raise ValueError("Incomplete capture checksum manifest")
    return entries


def health(row):
    packet = numeric(row.get("gnss_packet_age_s"))
    heading_age = numeric(row.get("gnss_heading_age_s"))
    fresh = packet is not None and 0 <= packet <= 1 and boolean(row.get("gnss_socket_available"))
    fixed = row.get("gnss_fix_quality") == "RTK Fixed"
    accuracy = numeric(row.get("gnss_relpos_heading_accuracy_deg"))
    baseline = numeric(row.get("gnss_relpos_length_m"))
    heading = (boolean(row.get("gnss_headValid")) and row.get("gnss_carrier") == "fixed"
               and heading_age is not None and -0.1 <= heading_age <= 1
               and accuracy is not None and 0 <= accuracy <= 1
               and boolean(row.get("gnss_relpos_valid"))
               and boolean(row.get("gnss_relpos_gnss_fix_ok"))
               and str(row.get("gnss_relpos_ref_pos_miss")).lower() == "false"
               and str(row.get("gnss_relpos_ref_obs_miss")).lower() == "false"
               and str(row.get("gnss_fatal_error")).lower() == "false")
    base_ok = baseline is not None and 0.8 <= baseline <= 1.3
    issues = []
    for good, name in ((fresh, "GPS packet stale/missing"), (fixed, "Position below RTK Fixed"),
                       (heading, "Heading quality/freshness failed"), (base_ok, "Baseline outside 0.80–1.30 m")):
        if not good:
            issues.append(name)
    code = "stale" if not fresh else "position" if not fixed else "heading" if not heading or not base_ok else "good"
    return {"fresh": fresh, "fixed": fixed, "heading": heading, "baselineOk": base_ok,
            "code": code, "issues": issues}


def compact(row, origin, start_mono):
    number = lambda key: numeric(row.get(key))
    lat, lon = number("gnss_lat"), number("gnss_lon")
    valid_position = lat is not None and lon is not None and -90 <= lat <= 90 and -180 <= lon <= 180
    sample = {
        "t": number("monotonic_s") - start_mono, "epoch": epoch(row["utc"]),
        "raw": int(row["sample"]), "phase": row["phase"], "dt": number("dt_s"),
        "segment": number("yaw_segment"), "yaw": number("relative_yaw_ccw_deg"),
        "compass": number("imu_compass_once_anchored_deg"), "error": number("imu_minus_gnss_deg"),
        "heading": number("gnss_heading_deg"), "speed": number("gnss_speed_mps"),
        "lat": lat, "lon": lon, "packetAge": number("gnss_packet_age_s"),
        "headingAge": number("gnss_heading_age_s"), "accuracy": number("gnss_relpos_heading_accuracy_deg"),
        "baseline": number("gnss_relpos_length_m"), "fix": row.get("gnss_fix_quality"),
        "carrier": row.get("gnss_carrier"), "headValid": boolean(row.get("gnss_headValid")),
        "loggerUsable": boolean(row.get("gnss_heading_usable")), "health": health(row),
        "x": (lon-origin[1])*origin[2] if valid_position else None,
        "y": (lat-origin[0])*110540.0 if valid_position else None,
    }
    sample.update({name: number(column) for name, column in SIGNALS.items()})
    return sample


def build_data(root, notes, interval=0.1):
    verified = verify_capture(root)
    metadata = json.loads((root / "metadata.json").read_text())
    samples, events, episodes, gaps = [], [], [], []
    states = ("fresh", "fixed", "heading", "baselineOk")
    names = {"fresh": "GPS packets stale/missing", "fixed": "Position below RTK Fixed",
             "heading": "Heading quality/freshness failed", "baselineOk": "Baseline outside 0.80–1.30 m"}
    active, previous, bucket = {}, None, None
    start_mono = previous_mono = None
    start_epoch = None
    origin = None
    counts = Counter()
    deltas = []
    envelope, pending = {}, None
    raw_count = 0

    def flush():
        if pending is not None:
            item = dict(pending)
            item["range"] = {key: [min(values), max(values)] for key, values in envelope.items() if values}
            samples.append(item)

    def transition(sample):
        nonlocal previous
        if previous is None:
            events.append({"t": sample["t"], "category": "capture", "label": "Recording starts · stationary calibration"})
        else:
            if previous["phase"] != sample["phase"]:
                events.append({"t": sample["t"], "category": "calibration", "label": "Calibration complete · observation recording"})
            if sample["segment"] is not None and previous["segment"] is not None and sample["segment"] != previous["segment"]:
                gap = {"t": sample["t"], "utc": dt.datetime.fromtimestamp(sample["epoch"], dt.timezone.utc).isoformat(),
                       "dt_s": sample["dt"], "segment": sample["segment"]}
                gaps.append(gap)
                events.append({"t": sample["t"], "category": "gap", "label": f"IMU timing gap {sample['dt']*1000:.1f} ms · yaw segment {int(sample['segment'])}"})
        for key in states:
            good = sample["health"][key]
            if not good and key not in active:
                active[key] = sample["t"]
                events.append({"t": sample["t"], "category": "health", "label": names[key]})
            elif good and key in active:
                begin = active.pop(key)
                episodes.append({"condition": key, "label": names[key], "start_s": begin,
                                 "end_s": sample["t"], "duration_s": sample["t"]-begin, "open_end": False})
                events.append({"t": sample["t"], "category": "health", "label": names[key] + " · recovered"})
        previous = sample

    with (root / "samples.csv").open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            mono = numeric(row.get("monotonic_s"))
            if mono is None or previous_mono is not None and mono <= previous_mono:
                raise ValueError("Nonfinite/non-increasing IMU monotonic time")
            previous_mono = mono
            if start_mono is None:
                start_mono, start_epoch = mono, epoch(row["utc"])
                lat, lon = numeric(row.get("gnss_lat")), numeric(row.get("gnss_lon"))
                if lat is None or lon is None:
                    raise ValueError("First row needs a finite GPS origin for this replay")
                origin = (lat, lon, 111320 * math.cos(math.radians(lat)))
            sample = compact(row, origin, start_mono)
            if previous is not None and sample["epoch"] < previous["epoch"]:
                raise ValueError("UTC moved backward; needs explicit clock-jump handling")
            transition(sample)
            counts[sample["phase"]] += 1
            counts["health_" + sample["health"]["code"]] += 1
            counts["logger_usable"] += int(sample["loggerUsable"])
            raw_count += 1
            if sample["dt"] is not None and sample["dt"] > 0:
                deltas.append(sample["dt"])
            counts["gyro_near_full_scale"] += int(any(abs(numeric(row.get(f"gyro_{axis}_raw"), 0)) >= 32760 for axis in "xyz"))
            counts["accel_near_full_scale"] += int(any(abs(numeric(row.get(f"accel_{axis}_raw"), 0)) >= 32760 for axis in "xyz"))
            new_bucket = math.floor(sample["t"] / interval)
            changed = pending is not None and (sample["phase"] != pending["phase"]
                       or sample["segment"] != pending["segment"] or sample["health"] != pending["health"])
            if bucket is not None and (new_bucket != bucket or changed):
                flush()
                envelope = {}
            bucket, pending = new_bucket, sample
            for name in SIGNALS:
                if sample[name] is not None:
                    envelope.setdefault(name, []).append(sample[name])
        flush()
    if not samples:
        raise ValueError("Empty capture")
    if raw_count != metadata.get("sample_rows"):
        raise ValueError("CSV row count disagrees with capture metadata")
    for key, begin in active.items():
        episodes.append({"condition": key, "label": names[key], "start_s": begin,
                         "end_s": previous["t"], "duration_s": previous["t"]-begin, "open_end": True})
    wall_times = [s["epoch"] for s in samples]
    for event in notes.get("operator_reported_events", []):
        stamp = epoch(event["local_time"])
        # Align operator wall-clock notes through recorded UTC/monotonic pairs.
        index = max(0, min(len(samples)-1, bisect.bisect_right(wall_times, stamp)-1))
        seconds = samples[index]["t"] + stamp - samples[index]["epoch"]
        if not 0 <= seconds <= samples[-1]["t"]:
            raise ValueError("Operator event is outside the capture")
        label = "Engine started" if event["event"] == "engine_started" else "Parked · engine off"
        events.append({"t": seconds, "category": "operator", "label": label + " · minute precision",
                       "uncertainUntil": seconds + 60, "reportedTime": event["local_time"]})
    events.append({"t": samples[-1]["t"], "category": "capture", "label": "Logger stopped cleanly"})
    events.sort(key=lambda event: event["t"])
    times = [s["t"] for s in samples]
    for event in events:
        event["sample"] = min(len(samples)-1, bisect.bisect_left(times, event["t"]))
    return {"title": "November 17, 2026 · Manual IMU / GPS replay", "run": root.name,
            "samples": samples, "events": events, "episodes": sorted(episodes, key=lambda x:x["start_s"]),
            "gaps": gaps, "metadata": metadata, "operatorNotes": notes,
            "summary": {"rawRows": raw_count, "playbackSamples": len(samples), "binSeconds": interval,
                        "duration_s": samples[-1]["t"], "startEpoch": start_epoch,
                        "median_dt_s": statistics.median(deltas), "max_dt_s": max(deltas),
                        "origin": list(origin[:2]), "counts": dict(counts)}, "verified": verified}


def render(payload):
    shared = SHARED_TEMPLATE.read_text(encoding="utf-8")
    style = shared.split("<style>", 1)[1].split("</style>", 1)[0]
    geometry = shared[shared.index("function viewport("):shared.index("function drawMap(")]
    encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    return TEMPLATE.read_text(encoding="utf-8").replace("__SHARED_STYLE__", style).replace("__SHARED_GEOMETRY__", geometry).replace("__DATA__", encoded)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--operator-notes", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    root, output = args.evidence_root.resolve(), args.output_dir.resolve()
    if root == output or root in output.parents:
        raise ValueError("Derived output must be outside the raw capture")
    payload = build_data(root, json.loads(args.operator_notes.read_text()))
    sources = [root / name for name in ("samples.csv", "gnss.jsonl", "metadata.json", "SHA256SUMS")]
    sources += [args.operator_notes, Path(__file__).resolve(), TEMPLATE, SHARED_TEMPLATE,
                LATEST_BUILDER, HERE / "build_tractor_mission_replays_20260919.py",
                HERE / "check_imu_replay_ui_20261117.cjs"]
    payload["source"] = [{"name": p.name, "sha256": digest(p)} for p in sources]
    output.mkdir(parents=True, exist_ok=True)
    page = render(payload)
    if any(token in page for token in ("__DATA__", "__SHARED_")):
        raise ValueError("Unexpanded replay template")
    html = output / "replay_20261117_imu_manual.html"
    html.write_text(page, encoding="utf-8", newline="\n")
    (output / "summary.json").write_text(json.dumps({k: payload[k] for k in ("summary", "gaps", "metadata", "source")}, indent=2)+"\n", encoding="utf-8")
    (output / "operator_notes.json").write_bytes(args.operator_notes.read_bytes())
    with (output / "gps_health_episodes.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=("condition", "label", "start_s", "end_s", "duration_s", "open_end"))
        writer.writeheader()
        writer.writerows(payload["episodes"])
    summary = payload["summary"]
    report = ["# November 17 manual-drive IMU / GPS historical replay", "",
              "Observation only. No motion authority, reconstructed controls, route or fused heading.", "",
              f"Capture: `{root.name}`. Commit: `{payload['metadata']['commit']}`.",
              f"Verified {summary['rawRows']:,} raw rows; {summary['duration_s']:.1f} seconds; {summary['playbackSamples']:,} playback samples.",
              f"Median host dt {summary['median_dt_s']*1000:.3f} ms; maximum {summary['max_dt_s']*1000:.3f} ms.", "",
              "## How to use", "",
              "Open `replay_20261117_imu_manual.html` locally. No server or internet needed. Play/pause, scrub, select a chart time window, click a chart, or jump/search events. Map zoom controls match the shared mission replay.", "",
              "## Evidence and limitations", "",
              "- Shared lineage: November 9 mission replay; September 19 digest/numeric/boolean helpers; current mission template CSS and map projection/zoom/drawing helpers are embedded at build time.",
              "- Playback uses approximately 100 ms bins, with raw per-bin min/max envelopes for gyro/acceleration. Phase, yaw-segment and health transitions force bin boundaries. Raw counts/timing/health episodes are computed before decimation. Point values are the last recorded row of each bin, not an average.",
              "- No GPS smoothing, position correction, heading fusion or turn classification. GPS coordinates are plotted as recorded, including degraded fixes. An offline local east/north grid replaces map tiles.",
              "- The health overlay includes the unchanged preflight baseline range 0.80–1.30 m in addition to packet, RTK and heading checks. It can be amber when the logger's narrower GPS=valid flag is true. These are observation labels, not reconstructed arbitrator decisions.",
              "- Packet age and heading-source age are separate. The producer's general timestamp is not a dedicated position-measurement timestamp; exact position age is unavailable here.",
              "- Wrapped compass heading plots break at north and at yaw-segment boundaries. Calibration has no corrected gyro/yaw. Two recorded gaps reset relative yaw and its anchor; never subtract yaw across segments.",
              "- Operator engine-on/off notes have minute precision: shaded 60-second uncertainty intervals. Engine-on does not establish exact movement start/end.",
              "- Angular disagreement is logged only when the logger considered GPS usable. It is not an independent absolute heading-error measurement. Heading/yaw remain experimental, not navigation.",
              "- Health episode durations may overlap across conditions. Do not sum them as independent stopped time. No motion interruptions can be inferred without command/status logs.",
              f"- Near-full-scale raw rows: gyro {summary['counts'].get('gyro_near_full_scale',0)}, acceleration {summary['counts'].get('accel_near_full_scale',0)}. Raw min/max envelopes are retained; host polling cannot recover missed samples.", "",
              "## Sampling gaps", ""]
    report += [f"- {g['utc']}: {g['dt_s']*1000:.3f} ms; new yaw segment {int(g['segment'])}." for g in payload["gaps"]]
    report += ["", "## SHA-256 source provenance", ""]
    report += [f"- `{s['name']}`: `{s['sha256']}`" for s in payload["source"]]
    (output / "REPORT.md").write_text("\n".join(report)+"\n", encoding="utf-8")
    readme = ["# November 17 IMU / GPS replay", "",
              "Open `replay_20261117_imu_manual.html` in a browser. It is a self-contained historical replay; no network, server, robot connection or external map tiles are used.", "",
              "Play/pause and real-time speed control share a timeline with map, health strip and six signal charts. Search/jump to engine events, sampling gaps or GPS-health changes. Click a chart to seek. Choose a local chart window to examine vibration or turns.", "",
              "Engine times are operator reports with unknown seconds. The shaded event-minute bands express that uncertainty. Timing gaps reset the recorded yaw into separate segments; final yaw is not whole-run drift. No planned mission or command-state reconstruction is shown.", "",
              "See `REPORT.md` for source hashes, definitions and limitations; `summary.json` for capture statistics; `gps_health_episodes.csv` for raw-resolution condition episodes; and `operator_notes.json` for the preserved engine-event notes.", "",
              "## Rebuild from the preserved raw Windows copy", "", "```powershell",
              "python field_testing/tools/build_tractor01_imu_replay_20261117.py --evidence-root field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261117_imu_manual/imu_20261117T161449_639816Z --operator-notes field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261117_imu_manual/operator_notes.json --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261117_imu_manual_replay", "```", "",
              "The builder verifies all original capture hashes before reading the CSV. Derived files remain separate from the unchanged raw evidence.", ""]
    (output / "README.md").write_text("\n".join(readme), encoding="utf-8")
    (output / "SHA256SUMS").write_text("\n".join(f"{digest(p).lower()}  {p.name}" for p in sorted(output.iterdir()) if p.is_file() and p.name != "SHA256SUMS")+"\n", encoding="utf-8")
    print(json.dumps({"html": str(html), "bytes": html.stat().st_size, **summary}, indent=2))


if __name__ == "__main__":
    main()
