#!/usr/bin/env python3
"""Evaluate armed phone-control timing against the provisional field gates."""

from __future__ import annotations

import argparse
import json
import math
from datetime import datetime
from pathlib import Path


ARMED = {"manual", "auto"}


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def newest_log(directory: Path) -> Path:
    matches = list(directory.glob("wifi_primary_control_*.jsonl"))
    if not matches:
        raise SystemExit(f"No Wi-Fi control logs found in {directory}")
    return max(matches, key=lambda path: path.stat().st_mtime)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path)
    parser.add_argument("--latest-dir", type=Path)
    parser.add_argument("--require-minutes", type=float, default=2.0)
    args = parser.parse_args()
    if bool(args.log) == bool(args.latest_dir):
        parser.error("provide either a log path or --latest-dir")
    path = args.log if args.log else newest_log(args.latest_dir)

    gaps: list[float] = []
    rtts: list[float] = []
    segment_start: datetime | None = None
    last_armed_time: datetime | None = None
    longest_armed_seconds = 0.0
    heartbeat_expiries = 0
    rejected = 0
    network_events: list[str] = []
    previous_mode = "pause"
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        try:
            record = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise SystemExit(f"Invalid JSON on line {line_number}: {exc}") from exc
        event = str(record.get("event", ""))
        if event == "heartbeat_expired":
            heartbeat_expiries += 1
        if event.startswith("network_"):
            network_events.append(event)
        if event == "command_received" and record.get("accepted") is False:
            rejected += 1
        if event != "command":
            continue
        mode = str(record.get("phone_mode", "")).lower()
        if mode in ARMED:
            try:
                event_time = datetime.fromisoformat(str(record.get("time", record.get("server_time", ""))))
            except ValueError:
                event_time = None
            if event_time is not None:
                if previous_mode not in ARMED or last_armed_time is None or (event_time - last_armed_time).total_seconds() > 1.0:
                    segment_start = event_time
                last_armed_time = event_time
                if segment_start is not None:
                    longest_armed_seconds = max(longest_armed_seconds, (event_time - segment_start).total_seconds())
            rtt = record.get("client_reported_rtt_ms")
            if isinstance(rtt, (int, float)) and math.isfinite(rtt):
                rtts.append(float(rtt))
            gap = record.get("command_gap_ms")
            if previous_mode in ARMED and isinstance(gap, (int, float)) and math.isfinite(gap):
                gaps.append(float(gap))
        else:
            segment_start = None
            last_armed_time = None
        previous_mode = mode

    duration_minutes = longest_armed_seconds / 60
    p95_rtt = percentile(rtts, 0.95)
    max_rtt = max(rtts, default=math.nan)
    max_gap = max(gaps, default=math.nan)
    failures = []
    if duration_minutes < args.require_minutes:
        failures.append(f"armed sample {duration_minutes:.1f} min is shorter than {args.require_minutes:.1f} min")
    if heartbeat_expiries:
        failures.append(f"{heartbeat_expiries} heartbeat/freshness expiry event(s)")
    if not rtts or p95_rtt > 100 or max_rtt > 250:
        failures.append(f"RTT gate failed (p95 {p95_rtt:.1f} ms, max {max_rtt:.1f} ms)")
    if not gaps or max_gap > 400:
        failures.append(f"command-gap gate failed (max {max_gap:.1f} ms)")

    print(f"Log: {path}")
    print(f"Longest continuous armed duration: {duration_minutes:.1f} min; command intervals: {len(gaps)}")
    print(f"RTT: p95 {p95_rtt:.1f} ms; max {max_rtt:.1f} ms")
    print(f"Maximum armed command gap: {max_gap:.1f} ms")
    print(f"Heartbeat expiries: {heartbeat_expiries}; rejected commands: {rejected}")
    print(f"Network diagnostic events: {len(network_events)} (review separately)")
    if failures:
        print("FAIL: " + "; ".join(failures))
        raise SystemExit(1)
    print("PASS: provisional phone timing gates met for this sample.")
    print("This timing result does not replace physical, preflight, route, or operator review.")


if __name__ == "__main__":
    main()
