#!/usr/bin/env python3
"""Analyze the valid 2026-10-30 stationary phone-network A/B windows.

Server acceptance timestamps are authoritative.  Phone timestamps are used only
for phone-local scheduler and round-trip measurements; this script never treats
phone/server wall-clock subtraction as one-way latency.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


TRIALS = {
    "A_zerotier": {
        "server": "wifi_primary_control_20261030_131635.jsonl",
        "phone": "phone_flight_recorder_20261030_131635.jsonl",
        "start": "2026-10-30T17:18:11+00:00",
        "end": "2026-10-30T17:28:11+00:00",
    },
    "B_direct_pre_walk": {
        "server": "wifi_primary_control_20261030_133500.jsonl",
        "phone": "phone_flight_recorder_20261030_133500.jsonl",
        "start": "2026-10-30T17:37:11+00:00",
        "end": "2026-10-30T17:43:59.999+00:00",
    },
    "B_direct_including_walk_fault": {
        "server": "wifi_primary_control_20261030_133500.jsonl",
        "phone": "phone_flight_recorder_20261030_133500.jsonl",
        "start": "2026-10-30T17:37:11+00:00",
        "end": "2026-10-30T17:47:11+00:00",
    },
}


def parse_time(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00")).astimezone(timezone.utc)


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    records = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        try:
            records.append(json.loads(raw))
        except json.JSONDecodeError as exc:
            raise SystemExit(f"{path}:{line_number}: invalid JSON: {exc}") from exc
    return records


def quantile(values: list[float], fraction: float) -> float | None:
    ordered = sorted(values)
    if not ordered:
        return None
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return round(ordered[lower], 3)
    value = ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)
    return round(value, 3)


def distribution(values: list[float]) -> dict[str, float | int | None]:
    return {
        "count": len(values),
        "p50": quantile(values, 0.50),
        "p95": quantile(values, 0.95),
        "p99": quantile(values, 0.99),
        "p99_9": quantile(values, 0.999),
        "max": round(max(values), 3) if values else None,
    }


def in_window(record_time: str, start: datetime, end: datetime) -> bool:
    when = parse_time(record_time)
    return start <= when <= end


def summarize_server(path: Path, start: datetime, end: datetime) -> dict[str, Any]:
    records = [
        record
        for record in load_jsonl(path)
        if record.get("time") and in_window(str(record["time"]), start, end)
    ]
    commands = [record for record in records if record.get("event") == "command"]
    auto_commands = [record for record in commands if record.get("mode") == "auto"]
    gaps = [float(record["command_gap_ms"]) for record in commands if isinstance(record.get("command_gap_ms"), (int, float))]
    auto_gaps = [float(record["command_gap_ms"]) for record in auto_commands if isinstance(record.get("command_gap_ms"), (int, float))]
    server_processing = [float(record["server_processing_ms"]) for record in commands if isinstance(record.get("server_processing_ms"), (int, float))]
    bridge_age = [float(record["bridge_age_ms"]) for record in commands if isinstance(record.get("bridge_age_ms"), (int, float))]
    client_rtt = [float(record["client_reported_rtt_ms"]) for record in commands if isinstance(record.get("client_reported_rtt_ms"), (int, float))]

    event_counts = Counter(str(record.get("event")) for record in records)
    safety_events = []
    for record in records:
        event = str(record.get("event"))
        if event in {
            "phone_freshness_expired",
            "auto_link_wait_entered",
            "auto_recovery_link_lost",
            "auto_recovery_window_expired",
            "auto_recovery_completed",
            "effective_state_transition",
            "session_invalidated",
            "command_rejected",
        }:
            safety_events.append(
                {
                    key: record.get(key)
                    for key in (
                        "time",
                        "event",
                        "reason",
                        "previous_state",
                        "effective_state",
                        "from_state",
                        "to_state",
                        "gap_ms",
                        "command_age_ms",
                        "sequence",
                    )
                    if key in record
                }
            )

    stop_episodes = []
    open_start: datetime | None = None
    for record in records:
        event = str(record.get("event"))
        when = parse_time(str(record["time"]))
        if event == "phone_freshness_expired" and open_start is None:
            open_start = when
        if (
            open_start is not None
            and event == "effective_state_transition"
            and record.get("to_state") == "auto_running"
        ):
            stop_episodes.append(
                {
                    "start": open_start.isoformat(),
                    "end": when.isoformat(),
                    "duration_seconds": round((when - open_start).total_seconds(), 3),
                    "recovery_reason": record.get("reason"),
                }
            )
            open_start = None
    if open_start is not None:
        stop_episodes.append(
            {
                "start": open_start.isoformat(),
                "end": None,
                "duration_seconds_within_window": round((end - open_start).total_seconds(), 3),
                "recovery_reason": "not recovered inside analysis window",
            }
        )

    return {
        "source": path.name,
        "events": len(records),
        "accepted_commands": len(commands),
        "accepted_rate_hz": round(len(commands) / (end - start).total_seconds(), 3),
        "auto_commands": len(auto_commands),
        "command_gap_ms": distribution(gaps),
        "auto_command_gap_ms": distribution(auto_gaps),
        "gaps_over_400_ms": sum(value > 400 for value in auto_gaps),
        "gaps_over_500_ms": sum(value > 500 for value in auto_gaps),
        "gaps_over_1000_ms": sum(value > 1000 for value in auto_gaps),
        "server_processing_ms": distribution(server_processing),
        "bridge_age_ms": distribution(bridge_age),
        "client_reported_rtt_ms": distribution(client_rtt),
        "event_counts": dict(sorted(event_counts.items())),
        "client_ips": sorted({str(record.get("client_ip")) for record in commands}),
        "authority_paths": sorted({str(record.get("authority_path")) for record in commands}),
        "sessions": len({str(record.get("session")) for record in commands}),
        "safety_timeline": safety_events,
        "stop_episodes": stop_episodes,
        "stop_episode_count": len(stop_episodes),
        "stopped_seconds_within_window": round(
            sum(
                float(item.get("duration_seconds", item.get("duration_seconds_within_window", 0.0)))
                for item in stop_episodes
            ),
            3,
        ),
    }


def summarize_phone(path: Path, start: datetime, end: datetime) -> dict[str, Any]:
    outer_records = load_jsonl(path)
    events = []
    for outer in outer_records:
        event = outer.get("event")
        if not isinstance(event, dict) or not event.get("time"):
            continue
        if in_window(str(event["time"]), start, end):
            events.append(event)
    event_counts = Counter(str(event.get("event")) for event in events)
    attempts = [event for event in events if event.get("event") == "send_attempt"]
    successes = [event for event in events if event.get("event") == "send_ok"]
    failures = [event for event in events if event.get("event") == "send_failed"]
    rtt = [float(event["rtt_ms"]) for event in successes if isinstance(event.get("rtt_ms"), (int, float))]
    network_rtt = [float(event["network_rtt_ms"]) for event in successes if isinstance(event.get("network_rtt_ms"), (int, float))]
    scheduler_delay = [float(event["scheduler_delay_ms"]) for event in attempts if isinstance(event.get("scheduler_delay_ms"), (int, float))]
    coalesced_wait = [float(event["coalesced_wait_ms"]) for event in attempts if isinstance(event.get("coalesced_wait_ms"), (int, float))]
    failure_reasons = Counter(
        f"status={event.get('status')} error={event.get('error')} abort={event.get('abort_reason')}"
        for event in failures
    )
    return {
        "source": path.name,
        "events": len(events),
        "event_counts": dict(sorted(event_counts.items())),
        "send_attempts": len(attempts),
        "send_ok": len(successes),
        "send_failed": len(failures),
        "rtt_ms": distribution(rtt),
        "network_rtt_ms": distribution(network_rtt),
        "scheduler_delay_ms": distribution(scheduler_delay),
        "coalesced_wait_ms": distribution(coalesced_wait),
        "scheduler_delay_over_100_ms": sum(value > 100 for value in scheduler_delay),
        "scheduler_delay_over_500_ms": sum(value > 500 for value in scheduler_delay),
        "failure_reasons": dict(failure_reasons),
        "visibility": dict(Counter(str(event.get("visibility")) for event in events)),
        "online": dict(Counter(str(event.get("online")) for event in events)),
    }


ROUTER_PATTERN = re.compile(
    r"^(?P<stamp>\w{3} \w{3}\s+\d+ \d\d:\d\d:\d\d \d{4}).*"
    r"(?P<kind>AP-STA-CONNECTED|AP-STA-DISCONNECTED|deauthenticated|disassociated).*"
    r"5a:55:ca:3f:5f:95",
    re.IGNORECASE,
)


def router_timeline(evidence_dir: Path) -> list[dict[str, str]]:
    path = evidence_dir / "router_gl" / "sft1200_4.8.3_log" / "system.log"
    if not path.exists():
        return []
    result = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = ROUTER_PATTERN.search(raw)
        if match:
            result.append({"local_time": match.group("stamp"), "event": match.group("kind"), "line": raw})
    return result


def zerotier_observations(evidence_dir: Path) -> dict[str, Any]:
    observations: dict[str, Any] = {}
    for name in ("trial_A_zerotier_before.txt", "trial_A_zerotier_after.txt"):
        path = evidence_dir / name
        text = path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""
        peer_line = next((line for line in text.splitlines() if "192.168.8.190/9994" in line), None)
        if peer_line is None:
            peer_line = next((line for line in text.splitlines() if line.startswith("82a60e38ec ")), None)
        observations[name] = {
            "phone_peer_line": peer_line,
            "route_to_phone": next((line.strip() for line in text.splitlines() if line.startswith("192.168.193.206 dev")), None),
        }
    return observations


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result: dict[str, Any] = {
        "method": {
            "server_clock": "authoritative for acceptance gaps and safety transitions",
            "phone_clock": "used for phone-local intervals and RTT only",
            "one_way_wall_clock_latency": "not calculated",
        },
        "trials": {},
        "router_phone_timeline": router_timeline(args.evidence_dir),
        "zerotier": zerotier_observations(args.evidence_dir),
    }
    for name, config in TRIALS.items():
        start = parse_time(config["start"])
        end = parse_time(config["end"])
        result["trials"][name] = {
            "window_utc": {"start": start.isoformat(), "end": end.isoformat()},
            "server": summarize_server(args.evidence_dir / config["server"], start, end),
            "phone": summarize_phone(args.evidence_dir / config["phone"], start, end),
        }
    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
