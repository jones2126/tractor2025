#!/usr/bin/env python3
"""Reproduce the 2026-10-21 phone-freshness telemetry analysis.

This script is intentionally read-only with respect to the evidence archive. It
verifies the archive and internal manifest, extracts to a temporary directory,
and writes only derived analysis artifacts beside this script.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import re
import statistics
import tarfile
import tempfile
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Iterable


EXPECTED_ARCHIVE_SHA256 = "a316fcd031e60afdba5cddd6d9baf29377956043f5b2a772e74bd04344387574"
EDT = timezone(timedelta(hours=-4), "EDT")
UTC = timezone.utc
ANCHOR = datetime.fromisoformat("2026-10-21T18:18:14.028+00:00")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dt(value: str) -> datetime:
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    return parsed if parsed.tzinfo else parsed.replace(tzinfo=UTC)


def fnum(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def truth(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def percentile(values: Iterable[float], fraction: float) -> float | None:
    ordered = sorted(float(v) for v in values if v is not None and math.isfinite(float(v)))
    if not ordered:
        return None
    position = (len(ordered) - 1) * fraction
    lo = math.floor(position)
    hi = math.ceil(position)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (position - lo)


def distribution(values: Iterable[float]) -> dict[str, Any]:
    clean = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    return {
        "n": len(clean),
        "min": min(clean) if clean else None,
        "p50": percentile(clean, 0.50),
        "p90": percentile(clean, 0.90),
        "p95": percentile(clean, 0.95),
        "p99": percentile(clean, 0.99),
        "max": max(clean) if clean else None,
        "mean": statistics.fmean(clean) if clean else None,
    }


def nearest(records: list[dict[str, Any]], when: datetime, max_delta_s: float | None = None) -> dict[str, Any] | None:
    if not records:
        return None
    stamps = [r["_time"] for r in records]
    idx = bisect.bisect_left(stamps, when)
    candidates = records[max(0, idx - 1) : min(len(records), idx + 1)]
    result = min(candidates, key=lambda row: abs((row["_time"] - when).total_seconds()))
    if max_delta_s is not None and abs((result["_time"] - when).total_seconds()) > max_delta_s:
        return None
    return result


def prior(records: list[dict[str, Any]], when: datetime) -> dict[str, Any] | None:
    stamps = [r["_time"] for r in records]
    idx = bisect.bisect_left(stamps, when)
    return records[idx - 1] if idx else None


def following(records: list[dict[str, Any]], when: datetime) -> dict[str, Any] | None:
    stamps = [r["_time"] for r in records]
    idx = bisect.bisect_right(stamps, when)
    return records[idx] if idx < len(records) else None


def compact(obj: Any, limit: int = 1200) -> str:
    text = json.dumps(obj, sort_keys=True, separators=(",", ":"), default=str)
    return text if len(text) <= limit else text[: limit - 3] + "..."


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            row["_line"] = line_number
            row["_time"] = dt(row["time"])
            rows.append(row)
    return rows


def load_flight(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            wrapper = json.loads(line)
            event = dict(wrapper["event"])
            event["_line"] = line_number
            event["_time"] = dt(event["time"])
            event["_upload_time"] = dt(wrapper["server_time"])
            event["_client_ip"] = wrapper.get("client_ip")
            rows.append(event)
    rows.sort(key=lambda row: (row["_time"], row["_line"]))
    return rows


def load_csv_window(
    path: Path,
    source: str,
    expiries: list[datetime],
    critical_start: datetime,
    critical_end: datetime,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    near: list[dict[str, Any]] = []
    critical: list[dict[str, Any]] = []
    with path.open(encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if source == "pursuit":
            next(reader, None)  # units/description row
        for row in reader:
            try:
                when = (
                    datetime.fromtimestamp(float(row["timestamp"]), UTC)
                    if source == "pursuit"
                    else dt(row["time"])
                )
            except (KeyError, TypeError, ValueError):
                continue
            row["_time"] = when
            if critical_start <= when <= critical_end:
                critical.append(row.copy())
            if any(abs((when - expiry).total_seconds()) <= 3.0 for expiry in expiries):
                near.append(row.copy())
    near.sort(key=lambda row: row["_time"])
    critical.sort(key=lambda row: row["_time"])
    return near, critical


def journal_window(path: Path, start: datetime, end: datetime) -> list[dict[str, Any]]:
    rows = []
    pattern = re.compile(r"^Oct 21 (\d\d:\d\d:\d\d) ")
    for line_number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        match = pattern.match(line)
        if not match:
            continue
        local = datetime.strptime("2026-10-21 " + match.group(1), "%Y-%m-%d %H:%M:%S").replace(tzinfo=EDT)
        when = local.astimezone(UTC)
        if start <= when <= end:
            rows.append({"_time": when, "_line": line_number, "line": line})
    return rows


def router_summary(paths: list[Path], expiries: list[datetime]) -> dict[str, Any]:
    result: dict[str, Any] = {"exports": [], "events_near_expiries": {}}
    line_re = re.compile(r"^(2026-\d\d-\d\d \d\d:\d\d:\d\d) (.+)$")
    all_events: list[tuple[datetime, str]] = []
    for path in paths:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        header = lines[:9]
        item = {"path": str(path), "sha256": sha256(path), "header": header, "event_count": 0, "dhcp_acks": 0}
        for line in lines:
            match = line_re.match(line)
            if not match:
                continue
            local = datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S").replace(tzinfo=EDT)
            when = local.astimezone(UTC)
            all_events.append((when, line))
            item["event_count"] += 1
            if "receive ack from server with ip" in line:
                item["dhcp_acks"] += 1
        result["exports"].append(item)
    unique_events = sorted(set(all_events), key=lambda pair: pair[0])
    for expiry in expiries:
        key = expiry.isoformat(timespec="milliseconds")
        nearby = [line for when, line in unique_events if abs((when - expiry).total_seconds()) <= 180]
        result["events_near_expiries"][key] = nearby
    acks = [when for when, line in unique_events if "receive ack from server with ip" in line]
    intervals = [(b - a).total_seconds() for a, b in zip(acks, acks[1:]) if (b - a).total_seconds() < 1000]
    result["unique_dhcp_ack_count"] = len(acks)
    result["dhcp_ack_interval_seconds"] = distribution(intervals)
    return result


def classify(expiry: datetime, browser_near: list[dict[str, Any]], pursuit: dict[str, Any] | None, accepted_rtt_ms: float | None) -> str:
    names = {str(item.get("event")) for item in browser_near}
    if names & {"visibility_change", "session_cleared_after_visibility_restore", "page_loaded"}:
        return "page lifecycle / reload"
    if names & {"offline", "browser_offline_fail_closed", "connection_hint_changed", "connection_path_changed_fail_closed"}:
        return "observable browser network/path change"
    if "ack_watchdog_expired" in names and accepted_rtt_ms is not None and accepted_rtt_ms >= 500:
        signature = "delayed response/ack (>500 ms), single-flight send queue"
    elif names & {"send_failed", "ack_watchdog_expired"}:
        signature = "request/ack failure with reconnect"
    else:
        signature = "isolated server freshness threshold crossing"
    if expiry < datetime.fromisoformat("2026-10-21T17:38:00+00:00"):
        return "pre-mission stationary/control preparation; " + signature
    if expiry < datetime.fromisoformat("2026-10-21T18:42:00+00:00"):
        return "primary mission Auto interruption; " + signature
    if expiry < datetime.fromisoformat("2026-10-21T19:02:30+00:00"):
        context = "continued-mission powered Auto" if pursuit and truth(pursuit.get("driving")) else "continued-mission recovery/wait"
        return context + "; " + signature
    return "post-mission maneuvering; " + signature


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("router_logs", nargs=2, type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)

    outer_hash = sha256(args.archive)
    if outer_hash != EXPECTED_ARCHIVE_SHA256:
        raise SystemExit(f"archive SHA-256 mismatch: {outer_hash}")

    with tempfile.TemporaryDirectory(prefix="tractor2025-phone-timeout-") as temporary:
        root = Path(temporary)
        with tarfile.open(args.archive, "r:gz") as archive:
            archive.extractall(root, filter="data")
        manifest_path = root / "diagnostics/collected_file_hashes.sha256"
        internal_hashes: list[dict[str, Any]] = []
        for line in manifest_path.read_text(encoding="utf-8").splitlines():
            expected, rel = line.split("  ./", 1)
            evidence = root / rel
            actual = sha256(evidence)
            internal_hashes.append({"path": rel, "expected": expected, "actual": actual, "ok": expected == actual})
        if not all(item["ok"] for item in internal_hashes):
            raise SystemExit("one or more internal evidence hashes failed")

        control = load_jsonl(root / "wifi_logs/wifi_primary_control_20261021_130733.jsonl")
        flight = load_flight(root / "wifi_logs/phone_flight_recorder_20261021_130733.jsonl")
        commands = [row for row in control if row.get("event") == "command"]
        expiries = [row for row in control if row.get("event") == "heartbeat_expired"]
        expiry_times = [row["_time"] for row in expiries]
        claims = [row for row in control if row.get("event") == "owner_claimed"]
        transitions = [row for row in control if row.get("event") == "mode_transition"]
        site = args.archive.resolve().parents[2]
        audit_path = site / (
            "mission_plans/20260915_master_boundary_replay/"
            "generated_master_manual_field_20260920/"
            "62_Collins_master_manual_resampled_1mps_20260920_audit.csv"
        )
        with audit_path.open(encoding="utf-8-sig", newline="") as handle:
            phase_by_waypoint = [row.get("phase", "") for row in csv.DictReader(handle)]

        critical_start = ANCHOR - timedelta(seconds=30)
        critical_end = ANCHOR + timedelta(seconds=30)
        pursuit_near, pursuit_critical = load_csv_window(
            root / "pursuit_logs/pursuit_log_20261021_133810.csv",
            "pursuit", expiry_times, critical_start, critical_end,
        )
        mission_near, mission_critical = load_csv_window(
            root / "mission_logs/master_manual_phone_20261021_133807.csv",
            "mission", expiry_times, critical_start, critical_end,
        )
        journal_critical = journal_window(root / "diagnostics/service_journal.log", critical_start, critical_end)

        # Server receive minus phone send-wall-clock. This is offset plus outbound
        # path time; the minimum is only an upper bound on the clock offset.
        phone_to_server = [fnum(row.get("estimated_phone_to_server_ms")) for row in commands]
        phone_to_server = [value for value in phone_to_server if value is not None]
        # The minimum observed server-minus-phone value is the tightest available
        # upper bound on the phone clock offset (it still contains non-negative
        # outbound transit). Use it only to align browser events diagnostically.
        clock_offset_bound_ms = min(phone_to_server)
        for row in flight:
            row["_aligned_time"] = row["_time"] + timedelta(milliseconds=clock_offset_bound_ms)
        upload_delays = [(row["_upload_time"] - row["_time"]).total_seconds() * 1000 for row in flight]

        event_rows = []
        for idx, expiry_record in enumerate(expiries):
            when = expiry_record["_time"]
            before = prior(commands, when)
            after = following(commands, when)
            p = nearest(pursuit_near, when, 1.0)
            m = nearest(mission_near, when, 1.0)
            browser_near = [row for row in flight if abs((row["_aligned_time"] - when).total_seconds()) <= 2.0]
            browser_30 = [row for row in flight if abs((row["_aligned_time"] - when).total_seconds()) <= 30.0]
            next_claim = following(claims, when)
            next_transition = following(transitions, when)
            prior_attempts = [row for row in flight if row.get("event") == "send_attempt" and row["_aligned_time"] <= when]
            prior_oks = [row for row in flight if row.get("event") == "send_ok" and row["_aligned_time"] <= when]
            last_attempt = prior_attempts[-1] if prior_attempts else None
            last_ok = prior_oks[-1] if prior_oks else None
            before_client_time = datetime.fromtimestamp(float(before["client_time_ms"]) / 1000, UTC) if before and before.get("client_time_ms") else None
            matching_attempts = [
                row for row in flight
                if before_client_time and row.get("event") == "send_attempt"
                and row.get("command_sequence") == before.get("sequence")
                and abs((row["_time"] - before_client_time).total_seconds()) < 0.25
            ]
            matching_oks = [
                row for row in flight
                if before_client_time and row.get("event") == "send_ok"
                and row.get("command_sequence") == before.get("sequence")
                and abs((row["_time"] - before_client_time).total_seconds()) < 2.0
            ]
            accepted_attempt = matching_attempts[-1] if matching_attempts else None
            accepted_ok = matching_oks[-1] if matching_oks else None
            accepted_rtt = fnum(accepted_ok.get("rtt_ms")) if accepted_ok else None
            wake_events = [row for row in flight if str(row.get("event", "")).startswith("wake_lock") and row["_aligned_time"] <= when]
            last_wake = wake_events[-1] if wake_events else None
            next_pause_samples = [row for row in mission_near if when <= row["_time"] <= when + timedelta(seconds=3) and str(row.get("trans_mode")) == "2"]
            first_pause = next_pause_samples[0] if next_pause_samples else None
            controller_pause_samples = [
                row for row in pursuit_near
                if when <= row["_time"] <= when + timedelta(seconds=3)
                and not truth(row.get("driving"))
                and str(row.get("handheld_mode")) in {"2", "3", "9"}
            ]
            first_controller_pause = controller_pause_samples[0] if controller_pause_samples else None
            next_neutral_feedback = [
                row for row in next_pause_samples
                if fnum(row.get("jrk_scaled_feedback")) is not None
                and abs(float(row["jrk_scaled_feedback"]) - 2836) <= 10
            ]
            first_neutral_feedback = next_neutral_feedback[0] if next_neutral_feedback else None
            waypoint = int(p["waypoint_idx"]) if p and str(p.get("waypoint_idx", "")).isdigit() else None
            frozen_waypoint = (
                int(first_controller_pause["waypoint_idx"])
                if first_controller_pause and str(first_controller_pause.get("waypoint_idx", "")).isdigit()
                else None
            )
            powered_auto = bool(
                p and truth(p.get("driving")) and str(p.get("handheld_mode")) == "0"
                and (fnum(p.get("speed_cmd_mps")) or 0) > 0
            )
            event_rows.append({
                "event_number": idx + 1,
                "utc": when.isoformat(timespec="milliseconds"),
                "edt": when.astimezone(EDT).isoformat(timespec="milliseconds"),
                "classification": classify(when, browser_near, p, accepted_rtt),
                "interrupted_powered_auto": powered_auto,
                "mission_elapsed_s": p.get("elapsed_s") if p else None,
                "waypoint": waypoint,
                "waypoints_total": p.get("waypoints_total") if p else None,
                "phase": phase_by_waypoint[waypoint] if waypoint is not None and waypoint < len(phase_by_waypoint) else None,
                "frozen_waypoint": frozen_waypoint,
                "frozen_phase": phase_by_waypoint[frozen_waypoint] if frozen_waypoint is not None and frozen_waypoint < len(phase_by_waypoint) else None,
                "controller_driving": p.get("driving") if p else None,
                "controller_wait_reason": p.get("wait_reason") if p else None,
                "handheld_mode": p.get("handheld_mode") if p else None,
                "actual_speed_mps": p.get("actual_speed_mps") if p else None,
                "rtk_fix": p.get("fix_quality") if p else (m.get("fix_quality") if m else None),
                "heading_valid": p.get("head_valid") if p else (m.get("head_valid") if m else None),
                "heading_deg": p.get("heading_compass_deg") if p else (m.get("heading_deg") if m else None),
                "session": expiry_record.get("session"),
                "client_id": expiry_record.get("client_id"),
                "prior_mode": before.get("mode") if before else None,
                "prior_sequence": before.get("sequence") if before else None,
                "following_sequence": after.get("sequence") if after else None,
                "prior_command_utc": before["_time"].isoformat(timespec="milliseconds") if before else None,
                "following_command_utc": after["_time"].isoformat(timespec="milliseconds") if after else None,
                "server_silence_before_expiry_ms": round((when - before["_time"]).total_seconds() * 1000, 1) if before else None,
                "server_silence_to_next_command_ms": round((after["_time"] - when).total_seconds() * 1000, 1) if after else None,
                "prior_client_reported_rtt_ms": before.get("client_reported_rtt_ms") if before else None,
                "following_client_reported_rtt_ms": after.get("client_reported_rtt_ms") if after else None,
                "prior_command_gap_ms": before.get("command_gap_ms") if before else None,
                "following_command_gap_ms": after.get("command_gap_ms") if after else None,
                "prior_server_processing_ms": before.get("server_processing_ms") if before else None,
                "prior_bridge_age_ms": before.get("bridge_age_ms") if before else None,
                "browser_clock_offset_bound_ms": clock_offset_bound_ms,
                "last_browser_send_attempt_phone_utc": last_attempt["_time"].isoformat(timespec="milliseconds") if last_attempt else None,
                "last_browser_send_attempt_aligned_utc": last_attempt["_aligned_time"].isoformat(timespec="milliseconds") if last_attempt else None,
                "last_browser_send_attempt_sequence": last_attempt.get("command_sequence") if last_attempt else None,
                "last_browser_send_ok_phone_utc": last_ok["_time"].isoformat(timespec="milliseconds") if last_ok else None,
                "last_browser_send_ok_aligned_utc": last_ok["_aligned_time"].isoformat(timespec="milliseconds") if last_ok else None,
                "last_browser_send_ok_sequence": last_ok.get("command_sequence") if last_ok else None,
                "last_accepted_request_browser_attempt_phone_utc": accepted_attempt["_time"].isoformat(timespec="milliseconds") if accepted_attempt else None,
                "last_accepted_request_browser_attempt_aligned_utc": accepted_attempt["_aligned_time"].isoformat(timespec="milliseconds") if accepted_attempt else None,
                "last_accepted_request_browser_response_phone_utc": accepted_ok["_time"].isoformat(timespec="milliseconds") if accepted_ok else None,
                "last_accepted_request_browser_response_aligned_utc": accepted_ok["_aligned_time"].isoformat(timespec="milliseconds") if accepted_ok else None,
                "last_accepted_request_browser_rtt_ms": accepted_rtt,
                "browser_events_within_2s": ";".join(str(row.get("event")) for row in browser_near),
                "browser_events_within_30s": ";".join(str(row.get("event")) for row in browser_30 if row.get("event") not in {"send_attempt", "send_ok"}),
                "visibility": browser_near[-1].get("visibility") if browser_near else None,
                "online": browser_near[-1].get("online") if browser_near else None,
                "wake_lock_evidence": ";".join(str(row.get("event")) for row in browser_30 if str(row.get("event", "")).startswith("wake_lock")),
                "last_wake_lock_event": last_wake.get("event") if last_wake else None,
                "last_wake_lock_event_aligned_utc": last_wake["_aligned_time"].isoformat(timespec="milliseconds") if last_wake else None,
                "next_claim_utc": next_claim["_time"].isoformat(timespec="milliseconds") if next_claim else None,
                "next_claim_session": next_claim.get("session") if next_claim else None,
                "next_mode_transition_utc": next_transition["_time"].isoformat(timespec="milliseconds") if next_transition else None,
                "next_mode": next_transition.get("to_mode") if next_transition else None,
                "next_mode_reason": next_transition.get("reason") if next_transition else None,
                "mission_trans_mode": m.get("trans_mode") if m else None,
                "jrk_target": m.get("jrk_target") if m else None,
                "jrk_feedback": m.get("jrk_scaled_feedback") if m else None,
                "teensy_cmd_age_ms": m.get("trans_cmd_vel_age_ms") if m else None,
                "controller_pause_first_seen_utc": first_controller_pause["_time"].isoformat(timespec="milliseconds") if first_controller_pause else None,
                "controller_pause_delay_ms": round((first_controller_pause["_time"] - when).total_seconds() * 1000, 1) if first_controller_pause else None,
                "teensy_pause_first_seen_utc": first_pause["_time"].isoformat(timespec="milliseconds") if first_pause else None,
                "neutral_target_delay_ms": round((first_pause["_time"] - when).total_seconds() * 1000, 1) if first_pause else None,
                "neutral_feedback_first_seen_utc": first_neutral_feedback["_time"].isoformat(timespec="milliseconds") if first_neutral_feedback else None,
                "neutral_feedback_delay_ms": round((first_neutral_feedback["_time"] - when).total_seconds() * 1000, 1) if first_neutral_feedback else None,
            })

        failure_ranges = [(when - timedelta(seconds=2), when + timedelta(seconds=2)) for when in expiry_times]
        armed = [row for row in commands if str(row.get("mode", "")).lower() in {"manual", "auto"}]
        failure_commands = [row for row in armed if any(start <= row["_time"] <= end for start, end in failure_ranges)]
        healthy_commands = [row for row in armed if all(abs((row["_time"] - when).total_seconds()) > 10 for when in expiry_times)]
        failure_flight = [row for row in flight if any(start <= row["_aligned_time"] <= end for start, end in failure_ranges)]
        healthy_flight = [row for row in flight if all(abs((row["_aligned_time"] - when).total_seconds()) > 10 for when in expiry_times)]

        def command_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
            fields = {
                "command_gap_ms": "command_gap_ms",
                "client_reported_rtt_ms": "client_reported_rtt_ms",
                "estimated_phone_to_server_ms": "estimated_phone_to_server_ms",
                "server_processing_ms": "server_processing_ms",
                "bridge_age_ms": "bridge_age_ms",
            }
            return {name: distribution(v for row in rows if (v := fnum(row.get(field))) is not None) for name, field in fields.items()}

        # Reproduce the existing qualification analyzer exactly, without changing
        # its gate interpretation. In particular, it includes a gap only when
        # both the prior and current command records are armed.
        analyzer_gaps: list[float] = []
        analyzer_rtts: list[float] = []
        previous_mode = "pause"
        segment_start: datetime | None = None
        last_armed: datetime | None = None
        longest_armed_s = 0.0
        for row in commands:
            mode = str(row.get("phone_mode") or row.get("mode", "")).lower()
            if mode in {"manual", "auto"}:
                if previous_mode not in {"manual", "auto"} or last_armed is None or (row["_time"] - last_armed).total_seconds() > 1.0:
                    segment_start = row["_time"]
                last_armed = row["_time"]
                if segment_start is not None:
                    longest_armed_s = max(longest_armed_s, (row["_time"] - segment_start).total_seconds())
                rtt = fnum(row.get("client_reported_rtt_ms"))
                if rtt is not None:
                    analyzer_rtts.append(rtt)
                gap = fnum(row.get("command_gap_ms"))
                if previous_mode in {"manual", "auto"} and gap is not None:
                    analyzer_gaps.append(gap)
            else:
                segment_start = None
                last_armed = None
            previous_mode = mode

        accepted_keys = {(row.get("session"), row.get("sequence")) for row in commands}
        received = [row for row in control if row.get("event") == "command_received"]
        unmatched_received = [row for row in received if (row.get("session"), row.get("sequence")) not in accepted_keys]
        sequence_gaps = []
        by_session: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for row in commands:
            by_session[str(row.get("session"))].append(row)
        for session, rows in by_session.items():
            for left, right in zip(rows, rows[1:]):
                delta = int(right["sequence"]) - int(left["sequence"])
                if delta != 1:
                    sequence_gaps.append({
                        "session": session,
                        "from_sequence": left["sequence"],
                        "to_sequence": right["sequence"],
                        "delta": delta,
                        "utc": right["_time"].isoformat(timespec="milliseconds"),
                    })

        def browser_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
            oks = [row for row in rows if row.get("event") == "send_ok"]
            counts = Counter(str(row.get("event")) for row in rows)
            failures = Counter(
                (str(row.get("status")), str(row.get("error")))
                for row in rows if row.get("event") in {"send_failed", "claim_failed"}
            )
            return {
                "event_counts": dict(sorted(counts.items())),
                "failure_details": [
                    {"status": status, "error": error, "count": count}
                    for (status, error), count in sorted(failures.items())
                ],
                "send_ok_rtt_ms": distribution(
                    value for row in oks if (value := fnum(row.get("rtt_ms"))) is not None
                ),
            }

        flight_counts = Counter(str(row.get("event")) for row in flight)
        metrics = {
            "archive": {"path": str(args.archive.resolve()), "sha256": outer_hash, "expected_sha256": EXPECTED_ARCHIVE_SHA256},
            "internal_hashes": internal_hashes,
            "record_counts": {
                "control": len(control), "commands": len(commands), "expiries": len(expiries),
                "flight": len(flight), "flight_events": dict(sorted(flight_counts.items())),
            },
            "control_event_counts": dict(sorted(Counter(str(row.get("event")) for row in control).items())),
            "existing_analyzer_reproduction": {
                "longest_continuous_armed_minutes": longest_armed_s / 60,
                "command_intervals": len(analyzer_gaps),
                "rtt_p95_ms": percentile(analyzer_rtts, 0.95),
                "rtt_max_ms": max(analyzer_rtts),
                "max_armed_command_gap_ms": max(analyzer_gaps),
                "heartbeat_expiries": len(expiries),
                "reported_rejected_commands": sum(1 for row in received if row.get("accepted") is False),
                "command_received_without_matching_accepted_command": len(unmatched_received),
                "unmatched_received": [
                    {key: (value.isoformat(timespec="milliseconds") if isinstance(value, datetime) else value)
                     for key, value in row.items() if not key.startswith("_") or key == "_time"}
                    for row in unmatched_received
                ],
            },
            "clock_domains": {
                "server": "UTC ISO-8601 generated by Tractor01 system clock",
                "mission": "UTC ISO-8601; aligns directly with server clock",
                "pursuit": "Unix epoch seconds; interpreted as UTC",
                "browser_event": "phone Date/new Date UTC wall clock plus performance.now monotonic_ms",
                "browser_upload": "server UTC receipt time of a later batched diagnostic upload; not event occurrence",
                "journal": "Tractor01 local EDT, one-second display resolution",
                "router": "AXE75 local EDT, one-second display resolution",
                "server_minus_phone_send_ms": distribution(phone_to_server),
                "browser_upload_delay_ms": distribution(upload_delays),
                "browser_alignment_offset_bound_ms": clock_offset_bound_ms,
            },
            "healthy_armed_commands_more_than_10s_from_expiry": command_metrics(healthy_commands),
            "armed_commands_within_2s_of_expiry": command_metrics(failure_commands),
            "all_armed_commands": command_metrics(armed),
            "healthy_browser_events_more_than_10s_from_expiry": browser_metrics(healthy_flight),
            "browser_events_within_2s_of_expiry": browser_metrics(failure_flight),
            "accepted_sequence_discontinuities": sequence_gaps,
            "router": router_summary(args.router_logs, expiry_times),
        }

        event_csv = out / "expiry_events.csv"
        with event_csv.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(event_rows[0]))
            writer.writeheader()
            writer.writerows(event_rows)

        timeline: list[dict[str, Any]] = []
        for row in control:
            if critical_start <= row["_time"] <= critical_end:
                clean = {k: v for k, v in row.items() if not k.startswith("_")}
                timeline.append({"_time": row["_time"], "source": "server", "event": row.get("event"), "details": compact(clean)})
        for row in flight:
            if critical_start <= row["_aligned_time"] <= critical_end:
                clean = {k: v for k, v in row.items() if not k.startswith("_")}
                timeline.append({"_time": row["_aligned_time"], "source": "browser", "event": row.get("event"), "details": compact(clean)})
        for row in pursuit_critical:
            keep = {key: row.get(key) for key in ("elapsed_s", "waypoint_idx", "driving", "wait_reason", "handheld_mode", "operator_link_stable_s", "speed_cmd_mps", "actual_speed_mps", "fix_quality", "head_valid", "heading_compass_deg", "reacquire_state")}
            timeline.append({"_time": row["_time"], "source": "pursuit", "event": "controller_sample", "details": compact(keep)})
        for row in mission_critical:
            keep = {key: row.get(key) for key in ("elapsed_sec", "jrk_target", "jrk_scaled_feedback", "trans_cmd_vel_mps", "trans_cmd_vel_age_ms", "trans_mode", "steer_state", "fix_quality", "head_valid", "heading_deg")}
            timeline.append({"_time": row["_time"], "source": "mission", "event": "combined_sample", "details": compact(keep)})
        for row in journal_critical:
            timeline.append({"_time": row["_time"], "source": "journal", "event": "journal_line", "details": row["line"]})
        timeline.sort(key=lambda row: (row["_time"], row["source"]))
        timeline_csv = out / "critical_window_timeline.csv"
        with timeline_csv.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["utc", "edt", "source", "event", "details"])
            writer.writeheader()
            for row in timeline:
                writer.writerow({
                    "utc": row["_time"].isoformat(timespec="milliseconds"),
                    "edt": row["_time"].astimezone(EDT).isoformat(timespec="milliseconds"),
                    "source": row["source"], "event": row["event"], "details": row["details"],
                })

        metrics_path = out / "metrics.json"
        metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({
            "archive_hash_ok": True,
            "internal_hashes_ok": True,
            "expiries": len(expiries),
            "outputs": [str(event_csv), str(timeline_csv), str(metrics_path)],
        }, indent=2))


if __name__ == "__main__":
    main()
