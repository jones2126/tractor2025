#!/usr/bin/env python3
"""Fail-closed mission preflight for tractor01.

Checks the systemd services, recent RTCM forwarding, the live dual-F9P UDP
state, heading validity, and the Teensy steering telemetry path.  The command
returns zero only when every mission-critical check passes.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import socket
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Any


# The Andon light is intentionally excluded while its hardware is disabled.
# It is an operator aid, not a navigation/control prerequisite.
SERVICES = ("rtcm-server.service", "teensy-bridge.service")
DEVICES = ("/dev/gps-base-link", "/dev/gps-heading", "/dev/teensy")
RTCM_RATE_RE = re.compile(r"Forwarded\s+(\d+)\s+bytes\s+\(~(\d+)\s+B/s\)")
RTCM_BAD_PATTERNS = (
    "connection refused",
    "tcp timeout",
    "connection closed",
    "reconnecting",
    "serial write incomplete",
    "fatal gps connection",
)


@dataclass
class Check:
    name: str
    passed: bool
    detail: str


def command_output(command: list[str], timeout: float = 5.0) -> tuple[int, str]:
    try:
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
        return result.returncode, result.stdout.strip()
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 127, str(exc)


def service_checks() -> list[Check]:
    checks = []
    for service in SERVICES:
        code, output = command_output(
            ["systemctl", "show", service, "--property=ActiveState,SubState", "--value"]
        )
        fields = output.splitlines()
        active = code == 0 and "active" in fields and "running" in fields
        checks.append(Check(service, active, output.replace("\n", "/") or "no status"))
    return checks


def device_checks() -> list[Check]:
    from pathlib import Path

    return [Check(device, Path(device).exists(), "present" if Path(device).exists() else "missing") for device in DEVICES]


def rtcm_journal_check(seconds: int) -> Check:
    code, journal = command_output(
        [
            "journalctl", "-u", "rtcm-server.service", "--since",
            f"{seconds} seconds ago", "--no-pager", "-o", "cat",
        ],
        timeout=8.0,
    )
    if code != 0:
        return Check("RTCM correction stream", False, f"journal unavailable: {journal}")

    rates = [(int(byte_count), int(rate)) for byte_count, rate in RTCM_RATE_RE.findall(journal)]
    bad_lines = [line.strip() for line in journal.splitlines() if any(p in line.lower() for p in RTCM_BAD_PATTERNS)]
    if bad_lines:
        return Check("RTCM correction stream", False, bad_lines[-1])
    if not rates:
        return Check(
            "RTCM correction stream",
            False,
            f"no forwarding report in the last {seconds}s; inspect rtcm-server journal",
        )
    byte_count, rate = rates[-1]
    return Check(
        "RTCM correction stream",
        byte_count > 0 and rate > 0,
        f"latest report: {byte_count} bytes at approximately {rate} B/s",
    )


def open_udp_listener(port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.bind(("", port))
    sock.settimeout(0.25)
    return sock


def collect_json_udp(port: int, seconds: float) -> list[tuple[float, dict[str, Any]]]:
    sock = open_udp_listener(port)
    deadline = time.monotonic() + seconds
    samples = []
    while time.monotonic() < deadline:
        try:
            payload, _ = sock.recvfrom(8192)
            parsed = json.loads(payload.decode("utf-8"))
            if isinstance(parsed, dict):
                samples.append((time.monotonic(), parsed))
        except socket.timeout:
            continue
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
    sock.close()
    return samples


def finite_number(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def gps_checks(
    samples: list[tuple[float, dict[str, Any]]],
    seconds: float,
    max_diff_age: float,
    max_heading_error: float,
    min_baseline: float,
    max_baseline: float,
) -> list[Check]:
    if not samples:
        return [Check("GPS UDP 6009", False, f"no packets received in {seconds:.1f}s")]

    elapsed = samples[-1][0] - samples[0][0]
    rate = (len(samples) - 1) / elapsed if elapsed > 0 and len(samples) > 1 else 0.0
    latest = samples[-1][1]
    fatal = bool(latest.get("fatal_error"))
    fix = str(latest.get("fix_quality", "Unknown"))
    diff_values = [float(item[1]["diff_age"]) for item in samples if finite_number(item[1].get("diff_age"))]
    diff_age = statistics.median(diff_values[-20:]) if diff_values else math.nan
    heading_values = [float(item[1]["heading_deg"]) for item in samples if finite_number(item[1].get("heading_deg"))]
    head_valid_count = sum(item[1].get("headValid") is True for item in samples)
    head_valid_fraction = head_valid_count / len(samples)
    fixed_count = sum(str(item[1].get("carrier", "none")).lower() == "fixed" for item in samples)
    fixed_fraction = fixed_count / len(samples)

    relpos_counts = [int(item[1]["relposned_count"]) for item in samples if finite_number(item[1].get("relposned_count"))]
    relpos_delta = max(0, relpos_counts[-1] - relpos_counts[0]) if len(relpos_counts) >= 2 else 0
    relpos_rate = relpos_delta / elapsed if elapsed > 0 else 0.0
    gnss_fix_fraction = sum(item[1].get("relpos_gnss_fix_ok") is True for item in samples) / len(samples)
    diff_solution_fraction = sum(item[1].get("relpos_diff_solution") is True for item in samples) / len(samples)
    relpos_valid_fraction = sum(item[1].get("relpos_valid") is True for item in samples) / len(samples)
    moving_fraction = sum(item[1].get("relpos_moving") is True for item in samples) / len(samples)
    normalized_fraction = sum(item[1].get("relpos_normalized") is True for item in samples) / len(samples)
    no_ref_miss_fraction = sum(
        item[1].get("relpos_ref_pos_miss") is False
        and item[1].get("relpos_ref_obs_miss") is False
        for item in samples
    ) / len(samples)
    baseline_values = [
        float(item[1]["relpos_length_m"])
        for item in samples
        if finite_number(item[1].get("relpos_length_m"))
    ]
    baseline = statistics.median(baseline_values[-20:]) if baseline_values else math.nan
    heading_error_values = [
        float(item[1]["relpos_heading_accuracy_deg"])
        for item in samples
        if finite_number(item[1].get("relpos_heading_accuracy_deg"))
    ]
    heading_error = statistics.median(heading_error_values[-20:]) if heading_error_values else math.nan

    checks = [
        Check("GPS UDP 6009", rate >= 15.0, f"{len(samples)} packets, approximately {rate:.2f} Hz"),
        Check(
            "GPS device health",
            not fatal,
            "no fatal device error" if not fatal else f"base={latest.get('fatal_base_reason')}; heading={latest.get('fatal_heading_reason')}",
        ),
        Check(
            "NAV-RELPOSNED stream",
            len(relpos_counts) >= 2 and relpos_rate >= 5.0,
            (
                f"counter advanced by {relpos_delta}, approximately {relpos_rate:.2f} Hz"
                if relpos_counts
                else "RELPOSNED counter is absent; update/restart rtcm-server"
            ),
        ),
        Check(
            "RELPOSNED solution flags",
            (
                gnss_fix_fraction >= 0.90
                and diff_solution_fraction >= 0.90
                and relpos_valid_fraction >= 0.90
                and moving_fraction >= 0.90
                and normalized_fraction >= 0.90
                and no_ref_miss_fraction >= 0.90
            ),
            (
                f"fixOK={gnss_fix_fraction:.1%}; diff={diff_solution_fraction:.1%}; "
                f"relPosValid={relpos_valid_fraction:.1%}; moving={moving_fraction:.1%}; "
                f"normalized={normalized_fraction:.1%}; reference data present={no_ref_miss_fraction:.1%}"
            ),
        ),
        Check("RTK fix quality", fix == "RTK Fixed", fix),
        Check(
            "Differential correction age",
            bool(diff_values) and diff_age <= max_diff_age,
            f"median recent diff_age={diff_age:.2f}s" if diff_values else "diff_age is absent",
        ),
        Check(
            "Heading GPS output",
            bool(heading_values) and head_valid_fraction >= 0.90,
            (
                f"heading={heading_values[-1]:.3f} deg; headValid in {head_valid_fraction:.1%} of packets"
                if heading_values else "heading_deg is absent"
            ),
        ),
        Check(
            "Heading carrier solution",
            fixed_fraction >= 0.90,
            f"fixed in {fixed_fraction:.1%} of packets",
        ),
        Check(
            "Heading baseline length",
            bool(baseline_values) and min_baseline <= baseline <= max_baseline,
            (
                f"median recent length={baseline:.4f} m (expected {min_baseline:.2f}-{max_baseline:.2f} m)"
                if baseline_values else "baseline length is absent"
            ),
        ),
        Check(
            "Heading accuracy estimate",
            bool(heading_error_values) and heading_error <= max_heading_error,
            (
                f"median recent accuracy={heading_error:.3f} deg (maximum {max_heading_error:.2f} deg)"
                if heading_error_values else "heading accuracy is absent"
            ),
        ),
    ]
    return checks


def collect_gps_until_ready(
    sample_seconds: float,
    wait_seconds: float,
    max_diff_age: float,
    max_heading_error: float,
    min_baseline: float,
    max_baseline: float,
) -> tuple[list[tuple[float, dict[str, Any]]], list[Check]]:
    deadline = time.monotonic() + max(sample_seconds, wait_seconds)
    attempt = 0
    samples: list[tuple[float, dict[str, Any]]] = []
    checks: list[Check] = []

    while True:
        attempt += 1
        remaining = deadline - time.monotonic()
        window = min(sample_seconds, max(0.0, remaining))
        if window <= 0:
            break
        samples = collect_json_udp(6009, window)
        checks = gps_checks(
            samples,
            window,
            max_diff_age,
            max_heading_error,
            min_baseline,
            max_baseline,
        )
        failures = [check.name for check in checks if not check.passed]
        if not failures:
            if attempt > 1:
                print(f"  GPS/heading became ready on check {attempt}.")
            break
        if "GPS device health" in failures:
            print("  GPS server reports a fatal device error; reconnect the receiver and restart rtcm-server.")
            break
        if time.monotonic() >= deadline:
            break
        print(f"  Check {attempt}: waiting on {', '.join(failures)}...")

    return samples, checks


def steering_checks(samples: list[tuple[float, dict[str, Any]]], seconds: float) -> list[Check]:
    unique: dict[int, tuple[float, dict[str, Any]]] = {}
    for received, message in samples:
        steering = message.get("steering", {})
        try:
            sequence = int(steering.get("sequence", 0))
        except (TypeError, ValueError):
            sequence = 0
        if sequence > 0:
            unique.setdefault(sequence, (received, message))

    ordered = [unique[key] for key in sorted(unique)]
    if len(ordered) < 2:
        return [Check("Steering UDP 6003", False, f"fewer than two positive sequences in {seconds:.1f}s")]
    elapsed = ordered[-1][0] - ordered[0][0]
    rate = (len(ordered) - 1) / elapsed if elapsed > 0 else 0.0
    latest_message = ordered[-1][1]
    steering = latest_message.get("steering", {})
    transmission = latest_message.get("transmission", {})
    steering_mode = steering.get("mode")
    transmission_mode = transmission.get("mode")
    state = str(steering.get("state", "UNKNOWN"))
    pwm = steering.get("pwm")
    paused = steering_mode == 0 and transmission_mode == 0
    return [
        Check("Steering telemetry", rate >= 18.0, f"{len(ordered)} unique sequences, approximately {rate:.2f} Hz"),
        Check(
            "Safe starting mode",
            paused,
            f"steering mode={steering_mode}; transmission mode={transmission_mode}; state={state}; pwm={pwm}",
        ),
    ]


def print_checks(checks: list[Check]) -> bool:
    width = max(len(check.name) for check in checks)
    for check in checks:
        label = "PASS" if check.passed else "FAIL"
        print(f"[{label}] {check.name:<{width}}  {check.detail}")
    return all(check.passed for check in checks)


def main() -> int:
    parser = argparse.ArgumentParser(description="Fail-closed tractor mission preflight")
    parser.add_argument("--sample-seconds", type=float, default=5.0)
    parser.add_argument("--heading-wait-seconds", type=float, default=60.0)
    parser.add_argument("--journal-seconds", type=int, default=30)
    parser.add_argument("--max-diff-age", type=float, default=5.0)
    parser.add_argument("--max-heading-error", type=float, default=1.0)
    parser.add_argument("--min-baseline", type=float, default=0.8)
    parser.add_argument("--max-baseline", type=float, default=1.3)
    args = parser.parse_args()
    if args.sample_seconds <= 0 or args.heading_wait_seconds <= 0:
        parser.error("sample and heading wait times must be positive")
    if args.min_baseline <= 0 or args.max_baseline < args.min_baseline:
        parser.error("baseline limits must be positive and ordered")

    print("Tractor01 mission preflight")
    print("Keep the tractor in Pause with blades disengaged.\n")

    checks = []
    checks.extend(service_checks())
    checks.extend(device_checks())
    checks.append(rtcm_journal_check(args.journal_seconds))

    print(
        f"Waiting up to {args.heading_wait_seconds:.0f}s for a stable "
        f"{args.sample_seconds:.1f}s GPS/heading window on UDP 6009..."
    )
    _, gps_results = collect_gps_until_ready(
        args.sample_seconds,
        args.heading_wait_seconds,
        args.max_diff_age,
        args.max_heading_error,
        args.min_baseline,
        args.max_baseline,
    )
    checks.extend(gps_results)

    print(f"Sampling steering UDP 6003 for {args.sample_seconds:.1f}s...\n")
    steering_samples = collect_json_udp(6003, args.sample_seconds)
    checks.extend(steering_checks(steering_samples, args.sample_seconds))

    passed = print_checks(checks)
    print()
    if passed:
        print("MISSION PREFLIGHT PASS")
        return 0
    print("MISSION PREFLIGHT FAIL — do not select Auto.")
    print("Inspect: sudo journalctl -u rtcm-server.service --since '2 minutes ago' --no-pager -l")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
