#!/usr/bin/env python3
"""Fail-closed mission preflight for tractor01.

Checks the systemd services, recent RTCM forwarding, the live dual-F9P UDP
state, stationary ground-speed data, and the Teensy steering/transmission
telemetry paths. An optional expected-firmware argument lets a mission require
the matching flashed Teensy build. The command returns zero only when every
mission-critical check passes.
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
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
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
REQUIRED_DATA_FRACTION = 0.90
DEFAULT_MAX_STATIONARY_SPEED_MPS = 0.20
DEFAULT_NEUTRAL_JRK_TARGET = 2836
MIN_JRK_DIAGNOSTIC_RATE_HZ = 2.0
MIN_GROUND_SPEED_RATE_HZ = 4.0
FIRMWARES_WITH_JRK_CURRENT_TELEMETRY = {
    "teensy_main_20260908_1p8_test",
    "teensy_main_20260914",
}


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
    max_stationary_speed: float,
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
    speed_values = [
        float(item[1]["speed_mps"])
        for item in samples
        if finite_number(item[1].get("speed_mps"))
    ]
    speed_fraction = len(speed_values) / len(samples)
    stationary_speed = statistics.median(speed_values[-20:]) if speed_values else math.nan
    speed_counts = [
        int(float(item[1]["speed_update_count"]))
        for item in samples
        if finite_number(item[1].get("speed_update_count"))
    ]
    speed_delta = max(0, speed_counts[-1] - speed_counts[0]) if len(speed_counts) >= 2 else 0
    speed_rate = speed_delta / elapsed if elapsed > 0 else 0.0
    speed_source = latest.get("speed_source")
    speed_timestamp = latest.get("speed_timestamp")

    checks = [
        Check("GPS UDP 6009", rate >= 15.0, f"{len(samples)} packets, approximately {rate:.2f} Hz"),
        Check(
            "GPS device health",
            not fatal,
            "no fatal device error" if not fatal else f"base={latest.get('fatal_base_reason')}; heading={latest.get('fatal_heading_reason')}",
        ),
        Check(
            "NAV-RELPOSNED stream",
            len(relpos_counts) >= 2 and relpos_rate >= 4.0,
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
            "GPS ground-speed stream",
            (
                speed_fraction >= REQUIRED_DATA_FRACTION
                and len(speed_counts) >= 2
                and speed_rate >= MIN_GROUND_SPEED_RATE_HZ
            ),
            (
                f"finite in {speed_fraction:.1%} of packets; {speed_source or 'unknown'} "
                f"counter advanced by {speed_delta}, approximately {speed_rate:.2f} Hz; "
                f"latest update={speed_timestamp or 'unknown'}"
                if speed_values
                else "speed_mps is absent; confirm VTG or valid RMC output"
            ),
        ),
        Check(
            "Stationary ground speed",
            bool(speed_values) and 0.0 <= stationary_speed <= max_stationary_speed,
            (
                f"median recent speed={stationary_speed:.3f} m/s "
                f"(maximum {max_stationary_speed:.2f} m/s while in Pause)"
                if speed_values
                else "speed_mps is absent"
            ),
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


def numeric_sample_summary(
    samples: list[tuple[float, dict[str, Any]]],
    key: str,
    digits: int = 1,
) -> str:
    values = [float(message[key]) for _, message in samples if finite_number(message.get(key))]
    if not values:
        return "absent"
    return (
        f"latest={values[-1]:.{digits}f}; median={statistics.median(values):.{digits}f}; "
        f"range={min(values):.{digits}f}-{max(values):.{digits}f}"
    )


def automatic_gps_failure_diagnostics(
    samples: list[tuple[float, dict[str, Any]]],
    gps_results: list[Check],
    all_checks: list[Check],
    journal_seconds: int,
) -> None:
    """Print read-only diagnostics after a GPS/heading pre-flight failure."""
    failed = [check.name for check in gps_results if not check.passed]
    if not failed:
        return

    print("\n===== AUTOMATIC GPS FAILURE DIAGNOSTICS =====")
    print(f"Triggered by: {', '.join(failed)}")
    print("No receiver settings or services are changed by these diagnostics.")

    for device in ("/dev/gps-base-link", "/dev/gps-heading"):
        path = Path(device)
        if path.exists():
            print(f"Device: {device} -> {path.resolve()}")
        else:
            print(f"Device: {device} is missing")

    if not samples:
        print("UDP 6009 supplied no usable JSON samples.")
        print("Likely area: rtcm-server publication, service health, or UDP binding.")
    else:
        messages = [message for _, message in samples]
        latest = messages[-1]
        fixes = Counter(str(message.get("fix_quality", "Unknown")) for message in messages)
        carriers = Counter(str(message.get("carrier", "none")).lower() for message in messages)
        diff_present = sum(finite_number(message.get("diff_age")) for message in messages)
        head_valid_fraction = sum(message.get("headValid") is True for message in messages) / len(messages)
        fixed_heading_fraction = sum(
            str(message.get("carrier", "none")).lower() == "fixed" for message in messages
        ) / len(messages)
        relpos_counts = [
            int(float(message["relposned_count"]))
            for message in messages
            if finite_number(message.get("relposned_count"))
        ]
        elapsed = samples[-1][0] - samples[0][0]
        relpos_delta = max(0, relpos_counts[-1] - relpos_counts[0]) if len(relpos_counts) >= 2 else 0
        relpos_rate = relpos_delta / elapsed if elapsed > 0 else 0.0

        print(f"Samples analyzed: {len(samples)}")
        print("Position fix distribution: " + ", ".join(f"{name}={count}" for name, count in sorted(fixes.items())))
        print(f"Base satellites used (NAV): {numeric_sample_summary(samples, 'base_numSV_used', 0)}")
        print(f"Base satellites used (GGA): {numeric_sample_summary(samples, 'base_numSV_used_gga', 0)}")
        print(f"Base satellites visible: {numeric_sample_summary(samples, 'base_numSV_visible', 0)}")
        print(f"Base mean C/N0: {numeric_sample_summary(samples, 'base_cno_mean_dbhz', 1)} dB-Hz")
        print(f"Base HDOP: {numeric_sample_summary(samples, 'hdop', 2)}")
        print(f"Correction age present: {diff_present}/{len(messages)} samples")
        if diff_present:
            print(f"Correction age: {numeric_sample_summary(samples, 'diff_age', 2)} seconds")
        print(
            f"Heading: headValid={head_valid_fraction:.1%}; "
            f"carrier distribution={dict(sorted(carriers.items()))}; "
            f"RELPOSNED approximately {relpos_rate:.2f} Hz"
        )
        print(f"Heading satellites used: {numeric_sample_summary(samples, 'heading_numSV_used', 0)}")
        print(f"Heading satellites visible: {numeric_sample_summary(samples, 'heading_numSV_visible', 0)}")
        print(f"Heading mean C/N0: {numeric_sample_summary(samples, 'heading_cno_mean_dbhz', 1)} dB-Hz")

        latest_fix = str(latest.get("fix_quality", "Unknown"))
        rtcm_ok = any(check.name == "RTCM correction stream" and check.passed for check in all_checks)
        heading_healthy = (
            head_valid_fraction >= REQUIRED_DATA_FRACTION
            and fixed_heading_fraction >= REQUIRED_DATA_FRACTION
            and relpos_rate >= 4.0
        )
        base_satellites = latest.get("base_numSV_used")
        if not finite_number(base_satellites):
            base_satellites = latest.get("base_numSV_used_gga")
        hdop = latest.get("hdop")

        print("\nAutomatic assessment:")
        if bool(latest.get("fatal_error")):
            print(
                "- GPS connection failure reported: "
                f"base={latest.get('fatal_base_reason')}; "
                f"heading={latest.get('fatal_heading_reason')}"
            )
        if heading_healthy and latest_fix != "RTK Fixed":
            print("- The heading receiver appears configured and healthy; do not reconfigure it from this result.")
            print(f"- The position/base-link receiver is reporting {latest_fix}, not RTK Fixed.")
        elif not heading_healthy:
            print("- The heading stream is not fully healthy; inspect its antenna, cable, device link, and configuration.")
            if relpos_rate < 1.0:
                print("- NAV-RELPOSNED is absent or too slow. A lost heading-F9P configuration is a likely cause when its satellite data is otherwise present.")
                print("- Safe configuration readback requires stopping rtcm-server first; do not run it while the service owns the serial port.")
                print("  sudo systemctl stop rtcm-server.service")
                print("  sudo python3 tractor_rpi/testing/configure_heading_f9p_20260727.py --port /dev/gps-heading")
                print("  sudo systemctl start rtcm-server.service")
        if rtcm_ok and latest_fix != "RTK Fixed":
            print("- RTCM bytes are being forwarded, but forwarding alone does not prove an RTK-fixed position solution.")
        if diff_present == 0:
            print("- The base receiver's GGA messages omit differential-correction age.")
        if finite_number(base_satellites) and float(base_satellites) < 10:
            print(f"- Only {int(float(base_satellites))} base satellites are used; check sky view and the position antenna connection.")
        if finite_number(hdop) and float(hdop) > 1.5:
            print(f"- Base HDOP is {float(hdop):.2f}; check for obstruction or multipath before driving.")
        if latest_fix in ("DGPS", "RTK Float"):
            print("- Keep the tractor in Pause under open sky, allow several minutes to converge, then rerun pre-flight.")

    code, journal = command_output(
        [
            "journalctl", "-u", "rtcm-server.service", "--since",
            f"{journal_seconds} seconds ago", "--no-pager", "-o", "cat",
        ],
        timeout=8.0,
    )
    print(f"\nRelevant rtcm-server journal lines from the last {journal_seconds} seconds:")
    if code != 0:
        print(f"- Journal unavailable: {journal}")
    else:
        terms = RTCM_BAD_PATTERNS + (
            "error", "warning", "fatal", "disconnect", "checksum", "overflow", "opened",
        )
        relevant = [
            line.strip() for line in journal.splitlines()
            if line.strip() and any(term in line.lower() for term in terms)
        ]
        if relevant:
            for line in relevant[-12:]:
                print(f"- {line}")
        else:
            print("- No matching error, warning, disconnect, or reconnect lines found.")
    print("===== END AUTOMATIC GPS FAILURE DIAGNOSTICS =====")


def collect_gps_until_ready(
    sample_seconds: float,
    wait_seconds: float,
    max_diff_age: float,
    max_heading_error: float,
    min_baseline: float,
    max_baseline: float,
    max_stationary_speed: float,
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
            max_stationary_speed,
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


def steering_checks(
    samples: list[tuple[float, dict[str, Any]]],
    seconds: float,
    neutral_jrk_target: int,
    expected_firmware: str | None = None,
) -> list[Check]:
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
    system = latest_message.get("system", {})

    transmission_samples = [message.get("transmission", {}) for _, message in ordered]
    required_transmission_fields = (
        "target",
        "current",
        "actual_target",
        "scaled_feedback",
        "duty_cycle_target",
        "duty_cycle",
        "errors_halting",
        "jrk_sequence",
        "jrk_valid",
        "jrk_read_latency_ms",
        "jrk_timeouts",
        "cmd_vel_mps",
    )
    complete_count = sum(
        all(finite_number(trans.get(field)) for field in required_transmission_fields)
        for trans in transmission_samples
    )
    complete_fraction = complete_count / len(transmission_samples)

    jrk_sequences = [
        int(float(trans["jrk_sequence"]))
        for trans in transmission_samples
        if finite_number(trans.get("jrk_sequence"))
    ]
    jrk_delta = max(0, jrk_sequences[-1] - jrk_sequences[0]) if len(jrk_sequences) >= 2 else 0
    jrk_rate = jrk_delta / elapsed if elapsed > 0 else 0.0
    jrk_valid_fraction = sum(
        finite_number(trans.get("jrk_valid")) and int(float(trans["jrk_valid"])) == 1
        for trans in transmission_samples
    ) / len(transmission_samples)
    timeout_values = [
        int(float(trans["jrk_timeouts"]))
        for trans in transmission_samples
        if finite_number(trans.get("jrk_timeouts"))
    ]
    timeout_delta = max(timeout_values) - min(timeout_values) if timeout_values else 0
    active_error_values = [
        int(float(trans["errors_halting"]))
        for trans in transmission_samples
        if finite_number(trans.get("errors_halting"))
    ]
    no_active_errors = (
        len(active_error_values) / len(transmission_samples) >= REQUIRED_DATA_FRACTION
        and all(value == 0 for value in active_error_values)
    )

    steering_mode = steering.get("mode")
    transmission_mode = transmission.get("mode")
    state = str(steering.get("state", "UNKNOWN"))
    pwm = steering.get("pwm")
    requested_target = transmission.get("target")
    actual_target = transmission.get("actual_target")
    paused = (
        steering_mode == 2
        and transmission_mode == 2
        and state == "PAUSE"
        and finite_number(pwm)
        and float(pwm) == 0.0
        and finite_number(requested_target)
        and int(float(requested_target)) == neutral_jrk_target
        and finite_number(actual_target)
        and int(float(actual_target)) == neutral_jrk_target
    )
    checks = [
        Check("Steering telemetry", rate >= 18.0, f"{len(ordered)} unique sequences, approximately {rate:.2f} Hz"),
        Check(
            "Transmission telemetry data",
            complete_fraction >= REQUIRED_DATA_FRACTION,
            (
                f"all calibration fields finite in {complete_fraction:.1%} of samples"
                if complete_fraction >= REQUIRED_DATA_FRACTION
                else f"complete in only {complete_fraction:.1%}; required fields: "
                + ", ".join(required_transmission_fields)
            ),
        ),
        Check(
            "JRK diagnostic stream",
            len(jrk_sequences) >= 2 and jrk_rate >= MIN_JRK_DIAGNOSTIC_RATE_HZ,
            f"counter advanced by {jrk_delta}, approximately {jrk_rate:.2f} Hz",
        ),
        Check(
            "JRK diagnostic validity",
            jrk_valid_fraction >= REQUIRED_DATA_FRACTION and timeout_delta == 0,
            (
                f"valid in {jrk_valid_fraction:.1%} of samples; "
                f"timeouts increased by {timeout_delta}"
            ),
        ),
        Check(
            "JRK active errors",
            no_active_errors,
            "none" if no_active_errors else f"halting error values: {sorted(set(active_error_values))}",
        ),
        Check(
            "Safe starting mode",
            paused,
            (
                f"steering mode={steering_mode}; transmission mode={transmission_mode}; "
                f"state={state}; steering pwm={pwm}; JRK requested/actual target="
                f"{requested_target}/{actual_target} (neutral {neutral_jrk_target})"
            ),
        ),
    ]
    if expected_firmware is not None:
        observed_firmware = system.get("firmware")
        checks.append(
            Check(
                "Teensy firmware identity",
                observed_firmware == expected_firmware,
                f"expected={expected_firmware}; observed={observed_firmware}",
            )
        )
    if expected_firmware in FIRMWARES_WITH_JRK_CURRENT_TELEMETRY:
        current_samples = [
            float(trans["motor_current_mA"])
            for trans in transmission_samples
            if finite_number(trans.get("motor_current_mA"))
            and finite_number(trans.get("motor_current_valid"))
            and int(float(trans["motor_current_valid"])) == 1
        ]
        peak_samples = [
            float(trans["peak_motor_current_mA"])
            for trans in transmission_samples
            if finite_number(trans.get("peak_motor_current_mA"))
            and finite_number(trans.get("motor_current_valid"))
            and int(float(trans["motor_current_valid"])) == 1
        ]
        current_fraction = len(current_samples) / len(transmission_samples)
        checks.append(
            Check(
                "JRK motor-current telemetry",
                current_fraction >= REQUIRED_DATA_FRACTION and bool(peak_samples),
                (
                    f"valid in {current_fraction:.1%} of samples; "
                    f"median current={statistics.median(current_samples) / 1000.0:.3f} A; "
                    f"maximum recent peak={max(peak_samples) / 1000.0:.3f} A"
                    if current_samples and peak_samples
                    else f"valid in only {current_fraction:.1%} of samples"
                ),
            )
        )
    return checks


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
    parser.add_argument(
        "--diagnostic-journal-seconds",
        type=int,
        default=180,
        help="journal history included automatically after a GPS failure (default 180s)",
    )
    parser.add_argument("--max-diff-age", type=float, default=5.0)
    parser.add_argument("--max-heading-error", type=float, default=1.0)
    parser.add_argument("--min-baseline", type=float, default=0.8)
    parser.add_argument("--max-baseline", type=float, default=1.3)
    parser.add_argument(
        "--max-stationary-speed",
        type=float,
        default=DEFAULT_MAX_STATIONARY_SPEED_MPS,
        help=(
            "maximum median GPS ground speed allowed while preflight requires Pause "
            f"(default {DEFAULT_MAX_STATIONARY_SPEED_MPS:.2f} m/s)"
        ),
    )
    parser.add_argument(
        "--neutral-jrk-target",
        type=int,
        default=DEFAULT_NEUTRAL_JRK_TARGET,
        help=f"expected requested/read-back JRK target in Pause (default {DEFAULT_NEUTRAL_JRK_TARGET})",
    )
    parser.add_argument(
        "--expected-firmware",
        help="require this firmware identity from the Teensy startup telemetry",
    )
    args = parser.parse_args()
    if args.sample_seconds <= 0 or args.heading_wait_seconds <= 0:
        parser.error("sample and heading wait times must be positive")
    if args.journal_seconds <= 0 or args.diagnostic_journal_seconds <= 0:
        parser.error("journal time windows must be positive")
    if args.min_baseline <= 0 or args.max_baseline < args.min_baseline:
        parser.error("baseline limits must be positive and ordered")
    if args.max_stationary_speed < 0:
        parser.error("maximum stationary speed must not be negative")
    if args.neutral_jrk_target < 0:
        parser.error("neutral JRK target must not be negative")

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
    gps_samples, gps_results = collect_gps_until_ready(
        args.sample_seconds,
        args.heading_wait_seconds,
        args.max_diff_age,
        args.max_heading_error,
        args.min_baseline,
        args.max_baseline,
        args.max_stationary_speed,
    )
    checks.extend(gps_results)

    print(f"Sampling steering UDP 6003 for {args.sample_seconds:.1f}s...\n")
    steering_samples = collect_json_udp(6003, args.sample_seconds)
    checks.extend(
        steering_checks(
            steering_samples,
            args.sample_seconds,
            args.neutral_jrk_target,
            args.expected_firmware,
        )
    )

    passed = print_checks(checks)
    print()
    if passed:
        print("MISSION PREFLIGHT PASS")
        return 0
    print("MISSION PREFLIGHT FAIL — do not select Auto.")
    automatic_gps_failure_diagnostics(
        gps_samples,
        gps_results,
        checks,
        args.diagnostic_journal_seconds,
    )
    if not any(not check.passed for check in gps_results):
        print("GPS/heading passed; inspect the failed service, device, or control check above.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
