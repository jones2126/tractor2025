#!/usr/bin/env python3
"""Verify that July/August steering telemetry reaches UDP 6003 at 20 Hz."""

import argparse
import json
import socket
import statistics
import time


REQUIRED_FIELDS = {
    "sequence", "teensy_timestamp_ms", "state", "setpoint", "current",
    "error", "direction", "pwm", "left_pwm", "right_pwm",
    "normalized_command", "pid_active", "deadband_active",
    "min_pwm_clamped", "pwm_saturated", "pid_dt_s", "integral_sum",
    "error_derivative", "p_term", "i_term", "d_term", "pid_output",
    "cmd_age_ms",
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--port", type=int, default=6003)
    parser.add_argument("--minimum-hz", type=float, default=18.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.bind(("", args.port))
    sock.settimeout(0.25)

    deadline = time.monotonic() + args.seconds
    samples = {}
    malformed = 0
    while time.monotonic() < deadline:
        try:
            payload, _ = sock.recvfrom(8192)
            message = json.loads(payload.decode("utf-8"))
            steer = message.get("steering", {})
            sequence = int(steer.get("sequence", 0))
            if sequence > 0:
                samples.setdefault(sequence, (time.monotonic(), steer))
        except socket.timeout:
            continue
        except (ValueError, UnicodeDecodeError, json.JSONDecodeError):
            malformed += 1
    sock.close()

    ordered = [samples[key] for key in sorted(samples)]
    if len(ordered) < 2:
        print("FAIL: fewer than two positive, unique steering sequences received.")
        print("The Teensy is probably still running the older telemetry firmware.")
        return 1

    first_q, last_q = min(samples), max(samples)
    elapsed = ordered[-1][0] - ordered[0][0]
    measured_hz = (len(ordered) - 1) / elapsed if elapsed > 0 else 0.0
    teensy_times = [int(item[1].get("teensy_timestamp_ms", 0)) for item in ordered]
    teensy_steps = [b - a for a, b in zip(teensy_times, teensy_times[1:]) if b > a]
    median_step = statistics.median(teensy_steps) if teensy_steps else float("nan")
    missing = sorted(REQUIRED_FIELDS - set(ordered[-1][1]))

    print(f"Unique sequences : {len(ordered)}")
    print(f"Sequence range   : {first_q}..{last_q}")
    print(f"Measured rate    : {measured_hz:.2f} Hz")
    print(f"Median Teensy dt : {median_step:.1f} ms")
    print(f"Malformed packets: {malformed}")
    print(f"Last state       : {ordered[-1][1].get('state')}")
    print(f"Last PID active  : {ordered[-1][1].get('pid_active')}")
    print(f"Missing fields   : {missing or 'none'}")

    failures = []
    if measured_hz < args.minimum_hz:
        failures.append(f"rate {measured_hz:.2f} Hz is below {args.minimum_hz:.2f} Hz")
    if not 40.0 <= median_step <= 65.0:
        failures.append(f"median Teensy interval {median_step:.1f} ms is not near 50 ms")
    if missing:
        failures.append("required extended fields are missing")
    if ordered[-1][1].get("state") == "UNKNOWN":
        failures.append("state is UNKNOWN, indicating an older source format")

    if failures:
        print("FAIL: " + "; ".join(failures))
        return 1
    print("PASS: complete source-sequenced steering telemetry is arriving at 20 Hz.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
