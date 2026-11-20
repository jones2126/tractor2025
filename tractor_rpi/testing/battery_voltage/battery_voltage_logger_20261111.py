#!/usr/bin/env python3
"""Continuously log read-only JRK VIN telemetry from UDP 6003.

This process never sends a command. It samples the existing Teensy bridge
broadcast into a small daily CSV independently of mission logging.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
from pathlib import Path
import socket
import time


FIELDS = (
    "time",
    "boot_id",
    "system_uptime_s",
    "telemetry_age_s",
    "vin_voltage_mV",
    "vin_voltage_valid",
    "firmware",
    "steering_mode",
    "transmission_mode",
    "jrk_sequence",
    "jrk_timeouts",
    "jrk_errors_halting",
    "voltage_context_hint",
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=6003)
    parser.add_argument("--interval-seconds", type=float, default=10.0)
    parser.add_argument("--retention-days", type=int, default=90)
    parser.add_argument("--output-dir", type=Path, default=Path("/var/lib/tractor-battery"))
    return parser.parse_args()


def voltage_context_hint(millivolts, valid):
    if not valid:
        return "invalid"
    if millivolts >= 13_800:
        return "charging_source_detected"
    if millivolts >= 12_000:
        return "no_charging_source_detected"
    return "low_voltage_observed"


def extract_row(message, *, now, telemetry_age_s, boot_id="", system_uptime_s=""):
    transmission = message.get("transmission", {})
    steering = message.get("steering", {})
    system = message.get("system", {})
    millivolts = transmission.get("vin_voltage_mV")
    valid = transmission.get("vin_voltage_valid")
    try:
        millivolts = int(millivolts)
        valid = (
            int(valid) == 1
            and 0 < millivolts < 100_000
            and telemetry_age_s <= 2.0
        )
    except (TypeError, ValueError):
        millivolts = ""
        valid = False
    return {
        "time": now.isoformat(timespec="milliseconds"),
        "boot_id": boot_id,
        "system_uptime_s": system_uptime_s,
        "telemetry_age_s": f"{max(0.0, telemetry_age_s):.3f}",
        "vin_voltage_mV": millivolts,
        "vin_voltage_valid": int(valid),
        "firmware": system.get("firmware", ""),
        "steering_mode": steering.get("mode", ""),
        "transmission_mode": transmission.get("mode", ""),
        "jrk_sequence": transmission.get("jrk_sequence", ""),
        "jrk_timeouts": transmission.get("jrk_timeouts", ""),
        "jrk_errors_halting": transmission.get("errors_halting", ""),
        "voltage_context_hint": voltage_context_hint(millivolts, valid),
    }


def append_row(output_dir, row):
    output_dir.mkdir(parents=True, exist_ok=True)
    day = dt.datetime.fromisoformat(row["time"]).date().isoformat()
    path = output_dir / f"tractor_battery_{day}.csv"
    needs_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        if needs_header:
            writer.writeheader()
        writer.writerow(row)
    return path


def apply_retention(output_dir, *, today, retention_days):
    """Remove only this logger's daily CSV files older than the retention window."""
    cutoff = today - dt.timedelta(days=retention_days)
    removed = []
    for path in output_dir.glob("tractor_battery_????-??-??.csv"):
        try:
            file_day = dt.date.fromisoformat(path.stem.removeprefix("tractor_battery_"))
        except ValueError:
            continue
        if file_day < cutoff:
            path.unlink()
            removed.append(path)
    return removed


def main():
    args = parse_args()
    if not math.isfinite(args.interval_seconds) or args.interval_seconds < 1.0:
        raise SystemExit("--interval-seconds must be finite and at least 1.0")
    if args.retention_days < 1:
        raise SystemExit("--retention-days must be at least 1")

    try:
        boot_id = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
    except OSError:
        boot_id = ""

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    receiver.bind(("", args.port))
    receiver.settimeout(1.0)

    latest = None
    latest_monotonic = None
    next_write = time.monotonic()
    while True:
        try:
            payload, _ = receiver.recvfrom(65_535)
            candidate = json.loads(payload.decode("utf-8"))
            if isinstance(candidate, dict):
                latest = candidate
                latest_monotonic = time.monotonic()
        except socket.timeout:
            pass
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue

        now_monotonic = time.monotonic()
        if latest is not None and now_monotonic >= next_write:
            now = dt.datetime.now().astimezone()
            try:
                uptime_s = f"{float(Path('/proc/uptime').read_text(encoding='ascii').split()[0]):.2f}"
            except (OSError, ValueError, IndexError):
                uptime_s = ""
            row = extract_row(
                latest,
                now=now,
                telemetry_age_s=now_monotonic - latest_monotonic,
                boot_id=boot_id,
                system_uptime_s=uptime_s,
            )
            path = append_row(args.output_dir, row)
            removed = apply_retention(
                args.output_dir, today=now.date(), retention_days=args.retention_days
            )
            print(
                f"{row['time']} voltage_mV={row['vin_voltage_mV']} "
                f"valid={row['vin_voltage_valid']} file={path}",
                flush=True,
            )
            for expired in removed:
                print(f"retention removed {expired}", flush=True)
            next_write = now_monotonic + args.interval_seconds


if __name__ == "__main__":
    main()
