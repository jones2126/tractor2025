#!/usr/bin/env python3
"""Send the RTK base battery voltage to ntfy every 30 minutes.

Run manually:
    python3 battery_voltage_ntfy_30min.py

Send one test notice and exit:
    python3 battery_voltage_ntfy_30min.py --once

Stop the repeating script with Ctrl+C.
"""

from __future__ import annotations

import argparse
import csv
import socket
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

import esp32_downloader_20260623 as esp32


NTFY_TOPIC = "rpi-rtkbase-jones2126"
NTFY_URL = f"https://ntfy.sh/{NTFY_TOPIC}"
DEFAULT_INTERVAL_MINUTES = 30


def read_latest_battery_voltage() -> float:
    """Download the current ESP32 CSV and return its newest voltage reading."""
    serial_connection = esp32.connect_esp32()
    if serial_connection is None:
        raise RuntimeError("could not connect to the ESP32 on /dev/esp32")

    temporary_path: Path | None = None
    try:
        esp32.sync_time(serial_connection)

        with tempfile.NamedTemporaryFile(
            prefix="rtkbase-battery-",
            suffix=".csv",
            delete=False,
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)

        if not esp32.download_csv_data(
            serial_connection,
            str(temporary_path),
            delete_after=False,
        ):
            raise RuntimeError("the ESP32 did not return CSV data")

        latest_voltage: float | None = None
        with temporary_path.open(newline="", encoding="utf-8") as csv_file:
            for row in csv.DictReader(csv_file):
                raw_voltage = (row.get("Battery_Voltage") or "").strip()
                if not raw_voltage:
                    continue
                try:
                    voltage = float(raw_voltage)
                except ValueError:
                    continue
                if 8.0 <= voltage <= 20.0:
                    latest_voltage = voltage

        if latest_voltage is None:
            raise RuntimeError("no valid battery voltage was found in the ESP32 data")
        return latest_voltage
    finally:
        serial_connection.close()
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def send_notification(voltage: float) -> None:
    hostname = socket.gethostname()
    message = f"{hostname} battery voltage: {voltage:.2f} V"
    request = urllib.request.Request(
        NTFY_URL,
        data=message.encode("utf-8"),
        method="POST",
        headers={
            "Title": "RTK base battery voltage",
            "Priority": "default",
            "Tags": "battery",
            "User-Agent": "rtkbase-battery-notifier/1.0",
        },
    )
    with urllib.request.urlopen(request, timeout=15) as response:
        if not 200 <= response.status < 300:
            raise RuntimeError(f"ntfy returned HTTP {response.status}")
    print(f"Sent: {message}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Send RTK base battery voltage to ntfy every 30 minutes."
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="send one notice and exit",
    )
    parser.add_argument(
        "--interval-minutes",
        type=float,
        default=DEFAULT_INTERVAL_MINUTES,
        help="minutes between readings (default: 30)",
    )
    args = parser.parse_args()

    if args.interval_minutes <= 0:
        parser.error("--interval-minutes must be greater than zero")

    print(
        f"Battery notifier started; topic={NTFY_TOPIC}, "
        f"interval={args.interval_minutes:g} minutes",
        flush=True,
    )

    try:
        while True:
            cycle_started = time.monotonic()
            try:
                voltage = read_latest_battery_voltage()
                send_notification(voltage)
            except (OSError, RuntimeError, urllib.error.URLError) as exc:
                print(f"Battery notice failed: {exc}", flush=True)

            if args.once:
                return

            elapsed = time.monotonic() - cycle_started
            sleep_seconds = max(0.0, args.interval_minutes * 60 - elapsed)
            print(f"Next reading in {sleep_seconds / 60:.1f} minutes", flush=True)
            time.sleep(sleep_seconds)
    except KeyboardInterrupt:
        print("\nBattery notifier stopped.", flush=True)


if __name__ == "__main__":
    main()
