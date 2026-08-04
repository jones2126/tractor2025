#!/usr/bin/env python3
"""Inspect rtcm-server health from systemd state, devices, and its journal.

The default mode prints a bounded diagnostic snapshot.  ``--follow`` streams
and classifies new journal events, which is also a useful foundation for a
future supervising agent or alerting service.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any
from urllib import error as urlerror
from urllib import request as urlrequest


SERVICE = "rtcm-server.service"
DEVICES = ("/dev/gps-base-link", "/dev/gps-heading")
FORWARD_RE = re.compile(
    r"Forwarded\s+(?P<bytes>\d+)\s+bytes\s+\(~(?P<rate>\d+)\s+B/s\).*?Total forwarded:\s*(?P<total>\d+)",
    re.IGNORECASE,
)

ERROR_TERMS = (
    "fatal",
    "could not open",
    "no such file",
    "connection refused",
    "connection closed",
    "tcp timeout",
    "serial error",
    "unexpected error",
    "write incomplete",
    "checksum failed",
    "buffer overflow",
)
WARNING_TERMS = ("reconnecting", "parse error")
GOOD_TERMS = ("connected successfully", "forwarded ", "rtcm server running")
DEFAULT_NTFY_TOPIC = "rpi-tractor01-jones2126"
NTFY_URL = "https://ntfy.sh/{topic}"


def run(command: list[str], timeout: float = 8.0) -> tuple[int, str]:
    try:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
        return completed.returncode, completed.stdout.strip()
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 127, str(exc)


def systemd_properties() -> dict[str, str]:
    names = (
        "ActiveState",
        "SubState",
        "MainPID",
        "NRestarts",
        "ExecMainStatus",
        "ActiveEnterTimestamp",
        "ExecStart",
    )
    code, output = run(["systemctl", "show", SERVICE, *[f"--property={name}" for name in names]])
    if code != 0:
        return {"error": output}
    properties = {}
    for line in output.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            properties[key] = value
    return properties


def journal_for_pid(pid: int, lines: int) -> tuple[int, str]:
    return run(
        [
            "journalctl",
            f"_PID={pid}",
            "-b",
            "--no-pager",
            "-o",
            "short-iso-precise",
            "-n",
            str(lines),
        ]
    )


def recent_journal(seconds: int) -> tuple[int, str]:
    return run(
        [
            "journalctl",
            "-u",
            SERVICE,
            "--since",
            f"{seconds} seconds ago",
            "--no-pager",
            "-o",
            "short-iso-precise",
        ]
    )


def classify(line: str) -> str:
    lowered = line.lower()
    if any(term in lowered for term in ERROR_TERMS):
        return "ERROR"
    if any(term in lowered for term in WARNING_TERMS):
        return "WARN"
    if any(term in lowered for term in GOOD_TERMS):
        return "OK"
    return "INFO"


def send_ntfy(topic: str, title: str, message: str, level: str = "INFO") -> tuple[bool, str]:
    priority = {"ERROR": "high", "WARN": "default", "OK": "low", "INFO": "min"}.get(level, "default")
    tags = {"ERROR": "rotating_light", "WARN": "warning", "OK": "white_check_mark", "INFO": "information_source"}.get(level, "tractor")
    payload = message.encode("utf-8", errors="replace")
    req = urlrequest.Request(
        NTFY_URL.format(topic=topic),
        data=payload,
        method="POST",
        headers={
            "Title": title,
            "Priority": priority,
            "Tags": tags,
            "Content-Type": "text/plain; charset=utf-8",
        },
    )
    try:
        with urlrequest.urlopen(req, timeout=8) as response:
            return 200 <= response.status < 300, f"HTTP {response.status}"
    except (urlerror.URLError, TimeoutError, OSError) as exc:
        return False, str(exc)


def device_report() -> list[dict[str, Any]]:
    report = []
    for name in DEVICES:
        path = Path(name)
        exists = path.exists()
        try:
            target = str(path.resolve(strict=True)) if exists else None
        except OSError as exc:
            target = f"resolve error: {exc}"
        report.append({"path": name, "exists": exists, "target": target})
    return report


def snapshot(args: argparse.Namespace) -> int:
    props = systemd_properties()
    try:
        pid = int(props.get("MainPID", "0"))
    except ValueError:
        pid = 0

    journal_code, journal = journal_for_pid(pid, args.lines) if pid else (1, "no MainPID")
    recent_code, recent = recent_journal(args.recent_seconds)
    devices = device_report()
    journal_lines = [line for line in journal.splitlines() if line.strip()]
    error_lines = [line for line in journal_lines if classify(line) == "ERROR"]
    warning_lines = [line for line in journal_lines if classify(line) == "WARN"]
    forwarding = [match.groupdict() for match in FORWARD_RE.finditer(journal)]
    recent_forwarding = [match.groupdict() for match in FORWARD_RE.finditer(recent)]
    fatal_startup = any("fatal gps connection" in line.lower() for line in journal_lines)
    all_devices_present = all(item["exists"] for item in devices)
    active = props.get("ActiveState") == "active" and props.get("SubState") == "running" and pid > 0
    correction_fresh = bool(recent_forwarding) and int(recent_forwarding[-1]["rate"]) > 0

    result = {
        "service": SERVICE,
        "active": active,
        "properties": props,
        "devices": devices,
        "journal_accessible": journal_code == 0 and recent_code == 0,
        "fatal_startup_state": fatal_startup,
        "errors": error_lines,
        "warnings": warning_lines,
        "latest_forwarding": forwarding[-1] if forwarding else None,
        "recent_forwarding": recent_forwarding[-1] if recent_forwarding else None,
        "correction_stream_fresh": correction_fresh,
    }
    snapshot_passed = active and all_devices_present and not fatal_startup and correction_fresh
    if args.notify_topic:
        latest_text = (
            f"{recent_forwarding[-1]['bytes']} bytes at approximately "
            f"{recent_forwarding[-1]['rate']} B/s"
            if recent_forwarding else "no recent forwarding report"
        )
        notify_level = "OK" if snapshot_passed else "ERROR"
        notify_message = (
            f"service={props.get('ActiveState', 'unknown')}/{props.get('SubState', 'unknown')}; "
            f"pid={pid}; devices={'present' if all_devices_present else 'missing'}; "
            f"fatal_startup={fatal_startup}; corrections={latest_text}"
        )
        notify_ok, notify_detail = send_ntfy(
            args.notify_topic,
            f"tractor01 RTCM snapshot {'PASS' if snapshot_passed else 'FAIL'}",
            notify_message,
            notify_level,
        )
    else:
        notify_ok, notify_detail = False, "not requested"

    if args.json:
        print(json.dumps(result, indent=2))
        if args.notify_topic:
            print(f"ntfy snapshot notification: {'sent' if notify_ok else 'FAILED'} ({notify_detail})", file=sys.stderr)
        return 0 if snapshot_passed else 1

    print("RTCM server journal inspection")
    print("==============================")
    print(f"Service       : {props.get('ActiveState', 'unknown')}/{props.get('SubState', 'unknown')}")
    print(f"Main PID      : {pid}")
    print(f"Restarts      : {props.get('NRestarts', 'unknown')}")
    print(f"Active since  : {props.get('ActiveEnterTimestamp', 'unknown')}")
    print(f"Exit status   : {props.get('ExecMainStatus', 'unknown')}")
    print(f"ExecStart     : {props.get('ExecStart', 'unknown')}")
    if args.notify_topic:
        print(f"ntfy snapshot : {'sent' if notify_ok else 'FAILED'} ({notify_detail})")
    print()

    print("Device paths")
    for item in devices:
        status = "PASS" if item["exists"] else "FAIL"
        print(f"[{status}] {item['path']} -> {item['target'] or 'missing'}")
    print()

    if forwarding:
        last = forwarding[-1]
        print(
            "Last forwarding report: "
            f"{last['bytes']} bytes, approximately {last['rate']} B/s, total {last['total']}"
        )
    else:
        print("Last forwarding report: none from the current service process")

    if recent_forwarding:
        last = recent_forwarding[-1]
        print(
            f"Recent ({args.recent_seconds}s) report: "
            f"{last['bytes']} bytes at approximately {last['rate']} B/s"
        )
    else:
        print(f"Recent ({args.recent_seconds}s) report: none")
    print()

    if error_lines:
        print("Errors from current service process")
        for line in error_lines[-args.show_errors :]:
            print(f"[ERROR] {line}")
        print()
    if warning_lines:
        print("Recent warnings")
        for line in warning_lines[-args.show_errors :]:
            print(f"[WARN]  {line}")
        print()

    failures = []
    if not active:
        failures.append("service is not active/running")
    if not all_devices_present:
        failures.append("one or more GPS device paths are currently missing")
    if fatal_startup:
        failures.append("current process entered the non-retrying fatal GPS startup state")
    if not correction_fresh:
        failures.append(f"no nonzero forwarding report in the last {args.recent_seconds} seconds")
    if journal_code != 0 or recent_code != 0:
        failures.append("journal could not be read; run this inspector with sudo")

    if failures:
        print("RESULT: FAIL")
        for failure in failures:
            print(f"  - {failure}")
        if fatal_startup and all_devices_present:
            print()
            print("LIKELY RECOVERY:")
            print("  The GPS paths exist now, but this process does not retry device opens.")
            print("  Restart it, then rerun this inspector:")
            print("    sudo systemctl restart rtcm-server.service")
            print("    sleep 12")
            print(f"    sudo python3 {Path(__file__).resolve()}")
        return 1

    print("RESULT: PASS — service, devices, and recent RTCM forwarding look healthy.")
    return 0


def follow(args: argparse.Namespace) -> int:
    props = systemd_properties()
    try:
        pid = int(props.get("MainPID", "0"))
    except ValueError:
        pid = 0
    if pid <= 0:
        print("FAIL: rtcm-server has no active MainPID", file=sys.stderr)
        return 1

    # Follow the unit rather than the current PID so monitoring survives an
    # rtcm-server restart and automatically observes the replacement process.
    command = ["journalctl", "-u", SERVICE, "-b", "-f", "-n", "0", "-o", "cat"]
    print(f"Following {SERVICE} (current PID {pid}); Ctrl+C to stop.")
    print("This mode classifies events but does not restart services.")
    if args.notify_topic:
        ok, detail = send_ntfy(
            args.notify_topic,
            "tractor01 RTCM monitor started",
            f"Following {SERVICE} PID {pid}. Verbose journal notifications are enabled.",
            "INFO",
        )
        print(f"ntfy startup notification: {'sent' if ok else 'FAILED'} ({detail})")
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        assert process.stdout is not None
        for line in process.stdout:
            line = line.rstrip()
            if line:
                level = classify(line)
                print(f"[{level}] {line}", flush=True)
                if args.notify_topic:
                    ok, detail = send_ntfy(
                        args.notify_topic,
                        f"tractor01 RTCM {level}",
                        line,
                        level,
                    )
                    if not ok:
                        print(f"[WARN] ntfy delivery failed: {detail}", flush=True)
    except KeyboardInterrupt:
        process.terminate()
        return 130
    finally:
        if process.poll() is None:
            process.terminate()
    return process.wait()


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect and classify rtcm-server journal health")
    parser.add_argument("--lines", type=int, default=500, help="current-PID journal lines to inspect")
    parser.add_argument("--recent-seconds", type=int, default=30, help="fresh-forwarding window")
    parser.add_argument("--show-errors", type=int, default=12)
    parser.add_argument("--json", action="store_true", help="machine-readable snapshot for a future agent")
    parser.add_argument("--follow", action="store_true", help="continuously classify new journal events")
    parser.add_argument(
        "--notify-topic",
        default=None,
        help=f"send every followed event to ntfy.sh (recommended topic: {DEFAULT_NTFY_TOPIC})",
    )
    args = parser.parse_args()
    return follow(args) if args.follow else snapshot(args)


if __name__ == "__main__":
    raise SystemExit(main())
