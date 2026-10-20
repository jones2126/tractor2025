#!/usr/bin/env python3
"""Guarded, idempotent Manual Recovery service workflow for tractor01."""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    import pwd
except ImportError:  # pragma: no cover - deployment is Linux
    pwd = None


NORMAL_SERVICE = "tractor-wifi-control.service"
RECOVERY_SERVICE = "tractor-wifi-recovery.service"
RECOVERY_INTERFACE = "eth0"
RECOVERY_ADDRESS = "192.168.8.20"
RECOVERY_SUBNET = "192.168.8.0/24"
EXPECTED_PHONE_SSID = "TractorField"
CONTROL_PORT = 6011
TELEMETRY_PORT = 6012
LOG_DIR = Path("/home/al/field_logs/manual_recovery")
TRANSITIONS = LOG_DIR / "recovery_service_transitions.jsonl"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def run(*args: str, check: bool = False) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, text=True, capture_output=True, check=check)


def service_active(name: str) -> bool:
    return run("systemctl", "is-active", "--quiet", name).returncode == 0


def interface_ready() -> bool:
    result = run("ip", "-4", "-o", "address", "show", "dev", RECOVERY_INTERFACE)
    return result.returncode == 0 and f"{RECOVERY_ADDRESS}/" in result.stdout


def mission_processes() -> list[str]:
    result = run("pgrep", "-af", "pure_pursuit_controller_")
    if result.returncode not in (0, 1):
        raise RuntimeError(f"could not inspect mission processes: {result.stderr.strip()}")
    return [line for line in result.stdout.splitlines() if line.strip()]


def ensure_log_dir() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    if pwd is None:
        raise RuntimeError("recovery workflow requires Linux account ownership")
    account = pwd.getpwnam("al")
    os.chown(LOG_DIR, account.pw_uid, account.pw_gid)
    LOG_DIR.chmod(0o750)


def append_transition(event: str, **fields: Any) -> None:
    ensure_log_dir()
    record = {"time": utc_now(), "event": event, **fields}
    with TRANSITIONS.open("a", encoding="utf-8", buffering=1) as handle:
        handle.write(json.dumps(record, separators=(",", ":")) + "\n")
    run("logger", "-t", "tractor-manual-recovery", json.dumps(record))


def send_pause_and_capture(timeout_s: float = 3.0) -> dict[str, Any] | None:
    """Request mission Pause and capture a paused telemetry checkpoint."""
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
    receiver.bind(("127.0.0.1", TELEMETRY_PORT))
    receiver.settimeout(0.25)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    encoded = json.dumps({"command": "pause", "source": "manual_recovery"}).encode()
    deadline = time.monotonic() + timeout_s
    latest = None
    try:
        while time.monotonic() < deadline:
            sender.sendto(encoded, ("127.0.0.1", CONTROL_PORT))
            try:
                data, _ = receiver.recvfrom(65535)
                candidate = json.loads(data.decode("utf-8"))
                if isinstance(candidate, dict):
                    latest = candidate
                    if candidate.get("software_paused") is True:
                        return candidate
            except (socket.timeout, UnicodeDecodeError, json.JSONDecodeError):
                pass
        return latest
    finally:
        sender.close()
        receiver.close()


def save_checkpoint(telemetry: dict[str, Any], processes: list[str]) -> Path:
    ensure_log_dir()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = LOG_DIR / f"mission_checkpoint_before_recovery_{stamp}.json"
    payload = {
        "captured_at": utc_now(),
        "reason": "manual_recovery_start",
        "mission_processes": processes,
        "controller": telemetry,
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def require_root() -> None:
    if os.geteuid() != 0:
        raise RuntimeError("run this command with sudo")


def start() -> None:
    require_root()
    if service_active(RECOVERY_SERVICE):
        print("Manual Recovery is already active (idempotent start).")
        show_status()
        return
    if not interface_ready():
        raise RuntimeError(
            f"{RECOVERY_ADDRESS} is not present on {RECOVERY_INTERFACE}; remain in Pause"
        )

    processes = mission_processes()
    checkpoint = None
    if processes:
        telemetry = send_pause_and_capture()
        if not telemetry or telemetry.get("software_paused") is not True:
            raise RuntimeError(
                "an active mission could not confirm software Pause; recovery was not started"
            )
        checkpoint = save_checkpoint(telemetry, processes)
        append_transition(
            "mission_paused",
            checkpoint=str(checkpoint),
            path_progress_m=telemetry.get("path_progress_m"),
            waypoint=telemetry.get("closest_idx", telemetry.get("waypoint_index")),
        )

    append_transition(
        "recovery_start_requested",
        normal_active=service_active(NORMAL_SERVICE),
        mission_processes=processes,
        checkpoint=str(checkpoint) if checkpoint else None,
    )
    result = run("systemctl", "start", RECOVERY_SERVICE)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "recovery service failed to start")
    if not service_active(RECOVERY_SERVICE) or service_active(NORMAL_SERVICE):
        run("systemctl", "stop", RECOVERY_SERVICE)
        raise RuntimeError("exclusive recovery ownership was not established; remain in Pause")
    append_transition("recovery_started", url=f"https://{RECOVERY_ADDRESS}:8765/")
    print("Manual Recovery is active and starts in Pause.")
    print(f"Phone SSID (verify on router): {EXPECTED_PHONE_SSID}")
    print(f"Open the recovery-only ntfy link for https://{RECOVERY_ADDRESS}:8765/")
    print("Auto is disabled. Limits: 25% reverse, 60% forward.")


def stop() -> None:
    require_root()
    was_active = service_active(RECOVERY_SERVICE)
    result = run("systemctl", "stop", RECOVERY_SERVICE)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "recovery service failed to stop")
    append_transition("recovery_stopped", was_active=was_active)
    print("Manual Recovery is stopped. Normal control was NOT restarted automatically.")
    print("The tractor remains in Pause; deliberately start normal control when appropriate.")


def show_status() -> None:
    processes = mission_processes()
    print(f"Recovery service: {'ACTIVE' if service_active(RECOVERY_SERVICE) else 'inactive'}")
    print(f"Normal service:   {'ACTIVE' if service_active(NORMAL_SERVICE) else 'inactive'}")
    print(
        f"Router interface: {'READY' if interface_ready() else 'NOT READY'} "
        f"({RECOVERY_INTERFACE} {RECOVERY_ADDRESS}, clients {RECOVERY_SUBNET})"
    )
    print(f"Expected phone SSID: {EXPECTED_PHONE_SSID} (field verification required)")
    print(f"Mission controller: {'running' if processes else 'not detected'}")
    for process in processes:
        print(f"  {process}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop", "status"))
    args = parser.parse_args()
    try:
        {"start": start, "stop": stop, "status": show_status}[args.action]()
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
