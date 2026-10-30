#!/usr/bin/env python3
"""Wi-Fi-primary phone control server for tractor01.

The phone is the operator control source; NRF is not required. The Teensy
retains the final 500 ms command watchdog and automatically pauses during a
feed interruption. A transient interruption from requested AUTO may recover
only for the same still-visible browser page, within a bounded window, while
the tractor remains in effective Pause through the stable-link gate.
"""

from __future__ import annotations

import argparse
from collections import deque
import ipaddress
import json
import math
import os
import secrets
import socket
import ssl
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from http import HTTPStatus
from pathlib import Path
from typing import Any
from urllib import error as urllib_error
from urllib import request as urllib_request


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

REPO_ROOT = HERE.parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import wifi_manual_control_field_server as base
from tractor_rpi.phone_auto_recovery_config_20261022 import (
    AUTO_LINK_RECOVERY_WINDOW_S,
    AUTO_LINK_STABLE_SECONDS,
    PHONE_HEARTBEAT_TIMEOUT_S,
)


PAGE_PATH = HERE / "wifi_primary_control_20261022.html"
EXPECTED_FIRMWARE = "teensy_main_20261003_wifi_v3"
LOCAL_CONTROL_HOSTNAME = "raspberrypi.local"
ZEROTIER_PEER_IP = "192.168.193.217"
ZEROTIER_PEER_PORT = 22
ZEROTIER_INITIAL_WINDOW_S = 90.0
NETWORK_POLL_S = 3.0
NETWORK_STABLE_CHECKS = 3
NOTIFY_RETRY_S = 15.0
SERVER_PHONE_FRESH_S = PHONE_HEARTBEAT_TIMEOUT_S
DEFAULT_DIAGNOSTICS_STATUS_PATH = Path(
    "/run/tractor-phone-control/diagnostics.json"
)
DEFAULT_CONTROL_SUBNET = "192.168.10.0/24"
ZEROTIER_NETWORK = ipaddress.ip_network("192.168.193.0/24")


@dataclass(frozen=True)
class AuthorityPolicy:
    name: str
    network: ipaddress.IPv4Network
    allow_auto: bool
    qualification: str
    min_drive_percent: int = -100
    max_drive_percent: int = 100
    recovery_only: bool = False
    interface_name: str = ""
    numeric_host: str = ""
    expected_ssid: str = ""


AUTHORITY_POLICIES = {
    "zerotier-primary": AuthorityPolicy(
        "zerotier-primary",
        ZEROTIER_NETWORK,
        True,
        "production target; home phone path verified, Tractor01 acceptance deferred",
    ),
    "tractorfield-mission": AuthorityPolicy(
        "tractorfield-mission",
        ipaddress.ip_network("192.168.10.0/24"),
        True,
        "historical-current; failed October 6 reliability qualification",
    ),
    "onboard-router-recovery": AuthorityPolicy(
        "onboard-router-recovery",
        ipaddress.ip_network("192.168.8.0/24"),
        False,
        (
            "Manual-only recovery; full drive and steering range owner-authorized "
            "2026-10-24; physical acceptance deferred"
        ),
        min_drive_percent=-100,
        max_drive_percent=100,
        recovery_only=True,
        interface_name="eth0",
        numeric_host="192.168.8.20",
        expected_ssid="TractorField",
    ),
}


def authority_policy(
    name: str,
    subnet_override: str | None = None,
    allow_zerotier_dry_run: bool = False,
) -> AuthorityPolicy:
    try:
        selected = AUTHORITY_POLICIES[name]
    except KeyError as exc:
        raise ValueError(f"unknown authority policy: {name}") from exc
    if subnet_override is None:
        return selected
    network = ipaddress.ip_network(subnet_override, strict=False)
    if network.version != 4:
        raise ValueError("control subnet must be an IPv4 network")
    if network.overlaps(ZEROTIER_NETWORK) and not allow_zerotier_dry_run:
        raise ValueError(
            "ZeroTier motion authority is not implemented or qualified; "
            "the overlay subnet cannot be used as a custom control subnet"
        )
    return AuthorityPolicy(
        f"{selected.name}-custom-subnet",
        network,
        selected.allow_auto,
        "UNQUALIFIED custom subnet; bench testing required",
        selected.min_drive_percent,
        selected.max_drive_percent,
        selected.recovery_only,
        selected.interface_name,
        selected.numeric_host,
        selected.expected_ssid,
    )


def acquire_ownership_lock(path: Path | None):
    """Hold a Linux advisory lock for the lifetime of the control server."""
    if path is None:
        return None
    try:
        import fcntl
    except ImportError as exc:  # pragma: no cover - deployment is Linux
        raise RuntimeError("--ownership-lock requires Linux fcntl support") from exc
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        handle.close()
        raise RuntimeError(
            f"another phone-control process owns {path}; remain in Pause"
        ) from exc
    handle.seek(0)
    handle.truncate()
    handle.write(f"pid={os.getpid()} policy=phone-control\n")
    handle.flush()
    return handle


def simulated_bridge_loop(state: "WifiPrimaryState") -> None:
    """Provide non-hardware telemetry only for an explicitly dry-run server."""
    while state.running:
        with state.lock:
            age_ms = (
                round((time.monotonic() - state.owner_at) * 1000)
                if state.owner_at
                else 0
            )
            effective_phone_mode = state.effective_phone_mode()
            phone_mode = {"pause": 0, "manual": 1, "auto": 2, "estop": 3}.get(
                effective_phone_mode, 0
            )
            low_level_mode = {"pause": 2, "manual": 1, "auto": 0, "estop": 2}.get(
                effective_phone_mode, 2
            )
            state.bridge = {
                "system": {"firmware": EXPECTED_FIRMWARE, "simulated": True},
                "steering": {
                    "mode": low_level_mode,
                    "state": "SIMULATED",
                    "cmd_age_ms": age_ms,
                },
                "transmission": {"mode": low_level_mode, "target": 0, "current": 0},
                "wifi_control": {
                    "mode": phone_mode,
                    "heartbeat_fresh": int(age_ms <= 500),
                    "estop_latched": int(effective_phone_mode == "estop"),
                    "command_age_ms": age_ms,
                    "simulated": True,
                },
            }
            state.bridge_at = time.monotonic()
        time.sleep(0.05)


def load_or_create_operator_key(path: Path | None) -> str:
    """Return a persistent bearer key, or an ephemeral key for manual tests."""
    if path is None:
        return secrets.token_urlsafe(18)

    def read_key() -> str:
        key = path.read_text(encoding="ascii").strip()
        if not 24 <= len(key) <= 128 or any(
            not (character.isalnum() or character in "-_") for character in key
        ):
            raise RuntimeError(f"invalid operator key file: {path}")
        path.chmod(0o600)
        return key

    try:
        return read_key()
    except FileNotFoundError:
        pass

    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    generated = secrets.token_urlsafe(24)
    try:
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        return read_key()
    with os.fdopen(descriptor, "w", encoding="ascii") as key_file:
        key_file.write(generated + "\n")
    return generated


def post_ntfy(
    topic: str,
    title: str,
    message: str,
    click_url: str,
    tags: str,
) -> bool:
    request = urllib_request.Request(
        f"https://ntfy.sh/{topic}",
        data=message.encode("utf-8"),
        headers={
            "Title": title,
            "Click": click_url,
            "Tags": tags,
        },
        method="POST",
    )
    try:
        with urllib_request.urlopen(request, timeout=5) as response:
            if not 200 <= response.status < 300:
                raise RuntimeError(f"ntfy returned HTTP {response.status}")
        print(f"ntfy: {title} sent to https://ntfy.sh/{topic}", flush=True)
        return True
    except (urllib_error.URLError, OSError, RuntimeError) as exc:
        print(f"WARNING: could not send '{title}' to ntfy: {exc}", flush=True)
        return False


def address_is_assigned(address: str, interface_prefix: str | None = None) -> bool:
    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "addr", "show"],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    if result.returncode != 0:
        return False
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) < 4 or fields[2] != "inet":
            continue
        interface = fields[1].split("@", 1)[0]
        assigned = fields[3].split("/", 1)[0]
        if assigned == address and (
            interface_prefix is None or interface.startswith(interface_prefix)
        ):
            return True
    return False


def private_local_addresses() -> list[str]:
    """Return private IPv4 addresses on non-loopback, non-ZeroTier interfaces."""
    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "scope", "global"],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (OSError, subprocess.SubprocessError):
        return []
    if result.returncode != 0:
        return []
    addresses: list[str] = []
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) < 4 or fields[2] != "inet":
            continue
        interface = fields[1].split("@", 1)[0]
        assigned = fields[3].split("/", 1)[0]
        try:
            parsed = ipaddress.ip_address(assigned)
        except ValueError:
            continue
        if interface.startswith("zt") or interface == "lo" or not parsed.is_private:
            continue
        if assigned not in addresses:
            addresses.append(assigned)
    return addresses


def zerotier_service_active() -> bool:
    try:
        result = subprocess.run(
            ["systemctl", "is-active", "--quiet", "zerotier-one.service"],
            check=False,
            timeout=3,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return result.returncode == 0


def tcp_peer_reachable(address: str, port: int) -> bool:
    try:
        with socket.create_connection((address, port), timeout=2):
            return True
    except OSError:
        return False


class ConnectivityMonitor:
    """Publish local and verified-ZeroTier URLs without gating local control."""

    def __init__(
        self,
        state: "WifiPrimaryState",
        topic: str,
        local_url: str,
        zerotier_url: str,
        no_ntfy: bool,
        peer_ip: str = ZEROTIER_PEER_IP,
        peer_port: int = ZEROTIER_PEER_PORT,
        initial_window_s: float = ZEROTIER_INITIAL_WINDOW_S,
        poll_s: float = NETWORK_POLL_S,
        stable_checks: int = NETWORK_STABLE_CHECKS,
        notify_retry_s: float = NOTIFY_RETRY_S,
        local_check: Any = None,
        zerotier_check: Any = None,
        notifier: Any = None,
        clock: Any = None,
    ) -> None:
        self.state = state
        self.topic = topic
        self.local_url = local_url
        self.zerotier_url = zerotier_url
        self.no_ntfy = no_ntfy
        self.peer_ip = peer_ip
        self.peer_port = peer_port
        self.initial_window_s = initial_window_s
        self.poll_s = poll_s
        self.stable_checks = stable_checks
        self.notify_retry_s = notify_retry_s
        self.local_check = local_check or (lambda: bool(private_local_addresses()))
        self.zerotier_check = zerotier_check or self.probe_zerotier
        self.notifier = notifier or post_ntfy
        self.clock = clock or time.monotonic
        self.started_at = self.clock()
        self.local_notice_sent = False
        self.delayed_notice_sent = False
        self.ever_zerotier_ready = False
        self.zerotier_ready = False
        self.successes = 0
        self.failures = 0
        self.pending_zerotier_notice: str | None = None
        self.next_attempt = {"local": 0.0, "delayed": 0.0, "zerotier": 0.0}
        self.last_probe_reason: str | None = None

    def log(self, event: str, **fields: Any) -> None:
        detail = " ".join(f"{key}={value}" for key, value in fields.items())
        print(
            f"Network readiness: {event}{' ' + detail if detail else ''}",
            flush=True,
        )
        self.state.log(f"network_{event}", **fields)

    def probe_zerotier(self) -> tuple[bool, str]:
        if not zerotier_service_active():
            return False, "zerotier-one inactive"
        if not address_is_assigned(base.TRACTOR_ZEROTIER_IP, "zt"):
            return False, f"{base.TRACTOR_ZEROTIER_IP} not assigned to zt interface"
        if not tcp_peer_reachable(self.peer_ip, self.peer_port):
            return False, f"NAS {self.peer_ip}:{self.peer_port} unreachable"
        return True, f"NAS {self.peer_ip}:{self.peer_port} reachable"

    def send_notice(
        self,
        kind: str,
        now: float,
        title: str,
        message: str,
        click_url: str,
        tags: str,
    ) -> bool:
        if now < self.next_attempt[kind]:
            return False
        if self.no_ntfy:
            return True
        self.state.ntfy_pending_kind = kind
        self.state.ntfy_last_attempt_epoch_ms = round(time.time() * 1000)
        sent = self.notifier(self.topic, title, message, click_url, tags)
        self.state.ntfy_last_result = "acknowledged" if sent else "queued_for_retry"
        if sent:
            self.state.ntfy_pending_kind = None
            self.state.ntfy_last_ack_epoch_ms = round(time.time() * 1000)
        if not sent:
            self.next_attempt[kind] = now + self.notify_retry_s
        return sent

    def step(self, now: float | None = None) -> None:
        now = self.clock() if now is None else now
        local_ready = bool(self.local_check())
        if local_ready and not self.local_notice_sent:
            self.local_notice_sent = self.send_notice(
                "local",
                now,
                "Tractor01 local control ready",
                "Connect the phone to the tractor's local router, then open "
                "this temporary Wi-Fi control link. Tractor control starts in Pause.\n\n"
                f"{self.local_url}",
                self.local_url,
                "tractor,wifi",
            )
            if self.local_notice_sent:
                self.log(
                    "local_ready",
                    hostname=LOCAL_CONTROL_HOSTNAME,
                    addresses=",".join(private_local_addresses()),
                )

        probe_ready, reason = self.zerotier_check()
        if reason != self.last_probe_reason:
            self.log("zerotier_probe", ready=probe_ready, detail=reason)
            self.last_probe_reason = reason
        if probe_ready:
            self.successes += 1
            self.failures = 0
        else:
            self.failures += 1
            self.successes = 0

        if not self.zerotier_ready and self.successes >= self.stable_checks:
            recovered = self.ever_zerotier_ready
            self.zerotier_ready = True
            self.ever_zerotier_ready = True
            self.pending_zerotier_notice = "recovered" if recovered else "ready"
            self.next_attempt["zerotier"] = 0.0
            self.log("zerotier_ready", recovered=recovered, checks=self.successes)
        elif self.zerotier_ready and self.failures >= self.stable_checks:
            self.zerotier_ready = False
            self.pending_zerotier_notice = None
            self.log("zerotier_lost", checks=self.failures, detail=reason)

        if self.zerotier_ready and self.pending_zerotier_notice:
            recovered = self.pending_zerotier_notice == "recovered"
            title = (
                "Tractor01 ZeroTier status recovered"
                if recovered
                else "Tractor01 ZeroTier status available"
            )
            sent = self.send_notice(
                "zerotier",
                now,
                title,
                "Remote status is reachable through the RPi5NAS peer. Motion "
                "commands are disabled on ZeroTier; connect the phone to the "
                "tractor's local router to control it.\n\n"
                f"{self.zerotier_url}",
                self.zerotier_url,
                "tractor,satellite",
            )
            if sent:
                self.pending_zerotier_notice = None
                self.log("zerotier_notice_sent", recovered=recovered)

        if (
            not self.ever_zerotier_ready
            and now - self.started_at >= self.initial_window_s
            and not self.delayed_notice_sent
        ):
            click_url = self.local_url if local_ready else self.zerotier_url
            self.delayed_notice_sent = self.send_notice(
                "delayed",
                now,
                "Tractor01 ZeroTier delayed",
                "ZeroTier has not passed the RPi5NAS reachability check after "
                f"{self.initial_window_s:.0f} seconds. The service will keep retrying. "
                "Local control is available only while connected to the tractor router.\n\n"
                f"{self.local_url}",
                click_url,
                "tractor,warning",
            )
            if self.delayed_notice_sent:
                self.log("zerotier_delayed", seconds=self.initial_window_s)

    def run(self) -> None:
        while self.state.running:
            self.step()
            deadline = self.clock() + self.poll_s
            while self.state.running and self.clock() < deadline:
                time.sleep(min(0.2, max(0.0, deadline - self.clock())))


class WifiPrimaryState(base.FieldState):
    def __init__(
        self,
        command_ip: str,
        dry_run: bool,
        log_path: Path,
        policy: AuthorityPolicy | None = None,
        clock: Any = None,
        auto_recovery_window_s: float = AUTO_LINK_RECOVERY_WINDOW_S,
        auto_link_stable_s: float = AUTO_LINK_STABLE_SECONDS,
    ) -> None:
        super().__init__(command_ip, dry_run, log_path)
        self.authority_policy = policy or AUTHORITY_POLICIES["tractorfield-mission"]
        self.clock = clock or time.monotonic
        self.auto_recovery_window_s = float(auto_recovery_window_s)
        self.auto_link_stable_s = float(auto_link_stable_s)
        if self.auto_recovery_window_s <= 0.0:
            raise ValueError("AUTO recovery window must be greater than zero")
        if self.auto_link_stable_s < 0.0:
            raise ValueError("AUTO link stable time must be zero or greater")
        self.output_lock = threading.RLock()
        self.requested_mode = "pause"
        self.effective_state = "pause_latched"
        self.recovery_started_at: float | None = None
        self.recovery_deadline: float | None = None
        self.recovery_link_fresh_since: float | None = None
        self.recovery_original_client: str | None = None
        self.recovery_original_session: str | None = None
        self.recovery_original_client_ip: str | None = None
        self.recovery_client_ip: str | None = None
        self.recovery_decision = "not pending"
        self.owner_client_ip: str | None = None
        if log_path.name.startswith("wifi_primary_control_"):
            client_log_name = log_path.name.replace(
                "wifi_primary_control_", "phone_flight_recorder_", 1
            )
        else:
            client_log_name = f"phone_flight_recorder_{log_path.name}"
        self.client_log_path = log_path.with_name(client_log_name)
        self.client_log_file = self.client_log_path.open(
            "a", encoding="utf-8", buffering=1
        )
        self.last_accepted_command_at = 0.0
        self.last_command_gap_ms: float | None = None
        self.session_claim_count = 0
        self.session_invalidation_count = 0
        self.last_safety_stop_reason: str | None = None
        self.last_safety_stop_epoch_ms: int | None = None
        self.last_rejection_reason: str | None = None
        self.ntfy_pending_kind: str | None = None
        self.ntfy_last_result = "not_attempted"
        self.ntfy_last_attempt_epoch_ms: int | None = None
        self.ntfy_last_ack_epoch_ms: int | None = None
        self.zerotier_transport = "not_measured"
        self.diagnostics_status_path: Path | None = None
        self._last_diagnostics_publish = 0.0
        self.client_event_ids: set[str] = set()
        self.client_event_id_order: deque[str] = deque(maxlen=4096)
        self.client_events_deduplicated = 0

    def recovery_pending(self) -> bool:
        return self.effective_state in ("auto_link_wait", "auto_link_stabilizing")

    def effective_phone_mode(self) -> str:
        if self.effective_state == "manual_running":
            return "manual"
        if self.effective_state == "auto_running":
            return "auto"
        if self.effective_state == "estop_latched":
            return "estop"
        return "pause"

    def recovery_remaining_s(self, now: float | None = None) -> float | None:
        if self.recovery_deadline is None:
            return None
        current = self.clock() if now is None else now
        return max(0.0, self.recovery_deadline - current)

    def clear_auto_recovery(self, decision: str) -> None:
        self.recovery_started_at = None
        self.recovery_deadline = None
        self.recovery_link_fresh_since = None
        self.recovery_original_client = None
        self.recovery_original_session = None
        self.recovery_original_client_ip = None
        self.recovery_client_ip = None
        self.recovery_decision = decision

    def latch_pause(self, decision: str) -> None:
        self.requested_mode = "pause"
        self.phone_mode = "pause"
        self.effective_state = "pause_latched"
        self.clear_auto_recovery(decision)
        self.last_decision = decision

    def ingest_client_events(
        self, data: Any, client_ip: str
    ) -> dict[str, Any]:
        """Append a bounded batch of diagnostic browser events.

        These records are diagnostic only. They never grant or refresh motion
        authority and deliberately do not alter owner_at or the active session.
        """
        if not isinstance(data, dict) or not isinstance(data.get("events"), list):
            raise ValueError("events list required")
        events = data["events"]
        if not 1 <= len(events) <= 64:
            raise ValueError("flight-recorder batch must contain 1 to 64 events")
        accepted = 0
        duplicates = 0
        with self.lock:
            for event in events:
                if not isinstance(event, dict):
                    raise ValueError("each flight-recorder event must be an object")
                encoded = json.dumps(event, separators=(",", ":"), allow_nan=False)
                if len(encoded.encode("utf-8")) > 2048:
                    raise ValueError("flight-recorder event exceeds 2048 bytes")
                event_id = str(event.get("event_id", ""))
                if event_id and event_id in self.client_event_ids:
                    duplicates += 1
                    self.client_events_deduplicated += 1
                    continue
                if event_id:
                    if len(self.client_event_id_order) == self.client_event_id_order.maxlen:
                        oldest = self.client_event_id_order.popleft()
                        self.client_event_ids.discard(oldest)
                    self.client_event_id_order.append(event_id)
                    self.client_event_ids.add(event_id)
                record = {
                    "server_time": base.utc_now(),
                    "client_ip": client_ip,
                    "event": event,
                }
                self.client_log_file.write(
                    json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n"
                )
                accepted += 1
        return {"accepted": accepted, "duplicates": duplicates}

    def close(self) -> None:
        try:
            super().close()
        finally:
            self.client_log_file.close()

    def bridge_ready(self) -> bool:
        bridge, age = self.bridge_snapshot()
        return (
            age <= base.BRIDGE_FRESH_S
            and bridge.get("system", {}).get("firmware") == EXPECTED_FIRMWARE
        )

    def reject_nonlocal_control(self, client_ip: str) -> None:
        """Latch Pause immediately if a motion request arrives off the LAN."""
        with self.lock:
            sequence = (
                int(self.last_command.get("sequence", -1))
                if self.last_command else -1
            )
            self.latch_pause("nonlocal control rejected; Pause latched")
            self.owner_session = None
            self.highest_sequence = -1
            self.owner_at = 0.0
            self.stop_sent_for_expiry = True
            self.rejected += 1
        self.send_stop_burst("nonlocal_control_rejected", sequence)
        self.log("nonlocal_control_rejected", client_ip=client_ip)

    def send_drive_command(
        self,
        drive_percent: float,
        steering_percent: float,
        sequence: int,
        reason: str,
        phone_mode: str | None = None,
    ) -> None:
        mode = phone_mode or self.phone_mode
        payload = {
            "drive_percent": round(drive_percent, 1),
            "angular_z": round(-steering_percent / 100.0, 3),
            "phone_mode": mode,
            "timestamp": time.time(),
            "source": "wifi_primary_20261022",
            "phone_sequence": sequence,
            "reason": reason,
        }
        with self.output_lock:
            if not self.dry_run:
                self.cmd_sock.sendto(
                    json.dumps(payload).encode("utf-8"),
                    (self.command_ip, base.CMD_VEL_PORT),
                )
            self.udp_sent += 1

    def send_stop_burst(
        self,
        reason: str,
        sequence: int = -1,
        steering_percent: float = 0.0,
    ) -> None:
        with self.output_lock:
            for _ in range(5):
                self.send_drive_command(
                    0.0, steering_percent, sequence, reason, phone_mode="pause"
                )
                time.sleep(0.02)
            self.last_stop_reason = reason
            self.log(
                "stop_burst",
                reason=reason,
                sequence=sequence,
                steering_percent=steering_percent,
            )

    def claim(
        self,
        client_id: str,
        client_ip: str | None = None,
        recover_auto: bool = False,
    ) -> dict[str, Any]:
        now = self.clock()
        recovered = False
        refusal_reason: str | None = None
        with self.lock:
            if not client_id or len(client_id) > 100:
                raise ValueError("invalid client identifier")
            if not self.bridge_ready():
                raise RuntimeError(
                    f"waiting for fresh {EXPECTED_FIRMWARE} telemetry"
                )
            if (
                self.owner_client
                and self.owner_client != client_id
                and now - self.owner_at <= base.OWNER_EXPIRE_S
            ):
                raise RuntimeError("another phone currently owns control")
            previous_session = self.owner_session
            previous_client = self.owner_client
            recovery_eligible = (
                recover_auto
                and self.recovery_pending()
                and self.requested_mode == "auto"
                and client_id == self.recovery_original_client
                and self.recovery_deadline is not None
                and now < self.recovery_deadline
                and self.authority_policy.allow_auto
            )
            if (
                recover_auto
                and not self.recovery_pending()
                and self.requested_mode == "auto"
                and self.effective_state == "auto_running"
            ):
                self.log(
                    "auto_recovery_claim_early",
                    client_id=client_id,
                    client_ip=client_ip,
                )
                raise RuntimeError("AUTO recovery is not ready; retry")
            if recover_auto and not recovery_eligible:
                refusal = "AUTO recovery refused"
                if not self.recovery_pending():
                    refusal = (
                        self.recovery_decision
                        if "expired" in self.recovery_decision.lower()
                        else "AUTO recovery refused: no recovery is pending"
                    )
                elif client_id != self.recovery_original_client:
                    refusal = "AUTO recovery refused: browser client changed"
                elif self.recovery_deadline is None or now >= self.recovery_deadline:
                    refusal = "AUTO recovery refused: recovery window expired"
                self.log(
                    "auto_recovery_refused",
                    reason=refusal,
                    original_client=self.recovery_original_client,
                    recovery_client=client_id,
                    client_ip=client_ip,
                )
                refusal_reason = refusal
                self.latch_pause(refusal)

            self.owner_client = client_id
            self.owner_client_ip = client_ip
            self.owner_session = secrets.token_urlsafe(16)
            self.session_claim_count += 1
            self.owner_at = now
            self.highest_sequence = -1
            bridge, _ = self.bridge_snapshot()
            estop_latched = (
                bridge.get("wifi_control", {}).get("estop_latched") == 1
            )
            if recovery_eligible and not estop_latched:
                recovered = True
                self.requested_mode = "auto"
                self.phone_mode = "auto"
                self.effective_state = "auto_link_stabilizing"
                self.recovery_link_fresh_since = None
                self.recovery_client_ip = client_ip
                self.recovery_decision = "same browser reclaimed; stable-link gate pending"
                self.last_decision = self.recovery_decision
            else:
                self.requested_mode = "estop" if estop_latched else "pause"
                self.phone_mode = self.requested_mode
                self.effective_state = (
                    "estop_latched" if estop_latched else "pause_latched"
                )
                self.clear_auto_recovery(
                    refusal_reason
                    or (
                        "E-stop cancelled automatic recovery"
                    if estop_latched
                    else "ordinary claim starts in Pause"
                    )
                )
                self.last_decision = (
                    "phone claimed with E-stop latched"
                    if estop_latched
                    else "phone claimed in Pause"
                )
            self.stop_sent_for_expiry = False
            session = self.owner_session
            self.log(
                "owner_claimed",
                client_id=client_id,
                client_ip=client_ip,
                session=session,
                authority_path=self.authority_policy.name,
                recover_auto=recover_auto,
                auto_recovery_accepted=recovered,
            )
            if recovered:
                self.log(
                    "auto_recovery_claimed",
                    original_client=self.recovery_original_client,
                    recovery_client=client_id,
                    original_client_ip=self.recovery_original_client_ip,
                    recovery_client_ip=client_ip,
                    recovery_remaining_s=self.recovery_remaining_s(now),
                )
            if previous_session:
                self.session_invalidation_count += 1
                self.log(
                    "session_invalidated",
                    client_id=previous_client,
                    session=previous_session,
                    reason="new_claim",
                )
        if estop_latched:
            for _ in range(5):
                self.send_drive_command(
                    0.0, 0.0, -1, "owner_claim_estop", phone_mode="estop"
                )
                time.sleep(0.02)
        else:
            self.send_stop_burst(
                "auto_recovery_claim" if recovered else "owner_claim"
            )
        return {
            "session": session,
            "phone_mode": self.requested_mode,
            "requested_mode": self.requested_mode,
            "effective_state": self.effective_state,
            "rate_hz": 5,
            "heartbeat_timeout_ms": 500,
            "automatic_link_recovery": recovered,
            "recovery_pending": self.recovery_pending(),
            "recovery_remaining_s": self.recovery_remaining_s(now),
            "recovery_decision": self.recovery_decision,
            "auto_link_stable_s": 0.0,
            "auto_link_stable_seconds": self.auto_link_stable_s,
            "authority_path": self.authority_policy.name,
            "authority_network": str(self.authority_policy.network),
            "auto_allowed": self.authority_policy.allow_auto,
            "min_drive_percent": self.authority_policy.min_drive_percent,
            "max_drive_percent": self.authority_policy.max_drive_percent,
            "recovery_only": self.authority_policy.recovery_only,
            "interface_name": self.authority_policy.interface_name,
            "numeric_host": self.authority_policy.numeric_host,
            "expected_ssid": self.authority_policy.expected_ssid,
            "client_ip": client_ip,
        }

    def accept_command(
        self, data: Any, client_ip: str | None = None
    ) -> tuple[HTTPStatus, dict[str, Any]]:
        received_monotonic = self.clock()
        received_epoch_ms = round(time.time() * 1000)
        if not isinstance(data, dict):
            return HTTPStatus.BAD_REQUEST, {
                "accepted": False,
                "error": "JSON object required",
            }
        try:
            client_id = str(data["client_id"])
            session = str(data["session"])
            sequence = int(data["sequence"])
            requested_mode = str(data["mode"]).lower()
            drive = float(data["drive_percent"])
            steering = float(data["steering_percent"])
            reason = str(data.get("reason", "heartbeat"))[:40]
            request_id = str(data.get("request_id", ""))[:80]
            scheduler_delay_ms = float(data.get("scheduler_delay_ms", math.nan))
            coalesced_wait_ms = float(data.get("coalesced_wait_ms", math.nan))
            client_time_ms = float(data.get("client_time_ms", math.nan))
            raw_client_rtt = data.get("client_rtt_ms")
            client_rtt_ms = (
                float(raw_client_rtt)
                if raw_client_rtt is not None
                else math.nan
            )
        except (KeyError, TypeError, ValueError):
            return HTTPStatus.BAD_REQUEST, {
                "accepted": False,
                "error": "malformed command",
            }

        self.log(
            "command_received",
            client_ip=client_ip,
            client_id=client_id,
            session=session,
            sequence=sequence,
            requested_mode=requested_mode,
            drive_percent=drive,
            steering_percent=steering,
            reason=reason,
            request_id=request_id or None,
        )

        with self.lock:
            if client_id != self.owner_client or session != self.owner_session:
                self.rejected += 1
                self.last_rejection_reason = "control session is not current"
                self.log(
                    "command_rejected",
                    request_id=request_id or None,
                    sequence=sequence,
                    reason=self.last_rejection_reason,
                )
                return HTTPStatus.CONFLICT, {
                    "accepted": False,
                    "error": "control session is not current",
                    "reclaim": True,
                    "phone_mode": self.requested_mode,
                    "effective_state": self.effective_state,
                }
            if sequence <= self.highest_sequence:
                self.rejected += 1
                self.last_rejection_reason = "stale or duplicate sequence"
                self.log(
                    "command_rejected",
                    request_id=request_id or None,
                    sequence=sequence,
                    reason=self.last_rejection_reason,
                )
                return HTTPStatus.CONFLICT, {
                    "accepted": False,
                    "error": "stale or duplicate sequence",
                }
            if requested_mode not in ("manual", "auto", "pause", "estop"):
                self.rejected += 1
                return HTTPStatus.BAD_REQUEST, {
                    "accepted": False,
                    "error": "mode must be Manual, Auto, Pause, or E-stop",
                }
            if requested_mode == "auto" and not self.authority_policy.allow_auto:
                self.rejected += 1
                return HTTPStatus.CONFLICT, {
                    "accepted": False,
                    "error": (
                        f"Auto is disabled by authority policy "
                        f"{self.authority_policy.name}"
                    ),
                    "phone_mode": self.requested_mode,
                }
            if (
                not base.finite_number(drive)
                or not self.authority_policy.min_drive_percent
                <= drive
                <= self.authority_policy.max_drive_percent
            ):
                self.rejected += 1
                return HTTPStatus.BAD_REQUEST, {
                    "accepted": False,
                    "error": (
                        "drive demand must be "
                        f"{self.authority_policy.min_drive_percent} to "
                        f"+{self.authority_policy.max_drive_percent} percent"
                    ),
                }
            if not base.finite_number(steering) or not -100 <= steering <= 100:
                self.rejected += 1
                return HTTPStatus.BAD_REQUEST, {
                    "accepted": False,
                    "error": "steering must be -100 to +100 percent",
                }
            if (
                requested_mode != self.requested_mode
                and requested_mode in ("manual", "auto")
                and reason != f"guarded_{requested_mode}"
            ):
                self.rejected += 1
                return HTTPStatus.CONFLICT, {
                    "accepted": False,
                    "error": f"{requested_mode.title()} requires MODE SELECT + {requested_mode.title()}",
                    "phone_mode": self.requested_mode,
                }
            if (
                self.requested_mode == "estop"
                and requested_mode in ("manual", "auto")
            ):
                self.rejected += 1
                return HTTPStatus.CONFLICT, {
                    "accepted": False,
                    "error": "E-stop is latched; use MODE SELECT + E-STOP to reset to Pause",
                    "phone_mode": self.requested_mode,
                }
            if (
                requested_mode in ("manual", "auto")
                and not self.bridge_ready()
            ):
                self.rejected += 1
                self.last_rejection_reason = (
                    "Teensy bridge or expected firmware is not fresh"
                )
                self.log(
                    "command_rejected",
                    request_id=request_id or None,
                    sequence=sequence,
                    reason=self.last_rejection_reason,
                )
                return HTTPStatus.SERVICE_UNAVAILABLE, {
                    "accepted": False,
                    "error": "Teensy bridge or expected firmware is not fresh",
                }

            previous_phone_mode = self.requested_mode
            previous_effective_state = self.effective_state
            if self.last_accepted_command_at:
                self.last_command_gap_ms = round(
                    (received_monotonic - self.last_accepted_command_at) * 1000, 1
                )
            self.last_accepted_command_at = received_monotonic
            self.highest_sequence = sequence
            self.owner_at = received_monotonic
            self.stop_sent_for_expiry = False
            reset_estop = (
                self.requested_mode == "estop"
                and requested_mode == "pause"
                and reason == "guarded_estop_reset"
            )
            if self.requested_mode == "estop" and requested_mode == "pause" and not reset_estop:
                requested_mode = "estop"

            recovery_completed = False
            recovery_was_pending = self.recovery_pending()
            if reset_estop:
                self.latch_pause("guarded E-stop reset to Pause")
                wire_mode = "reset_pause"
            elif requested_mode == "estop":
                self.requested_mode = "estop"
                self.phone_mode = "estop"
                self.effective_state = "estop_latched"
                self.clear_auto_recovery("E-stop cancelled automatic recovery")
                self.last_decision = "estop"
                wire_mode = "estop"
            elif requested_mode == "pause":
                self.latch_pause(
                    "explicit Pause cancelled automatic recovery"
                    if self.recovery_pending()
                    else "pause"
                )
                wire_mode = "pause"
            elif requested_mode == "manual":
                recovery_was_pending = self.recovery_pending()
                self.requested_mode = "manual"
                self.phone_mode = "manual"
                self.effective_state = "manual_running"
                self.clear_auto_recovery(
                    "Manual cancelled automatic recovery"
                    if recovery_was_pending
                    else "manual selected"
                )
                self.last_decision = "manual"
                wire_mode = "manual"
            elif self.recovery_pending():
                self.requested_mode = "auto"
                self.phone_mode = "auto"
                if (
                    self.recovery_deadline is None
                    or received_monotonic >= self.recovery_deadline
                ):
                    self.latch_pause("AUTO recovery window expired")
                    wire_mode = "pause"
                else:
                    if self.recovery_link_fresh_since is None:
                        self.recovery_link_fresh_since = received_monotonic
                        self.log(
                            "auto_recovery_stable_gate_started",
                            client_id=client_id,
                            client_ip=client_ip,
                            required_seconds=self.auto_link_stable_s,
                        )
                    stable_s = max(
                        0.0,
                        received_monotonic - self.recovery_link_fresh_since,
                    )
                    if stable_s >= self.auto_link_stable_s:
                        recovery_completed = True
                        self.effective_state = "auto_running"
                        self.last_decision = "automatic AUTO recovery completed"
                        self.recovery_decision = self.last_decision
                        wire_mode = "auto"
                    else:
                        self.effective_state = "auto_link_stabilizing"
                        self.last_decision = (
                            f"AUTO link stable for {stable_s:.1f}/"
                            f"{self.auto_link_stable_s:.1f} s"
                        )
                        wire_mode = "pause"
            else:
                self.requested_mode = "auto"
                self.phone_mode = "auto"
                self.effective_state = "auto_running"
                self.last_decision = "auto"
                wire_mode = "auto"

            effective_drive = drive if wire_mode == "manual" else 0.0
            effective_steering = steering if wire_mode == "manual" else 0.0
            self.last_command = {
                "client_id": client_id,
                "client_ip": client_ip,
                "sequence": sequence,
                "mode": self.requested_mode,
                "effective_state": self.effective_state,
                "wire_mode": wire_mode,
                "drive_percent": round(effective_drive, 1),
                "steering_percent": round(effective_steering, 1),
                "reason": reason,
            }
            self.accepted += 1

        self.send_drive_command(
            effective_drive,
            effective_steering,
            sequence,
            reason,
            phone_mode=wire_mode,
        )
        if previous_phone_mode != self.requested_mode:
            self.log(
                "mode_transition",
                client_ip=client_ip,
                client_id=client_id,
                session=session,
                from_mode=previous_phone_mode,
                to_mode=self.requested_mode,
                reason=reason,
            )
        if previous_effective_state != self.effective_state:
            self.log(
                "effective_state_transition",
                from_state=previous_effective_state,
                to_state=self.effective_state,
                requested_mode=self.requested_mode,
                reason=self.last_decision,
            )
        if (
            recovery_was_pending
            and requested_mode in ("pause", "manual", "estop")
            and not self.recovery_pending()
        ):
            self.log(
                "auto_recovery_cancelled_by_operator",
                selected_mode=requested_mode,
                client_id=client_id,
                client_ip=client_ip,
            )
        if recovery_completed:
            self.log(
                "auto_recovery_stable_gate_satisfied",
                client_id=client_id,
                stable_seconds=self.auto_link_stable_s,
            )
            self.log(
                "auto_recovery_completed",
                original_client=self.recovery_original_client,
                recovery_client=client_id,
                original_client_ip=self.recovery_original_client_ip,
                recovery_client_ip=client_ip,
            )
            self.clear_auto_recovery("automatic AUTO recovery completed")
        processing_ms = round((self.clock() - received_monotonic) * 1000, 2)
        phone_to_server_ms = (
            round(received_epoch_ms - client_time_ms, 2)
            if math.isfinite(client_time_ms)
            else None
        )
        bridge, bridge_age = self.bridge_snapshot()
        self.log(
            "command",
            **self.last_command,
            session=session,
            authority_path=self.authority_policy.name,
            client_time_ms=client_time_ms if math.isfinite(client_time_ms) else None,
            client_reported_rtt_ms=(
                round(client_rtt_ms, 2) if math.isfinite(client_rtt_ms) else None
            ),
            estimated_phone_to_server_ms=phone_to_server_ms,
            request_id=request_id or None,
            client_scheduler_delay_ms=(
                round(scheduler_delay_ms, 2)
                if math.isfinite(scheduler_delay_ms)
                else None
            ),
            client_coalesced_wait_ms=(
                round(coalesced_wait_ms, 2)
                if math.isfinite(coalesced_wait_ms)
                else None
            ),
            server_processing_ms=processing_ms,
            bridge_age_ms=(
                round(bridge_age * 1000, 1) if math.isfinite(bridge_age) else None
            ),
            transmission=bridge.get("transmission", {}),
            steering=bridge.get("steering", {}),
            wifi_control=bridge.get("wifi_control", {}),
            command_gap_ms=self.last_command_gap_ms,
        )
        return HTTPStatus.OK, {
            "accepted": True,
            "decision": wire_mode,
            "sequence": sequence,
            "server_received_time_ms": received_epoch_ms,
            "server_response_time_ms": round(time.time() * 1000),
            "server_processing_ms": processing_ms,
            "phone_mode": self.requested_mode,
            "requested_mode": self.requested_mode,
            "effective_state": self.effective_state,
            "recovery_pending": self.recovery_pending(),
            "recovery_remaining_s": self.recovery_remaining_s(),
            "auto_link_stable_s": (
                0.0
                if self.recovery_link_fresh_since is None
                else max(0.0, self.clock() - self.recovery_link_fresh_since)
            ),
            "auto_link_stable_seconds": self.auto_link_stable_s,
        }

    def snapshot(self) -> dict[str, Any]:
        result = super().snapshot()
        now = self.clock()
        bridge, _ = self.bridge_snapshot()
        result["system"] = bridge.get("system", {})
        result["wifi_control"] = bridge.get("wifi_control", {})
        result["wifi_primary_ready"] = self.bridge_ready()
        result["expected_firmware"] = EXPECTED_FIRMWARE
        result["authority_path"] = self.authority_policy.name
        result["authority_network"] = str(self.authority_policy.network)
        result["authority_auto_allowed"] = self.authority_policy.allow_auto
        result["authority_qualification"] = self.authority_policy.qualification
        result["min_drive_percent"] = self.authority_policy.min_drive_percent
        result["max_drive_percent"] = self.authority_policy.max_drive_percent
        result["recovery_only"] = self.authority_policy.recovery_only
        result["control_interface"] = self.authority_policy.interface_name
        result["numeric_control_host"] = self.authority_policy.numeric_host
        result["expected_ssid"] = self.authority_policy.expected_ssid
        result["last_command_gap_ms"] = self.last_command_gap_ms
        result["client_events_deduplicated"] = self.client_events_deduplicated
        result["session_claim_count"] = self.session_claim_count
        result["session_invalidation_count"] = self.session_invalidation_count
        result["last_safety_stop_reason"] = self.last_safety_stop_reason
        result["last_safety_stop_epoch_ms"] = self.last_safety_stop_epoch_ms
        result["last_rejection_reason"] = self.last_rejection_reason
        result["zerotier_transport"] = self.zerotier_transport
        result["ntfy"] = {
            "pending_kind": self.ntfy_pending_kind,
            "last_result": self.ntfy_last_result,
            "last_attempt_epoch_ms": self.ntfy_last_attempt_epoch_ms,
            "last_ack_epoch_ms": self.ntfy_last_ack_epoch_ms,
        }
        result["session_active"] = self.owner_session is not None
        result["phone_mode"] = self.requested_mode
        result["requested_mode"] = self.requested_mode
        result["effective_state"] = self.effective_state
        result["effective_phone_mode"] = self.effective_phone_mode()
        result["recovery_pending"] = self.recovery_pending()
        result["recovery_remaining_s"] = self.recovery_remaining_s(now)
        result["recovery_decision"] = self.recovery_decision
        result["recovery_original_client"] = self.recovery_original_client
        result["recovery_client"] = self.owner_client if self.recovery_pending() else None
        result["recovery_original_client_ip"] = self.recovery_original_client_ip
        result["recovery_client_ip"] = self.recovery_client_ip
        result["server_heartbeat_fresh"] = bool(
            self.owner_at and now - self.owner_at <= SERVER_PHONE_FRESH_S
        )
        result["auto_recovery_window_seconds"] = self.auto_recovery_window_s
        result["auto_link_stable_seconds"] = self.auto_link_stable_s
        result["auto_link_stable_s"] = (
            0.0
            if self.recovery_link_fresh_since is None
            else max(0.0, now - self.recovery_link_fresh_since)
        )
        return result

    def safety_step(self, now: float | None = None) -> bool:
        now = self.clock() if now is None else now
        should_stop = False
        sequence = -1
        event = ""
        expired_session = None
        expired_client = None
        with self.lock:
            age = now - self.owner_at if self.owner_at else math.inf
            if self.recovery_pending() and (
                self.recovery_deadline is None or now >= self.recovery_deadline
            ):
                should_stop = True
                event = "auto_recovery_window_expired"
                expired_session = self.owner_session
                expired_client = self.owner_client
                self.latch_pause("AUTO recovery window expired; Pause latched")
                self.owner_session = None
                self.highest_sequence = -1
                self.stop_sent_for_expiry = True
            elif (
                self.effective_state in ("manual_running", "auto_running")
                and age > SERVER_PHONE_FRESH_S
                and not self.stop_sent_for_expiry
            ):
                should_stop = True
                event = "phone_freshness_expired"
                expired_session = self.owner_session
                expired_client = self.owner_client
                self.stop_sent_for_expiry = True
                if self.effective_state == "auto_running":
                    timeout_at = self.owner_at + SERVER_PHONE_FRESH_S
                    self.requested_mode = "auto"
                    self.phone_mode = "auto"
                    self.effective_state = "auto_link_wait"
                    self.recovery_started_at = timeout_at
                    self.recovery_deadline = timeout_at + self.auto_recovery_window_s
                    self.recovery_link_fresh_since = None
                    self.recovery_original_client = self.owner_client
                    self.recovery_original_session = self.owner_session
                    self.recovery_original_client_ip = self.owner_client_ip
                    self.recovery_client_ip = None
                    self.recovery_decision = "AUTO heartbeat expired; waiting for same page"
                    self.last_decision = self.recovery_decision
                else:
                    self.latch_pause("Manual heartbeat expired; Pause latched")
                self.owner_session = None
                self.highest_sequence = -1
            elif (
                self.effective_state == "auto_link_stabilizing"
                and age > SERVER_PHONE_FRESH_S
                and not self.stop_sent_for_expiry
            ):
                should_stop = True
                event = "auto_recovery_link_lost"
                expired_session = self.owner_session
                expired_client = self.owner_client
                self.effective_state = "auto_link_wait"
                self.recovery_link_fresh_since = None
                self.recovery_decision = "link lost during stable-link gate"
                self.last_decision = self.recovery_decision
                self.owner_session = None
                self.highest_sequence = -1
                self.stop_sent_for_expiry = True

            if should_stop:
                if self.last_command:
                    sequence = int(self.last_command.get("sequence", -1))
                self.last_command = {
                    "client_id": self.owner_client,
                    "sequence": sequence,
                    "mode": self.requested_mode,
                    "effective_state": self.effective_state,
                    "wire_mode": "pause",
                    "drive_percent": 0.0,
                    "steering_percent": 0.0,
                    "reason": event,
                }

        if should_stop:
            self.last_safety_stop_reason = event
            self.last_safety_stop_epoch_ms = round(time.time() * 1000)
            if expired_session:
                self.session_invalidation_count += 1
            self.log(
                event,
                client_id=expired_client,
                session=expired_session,
                freshness_limit_ms=round(SERVER_PHONE_FRESH_S * 1000),
                recovery_remaining_s=self.recovery_remaining_s(now),
            )
            if event == "phone_freshness_expired" and self.effective_state == "auto_link_wait":
                self.log(
                    "auto_link_wait_entered",
                    requested_mode="auto",
                    client_id=expired_client,
                    original_session=expired_session,
                    recovery_window_seconds=self.auto_recovery_window_s,
                )
            if expired_session:
                self.log(
                    "session_invalidated",
                    client_id=expired_client,
                    session=expired_session,
                    reason=event,
                )
            self.send_stop_burst(event, sequence)
        return should_stop

    def publish_diagnostics(self) -> None:
        path = self.diagnostics_status_path
        if path is None:
            return
        snapshot = self.snapshot()
        record = {
            "schema": 1,
            "updated_epoch_ms": round(time.time() * 1000),
            "authority_path": snapshot.get("authority_path"),
            "effective_state": snapshot.get("effective_state"),
            "requested_mode": snapshot.get("requested_mode"),
            "server_heartbeat_fresh": snapshot.get("server_heartbeat_fresh"),
            "last_command_gap_ms": snapshot.get("last_command_gap_ms"),
            "accepted_commands": snapshot.get("accepted"),
            "rejected_commands": snapshot.get("rejected"),
            "session_claim_count": self.session_claim_count,
            "session_invalidation_count": self.session_invalidation_count,
            "last_safety_stop_reason": self.last_safety_stop_reason,
            "last_safety_stop_epoch_ms": self.last_safety_stop_epoch_ms,
            "last_rejection_reason": self.last_rejection_reason,
            "client_events_deduplicated": self.client_events_deduplicated,
            "zerotier_transport": self.zerotier_transport,
            "ntfy": snapshot.get("ntfy"),
            "communications_degraded": {
                "enabled": False,
                "state": "DISABLED_PENDING_STATIONARY_APPROVAL",
                "maximum_seconds": 30,
                "maximum_distance_m": 30,
            },
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(record, separators=(",", ":"), allow_nan=False),
            encoding="utf-8",
        )
        os.replace(temporary, path)

    def safety_loop(self) -> None:
        while self.running:
            self.safety_step()
            now = self.clock()
            if now - self._last_diagnostics_publish >= 0.2:
                try:
                    self.publish_diagnostics()
                except OSError as exc:
                    self.log("diagnostics_publish_failed", error=str(exc))
                self._last_diagnostics_publish = now
            time.sleep(0.05)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--command-ip", default="127.0.0.1")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--simulate-bridge",
        action="store_true",
        help="serve fake expected-firmware telemetry; requires --dry-run",
    )
    parser.add_argument("--log-dir", type=Path, default=Path("/home/al/field_logs"))
    parser.add_argument(
        "--diagnostics-status-file",
        type=Path,
        default=DEFAULT_DIAGNOSTICS_STATUS_PATH,
        help="atomic status file read by the mission dashboard Messages tab",
    )
    parser.add_argument("--tls-cert", type=Path)
    parser.add_argument("--tls-key", type=Path)
    parser.add_argument(
        "--operator-key-file",
        type=Path,
        help=(
            "persist the URL access key across restarts; omit only for an "
            "ephemeral manual test"
        ),
    )
    parser.add_argument("--no-ntfy", action="store_true")
    parser.add_argument(
        "--ntfy-topic",
        default=os.environ.get("TRACTOR_NTFY_TOPIC", base.DEFAULT_NTFY_TOPIC),
    )
    parser.add_argument("--zerotier-peer", default=ZEROTIER_PEER_IP)
    parser.add_argument("--zerotier-peer-port", type=int, default=ZEROTIER_PEER_PORT)
    parser.add_argument(
        "--zerotier-initial-window",
        type=float,
        default=ZEROTIER_INITIAL_WINDOW_S,
    )
    parser.add_argument("--network-poll-seconds", type=float, default=NETWORK_POLL_S)
    parser.add_argument(
        "--authority-policy",
        choices=sorted(AUTHORITY_POLICIES),
        default="tractorfield-mission",
        help="select exactly one startup motion-authority path",
    )
    parser.add_argument(
        "--control-subnet",
        default=None,
        help="unqualified explicit IPv4 override for bench development only",
    )
    parser.add_argument(
        "--ownership-lock",
        type=Path,
        help="exclusive Linux lock shared by normal and recovery control services",
    )
    args = parser.parse_args()

    if bool(args.tls_cert) != bool(args.tls_key):
        raise SystemExit("--tls-cert and --tls-key must be supplied together")
    if args.simulate_bridge and not args.dry_run:
        raise SystemExit("--simulate-bridge requires --dry-run")
    if args.control_subnet and not args.dry_run:
        raise SystemExit("--control-subnet override requires --dry-run")
    try:
        selected_policy = authority_policy(
            args.authority_policy,
            args.control_subnet,
            allow_zerotier_dry_run=args.dry_run,
        )
    except ValueError as exc:
        raise SystemExit(f"invalid authority policy: {exc}") from exc
    control_network = selected_policy.network
    if selected_policy.recovery_only and not args.dry_run:
        if args.host != selected_policy.numeric_host:
            raise SystemExit(
                "recovery must bind only to "
                f"{selected_policy.numeric_host}, not {args.host}"
            )
        if not address_is_assigned(
            selected_policy.numeric_host, selected_policy.interface_name
        ):
            raise SystemExit(
                f"{selected_policy.numeric_host} is not assigned to "
                f"{selected_policy.interface_name}; recovery remains in Pause"
            )

    if not PAGE_PATH.is_file():
        raise SystemExit(f"Missing phone interface: {PAGE_PATH}")
    base.PAGE_PATH = PAGE_PATH
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = args.log_dir / f"wifi_primary_control_{stamp}.jsonl"
    operator_key = load_or_create_operator_key(args.operator_key_file)
    try:
        ownership_lock = acquire_ownership_lock(args.ownership_lock)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    state = WifiPrimaryState(args.command_ip, args.dry_run, log_path, selected_policy)
    state.diagnostics_status_path = args.diagnostics_status_file
    # Every process start is a fresh, unarmed control session. Send an explicit
    # Pause burst as well as relying on the Teensy's independent 500 ms timeout.
    state.send_stop_burst("server_startup")
    threading.Thread(
        target=base.udp_listener,
        args=(state, base.STATUS_PORT, "bridge"),
        daemon=True,
    ).start()
    if args.simulate_bridge:
        threading.Thread(target=simulated_bridge_loop, args=(state,), daemon=True).start()
    # Do not bind the dedicated mission-dashboard GPS feed (UDP 6013).
    # Wi-Fi-primary control needs only fresh Teensy bridge telemetry; consuming
    # 6013 here can prevent the separately running mission dashboard from
    # receiving the position needed for distance-to-start guidance.
    threading.Thread(target=state.safety_loop, daemon=True).start()
    server = base.QuietThreadingHTTPServer(
        (args.host, args.port),
        base.handler_factory(state, operator_key, control_network),
    )
    scheme = "http"
    if args.tls_cert:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(args.tls_cert, args.tls_key)
        server.socket = context.wrap_socket(server.socket, server_side=True)
        scheme = "https"
    zero = f"{scheme}://{base.TRACTOR_ZEROTIER_IP}:{args.port}/?key={operator_key}"
    local_host = selected_policy.numeric_host or LOCAL_CONTROL_HOSTNAME
    local = f"{scheme}://{local_host}:{args.port}/?key={operator_key}"
    if selected_policy.recovery_only:
        if not args.no_ntfy:
            threading.Thread(
                target=post_ntfy,
                args=(
                    args.ntfy_topic,
                    "Tractor01 RECOVERY MANUAL ONLY",
                    "Connect to the onboard router and use this Manual-only "
                    "recovery link. Starts in Pause; Auto is disabled.\n\n" + local,
                    local,
                    "tractor,warning",
                ),
                daemon=True,
            ).start()
    else:
        connectivity = ConnectivityMonitor(
            state,
            args.ntfy_topic,
            local,
            zero,
            args.no_ntfy,
            peer_ip=args.zerotier_peer,
            peer_port=args.zerotier_peer_port,
            initial_window_s=args.zerotier_initial_window,
            poll_s=args.network_poll_seconds,
        )
        threading.Thread(target=connectivity.run, daemon=True).start()
    print("Wi-Fi-primary tractor01 control")
    print(f"Expected:  {EXPECTED_FIRMWARE}")
    zero_label = (
        "ZeroTier control"
        if selected_policy.network.overlaps(ZEROTIER_NETWORK)
        else "ZeroTier status only (motion disabled)"
    )
    if not selected_policy.recovery_only:
        print(f"{zero_label}: {zero}")
    print(f"Local control: {local}")
    print(f"Control clients allowed: {control_network}")
    print(
        f"Authority policy: {selected_policy.name}; "
        f"Auto {'allowed' if selected_policy.allow_auto else 'DISABLED'}; "
        f"{selected_policy.qualification}"
    )
    print(
        "Access:   persistent URL key"
        if args.operator_key_file
        else "Access:   temporary URL key (changes at restart)"
    )
    print(f"Log:      {log_path}")
    print(f"UDP 6004: {'DISABLED (--dry-run)' if args.dry_run else args.command_ip}")
    if scheme == "http":
        print("Voice warning: phone microphone recognition may require trusted HTTPS.")
    print(
        "ZeroTier verification: "
        f"{args.zerotier_peer}:{args.zerotier_peer_port}, "
        f"{NETWORK_STABLE_CHECKS} successful checks, "
        f"{args.zerotier_initial_window:.0f}s initial window",
        flush=True,
    )
    print("Ctrl+C sends Pause/neutral before shutdown.")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\nStopping Wi-Fi control...")
    finally:
        server.server_close()
        state.close()
        if ownership_lock is not None:
            ownership_lock.close()


if __name__ == "__main__":
    main()
