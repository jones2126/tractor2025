#!/usr/bin/env python3
"""Wi-Fi-primary phone control server for tractor01.

The phone is the operator control source; NRF is not required. The Teensy
retains the final 500 ms command watchdog and automatically pauses during a
feed interruption. When the same phone heartbeat resumes, the selected Manual
or Auto authority resumes. A deliberate phone STOP/Pause remains latched until
the operator deliberately selects Manual or Auto again.
"""

from __future__ import annotations

import argparse
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
from datetime import datetime
from http import HTTPStatus
from pathlib import Path
from typing import Any
from urllib import error as urllib_error
from urllib import request as urllib_request


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import wifi_manual_control_field_server as base


PAGE_PATH = HERE / "wifi_primary_control_20261003.html"
EXPECTED_FIRMWARE = "teensy_main_20261003_wifi_v3"
LOCAL_CONTROL_HOSTNAME = "raspberrypi.local"
ZEROTIER_PEER_IP = "192.168.193.217"
ZEROTIER_PEER_PORT = 22
ZEROTIER_INITIAL_WINDOW_S = 90.0
NETWORK_POLL_S = 3.0
NETWORK_STABLE_CHECKS = 3
NOTIFY_RETRY_S = 15.0


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
        sent = self.notifier(self.topic, title, message, click_url, tags)
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
                "Tractor01 ZeroTier recovered"
                if recovered
                else "Tractor01 ZeroTier control ready"
            )
            sent = self.send_notice(
                "zerotier",
                now,
                title,
                "Verified through the always-on RPi5NAS peer. Open this temporary "
                "Wi-Fi control link. Tractor control starts in Pause.\n\n"
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
    def bridge_ready(self) -> bool:
        bridge, age = self.bridge_snapshot()
        return (
            age <= base.BRIDGE_FRESH_S
            and bridge.get("system", {}).get("firmware") == EXPECTED_FIRMWARE
        )

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
            "source": "wifi_primary_20261003",
            "phone_sequence": sequence,
            "reason": reason,
        }
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

    def claim(self, client_id: str) -> dict[str, Any]:
        now = time.monotonic()
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
            self.owner_client = client_id
            self.owner_session = secrets.token_urlsafe(16)
            self.owner_at = now
            self.highest_sequence = -1
            bridge, _ = self.bridge_snapshot()
            estop_latched = (
                bridge.get("wifi_control", {}).get("estop_latched") == 1
            )
            self.phone_mode = "estop" if estop_latched else "pause"
            self.last_decision = (
                "phone claimed with E-stop latched"
                if estop_latched
                else "phone claimed in Pause"
            )
            self.stop_sent_for_expiry = False
            session = self.owner_session
            self.log("owner_claimed", client_id=client_id, session=session)
        if estop_latched:
            for _ in range(5):
                self.send_drive_command(
                    0.0, 0.0, -1, "owner_claim_estop", phone_mode="estop"
                )
                time.sleep(0.02)
        else:
            self.send_stop_burst("owner_claim")
        return {
            "session": session,
            "phone_mode": self.phone_mode,
            "rate_hz": 5,
            "heartbeat_timeout_ms": 500,
            "automatic_link_recovery": True,
            "min_drive_percent": -100,
            "max_drive_percent": 100,
        }

    def accept_command(self, data: Any) -> tuple[HTTPStatus, dict[str, Any]]:
        received_monotonic = time.monotonic()
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

        with self.lock:
            if client_id != self.owner_client or session != self.owner_session:
                self.rejected += 1
                return HTTPStatus.CONFLICT, {
                    "accepted": False,
                    "error": "control session is not current",
                    "reclaim": True,
                }
            if sequence <= self.highest_sequence:
                self.rejected += 1
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
            if not base.finite_number(drive) or not -100 <= drive <= 100:
                self.rejected += 1
                return HTTPStatus.BAD_REQUEST, {
                    "accepted": False,
                    "error": "drive demand must be -100 to +100 percent",
                }
            if not base.finite_number(steering) or not -100 <= steering <= 100:
                self.rejected += 1
                return HTTPStatus.BAD_REQUEST, {
                    "accepted": False,
                    "error": "steering must be -100 to +100 percent",
                }
            if (
                requested_mode != self.phone_mode
                and requested_mode in ("manual", "auto")
                and reason != f"guarded_{requested_mode}"
            ):
                self.rejected += 1
                return HTTPStatus.CONFLICT, {
                    "accepted": False,
                    "error": f"{requested_mode.title()} requires MODE SELECT + {requested_mode.title()}",
                }
            if (
                self.phone_mode == "estop"
                and requested_mode in ("manual", "auto")
            ):
                self.rejected += 1
                return HTTPStatus.CONFLICT, {
                    "accepted": False,
                    "error": "E-stop is latched; use MODE SELECT + E-STOP to reset to Pause",
                }
            if (
                requested_mode in ("manual", "auto")
                and not self.bridge_ready()
            ):
                self.rejected += 1
                return HTTPStatus.SERVICE_UNAVAILABLE, {
                    "accepted": False,
                    "error": "Teensy bridge or expected firmware is not fresh",
                }

            self.highest_sequence = sequence
            self.owner_at = received_monotonic
            self.stop_sent_for_expiry = False
            reset_estop = (
                self.phone_mode == "estop"
                and requested_mode == "pause"
                and reason == "guarded_estop_reset"
            )
            if self.phone_mode == "estop" and requested_mode == "pause" and not reset_estop:
                requested_mode = "estop"
            wire_mode = "reset_pause" if reset_estop else requested_mode
            self.phone_mode = "pause" if reset_estop else requested_mode
            self.last_decision = wire_mode
            effective_drive = drive if self.phone_mode == "manual" else 0.0
            effective_steering = steering if self.phone_mode == "manual" else 0.0
            self.last_command = {
                "client_id": client_id,
                "sequence": sequence,
                "mode": self.phone_mode,
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
        processing_ms = round((time.monotonic() - received_monotonic) * 1000, 2)
        phone_to_server_ms = (
            round(received_epoch_ms - client_time_ms, 2)
            if math.isfinite(client_time_ms)
            else None
        )
        self.log(
            "command",
            **self.last_command,
            client_time_ms=client_time_ms if math.isfinite(client_time_ms) else None,
            client_reported_rtt_ms=(
                round(client_rtt_ms, 2) if math.isfinite(client_rtt_ms) else None
            ),
            estimated_phone_to_server_ms=phone_to_server_ms,
            server_processing_ms=processing_ms,
        )
        return HTTPStatus.OK, {
            "accepted": True,
            "decision": wire_mode,
            "sequence": sequence,
            "server_time_ms": round(time.time() * 1000),
            "phone_mode": self.phone_mode,
        }

    def snapshot(self) -> dict[str, Any]:
        result = super().snapshot()
        bridge, _ = self.bridge_snapshot()
        result["wifi_control"] = bridge.get("wifi_control", {})
        result["wifi_primary_ready"] = self.bridge_ready()
        result["expected_firmware"] = EXPECTED_FIRMWARE
        return result

    def safety_loop(self) -> None:
        # Keep the operator's requested mode across a temporary link outage.
        # A Pause packet is sent once on expiry, and the next valid phone
        # heartbeat automatically restores the retained Manual or Auto state.
        while self.running:
            should_stop = False
            sequence = -1
            with self.lock:
                age = time.monotonic() - self.owner_at if self.owner_at else math.inf
                if (
                    self.phone_mode in ("manual", "auto")
                    and age > base.PHONE_FRESH_S
                    and not self.stop_sent_for_expiry
                ):
                    should_stop = True
                    self.stop_sent_for_expiry = True
                    self.last_decision = "heartbeat expired; Teensy paused"
                    if self.last_command:
                        sequence = int(self.last_command.get("sequence", -1))
            if should_stop:
                self.send_stop_burst("phone_freshness_expired", sequence)
            time.sleep(0.05)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--command-ip", default="127.0.0.1")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--log-dir", type=Path, default=Path("/home/al/field_logs"))
    parser.add_argument("--tls-cert", type=Path)
    parser.add_argument("--tls-key", type=Path)
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
    args = parser.parse_args()

    if bool(args.tls_cert) != bool(args.tls_key):
        raise SystemExit("--tls-cert and --tls-key must be supplied together")

    if not PAGE_PATH.is_file():
        raise SystemExit(f"Missing phone interface: {PAGE_PATH}")
    base.PAGE_PATH = PAGE_PATH
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = args.log_dir / f"wifi_primary_control_{stamp}.jsonl"
    operator_key = secrets.token_urlsafe(18)
    state = WifiPrimaryState(args.command_ip, args.dry_run, log_path)
    # Every process start is a fresh, unarmed control session. Send an explicit
    # Pause burst as well as relying on the Teensy's independent 500 ms timeout.
    state.send_stop_burst("server_startup")
    threading.Thread(
        target=base.udp_listener,
        args=(state, base.STATUS_PORT, "bridge"),
        daemon=True,
    ).start()
    # Do not bind the dedicated mission-dashboard GPS feed (UDP 6013).
    # Wi-Fi-primary control needs only fresh Teensy bridge telemetry; consuming
    # 6013 here can prevent the separately running mission dashboard from
    # receiving the position needed for distance-to-start guidance.
    threading.Thread(target=state.safety_loop, daemon=True).start()
    server = base.QuietThreadingHTTPServer(
        (args.host, args.port), base.handler_factory(state, operator_key)
    )
    scheme = "http"
    if args.tls_cert:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(args.tls_cert, args.tls_key)
        server.socket = context.wrap_socket(server.socket, server_side=True)
        scheme = "https"
    zero = f"{scheme}://{base.TRACTOR_ZEROTIER_IP}:{args.port}/?key={operator_key}"
    local = f"{scheme}://{LOCAL_CONTROL_HOSTNAME}:{args.port}/?key={operator_key}"
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
    print(f"ZeroTier: {zero}")
    print(f"Local:    {local}")
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


if __name__ == "__main__":
    main()
