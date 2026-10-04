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
import json
import math
import os
import secrets
import socket
import ssl
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


def notify_control_urls(topic: str, zerotier_url: str, local_url: str) -> None:
    message = (
        "Open the Wi-Fi-primary Tractor01 control page. The temporary link "
        "contains the operator key.\n\n"
        f"ZeroTier: {zerotier_url}\nLocal Wi-Fi: {local_url}"
    )
    request = urllib_request.Request(
        f"https://ntfy.sh/{topic}",
        data=message.encode("utf-8"),
        headers={
            "Title": "Tractor01 Wi-Fi control ready",
            "Click": zerotier_url,
            "Tags": "tractor",
        },
        method="POST",
    )
    try:
        with urllib_request.urlopen(request, timeout=5) as response:
            if not 200 <= response.status < 300:
                raise RuntimeError(f"ntfy returned HTTP {response.status}")
        print(f"ntfy: control links sent to https://ntfy.sh/{topic}")
    except (urllib_error.URLError, OSError, RuntimeError) as exc:
        print(f"WARNING: could not send control links to ntfy: {exc}")


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
    local = f"{scheme}://{base.TRACTOR_LOCAL_IP}:{args.port}/?key={operator_key}"
    print("Wi-Fi-primary tractor01 control")
    print(f"Expected:  {EXPECTED_FIRMWARE}")
    print(f"ZeroTier: {zero}")
    print(f"Local:    {local}")
    print(f"Log:      {log_path}")
    print(f"UDP 6004: {'DISABLED (--dry-run)' if args.dry_run else args.command_ip}")
    if scheme == "http":
        print("Voice warning: phone microphone recognition may require trusted HTTPS.")
    if not args.no_ntfy:
        notify_control_urls(args.ntfy_topic, zero, local)
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
