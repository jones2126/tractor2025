#!/usr/bin/env python3
"""NRF-supervised Wi-Fi manual-control field experiment for tractor01.

The physical handheld remains the authoritative mode selector. For phone
driving, the handheld must be in AUTO; this server then publishes bounded
forward-only cmd_vel messages to the existing localhost UDP 6004 bridge path.
Handheld PAUSE, NRF loss, the Teensy's cmd_vel timeout, or loss of the phone
session stops motion. This program does not provide NRF-independent control.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import secrets
import socket
import threading
import time
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib import error as urllib_error
from urllib import request as urllib_request
from urllib.parse import parse_qs, urlparse


HERE = Path(__file__).resolve().parent
PAGE_PATH = HERE / "wifi_manual_control_field.html"
TRACTOR_ZEROTIER_IP = "192.168.193.76"
TRACTOR_LOCAL_IP = "192.168.1.151"
STATUS_PORT = 6003
GPS_PORT = 6002
CMD_VEL_PORT = 6004
MAX_FORWARD_MPS = 0.30
MAX_STEERING_PERCENT = 100
BRIDGE_FRESH_S = 0.75
PHONE_FRESH_S = 0.80
OWNER_EXPIRE_S = 3.0
DEFAULT_NTFY_TOPIC = "rpi-tractor01-jones2126"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def finite_number(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def notify_control_urls(topic: str, zerotier_url: str, local_url: str) -> None:
    """Send the temporary operator URLs without preventing server startup."""
    message = (
        "Open the NRF-supervised Wi-Fi control page. This temporary link "
        "includes the operator key.\n\n"
        f"ZeroTier: {zerotier_url}\n"
        f"Local Wi-Fi: {local_url}"
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


class FieldState:
    def __init__(self, command_ip: str, dry_run: bool, log_path: Path) -> None:
        self.lock = threading.RLock()
        self.command_ip = command_ip
        self.dry_run = dry_run
        self.bridge: dict[str, Any] = {}
        self.bridge_at = 0.0
        self.gps: dict[str, Any] = {}
        self.gps_at = 0.0
        self.owner_client: str | None = None
        self.owner_session: str | None = None
        self.owner_at = 0.0
        self.highest_sequence = -1
        self.phone_mode = "pause"
        self.last_command: dict[str, Any] | None = None
        self.last_decision = "waiting for handheld Pause"
        self.last_stop_reason = "startup"
        self.stop_sent_for_expiry = False
        self.accepted = 0
        self.rejected = 0
        self.udp_sent = 0
        self.running = True
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_path = log_path
        self.log_file = log_path.open("a", encoding="utf-8", buffering=1)
        self.log("server_start", dry_run=dry_run, command_ip=command_ip)

    def log(self, event: str, **fields: Any) -> None:
        record = {"time": utc_now(), "event": event, **fields}
        self.log_file.write(json.dumps(record, separators=(",", ":")) + "\n")

    def bridge_snapshot(self) -> tuple[dict[str, Any], float]:
        with self.lock:
            return dict(self.bridge), time.monotonic() - self.bridge_at if self.bridge_at else math.inf

    def actual_mode(self) -> int | None:
        bridge, age = self.bridge_snapshot()
        if age > BRIDGE_FRESH_S:
            return None
        transmission = bridge.get("transmission", {})
        steering = bridge.get("steering", {})
        trans_mode = transmission.get("mode")
        steer_mode = steering.get("mode")
        if trans_mode == steer_mode and finite_number(trans_mode):
            return int(trans_mode)
        return None

    def radio_good(self) -> bool:
        bridge, age = self.bridge_snapshot()
        return age <= BRIDGE_FRESH_S and bridge.get("radio", {}).get("signal") == "GOOD"

    def send_cmd_vel(self, speed_mps: float, steering_percent: float, sequence: int, reason: str) -> None:
        payload = {
            "linear_x": round(speed_mps, 3),
            # Existing Teensy convention: angular_z +1 is full left.
            "angular_z": round(-steering_percent / 100.0, 3),
            "timestamp": time.time(),
            "source": "wifi_manual_nrf_supervised",
            "phone_sequence": sequence,
            "reason": reason,
        }
        if not self.dry_run:
            self.cmd_sock.sendto(json.dumps(payload).encode("utf-8"), (self.command_ip, CMD_VEL_PORT))
        self.udp_sent += 1

    def send_stop_burst(self, reason: str, sequence: int = -1, steering_percent: float = 0.0) -> None:
        for _ in range(5):
            self.send_cmd_vel(0.0, steering_percent, sequence, reason)
            time.sleep(0.02)
        self.last_stop_reason = reason
        self.log("stop_burst", reason=reason, sequence=sequence, steering_percent=steering_percent)

    def claim(self, client_id: str) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            if not client_id or len(client_id) > 100:
                raise ValueError("invalid client identifier")
            if self.owner_client and self.owner_client != client_id and now - self.owner_at <= OWNER_EXPIRE_S:
                raise RuntimeError("another phone currently owns control")
            mode = self.actual_mode()
            if mode != 2 or not self.radio_good():
                raise RuntimeError("put the physical handheld in Pause with a good NRF link before connecting")
            self.owner_client = client_id
            self.owner_session = secrets.token_urlsafe(16)
            self.owner_at = now
            self.highest_sequence = -1
            self.phone_mode = "pause"
            self.last_decision = "phone claimed; handheld Pause confirmed"
            self.stop_sent_for_expiry = False
            session = self.owner_session
            self.log("owner_claimed", client_id=client_id, session=session)
        self.send_stop_burst("owner_claim", steering_percent=0.0)
        return {"session": session, "phone_mode": "pause", "rate_hz": 5, "max_forward_mps": MAX_FORWARD_MPS}

    def accept_command(self, data: Any) -> tuple[HTTPStatus, dict[str, Any]]:
        received = time.monotonic()
        if not isinstance(data, dict):
            return HTTPStatus.BAD_REQUEST, {"accepted": False, "error": "JSON object required"}
        try:
            client_id = str(data["client_id"])
            session = str(data["session"])
            sequence = int(data["sequence"])
            phone_mode = str(data["mode"]).lower()
            speed = float(data["speed_mps"])
            steering = float(data["steering_percent"])
            reason = str(data.get("reason", "heartbeat"))[:40]
        except (KeyError, TypeError, ValueError):
            return HTTPStatus.BAD_REQUEST, {"accepted": False, "error": "malformed command"}

        with self.lock:
            if client_id != self.owner_client or session != self.owner_session:
                self.rejected += 1
                return HTTPStatus.CONFLICT, {"accepted": False, "error": "control session is not current", "reclaim": True}
            if sequence <= self.highest_sequence:
                self.rejected += 1
                self.log("command_rejected", reason="stale_sequence", sequence=sequence, highest=self.highest_sequence)
                return HTTPStatus.CONFLICT, {"accepted": False, "error": "stale or duplicate sequence", "highest": self.highest_sequence}
            if phone_mode not in ("manual", "pause"):
                self.rejected += 1
                return HTTPStatus.BAD_REQUEST, {"accepted": False, "error": "this experiment supports phone Manual or Pause only"}
            if not finite_number(speed) or not 0.0 <= speed <= MAX_FORWARD_MPS:
                self.rejected += 1
                return HTTPStatus.BAD_REQUEST, {"accepted": False, "error": f"speed must be 0.00 to {MAX_FORWARD_MPS:.2f} m/s"}
            if not finite_number(steering) or not -MAX_STEERING_PERCENT <= steering <= MAX_STEERING_PERCENT:
                self.rejected += 1
                return HTTPStatus.BAD_REQUEST, {"accepted": False, "error": "steering must be -100 to +100 percent"}
            if self.phone_mode == "pause" and phone_mode == "manual" and reason != "guarded_manual":
                self.rejected += 1
                return HTTPStatus.CONFLICT, {"accepted": False, "error": "Manual requires a new guarded MODE SELECT action"}

            self.highest_sequence = sequence
            self.owner_at = received
            self.stop_sent_for_expiry = False
            actual_mode = self.actual_mode()
            radio_good = self.radio_good()

            if phone_mode == "pause" or reason == "guarded_stop":
                self.phone_mode = "pause"
                self.last_decision = "Pause accepted; neutral burst sent"
                decision = "pause"
            elif actual_mode != 0 or not radio_good:
                self.phone_mode = "pause"
                self.last_decision = "rejected motion: handheld is not fresh AUTO with good NRF"
                self.rejected += 1
                self.log("command_rejected", reason="handheld_not_auto", sequence=sequence, actual_mode=actual_mode, radio_good=radio_good)
                decision = "handheld_not_auto"
            else:
                self.phone_mode = "manual"
                self.last_decision = "bounded cmd_vel accepted"
                decision = "drive"

            command = {
                "client_id": client_id,
                "session": session,
                "sequence": sequence,
                "mode": self.phone_mode,
                "speed_mps": round(speed, 2),
                "steering_percent": round(steering),
                "reason": reason,
                "decision": decision,
            }
            self.last_command = command
            if decision in ("pause", "drive"):
                self.accepted += 1

        if decision == "drive":
            self.send_cmd_vel(speed, steering, sequence, reason)
        else:
            self.send_stop_burst(decision, sequence, steering)

        self.log("command", **command, actual_mode=actual_mode, radio_good=radio_good)
        status = HTTPStatus.OK if decision in ("pause", "drive") else HTTPStatus.CONFLICT
        return status, {
            "accepted": decision in ("pause", "drive"),
            "decision": decision,
            "sequence": sequence,
            "server_time_ms": round(time.time() * 1000),
            "phone_mode": self.phone_mode,
        }

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            bridge_age = None if not self.bridge_at else round(now - self.bridge_at, 3)
            gps_age = None if not self.gps_at else round(now - self.gps_at, 3)
            owner_age = None if not self.owner_at else round(now - self.owner_at, 3)
            bridge = dict(self.bridge)
            gps = dict(self.gps)
            last_command = dict(self.last_command) if self.last_command else None
            return {
                "server_time": utc_now(),
                "dry_run": self.dry_run,
                "phone_mode": self.phone_mode,
                "owner_connected": owner_age is not None and owner_age <= PHONE_FRESH_S,
                "owner_age_s": owner_age,
                "bridge_age_s": bridge_age,
                "gps_age_s": gps_age,
                "actual_mode": self.actual_mode(),
                "radio_good": self.radio_good(),
                "radio": bridge.get("radio", {}),
                "steering": bridge.get("steering", {}),
                "transmission": bridge.get("transmission", {}),
                "gps": gps,
                "last_command": last_command,
                "last_decision": self.last_decision,
                "last_stop_reason": self.last_stop_reason,
                "accepted": self.accepted,
                "rejected": self.rejected,
                "udp_sent": self.udp_sent,
            }

    def safety_loop(self) -> None:
        while self.running:
            should_stop = False
            reason = ""
            sequence = -1
            steering = 0.0
            with self.lock:
                owner_age = time.monotonic() - self.owner_at if self.owner_at else math.inf
                actual_mode = self.actual_mode()
                if self.phone_mode == "manual" and owner_age > PHONE_FRESH_S:
                    reason = "phone_freshness_expired"
                    should_stop = not self.stop_sent_for_expiry
                elif self.phone_mode == "manual" and (actual_mode != 0 or not self.radio_good()):
                    reason = "handheld_left_auto_or_nrf_lost"
                    should_stop = not self.stop_sent_for_expiry
                if should_stop:
                    self.phone_mode = "pause"
                    self.last_decision = reason
                    self.stop_sent_for_expiry = True
                    if self.last_command:
                        sequence = int(self.last_command.get("sequence", -1))
                        steering = float(self.last_command.get("steering_percent", 0.0))
            if should_stop:
                self.send_stop_burst(reason, sequence, steering)
            time.sleep(0.05)

    def close(self) -> None:
        self.running = False
        try:
            self.send_stop_burst("server_shutdown")
        finally:
            self.log("server_stop")
            self.log_file.close()
            self.cmd_sock.close()


def udp_listener(state: FieldState, port: int, kind: str) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except OSError:
            pass
    sock.bind(("", port))
    sock.settimeout(0.5)
    while state.running:
        try:
            data, _ = sock.recvfrom(65535)
            message = json.loads(data.decode("utf-8"))
            with state.lock:
                if kind == "bridge":
                    state.bridge = message
                    state.bridge_at = time.monotonic()
                else:
                    state.gps = message
                    state.gps_at = time.monotonic()
        except socket.timeout:
            continue
        except (OSError, ValueError, UnicodeDecodeError):
            continue
    sock.close()


def handler_factory(state: FieldState, operator_key: str):
    page = PAGE_PATH.read_bytes()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "TractorWifiFieldTest/1.0"

        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def authorized(self) -> bool:
            query_key = parse_qs(urlparse(self.path).query).get("key", [""])[0]
            supplied = self.headers.get("X-Operator-Key", "") or query_key
            return secrets.compare_digest(supplied, operator_key)

        def send_bytes(self, status: HTTPStatus, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
            self.send_bytes(status, "application/json; charset=utf-8", json.dumps(payload, separators=(",", ":")).encode())

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if not self.authorized():
                self.send_json(HTTPStatus.FORBIDDEN, {"error": "invalid operator key"})
                return
            if path == "/":
                self.send_bytes(HTTPStatus.OK, "text/html; charset=utf-8", page)
            elif path == "/api/state":
                self.send_json(HTTPStatus.OK, state.snapshot())
            else:
                self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            if not self.authorized():
                self.send_json(HTTPStatus.FORBIDDEN, {"error": "invalid operator key"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 8192:
                    raise ValueError("invalid request size")
                data = json.loads(self.rfile.read(length).decode("utf-8"))
            except (ValueError, UnicodeDecodeError, json.JSONDecodeError) as exc:
                self.send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
                return
            path = urlparse(self.path).path
            try:
                if path == "/api/claim":
                    result = state.claim(str(data.get("client_id", "")))
                    self.send_json(HTTPStatus.OK, result)
                    return
                if path == "/api/command":
                    status, result = state.accept_command(data)
                    self.send_json(status, result)
                    return
                self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            except (RuntimeError, ValueError) as exc:
                self.send_json(HTTPStatus.CONFLICT, {"error": str(exc)})

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--command-ip", default="127.0.0.1")
    parser.add_argument("--dry-run", action="store_true", help="log accepted commands without sending UDP 6004")
    parser.add_argument("--log-dir", type=Path, default=Path("/home/al/field_logs"))
    parser.add_argument(
        "--ntfy-topic",
        default=os.environ.get("TRACTOR_NTFY_TOPIC", DEFAULT_NTFY_TOPIC),
        help="ntfy.sh topic for the temporary control URLs",
    )
    parser.add_argument("--no-ntfy", action="store_true", help="do not send the startup URLs to ntfy.sh")
    args = parser.parse_args()

    if not PAGE_PATH.is_file():
        raise SystemExit(f"Missing phone interface: {PAGE_PATH}")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = args.log_dir / f"wifi_manual_control_{stamp}.jsonl"
    operator_key = secrets.token_urlsafe(18)
    state = FieldState(args.command_ip, args.dry_run, log_path)
    threading.Thread(target=udp_listener, args=(state, STATUS_PORT, "bridge"), daemon=True).start()
    threading.Thread(target=udp_listener, args=(state, GPS_PORT, "gps"), daemon=True).start()
    threading.Thread(target=state.safety_loop, daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.port), handler_factory(state, operator_key))

    zerotier_url = f"http://{TRACTOR_ZEROTIER_IP}:{args.port}/?key={operator_key}"
    local_url = f"http://{TRACTOR_LOCAL_IP}:{args.port}/?key={operator_key}"

    print("NRF-supervised Wi-Fi manual-control FIELD EXPERIMENT")
    print("Blades off. Keep the handheld and physical emergency stop available.")
    print("Start with the handheld in Pause; use handheld Auto only while phone control is armed.")
    print(f"ZeroTier: {zerotier_url}")
    print(f"Local:    {local_url}")
    print(f"Log:      {log_path}")
    print(f"UDP 6004: {'DISABLED (--dry-run)' if args.dry_run else args.command_ip}")
    if not args.no_ntfy:
        notify_control_urls(args.ntfy_topic, zerotier_url, local_url)
    print("Press Ctrl+C to stop; shutdown sends a neutral command burst.")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\nStopping field experiment...")
    finally:
        server.server_close()
        state.close()


if __name__ == "__main__":
    main()
