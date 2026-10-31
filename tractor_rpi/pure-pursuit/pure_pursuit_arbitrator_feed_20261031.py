#!/usr/bin/env python3
"""Run the frozen Pure Pursuit controller with an additive safety feed.

The wrapper duplicates its existing dashboard telemetry to a dedicated local
port, adds a normalized-LF SHA-256 of the exact mission file, and leaves the
original command and dashboard destinations unchanged.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import runpy
import socket
import sys


SAFETY_OPTION = "--arbitrator-safety-port"
DEFAULT_SAFETY_PORT = 6015


def normalized_lf_sha256(path: Path) -> str:
    data = path.read_bytes().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return hashlib.sha256(data).hexdigest()


def pop_option(argv: list[str], name: str, default: int) -> int:
    if name not in argv:
        return default
    index = argv.index(name)
    try:
        value = int(argv[index + 1])
    except (IndexError, ValueError) as exc:
        raise SystemExit(f"{name} requires an integer port") from exc
    del argv[index:index + 2]
    return value


class SafetyDuplicatingSocket:
    def __init__(
        self,
        original_socket,
        *,
        telemetry_port: int,
        safety_port: int,
        route_sha256: str,
    ):
        self._socket = original_socket
        self._telemetry_port = telemetry_port
        self._safety_port = safety_port
        self._route_sha256 = route_sha256
        self._sequence = 0

    def sendto(self, data, address):
        result = self._socket.sendto(data, address)
        if address[1] == self._telemetry_port:
            try:
                payload = json.loads(data.decode("utf-8"))
                self._sequence += 1
                payload.update(
                    schema=1,
                    kind="pure_pursuit_safety",
                    route_sha256=self._route_sha256,
                    safety_sequence=self._sequence,
                )
                # Non-running rows omit command fields; zero is the only safe
                # proposal in those states.
                payload.setdefault("speed_cmd_mps", 0.0)
                payload.setdefault("steer_normalized", 0.0)
                payload.setdefault("cross_track_err_m", 0.0)
                self._socket.sendto(
                    json.dumps(payload, separators=(",", ":")).encode("utf-8"),
                    ("127.0.0.1", self._safety_port),
                )
            except (KeyError, TypeError, ValueError, UnicodeError, json.JSONDecodeError):
                pass
        return result

    def __getattr__(self, name):
        return getattr(self._socket, name)


def main() -> None:
    argv = list(sys.argv)
    safety_port = pop_option(argv, SAFETY_OPTION, DEFAULT_SAFETY_PORT)
    if len(argv) < 2:
        raise SystemExit("mission file must be the first argument")
    mission_file = Path(argv[1]).resolve()
    if not mission_file.is_file():
        raise SystemExit(f"mission file not found: {mission_file}")
    try:
        telemetry_index = argv.index("--telemetry-port")
        telemetry_port = int(argv[telemetry_index + 1])
    except (ValueError, IndexError) as exc:
        raise SystemExit("--telemetry-port is required for arbitrator feed") from exc
    if telemetry_port <= 0 or safety_port <= 0 or telemetry_port == safety_port:
        raise SystemExit("dashboard and arbitrator telemetry ports must be distinct")

    route_hash = normalized_lf_sha256(mission_file)
    original_socket_class = socket.socket

    def socket_factory(*args, **kwargs):
        return SafetyDuplicatingSocket(
            original_socket_class(*args, **kwargs),
            telemetry_port=telemetry_port,
            safety_port=safety_port,
            route_sha256=route_hash,
        )

    socket.socket = socket_factory
    sys.argv = argv
    controller = Path(__file__).with_name("pure_pursuit_controller_20261022.py")
    runpy.run_path(str(controller), run_name="__main__")


if __name__ == "__main__":
    main()
