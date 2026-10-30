#!/usr/bin/env python3
"""Republish Tractor01's existing ZeroTier phone-control URL to ntfy.

This does not restart the control service, change its persistent key, claim a
control session, or send a motion command.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess

import wifi_primary_control_20261022 as primary


DEFAULT_KEY_FILE = Path("/home/al/.config/tractor-wifi-control/operator.key")
DEFAULT_TOPIC = "rpi-tractor01-jones2126"


def read_operator_key(path: Path) -> str:
    key = path.read_text(encoding="ascii").strip()
    if not 24 <= len(key) <= 128 or any(
        not (character.isalnum() or character in "-_") for character in key
    ):
        raise ValueError(f"invalid operator key file: {path}")
    return key


def service_is_active() -> bool:
    result = subprocess.run(
        ["systemctl", "is-active", "--quiet", "tractor-wifi-control.service"],
        check=False,
        timeout=5,
    )
    return result.returncode == 0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--operator-key-file", type=Path, default=DEFAULT_KEY_FILE)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--tractor-ip", default=primary.base.TRACTOR_ZEROTIER_IP)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--skip-service-check",
        action="store_true",
        help="for an isolated bench test only",
    )
    args = parser.parse_args()

    if not args.skip_service_check and not service_is_active():
        raise SystemExit(
            "tractor-wifi-control.service is not active; remain in Pause and inspect it"
        )
    try:
        key = read_operator_key(args.operator_key_file)
    except (OSError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc

    url = f"https://{args.tractor_ip}:{args.port}/?key={key}"
    sent = primary.post_ntfy(
        args.topic,
        "Tractor01 ZeroTier phone control link",
        "Open the existing Tractor01 phone-control page. It starts in Pause.\n\n"
        + url,
        url,
        "tractor,satellite",
    )
    if not sent:
        raise SystemExit("ntfy did not acknowledge the URL notification")
    print("Existing ZeroTier phone-control URL republished; the service was not restarted.")


if __name__ == "__main__":
    main()

