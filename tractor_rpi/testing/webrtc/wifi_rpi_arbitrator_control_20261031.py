#!/usr/bin/env python3
"""Additive phone server that publishes explicit arbitrator intent samples.

Legacy UDP 6004 output is retained for a motor-off comparison only. The new
versioned side feed preserves requested AUTO during an operator-link outage,
while explicitly marking the operator link stale. No installed service points
to this file.
"""

from __future__ import annotations

import json
import os
import secrets
import socket
import threading
import time
from typing import Any

import wifi_primary_control_20261022 as base

from tractor_rpi.rpi_arbitrator_wire_20261031 import PHONE_INTENT_PORT


class ArbitratorWifiState(base.WifiPrimaryState):
    def __init__(self, *args: Any, **kwargs: Any):
        self._intent_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._intent_sequence = 0
        self._intent_producer_session = secrets.token_urlsafe(16)
        self._intent_lock = threading.Lock()
        super().__init__(*args, **kwargs)

    def send_drive_command(
        self,
        drive_percent: float,
        steering_percent: float,
        sequence: int,
        reason: str,
        phone_mode: str | None = None,
    ) -> None:
        super().send_drive_command(
            drive_percent,
            steering_percent,
            sequence,
            reason,
            phone_mode=phone_mode,
        )
        now = self.clock()
        # A separate lock avoids re-entering the base state lock when this is
        # called from one of its safety transitions.
        with self._intent_lock:
            requested_mode = self.requested_mode
            operator_link_fresh = bool(
                self.owner_session
                and self.owner_at
                and now - self.owner_at <= base.SERVER_PHONE_FRESH_S
            )
            effective_state = self.effective_state
            browser_session = self.owner_session
            self._intent_sequence += 1
            intent_sequence = self._intent_sequence
        payload = {
            "schema": 1,
            "kind": "phone_intent",
            "producer_session": self._intent_producer_session,
            "intent_sequence": intent_sequence,
            "requested_mode": requested_mode,
            "operator_link_fresh": operator_link_fresh,
            "manual_drive_percent": (
                round(float(drive_percent), 1) if requested_mode == "manual" else 0.0
            ),
            "manual_steering": (
                round(-float(steering_percent) / 100.0, 3)
                if requested_mode == "manual" else 0.0
            ),
            "effective_state": effective_state,
            "browser_session": browser_session,
            "browser_sequence": sequence,
            "wire_mode": phone_mode or self.phone_mode,
            "reason": reason,
            "emitted_monotonic_s": now,
            "emitted_epoch_ms": round(time.time() * 1000),
        }
        self._intent_sock.sendto(
            json.dumps(payload, separators=(",", ":")).encode("utf-8"),
            ("127.0.0.1", PHONE_INTENT_PORT),
        )


def main() -> None:
    expected_firmware = os.environ.get("TRACTOR_ARBITRATOR_EXPECTED_FIRMWARE")
    if expected_firmware:
        base.EXPECTED_FIRMWARE = expected_firmware
    base.WifiPrimaryState = ArbitratorWifiState
    base.main()


if __name__ == "__main__":
    main()
