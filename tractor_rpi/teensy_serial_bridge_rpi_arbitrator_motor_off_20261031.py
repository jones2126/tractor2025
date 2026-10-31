#!/usr/bin/env python3
"""Motor-off live consumer for versioned RPi arbitrator feeds.

This bridge extends the proven Pause-only command-lease bridge. It receives
explicit phone intent on UDP 6014 and Pure Pursuit safety telemetry on UDP
6015, evaluates the full supervisor, and publishes the joined decision for
inspection. Its actual serial scheduler remains the inherited hard-coded
Pause-only scheduler: no feed can make this file send Manual or AUTO.

No installed service references this file.
"""

from __future__ import annotations

from dataclasses import asdict
import json
import math
import os
import select
import socket
import threading
import time
from typing import Any

from tractor_rpi.rpi_arbitrator_inputs_20261031 import (
    RpiArbitratorInputCollector,
    TeensySafetySample,
)
from tractor_rpi.rpi_arbitrator_wire_20261031 import (
    PHONE_INTENT_PORT,
    PURE_PURSUIT_SAFETY_PORT,
    parse_phone_intent,
    parse_pure_pursuit_safety,
)
from tractor_rpi.rpi_safety_supervisor_20261031 import RpiSafetySupervisor
from tractor_rpi.teensy_serial_bridge_rpi_lease_pause_20261031 import (
    EXPECTED_FIRMWARE,
    TeensySerialBridge as PauseOnlyBridge,
    base,
)


def validated_route_hash(value: str) -> str:
    result = value.strip().lower()
    if len(result) != 64 or any(c not in "0123456789abcdef" for c in result):
        raise ValueError("TRACTOR_EXPECTED_ROUTE_SHA256 must be 64 hex characters")
    return result


class ArbitratorFeedReceiver:
    def __init__(self, expected_route_sha256: str):
        self.collector = RpiArbitratorInputCollector(
            expected_route_sha256=expected_route_sha256,
            # This executable is motor-off and therefore never authorizes the
            # degraded state or motion, even for its inspection-only decision.
            degraded_preauthorized=False,
        )
        self.supervisor = RpiSafetySupervisor()
        self.accepted_phone = 0
        self.accepted_pursuit = 0
        self.rejected = 0
        self.last_rejection_reason: str | None = None
        self.last_phone_at: float | None = None
        self.last_pursuit_at: float | None = None
        self._lock = threading.Lock()

    def ingest(self, payload: Any, *, now_monotonic_s: float) -> None:
        try:
            if isinstance(payload, dict) and payload.get("kind") == "phone_intent":
                sample = parse_phone_intent(
                    payload, received_monotonic_s=now_monotonic_s
                )
                with self._lock:
                    self.collector.ingest_phone(sample)
                    self.accepted_phone += 1
                    self.last_phone_at = now_monotonic_s
            elif isinstance(payload, dict) and payload.get("kind") == "pure_pursuit_safety":
                command, health = parse_pure_pursuit_safety(
                    payload, received_monotonic_s=now_monotonic_s
                )
                with self._lock:
                    self.collector.ingest_pure_pursuit_command(command)
                    self.collector.ingest_pure_pursuit_health(health)
                    self.accepted_pursuit += 1
                    self.last_pursuit_at = now_monotonic_s
            else:
                raise ValueError("unknown arbitrator feed kind")
        except (TypeError, ValueError) as exc:
            with self._lock:
                self.rejected += 1
                self.last_rejection_reason = str(exc)

    def snapshot(
        self,
        *,
        now_monotonic_s: float,
        teensy_sample: TeensySafetySample | None,
    ) -> dict[str, Any]:
        with self._lock:
            if teensy_sample is not None:
                self.collector.ingest_teensy(teensy_sample)
            inputs = self.collector.build(now_monotonic_s=now_monotonic_s)
            # The persistent observer shows fail-latched history. A fresh
            # supervisor also reports the current measured state so expected
            # startup telemetry arrival does not masquerade as a current
            # steering/transmission fault. Actual serial output is separately
            # and unconditionally Pause-only.
            latched_decision = self.supervisor.evaluate(inputs)
            current_decision = RpiSafetySupervisor().evaluate(inputs)
            guard = (
                None if current_decision.guard is None
                else asdict(current_decision.guard)
            )
            return {
                "schema": 1,
                "approved_for_motion": False,
                "actual_serial_output": "PAUSE_ONLY",
                "expected_firmware": EXPECTED_FIRMWARE,
                "accepted_phone_samples": self.accepted_phone,
                "accepted_pure_pursuit_samples": self.accepted_pursuit,
                "rejected_samples": self.rejected,
                "last_rejection_reason": self.last_rejection_reason,
                "phone_feed_age_s": (
                    None if self.last_phone_at is None
                    else max(0.0, now_monotonic_s - self.last_phone_at)
                ),
                "pure_pursuit_feed_age_s": (
                    None if self.last_pursuit_at is None
                    else max(0.0, now_monotonic_s - self.last_pursuit_at)
                ),
                "joined_decision": current_decision.state.value,
                "joined_mode": int(current_decision.mode),
                "joined_drive": current_decision.drive,
                "joined_steering": current_decision.steering,
                "exact_reason": current_decision.reason,
                "latched_observer_decision": latched_decision.state.value,
                "latched_observer_reason": latched_decision.reason,
                "guard": guard,
                "health": asdict(inputs.health),
                "additional_local_faults": list(inputs.additional_local_faults),
                "input_fault": self.collector.input_fault,
            }


def make_udp_listener(port: int) -> socket.socket:
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", port))
    listener.setblocking(False)
    return listener


class TeensySerialBridge(PauseOnlyBridge):
    def __init__(self):
        route_hash = validated_route_hash(
            os.environ.get("TRACTOR_EXPECTED_ROUTE_SHA256", "")
        )
        self.arbitrator_feeds = ArbitratorFeedReceiver(route_hash)
        self._phone_intent_sock = make_udp_listener(PHONE_INTENT_PORT)
        self._pursuit_safety_sock = make_udp_listener(PURE_PURSUIT_SAFETY_PORT)
        super().__init__()

    def arbitrator_feed_thread(self):
        sockets = [self._phone_intent_sock, self._pursuit_safety_sock]
        base.base.logger.info(
            "Motor-off arbitrator feed receiver started on UDP %d and %d",
            PHONE_INTENT_PORT,
            PURE_PURSUIT_SAFETY_PORT,
        )
        while self.running:
            try:
                ready, _, _ = select.select(sockets, [], [], 0.1)
                for listener in ready:
                    data, _ = listener.recvfrom(65535)
                    try:
                        payload = json.loads(data.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        payload = None
                    self.arbitrator_feeds.ingest(
                        payload, now_monotonic_s=time.monotonic()
                    )
            except OSError as exc:
                if self.running:
                    base.base.logger.error("Arbitrator feed receiver fault: %s", exc)
        base.base.logger.info("Motor-off arbitrator feed receiver stopped")

    def _teensy_safety_sample(self, now: float) -> TeensySafetySample | None:
        with self._lease_lock:
            status = self._lease_status
            status_at = self._lease_status_at
        if status is None:
            return None
        steer = self.latest_data.get("STEER", {})
        trans = self.latest_data.get("TRANS", {})
        wall_now = time.time()
        steering_healthy = bool(
            wall_now - steer.get("last_update", 0.0) <= 0.5
            and int(steer.get("sf", 1)) == 0
            and int(steer.get("rb", 1)) == 0
        )
        transmission_healthy = bool(
            wall_now - trans.get("last_update", 0.0) <= 0.5
            and int(trans.get("jv", 0)) == 1
            and int(trans.get("eh", 1)) == 0
        )
        return TeensySafetySample(
            received_monotonic_s=status_at,
            lease_status=status,
            # Fail closed: this motor-off bridge does not claim that a local
            # physical Pause or e-stop input is released for motion.
            physical_pause_released=False,
            physical_estop_released=False,
            steering_control_healthy=steering_healthy,
            transmission_control_healthy=transmission_healthy,
        )

    def create_broadcast_message(self):
        message = super().create_broadcast_message()
        now = time.monotonic()
        message["rpi_arbitrator_motor_off"] = self.arbitrator_feeds.snapshot(
            now_monotonic_s=now,
            teensy_sample=self._teensy_safety_sample(now),
        )
        return message

    def run(self):
        threading.Thread(target=self.arbitrator_feed_thread, daemon=True).start()
        try:
            super().run()
        finally:
            self._phone_intent_sock.close()
            self._pursuit_safety_sock.close()


def main() -> None:
    try:
        bridge = TeensySerialBridge()
        bridge.run()
    except Exception as exc:
        base.base.logger.error(
            "Fatal motor-off arbitrator consumer error: %s", exc, exc_info=True
        )


if __name__ == "__main__":
    main()
