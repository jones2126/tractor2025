#!/usr/bin/env python3
"""Motion-capable RPi safety arbitrator and Teensy lease bridge candidate.

The phone supplies operator intent, Pure Pursuit supplies proposals, and this
process is the only component allowed to send movement frames to the Teensy.
Legacy UDP 6004 packets are rejected by the inherited gate. The local 500 ms
Teensy lease and actuator checks remain immediate. The phone E-stop grounds
the ignition through the Teensy relay; the red tractor E-stop and rear
transmission release remain independent hardware barriers.

This executable fails closed unless its exact package environment gates are
present. No installed service references it in this revision.
"""

from __future__ import annotations

from dataclasses import asdict
import os
import threading
import time
from typing import Any

from tractor_rpi.rpi_arbitrator_inputs_20261031 import RpiArbitratorInputCollector
from tractor_rpi.rpi_command_lease_protocol_20261031 import RpiCommandMode
from tractor_rpi.rpi_command_lease_runtime_20261031 import RpiCommandLeaseRuntime
from tractor_rpi.rpi_safety_supervisor_20261031 import (
    RpiSafetySupervisor,
    SupervisorState,
)
from tractor_rpi.teensy_serial_bridge_rpi_arbitrator_motor_off_20261031 import (
    ArbitratorFeedReceiver,
    TeensySerialBridge as MotorOffBridge,
    validated_route_hash,
)
from tractor_rpi.teensy_serial_bridge_rpi_lease_pause_20261031 import (
    TeensySerialBridge as PauseOnlyBridge,
    base,
)


EXPECTED_FIRMWARE = "teensy_main_20261104_rpi_command_lease_v3"
MOTION_GATE = "RUN_DIRECT_OUTER_PERIMETER_BLADES_OFF"


def stopped_phone_pause_reset_ready(inputs, status, sample, latest_data) -> bool:
    """Allow latch recovery only from a fresh, healthy, zero-output Pause."""
    if (
        inputs.requested_mode != RpiCommandMode.PAUSE
        or not inputs.phone_link_fresh
        or status is None
        or sample is None
    ):
        return False
    if (
        status.mode != RpiCommandMode.PAUSE
        or not status.arrival_fresh
        or not status.lease_usable
        or status.expiry_latched
        or status.estop_latched
        or not sample.steering_control_healthy
        or not sample.transmission_control_healthy
    ):
        return False
    steer = latest_data.get("STEER", {})
    trans = latest_data.get("TRANS", {})
    try:
        return float(steer.get("p", 0.0)) == 0.0 and float(
            trans.get("dc", 0.0)
        ) == 0.0
    except (TypeError, ValueError):
        return False


def require_production_environment() -> tuple[str, bool]:
    route_hash = validated_route_hash(
        os.environ.get("TRACTOR_EXPECTED_ROUTE_SHA256", "")
    )
    if os.environ.get("TRACTOR_MOTION_AUTHORIZATION") != MOTION_GATE:
        raise RuntimeError("motion authorization environment gate is closed")
    degraded = os.environ.get("TRACTOR_COMMUNICATIONS_DEGRADED") == "30S_30M"
    return route_hash, degraded


class ProductionFeedReceiver(ArbitratorFeedReceiver):
    def __init__(self, expected_route_sha256: str, *, degraded: bool):
        super().__init__(expected_route_sha256)
        self.collector = RpiArbitratorInputCollector(
            expected_route_sha256=expected_route_sha256,
            degraded_preauthorized=degraded,
            phone_replaces_nrf_mode_switch=True,
        )
        self.supervisor = RpiSafetySupervisor()


class TeensySerialBridge(MotorOffBridge):
    def __init__(self):
        route_hash, degraded = require_production_environment()
        super().__init__()
        self.arbitrator_feeds = ProductionFeedReceiver(route_hash, degraded=degraded)
        self.runtime = RpiCommandLeaseRuntime(
            self.adapter,
            supervisor=self.arbitrator_feeds.supervisor,
        )
        self.degraded_preauthorized = degraded
        self._decision_lock = threading.Lock()
        self._latest_result = None
        self._pause_reset_armed = True

    def command_lease_thread(self):
        base.base.logger.info("RPi safety-arbitrator scheduler started")
        while self.running:
            now = time.monotonic()
            with self._lease_lock:
                status = self._lease_status
            try:
                sample = self._teensy_safety_sample(now)
                with self.arbitrator_feeds._lock:
                    if sample is not None:
                        self.arbitrator_feeds.collector.ingest_teensy(sample)
                    inputs = self.arbitrator_feeds.collector.build(
                        now_monotonic_s=now
                    )
                    if inputs.requested_mode != RpiCommandMode.PAUSE:
                        self._pause_reset_armed = True
                    if self._pause_reset_armed and stopped_phone_pause_reset_ready(
                        inputs, status, sample, self.latest_data
                    ):
                        self.runtime.reset_after_stopped_pause(
                            physical_pause_active=True,
                            stopped=True,
                            operator_acknowledged=True,
                        )
                        self._pause_reset_armed = False
                        base.base.logger.info(
                            "Guarded stopped-Pause latch reset completed"
                        )
                    result = self.runtime.tick(
                        now_monotonic_s=now,
                        inputs=inputs,
                        status=status,
                    )
                    if result.decision.state == SupervisorState.PAUSE_LATCHED:
                        self._pause_reset_armed = True
                with self._decision_lock:
                    self._latest_result = result
                if result.frame_sent is not None:
                    self.stats["commands_sent"] += 1
                    self.cmd_vel_sent_count += 1
            except Exception as exc:
                base.base.logger.error(
                    "Arbitrator scheduler fault: %s", exc, exc_info=True
                )
                self.running = False
                break
            time.sleep(0.05)

    def _arbitrator_snapshot(self, now: float) -> dict[str, Any]:
        with self.arbitrator_feeds._lock:
            inputs = self.arbitrator_feeds.collector.build(now_monotonic_s=now)
            input_fault = self.arbitrator_feeds.collector.input_fault
        with self._decision_lock:
            result = self._latest_result
        if result is None:
            return {
                "schema": 2,
                "motion_capable": True,
                "actual_serial_output": "PAUSE_STARTUP",
                "expected_firmware": EXPECTED_FIRMWARE,
                "communications_degraded_preauthorized": self.degraded_preauthorized,
                "exact_reason": "waiting for first arbitrator decision",
            }
        decision = result.decision
        return {
            "schema": 2,
            "motion_capable": True,
            "actual_serial_output": decision.mode.name,
            "expected_firmware": EXPECTED_FIRMWARE,
            "accepted_phone_samples": self.arbitrator_feeds.accepted_phone,
            "accepted_pure_pursuit_samples": self.arbitrator_feeds.accepted_pursuit,
            "rejected_samples": self.arbitrator_feeds.rejected,
            "last_rejection_reason": self.arbitrator_feeds.last_rejection_reason,
            "requested_mode": inputs.requested_mode.name,
            "phone_link_fresh": inputs.phone_link_fresh,
            "communications_degraded_preauthorized": self.degraded_preauthorized,
            "joined_decision": decision.state.value,
            "joined_mode": int(decision.mode),
            "joined_drive": decision.drive,
            "joined_steering": decision.steering,
            "exact_reason": decision.reason,
            "guard": None if decision.guard is None else asdict(decision.guard),
            "health": asdict(inputs.health),
            "additional_local_faults": list(inputs.additional_local_faults),
            "input_fault": input_fault,
            "route_progress_m": inputs.route_progress_m,
            "distance_envelope_source": "ROUTE_PROGRESS_ONLY",
            "waiting_for_pause_ack": result.waiting_for_pause_ack,
            "scheduler_fault_latched": result.scheduler_fault_latched,
        }

    def create_broadcast_message(self):
        message = PauseOnlyBridge.create_broadcast_message(self)
        message["rpi_arbitrator"] = self._arbitrator_snapshot(time.monotonic())
        lease = message.setdefault("rpi_command_lease", {})
        lease["expected_firmware"] = EXPECTED_FIRMWARE
        lease["pause_only"] = False
        return message


def main() -> None:
    try:
        bridge = TeensySerialBridge()
        bridge.run()
    except Exception as exc:
        base.base.logger.error(
            "Fatal production arbitrator bridge error: %s", exc, exc_info=True
        )


if __name__ == "__main__":
    main()
