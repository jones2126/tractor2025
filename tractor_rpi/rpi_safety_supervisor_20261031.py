#!/usr/bin/env python3
"""Pure RPi safety arbitrator for phone intent and Pure Pursuit proposals.

This module has no sockets, serial device, service integration, or automatic
startup. It converts already-measured inputs into one safe command decision.
The Teensy adapter remains a separate final transport boundary.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from tractor_rpi.communications_degraded_guard_20261029 import (
    CommunicationsDegradedGuard,
    GuardDecision,
    GuardResult,
    LocalSafetyHealth,
)
from tractor_rpi.rpi_command_lease_adapter_20261031 import RpiCommandLeaseAdapter
from tractor_rpi.rpi_command_lease_protocol_20261031 import (
    MAX_AUTO_SPEED_MPS,
    RpiCommandMode,
)


class SupervisorState(str, Enum):
    PAUSE = "PAUSE"
    MANUAL = "MANUAL"
    AUTO = "AUTO"
    COMMUNICATIONS_DEGRADED = "COMMUNICATIONS_DEGRADED"
    PAUSE_LATCHED = "PAUSE_LATCHED"


@dataclass(frozen=True)
class SupervisorInputs:
    now_monotonic_s: float
    requested_mode: RpiCommandMode
    phone_link_fresh: bool
    communications_degraded_preauthorized: bool
    auto_running: bool
    manual_drive_percent: float
    manual_steering: float
    pure_pursuit_speed_mps: float
    pure_pursuit_steering: float
    route_progress_m: float
    local_travel_m: float
    health: LocalSafetyHealth


@dataclass(frozen=True)
class SupervisorDecision:
    state: SupervisorState
    mode: RpiCommandMode
    drive: float
    steering: float
    reason: str
    guard: GuardResult | None


ALWAYS_REQUIRED_HEALTH = (
    "physical_pause_released",
    "physical_estop_released",
    "teensy_status_fresh",
    "teensy_command_watchdog_healthy",
    "controller_telemetry_fresh",
    "speed_within_limit",
)

AUTO_REQUIRED_HEALTH = ALWAYS_REQUIRED_HEALTH + (
    "pure_pursuit_command_fresh",
    "gps_fresh",
    "rtk_fixed",
    "heading_valid_fixed",
    "route_defined",
    "inside_route_corridor",
)


class RpiSafetySupervisor:
    """Own the sole RPi decision before the generic Teensy adapter."""

    def __init__(self, *, max_seconds: float = 30.0, max_distance_m: float = 30.0):
        self.guard = CommunicationsDegradedGuard(max_seconds, max_distance_m)
        self._pause_latched_reason: str | None = None

    @staticmethod
    def _pause(
        state: SupervisorState,
        reason: str,
        guard: GuardResult | None = None,
    ) -> SupervisorDecision:
        return SupervisorDecision(
            state=state,
            mode=RpiCommandMode.PAUSE,
            drive=0.0,
            steering=0.0,
            reason=reason,
            guard=guard,
        )

    @staticmethod
    def _failed_health(health: LocalSafetyHealth, required: tuple[str, ...]) -> list[str]:
        return [name for name in required if not getattr(health, name)]

    @staticmethod
    def _finite_inputs(inputs: SupervisorInputs) -> bool:
        return all(
            math.isfinite(value)
            for value in (
                inputs.now_monotonic_s,
                inputs.manual_drive_percent,
                inputs.manual_steering,
                inputs.pure_pursuit_speed_mps,
                inputs.pure_pursuit_steering,
                inputs.route_progress_m,
                inputs.local_travel_m,
            )
        )

    def evaluate(self, inputs: SupervisorInputs) -> SupervisorDecision:
        if self._pause_latched_reason:
            return self._pause(
                SupervisorState.PAUSE_LATCHED,
                self._pause_latched_reason,
            )

        if not isinstance(inputs.requested_mode, RpiCommandMode):
            self._pause_latched_reason = "invalid requested mode"
            return self._pause(SupervisorState.PAUSE_LATCHED, self._pause_latched_reason)
        if not self._finite_inputs(inputs):
            self._pause_latched_reason = "non-finite RPi supervisor input"
            return self._pause(SupervisorState.PAUSE_LATCHED, self._pause_latched_reason)

        if inputs.requested_mode == RpiCommandMode.PAUSE:
            return self._pause(SupervisorState.PAUSE, "operator requested Pause")

        required = (
            ALWAYS_REQUIRED_HEALTH
            if inputs.requested_mode == RpiCommandMode.MANUAL_PERCENT
            else AUTO_REQUIRED_HEALTH
        )
        failed = self._failed_health(inputs.health, required)
        if failed:
            self._pause_latched_reason = "local safety check failed: " + ", ".join(failed)
            return self._pause(SupervisorState.PAUSE_LATCHED, self._pause_latched_reason)

        if inputs.requested_mode == RpiCommandMode.MANUAL_PERCENT:
            if not inputs.phone_link_fresh:
                self._pause_latched_reason = "phone link stale in Manual"
                return self._pause(SupervisorState.PAUSE_LATCHED, self._pause_latched_reason)
            if not -100.0 <= inputs.manual_drive_percent <= 100.0:
                self._pause_latched_reason = "Manual drive outside -100..100 percent"
                return self._pause(SupervisorState.PAUSE_LATCHED, self._pause_latched_reason)
            if not -1.0 <= inputs.manual_steering <= 1.0:
                self._pause_latched_reason = "Manual steering outside -1..1"
                return self._pause(SupervisorState.PAUSE_LATCHED, self._pause_latched_reason)
            return SupervisorDecision(
                SupervisorState.MANUAL,
                RpiCommandMode.MANUAL_PERCENT,
                inputs.manual_drive_percent,
                inputs.manual_steering,
                "fresh phone Manual command and local safety checks healthy",
                None,
            )

        if not inputs.auto_running:
            self._pause_latched_reason = "Auto requested while mission is not running"
            return self._pause(SupervisorState.PAUSE_LATCHED, self._pause_latched_reason)
        if not 0.0 <= inputs.pure_pursuit_speed_mps <= MAX_AUTO_SPEED_MPS:
            self._pause_latched_reason = (
                f"Pure Pursuit speed outside 0..{MAX_AUTO_SPEED_MPS} m/s"
            )
            return self._pause(SupervisorState.PAUSE_LATCHED, self._pause_latched_reason)
        if not -1.0 <= inputs.pure_pursuit_steering <= 1.0:
            self._pause_latched_reason = "Pure Pursuit steering outside -1..1"
            return self._pause(SupervisorState.PAUSE_LATCHED, self._pause_latched_reason)

        guard = self.guard.evaluate(
            now=inputs.now_monotonic_s,
            route_progress_m=inputs.route_progress_m,
            local_travel_m=inputs.local_travel_m,
            phone_link_fresh=inputs.phone_link_fresh,
            auto_running=True,
            preauthorized=inputs.communications_degraded_preauthorized,
            health=inputs.health,
        )
        if guard.decision == GuardDecision.PAUSE_REQUIRED:
            return self._pause(SupervisorState.PAUSE_LATCHED, guard.reason, guard)
        state = (
            SupervisorState.COMMUNICATIONS_DEGRADED
            if guard.decision == GuardDecision.COMMUNICATIONS_DEGRADED
            else SupervisorState.AUTO
        )
        return SupervisorDecision(
            state=state,
            mode=RpiCommandMode.AUTO_MPS,
            drive=inputs.pure_pursuit_speed_mps,
            steering=inputs.pure_pursuit_steering,
            reason=guard.reason,
            guard=guard,
        )

    def reset_after_stopped_pause(
        self,
        *,
        physical_pause_active: bool,
        stopped: bool,
        operator_acknowledged: bool,
    ) -> None:
        self.guard.reset_after_stopped_pause(
            physical_pause_active=physical_pause_active,
            stopped=stopped,
            operator_acknowledged=operator_acknowledged,
        )
        self._pause_latched_reason = None


def transmit_decision_once(
    adapter: RpiCommandLeaseAdapter,
    decision: SupervisorDecision,
) -> bytes:
    """Send one decision; cadence and telemetry acknowledgement remain external."""
    if decision.mode == RpiCommandMode.PAUSE:
        return adapter.send_pause()
    if decision.mode == RpiCommandMode.MANUAL_PERCENT:
        return adapter.send_manual(decision.drive, decision.steering)
    if decision.mode == RpiCommandMode.AUTO_MPS:
        return adapter.send_auto(decision.drive, decision.steering)
    raise RuntimeError("unsupported supervisor decision mode")
