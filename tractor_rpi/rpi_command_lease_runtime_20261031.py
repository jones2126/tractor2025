#!/usr/bin/env python3
"""Offline scheduler joining the RPi safety supervisor to the serial adapter.

This class does not open serial, UDP, or service resources. The caller owns
the measured input snapshots, telemetry reception, and serial-like sink.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from tractor_rpi.rpi_command_lease_adapter_20261031 import (
    RpiCommandLeaseAdapter,
    RpiLeaseStatus,
)
from tractor_rpi.rpi_command_lease_protocol_20261031 import RpiCommandMode
from tractor_rpi.rpi_safety_supervisor_20261031 import (
    RpiSafetySupervisor,
    SupervisorDecision,
    SupervisorInputs,
    SupervisorState,
    transmit_decision_once,
)


@dataclass(frozen=True)
class RuntimeResult:
    decision: SupervisorDecision
    frame_sent: bytes | None
    waiting_for_pause_ack: bool
    scheduler_fault_latched: bool


class RpiCommandLeaseRuntime:
    """Rate-limit frames and fail closed on an RPi scheduling stall."""

    def __init__(
        self,
        adapter: RpiCommandLeaseAdapter,
        supervisor: RpiSafetySupervisor | None = None,
        *,
        motion_interval_s: float = 0.1,
        pause_retry_s: float = 0.25,
        maximum_tick_gap_s: float = 0.25,
    ):
        for name, value in (
            ("motion_interval_s", motion_interval_s),
            ("pause_retry_s", pause_retry_s),
            ("maximum_tick_gap_s", maximum_tick_gap_s),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if motion_interval_s >= 0.5 or pause_retry_s >= 0.5:
            raise ValueError("command and Pause retry intervals must stay below 500 ms")
        if maximum_tick_gap_s >= 0.5:
            raise ValueError("maximum tick gap must stay below 500 ms")
        self.adapter = adapter
        self.supervisor = supervisor or RpiSafetySupervisor()
        self.motion_interval_s = motion_interval_s
        self.pause_retry_s = pause_retry_s
        self.maximum_tick_gap_s = maximum_tick_gap_s
        self._last_tick_s: float | None = None
        self._last_send_s: float | None = None
        self._last_effective_mode: RpiCommandMode | None = None
        self._scheduler_fault_reason: str | None = None

    @staticmethod
    def _forced_pause(reason: str) -> SupervisorDecision:
        return SupervisorDecision(
            state=SupervisorState.PAUSE_LATCHED,
            mode=RpiCommandMode.PAUSE,
            drive=0.0,
            steering=0.0,
            reason=reason,
            guard=None,
        )

    def tick(
        self,
        *,
        now_monotonic_s: float,
        inputs: SupervisorInputs,
        status: RpiLeaseStatus | None = None,
    ) -> RuntimeResult:
        if not math.isfinite(now_monotonic_s):
            raise ValueError("now_monotonic_s must be finite")
        if inputs.now_monotonic_s != now_monotonic_s:
            raise ValueError("input and runtime monotonic timestamps must match")
        if self._last_tick_s is not None:
            if now_monotonic_s < self._last_tick_s:
                self._scheduler_fault_reason = "RPi monotonic time moved backwards"
            elif now_monotonic_s - self._last_tick_s > self.maximum_tick_gap_s:
                self._scheduler_fault_reason = (
                    "RPi command scheduler gap exceeded "
                    f"{self.maximum_tick_gap_s:.3f} seconds"
                )
        self._last_tick_s = now_monotonic_s

        if status is not None:
            self.adapter.observe_status(status)

        decision = (
            self._forced_pause(self._scheduler_fault_reason)
            if self._scheduler_fault_reason
            else self.supervisor.evaluate(inputs)
        )

        frame = None
        if decision.mode == RpiCommandMode.PAUSE:
            due = (
                self._last_send_s is None
                or self._last_effective_mode != RpiCommandMode.PAUSE
                or now_monotonic_s - self._last_send_s >= self.pause_retry_s
            )
            if due:
                frame = transmit_decision_once(self.adapter, decision)
        elif not self.adapter.claim_acknowledged:
            due = (
                self._last_send_s is None
                or self._last_effective_mode != RpiCommandMode.PAUSE
                or now_monotonic_s - self._last_send_s >= self.pause_retry_s
            )
            if due:
                frame = self.adapter.send_pause()
        else:
            due = (
                self._last_send_s is None
                or self._last_effective_mode != decision.mode
                or now_monotonic_s - self._last_send_s >= self.motion_interval_s
            )
            if due:
                frame = transmit_decision_once(self.adapter, decision)

        if frame is not None:
            self._last_send_s = now_monotonic_s
            self._last_effective_mode = (
                RpiCommandMode.PAUSE
                if decision.mode == RpiCommandMode.PAUSE
                or not self.adapter.claim_acknowledged
                else decision.mode
            )

        return RuntimeResult(
            decision=decision,
            frame_sent=frame,
            waiting_for_pause_ack=(
                decision.mode != RpiCommandMode.PAUSE
                and not self.adapter.claim_acknowledged
            ),
            scheduler_fault_latched=self._scheduler_fault_reason is not None,
        )

    def reset_after_stopped_pause(
        self,
        *,
        physical_pause_active: bool,
        stopped: bool,
        operator_acknowledged: bool,
    ) -> None:
        self.supervisor.reset_after_stopped_pause(
            physical_pause_active=physical_pause_active,
            stopped=stopped,
            operator_acknowledged=operator_acknowledged,
        )
        self._scheduler_fault_reason = None
        self._last_tick_s = None
        self._last_send_s = None
        self._last_effective_mode = None
