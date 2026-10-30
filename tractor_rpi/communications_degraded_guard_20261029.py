#!/usr/bin/env python3
"""Bounded supervisory guard for a future COMMUNICATIONS DEGRADED state.

This module does not send motion commands and does not change the 500 ms
Pi-to-Teensy or Pure Pursuit command watchdogs.  It only decides whether an
already-running AUTO mission is inside a pre-authorized time-and-distance
envelope while every required local safety input remains healthy.

The incident budget is deliberately sticky.  Brief phone reconnects do not
reset it.  Reset requires an explicit operator acknowledgement while stopped
in Pause, preventing reconnect churn from extending degraded travel forever.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class GuardDecision(str, Enum):
    NORMAL = "NORMAL"
    COMMUNICATIONS_DEGRADED = "COMMUNICATIONS_DEGRADED"
    PAUSE_REQUIRED = "PAUSE_REQUIRED"


@dataclass(frozen=True)
class LocalSafetyHealth:
    physical_pause_released: bool
    physical_estop_released: bool
    teensy_status_fresh: bool
    teensy_command_watchdog_healthy: bool
    pure_pursuit_command_fresh: bool
    controller_telemetry_fresh: bool
    gps_fresh: bool
    rtk_fixed: bool
    heading_valid_fixed: bool
    route_defined: bool
    inside_route_corridor: bool
    speed_within_limit: bool

    def failed_checks(self) -> list[str]:
        return [name for name, healthy in self.__dict__.items() if not healthy]


@dataclass(frozen=True)
class GuardResult:
    decision: GuardDecision
    reason: str
    elapsed_s: float
    distance_m: float
    remaining_s: float
    remaining_m: float
    incident_id: int


class CommunicationsDegradedGuard:
    """Track one bounded incident until a stopped operator reset occurs."""

    def __init__(self, max_seconds: float = 30.0, max_distance_m: float = 30.0):
        if not 0 < max_seconds <= 30.0:
            raise ValueError("max_seconds must be in (0, 30]")
        if not 0 < max_distance_m <= 30.0:
            raise ValueError("max_distance_m must be in (0, 30]")
        self.max_seconds = float(max_seconds)
        self.max_distance_m = float(max_distance_m)
        self._incident_started_at: float | None = None
        self._incident_start_progress_m: float | None = None
        self._incident_start_local_travel_m: float | None = None
        self._incident_id = 0
        self._latched_pause_reason: str | None = None

    @property
    def incident_active(self) -> bool:
        return self._incident_started_at is not None

    def _result(
        self,
        decision: GuardDecision,
        reason: str,
        now: float,
        route_progress_m: float,
        local_travel_m: float,
    ) -> GuardResult:
        if self._incident_started_at is None:
            elapsed = distance = 0.0
        else:
            elapsed = max(0.0, now - self._incident_started_at)
            distance = max(
                0.0,
                route_progress_m - (self._incident_start_progress_m or 0.0),
                local_travel_m - (self._incident_start_local_travel_m or 0.0),
            )
        return GuardResult(
            decision=decision,
            reason=reason,
            elapsed_s=elapsed,
            distance_m=distance,
            remaining_s=max(0.0, self.max_seconds - elapsed),
            remaining_m=max(0.0, self.max_distance_m - distance),
            incident_id=self._incident_id,
        )

    def evaluate(
        self,
        *,
        now: float,
        route_progress_m: float,
        local_travel_m: float = 0.0,
        phone_link_fresh: bool,
        auto_running: bool,
        preauthorized: bool,
        health: LocalSafetyHealth,
    ) -> GuardResult:
        if not all(map(math.isfinite, (now, route_progress_m, local_travel_m))):
            self._latched_pause_reason = "non-finite degraded-state input"
        if self._latched_pause_reason:
            return self._result(
                GuardDecision.PAUSE_REQUIRED,
                self._latched_pause_reason,
                now,
                route_progress_m,
                local_travel_m,
            )
        if not auto_running:
            return self._result(
                GuardDecision.NORMAL,
                "AUTO is not running",
                now,
                route_progress_m,
                local_travel_m,
            )
        if phone_link_fresh and not self.incident_active:
            return self._result(
                GuardDecision.NORMAL,
                "phone link is fresh",
                now,
                route_progress_m,
                local_travel_m,
            )
        if not preauthorized:
            self._latched_pause_reason = "communications degraded was not pre-authorized"
        failed = health.failed_checks()
        if failed:
            self._latched_pause_reason = "local safety check failed: " + ", ".join(failed)
        if self._latched_pause_reason:
            return self._result(
                GuardDecision.PAUSE_REQUIRED,
                self._latched_pause_reason,
                now,
                route_progress_m,
                local_travel_m,
            )
        if not self.incident_active:
            self._incident_id += 1
            self._incident_started_at = now
            self._incident_start_progress_m = route_progress_m
            self._incident_start_local_travel_m = local_travel_m
        result = self._result(
            GuardDecision.COMMUNICATIONS_DEGRADED,
            "phone supervisory link is stale; local safety checks remain healthy",
            now,
            route_progress_m,
            local_travel_m,
        )
        if result.elapsed_s >= self.max_seconds:
            self._latched_pause_reason = "communications degraded time envelope exhausted"
        elif result.distance_m >= self.max_distance_m:
            self._latched_pause_reason = "communications degraded distance envelope exhausted"
        if self._latched_pause_reason:
            return self._result(
                GuardDecision.PAUSE_REQUIRED,
                self._latched_pause_reason,
                now,
                route_progress_m,
                local_travel_m,
            )
        if phone_link_fresh:
            return GuardResult(
                **{
                    **result.__dict__,
                    "reason": (
                        "phone link recovered; incident budget remains active until "
                        "operator reset while stopped in Pause"
                    ),
                }
            )
        return result

    def reset_after_stopped_pause(
        self, *, physical_pause_active: bool, stopped: bool, operator_acknowledged: bool
    ) -> None:
        if not (physical_pause_active and stopped and operator_acknowledged):
            raise RuntimeError(
                "reset requires physical Pause, confirmed stop, and operator acknowledgement"
            )
        self._incident_started_at = None
        self._incident_start_progress_m = None
        self._incident_start_local_travel_m = None
        self._latched_pause_reason = None

