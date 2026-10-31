#!/usr/bin/env python3
"""Measured-input boundary for the full RPi command arbitrator.

All freshness is based on local monotonic arrival time.  No phone-to-RPi
wall-clock subtraction is used.  The collector retains the last explicit
operator intent through an AUTO phone outage, but every other required local
input must remain fresh and healthy.  It emits no serial or UDP commands.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from tractor_rpi.communications_degraded_guard_20261029 import LocalSafetyHealth
from tractor_rpi.rpi_command_lease_adapter_20261031 import RpiLeaseStatus
from tractor_rpi.rpi_command_lease_protocol_20261031 import (
    MAX_AUTO_SPEED_MPS,
    RpiCommandMode,
)
from tractor_rpi.rpi_safety_supervisor_20261031 import SupervisorInputs


@dataclass(frozen=True)
class ArbitratorThresholds:
    phone_fresh_s: float = 0.5
    pure_pursuit_command_fresh_s: float = 0.5
    pure_pursuit_health_fresh_s: float = 0.5
    teensy_status_fresh_s: float = 0.5
    gps_max_age_s: float = 1.0
    heading_accuracy_max_deg: float = 1.0
    cross_track_max_m: float = 2.0
    auto_speed_max_mps: float = MAX_AUTO_SPEED_MPS

    def __post_init__(self) -> None:
        values = self.__dict__.values()
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("all arbitrator thresholds must be finite and positive")
        for name in (
            "phone_fresh_s",
            "pure_pursuit_command_fresh_s",
            "pure_pursuit_health_fresh_s",
            "teensy_status_fresh_s",
        ):
            if getattr(self, name) > 0.5:
                raise ValueError(f"{name} cannot exceed the 500 ms local lease")
        if self.auto_speed_max_mps > MAX_AUTO_SPEED_MPS:
            raise ValueError("AUTO speed threshold exceeds the firmware limit")


@dataclass(frozen=True)
class PhoneIntentSample:
    received_monotonic_s: float
    requested_mode: RpiCommandMode
    session: str
    sequence: int
    manual_drive_percent: float = 0.0
    manual_steering: float = 0.0


@dataclass(frozen=True)
class PurePursuitCommandSample:
    received_monotonic_s: float
    sequence: int
    route_sha256: str
    speed_mps: float
    steering: float


@dataclass(frozen=True)
class PurePursuitHealthSample:
    received_monotonic_s: float
    route_sha256: str
    controller_state: str
    gps_age_s: float
    fix_quality: str
    heading_valid: bool
    heading_carrier: str
    heading_accuracy_deg: float
    route_progress_m: float
    cross_track_error_m: float


@dataclass(frozen=True)
class TeensySafetySample:
    received_monotonic_s: float
    lease_status: RpiLeaseStatus
    physical_pause_released: bool
    physical_estop_released: bool
    steering_control_healthy: bool
    transmission_control_healthy: bool


class RpiArbitratorInputCollector:
    """Join explicit samples and derive one fail-closed supervisor snapshot."""

    def __init__(
        self,
        *,
        expected_route_sha256: str,
        degraded_preauthorized: bool,
        thresholds: ArbitratorThresholds | None = None,
    ):
        route_hash = expected_route_sha256.strip().lower()
        if len(route_hash) != 64 or any(c not in "0123456789abcdef" for c in route_hash):
            raise ValueError("expected route SHA-256 must be 64 lowercase hex characters")
        self.expected_route_sha256 = route_hash
        self.degraded_preauthorized = bool(degraded_preauthorized)
        self.thresholds = thresholds or ArbitratorThresholds()
        self.phone: PhoneIntentSample | None = None
        self.pure_pursuit_command: PurePursuitCommandSample | None = None
        self.pure_pursuit_health: PurePursuitHealthSample | None = None
        self.teensy: TeensySafetySample | None = None
        self.local_travel_m = 0.0
        self._last_pp_command_at: float | None = None
        self._last_pp_speed_mps = 0.0
        self._last_phone_key: tuple[str, int] | None = None
        self._last_pp_sequence: int | None = None
        self._latched_input_fault: str | None = None

    @staticmethod
    def _finite(*values: float) -> bool:
        return all(math.isfinite(value) for value in values)

    def ingest_phone(self, sample: PhoneIntentSample) -> None:
        if not self._finite(
            sample.received_monotonic_s,
            sample.manual_drive_percent,
            sample.manual_steering,
        ):
            self._latched_input_fault = "non-finite phone intent"
            return
        key = (sample.session, sample.sequence)
        if self._last_phone_key is not None:
            last_session, last_sequence = self._last_phone_key
            if sample.session == last_session and sample.sequence <= last_sequence:
                self._latched_input_fault = "stale or duplicate phone intent sequence"
                return
        self.phone = sample
        self._last_phone_key = key

    def ingest_pure_pursuit_command(self, sample: PurePursuitCommandSample) -> None:
        if not self._finite(sample.received_monotonic_s, sample.speed_mps, sample.steering):
            self._latched_input_fault = "non-finite Pure Pursuit command"
            return
        if self._last_pp_sequence is not None and sample.sequence <= self._last_pp_sequence:
            self._latched_input_fault = "stale or duplicate Pure Pursuit sequence"
            return
        if self._last_pp_command_at is not None:
            elapsed = sample.received_monotonic_s - self._last_pp_command_at
            if elapsed < 0:
                self._latched_input_fault = "Pure Pursuit monotonic arrival moved backwards"
                return
            # Command-integrated travel is deliberately conservative and is
            # compared with route progress by the sticky degraded guard.
            self.local_travel_m += abs(self._last_pp_speed_mps) * elapsed
        self.pure_pursuit_command = sample
        self._last_pp_command_at = sample.received_monotonic_s
        self._last_pp_speed_mps = sample.speed_mps
        self._last_pp_sequence = sample.sequence

    def ingest_pure_pursuit_health(self, sample: PurePursuitHealthSample) -> None:
        if not self._finite(
            sample.received_monotonic_s,
            sample.gps_age_s,
            sample.heading_accuracy_deg,
            sample.route_progress_m,
            sample.cross_track_error_m,
        ):
            self._latched_input_fault = "non-finite Pure Pursuit health"
            return
        if (
            self.pure_pursuit_health is not None
            and sample.route_progress_m + 0.05
            < self.pure_pursuit_health.route_progress_m
        ):
            self._latched_input_fault = "route progress moved backwards"
            return
        self.pure_pursuit_health = sample

    def ingest_teensy(self, sample: TeensySafetySample) -> None:
        if not math.isfinite(sample.received_monotonic_s):
            self._latched_input_fault = "non-finite Teensy arrival time"
            return
        self.teensy = sample

    @staticmethod
    def _fresh(now: float, received: float | None, limit: float) -> bool:
        return received is not None and 0.0 <= now - received <= limit

    def build(self, *, now_monotonic_s: float) -> SupervisorInputs:
        if not math.isfinite(now_monotonic_s):
            raise ValueError("now_monotonic_s must be finite")
        t = self.thresholds
        phone = self.phone
        command = self.pure_pursuit_command
        pp = self.pure_pursuit_health
        teensy = self.teensy

        phone_fresh = self._fresh(
            now_monotonic_s,
            None if phone is None else phone.received_monotonic_s,
            t.phone_fresh_s,
        )
        pp_command_fresh = self._fresh(
            now_monotonic_s,
            None if command is None else command.received_monotonic_s,
            t.pure_pursuit_command_fresh_s,
        )
        pp_health_fresh = self._fresh(
            now_monotonic_s,
            None if pp is None else pp.received_monotonic_s,
            t.pure_pursuit_health_fresh_s,
        )
        teensy_fresh = self._fresh(
            now_monotonic_s,
            None if teensy is None else teensy.received_monotonic_s,
            t.teensy_status_fresh_s,
        )

        route_defined = bool(
            command
            and pp
            and command.route_sha256.lower() == self.expected_route_sha256
            and pp.route_sha256.lower() == self.expected_route_sha256
        )
        health = LocalSafetyHealth(
            physical_pause_released=bool(teensy and teensy.physical_pause_released),
            physical_estop_released=bool(teensy and teensy.physical_estop_released),
            teensy_status_fresh=teensy_fresh,
            teensy_command_watchdog_healthy=bool(
                teensy_fresh
                and teensy
                and teensy.lease_status.arrival_fresh
                and teensy.lease_status.lease_usable
                and not teensy.lease_status.expiry_latched
            ),
            pure_pursuit_command_fresh=pp_command_fresh,
            controller_telemetry_fresh=pp_health_fresh,
            gps_fresh=bool(pp_health_fresh and pp and 0 <= pp.gps_age_s <= t.gps_max_age_s),
            rtk_fixed=bool(pp and pp.fix_quality.strip().lower() == "rtk fixed"),
            heading_valid_fixed=bool(
                pp
                and pp.heading_valid
                and pp.heading_carrier.strip().lower() == "fixed"
                and 0 <= pp.heading_accuracy_deg <= t.heading_accuracy_max_deg
            ),
            route_defined=route_defined,
            inside_route_corridor=bool(
                pp and 0 <= pp.cross_track_error_m <= t.cross_track_max_m
            ),
            speed_within_limit=bool(
                command and 0 <= command.speed_mps <= t.auto_speed_max_mps
            ),
        )
        additional_faults = []
        if self._latched_input_fault:
            additional_faults.append(self._latched_input_fault)
        if teensy is None or not teensy.steering_control_healthy:
            additional_faults.append("steering_control_healthy")
        if teensy is None or not teensy.transmission_control_healthy:
            additional_faults.append("transmission_control_healthy")

        requested_mode = phone.requested_mode if phone else RpiCommandMode.PAUSE
        return SupervisorInputs(
            now_monotonic_s=now_monotonic_s,
            requested_mode=requested_mode,
            phone_link_fresh=phone_fresh,
            communications_degraded_preauthorized=self.degraded_preauthorized,
            auto_running=bool(pp and pp.controller_state == "RUNNING"),
            manual_drive_percent=0.0 if phone is None else phone.manual_drive_percent,
            manual_steering=0.0 if phone is None else phone.manual_steering,
            pure_pursuit_speed_mps=0.0 if command is None else command.speed_mps,
            pure_pursuit_steering=0.0 if command is None else command.steering,
            route_progress_m=0.0 if pp is None else pp.route_progress_m,
            local_travel_m=self.local_travel_m,
            health=health,
            additional_local_faults=tuple(additional_faults),
        )

    @property
    def input_fault(self) -> str | None:
        return self._latched_input_fault
