#!/usr/bin/env python3
"""Strict versioned wire schemas for RPi arbitrator producers.

The caller supplies the local monotonic receive time. Producer timestamps are
diagnostic only and never participate in freshness decisions.
"""

from __future__ import annotations

from typing import Any
import math

from tractor_rpi.rpi_arbitrator_inputs_20261031 import (
    PhoneIntentSample,
    PurePursuitCommandSample,
    PurePursuitHealthSample,
)
from tractor_rpi.rpi_command_lease_protocol_20261031 import RpiCommandMode


PHONE_INTENT_PORT = 6014
PURE_PURSUIT_SAFETY_PORT = 6015
WIRE_SCHEMA = 1


def _object(payload: Any, kind: str) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError(f"{kind} payload must be a JSON object")
    if payload.get("schema") != WIRE_SCHEMA or payload.get("kind") != kind:
        raise ValueError(f"unsupported {kind} schema")
    return payload


def _finite_float(payload: dict[str, Any], name: str) -> float:
    try:
        value = float(payload[name])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _integer(payload: dict[str, Any], name: str) -> int:
    value = payload.get(name)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _boolean(payload: dict[str, Any], name: str) -> bool:
    value = payload.get(name)
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be a boolean")
    return value


def parse_phone_intent(
    payload: Any, *, received_monotonic_s: float
) -> PhoneIntentSample:
    data = _object(payload, "phone_intent")
    modes = {
        "pause": RpiCommandMode.PAUSE,
        "manual": RpiCommandMode.MANUAL_PERCENT,
        "auto": RpiCommandMode.AUTO_MPS,
    }
    try:
        mode = modes[str(data["requested_mode"]).lower()]
    except (KeyError, ValueError) as exc:
        raise ValueError("requested_mode must be pause, manual, or auto") from exc
    session = str(data.get("producer_session", ""))
    if not session or len(session) > 100:
        raise ValueError("producer_session is required")
    return PhoneIntentSample(
        received_monotonic_s=received_monotonic_s,
        requested_mode=mode,
        session=session,
        sequence=_integer(data, "intent_sequence"),
        manual_drive_percent=_finite_float(data, "manual_drive_percent"),
        manual_steering=_finite_float(data, "manual_steering"),
        operator_link_fresh=_boolean(data, "operator_link_fresh"),
    )


def parse_pure_pursuit_safety(
    payload: Any, *, received_monotonic_s: float
) -> tuple[PurePursuitCommandSample, PurePursuitHealthSample]:
    data = _object(payload, "pure_pursuit_safety")
    route_hash = str(data.get("route_sha256", "")).lower()
    if len(route_hash) != 64 or any(c not in "0123456789abcdef" for c in route_hash):
        raise ValueError("route_sha256 must be 64 hexadecimal characters")
    sequence = _integer(data, "safety_sequence")
    state = str(data.get("controller_state", ""))
    if state not in {"RUNNING", "PAUSED", "WAITING", "WAITING_REACQUIRE", "WAITING_OPERATOR_CYCLE"}:
        raise ValueError("invalid controller_state")
    speed = _finite_float(data, "speed_cmd_mps")
    steering = _finite_float(data, "steer_normalized")
    command = PurePursuitCommandSample(
        received_monotonic_s, sequence, route_hash, speed, steering
    )
    health = PurePursuitHealthSample(
        received_monotonic_s=received_monotonic_s,
        route_sha256=route_hash,
        controller_state=state,
        gps_age_s=_finite_float(data, "gps_age_s"),
        fix_quality=str(data.get("fix_quality", "")),
        heading_valid=_boolean(data, "head_valid"),
        heading_carrier=str(data.get("heading_carrier", "")),
        heading_accuracy_deg=_finite_float(data, "heading_accuracy_deg"),
        route_progress_m=_finite_float(data, "path_progress_m"),
        cross_track_error_m=_finite_float(data, "cross_track_err_m"),
    )
    return command, health
