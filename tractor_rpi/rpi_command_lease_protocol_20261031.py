#!/usr/bin/env python3
"""Offline reference model for a generic RPi-to-Teensy command lease.

This module has no I/O and is not imported by a launcher or service.  It makes
the proposed serial contract executable in tests before any firmware is
written or flashed.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math


MAX_SEQUENCE = (1 << 32) - 1
MAX_LEASE_MS = 500
MAX_AUTO_SPEED_MPS = 1.8


class RpiCommandMode(IntEnum):
    PAUSE = 0
    MANUAL_PERCENT = 1
    AUTO_MPS = 2


@dataclass(frozen=True)
class RpiCommand:
    sequence: int
    mode: RpiCommandMode
    drive: float
    steering: float


@dataclass(frozen=True)
class LeaseDecision:
    command: RpiCommand
    lease_fresh: bool
    command_age_ms: float
    reason: str


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE used by the proposed ASCII serial frame."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _validate(command: RpiCommand) -> None:
    if not isinstance(command.sequence, int) or not 0 <= command.sequence <= MAX_SEQUENCE:
        raise ValueError("sequence must be an unsigned 32-bit integer")
    if not isinstance(command.mode, RpiCommandMode):
        raise ValueError("mode must be an RpiCommandMode")
    if not math.isfinite(command.drive) or not math.isfinite(command.steering):
        raise ValueError("drive and steering must be finite")
    if not -1.0 <= command.steering <= 1.0:
        raise ValueError("steering must be within -1..1")
    if command.mode == RpiCommandMode.PAUSE:
        if command.drive != 0.0 or command.steering != 0.0:
            raise ValueError("Pause must carry zero drive and zero steering")
    elif command.mode == RpiCommandMode.MANUAL_PERCENT:
        if not -100.0 <= command.drive <= 100.0:
            raise ValueError("Manual drive must be within -100..100 percent")
    elif not 0.0 <= command.drive <= MAX_AUTO_SPEED_MPS:
        raise ValueError(f"Auto drive must be within 0..{MAX_AUTO_SPEED_MPS} m/s")


def encode_frame(command: RpiCommand) -> bytes:
    _validate(command)
    body = (
        f"RPICMD,{command.sequence},{int(command.mode)},"
        f"{command.drive:.4f},{command.steering:.4f}"
    ).encode("ascii")
    return body + f"*{crc16_ccitt(body):04X}\n".encode("ascii")


def decode_frame(frame: bytes | str) -> RpiCommand:
    if isinstance(frame, bytes):
        try:
            text = frame.decode("ascii")
        except UnicodeDecodeError as exc:
            raise ValueError("frame must be ASCII") from exc
    else:
        text = frame
    if "\x00" in text or len(text) > 96:
        raise ValueError("frame contains invalid bytes or is too long")
    text = text.rstrip("\r\n")
    if text.count("*") != 1:
        raise ValueError("frame must contain one checksum separator")
    body_text, checksum_text = text.split("*", 1)
    if len(checksum_text) != 4:
        raise ValueError("checksum must contain four hex digits")
    try:
        supplied_checksum = int(checksum_text, 16)
    except ValueError as exc:
        raise ValueError("checksum is not hexadecimal") from exc
    body = body_text.encode("ascii")
    if crc16_ccitt(body) != supplied_checksum:
        raise ValueError("checksum mismatch")
    fields = body_text.split(",")
    if len(fields) != 5 or fields[0] != "RPICMD":
        raise ValueError("invalid RPICMD field layout")
    try:
        command = RpiCommand(
            sequence=int(fields[1], 10),
            mode=RpiCommandMode(int(fields[2], 10)),
            drive=float(fields[3]),
            steering=float(fields[4]),
        )
    except (TypeError, ValueError) as exc:
        raise ValueError("invalid RPICMD field value") from exc
    _validate(command)
    return command


def sequence_is_newer(candidate: int, previous: int) -> bool:
    """Return true for a forward uint32 sequence, including wraparound."""
    if not 0 <= candidate <= MAX_SEQUENCE or not 0 <= previous <= MAX_SEQUENCE:
        raise ValueError("sequence values must be unsigned 32-bit integers")
    difference = (candidate - previous) & MAX_SEQUENCE
    return 0 < difference < (1 << 31)


class CommandLeaseReference:
    """Reference behavior that future Teensy firmware must match."""

    def __init__(self, timeout_ms: int = MAX_LEASE_MS):
        if not isinstance(timeout_ms, int) or not 0 < timeout_ms <= MAX_LEASE_MS:
            raise ValueError(f"timeout_ms must be an integer in 1..{MAX_LEASE_MS}")
        self.timeout_ms = timeout_ms
        self._last_command: RpiCommand | None = None
        self._last_accepted_ms: float | None = None

    def accept(self, frame: bytes | str, *, now_ms: float) -> bool:
        if not math.isfinite(now_ms):
            return False
        try:
            command = decode_frame(frame)
        except (TypeError, ValueError):
            return False
        if self._last_command is not None and not sequence_is_newer(
            command.sequence, self._last_command.sequence
        ):
            return False
        self._last_command = command
        self._last_accepted_ms = now_ms
        return True

    @staticmethod
    def _pause(sequence: int = 0) -> RpiCommand:
        return RpiCommand(sequence, RpiCommandMode.PAUSE, 0.0, 0.0)

    def decide(
        self,
        *,
        now_ms: float,
        physical_pause_active: bool = False,
        physical_estop_active: bool = False,
        steering_fault_latched: bool = False,
        transmission_fault_latched: bool = False,
    ) -> LeaseDecision:
        sequence = self._last_command.sequence if self._last_command else 0
        if physical_estop_active:
            return LeaseDecision(self._pause(sequence), False, math.inf, "physical e-stop active")
        if physical_pause_active:
            return LeaseDecision(self._pause(sequence), False, math.inf, "physical Pause active")
        if steering_fault_latched:
            return LeaseDecision(self._pause(sequence), False, math.inf, "steering fault latched")
        if transmission_fault_latched:
            return LeaseDecision(self._pause(sequence), False, math.inf, "transmission fault latched")
        if self._last_command is None or self._last_accepted_ms is None:
            return LeaseDecision(self._pause(), False, math.inf, "no accepted RPi command")
        age_ms = max(0.0, now_ms - self._last_accepted_ms)
        if not math.isfinite(now_ms) or now_ms < self._last_accepted_ms:
            return LeaseDecision(self._pause(sequence), False, math.inf, "invalid monotonic time")
        if age_ms > self.timeout_ms:
            return LeaseDecision(self._pause(sequence), False, age_ms, "RPi command lease expired")
        return LeaseDecision(self._last_command, True, age_ms, "RPi command lease fresh")
