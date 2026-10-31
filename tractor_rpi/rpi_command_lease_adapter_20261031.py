#!/usr/bin/env python3
"""Opt-in RPi serial adapter for the generic Teensy command lease.

The adapter has no sockets, service integration, or automatic startup.  A
caller supplies the already-arbitrated safe command and a serial-like sink.
New sessions begin in Pause and motion remains blocked until matching Teensy
RPI_CTL telemetry acknowledges that Pause claim.
"""

from __future__ import annotations

from dataclasses import dataclass
import secrets
import threading
from typing import Protocol

from tractor_rpi.rpi_command_lease_protocol_20261031 import (
    MAX_SEQUENCE,
    RpiCommand,
    RpiCommandMode,
    encode_frame,
)


class SerialSink(Protocol):
    def write(self, data: bytes) -> int | None: ...
    def flush(self) -> None: ...


@dataclass(frozen=True)
class RpiLeaseStatus:
    source_time_ms: int
    mode: RpiCommandMode
    arrival_fresh: bool
    lease_usable: bool
    expiry_latched: bool
    command_age_ms: int
    session: int
    sequence: int
    accepted_count: int
    rejected_count: int
    reject_reason: int


def parse_rpi_ctl_line(line: str) -> RpiLeaseStatus:
    fields = line.strip().split(",")
    if len(fields) < 4 or fields[2] != "RPI_CTL":
        raise ValueError("not an RPI_CTL telemetry line")
    try:
        source_time_ms = int(fields[1], 10)
        values = dict(field.split("=", 1) for field in fields[3:])
        required = {"m", "f", "u", "l", "a", "ss", "s", "ok", "rj", "rr"}
        if set(values) != required:
            raise ValueError("RPI_CTL fields do not match the expected schema")
        status = RpiLeaseStatus(
            source_time_ms=source_time_ms,
            mode=RpiCommandMode(int(values["m"], 10)),
            arrival_fresh=bool(int(values["f"], 10)),
            lease_usable=bool(int(values["u"], 10)),
            expiry_latched=bool(int(values["l"], 10)),
            command_age_ms=int(values["a"], 10),
            session=int(values["ss"], 10),
            sequence=int(values["s"], 10),
            accepted_count=int(values["ok"], 10),
            rejected_count=int(values["rj"], 10),
            reject_reason=int(values["rr"], 10),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("invalid RPI_CTL telemetry value") from exc
    for boolean_value in (values["f"], values["u"], values["l"]):
        if boolean_value not in {"0", "1"}:
            raise ValueError("RPI_CTL boolean fields must be 0 or 1")
    for value in (
        status.source_time_ms,
        status.command_age_ms,
        status.session,
        status.sequence,
        status.accepted_count,
        status.rejected_count,
        status.reject_reason,
    ):
        if value < 0:
            raise ValueError("RPI_CTL counters must be non-negative")
    if status.session > MAX_SEQUENCE or status.sequence > MAX_SEQUENCE:
        raise ValueError("RPI_CTL session and sequence must be uint32")
    return status


class RpiCommandLeaseAdapter:
    """Serialize one safe command stream and require a Pause session claim."""

    def __init__(self, sink: SerialSink, *, session: int | None = None):
        self.sink = sink
        self.session = secrets.randbits(32) if session is None else session
        if not isinstance(self.session, int) or not 0 <= self.session <= MAX_SEQUENCE:
            raise ValueError("session must be an unsigned 32-bit integer")
        self._sequence = MAX_SEQUENCE
        self._claim_acknowledged = False
        self._last_pause_sequence: int | None = None
        self._lock = threading.Lock()
        self.frames_sent = 0

    @property
    def claim_acknowledged(self) -> bool:
        return self._claim_acknowledged

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) & MAX_SEQUENCE
        return self._sequence

    def _write(self, command: RpiCommand) -> bytes:
        frame = encode_frame(command)
        written = self.sink.write(frame)
        if written is not None and written != len(frame):
            raise OSError(f"short serial write: {written} of {len(frame)} bytes")
        self.sink.flush()
        self.frames_sent += 1
        return frame

    def send_pause(self) -> bytes:
        with self._lock:
            # Every Pause is a re-arm boundary. Motion stays blocked until the
            # Teensy reports this exact Pause sequence as fresh and usable.
            self._claim_acknowledged = False
            command = RpiCommand(
                self.session,
                self._next_sequence(),
                RpiCommandMode.PAUSE,
                0.0,
                0.0,
            )
            self._last_pause_sequence = command.sequence
            return self._write(command)

    def send_manual(self, drive_percent: float, steering: float) -> bytes:
        return self._send_motion(
            RpiCommandMode.MANUAL_PERCENT, drive_percent, steering
        )

    def send_auto(self, speed_mps: float, steering: float) -> bytes:
        return self._send_motion(RpiCommandMode.AUTO_MPS, speed_mps, steering)

    def _send_motion(
        self, mode: RpiCommandMode, drive: float, steering: float
    ) -> bytes:
        with self._lock:
            if not self._claim_acknowledged:
                raise RuntimeError(
                    "motion blocked until matching Teensy Pause claim is acknowledged"
                )
            return self._write(
                RpiCommand(
                    self.session,
                    self._next_sequence(),
                    mode,
                    drive,
                    steering,
                )
            )

    def observe_status(self, status: RpiLeaseStatus) -> bool:
        with self._lock:
            if status.session != self.session or status.expiry_latched:
                self._claim_acknowledged = False
                return False
            if (
                self._last_pause_sequence is not None
                and status.sequence == self._last_pause_sequence
                and status.mode == RpiCommandMode.PAUSE
                and status.arrival_fresh
                and status.lease_usable
            ):
                self._claim_acknowledged = True
            return self._claim_acknowledged
