#!/usr/bin/env python3
"""In-memory serial fault channel for the RPi command-lease protocol."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import heapq

from tractor_rpi.rpi_command_lease_protocol_20261031 import CommandLeaseReference


class SerialFault(str, Enum):
    PASS = "pass"
    DROP = "drop"
    DUPLICATE = "duplicate"
    CORRUPT = "corrupt"


@dataclass(order=True)
class ScheduledFrame:
    delivery_ms: float
    order: int
    frame: bytes = field(compare=False)
    physical_pause_active: bool = field(compare=False, default=False)


class RpiCommandLeaseSerialFaultSimulator:
    def __init__(self, receiver: CommandLeaseReference | None = None):
        self.receiver = receiver or CommandLeaseReference()
        self._pending: list[ScheduledFrame] = []
        self._order = 0
        self.accepted = 0
        self.rejected = 0

    @staticmethod
    def _corrupt(frame: bytes) -> bytes:
        mutable = bytearray(frame)
        separator = mutable.find(b"*")
        if separator <= 1:
            return b"corrupt\n"
        index = separator - 1
        mutable[index] = ord("9") if mutable[index] != ord("9") else ord("8")
        return bytes(mutable)

    def transmit(
        self,
        frame: bytes,
        *,
        sent_ms: float,
        delay_ms: float = 0.0,
        fault: SerialFault = SerialFault.PASS,
        physical_pause_active: bool = False,
    ) -> None:
        if fault == SerialFault.DROP:
            return
        delivered_frame = self._corrupt(frame) if fault == SerialFault.CORRUPT else frame
        copies = 2 if fault == SerialFault.DUPLICATE else 1
        for copy in range(copies):
            self._order += 1
            heapq.heappush(
                self._pending,
                ScheduledFrame(
                    sent_ms + delay_ms + copy * 0.001,
                    self._order,
                    delivered_frame,
                    physical_pause_active,
                ),
            )

    def deliver_until(self, now_ms: float) -> None:
        while self._pending and self._pending[0].delivery_ms <= now_ms:
            scheduled = heapq.heappop(self._pending)
            accepted = self.receiver.accept(
                scheduled.frame,
                now_ms=scheduled.delivery_ms,
                physical_pause_active=scheduled.physical_pause_active,
            )
            if accepted:
                self.accepted += 1
            else:
                self.rejected += 1
