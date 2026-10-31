#!/usr/bin/env python3
"""Motor-off integration bridge for the RPi command-lease candidate.

This additive bridge owns the normal serial/UDP telemetry path but replaces
legacy CMD/WIFI forwarding with the generic RPICMD runtime.  It is deliberately
Pause-only: zero Pause packets are counted, while Manual, Auto, Pure Pursuit,
malformed, and nonzero packets are rejected and can never reach the adapter.

It is not installed by any production service and must not be used for motion.
"""

from __future__ import annotations

from dataclasses import asdict
import json
import math
from pathlib import Path
import select
import socket
import sys
import threading
import time
from typing import Any


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
REPO_ROOT = HERE.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import teensy_serial_bridge_20261002 as base
from tractor_rpi.communications_degraded_guard_20261029 import LocalSafetyHealth
from tractor_rpi.rpi_command_lease_adapter_20261031 import (
    RpiCommandLeaseAdapter,
    RpiLeaseStatus,
    parse_rpi_ctl_line,
)
from tractor_rpi.rpi_command_lease_protocol_20261031 import RpiCommandMode
from tractor_rpi.rpi_command_lease_runtime_20261031 import RpiCommandLeaseRuntime
from tractor_rpi.rpi_safety_supervisor_20261031 import SupervisorInputs


EXPECTED_FIRMWARE = "teensy_main_20261031_rpi_command_lease_v2"


class SynchronizedSerial:
    """Serialize writes from the GPS and command-lease threads."""

    def __init__(self, serial_port: Any):
        self._serial = serial_port
        self._write_lock = threading.Lock()

    def write(self, data: bytes) -> int | None:
        with self._write_lock:
            return self._serial.write(data)

    def flush(self) -> None:
        with self._write_lock:
            self._serial.flush()

    def __getattr__(self, name: str) -> Any:
        return getattr(self._serial, name)


class PauseOnlyUdpGate:
    """Validate UDP 6004 input without ever producing a motion command."""

    def __init__(self):
        self.accepted_pause_packets = 0
        self.rejected_packets = 0
        self.last_pause_monotonic_s: float | None = None
        self.last_rejection_reason: str | None = None

    def ingest(self, payload: Any, *, now_monotonic_s: float) -> bool:
        if not math.isfinite(now_monotonic_s):
            raise ValueError("now_monotonic_s must be finite")
        reason = None
        if not isinstance(payload, dict):
            reason = "UDP command is not a JSON object"
        else:
            try:
                mode = str(payload.get("phone_mode", "")).lower()
                drive = float(payload.get("drive_percent", math.nan))
                steering = float(payload.get("angular_z", math.nan))
            except (TypeError, ValueError):
                reason = "UDP Pause fields are not numeric"
            else:
                if mode != "pause":
                    reason = "motor-off bridge rejects every non-Pause phone mode"
                elif not math.isfinite(drive) or not math.isfinite(steering):
                    reason = "UDP Pause fields must be finite"
                elif drive != 0.0 or steering != 0.0:
                    reason = "motor-off Pause packet must carry zero drive and steering"

        if reason is not None:
            self.rejected_packets += 1
            self.last_rejection_reason = reason
            return False
        self.accepted_pause_packets += 1
        self.last_pause_monotonic_s = now_monotonic_s
        return True


def pause_supervisor_inputs(now_monotonic_s: float) -> SupervisorInputs:
    """Create the only decision permitted by this motor-off bridge."""
    health = LocalSafetyHealth(**{name: False for name in LocalSafetyHealth.__annotations__})
    return SupervisorInputs(
        now_monotonic_s=now_monotonic_s,
        requested_mode=RpiCommandMode.PAUSE,
        phone_link_fresh=False,
        communications_degraded_preauthorized=False,
        auto_running=False,
        manual_drive_percent=0.0,
        manual_steering=0.0,
        pure_pursuit_speed_mps=0.0,
        pure_pursuit_steering=0.0,
        route_progress_m=0.0,
        local_travel_m=0.0,
        health=health,
    )


class TeensySerialBridge(base.TeensySerialBridge):
    """Telemetry-compatible bridge with a hard Pause-only command boundary."""

    def __init__(self):
        self._lease_lock = threading.Lock()
        self._lease_status: RpiLeaseStatus | None = None
        self._lease_status_at = 0.0
        self.pause_gate = PauseOnlyUdpGate()
        super().__init__()
        self.ser = SynchronizedSerial(self.ser)
        self.adapter = RpiCommandLeaseAdapter(self.ser)
        self.runtime = RpiCommandLeaseRuntime(self.adapter)

    def parse_message(self, line):
        parsed = super().parse_message(line)
        if parsed and parsed.get("subsystem") == "RPI_CTL":
            try:
                status = parse_rpi_ctl_line(line)
            except ValueError as exc:
                base.base.logger.error("Invalid RPI_CTL telemetry: %s", exc)
            else:
                with self._lease_lock:
                    self._lease_status = status
                    self._lease_status_at = time.monotonic()
                self.adapter.observe_status(status)
        return parsed

    def listen_for_commands(self):
        base.base.logger.info("Pause-only UDP command listener started")
        while self.running:
            try:
                ready = select.select([self.command_sock], [], [], 0.1)
                if not ready[0]:
                    continue
                data, address = self.command_sock.recvfrom(8192)
                self.stats["commands_received"] += 1
                try:
                    payload = json.loads(data.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    accepted = self.pause_gate.ingest(
                        None, now_monotonic_s=time.monotonic()
                    )
                else:
                    accepted = self.pause_gate.ingest(
                        payload, now_monotonic_s=time.monotonic()
                    )
                if not accepted:
                    base.base.logger.warning(
                        "Rejected motor-off UDP command from %s: %s",
                        address[0],
                        self.pause_gate.last_rejection_reason,
                    )
            except OSError as exc:
                if self.running:
                    base.base.logger.error("Pause-only command listener error: %s", exc)
        base.base.logger.info("Pause-only UDP command listener stopped")

    def command_lease_thread(self):
        base.base.logger.info("Pause-only RPi command-lease scheduler started")
        while self.running:
            now = time.monotonic()
            with self._lease_lock:
                status = self._lease_status
            try:
                result = self.runtime.tick(
                    now_monotonic_s=now,
                    inputs=pause_supervisor_inputs(now),
                    status=status,
                )
                if result.frame_sent is not None:
                    self.stats["commands_sent"] += 1
                    self.cmd_vel_sent_count += 1
            except Exception as exc:
                base.base.logger.error("Command-lease scheduler fault: %s", exc)
                self.running = False
                break
            time.sleep(0.05)

    def create_broadcast_message(self):
        message = super().create_broadcast_message()
        with self._lease_lock:
            status = self._lease_status
            status_age_s = (
                None
                if status is None
                else max(0.0, time.monotonic() - self._lease_status_at)
            )
        lease = {
            "expected_firmware": EXPECTED_FIRMWARE,
            "pause_only": True,
            "session": self.adapter.session,
            "pause_claim_acknowledged": self.adapter.claim_acknowledged,
            "frames_sent": self.adapter.frames_sent,
            "accepted_pause_packets": self.pause_gate.accepted_pause_packets,
            "rejected_packets": self.pause_gate.rejected_packets,
            "last_rejection_reason": self.pause_gate.last_rejection_reason,
            "status_age_s": status_age_s,
        }
        if status is not None:
            lease["teensy_status"] = {
                **asdict(status),
                "mode": int(status.mode),
            }
            message["wifi_control"] = {
                "mode": int(status.mode),
                "heartbeat_fresh": int(status.arrival_fresh),
                "estop_latched": 0,
                "command_age_ms": status.command_age_ms,
                "age": status_age_s,
                "source": "rpi_command_lease_pause_only",
            }
        message["rpi_command_lease"] = lease
        return message

    def run(self):
        threading.Thread(target=self.command_lease_thread, daemon=True).start()
        super().run()


def main():
    try:
        bridge = TeensySerialBridge()
        bridge.run()
    except Exception as exc:
        base.base.logger.error("Fatal Pause-only command-lease bridge error: %s", exc, exc_info=True)


if __name__ == "__main__":
    main()
