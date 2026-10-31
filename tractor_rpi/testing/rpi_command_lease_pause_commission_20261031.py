#!/usr/bin/env python3
"""Motor-off, Pause-only commissioning tool for the RPi lease firmware.

This tool can never request Manual or Auto. It repeatedly sends checksummed
Pause frames, reads RPI_CTL acknowledgements, and records JSONL evidence.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

from tractor_rpi.rpi_command_lease_adapter_20261031 import (
    RpiCommandLeaseAdapter,
    RpiLeaseStatus,
    parse_rpi_ctl_line,
)
from tractor_rpi.rpi_command_lease_protocol_20261031 import RpiCommandMode, decode_frame


EXPECTED_FIRMWARE = "teensy_main_20261031_rpi_command_lease_v1"
MINIMUM_MATCHING_PAUSE_STATUSES = 5


@dataclass
class CommissionEvidence:
    session: int
    firmware_seen: bool = False
    matching_pause_statuses: int = 0
    non_pause_status_seen: bool = False
    expiry_latch_seen: bool = False
    foreign_session_seen_after_match: bool = False
    last_status: RpiLeaseStatus | None = None

    def process_line(self, line: str) -> RpiLeaseStatus | None:
        if "SYS" in line and f"fw={EXPECTED_FIRMWARE}" in line:
            self.firmware_seen = True
        if ",RPI_CTL," not in line:
            return None
        status = parse_rpi_ctl_line(line)
        self.last_status = status
        if status.mode != RpiCommandMode.PAUSE:
            self.non_pause_status_seen = True
        if status.expiry_latched:
            self.expiry_latch_seen = True
        if status.session != self.session and self.matching_pause_statuses > 0:
            self.foreign_session_seen_after_match = True
        if (
            status.session == self.session
            and status.mode == RpiCommandMode.PAUSE
            and status.arrival_fresh
            and status.lease_usable
            and not status.expiry_latched
        ):
            self.matching_pause_statuses += 1
        return status

    @property
    def passed(self) -> bool:
        return (
            self.firmware_seen
            and self.matching_pause_statuses >= MINIMUM_MATCHING_PAUSE_STATUSES
            and not self.non_pause_status_seen
            and not self.expiry_latch_seen
            and not self.foreign_session_seen_after_match
        )


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def write_event(log_file, event: str, **details) -> None:
    record = {"time": utc_now(), "event": event, **details}
    log_file.write(json.dumps(record, sort_keys=True) + "\n")
    log_file.flush()


def require_physical_confirmations(args) -> None:
    required = {
        "engine off": args.engine_off_confirmed,
        "blades off": args.blades_off_confirmed,
        "rear transmission lever out": args.rear_lever_out_confirmed,
        "physical Pause selected": args.physical_pause_confirmed,
        "wheels chocked": args.wheels_chocked_confirmed,
    }
    missing = [name for name, confirmed in required.items() if not confirmed]
    if missing:
        raise RuntimeError("missing physical confirmation: " + ", ".join(missing))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/teensy")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--duration-seconds", type=float, default=60.0)
    parser.add_argument("--log-dir", type=Path, required=True)
    parser.add_argument("--engine-off-confirmed", action="store_true")
    parser.add_argument("--blades-off-confirmed", action="store_true")
    parser.add_argument("--rear-lever-out-confirmed", action="store_true")
    parser.add_argument("--physical-pause-confirmed", action="store_true")
    parser.add_argument("--wheels-chocked-confirmed", action="store_true")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    require_physical_confirmations(args)
    if not 10.0 <= args.duration_seconds <= 600.0:
        raise RuntimeError("duration must be between 10 and 600 seconds")

    # Imported only in the live CLI path so unit tests need no serial device.
    import serial

    args.log_dir.mkdir(parents=True, exist_ok=True)
    log_path = args.log_dir / (
        "rpi_command_lease_pause_commission_"
        + datetime.now().strftime("%Y%m%d_%H%M%S")
        + ".jsonl"
    )
    serial_port = serial.Serial(
        args.port,
        args.baud,
        timeout=0.02,
        write_timeout=0.1,
        exclusive=True,
    )
    adapter = RpiCommandLeaseAdapter(serial_port)
    evidence = CommissionEvidence(adapter.session)
    started = time.monotonic()
    next_pause = started

    print(f"Pause-only commissioning session: {adapter.session}")
    print(f"Evidence: {log_path}")
    print("This tool cannot request Manual or Auto.")

    with log_path.open("a", encoding="utf-8") as log_file:
        write_event(
            log_file,
            "start",
            session=adapter.session,
            port=args.port,
            baud=args.baud,
            expected_firmware=EXPECTED_FIRMWARE,
        )
        try:
            while time.monotonic() - started < args.duration_seconds:
                now = time.monotonic()
                if now >= next_pause:
                    frame = adapter.send_pause()
                    command = decode_frame(frame)
                    write_event(
                        log_file,
                        "pause_sent",
                        session=command.session,
                        sequence=command.sequence,
                    )
                    next_pause = now + 0.25

                line = serial_port.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                write_event(log_file, "serial_line", line=line)
                try:
                    status = evidence.process_line(line)
                except ValueError as exc:
                    if ",RPI_CTL," in line:
                        write_event(log_file, "invalid_rpi_ctl", error=str(exc), line=line)
                    continue
                if status is not None:
                    adapter.observe_status(status)
        finally:
            # Best-effort Pause burst before releasing the serial device.
            for _ in range(3):
                try:
                    frame = adapter.send_pause()
                    command = decode_frame(frame)
                    write_event(
                        log_file,
                        "shutdown_pause_sent",
                        sequence=command.sequence,
                    )
                    time.sleep(0.05)
                except Exception as exc:
                    write_event(log_file, "shutdown_pause_failed", error=str(exc))
            serial_port.close()
            write_event(
                log_file,
                "finish",
                passed=evidence.passed,
                firmware_seen=evidence.firmware_seen,
                matching_pause_statuses=evidence.matching_pause_statuses,
                non_pause_status_seen=evidence.non_pause_status_seen,
                expiry_latch_seen=evidence.expiry_latch_seen,
                foreign_session_seen_after_match=evidence.foreign_session_seen_after_match,
            )

    print(
        "PASS" if evidence.passed else "FAIL",
        f"firmware={evidence.firmware_seen}",
        f"matching_pause_statuses={evidence.matching_pause_statuses}",
        f"non_pause={evidence.non_pause_status_seen}",
        f"expiry_latch={evidence.expiry_latch_seen}",
        f"foreign_session={evidence.foreign_session_seen_after_match}",
    )
    return 0 if evidence.passed else 1


if __name__ == "__main__":
    sys.exit(main())
