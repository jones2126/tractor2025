#!/usr/bin/env python3
"""Recover the tractor heading F9P configuration without u-center.

This deliberately changes only the settings needed by rtcm_server_20260727.py:

* UART1 enabled at 115200 baud with RTCM3 input enabled
* UBX output enabled on USB
* UBX-NAV-RELPOSNED enabled on USB once per navigation solution
* 100 ms measurement period and one measurement per navigation solution (10 Hz)

Settings are written to RAM, battery-backed RAM, and flash.  The script does
not factory-reset the receiver and does not change constellation, antenna, or
positioning-model settings.

Stop rtcm-server before using this utility because only one process may own the
serial device.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from pathlib import Path

import serial


SYNC = b"\xb5\x62"
CLASS_CFG = 0x06
ID_VALSET = 0x8A
ID_VALGET = 0x8B
CLASS_ACK = 0x05
ID_ACK_NAK = 0x00
ID_ACK_ACK = 0x01
CLASS_NAV = 0x01
ID_RELPOSNED = 0x3C

# Configuration key IDs and value widths from the u-blox F9 interface
# description.  Boolean/L and U1 values are both encoded as one byte.
SETTINGS = [
    ("CFG-UART1-ENABLED", 0x10520005, "<B", 1),
    ("CFG-UART1-BAUDRATE", 0x40520001, "<I", 115200),
    ("CFG-UART1INPROT-RTCM3X", 0x10730004, "<B", 1),
    ("CFG-USBOUTPROT-UBX", 0x10780001, "<B", 1),
    ("CFG-MSGOUT-UBX_NAV_RELPOSNED_USB", 0x20910090, "<B", 1),
    ("CFG-RATE-MEAS", 0x30210001, "<H", 100),
    ("CFG-RATE-NAV", 0x30210002, "<H", 1),
]

LAYERS_RAM_BBR_FLASH = 0x07


def checksum(data: bytes) -> tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for byte in data:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def frame(message_class: int, message_id: int, payload: bytes) -> bytes:
    body = struct.pack("<BBH", message_class, message_id, len(payload)) + payload
    return SYNC + body + bytes(checksum(body))


def read_frame(port: serial.Serial, deadline: float):
    """Read one checksum-valid UBX frame while ignoring NMEA/other bytes."""
    sync_state = 0
    while time.monotonic() < deadline:
        byte = port.read(1)
        if not byte:
            continue
        value = byte[0]
        if sync_state == 0:
            sync_state = 1 if value == SYNC[0] else 0
            continue
        if value != SYNC[1]:
            sync_state = 1 if value == SYNC[0] else 0
            continue

        header = port.read(4)
        if len(header) != 4:
            sync_state = 0
            continue
        message_class, message_id, length = struct.unpack("<BBH", header)
        body = port.read(length + 2)
        if len(body) != length + 2:
            sync_state = 0
            continue
        payload = body[:length]
        if checksum(header + payload) != (body[-2], body[-1]):
            sync_state = 0
            continue
        return message_class, message_id, payload
    return None


def expect_ack(
    port: serial.Serial,
    expected_class: int,
    expected_id: int,
    timeout_s: float = 3.0,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = read_frame(port, deadline)
        if message is None:
            break
        message_class, message_id, payload = message
        if message_class != CLASS_ACK or len(payload) < 2:
            continue
        if payload[:2] != bytes((expected_class, expected_id)):
            continue
        if message_id == ID_ACK_ACK:
            return
        if message_id == ID_ACK_NAK:
            raise RuntimeError(
                f"receiver rejected UBX {expected_class:02X}/{expected_id:02X}"
            )
    raise TimeoutError(
        f"no ACK received for UBX {expected_class:02X}/{expected_id:02X}"
    )


def valget(port: serial.Serial) -> dict[str, int]:
    payload = struct.pack("<BBH", 0, 0, 0)
    payload += b"".join(struct.pack("<I", key) for _, key, _, _ in SETTINGS)
    port.write(frame(CLASS_CFG, ID_VALGET, payload))
    port.flush()

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        message = read_frame(port, deadline)
        if message is None:
            break
        message_class, message_id, response = message
        if message_class != CLASS_CFG or message_id != ID_VALGET:
            continue
        if len(response) < 4:
            raise RuntimeError("short UBX-CFG-VALGET response")
        offset = 4
        values: dict[int, int] = {}
        formats = {key: fmt for _, key, fmt, _ in SETTINGS}
        while offset + 4 <= len(response):
            key = struct.unpack_from("<I", response, offset)[0]
            offset += 4
            fmt = formats.get(key)
            if fmt is None:
                raise RuntimeError(f"unexpected key 0x{key:08X} in VALGET")
            width = struct.calcsize(fmt)
            if offset + width > len(response):
                raise RuntimeError("truncated value in UBX-CFG-VALGET response")
            values[key] = int(struct.unpack_from(fmt, response, offset)[0])
            offset += width
        return {
            name: values[key]
            for name, key, _, _ in SETTINGS
            if key in values
        }
    raise TimeoutError("no UBX-CFG-VALGET response received")


def apply_settings(port: serial.Serial) -> None:
    # Version 0, layer mask, transaction=0, reserved=0.
    payload = struct.pack("<BBBB", 0, LAYERS_RAM_BBR_FLASH, 0, 0)
    for _name, key, fmt, value in SETTINGS:
        payload += struct.pack("<I", key)
        payload += struct.pack(fmt, value)
    port.write(frame(CLASS_CFG, ID_VALSET, payload))
    port.flush()
    expect_ack(port, CLASS_CFG, ID_VALSET)


def count_relposned(port: serial.Serial, seconds: float = 3.0) -> int:
    count = 0
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        message = read_frame(port, deadline)
        if message is None:
            break
        message_class, message_id, _payload = message
        if message_class == CLASS_NAV and message_id == ID_RELPOSNED:
            count += 1
    return count


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Recover tractor heading F9P RELPOSNED configuration"
    )
    parser.add_argument("--port", default="/dev/gps-heading")
    parser.add_argument(
        "--apply",
        action="store_true",
        help="write the guarded heading configuration",
    )
    parser.add_argument(
        "--backup",
        default="heading_f9p_config_before_20260727.json",
        help="JSON file for the targeted pre-change values",
    )
    args = parser.parse_args()

    resolved = Path(args.port).resolve()
    print(f"Serial device: {args.port} -> {resolved}")
    if not args.apply:
        print("Read-only mode. Add --apply to configure the heading receiver.")

    with serial.Serial(args.port, 115200, timeout=0.15) as port:
        time.sleep(0.5)
        before = valget(port)
        print("Current targeted settings:")
        print(json.dumps(before, indent=2, sort_keys=True))

        if not args.apply:
            return 0

        confirmation = input(
            "Type CONFIGURE HEADING to write RAM, BBR, and Flash: "
        )
        if confirmation != "CONFIGURE HEADING":
            print("Aborted; no settings were changed.")
            return 1

        Path(args.backup).write_text(
            json.dumps(before, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"Saved targeted-value backup: {Path(args.backup).resolve()}")

        apply_settings(port)
        print("Receiver acknowledged UBX-CFG-VALSET.")
        time.sleep(0.5)

        after = valget(port)
        print("Configured targeted settings:")
        print(json.dumps(after, indent=2, sort_keys=True))

        expected = {name: value for name, _key, _fmt, value in SETTINGS}
        if after != expected:
            missing_or_different = {
                name: {"expected": value, "actual": after.get(name)}
                for name, value in expected.items()
                if after.get(name) != value
            }
            raise RuntimeError(
                "configuration readback mismatch: "
                + json.dumps(missing_or_different, sort_keys=True)
            )

        relpos_count = count_relposned(port)
        if relpos_count == 0:
            raise RuntimeError(
                "configuration readback passed, but no UBX-NAV-RELPOSNED "
                "frames were observed in 3 seconds"
            )
        print(
            f"PASS: observed {relpos_count} UBX-NAV-RELPOSNED frame(s) "
            "in 3 seconds."
        )
        print(
            "Restart rtcm-server, then verify heading_deg/headValid on UDP 6009."
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, serial.SerialException, RuntimeError, TimeoutError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
