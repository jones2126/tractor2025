#!/usr/bin/env python3
"""Guarded tractor01 dual-F9P 5 Hz moving-base A/B configuration.

The utility is read-only unless --apply, --restore, or --heading-startup is
supplied. It changes only the listed navigation-rate, UART1 protocol/message,
and Heading USB output keys. Apply writes RAM, battery-backed RAM, and flash,
saves every changed key for both receivers first, and verifies the stored
target values independently.

Stop rtcm-server before running because it normally owns both serial devices.
Do not use ArduSimple stock configuration files on the tractor's custom
UART1-to-UART1 wiring.

The non-interactive --heading-startup action is intended only for the
rtcm-server systemd ExecStartPre step. It rewrites and verifies the known
heading profile in volatile RAM on every service start without wearing flash.
"""

from __future__ import annotations

import argparse
import json
import struct
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import serial


SYNC = b"\xb5\x62"
CLASS_CFG = 0x06
ID_VALSET = 0x8A
ID_VALGET = 0x8B
CLASS_ACK = 0x05
ID_ACK_NAK = 0x00
ID_ACK_ACK = 0x01
LAYERS_RAM_BBR_FLASH = 0x07
LAYER_RAM = 0x01


@dataclass(frozen=True)
class Setting:
    name: str
    key_id: int
    fmt: str
    target: int


RATE_SETTINGS = (
    Setting("CFG-RATE-MEAS", 0x30210001, "<H", 200),
    Setting("CFG-RATE-NAV", 0x30210002, "<H", 1),
)

BASE_SETTINGS = RATE_SETTINGS + (
    Setting("CFG-UART1-ENABLED", 0x10520005, "<B", 1),
    Setting("CFG-UART1-BAUDRATE", 0x40520001, "<I", 115200),
    Setting("CFG-UART1INPROT-RTCM3X", 0x10730004, "<B", 0),
    Setting("CFG-UART1OUTPROT-RTCM3X", 0x10740004, "<B", 1),
    Setting("CFG-USBINPROT-RTCM3X", 0x10770004, "<B", 1),
    Setting("CFG-MSGOUT-RTCM_3X_TYPE1074_UART1", 0x2091035F, "<B", 1),
    Setting("CFG-MSGOUT-RTCM_3X_TYPE1084_UART1", 0x20910364, "<B", 1),
    Setting("CFG-MSGOUT-RTCM_3X_TYPE1094_UART1", 0x20910369, "<B", 1),
    Setting("CFG-MSGOUT-RTCM_3X_TYPE1124_UART1", 0x2091036E, "<B", 1),
    Setting("CFG-MSGOUT-RTCM_3X_TYPE1077_UART1", 0x209102CD, "<B", 0),
    Setting("CFG-MSGOUT-RTCM_3X_TYPE1087_UART1", 0x209102D2, "<B", 0),
    Setting("CFG-MSGOUT-RTCM_3X_TYPE1097_UART1", 0x20910319, "<B", 0),
    Setting("CFG-MSGOUT-RTCM_3X_TYPE1127_UART1", 0x209102D7, "<B", 0),
    Setting("CFG-MSGOUT-RTCM_3X_TYPE1230_UART1", 0x20910304, "<B", 1),
    Setting("CFG-MSGOUT-RTCM_3X_TYPE4072_0_UART1", 0x209102FF, "<B", 1),
    Setting("CFG-MSGOUT-RTCM_3X_TYPE4072_1_UART1", 0x20910382, "<B", 0),
)

NMEA_USB_SETTINGS = (
    Setting("CFG-MSGOUT-NMEA_ID_DTM_USB", 0x209100A9, "<B", 0),
    Setting("CFG-MSGOUT-NMEA_ID_GBS_USB", 0x209100E0, "<B", 0),
    Setting("CFG-MSGOUT-NMEA_ID_GGA_USB", 0x209100BD, "<B", 0),
    Setting("CFG-MSGOUT-NMEA_ID_GLL_USB", 0x209100CC, "<B", 0),
    Setting("CFG-MSGOUT-NMEA_ID_GNS_USB", 0x209100B8, "<B", 0),
    Setting("CFG-MSGOUT-NMEA_ID_GRS_USB", 0x209100D1, "<B", 0),
    Setting("CFG-MSGOUT-NMEA_ID_GSA_USB", 0x209100C2, "<B", 0),
    Setting("CFG-MSGOUT-NMEA_ID_GST_USB", 0x209100D6, "<B", 0),
    Setting("CFG-MSGOUT-NMEA_ID_GSV_USB", 0x209100C7, "<B", 0),
    Setting("CFG-MSGOUT-NMEA_ID_RMC_USB", 0x209100AE, "<B", 0),
    Setting("CFG-MSGOUT-NMEA_ID_VLW_USB", 0x209100EA, "<B", 0),
    Setting("CFG-MSGOUT-NMEA_ID_VTG_USB", 0x209100B3, "<B", 0),
    Setting("CFG-MSGOUT-NMEA_ID_ZDA_USB", 0x209100DB, "<B", 0),
)

HEADING_SETTINGS = RATE_SETTINGS + (
    Setting("CFG-UART1-ENABLED", 0x10520005, "<B", 1),
    Setting("CFG-UART1-BAUDRATE", 0x40520001, "<I", 115200),
    Setting("CFG-UART1INPROT-RTCM3X", 0x10730004, "<B", 1),
    Setting("CFG-UART1OUTPROT-RTCM3X", 0x10740004, "<B", 0),
    Setting("CFG-USBOUTPROT-UBX", 0x10780001, "<B", 1),
    Setting("CFG-USBOUTPROT-NMEA", 0x10780002, "<B", 0),
    Setting("CFG-MSGOUT-UBX_NAV_RELPOSNED_USB", 0x20910090, "<B", 1),
) + NMEA_USB_SETTINGS


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


def valget(port: serial.Serial, settings: tuple[Setting, ...]) -> dict[str, int]:
    by_id = {setting.key_id: setting for setting in settings}
    payload = struct.pack("<BBH", 0, 0, 0)
    payload += b"".join(struct.pack("<I", setting.key_id) for setting in settings)
    port.reset_input_buffer()
    port.write(frame(CLASS_CFG, ID_VALGET, payload))
    port.flush()
    deadline = time.monotonic() + 4.0
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
        values: dict[str, int] = {}
        while offset + 4 <= len(response):
            key_id = struct.unpack_from("<I", response, offset)[0]
            offset += 4
            setting = by_id.get(key_id)
            if setting is None:
                raise RuntimeError(f"unexpected configuration key 0x{key_id:08X}")
            width = struct.calcsize(setting.fmt)
            if offset + width > len(response):
                raise RuntimeError(f"truncated value for {setting.name}")
            values[setting.name] = int(
                struct.unpack_from(setting.fmt, response, offset)[0]
            )
            offset += width
        missing = [setting.name for setting in settings if setting.name not in values]
        if missing:
            raise RuntimeError(f"missing configuration values: {missing}")
        return values
    raise TimeoutError("no UBX-CFG-VALGET response received")


def wait_for_ack(port: serial.Serial) -> str:
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        message = read_frame(port, deadline)
        if message is None:
            break
        message_class, message_id, payload = message
        if message_class != CLASS_ACK or len(payload) < 2:
            continue
        if tuple(payload[:2]) != (CLASS_CFG, ID_VALSET):
            continue
        if message_id == ID_ACK_ACK:
            return "ACK"
        if message_id == ID_ACK_NAK:
            return "NAK"
    return "NO_ACK"


def valset(
    port: serial.Serial,
    settings: tuple[Setting, ...],
    values: dict[str, int],
    layer_mask: int = LAYERS_RAM_BBR_FLASH,
):
    payload = struct.pack("<BBBB", 0, layer_mask, 0, 0)
    for setting in settings:
        payload += struct.pack("<I", setting.key_id)
        payload += struct.pack(setting.fmt, int(values[setting.name]))
    port.reset_input_buffer()
    port.write(frame(CLASS_CFG, ID_VALSET, payload))
    port.flush()
    return wait_for_ack(port)


def read_values(port_name: str, settings: tuple[Setting, ...]) -> dict[str, int]:
    with serial.Serial(port_name, 115200, timeout=0.15) as port:
        time.sleep(0.4)
        return valget(port, settings)


def write_and_verify(
    label: str,
    port_name: str,
    settings: tuple[Setting, ...],
    values: dict[str, int],
    layer_mask: int = LAYERS_RAM_BBR_FLASH,
) -> None:
    with serial.Serial(port_name, 115200, timeout=0.15) as port:
        time.sleep(0.4)
        ack = valset(port, settings, values, layer_mask=layer_mask)
        print(f"{label}: UBX-CFG-VALSET response: {ack}")
        if ack == "NAK":
            raise RuntimeError(f"{label}: receiver rejected UBX-CFG-VALSET")
        time.sleep(0.8)
        actual = valget(port, settings)
    mismatches = {
        name: {"expected": expected, "actual": actual.get(name)}
        for name, expected in values.items()
        if actual.get(name) != expected
    }
    if mismatches:
        raise RuntimeError(f"{label}: readback mismatch: {mismatches}")
    print(f"{label}: PASS independent readback of {len(values)} targeted values")


def target_values(settings: tuple[Setting, ...]) -> dict[str, int]:
    return {setting.name: setting.target for setting in settings}


def print_comparison(label: str, current: dict[str, int], target: dict[str, int]):
    print(f"\n{label} current -> 5 Hz target:")
    for name, expected in target.items():
        marker = "=" if current[name] == expected else "->"
        print(f"  {name}: {current[name]} {marker} {expected}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-port", default="/dev/gps-base-link")
    parser.add_argument("--heading-port", default="/dev/gps-heading")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--apply", action="store_true")
    action.add_argument("--restore", metavar="BACKUP_JSON")
    action.add_argument(
        "--heading-startup",
        action="store_true",
        help=(
            "non-interactively rewrite and verify the Heading 5 Hz profile "
            "in volatile RAM for the rtcm-server ExecStartPre step"
        ),
    )
    parser.add_argument(
        "--device-wait-seconds",
        type=float,
        default=30.0,
        help="maximum wait for --heading-port to appear in startup mode",
    )
    parser.add_argument(
        "--backup",
        default="/home/al/dual_f9p_before_5hz_20260923.json",
        help="targeted-value backup written before --apply",
    )
    args = parser.parse_args()

    if args.heading_startup:
        if args.device_wait_seconds < 0:
            parser.error("--device-wait-seconds must not be negative")
        deadline = time.monotonic() + args.device_wait_seconds
        heading_path = Path(args.heading_port)
        while not heading_path.exists() and time.monotonic() < deadline:
            time.sleep(0.25)
        if not heading_path.exists():
            raise RuntimeError(
                f"heading device did not appear within "
                f"{args.device_wait_seconds:.1f}s: {args.heading_port}"
            )
        heading_target = target_values(HEADING_SETTINGS)
        print(
            "HEADING STARTUP RECOVERY: writing the tractor01 5 Hz moving-base "
            "profile to volatile RAM"
        )
        write_and_verify(
            "Heading startup",
            args.heading_port,
            HEADING_SETTINGS,
            heading_target,
            layer_mask=LAYER_RAM,
        )
        print(
            "PASS: heading startup profile is verified in RAM; releasing "
            "the serial port to rtcm-server"
        )
        return 0

    if args.restore:
        backup_path = Path(args.restore)
        backup = json.loads(backup_path.read_text(encoding="utf-8"))
        confirmation = input(
            "Type RESTORE DUAL F9P BACKUP to write the saved targeted values: "
        )
        if confirmation != "RESTORE DUAL F9P BACKUP":
            print("Aborted; no settings were changed.")
            return 1
        write_and_verify(
            "Base-Link", args.base_port, BASE_SETTINGS, backup["base"]["values"]
        )
        write_and_verify(
            "Heading", args.heading_port, HEADING_SETTINGS, backup["heading"]["values"]
        )
        print("PASS: targeted dual-F9P values restored and verified.")
        return 0

    base_before = read_values(args.base_port, BASE_SETTINGS)
    heading_before = read_values(args.heading_port, HEADING_SETTINGS)
    base_target = target_values(BASE_SETTINGS)
    heading_target = target_values(HEADING_SETTINGS)
    print("READ-ONLY DUAL-F9P 5 HZ PROFILE REVIEW" if not args.apply else "DUAL-F9P 5 HZ PROFILE APPLY")
    print("No stock receiver file or factory reset is used.")
    print_comparison("Base-Link", base_before, base_target)
    print_comparison("Heading", heading_before, heading_target)
    if not args.apply:
        print("\nRead-only mode. Add --apply only after reviewing this complete diff.")
        return 0

    backup_path = Path(args.backup)
    backup = {
        "schema": "tractor01-dual-f9p-targeted-backup-v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "base": {
            "port": args.base_port,
            "resolved": str(Path(args.base_port).resolve()),
            "values": base_before,
        },
        "heading": {
            "port": args.heading_port,
            "resolved": str(Path(args.heading_port).resolve()),
            "values": heading_before,
        },
    }
    backup_path.write_text(json.dumps(backup, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"\nSaved every targeted pre-change value: {backup_path}")
    print(
        "Restore command: sudo python3 "
        "tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py "
        f"--restore {backup_path}"
    )
    confirmation = input(
        "Type APPLY DUAL F9P 5HZ PROFILE to write both receivers: "
    )
    if confirmation != "APPLY DUAL F9P 5HZ PROFILE":
        print("Aborted; backup retained and no settings were changed.")
        return 1

    write_and_verify("Base-Link", args.base_port, BASE_SETTINGS, base_target)
    write_and_verify("Heading", args.heading_port, HEADING_SETTINGS, heading_target)
    print("PASS: dual-F9P 5 Hz profile applied and independently verified.")
    print("Restart rtcm-server, wait for LED4, then rerun the 120-second audit.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, TimeoutError, serial.SerialException) as exc:
        raise SystemExit(f"ERROR: {exc}") from exc
