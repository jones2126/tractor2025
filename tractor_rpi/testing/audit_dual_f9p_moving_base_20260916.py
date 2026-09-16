#!/usr/bin/env python3
"""Read-only configuration and RELPOSNED audit for tractor01's dual F9Ps.

Stop rtcm-server before running this utility because the service normally owns
both USB serial ports.  This script sends only UBX poll/CFG-VALGET requests; it
does not use CFG-VALSET and cannot change receiver configuration.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial


SYNC = b"\xb5\x62"
CLASS_CFG = 0x06
ID_VALGET = 0x8B
CLASS_NAV = 0x01
ID_RELPOSNED = 0x3C


@dataclass(frozen=True)
class Key:
    name: str
    key_id: int
    fmt: str


KEYS = (
    Key("CFG-RATE-MEAS", 0x30210001, "<H"),
    Key("CFG-RATE-NAV", 0x30210002, "<H"),
    Key("CFG-UART1-ENABLED", 0x10520005, "<B"),
    Key("CFG-UART1-BAUDRATE", 0x40520001, "<I"),
    Key("CFG-UART1INPROT-RTCM3X", 0x10730004, "<B"),
    Key("CFG-UART1OUTPROT-RTCM3X", 0x10740004, "<B"),
    Key("CFG-USBINPROT-RTCM3X", 0x10770004, "<B"),
    Key("CFG-USBOUTPROT-UBX", 0x10780001, "<B"),
    Key("CFG-TMODE-MODE", 0x20030001, "<B"),
    Key("CFG-MSGOUT-UBX_NAV_RELPOSNED_USB", 0x20910090, "<B"),
    Key("CFG-MSGOUT-RTCM_3X_TYPE1074_UART1", 0x2091035F, "<B"),
    Key("CFG-MSGOUT-RTCM_3X_TYPE1084_UART1", 0x20910364, "<B"),
    Key("CFG-MSGOUT-RTCM_3X_TYPE1094_UART1", 0x20910369, "<B"),
    Key("CFG-MSGOUT-RTCM_3X_TYPE1124_UART1", 0x2091036E, "<B"),
    Key("CFG-MSGOUT-RTCM_3X_TYPE1077_UART1", 0x209102CD, "<B"),
    Key("CFG-MSGOUT-RTCM_3X_TYPE1087_UART1", 0x209102D2, "<B"),
    Key("CFG-MSGOUT-RTCM_3X_TYPE1097_UART1", 0x20910319, "<B"),
    Key("CFG-MSGOUT-RTCM_3X_TYPE1127_UART1", 0x209102D7, "<B"),
    Key("CFG-MSGOUT-RTCM_3X_TYPE1230_UART1", 0x20910304, "<B"),
    Key("CFG-MSGOUT-RTCM_3X_TYPE4072_0_UART1", 0x209102FF, "<B"),
    Key("CFG-MSGOUT-RTCM_3X_TYPE4072_1_UART1", 0x20910382, "<B"),
)
KEY_BY_ID = {item.key_id: item for item in KEYS}

MSM4_NAMES = tuple(
    f"CFG-MSGOUT-RTCM_3X_TYPE{number}_UART1"
    for number in (1074, 1084, 1094, 1124)
)
MSM7_NAMES = tuple(
    f"CFG-MSGOUT-RTCM_3X_TYPE{number}_UART1"
    for number in (1077, 1087, 1097, 1127)
)


def checksum(data: bytes) -> tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for byte in data:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def frame(message_class: int, message_id: int, payload: bytes = b"") -> bytes:
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


def valget(port: serial.Serial, layer: int) -> dict[str, int]:
    payload = struct.pack("<BBH", 0, layer, 0)
    payload += b"".join(struct.pack("<I", item.key_id) for item in KEYS)
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
        result: dict[str, int] = {}
        while offset + 4 <= len(response):
            key_id = struct.unpack_from("<I", response, offset)[0]
            offset += 4
            item = KEY_BY_ID.get(key_id)
            if item is None:
                raise RuntimeError(f"unexpected configuration key 0x{key_id:08X}")
            width = struct.calcsize(item.fmt)
            if offset + width > len(response):
                raise RuntimeError(f"truncated value for {item.name}")
            result[item.name] = int(struct.unpack_from(item.fmt, response, offset)[0])
            offset += width
        return result
    raise TimeoutError(f"no UBX-CFG-VALGET response for layer {layer}")


def read_receiver(port_name: str) -> tuple[str, dict[str, dict[str, int]]]:
    resolved = str(Path(port_name).resolve())
    layers: dict[str, dict[str, int]] = {}
    with serial.Serial(port_name, 115200, timeout=0.15) as port:
        time.sleep(0.4)
        for name, number in (("RAM", 0), ("BBR", 1), ("FLASH", 2)):
            try:
                layers[name] = valget(port, number)
            except TimeoutError:
                layers[name] = {}
    return resolved, layers


def check_equal(
    checks: list[dict[str, object]],
    receiver: str,
    values: dict[str, int],
    name: str,
    expected: int,
) -> None:
    actual = values.get(name)
    checks.append(
        {
            "status": "PASS" if actual == expected else "FAIL",
            "receiver": receiver,
            "check": name,
            "expected": expected,
            "actual": actual,
        }
    )


def audit_configuration(
    base_layers: dict[str, dict[str, int]],
    heading_layers: dict[str, dict[str, int]],
) -> tuple[list[dict[str, object]], list[str]]:
    checks: list[dict[str, object]] = []
    warnings: list[str] = []
    base = base_layers.get("RAM", {})
    heading = heading_layers.get("RAM", {})

    for receiver, values in (("base", base), ("heading", heading)):
        check_equal(checks, receiver, values, "CFG-RATE-MEAS", 100)
        check_equal(checks, receiver, values, "CFG-RATE-NAV", 1)
        check_equal(checks, receiver, values, "CFG-UART1-ENABLED", 1)
        check_equal(checks, receiver, values, "CFG-UART1-BAUDRATE", 115200)
        check_equal(checks, receiver, values, "CFG-TMODE-MODE", 0)

    check_equal(checks, "base", base, "CFG-UART1INPROT-RTCM3X", 0)
    check_equal(checks, "base", base, "CFG-UART1OUTPROT-RTCM3X", 1)
    check_equal(checks, "base", base, "CFG-USBINPROT-RTCM3X", 1)
    check_equal(
        checks, "base", base, "CFG-MSGOUT-RTCM_3X_TYPE4072_0_UART1", 1
    )

    check_equal(checks, "heading", heading, "CFG-UART1INPROT-RTCM3X", 1)
    check_equal(checks, "heading", heading, "CFG-UART1OUTPROT-RTCM3X", 0)
    check_equal(checks, "heading", heading, "CFG-USBOUTPROT-UBX", 1)
    check_equal(
        checks, "heading", heading, "CFG-MSGOUT-UBX_NAV_RELPOSNED_USB", 1
    )

    msm4 = {name: base.get(name) for name in MSM4_NAMES}
    msm7 = {name: base.get(name) for name in MSM7_NAMES}
    msm4_complete = all(value == 1 for value in msm4.values())
    msm7_complete = all(value == 1 for value in msm7.values())
    checks.append(
        {
            "status": "PASS" if msm4_complete or msm7_complete else "FAIL",
            "receiver": "base",
            "check": "complete MSM4 or MSM7 constellation set on UART1",
            "expected": "1074/1084/1094/1124 or 1077/1087/1097/1127",
            "actual": {"MSM4": msm4, "MSM7": msm7},
        }
    )
    if msm7_complete and not msm4_complete:
        warnings.append(
            "Base-Link is sending the larger MSM7 family at every 10 Hz epoch. "
            "Modern F9P moving-base guidance recommends MSM4 to reduce UART load."
        )
    if base.get("CFG-MSGOUT-RTCM_3X_TYPE4072_1_UART1", 0):
        warnings.append(
            "RTCM 4072.1 is enabled. It is unnecessary on HPG 1.13+ and consumes "
            "additional UART bandwidth; firmware version must be confirmed before disabling it."
        )

    for receiver, layers in (("base", base_layers), ("heading", heading_layers)):
        ram = layers.get("RAM", {})
        for saved_name in ("BBR", "FLASH"):
            saved = layers.get(saved_name, {})
            if not saved:
                warnings.append(
                    f"{receiver}: {saved_name} layer could not be read; persistence is unverified."
                )
                continue
            differences = {
                name: {"RAM": value, saved_name: saved.get(name)}
                for name, value in ram.items()
                if saved.get(name) != value
            }
            if differences:
                warnings.append(
                    f"{receiver}: RAM differs from {saved_name}: "
                    + json.dumps(differences, sort_keys=True)
                )
    return checks, warnings


def parse_relposned(payload: bytes) -> dict[str, object] | None:
    if len(payload) < 64:
        return None
    i_tow = struct.unpack_from("<I", payload, 4)[0]
    length_cm = struct.unpack_from("<i", payload, 20)[0]
    hp_length_01mm = struct.unpack_from("<b", payload, 35)[0]
    accuracy_01mm = struct.unpack_from("<I", payload, 48)[0]
    flags = struct.unpack_from("<I", payload, 60)[0]
    carrier_code = (flags >> 3) & 0x03
    carrier = {0: "none", 1: "float", 2: "fixed"}.get(carrier_code, "reserved")
    return {
        "itow_ms": i_tow,
        "fix_ok": bool(flags & (1 << 0)),
        "diff_solution": bool(flags & (1 << 1)),
        "relpos_valid": bool(flags & (1 << 2)),
        "carrier": carrier,
        "moving": bool(flags & (1 << 5)),
        "ref_pos_miss": bool(flags & (1 << 6)),
        "ref_obs_miss": bool(flags & (1 << 7)),
        "head_valid": bool(flags & (1 << 8)),
        "baseline_m": length_cm * 0.01 + hp_length_01mm * 0.0001,
        "accuracy_m": accuracy_01mm * 0.0001,
    }


def observe_relposned(port_name: str, seconds: float) -> dict[str, object]:
    samples: list[dict[str, object]] = []
    observed_times: list[float] = []
    with serial.Serial(port_name, 115200, timeout=0.15) as port:
        port.reset_input_buffer()
        started = time.monotonic()
        deadline = started + seconds
        while time.monotonic() < deadline:
            message = read_frame(port, deadline)
            if message is None:
                break
            message_class, message_id, payload = message
            if message_class == CLASS_NAV and message_id == ID_RELPOSNED:
                parsed = parse_relposned(payload)
                if parsed is not None:
                    samples.append(parsed)
                    observed_times.append(time.monotonic())

    valid = [
        sample
        for sample in samples
        if sample["head_valid"]
        and sample["relpos_valid"]
        and sample["moving"]
        and sample["carrier"] == "fixed"
    ]
    invalid = [sample for sample in samples if sample not in valid]
    baselines = [float(sample["baseline_m"]) for sample in valid]
    accuracies = [float(sample["accuracy_m"]) for sample in valid]
    duration = max(0.001, time.monotonic() - started)
    intervals = [b - a for a, b in zip(observed_times, observed_times[1:])]
    invalid_examples = invalid[:10]
    return {
        "requested_seconds": seconds,
        "observed_seconds": round(duration, 3),
        "frames": len(samples),
        "frame_rate_hz": round(len(samples) / duration, 3),
        "valid_fixed_frames": len(valid),
        "invalid_frames": len(invalid),
        "valid_percent": round(100.0 * len(valid) / len(samples), 3) if samples else 0.0,
        "median_interframe_seconds": round(statistics.median(intervals), 4) if intervals else None,
        "median_valid_baseline_m": round(statistics.median(baselines), 4) if baselines else None,
        "median_valid_accuracy_m": round(statistics.median(accuracies), 4) if accuracies else None,
        "invalid_examples": invalid_examples,
    }


def print_checks(checks: list[dict[str, object]]) -> None:
    for item in checks:
        status = item["status"]
        receiver = item["receiver"]
        name = item["check"]
        print(
            f"[{status}] {receiver:7s} {name}: "
            f"actual={item['actual']!r}; expected={item['expected']!r}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only dual-F9P moving-base configuration audit"
    )
    parser.add_argument("--base-port", default="/dev/gps-base-link")
    parser.add_argument("--heading-port", default="/dev/gps-heading")
    parser.add_argument("--observe-seconds", type=float, default=120.0)
    parser.add_argument("--json-output")
    args = parser.parse_args()
    if not math.isfinite(args.observe_seconds) or args.observe_seconds < 5:
        parser.error("--observe-seconds must be at least 5")

    print("READ-ONLY DUAL-F9P MOVING-BASE AUDIT")
    print("No receiver settings will be changed.\n")
    base_resolved, base_layers = read_receiver(args.base_port)
    heading_resolved, heading_layers = read_receiver(args.heading_port)
    print(f"Base:    {args.base_port} -> {base_resolved}")
    print(f"Heading: {args.heading_port} -> {heading_resolved}\n")

    checks, warnings = audit_configuration(base_layers, heading_layers)
    print_checks(checks)
    for warning in warnings:
        print(f"[WARN] {warning}")

    print(
        f"\nObserving raw heading RELPOSNED for {args.observe_seconds:.0f} seconds..."
    )
    observation = observe_relposned(args.heading_port, args.observe_seconds)
    print(json.dumps(observation, indent=2, sort_keys=True))

    failed = [item for item in checks if item["status"] == "FAIL"]
    if observation["frames"] == 0:
        failed.append({"check": "RELPOSNED frames observed"})
    elif observation["invalid_frames"]:
        warnings.append(
            f"Observed {observation['invalid_frames']} invalid RELPOSNED frame(s)."
        )

    report = {
        "base": {"port": args.base_port, "resolved": base_resolved, "layers": base_layers},
        "heading": {
            "port": args.heading_port,
            "resolved": heading_resolved,
            "layers": heading_layers,
        },
        "checks": checks,
        "warnings": warnings,
        "relposned_observation": observation,
        "result": "FAIL" if failed else "PASS_WITH_WARNINGS" if warnings else "PASS",
    }
    if args.json_output:
        output = Path(args.json_output)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"Saved audit report: {output.resolve()}")

    print(f"\nAUDIT RESULT: {report['result']}")
    return 2 if failed else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, serial.SerialException, RuntimeError, TimeoutError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
