#!/usr/bin/env python3
"""Dated tractor01 preflight with a Heading-F9P satellite gate.

This successor preserves every check from mission_preflight_20260804.py and
adds a fail-closed check of the Heading receiver's live NAV-PVT/NAV-SAT
satellites-used count:

* greater than 25: PASS;
* 24 or 25: WARNING (the preflight may continue); and
* 23 or fewer, missing, or inadequately sampled: FAIL.
"""

from __future__ import annotations

import re
import statistics
import sys
from pathlib import Path
from typing import Any


SCRIPT_DIR = str(Path(__file__).resolve().parent)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import mission_preflight_20260804 as base


HEADING_SATELLITE_PASS_ABOVE = 25
HEADING_SATELLITE_WARNING_ABOVE = 23

EXPECTED_ETH0_ADDRESS = "192.168.8.20"
EXPECTED_ETH0_GATEWAY = "192.168.8.1"
EXPECTED_ZEROTIER_ADDRESS = "192.168.193.76"
RTK_BASE_ZEROTIER_ADDRESS = "192.168.193.88"


def heading_startup_configuration_check() -> base.Check:
    code, output = base.command_output(
        [
            "systemctl",
            "show",
            "rtcm-server.service",
            "--property=ExecStartPre",
            "--value",
        ]
    )
    required = (
        "configure_dual_f9p_5hz_profile_20260923.py",
        "--heading-startup",
    )
    hook_installed = code == 0 and all(item in output for item in required)
    explicit_success = "code=exited" in output and "status=0" in output
    journal_proof = False
    if hook_installed and not explicit_success:
        journal_code, journal = base.command_output(
            [
                "journalctl",
                "-u",
                "rtcm-server.service",
                "-b",
                "--no-pager",
                "-o",
                "cat",
            ],
            timeout=8.0,
        )
        journal_proof = journal_code == 0 and all(
            marker in journal
            for marker in (
                "HEADING STARTUP RECOVERY",
                "Heading startup: PASS independent readback",
                "PASS: heading startup profile is verified in RAM",
            )
        )
    passed = hook_installed and (explicit_success or journal_proof)
    detail = (
        "current rtcm-server invocation completed the RAM-only Heading-F9P "
        "write and independent readback"
        if passed
        else "installed/current rtcm-server ExecStartPre proof is missing or failed"
    )
    return base.Check("Heading startup configuration", passed, detail)


def network_path_checks() -> list[base.Check]:
    checks: list[base.Check] = []

    code, output = base.command_output(
        ["ip", "-o", "-4", "addr", "show", "dev", "eth0"]
    )
    address_ok = code == 0 and re.search(
        rf"\binet\s+{re.escape(EXPECTED_ETH0_ADDRESS)}/24\b", output
    ) is not None
    checks.append(
        base.Check(
            "Wired mission address",
            address_ok,
            (
                f"eth0 has {EXPECTED_ETH0_ADDRESS}/24"
                if address_ok
                else output or "eth0 address unavailable"
            ),
        )
    )

    code, output = base.command_output(["ip", "-4", "route", "get", "1.1.1.1"])
    route_ok = code == 0 and all(
        token in output
        for token in (
            f"via {EXPECTED_ETH0_GATEWAY}",
            "dev eth0",
            f"src {EXPECTED_ETH0_ADDRESS}",
        )
    )
    checks.append(
        base.Check(
            "Wired default network path",
            route_ok,
            output.replace("\n", " ") or "route unavailable",
        )
    )

    code, output = base.command_output(["iw", "dev", "wlan0", "link"])
    ssid_match = re.search(r"^\s*SSID:\s*(.+?)\s*$", output, re.MULTILINE)
    ssid = ssid_match.group(1) if ssid_match else None
    wifi_ok = code == 0 and ssid != "TractorField"
    wifi_detail = (
        "not connected"
        if "Not connected" in output
        else f"SSID={ssid or 'unknown'}"
    )
    checks.append(base.Check("Pi direct TractorField Wi-Fi", wifi_ok, wifi_detail))

    code, output = base.command_output(
        ["ping", "-I", "eth0", "-c", "3", "-W", "2", EXPECTED_ETH0_GATEWAY],
        timeout=10.0,
    )
    loss_match = re.search(r"([0-9.]+)% packet loss", output)
    loss = float(loss_match.group(1)) if loss_match else 100.0
    gateway_ok = code == 0 and loss == 0.0
    checks.append(
        base.Check(
            "GL router wired gateway",
            gateway_ok,
            f"{loss:g}% packet loss to {EXPECTED_ETH0_GATEWAY} over eth0",
        )
    )

    code, output = base.command_output(
        ["ip", "-4", "route", "get", RTK_BASE_ZEROTIER_ADDRESS]
    )
    zt_match = re.search(r"\bdev\s+(zt\S+)", output)
    zt_ok = (
        code == 0
        and zt_match is not None
        and f"src {EXPECTED_ZEROTIER_ADDRESS}" in output
    )
    checks.append(
        base.Check(
            "RTK base ZeroTier route",
            zt_ok,
            output.replace("\n", " ") or "route unavailable",
        )
    )
    return checks


def heading_satellites_used_check(
    samples: list[tuple[float, dict[str, Any]]],
) -> base.Check:
    values = [
        int(float(message["heading_numSV_used"]))
        for _, message in samples
        if base.finite_number(message.get("heading_numSV_used"))
    ]
    required_samples = max(1, int(len(samples) * base.REQUIRED_DATA_FRACTION))
    if not samples or len(values) < required_samples:
        return base.Check(
            "Heading satellites used",
            False,
            (
                f"valid in {len(values)}/{len(samples)} packets; require at least "
                f"{base.REQUIRED_DATA_FRACTION:.0%} live coverage"
            ),
        )

    recent = values[-20:]
    median_used = statistics.median(recent)
    detail = (
        f"median recent={median_used:g}; latest={values[-1]}; "
        f"range={min(recent)}-{max(recent)}; "
        f"PASS >{HEADING_SATELLITE_PASS_ABOVE}, "
        f"WARNING {HEADING_SATELLITE_WARNING_ABOVE + 1}-"
        f"{HEADING_SATELLITE_PASS_ABOVE}, "
        f"FAIL <= {HEADING_SATELLITE_WARNING_ABOVE}"
    )
    if median_used > HEADING_SATELLITE_PASS_ABOVE:
        return base.Check("Heading satellites used", True, detail)
    if median_used > HEADING_SATELLITE_WARNING_ABOVE:
        check = base.Check("Heading satellites used", True, detail)
        check.warning = True
        return check
    return base.Check("Heading satellites used", False, detail)


_legacy_gps_checks = base.gps_checks
_legacy_service_checks = base.service_checks


def service_checks() -> list[base.Check]:
    return (
        _legacy_service_checks()
        + [heading_startup_configuration_check()]
        + network_path_checks()
    )


def gps_checks(*args: Any, **kwargs: Any) -> list[base.Check]:
    samples = args[0] if args else kwargs["samples"]
    checks = _legacy_gps_checks(*args, **kwargs)
    checks.append(heading_satellites_used_check(samples))
    return checks


def print_checks(checks: list[base.Check]) -> bool:
    width = max(len(check.name) for check in checks)
    for check in checks:
        if getattr(check, "warning", False):
            label = "WARN"
        else:
            label = "PASS" if check.passed else "FAIL"
        print(f"[{label}] {check.name:<{width}}  {check.detail}")
    return all(check.passed for check in checks)


# The legacy main function resolves these names from its module at runtime.
# Replace only the two extension points needed for this dated successor.
base.gps_checks = gps_checks
base.print_checks = print_checks
base.service_checks = service_checks


if __name__ == "__main__":
    raise SystemExit(base.main())
