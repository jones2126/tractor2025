#!/usr/bin/env python3
"""Fail-closed verifier for the disabled phone-arbitrator package."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
SOURCE = HERE.parent / "20260928_outer_perimeter_field_test"
ROUTE = SOURCE / "generated/62_Collins_outer_perimeter_field_test_1mps_20260928.txt"
EXPECTED_ROUTE = "b47660b4c74d5e26ca5d9448f069fd7193c068cf62e3c0b62d5c083d9aed4d64"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(f"FAIL: {message}")


def normalized_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def main() -> None:
    package = json.loads((HERE / "package.json").read_text(encoding="utf-8"))
    require(package["approved_for_install"] is True, "install approval missing")
    require(package["approved_for_flash"] is True, "flash approval missing")
    require(
        package["approved_for_candidate_service_start"] is True,
        "candidate service-start approval missing",
    )
    require(
        package["approved_for_motor_off_commissioning"] is True,
        "motor-off commissioning approval missing",
    )
    require(package["approved_for_motion"] is False, "motion gate must be false")
    require(package["blades_required_off"] is True, "blades-off gate missing")
    require(normalized_hash(ROUTE) == EXPECTED_ROUTE, "reviewed route hash mismatch")
    require(package["route_sha256"] == EXPECTED_ROUTE, "package route hash mismatch")

    bridge = (REPO / "tractor_rpi/teensy_serial_bridge_rpi_arbitrator_20261104.py").read_text(encoding="utf-8")
    phone = (REPO / "tractor_rpi/testing/webrtc/wifi_rpi_arbitrator_control_20261104.py").read_text(encoding="utf-8")
    firmware = (REPO / "tractor_teensy/src/teensy_main_20261104_rpi_command_lease.cpp").read_text(encoding="utf-8")
    launcher = (HERE / "run_outer_perimeter_20261104.sh").read_text(encoding="utf-8")
    installer = (HERE / "install_candidate_20261104.sh").read_text(encoding="utf-8")
    activator = (HERE / "activate_candidate_motor_off_20261104.sh").read_text(encoding="utf-8")
    bridge_unit = (HERE / "tractor-rpi-arbitrator.service").read_text(encoding="utf-8")
    phone_unit = (HERE / "tractor-phone-arbitrator.service").read_text(encoding="utf-8")

    require("MOTION_GATE = \"RUN_DIRECT_OUTER_PERIMETER_BLADES_OFF\"" in bridge, "bridge motion gate missing")
    require("phone_replaces_nrf_mode_switch=True" in bridge, "phone mode authority missing")
    require("UDP 6004" in phone and "do not call" in phone, "legacy phone output is not isolated")
    require("teensy_main_20261104_rpi_command_lease_v3" in firmware, "v3 firmware identity missing")
    require("APPROVED_FOR_FIELD=false" in launcher, "field launcher must remain disabled")
    require("APPROVED_FOR_INSTALL=true" in installer, "installer approval missing")
    require("systemctl enable" not in installer, "installer must not boot-enable services")
    require(
        "APPROVED_FOR_MOTOR_OFF_COMMISSIONING=true" in activator,
        "motor-off activation approval missing",
    )
    for confirmation in (
        "--engine-off-confirmed",
        "--blades-off-confirmed",
        "--rear-lever-out-confirmed",
        "--red-estop-ready-confirmed",
        "--wheels-chocked-confirmed",
    ):
        require(confirmation in activator, f"activation confirmation missing: {confirmation}")
    require("pgrep -f '[p]ure_pursuit'" in activator, "Pure Pursuit exclusion missing")
    require("--arbitrator-proposal-only" in launcher, "Pure Pursuit is not proposal-only")
    require("--port 16004" in launcher, "unused proposal destination missing")
    require("WantedBy=" not in bridge_unit + phone_unit, "candidate services must not boot")
    require(
        bridge_unit.count("WorkingDirectory=/home/al/tractor2025-rpi-lease") == 1,
        "bridge must use the exact candidate worktree",
    )
    require(
        "/home/al/tractor2025-rpi-lease/tractor_rpi/testing/webrtc/" in phone_unit,
        "phone service must use the exact candidate worktree",
    )
    require("Conflicts=teensy-bridge.service" in bridge_unit, "serial-owner conflict missing")
    require("Requires=tractor-rpi-arbitrator.service" in phone_unit, "phone/arbitrator ordering missing")

    print("PASS: motion-capable phone-arbitrator package verified.")
    print("      Exact outer-perimeter route; phone-only operator modes; v3 E-stop relay support.")
    print("      Pure Pursuit is proposal-only; one RPi arbitrator owns Teensy commands.")
    print("INSTALL, FLASH, CANDIDATE SERVICE START, AND MOTOR-OFF COMMISSIONING AUTHORIZED.")
    print("ENGINE START AND PHYSICAL MOTION ARE NOT AUTHORIZED.")


if __name__ == "__main__":
    main()
