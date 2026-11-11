#!/usr/bin/env python3
"""Fail-closed verifier for the approved phone-controlled field package."""

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
    require(package["production_boot_prepared"] is True, "production boot is not prepared")
    require(
        package["approved_for_production_boot_activation"] is True,
        "production boot activation approval missing",
    )
    require(package["approved_for_motion"] is True, "field-motion approval missing")
    require(package["production_reboot_test_passed"] is True, "production reboot evidence missing")
    require(package["motor_off_manual_test_passed"] is True, "Manual steering evidence missing")
    require(package["manual_repositioning_passed"] is True, "start-position evidence missing")
    require(package["blades_required_off"] is True, "blades-off gate missing")
    require(normalized_hash(ROUTE) == EXPECTED_ROUTE, "reviewed route hash mismatch")
    require(package["route_sha256"] == EXPECTED_ROUTE, "package route hash mismatch")

    bridge = (REPO / "tractor_rpi/teensy_serial_bridge_rpi_arbitrator_20261104.py").read_text(encoding="utf-8")
    phone = (REPO / "tractor_rpi/testing/webrtc/wifi_rpi_arbitrator_control_20261104.py").read_text(encoding="utf-8")
    firmware = (REPO / "tractor_teensy/src/teensy_main_20261110.cpp").read_text(encoding="utf-8")
    launcher = (HERE / "run_outer_perimeter_20261104.sh").read_text(encoding="utf-8")
    installer = (HERE / "install_candidate_20261104.sh").read_text(encoding="utf-8")
    boot_installer = (HERE / "install_production_boot_20261109.sh").read_text(encoding="utf-8")
    activator = (HERE / "activate_candidate_motor_off_20261104.sh").read_text(encoding="utf-8")
    production_dashboard = (HERE / "mission_dashboard_production_20261109.py").read_text(encoding="utf-8")
    production_preflight = (REPO / "tractor_rpi/testing/mission_preflight_phone_control_20261109.py").read_text(encoding="utf-8")
    bridge_unit = (HERE / "tractor-rpi-arbitrator.service").read_text(encoding="utf-8")
    phone_unit = (HERE / "tractor-phone-arbitrator.service").read_text(encoding="utf-8")

    require("MOTION_GATE = \"RUN_DIRECT_OUTER_PERIMETER_BLADES_OFF\"" in bridge, "bridge motion gate missing")
    require("phone_replaces_nrf_mode_switch=True" in bridge, "phone mode authority missing")
    require("UDP 6004" in phone and "do not call" in phone, "legacy phone output is not isolated")
    require("teensy_main_20261110_rpi_command_lease_v5" in firmware, "v5 firmware identity missing")
    require("APPROVED_FOR_FIELD=true" in launcher, "field launcher approval missing")
    require(
        "mission_preflight_phone_control_20261109.py" in launcher,
        "production phone-control preflight missing",
    )
    require(
        "RUN PHONE OUTER PERIMETER BLADES OFF" in launcher,
        "exact field confirmation missing",
    )
    require(
        'dashboard.EXPECTED_CONFIRMATION = "RUN PHONE OUTER PERIMETER BLADES OFF"'
        in production_dashboard,
        "dashboard confirmation does not match launcher",
    )
    for required in (
        "actual_serial_output",
        "phone_link_fresh",
        "scheduler_fault_latched",
        "additional_local_faults",
        "communications_degraded_preauthorized",
        "ROUTE_PROGRESS_ONLY",
    ):
        require(required in production_dashboard, f"dashboard safety gate missing: {required}")
    for service in (
        "rtcm-server.service",
        "tractor-rpi-arbitrator.service",
        "tractor-phone-arbitrator.service",
    ):
        require(service in production_preflight, f"production preflight service missing: {service}")
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
    require(
        bridge_unit.count("WantedBy=multi-user.target") == 1,
        "arbitrator boot target missing",
    )
    require(
        phone_unit.count("WantedBy=multi-user.target") == 1,
        "phone-service boot target missing",
    )
    require(
        "--local-control-ntfy-only" in phone_unit,
        "phone service must suppress ZeroTier startup notices",
    )
    require(
        "GL-SFT1200-9f4 (router 192.168.8.1)" in phone_unit,
        "operator-facing GL-router startup instructions are missing",
    )
    require(
        "APPROVED_FOR_PRODUCTION_BOOT_ACTIVATION=true" in boot_installer,
        "production boot installer approval missing",
    )
    require(
        "systemctl disable tractor-wifi-control.service teensy-bridge.service" in boot_installer,
        "legacy boot services are not explicitly disabled",
    )
    require(
        "systemctl enable tractor-rpi-arbitrator.service tractor-phone-arbitrator.service" in boot_installer,
        "v5 boot services are not explicitly enabled",
    )
    require(
        bridge_unit.count("WorkingDirectory=/home/al/tractor2025") == 1,
        "bridge must use the canonical repository checkout",
    )
    require(
        "/home/al/tractor2025/tractor_rpi/testing/webrtc/" in phone_unit,
        "phone service must use the canonical repository checkout",
    )
    require("Conflicts=teensy-bridge.service" in bridge_unit, "serial-owner conflict missing")
    require("Requires=tractor-rpi-arbitrator.service" in phone_unit, "phone/arbitrator ordering missing")

    print("PASS: motion-capable phone-arbitrator package verified.")
    print("      Exact outer-perimeter route; phone-only operator modes; v5 E-stop relay support.")
    print("      Pure Pursuit is proposal-only; one RPi arbitrator owns Teensy commands.")
    print("INSTALL, FLASH, CANDIDATE SERVICE START, AND MOTOR-OFF COMMISSIONING AUTHORIZED.")
    print("PRODUCTION BOOT ACTIVATION AUTHORIZED; REBOOT TEST PASSED.")
    print("FIELD MOTION AUTHORIZED: exact outer perimeter, 1.00 m/s, blades off.")
    print("Start through mission_dashboard_production_20261109.py only.")


if __name__ == "__main__":
    main()
