#!/usr/bin/env python3

import json
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parent
REPO_ROOT = PACKAGE_DIR.parents[4]


def require(condition, message):
    if not condition:
        raise SystemExit("FAIL: " + message)


def main():
    package = json.loads((PACKAGE_DIR / "package.json").read_text(encoding="utf-8"))
    require(package["approved_for_flash"] is False, "flash approval must remain false")
    require(package["approved_for_motion"] is False, "motion approval must remain false")

    candidate = REPO_ROOT / "tractor_teensy/src/teensy_main_20261031_rpi_command_lease.cpp"
    candidate_build = REPO_ROOT / "tractor_teensy/platformio.rpi-command-lease.ini"
    rollback_build = REPO_ROOT / "tractor_teensy/platformio.wifi-primary.ini"
    pause_tool = REPO_ROOT / "tractor_rpi/testing/rpi_command_lease_pause_commission_20261031.py"
    prepare = PACKAGE_DIR / "prepare_motor_off_artifacts_20261031.sh"
    checklist = PACKAGE_DIR / "FIELD_CHECKLIST_MOTOR_OFF_20261031.txt"
    for path in (candidate, candidate_build, rollback_build, pause_tool, prepare, checklist):
        require(path.is_file(), f"missing {path.relative_to(REPO_ROOT)}")

    candidate_text = candidate.read_text(encoding="utf-8")
    candidate_build_text = candidate_build.read_text(encoding="utf-8")
    rollback_build_text = rollback_build.read_text(encoding="utf-8")
    pause_tool_text = pause_tool.read_text(encoding="utf-8")
    prepare_text = prepare.read_text(encoding="utf-8")

    require(
        'TRACTOR_FIRMWARE_ID "teensy_main_20261031_rpi_command_lease_v2"' in candidate_text,
        "candidate identity mismatch",
    )
    require("RPI_COMMAND_LEASE_MS <= 500UL" in candidate_text, "500 ms cap missing")
    require("rpiLeaseExpiredLatched" in candidate_text, "expiry latch missing")
    require("sessionChanged && parsedMode != RPI_MODE_PAUSE" in candidate_text, "Pause claim gate missing")
    session_gate = candidate_text.split("const bool sessionChanged", 1)[1].split(
        "if (!sessionChanged", 1
    )[0]
    require("radioStats" not in session_gate, "obsolete NRF dependency in session gate")
    require("radioData" not in session_gate, "obsolete NRF mode dependency in session gate")
    require(
        "teensy_main_20261031_rpi_command_lease.cpp" in candidate_build_text,
        "candidate build selects wrong source",
    )
    require(
        "teensy_main_20261003_wifi.cpp" in rollback_build_text,
        "rollback build selects wrong source",
    )
    require("send_auto(" not in pause_tool_text, "Pause tool contains Auto send")
    require("send_manual(" not in pause_tool_text, "Pause tool contains Manual send")
    require("adapter.send_pause()" in pause_tool_text, "Pause tool does not send Pause")
    require(
        'EXPECTED_FIRMWARE = "teensy_main_20261031_rpi_command_lease_v2"'
        in pause_tool_text,
        "Pause tool expects wrong candidate identity",
    )
    require(
        "physical-pause-confirmed" not in pause_tool_text,
        "obsolete NRF/physical handheld confirmation remains",
    )
    require(
        "teensy_main_20261031_rpi_command_lease_v2_CANDIDATE.hex" in prepare_text,
        "artifact preparation uses wrong candidate identity",
    )
    require(
        "teensy_main_20261031_rpi_command_lease_v1_CANDIDATE.hex" not in prepare_text,
        "obsolete v1 artifact name remains",
    )
    require("-t upload" not in prepare_text, "preparation script must not upload")

    print("PASS: disabled motor-off command-lease package verified.")
    print("      Candidate and rollback builds are separate and explicit.")
    print("      Pause-only tool contains no Manual or Auto send call.")
    print("NOT FLASH AUTHORIZATION: approved_for_flash=false.")
    print("NOT MOTION AUTHORIZATION: approved_for_motion=false.")


if __name__ == "__main__":
    main()
