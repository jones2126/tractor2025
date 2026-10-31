#!/usr/bin/env python3

import json
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]


def require(condition, message):
    if not condition:
        raise SystemExit("FAIL: " + message)


def main():
    package = json.loads((HERE / "package.json").read_text(encoding="utf-8"))
    require(package["approved_for_flash"] is False, "flash must remain disabled")
    require(package["approved_for_motion"] is False, "motion must remain disabled")
    require(package["installs_services"] is False, "service installation forbidden")
    require(package["enables_services"] is False, "service enablement forbidden")

    bridge = ROOT / "tractor_rpi/teensy_serial_bridge_rpi_lease_pause_20261031.py"
    server = ROOT / "tractor_rpi/testing/webrtc/wifi_rpi_lease_pause_only_20261031.py"
    page = ROOT / "tractor_rpi/testing/webrtc/wifi_rpi_lease_pause_only_20261031.html"
    test = ROOT / "tractor_rpi/testing/test_rpi_command_lease_pause_bridge_20261031.py"
    checklist = HERE / "FIELD_CHECKLIST_PHONE_MOTOR_OFF_20261031.txt"
    for path in (bridge, server, page, test, checklist):
        require(path.is_file(), f"missing {path.relative_to(ROOT)}")

    bridge_text = bridge.read_text(encoding="utf-8")
    server_text = server.read_text(encoding="utf-8")
    page_text = page.read_text(encoding="utf-8")
    checklist_text = checklist.read_text(encoding="utf-8")
    for forbidden in (
        "send_cmd_vel_to_teensy(",
        "send_wifi_drive_to_teensy(",
        "send_manual(",
        "send_auto(",
    ):
        require(forbidden not in bridge_text, f"motion call present: {forbidden}")
    require(
        "requested_mode=RpiCommandMode.PAUSE" in bridge_text,
        "bridge is not hard-coded to supervisor Pause",
    )
    require("mode != \"pause\"" in server_text, "phone wrapper lacks Pause gate")
    require("min_drive_percent=0" in server_text, "phone drive range is not zero")
    require("max_drive_percent=0" in server_text, "phone drive range is not zero")
    require("NRF" not in server_text, "phone wrapper must not depend on NRF")
    require("MOTOR-OFF TEST" in page_text, "phone page lacks motor-off banner")
    require("PAUSE ONLY" in page_text, "phone page lacks Pause-only banner")
    require('data-mode="manual"' not in page_text, "phone page exposes Manual")
    require('data-mode="auto"' not in page_text, "phone page exposes Auto")
    require('type="range"' not in page_text, "phone page exposes motion sliders")
    require("pause_session_accepted" in bridge_text, "stable Pause-session metric missing")
    require(
        "latest_pause_frame_acknowledged" in bridge_text,
        "latest-frame acknowledgement metric missing",
    )
    require(
        '"pause_claim_acknowledged"' not in bridge_text,
        "ambiguous Pause-claim metric remains",
    )
    require("platformio.rpi-command-lease.ini" in checklist_text, "candidate flash step missing")
    require("platformio.wifi-primary.ini" in checklist_text, "rollback step missing")
    require("MANDATORY SAME-SESSION" in checklist_text, "mandatory rollback hold missing")
    require("does not authorize candidate motion" in checklist_text, "motion prohibition missing")
    require(
        '"$TRACTOR_PHONE_LEASE_EVIDENCE_DIR/ownership.lock"' in checklist_text,
        "temporary ownership lock must use the operator-owned evidence directory",
    )
    require(
        "/run/tractor-phone-control" not in checklist_text,
        "interactive checklist must not require a systemd-created runtime directory",
    )

    print("PASS: disabled motor-off phone integration package verified.")
    print("      UDP bridge and phone wrapper are hard-gated to zero Pause.")
    print("      No service is installed or enabled by this package.")
    print("NOT FLASH AUTHORIZATION: approved_for_flash=false.")
    print("NOT MOTION AUTHORIZATION: approved_for_motion=false.")


if __name__ == "__main__":
    main()
