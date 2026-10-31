#!/usr/bin/env python3
"""Read-only verification for the disabled Approval B preparation package."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
SOURCE = HERE.parent / "20260928_outer_perimeter_field_test"


def require_text(path: Path, expected: str) -> None:
    text = path.read_text(encoding="utf-8")
    if expected not in text:
        raise SystemExit(f"FAIL: {path.relative_to(REPO)} lacks {expected!r}")


def main() -> None:
    result = subprocess.run(
        [sys.executable, str(SOURCE / "verify_outer_perimeter_field_20260928.py")],
        cwd=REPO,
        check=False,
    )
    if result.returncode:
        raise SystemExit("FAIL: frozen source route verification failed")

    project = json.loads((HERE / "approval_b_project.json").read_text())
    if project["status"] != "DRAFT - UNTESTED IN THE FIELD":
        raise SystemExit("FAIL: package status changed")
    if project["field_authorized"] or project["blades_authorized"]:
        raise SystemExit("FAIL: authorization gate is not closed")
    if project["communications_degraded_enabled"]:
        raise SystemExit("FAIL: COMMUNICATIONS DEGRADED must remain disabled")
    if project["phone_freshness_timeout_ms"] != 500:
        raise SystemExit("FAIL: phone freshness timeout changed")

    launcher = HERE / "run_approval_b_outer_perimeter_20261031.sh"
    require_text(launcher, "APPROVED_FOR_FIELD=false")
    require_text(launcher, "pure_pursuit_controller_20261022.py")
    require_text(launcher, "field_test_logger_20260828.py")
    require_text(launcher, "tractor-wifi-direct-mission.service")
    direct_unit = REPO / "tractor_rpi/setup/tractor-wifi-direct-mission.service"
    require_text(direct_unit, "--authority-policy onboard-router-mission")
    require_text(direct_unit, "--host 192.168.8.20")
    require_text(direct_unit, "# Deliberately has no boot target")
    require_text(
        REPO / "tractor_rpi/testing/webrtc/wifi_primary_control_20261022.py",
        '"onboard-router-mission"',
    )
    heading = HERE / "prepare_heading_and_preflight_20261031.sh"
    require_text(heading, "configure_dual_f9p_5hz_profile_20260923.py")
    require_text(heading, "systemctl is-active rtcm-server.service")
    require_text(heading, "mission_preflight_20261002.py")
    require_text(
        HERE / "collect_approval_b_evidence_20261031.sh",
        "find . -type f ! -name SHA256SUMS",
    )
    require_text(
        HERE / "run_approval_b_software_tests_20261030.sh",
        "test_communications_degraded_guard_20261029",
    )
    require_text(
        HERE / "mission_dashboard_approval_b_review_20261031.py",
        "Pure Pursuit start is disabled",
    )
    require_text(
        HERE / "start_approval_b_review_dashboard_20261031.sh",
        "Review dashboard only",
    )
    checklist = HERE / "FIELD_CHECKLIST_20261031.txt"
    require_text(checklist, "KNOWN REQUIREMENT BLOCKER")
    require_text(checklist, "COMMAND RULE: Every required software action")
    require_text(checklist, "run_approval_b_software_tests_20261030.sh")
    require_text(checklist, "test \"${PIPESTATUS[0]}\" -eq 0")
    require_text(checklist, "sudo systemctl start tractor-wifi-direct-mission.service")
    require_text(checklist, "The dashboard is the mission server")

    print("PASS: disabled Approval B package and immutable route verified.")
    print("      3,462 waypoints; 689.949 m; nominal 11.50 min at 1.00 m/s.")
    print("      Direct policy is isolated, non-booting, and bound to 192.168.8.20.")
    print("      COMMUNICATIONS DEGRADED is disabled; 500 ms freshness is unchanged.")
    print("NOT FIELD AUTHORIZATION: APPROVED_FOR_FIELD=false.")


if __name__ == "__main__":
    main()
