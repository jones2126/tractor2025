#!/usr/bin/env python3
"""Read-only preparation checks for the two-mission 62 Collins field day."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
MASTER = REPO / "field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test"
PERIMETER = REPO / "field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test"


def run(label: str, *command: str) -> None:
    print(f"\n--- {label} ---")
    result = subprocess.run(command, cwd=REPO, text=True)
    if result.returncode:
        raise SystemExit(f"FAIL: {label}")


def require_text(path: Path, expected: str) -> None:
    if not path.is_file():
        raise SystemExit(f"FAIL: missing {path.relative_to(REPO)}")
    if expected not in path.read_text(encoding="utf-8"):
        raise SystemExit(f"FAIL: {path.relative_to(REPO)} does not contain {expected!r}")


def main() -> None:
    run(
        "Mission 1 immutable route",
        sys.executable,
        str(MASTER / "verify_master_manual_phone_20261006.py"),
    )
    run(
        "Mission 2 approved perimeter package",
        sys.executable,
        str(PERIMETER / "verify_consolidated_perimeter_20260929.py"),
    )
    run(
        "Phone authority, stale-dashboard, and Manual Recovery tests",
        sys.executable,
        "-m",
        "unittest",
        "tractor_rpi.testing.test_wifi_primary_control_20261003",
        "tractor_rpi.testing.test_mission_dashboard_stale_state_20261019",
        "tractor_rpi.testing.test_manual_recovery_20261019",
        "tractor_rpi.testing.test_phone_control_log_analyzer_20261021",
    )
    require_text(
        REPO / "tractor_rpi/setup/tractor-wifi-control.service",
        "--authority-policy zerotier-primary",
    )
    require_text(
        REPO / "tractor_rpi/setup/tractor-wifi-recovery.service",
        "--authority-policy onboard-router-recovery",
    )
    require_text(
        REPO / "tractor_rpi/setup/tractor-wifi-recovery.service",
        "# Deliberately has no boot target",
    )
    print("\nPASS: both immutable mission packages and the prepared control software verified.")
    print("NOT FIELD AUTHORIZATION: Tractor01 stationary, recovery, network-soak, and fresh preflight gates remain required.")


if __name__ == "__main__":
    main()
