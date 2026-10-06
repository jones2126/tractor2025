#!/usr/bin/env python3
"""Verify the immutable September 23 route used by the phone-control rerun."""

from __future__ import annotations

import hashlib
import runpy
from pathlib import Path


HERE = Path(__file__).resolve().parent
SOURCE_PACKAGE = HERE.parent / "20260915_master_boundary_replay"
MISSION = (
    SOURCE_PACKAGE
    / "generated_master_manual_field_20260920"
    / "62_Collins_master_manual_resampled_1mps_20260920.txt"
)
SOURCE_VERIFY = SOURCE_PACKAGE / "verify_master_manual_field_20260920.py"
EXPECTED_SHA256 = "0276fa22f2c7def0a516b2dcd5516bfd3b05fa647f3b50607a6ca5ec218436b4"


def main() -> None:
    if not MISSION.is_file():
        raise SystemExit(f"ERROR: mission file not found: {MISSION}")
    digest = hashlib.sha256(MISSION.read_bytes()).hexdigest()
    if digest != EXPECTED_SHA256:
        raise SystemExit(
            "ERROR: September 23 mission checksum mismatch; "
            f"expected {EXPECTED_SHA256}, got {digest}"
        )
    if not SOURCE_VERIFY.is_file():
        raise SystemExit(f"ERROR: source verifier not found: {SOURCE_VERIFY}")
    runpy.run_path(str(SOURCE_VERIFY), run_name="__main__")
    print("PASS: phone-control rerun points to the immutable September 23 mission.")


if __name__ == "__main__":
    main()
