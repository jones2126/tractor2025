#!/usr/bin/env python3
"""Prevent RPi command-lease work from changing proven field paths."""

from __future__ import annotations

import hashlib
import subprocess
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
BASE_REVISION = "3e0b012cb470542735431a6adb4ba9a78bbd6659"

FROZEN_FILES = {
    "tractor_teensy/src/archive/teensy_main_20260804.cpp":
        "9588bdb5d37954c6aac610d50440193349dae637e6a9a6c1097fe9783b450b76",
    "tractor_teensy/src/teensy_main_20260914.cpp":
        "9809a2a5a749cfa843551bdf0d08e8d52077656184665694ac9ae6c53085b801",
    "tractor_teensy/src/teensy_main_20261003_wifi.cpp":
        "0fcfed36f8f293f4864f13e047dab54def0f6c871b597321de8414e85cd61ee2",
    "tractor_rpi/teensy_serial_bridge_20260728.py":
        "0a6ddb487689f6fbcbec23823fd5ee95b2e771cd3800507dcad9640ba555882e",
    "tractor_rpi/pure-pursuit/pure_pursuit_controller_20261022.py":
        "f4f43b24c0b8c7529c10a8335d829d4ae22ec00ed01b56ba074a7f4b8c54c58b",
    "tractor_rpi/testing/webrtc/wifi_primary_control_20261022.py":
        "9b3f7b1bb8b65dd4fafbd32ed645126ca5a1ec4f30d4b1a55878465b085eab15",
    "tractor_rpi/communications_degraded_guard_20261029.py":
        "b89aae435f5ac8592770e82d92d2239839dd6890ee4c7f1455d4b352afa67d1d",
}

FROZEN_EXACT_PATHS = {
    *FROZEN_FILES,
    "tractor_teensy/platformio.ini",
    "tractor_teensy/platformio.wifi-primary.ini",
}

FROZEN_PREFIXES = (
    "field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/"
    "20261031_APPROVAL_B_DIRECT_OUTER_PERIMETER_DRAFT_UNTESTED/",
    "tractor_rpi/setup/",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class RpiCommandLeaseChangeBoundaryTests(unittest.TestCase):
    def test_frozen_reference_files_match_field_baseline(self):
        for relative_path, expected in FROZEN_FILES.items():
            with self.subTest(path=relative_path):
                self.assertEqual(expected, sha256(REPO_ROOT / relative_path))

    def test_branch_diff_does_not_touch_frozen_paths(self):
        result = subprocess.run(
            [
                "git",
                "diff",
                "--name-only",
                BASE_REVISION,
                "--",
            ],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        changed = {
            line.strip().replace("\\", "/")
            for line in result.stdout.splitlines()
            if line.strip()
        }
        violations = sorted(
            path
            for path in changed
            if path in FROZEN_EXACT_PATHS
            or any(path.startswith(prefix) for prefix in FROZEN_PREFIXES)
        )
        self.assertEqual([], violations, f"frozen paths changed: {violations}")


if __name__ == "__main__":
    unittest.main()
