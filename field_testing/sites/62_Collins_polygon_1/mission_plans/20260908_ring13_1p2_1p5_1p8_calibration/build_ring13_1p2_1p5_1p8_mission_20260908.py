#!/usr/bin/env python3
"""Build the 2026-09-08 Ring 13 1.2/1.5/1.8 m/s calibration mission.

The validated 2026-08-31 four-lap mission is the immutable geometry source.
This builder keeps the 66-row transit and the first three Ring 13 laps,
changes only column 5 (commanded speed), and drops the historical fourth lap.
"""

from __future__ import annotations

import hashlib
from collections import Counter
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SOURCE = (
    SCRIPT_DIR.parent
    / "20260831_speed_and_skip_turn_tests"
    / "01_speed_settings"
    / "62_Collins_ring13_four_speed_settings_REVIEW_TEST_20260831.txt"
)
OUTPUT = SCRIPT_DIR / "62_Collins_ring13_1p2_1p5_1p8_settings_20260908.txt"

EXPECTED_SOURCE_SHA256 = "7546efab90c6d6a1045342ed9a3b30ec306a82e404c68d1314bc3f31c907fa60"
EXPECTED_SOURCE_ROWS = 653
EXPECTED_OUTPUT_ROWS = 506

# 1-based inclusive row ranges in the historical mission.
SEGMENTS = (
    (1, 66, "0.75", "transit"),
    (67, 212, "1.20", "lap 1"),
    (213, 359, "1.50", "lap 2"),
    (360, 506, "1.80", "lap 3"),
)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def speed_for_row(row_number: int) -> str:
    for start, end, speed, _label in SEGMENTS:
        if start <= row_number <= end:
            return speed
    raise ValueError(f"row {row_number} is outside the new mission")


def main() -> None:
    if not SOURCE.is_file():
        raise SystemExit(f"ERROR: source mission not found: {SOURCE}")

    source_bytes = SOURCE.read_bytes()
    source_sha = sha256_bytes(source_bytes)
    if source_sha != EXPECTED_SOURCE_SHA256:
        raise SystemExit(
            "ERROR: historical source mission SHA-256 changed.\n"
            f"Expected: {EXPECTED_SOURCE_SHA256}\n"
            f"Actual  : {source_sha}"
        )

    source_lines = SOURCE.read_text(encoding="utf-8").splitlines()
    if len(source_lines) != EXPECTED_SOURCE_ROWS:
        raise SystemExit(
            f"ERROR: expected {EXPECTED_SOURCE_ROWS} source rows, got {len(source_lines)}"
        )

    output_lines: list[str] = []
    speeds: Counter[str] = Counter()

    for row_number, line in enumerate(source_lines[:EXPECTED_OUTPUT_ROWS], start=1):
        parts = line.split()
        if len(parts) != 5:
            raise SystemExit(
                f"ERROR: source row {row_number} has {len(parts)} columns; expected 5"
            )

        # Preserve latitude, longitude, heading and lookahead exactly as strings.
        parts[4] = speed_for_row(row_number)
        speeds[parts[4]] += 1
        output_lines.append(" ".join(parts))

    if len(output_lines) != EXPECTED_OUTPUT_ROWS:
        raise SystemExit(
            f"ERROR: expected {EXPECTED_OUTPUT_ROWS} output rows, got {len(output_lines)}"
        )

    expected_counts = Counter({"0.75": 66, "1.20": 146, "1.50": 147, "1.80": 147})
    if speeds != expected_counts:
        raise SystemExit(f"ERROR: unexpected speed-row counts: {dict(speeds)}")

    # Geometry/lookahead guard: the first four fields must be byte-for-byte
    # equivalent to the corresponding historical source fields.
    for row_number, (source_line, output_line) in enumerate(
        zip(source_lines[:EXPECTED_OUTPUT_ROWS], output_lines), start=1
    ):
        if source_line.split()[:4] != output_line.split()[:4]:
            raise SystemExit(f"ERROR: geometry changed at row {row_number}")

    # Historical mission uses 1.50 m transit lookahead and 2.00 m Ring 13 lookahead.
    if any(line.split()[3] != "1.50" for line in output_lines[:66]):
        raise SystemExit("ERROR: transit lookahead is not uniformly 1.50 m")
    if any(line.split()[3] != "2.00" for line in output_lines[66:]):
        raise SystemExit("ERROR: Ring 13 lookahead is not uniformly 2.00 m")

    output_text = "\n".join(output_lines) + "\n"
    OUTPUT.write_text(output_text, encoding="utf-8", newline="\n")
    output_sha = sha256_bytes(output_text.encode("utf-8"))

    print(f"Created: {OUTPUT}")
    print(f"Rows   : {EXPECTED_OUTPUT_ROWS}")
    print("Speeds : transit 0.75; lap1 1.20; lap2 1.50; lap3 1.80 m/s")
    print(f"SHA256 : {output_sha}")


if __name__ == "__main__":
    main()
