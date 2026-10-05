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


if __name__ == "__main__":
    raise SystemExit(base.main())
