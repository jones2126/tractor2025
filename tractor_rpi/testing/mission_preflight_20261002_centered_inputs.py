#!/usr/bin/env python3
"""Dated preflight requiring centered tractor and handheld raw inputs.

Extends mission_preflight_20261002.py. Checks are evaluated only from fresh
UDP 6003 telemetry while the handheld radio link is good and Pause is active.
"""

from __future__ import annotations

import statistics
import sys
from pathlib import Path
from typing import Any


SCRIPT_DIR = str(Path(__file__).resolve().parent)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import mission_preflight_20261002 as dated


base = dated.base
HANDHELD_CENTER = 500
HANDHELD_TOLERANCE = 100
TRACTOR_STEERING_CENTER = 525
TRACTOR_STEERING_TOLERANCE = 50
MAX_INPUT_AGE_SECONDS = 1.0

HANDHELD_FIELDS = (
    ("Handheld steering raw (pin 16)", "handheld_steering_raw"),
    ("Handheld additional raw (pin 14)", "handheld_additional_raw"),
    ("Handheld transmission raw (pin 15)", "handheld_transmission_raw"),
    ("Handheld pot4 raw (pin 17)", "handheld_pot4_raw"),
)


def centered_value_check(
    name: str,
    values: list[float],
    target: float,
    tolerance: float,
    total_samples: int,
) -> base.Check:
    required = max(1, int(total_samples * base.REQUIRED_DATA_FRACTION))
    if len(values) < required:
        return base.Check(
            name,
            False,
            f"valid fresh samples={len(values)}/{total_samples}; required >= {required}",
        )
    median_value = statistics.median(values)
    low, high = target - tolerance, target + tolerance
    return base.Check(
        name,
        low <= median_value <= high,
        (
            f"samples={len(values)}; latest={values[-1]:g}; median={median_value:g}; "
            f"range={min(values):g}-{max(values):g}; expected={target:g} +/- {tolerance:g} "
            f"({low:g}-{high:g})"
        ),
    )


def centered_input_checks(
    samples: list[tuple[float, dict[str, Any]]],
) -> list[base.Check]:
    if not samples:
        return [base.Check("Centered raw inputs", False, "no UDP 6003 samples")]

    latest = samples[-1][1]
    steering = latest.get("steering", {})
    transmission = latest.get("transmission", {})
    paused = (
        steering.get("mode") == 2
        and transmission.get("mode") == 2
        and steering.get("state") == "PAUSE"
    )
    if not paused:
        return [
            base.Check(
                "Centered raw inputs",
                False,
                "handheld must be connected and in Pause before centered inputs are evaluated",
            )
        ]

    checks = []
    tractor_values = [
        float(message.get("steering", {}).get("current"))
        for _, message in samples
        if base.finite_number(message.get("steering", {}).get("current"))
    ]
    checks.append(
        centered_value_check(
            "Tractor steering sensor raw",
            tractor_values,
            TRACTOR_STEERING_CENTER,
            TRACTOR_STEERING_TOLERANCE,
            len(samples),
        )
    )

    for label, field in HANDHELD_FIELDS:
        values = []
        for _, message in samples:
            radio = message.get("radio", {})
            fresh = (
                radio.get("handheld_inputs_signal_good") == 1
                and base.finite_number(radio.get("handheld_inputs_age"))
                and float(radio["handheld_inputs_age"]) <= MAX_INPUT_AGE_SECONDS
            )
            if fresh and base.finite_number(radio.get(field)):
                values.append(float(radio[field]))
        checks.append(
            centered_value_check(
                label,
                values,
                HANDHELD_CENTER,
                HANDHELD_TOLERANCE,
                len(samples),
            )
        )
    return checks


_previous_steering_checks = base.steering_checks


def steering_checks(*args: Any, **kwargs: Any) -> list[base.Check]:
    samples = args[0] if args else kwargs["samples"]
    checks = _previous_steering_checks(*args, **kwargs)
    checks.extend(centered_input_checks(samples))
    return checks


base.steering_checks = steering_checks


if __name__ == "__main__":
    raise SystemExit(base.main())
