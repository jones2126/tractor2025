#!/usr/bin/env python3
"""Production preflight for the V3 phone-control service architecture."""

from __future__ import annotations

from pathlib import Path
import sys


REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from tractor_rpi.testing import mission_preflight_20261002 as profile


# The V3 RPi safety controller replaces the retired legacy serial bridge.
# Keep every GPS, heading, neutral-output, firmware, and network check from the
# reviewed 20261002 preflight while requiring the production service owners.
profile.base.SERVICES = (
    "rtcm-server.service",
    "tractor-rpi-arbitrator.service",
    "tractor-phone-arbitrator.service",
)


if __name__ == "__main__":
    raise SystemExit(profile.base.main())
