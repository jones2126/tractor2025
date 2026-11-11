#!/usr/bin/env python3
"""Phone control server whose only command output is the RPi arbitrator feed.

Unlike the motor-off comparison wrapper, this production candidate never
sends legacy UDP 6004 commands. The web UI, authentication, session handling,
Pause, E-stop, diagnostics, and ntfy behavior remain inherited.
"""

from __future__ import annotations

import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import wifi_rpi_arbitrator_control_20261031 as earlier


EXPECTED_FIRMWARE = "teensy_main_20261110_rpi_command_lease_v5"


class ArbitratorOnlyWifiState(earlier.ArbitratorWifiState):
    def send_drive_command(
        self,
        drive_percent: float,
        steering_percent: float,
        sequence: int,
        reason: str,
        phone_mode: str | None = None,
    ) -> None:
        # Intentionally do not call the legacy server's UDP 6004 sender.
        self.emit_arbitrator_intent(
            drive_percent,
            steering_percent,
            sequence,
            reason,
            phone_mode=phone_mode,
        )


def main() -> None:
    earlier.base.EXPECTED_FIRMWARE = os.environ.get(
        "TRACTOR_ARBITRATOR_EXPECTED_FIRMWARE", EXPECTED_FIRMWARE
    )
    earlier.base.WifiPrimaryState = ArbitratorOnlyWifiState
    earlier.base.main()


if __name__ == "__main__":
    main()
