#!/usr/bin/env python3
"""Phone-server wrapper for motor-off RPi command-lease integration.

The ordinary authenticated HTTPS/session implementation is reused, but the
expected firmware is the v2 lease candidate and every non-Pause command is
rejected.  This file is not referenced by any installed service.
"""

from __future__ import annotations

from http import HTTPStatus
from pathlib import Path
from typing import Any

import wifi_primary_control_20261022 as base


EXPECTED_FIRMWARE = "teensy_main_20261031_rpi_command_lease_v2"
POLICY_NAME = "rpi-lease-pause-only"
PAGE_PATH = Path(__file__).with_name("wifi_rpi_lease_pause_only_20261031.html")


class PauseOnlyWifiState(base.WifiPrimaryState):
    def snapshot(self) -> dict[str, Any]:
        result = super().snapshot()
        bridge, bridge_age = self.bridge_snapshot()
        result["rpi_command_lease"] = bridge.get("rpi_command_lease", {})
        result["rpi_command_lease_bridge_age_s"] = (
            None if not base.math.isfinite(bridge_age) else round(bridge_age, 3)
        )
        result["pause_only"] = True
        return result

    def accept_command(
        self, data: Any, client_ip: str | None = None
    ) -> tuple[HTTPStatus, dict[str, Any]]:
        if not isinstance(data, dict):
            return HTTPStatus.BAD_REQUEST, {
                "accepted": False,
                "error": "JSON object required",
            }
        try:
            mode = str(data.get("mode", "")).lower()
            drive = float(data.get("drive_percent", 0.0))
            steering = float(data.get("steering_percent", 0.0))
        except (TypeError, ValueError):
            mode, drive, steering = "invalid", 1.0, 1.0
        if mode != "pause" or drive != 0.0 or steering != 0.0:
            with self.lock:
                self.rejected += 1
                self.last_rejection_reason = (
                    "motor-off command-lease integration accepts only zero Pause"
                )
                self.latch_pause(self.last_rejection_reason)
            self.send_stop_burst("pause_only_rejected_motion")
            return HTTPStatus.CONFLICT, {
                "accepted": False,
                "error": self.last_rejection_reason,
                "phone_mode": "pause",
                "effective_state": "pause_latched",
            }
        return super().accept_command(data, client_ip)


def main() -> None:
    base.EXPECTED_FIRMWARE = EXPECTED_FIRMWARE
    base.PAGE_PATH = PAGE_PATH
    base.WifiPrimaryState = PauseOnlyWifiState
    base.AUTHORITY_POLICIES[POLICY_NAME] = base.AuthorityPolicy(
        POLICY_NAME,
        base.ipaddress.ip_network("192.168.8.0/24"),
        False,
        "motor-off Pause-only RPi command-lease integration",
        min_drive_percent=0,
        max_drive_percent=0,
        recovery_only=False,
        interface_name="eth0",
        numeric_host="192.168.8.20",
        expected_ssid="GL-SFT1200-9f4",
    )
    base.main()


if __name__ == "__main__":
    main()
