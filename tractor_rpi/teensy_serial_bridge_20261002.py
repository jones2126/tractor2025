#!/usr/bin/env python3
"""Telemetry compatibility layer for the RPi-command-lease firmware.

The inherited 20260728 parser remains the stable implementation for STEER,
TRANS, JRK, SYS, and GPS records. NRF24 RADIO/RADIO_INPUTS records were retired
from Tractor01 firmware on 2026-11-10, so this layer removes the obsolete radio
object and raw handheld field from UDP 6003 broadcasts.
"""

from __future__ import annotations

import sys
from pathlib import Path


SCRIPT_DIR = str(Path(__file__).resolve().parent)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import teensy_serial_bridge_20260728 as base


class TeensySerialBridge(base.TeensySerialBridge):
    def create_broadcast_message(self):
        message = super().create_broadcast_message()
        message.pop("radio", None)
        message.get("transmission", {}).pop("radio_transmission_raw", None)
        message.setdefault("system", {})["command_authority"] = "rpi_command_lease"
        return message


def main():
    try:
        bridge = TeensySerialBridge()
        bridge.run()
    except Exception as exc:
        base.logger.error("Fatal error: %s", exc, exc_info=True)


if __name__ == "__main__":
    main()
