#!/usr/bin/env python3
"""Telemetry-only successor to teensy_serial_bridge_20260728.py.

Adds the four handheld raw values from tractor Teensy RADIO_INPUTS records to
the existing UDP 6003 radio object. All serial command, safety notification,
and source-synchronized broadcast behavior remains inherited unchanged.
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
        inputs = self.latest_data.get("RADIO_INPUTS")
        if inputs:
            radio = message.setdefault("radio", {})
            radio.update(
                {
                    "handheld_steering_raw": inputs.get("s"),
                    "handheld_additional_raw": inputs.get("t"),
                    "handheld_transmission_raw": inputs.get("x"),
                    "handheld_pot4_raw": inputs.get("p4"),
                    "handheld_inputs_signal_good": int(inputs.get("sg", 0)),
                    "handheld_inputs_radio_age_ms": inputs.get("a"),
                    "handheld_inputs_age": base.time.time()
                    - inputs.get("last_update", base.time.time()),
                }
            )
        return message


def main():
    try:
        bridge = TeensySerialBridge()
        bridge.run()
    except Exception as exc:
        base.logger.error("Fatal error: %s", exc, exc_info=True)


if __name__ == "__main__":
    main()
