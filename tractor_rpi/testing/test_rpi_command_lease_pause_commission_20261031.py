#!/usr/bin/env python3

import unittest
from pathlib import Path

from tractor_rpi.testing.rpi_command_lease_pause_commission_20261031 import (
    CommissionEvidence,
    EXPECTED_FIRMWARE,
    build_parser,
    require_physical_confirmations,
)


def status_line(*, session=42, sequence=0, mode=0, fresh=1, usable=1, latched=0):
    return (
        f"1,100,RPI_CTL,m={mode},f={fresh},u={usable},l={latched},a=10,"
        f"ss={session},s={sequence},ok=1,rj=0,rr=0"
    )


class PauseOnlyCommissionTests(unittest.TestCase):
    def test_requires_every_physical_confirmation(self):
        parser = build_parser()
        base = ["--log-dir", "/tmp/evidence"]
        args = parser.parse_args(base)
        with self.assertRaises(RuntimeError):
            require_physical_confirmations(args)

        args = parser.parse_args(
            base
            + [
                "--engine-off-confirmed",
                "--blades-off-confirmed",
                "--rear-lever-out-confirmed",
                "--physical-pause-confirmed",
                "--wheels-chocked-confirmed",
            ]
        )
        require_physical_confirmations(args)

    def test_requires_identity_and_five_matching_pause_statuses(self):
        evidence = CommissionEvidence(session=42)
        evidence.process_line(f"1,0,SYS,start,fw={EXPECTED_FIRMWARE},steer_hz=20")
        for sequence in range(5):
            evidence.process_line(status_line(sequence=sequence))
        self.assertTrue(evidence.passed)

    def test_wrong_session_non_pause_or_expiry_latch_fails(self):
        for bad_line in (
            status_line(session=43),
            status_line(mode=2),
            status_line(usable=0, latched=1),
        ):
            with self.subTest(line=bad_line):
                evidence = CommissionEvidence(session=42, firmware_seen=True)
                for sequence in range(5):
                    evidence.process_line(status_line(sequence=sequence))
                evidence.process_line(bad_line)
                self.assertFalse(evidence.passed)

    def test_live_tool_contains_no_motion_send_call(self):
        source = Path(__file__).with_name(
            "rpi_command_lease_pause_commission_20261031.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("send_auto(", source)
        self.assertNotIn("send_manual(", source)
        self.assertIn("adapter.send_pause()", source)


if __name__ == "__main__":
    unittest.main()
