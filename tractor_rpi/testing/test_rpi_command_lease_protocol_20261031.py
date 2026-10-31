#!/usr/bin/env python3

import unittest

from tractor_rpi.rpi_command_lease_protocol_20261031 import (
    CommandLeaseReference,
    MAX_SEQUENCE,
    RpiCommand,
    RpiCommandMode,
    decode_frame,
    encode_frame,
    sequence_is_newer,
)


class RpiCommandLeaseProtocolTests(unittest.TestCase):
    def test_all_modes_round_trip(self):
        commands = (
            RpiCommand(1, RpiCommandMode.PAUSE, 0.0, 0.0),
            RpiCommand(2, RpiCommandMode.MANUAL_PERCENT, -25.0, 0.5),
            RpiCommand(3, RpiCommandMode.AUTO_MPS, 1.0, -0.25),
        )
        for command in commands:
            with self.subTest(mode=command.mode):
                self.assertEqual(command, decode_frame(encode_frame(command)))

    def test_invalid_values_are_rejected_before_encoding(self):
        invalid = (
            RpiCommand(1, RpiCommandMode.PAUSE, 1.0, 0.0),
            RpiCommand(1, RpiCommandMode.MANUAL_PERCENT, 101.0, 0.0),
            RpiCommand(1, RpiCommandMode.AUTO_MPS, -0.1, 0.0),
            RpiCommand(1, RpiCommandMode.AUTO_MPS, 1.0, 1.1),
        )
        for command in invalid:
            with self.subTest(command=command):
                with self.assertRaises(ValueError):
                    encode_frame(command)

    def test_corrupt_malformed_and_non_ascii_frames_do_not_refresh_lease(self):
        receiver = CommandLeaseReference()
        good = encode_frame(RpiCommand(10, RpiCommandMode.AUTO_MPS, 1.0, 0.0))
        self.assertTrue(receiver.accept(good, now_ms=1000))
        corrupt = good.replace(b"1.0000", b"1.1000")
        self.assertFalse(receiver.accept(corrupt, now_ms=1400))
        self.assertFalse(receiver.accept(b"RPICMD,bad\n", now_ms=1450))
        self.assertFalse(receiver.accept(b"\xff\n", now_ms=1490))
        self.assertFalse(receiver.decide(now_ms=1500.001).lease_fresh)

    def test_duplicate_and_reordered_frames_do_not_refresh_lease(self):
        receiver = CommandLeaseReference()
        newest = encode_frame(RpiCommand(20, RpiCommandMode.AUTO_MPS, 1.0, 0.0))
        older = encode_frame(RpiCommand(19, RpiCommandMode.AUTO_MPS, 1.0, 0.0))
        self.assertTrue(receiver.accept(newest, now_ms=1000))
        self.assertFalse(receiver.accept(newest, now_ms=1400))
        self.assertFalse(receiver.accept(older, now_ms=1450))
        self.assertFalse(receiver.decide(now_ms=1501).lease_fresh)

    def test_sequence_wrap_is_forward_but_replay_is_not(self):
        self.assertTrue(sequence_is_newer(0, MAX_SEQUENCE))
        self.assertFalse(sequence_is_newer(MAX_SEQUENCE, 0))
        self.assertFalse(sequence_is_newer(10, 10))

    def test_lease_cannot_be_configured_above_500_ms(self):
        with self.assertRaises(ValueError):
            CommandLeaseReference(timeout_ms=501)

    def test_exact_500_ms_is_fresh_and_first_crossing_pauses(self):
        receiver = CommandLeaseReference()
        frame = encode_frame(RpiCommand(1, RpiCommandMode.AUTO_MPS, 1.0, 0.0))
        self.assertTrue(receiver.accept(frame, now_ms=1000))
        self.assertTrue(receiver.decide(now_ms=1500).lease_fresh)
        expired = receiver.decide(now_ms=1500.001)
        self.assertFalse(expired.lease_fresh)
        self.assertEqual(RpiCommandMode.PAUSE, expired.command.mode)
        self.assertEqual(0.0, expired.command.drive)

    def test_every_local_teensy_safety_input_pauses_immediately(self):
        for fault in (
            "physical_pause_active",
            "physical_estop_active",
            "steering_fault_latched",
            "transmission_fault_latched",
        ):
            with self.subTest(fault=fault):
                receiver = CommandLeaseReference()
                frame = encode_frame(
                    RpiCommand(1, RpiCommandMode.MANUAL_PERCENT, 25.0, 0.25)
                )
                self.assertTrue(receiver.accept(frame, now_ms=1000))
                decision = receiver.decide(now_ms=1001, **{fault: True})
                self.assertEqual(RpiCommandMode.PAUSE, decision.command.mode)
                self.assertEqual(0.0, decision.command.drive)


if __name__ == "__main__":
    unittest.main()
