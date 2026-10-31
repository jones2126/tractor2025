#!/usr/bin/env python3

import unittest

from tractor_rpi.rpi_command_lease_adapter_20261031 import (
    RpiCommandLeaseAdapter,
    parse_rpi_ctl_line,
)
from tractor_rpi.rpi_command_lease_protocol_20261031 import (
    RpiCommand,
    RpiCommandMode,
    decode_frame,
    encode_frame,
)
from tractor_rpi.testing.rpi_command_lease_serial_fault_simulator_20261031 import (
    RpiCommandLeaseSerialFaultSimulator,
    SerialFault,
)


class FakeSerial:
    def __init__(self, short_write=False):
        self.frames = []
        self.flushes = 0
        self.short_write = short_write

    def write(self, data):
        self.frames.append(data)
        return len(data) - 1 if self.short_write else len(data)

    def flush(self):
        self.flushes += 1


def status_line(
    *, session=42, sequence=0, mode=0, fresh=1, usable=1, latched=0
):
    return (
        f"1,100,RPI_CTL,m={mode},f={fresh},u={usable},l={latched},a=10,"
        f"ss={session},s={sequence},ok=1,rj=0,rr=0"
    )


class RpiCommandLeaseAdapterTests(unittest.TestCase):
    def test_motion_is_blocked_until_matching_pause_claim_ack(self):
        sink = FakeSerial()
        adapter = RpiCommandLeaseAdapter(sink, session=42)
        claim = adapter.send_pause()
        self.assertEqual(RpiCommandMode.PAUSE, decode_frame(claim).mode)
        with self.assertRaises(RuntimeError):
            adapter.send_auto(1.0, 0.0)
        self.assertTrue(adapter.observe_status(parse_rpi_ctl_line(status_line())))
        motion = decode_frame(adapter.send_auto(1.0, 0.25))
        self.assertEqual(RpiCommandMode.AUTO_MPS, motion.mode)
        self.assertEqual(1, motion.sequence)

        adapter.send_pause()
        with self.assertRaises(RuntimeError):
            adapter.send_auto(1.0, 0.0)
        self.assertTrue(
            adapter.observe_status(parse_rpi_ctl_line(status_line(sequence=2)))
        )

    def test_wrong_session_or_expiry_latch_revokes_motion_authority(self):
        adapter = RpiCommandLeaseAdapter(FakeSerial(), session=42)
        adapter.send_pause()
        self.assertTrue(adapter.observe_status(parse_rpi_ctl_line(status_line())))
        self.assertFalse(
            adapter.observe_status(parse_rpi_ctl_line(status_line(session=43)))
        )
        with self.assertRaises(RuntimeError):
            adapter.send_manual(0.0, 0.0)

        adapter.send_pause()
        self.assertFalse(
            adapter.observe_status(
                parse_rpi_ctl_line(status_line(sequence=1, latched=1, usable=0))
            )
        )

    def test_short_serial_write_fails_closed(self):
        adapter = RpiCommandLeaseAdapter(FakeSerial(short_write=True), session=42)
        with self.assertRaises(OSError):
            adapter.send_pause()

    def test_rpi_ctl_parser_rejects_schema_or_boolean_changes(self):
        with self.assertRaises(ValueError):
            parse_rpi_ctl_line(status_line().replace(",rr=0", ""))
        with self.assertRaises(ValueError):
            parse_rpi_ctl_line(status_line().replace("f=1", "f=2"))


class RpiCommandLeaseSerialFaultTests(unittest.TestCase):
    def test_drop_causes_expiry_at_first_crossing_over_500_ms(self):
        simulator = RpiCommandLeaseSerialFaultSimulator()
        claim = encode_frame(RpiCommand(42, 0, RpiCommandMode.PAUSE, 0.0, 0.0))
        motion = encode_frame(RpiCommand(42, 1, RpiCommandMode.AUTO_MPS, 1.0, 0.0))
        simulator.transmit(claim, sent_ms=0, physical_pause_active=True)
        simulator.transmit(motion, sent_ms=1)
        simulator.transmit(motion, sent_ms=100, fault=SerialFault.DROP)
        simulator.deliver_until(501)
        self.assertTrue(simulator.receiver.decide(now_ms=501).lease_fresh)
        self.assertFalse(simulator.receiver.decide(now_ms=501.001).lease_fresh)

    def test_corruption_and_duplicate_do_not_extend_lease(self):
        simulator = RpiCommandLeaseSerialFaultSimulator()
        claim = encode_frame(RpiCommand(42, 0, RpiCommandMode.PAUSE, 0.0, 0.0))
        motion = encode_frame(RpiCommand(42, 1, RpiCommandMode.AUTO_MPS, 1.0, 0.0))
        next_motion = encode_frame(
            RpiCommand(42, 2, RpiCommandMode.AUTO_MPS, 1.0, 0.0)
        )
        simulator.transmit(claim, sent_ms=0, physical_pause_active=True)
        simulator.transmit(motion, sent_ms=1)
        simulator.transmit(next_motion, sent_ms=200, fault=SerialFault.CORRUPT)
        simulator.transmit(motion, sent_ms=300, fault=SerialFault.DUPLICATE)
        simulator.deliver_until(502)
        self.assertGreaterEqual(simulator.rejected, 3)
        self.assertFalse(simulator.receiver.decide(now_ms=502).lease_fresh)

    def test_delayed_motion_cannot_resume_until_pause_rearm(self):
        simulator = RpiCommandLeaseSerialFaultSimulator()
        frames = {
            "claim": encode_frame(
                RpiCommand(42, 0, RpiCommandMode.PAUSE, 0.0, 0.0)
            ),
            "motion1": encode_frame(
                RpiCommand(42, 1, RpiCommandMode.AUTO_MPS, 1.0, 0.0)
            ),
            "motion2": encode_frame(
                RpiCommand(42, 2, RpiCommandMode.AUTO_MPS, 1.0, 0.0)
            ),
            "rearm": encode_frame(
                RpiCommand(42, 3, RpiCommandMode.PAUSE, 0.0, 0.0)
            ),
            "motion3": encode_frame(
                RpiCommand(42, 4, RpiCommandMode.AUTO_MPS, 1.0, 0.0)
            ),
        }
        simulator.transmit(frames["claim"], sent_ms=0, physical_pause_active=True)
        simulator.transmit(frames["motion1"], sent_ms=1)
        simulator.deliver_until(502)
        self.assertFalse(simulator.receiver.decide(now_ms=502).lease_fresh)

        simulator.transmit(frames["motion2"], sent_ms=100, delay_ms=403)
        simulator.deliver_until(503)
        self.assertFalse(simulator.receiver.decide(now_ms=504).lease_fresh)

        simulator.transmit(frames["rearm"], sent_ms=505)
        simulator.deliver_until(505)
        self.assertTrue(simulator.receiver.decide(now_ms=506).lease_fresh)
        simulator.transmit(frames["motion3"], sent_ms=507)
        simulator.deliver_until(507)
        self.assertEqual(
            RpiCommandMode.AUTO_MPS,
            simulator.receiver.decide(now_ms=508).command.mode,
        )

    def test_reordering_rejects_older_sequence(self):
        simulator = RpiCommandLeaseSerialFaultSimulator()
        claim = encode_frame(RpiCommand(42, 0, RpiCommandMode.PAUSE, 0.0, 0.0))
        first = encode_frame(RpiCommand(42, 1, RpiCommandMode.AUTO_MPS, 1.0, 0.0))
        second = encode_frame(RpiCommand(42, 2, RpiCommandMode.AUTO_MPS, 1.0, 0.0))
        simulator.transmit(claim, sent_ms=0, physical_pause_active=True)
        simulator.transmit(first, sent_ms=1, delay_ms=100)
        simulator.transmit(second, sent_ms=2, delay_ms=10)
        simulator.deliver_until(200)
        self.assertEqual(2, simulator.accepted)
        self.assertEqual(1, simulator.rejected)


if __name__ == "__main__":
    unittest.main()
