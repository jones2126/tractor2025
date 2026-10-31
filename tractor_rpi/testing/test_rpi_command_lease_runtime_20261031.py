#!/usr/bin/env python3

import unittest

from tractor_rpi.communications_degraded_guard_20261029 import LocalSafetyHealth
from tractor_rpi.rpi_command_lease_adapter_20261031 import (
    RpiCommandLeaseAdapter,
    parse_rpi_ctl_line,
)
from tractor_rpi.rpi_command_lease_protocol_20261031 import (
    RpiCommandMode,
    decode_frame,
)
from tractor_rpi.rpi_command_lease_runtime_20261031 import RpiCommandLeaseRuntime
from tractor_rpi.rpi_safety_supervisor_20261031 import SupervisorInputs, SupervisorState


class FakeSerial:
    def __init__(self):
        self.frames = []

    def write(self, frame):
        self.frames.append(frame)
        return len(frame)

    def flush(self):
        pass


def healthy():
    return LocalSafetyHealth(
        **{name: True for name in LocalSafetyHealth.__dataclass_fields__}
    )


def inputs(now=0.0, **overrides):
    values = dict(
        now_monotonic_s=now,
        requested_mode=RpiCommandMode.AUTO_MPS,
        phone_link_fresh=True,
        communications_degraded_preauthorized=True,
        auto_running=True,
        manual_drive_percent=0.0,
        manual_steering=0.0,
        pure_pursuit_speed_mps=1.0,
        pure_pursuit_steering=0.2,
        route_progress_m=100.0,
        local_travel_m=50.0,
        health=healthy(),
    )
    values.update(overrides)
    return SupervisorInputs(**values)


def status(sequence, *, session=42, mode=0, fresh=1, usable=1, latched=0):
    return parse_rpi_ctl_line(
        f"1,100,RPI_CTL,m={mode},f={fresh},u={usable},l={latched},a=10,"
        f"ss={session},s={sequence},ok=1,rj=0,rr=0"
    )


class RpiCommandLeaseRuntimeTests(unittest.TestCase):
    def make_runtime(self):
        sink = FakeSerial()
        adapter = RpiCommandLeaseAdapter(sink, session=42)
        return sink, adapter, RpiCommandLeaseRuntime(adapter)

    def test_auto_first_sends_pause_claim_then_waits_for_ack(self):
        _, _, runtime = self.make_runtime()
        first = runtime.tick(now_monotonic_s=0.0, inputs=inputs(0.0))
        self.assertTrue(first.waiting_for_pause_ack)
        self.assertEqual(RpiCommandMode.PAUSE, decode_frame(first.frame_sent).mode)

        waiting = runtime.tick(now_monotonic_s=0.1, inputs=inputs(0.1))
        self.assertIsNone(waiting.frame_sent)
        acknowledged = runtime.tick(
            now_monotonic_s=0.2,
            inputs=inputs(0.2),
            status=status(0),
        )
        self.assertFalse(acknowledged.waiting_for_pause_ack)
        self.assertEqual(RpiCommandMode.AUTO_MPS, decode_frame(acknowledged.frame_sent).mode)

    def test_motion_frames_are_rate_limited_below_500_ms(self):
        _, _, runtime = self.make_runtime()
        runtime.tick(now_monotonic_s=0.0, inputs=inputs(0.0))
        runtime.tick(now_monotonic_s=0.01, inputs=inputs(0.01), status=status(0))
        early = runtime.tick(now_monotonic_s=0.05, inputs=inputs(0.05))
        due = runtime.tick(now_monotonic_s=0.11, inputs=inputs(0.11))
        self.assertIsNone(early.frame_sent)
        self.assertEqual(RpiCommandMode.AUTO_MPS, decode_frame(due.frame_sent).mode)

    def test_scheduler_gap_latches_pause_and_requires_operator_reset(self):
        _, _, runtime = self.make_runtime()
        runtime.tick(now_monotonic_s=0.0, inputs=inputs(0.0))
        runtime.tick(now_monotonic_s=0.01, inputs=inputs(0.01), status=status(0))
        fault = runtime.tick(now_monotonic_s=0.30, inputs=inputs(0.30))
        self.assertTrue(fault.scheduler_fault_latched)
        self.assertEqual(SupervisorState.PAUSE_LATCHED, fault.decision.state)
        self.assertEqual(RpiCommandMode.PAUSE, decode_frame(fault.frame_sent).mode)

        held = runtime.tick(now_monotonic_s=0.31, inputs=inputs(0.31))
        self.assertTrue(held.scheduler_fault_latched)
        with self.assertRaises(RuntimeError):
            runtime.reset_after_stopped_pause(
                physical_pause_active=False,
                stopped=True,
                operator_acknowledged=True,
            )
        runtime.reset_after_stopped_pause(
            physical_pause_active=True,
            stopped=True,
            operator_acknowledged=True,
        )
        reset = runtime.tick(
            now_monotonic_s=1.0,
            inputs=inputs(1.0, requested_mode=RpiCommandMode.PAUSE),
        )
        self.assertFalse(reset.scheduler_fault_latched)

    def test_teensy_expiry_latch_revokes_motion_and_sends_pause(self):
        _, _, runtime = self.make_runtime()
        runtime.tick(now_monotonic_s=0.0, inputs=inputs(0.0))
        runtime.tick(now_monotonic_s=0.01, inputs=inputs(0.01), status=status(0))
        result = runtime.tick(
            now_monotonic_s=0.10,
            inputs=inputs(0.10),
            status=status(1, mode=2, usable=0, latched=1),
        )
        self.assertTrue(result.waiting_for_pause_ack)
        self.assertEqual(RpiCommandMode.PAUSE, decode_frame(result.frame_sent).mode)

    def test_phone_loss_in_auto_keeps_bounded_auto_but_manual_loss_pauses(self):
        _, _, auto_runtime = self.make_runtime()
        auto_runtime.tick(now_monotonic_s=0.0, inputs=inputs(0.0))
        degraded = auto_runtime.tick(
            now_monotonic_s=0.01,
            inputs=inputs(0.01, phone_link_fresh=False),
            status=status(0),
        )
        self.assertEqual(SupervisorState.COMMUNICATIONS_DEGRADED, degraded.decision.state)
        self.assertEqual(RpiCommandMode.AUTO_MPS, decode_frame(degraded.frame_sent).mode)

        _, _, manual_runtime = self.make_runtime()
        manual = manual_runtime.tick(
            now_monotonic_s=0.0,
            inputs=inputs(
                0.0,
                requested_mode=RpiCommandMode.MANUAL_PERCENT,
                phone_link_fresh=False,
            ),
        )
        self.assertEqual(SupervisorState.PAUSE_LATCHED, manual.decision.state)
        self.assertEqual(RpiCommandMode.PAUSE, decode_frame(manual.frame_sent).mode)

    def test_invalid_runtime_intervals_cannot_approach_500_ms(self):
        adapter = RpiCommandLeaseAdapter(FakeSerial(), session=42)
        for kwargs in (
            {"motion_interval_s": 0.5},
            {"pause_retry_s": 0.5},
            {"maximum_tick_gap_s": 0.5},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    RpiCommandLeaseRuntime(adapter, **kwargs)


if __name__ == "__main__":
    unittest.main()
