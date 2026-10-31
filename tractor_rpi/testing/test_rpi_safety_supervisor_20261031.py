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
from tractor_rpi.rpi_safety_supervisor_20261031 import (
    RpiSafetySupervisor,
    SupervisorInputs,
    SupervisorState,
    transmit_decision_once,
)


def healthy(**overrides):
    values = {name: True for name in LocalSafetyHealth.__dataclass_fields__}
    values.update(overrides)
    return LocalSafetyHealth(**values)


def inputs(**overrides):
    values = dict(
        now_monotonic_s=10.0,
        requested_mode=RpiCommandMode.AUTO_MPS,
        phone_link_fresh=True,
        communications_degraded_preauthorized=True,
        auto_running=True,
        manual_drive_percent=0.0,
        manual_steering=0.0,
        pure_pursuit_speed_mps=1.0,
        pure_pursuit_steering=0.25,
        route_progress_m=100.0,
        local_travel_m=50.0,
        health=healthy(),
    )
    values.update(overrides)
    return SupervisorInputs(**values)


class FakeSerial:
    def __init__(self):
        self.frames = []

    def write(self, frame):
        self.frames.append(frame)
        return len(frame)

    def flush(self):
        pass


class RpiSafetySupervisorTests(unittest.TestCase):
    def test_pause_is_always_zero(self):
        supervisor = RpiSafetySupervisor()
        result = supervisor.evaluate(inputs(requested_mode=RpiCommandMode.PAUSE))
        self.assertEqual(SupervisorState.PAUSE, result.state)
        self.assertEqual(RpiCommandMode.PAUSE, result.mode)
        self.assertEqual((0.0, 0.0), (result.drive, result.steering))

    def test_fresh_manual_and_auto_pass_their_measured_commands(self):
        supervisor = RpiSafetySupervisor()
        manual = supervisor.evaluate(
            inputs(
                requested_mode=RpiCommandMode.MANUAL_PERCENT,
                manual_drive_percent=-25.0,
                manual_steering=-0.5,
                health=healthy(pure_pursuit_command_fresh=False, rtk_fixed=False),
            )
        )
        auto = supervisor.evaluate(inputs())
        self.assertEqual(SupervisorState.MANUAL, manual.state)
        self.assertEqual((-25.0, -0.5), (manual.drive, manual.steering))
        self.assertEqual(SupervisorState.AUTO, auto.state)
        self.assertEqual((1.0, 0.25), (auto.drive, auto.steering))

    def test_manual_phone_loss_is_immediate_latched_pause(self):
        supervisor = RpiSafetySupervisor()
        result = supervisor.evaluate(
            inputs(
                requested_mode=RpiCommandMode.MANUAL_PERCENT,
                phone_link_fresh=False,
            )
        )
        self.assertEqual(SupervisorState.PAUSE_LATCHED, result.state)
        self.assertIn("Manual", result.reason)

    def test_auto_phone_loss_enters_bounded_state(self):
        supervisor = RpiSafetySupervisor()
        result = supervisor.evaluate(inputs(phone_link_fresh=False))
        self.assertEqual(SupervisorState.COMMUNICATIONS_DEGRADED, result.state)
        self.assertEqual(RpiCommandMode.AUTO_MPS, result.mode)
        self.assertEqual(30.0, result.guard.remaining_s)
        self.assertEqual(30.0, result.guard.remaining_m)

    def test_reconnect_churn_does_not_reset_time_or_distance(self):
        supervisor = RpiSafetySupervisor()
        first = supervisor.evaluate(inputs(now_monotonic_s=10, phone_link_fresh=False))
        recovered = supervisor.evaluate(
            inputs(
                now_monotonic_s=20,
                route_progress_m=110,
                local_travel_m=60,
                phone_link_fresh=True,
            )
        )
        lost_again = supervisor.evaluate(
            inputs(
                now_monotonic_s=25,
                route_progress_m=115,
                local_travel_m=65,
                phone_link_fresh=False,
            )
        )
        self.assertEqual(first.guard.incident_id, lost_again.guard.incident_id)
        self.assertEqual(10.0, recovered.guard.elapsed_s)
        self.assertEqual(15.0, lost_again.guard.elapsed_s)
        self.assertEqual(15.0, lost_again.guard.distance_m)

    def test_time_and_distance_limits_each_pause(self):
        for later in (
            inputs(now_monotonic_s=40, route_progress_m=101, phone_link_fresh=False),
            inputs(now_monotonic_s=11, route_progress_m=130, phone_link_fresh=False),
        ):
            with self.subTest(later=later):
                supervisor = RpiSafetySupervisor()
                supervisor.evaluate(inputs(now_monotonic_s=10, phone_link_fresh=False))
                result = supervisor.evaluate(later)
                self.assertEqual(SupervisorState.PAUSE_LATCHED, result.state)
                self.assertEqual(RpiCommandMode.PAUSE, result.mode)

    def test_every_auto_health_failure_pauses_even_with_fresh_phone(self):
        for field in LocalSafetyHealth.__dataclass_fields__:
            with self.subTest(field=field):
                supervisor = RpiSafetySupervisor()
                result = supervisor.evaluate(inputs(health=healthy(**{field: False})))
                self.assertEqual(SupervisorState.PAUSE_LATCHED, result.state)
                self.assertIn(field, result.reason)

    def test_degraded_operation_must_be_preauthorized(self):
        supervisor = RpiSafetySupervisor()
        result = supervisor.evaluate(
            inputs(
                phone_link_fresh=False,
                communications_degraded_preauthorized=False,
            )
        )
        self.assertEqual(SupervisorState.PAUSE_LATCHED, result.state)
        self.assertIn("pre-authorized", result.reason)

    def test_reset_requires_physical_pause_stop_and_operator_ack(self):
        supervisor = RpiSafetySupervisor()
        supervisor.evaluate(
            inputs(
                requested_mode=RpiCommandMode.MANUAL_PERCENT,
                phone_link_fresh=False,
            )
        )
        with self.assertRaises(RuntimeError):
            supervisor.reset_after_stopped_pause(
                physical_pause_active=False,
                stopped=True,
                operator_acknowledged=True,
            )
        supervisor.reset_after_stopped_pause(
            physical_pause_active=True,
            stopped=True,
            operator_acknowledged=True,
        )
        result = supervisor.evaluate(inputs(requested_mode=RpiCommandMode.PAUSE))
        self.assertEqual(SupervisorState.PAUSE, result.state)

    def test_adapter_integration_requires_pause_ack_before_auto(self):
        supervisor = RpiSafetySupervisor()
        adapter = RpiCommandLeaseAdapter(FakeSerial(), session=42)
        pause = supervisor.evaluate(inputs(requested_mode=RpiCommandMode.PAUSE))
        claim = decode_frame(transmit_decision_once(adapter, pause))
        self.assertEqual(RpiCommandMode.PAUSE, claim.mode)

        auto = supervisor.evaluate(inputs())
        with self.assertRaises(RuntimeError):
            transmit_decision_once(adapter, auto)
        status = parse_rpi_ctl_line(
            "1,100,RPI_CTL,m=0,f=1,u=1,l=0,a=10,"
            "ss=42,s=0,ok=1,rj=0,rr=0"
        )
        self.assertTrue(adapter.observe_status(status))
        sent = decode_frame(transmit_decision_once(adapter, auto))
        self.assertEqual(RpiCommandMode.AUTO_MPS, sent.mode)
        self.assertEqual(1.0, sent.drive)


if __name__ == "__main__":
    unittest.main()
