#!/usr/bin/env python3

import unittest

from tractor_rpi.communications_degraded_guard_20261029 import (
    CommunicationsDegradedGuard,
    GuardDecision,
    LocalSafetyHealth,
)


def healthy(**overrides):
    values = {name: True for name in LocalSafetyHealth.__dataclass_fields__}
    values.update(overrides)
    return LocalSafetyHealth(**values)


class CommunicationsDegradedGuardTests(unittest.TestCase):
    def test_fresh_phone_is_normal(self):
        guard = CommunicationsDegradedGuard()
        result = guard.evaluate(
            now=10,
            route_progress_m=100,
            phone_link_fresh=True,
            auto_running=True,
            preauthorized=True,
            health=healthy(),
        )
        self.assertEqual(GuardDecision.NORMAL, result.decision)

    def test_loss_enters_bounded_state_and_reconnect_does_not_reset_budget(self):
        guard = CommunicationsDegradedGuard()
        first = guard.evaluate(
            now=10,
            route_progress_m=100,
            phone_link_fresh=False,
            auto_running=True,
            preauthorized=True,
            health=healthy(),
        )
        recovered = guard.evaluate(
            now=20,
            route_progress_m=110,
            phone_link_fresh=True,
            auto_running=True,
            preauthorized=True,
            health=healthy(),
        )
        lost_again = guard.evaluate(
            now=25,
            route_progress_m=115,
            phone_link_fresh=False,
            auto_running=True,
            preauthorized=True,
            health=healthy(),
        )
        self.assertEqual(GuardDecision.COMMUNICATIONS_DEGRADED, first.decision)
        self.assertEqual(10, recovered.elapsed_s)
        self.assertEqual(first.incident_id, lost_again.incident_id)
        self.assertEqual(15, lost_again.elapsed_s)

    def test_time_and_distance_envelopes_are_independent_hard_limits(self):
        for now, progress, expected in ((40, 101, "time"), (11, 130, "distance")):
            with self.subTest(expected=expected):
                guard = CommunicationsDegradedGuard()
                guard.evaluate(
                    now=10,
                    route_progress_m=100,
                    phone_link_fresh=False,
                    auto_running=True,
                    preauthorized=True,
                    health=healthy(),
                )
                result = guard.evaluate(
                    now=now,
                    route_progress_m=progress,
                    phone_link_fresh=False,
                    auto_running=True,
                    preauthorized=True,
                    health=healthy(),
                )
                self.assertEqual(GuardDecision.PAUSE_REQUIRED, result.decision)
                self.assertIn(expected, result.reason)

    def test_local_integrated_travel_cannot_be_hidden_by_route_progress(self):
        guard = CommunicationsDegradedGuard()
        guard.evaluate(
            now=10,
            route_progress_m=100,
            local_travel_m=50,
            phone_link_fresh=False,
            auto_running=True,
            preauthorized=True,
            health=healthy(),
        )
        result = guard.evaluate(
            now=11,
            route_progress_m=101,
            local_travel_m=80,
            phone_link_fresh=False,
            auto_running=True,
            preauthorized=True,
            health=healthy(),
        )
        self.assertEqual(GuardDecision.PAUSE_REQUIRED, result.decision)
        self.assertIn("distance", result.reason)

    def test_every_local_health_check_fails_closed(self):
        for field in LocalSafetyHealth.__dataclass_fields__:
            with self.subTest(field=field):
                guard = CommunicationsDegradedGuard()
                result = guard.evaluate(
                    now=10,
                    route_progress_m=100,
                    phone_link_fresh=False,
                    auto_running=True,
                    preauthorized=True,
                    health=healthy(**{field: False}),
                )
                self.assertEqual(GuardDecision.PAUSE_REQUIRED, result.decision)
                self.assertIn(field, result.reason)

    def test_reset_requires_stopped_physical_pause_and_ack(self):
        guard = CommunicationsDegradedGuard()
        guard.evaluate(
            now=10,
            route_progress_m=100,
            phone_link_fresh=False,
            auto_running=True,
            preauthorized=True,
            health=healthy(),
        )
        with self.assertRaises(RuntimeError):
            guard.reset_after_stopped_pause(
                physical_pause_active=False, stopped=True, operator_acknowledged=True
            )
        guard.reset_after_stopped_pause(
            physical_pause_active=True, stopped=True, operator_acknowledged=True
        )
        self.assertFalse(guard.incident_active)


if __name__ == "__main__":
    unittest.main()
