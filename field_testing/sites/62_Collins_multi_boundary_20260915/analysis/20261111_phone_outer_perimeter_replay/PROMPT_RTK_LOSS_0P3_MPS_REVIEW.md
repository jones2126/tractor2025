# Prompt: review a bounded 0.3 m/s response to loss of RTK Fixed

**DRAFT - UNTESTED IN THE FIELD**

Use this prompt in a future, separate change session. It requests inspection,
design, and a proposed plan first. It does not authorize controller edits,
service changes, installation, flashing, or field motion.

## Prompt

Inspect the current Tractor01 production phone-controlled outer-perimeter stack
and propose a safety-reviewed way to command no more than **0.3 m/s** when the
position solution temporarily drops below `RTK Fixed`, instead of immediately
commanding Pause for that condition alone.

Start with evidence and a detailed proposed plan only. Stop for explicit
approval before editing any file.

Use the November 11 replay evidence in this directory, especially:

- `REPORT.md`
- `safety_condition_loss_episodes.csv`
- `safety_pause_timeline.csv`
- `pause_transition_details.csv`

Also compare the November 9 replay and counterfactual artifacts in
`../20261109_phone_outer_perimeter_replay/`.

Trace the complete production decision path, not just Pure Pursuit. At minimum,
inspect:

- `tractor_rpi/pure-pursuit/pure_pursuit_controller_20261022.py`
- `tractor_rpi/pure-pursuit/pure_pursuit_arbitrator_feed_20261031.py`
- `tractor_rpi/rpi_arbitrator_inputs_20261031.py`
- `tractor_rpi/rpi_safety_supervisor_20261031.py`
- `tractor_rpi/communications_degraded_guard_20261029.py`
- the production mission launcher and dashboard in
  `field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED/`
- all relevant unit, integration, replay, preflight, and dashboard tests

The current implementation has at least two independent RTK-Fixed gates:
Pure Pursuit's `is_drivable()` check and the RPi supervisor's
`AUTO_REQUIRED_HEALTH`/`rtk_fixed` check. Explain how the proposed state is
represented consistently across both layers without weakening unrelated
guards or creating contradictory controller/supervisor states.

The proposed behavior must be narrowly scoped to a fresh, finite position
stream whose only degraded position attribute is loss of `RTK Fixed`. Determine
from recorded evidence whether `RTK Float` and `DGPS` should be treated
differently. Do not assume that either is accurate enough without quantifying
the available evidence and uncertainty.

Preserve immediate zero-speed Pause and the existing sticky recovery behavior
for every unrelated safety condition, including:

- stale, absent, malformed, or non-finite GPS data;
- `headValid=False`, non-fixed heading carrier, invalid baseline geometry, or
  heading accuracy beyond the approved threshold;
- E-stop or operator Pause;
- stale phone, controller, Pure Pursuit, Teensy status, or command lease;
- steering, transmission, JRK, watchdog, or local safety faults;
- undefined route, cross-track corridor violation, blocked reacquisition, or
  non-finite control values;
- any simultaneous RTK and heading/safety fault.

Propose explicit bounds for the degraded-position state, including maximum
speed (hard ceiling 0.3 m/s), maximum time, maximum distance, permitted fix
qualities, route-corridor threshold, entry/exit hysteresis, recovery stability
period, operator indications, latch/reset semantics, and what happens when any
bound is exhausted. Default to Pause when state is missing or ambiguous.

Require first-class telemetry and logs that distinguish normal Auto,
RTK-degraded slow crawl, recovery stabilization, and Pause-latched states. The
dashboard must show the exact condition, speed cap, elapsed time/distance,
remaining envelope, and recovery requirement.

Include a counterfactual replay of both the November 9 and November 11 runs.
Report, for each RTK-loss episode, the recorded duration, estimated distance at
0.3 m/s, overlapping heading or corridor faults, and whether the proposed
policy would have crawled or paused. Do not treat counterfactual output as field
validation.

Provide:

1. current behavior and all enforcement points;
2. evidence-backed hazards and assumptions;
3. proposed state machine and exact invariants;
4. file-by-file change plan;
5. test plan covering unit tests, fault injection, replay, service integration,
   stationary motor-off checks, and a later separately approved blades-off
   field test;
6. rollback plan and acceptance criteria;
7. unresolved questions requiring Al's decision.

Do not modify the Teensy firmware unless inspection proves a protocol change is
unavoidable. Do not change deployed services, flash hardware, or authorize
field motion. Stop for Approval A after presenting the inspection findings and
proposed plan.
