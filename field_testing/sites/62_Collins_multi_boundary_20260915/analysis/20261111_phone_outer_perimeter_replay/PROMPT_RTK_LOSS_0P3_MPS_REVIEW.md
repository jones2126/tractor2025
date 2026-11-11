# Prompt: review bounded 0.3 m/s continuation during RTK or heading degradation

**DRAFT - UNTESTED IN THE FIELD**

Use this prompt in a future, separate change session. It requests inspection,
design, and a proposed plan first. It does not authorize controller edits,
service changes, installation, flashing, or field motion.

## Prompt

Inspect the current Tractor01 production phone-controlled outer-perimeter stack
and propose a safety-reviewed way to command no more than **0.3 m/s**, while
continuing the mission, when one or more of these temporary degradations occurs:

- RTK fix drops below `RTK Fixed`;
- heading validity is lost (`headValid=False`); or
- heading carrier is not fixed.

This is the requested behavior to design and evaluate. Do not silently narrow
the proposal back to RTK loss alone. If inspection shows that any requested
continuation cannot be bounded safely with the available sensors, identify the
specific missing evidence or invariant and propose the smallest additional
measurement or guard needed for review.

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

## Nine November 11 triggers in scope

The requested policy applies to each of the nine recorded Pure Pursuit safety
interruptions below, in the recorded order. The future replay must evaluate
each row individually rather than only grouping by trigger name.

| # | Trigger | Raw breached-condition duration | Route waypoint |
|---:|---|---:|---:|
| 1 | RTK fix dropped below RTK Fixed | 2.954 s | 10 |
| 2 | heading validity lost | 0.952 s | 191 |
| 3 | heading carrier was not fixed | 0.751 s | 636 |
| 4 | heading carrier was not fixed | 1.953 s | 731 |
| 5 | RTK fix dropped below RTK Fixed | 1.002 s | 759 |
| 6 | heading carrier was not fixed | 1.002 s | 1120 |
| 7 | RTK fix dropped below RTK Fixed | 1.001 s | 1139 |
| 8 | heading validity lost | 2.203 s | 1335 |
| 9 | RTK fix dropped below RTK Fixed | 0.200 s | 1858 |

The values above come from `safety_pause_timeline.csv`. Cross-check them against
the independent raw episodes in `safety_condition_loss_episodes.csv` and
explain any difference before using them as acceptance evidence.

## Complete enforcement path

The current implementation has independent gates in both Pure Pursuit and the
RPi safety supervisor. Pure Pursuit's `is_drivable()` checks RTK quality,
`headValid`, carrier state, baseline geometry, and heading accuracy. The RPi
supervisor independently requires `rtk_fixed` and `heading_valid_fixed` in
`AUTO_REQUIRED_HEALTH`. Explain how the proposed degraded-navigation state is
represented consistently across both layers without weakening unrelated guards
or creating contradictory controller/supervisor states.

The proposed behavior must be narrowly scoped to fresh, finite position and
heading streams where the degraded attributes are limited to the three
requested trigger classes above. Determine from recorded evidence whether RTK
Float, DGPS, heading carrier `float`, carrier `none`, and `headValid=False`
require different bounds. Define what numeric heading source remains usable
during each requested heading-degradation case, how old it may be, and how its
uncertainty is bounded. Do not assume that a degraded value remains accurate
without quantifying the available evidence and uncertainty.

If two or all three requested degradations occur together, treat that combined
case explicitly. Do not accidentally fall through to unrestricted normal Auto.
The proposal must either define a fail-closed bounded 0.3 m/s state for the
combination or document why a specific combination requires Pause.

Preserve immediate zero-speed Pause and the existing sticky recovery behavior
for every condition outside the explicitly requested degradation state,
including:

- stale, absent, malformed, or non-finite GPS data;
- absent, stale, or non-finite numeric heading; invalid baseline geometry; or
  heading accuracy beyond the separately approved degraded-state threshold;
- E-stop or operator Pause;
- stale phone, controller, Pure Pursuit, Teensy status, or command lease;
- steering, transmission, JRK, watchdog, or local safety faults;
- undefined route, cross-track corridor violation, blocked reacquisition, or
  non-finite control values;
- any unrecognized, unmeasured, or ambiguous navigation fault.

Propose explicit bounds for the degraded-navigation state, including maximum
speed (hard ceiling 0.3 m/s), maximum time, maximum distance, permitted fix and
carrier states, usable heading age/uncertainty, route-corridor threshold,
entry/exit hysteresis, recovery stability period, operator indications,
latch/reset semantics, and what happens when any bound is exhausted. Default
to Pause when state is missing or ambiguous.

Require first-class telemetry and logs that distinguish normal Auto,
0.3 m/s degraded-navigation continuation, recovery stabilization, and
Pause-latched states. The dashboard must show the exact condition, speed cap,
elapsed time/distance,
remaining envelope, and recovery requirement.

Include a counterfactual replay of both the November 9 and November 11 runs.
Report, for every RTK-fix, heading-validity, and heading-carrier episode, the
recorded duration, estimated distance at 0.3 m/s, overlapping navigation or
corridor faults, numeric heading/position data actually available, and whether
the proposed policy would have continued at 0.3 m/s or paused. Include the nine
November 11 rows above as individually named acceptance cases. Do not treat
counterfactual output as field validation.

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
