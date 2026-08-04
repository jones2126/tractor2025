# Northern-slope straight steering findings — 2026-08-04

Four isolated east/west missions were run at 0.50 m/s with 2.0 m lookahead.
The field and Pure Pursuit logs both recorded at 20 Hz. All four missions ended
normally with `Goal reached` and clean logger shutdown.

## Central settled-path comparison

The comparison below uses the central 40–80% of each mission's waypoint range.
This excludes initial path acquisition, manual positioning, and endpoint
stopping behavior.

| Run | Direction | Mean signed geometric error | Mean absolute geometric error | 95th percentile | Mean controller lookahead error |
|---|---|---:|---:|---:|---:|
| 1E | East | 0.9 cm north | 2.0 cm | 4.3 cm | 3.9 cm |
| 1W | West | 2.4 cm south | 2.6 cm | 6.1 cm | 7.6 cm |
| 2E | East | 1.8 cm north | 2.6 cm | 7.8 cm | 4.0 cm |
| 2W | West | 2.4 cm south | 3.5 cm | 8.8 cm | 8.1 cm |

The directional effect repeated: eastbound tracks averaged slightly north of
the line and westbound tracks averaged about 2.4 cm south. The magnitude is
small relative to the 0.9652 m lane spacing and is not evidence that integral
or derivative gain should be added yet.

## Low-level steering behavior

Across the broader settled windows, the steering-pot error was within the
firmware deadband approximately 88–94% of the time. Median steering PWM was
zero. When correction was required, the configured minimum PWM clamp produced
150 PWM; there was no PWM saturation. PID telemetry remained active and RTK
quality remained Fixed throughout the compared samples.

This is effectively a quiet, intermittent correction strategy. It tracked the
straight lines well at 0.50 m/s, but the combination of a 10-count deadband and
150 minimum PWM means a small `Ki` would accumulate while the actuator cannot
move and may then release as a larger correction. Therefore, do not add `Ki`
or `Kd` based only on these runs.

## Path-acquisition warning

The full 1E Auto interval integrated to approximately 39 m of GPS travel for a
20.5 m mission. Tractor01 entered Auto well away from the intended start and
used substantial distance to acquire the path. Acquisition portions of all
runs are intentionally excluded from the settled-path table.

Future straight tests should enter Auto only when tractor01 is near the first
waypoint and aligned with the mission heading. A future launcher improvement
should block Auto commands when start-position or start-heading error exceeds
configured limits.

## Recommendation

Keep the current `Kp=1.0`, `Ki=0.0`, and `Kd=0.0` for the next test. Before
adding integral gain, address the steering sensor mechanically (planned
non-contact magnetic sensor), then retest straight-wheel repeatability and
review whether the 10-count deadband and 150 minimum PWM remain appropriate.

Do not infer coverage-mission turn performance from these straight tests. They
validate steady straight tracking only; path acquisition and keyhole turns are
separate behaviors.
