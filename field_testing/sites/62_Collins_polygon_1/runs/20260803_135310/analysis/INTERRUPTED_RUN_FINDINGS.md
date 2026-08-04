# Interrupted-run findings — 2026-08-03 13:53:10

## Sources

- `pursuit_log_20260803_135312.csv`: 44,716 numeric controller rows at 20 Hz.
- `62_Collins_polygon_1_20260803_135310.csv`: 8,970 field-logger rows.
- `62_Collins_polygon_1_mission.txt`: exact mission used by the launcher.
- Site mission audit and final boundary from `field_testing/sites/62_Collins_polygon_1`.

Times below are local EDT.

## Missed-waypoint event

The clearest loss-of-alignment event centers on controller waypoint index
`1445` (mission-audit waypoint `1446`, because the pursuit log is zero-based).

- The active index remained `1445` from 14:16:01.641 through 14:16:43.747:
  42.106 seconds and 198 Auto-aligned pursuit samples.
- The lookahead target remained fixed at local XY `(-23.9905, -36.4704)`.
- Controller cross-track error was already 1.668 m at the start and varied as
  the tractor traveled around the missed target; the run later exceeded 4 m.
- GPS remained `RTK Fixed` and heading remained valid throughout all 198
  samples. This is not explained by an RTK fix or heading-validity loss.
- At the start of this interval, the logged steering setpoint/current/error was
  `197 / 189 / +8` counts. The same snapshot remained at the end, so the first
  failure is not accompanied by a large steering-feedback error.

The controller selects the first future waypoint farther away than its
lookahead distance. It does not test whether a waypoint has already been
passed. If a tight connector waypoint is missed and remains farther than the
lookahead radius, the index can stop advancing and the controller can keep
turning toward a target now behind or beside the tractor. The data is
consistent with that failure mode.

## Steering-loss evidence

After Manual intervention, Auto was selected again at about 14:17:21.814. The
controller was then on index `1446` with cross-track error about 4.60 m.

- Steering setpoint: 447 counts.
- Steering feedback: 739 counts.
- Steering error: -292 counts.
- Steering PWM: 255, the maximum logged magnitude.
- Actual speed rose from near zero while the feedback remained at 739 in the
  sampled interval.

This proves that the steering loop requested maximum correction while feedback
was far from the setpoint and not converging in the recorded snapshots. It is
consistent with the reported IBT-2 latch or an unavailable steering actuator,
but the CSV alone cannot distinguish an IBT-2 protection latch from loss of
motor power, wiring interruption, or another actuator-side fault.

The `steer_pwm_saturated` field remained false despite `steer_pwm=255`, so that
diagnostic flag should not be relied upon until its firmware meaning is
reviewed.

## Other observations

- The field log contains five separate Auto intervals, reflecting supervised
  pauses and restarts rather than one uninterrupted mission.
- The combined analyzer measured 776.2 seconds of Auto-driving samples after
  carrying each observed field-logger mode forward between bridge-status
  bursts. The earlier 0.15-second nearest-record alignment retained only 171.7
  seconds and made the map appear dashed. Whole-run metrics are still dominated
  by acquisition, turns, the missed-waypoint loop, and restarts; they should not
  be treated as straight-row tuning results.
- The mission should not be rerun unchanged until waypoint advancement can
  recover safely from a passed connector point and steering-loss behavior is
  reviewed.

## Recommended next engineering checks

1. Add a passed-waypoint/along-track advancement rule or a bounded nearest-
   forward reacquisition rule to Pure Pursuit.
2. Add a watchdog that commands neutral/Pause when a waypoint index remains
   unchanged while CTE grows or when steering feedback does not respond to
   sustained high PWM.
3. Review why `steer_pwm_saturated` was false at a logged PWM of 255.
4. Bench-test IBT-2 latch behavior and reset recovery before another full-speed
   mission.
5. Use straight-row-only samples for steering PID evaluation; exclude connector
   recovery and manual intervention windows.
