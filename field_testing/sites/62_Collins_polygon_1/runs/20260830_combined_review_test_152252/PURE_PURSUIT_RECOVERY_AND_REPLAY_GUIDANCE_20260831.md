# Pure Pursuit recovery and replay guidance - 2026-08-31

## Confirmed controller failure

`pure_pursuit_controller_20260714.py` lines 353-359 search forward from the
current index and select the first stored waypoint whose Euclidean distance is
greater than that waypoint's configured lookahead.

That rule has three related problems:

1. It does not test whether the waypoint is ahead of or behind the tractor.
2. It cannot advance past a missed waypoint that remains just outside the
   lookahead circle.
3. It uses configured lookahead squared in the steering denominator even when
   the selected stored waypoint is farther away than the configured lookahead.

The 2026-08-30 failure is a direct example: waypoints 3041-3044 remained beyond
the 1.0 m circle and became permanent targets while the tractor circled them.

There is a second end-of-mission issue at lines 369-376. The controller declares
the goal reached whenever the endpoint is no more than `pos_tol` ahead in the
tractor frame. A distant endpoint that is beside or behind the tractor can
satisfy that test. End-of-path completion should include Euclidean distance or
separate along-track and lateral tolerances.

## Recommended change sequence

Do not start with a circle counter. Fix path progress first:

1. Project the tractor onto a bounded forward window of path segments, not only
   onto individual waypoints.
2. Keep along-path progress monotonic.
3. Select the lookahead target at an actual path/circle intersection, or by
   interpolating the point at `progress + lookahead` along the polyline.
4. Permit a bounded forward reacquisition when the nearest forward segment is
   clearly closer than the retained segment.
5. If reacquisition would jump too far or the tractor is more than a configured
   distance from the path, command neutral and require operator intervention.

Add a progress watchdog as a secondary defense. Suggested initial replay-only
thresholds are:

- same target for more than `max(5 seconds, 3 * lookahead / commanded_speed)`;
- tractor has traveled at least 5 m while along-path progress has advanced less
  than 0.5 m; or
- steering has remained saturated for more than 3 seconds while target
  distance is not decreasing.

When a watchdog fires, first try bounded forward segment reacquisition. If no
safe candidate is found, stop; do not keep steering toward the old point.

Every recovery should add explicit log fields: recovery state, old/new index,
nearest-segment index, along-path progress, target distance, distance from path,
reason, and whether the controller stopped.

## Replay tool

Generated replay:

`pure_pursuit_replay_2970_3060_20260831.html`

Builder:

`field_testing/tools/build_pure_pursuit_replay_20260831.py`

The replay synchronizes the pursuit log with the field logger and displays:

- actual pose and heading;
- recorded and recomputed target waypoints;
- target distance, lookahead, `yt`, steering angle, and normalized command;
- commanded and actual speed;
- JRK target and feedback;
- steering-pot target, current position, error, PWM, and saturation;
- RTK fix and heading-valid state.

Use the step slider or Play button to move through waypoints 2970-3060. Change
the lookahead slider to replay the current selection rule at 0.5-3.0 m. The
"Prototype forward reacquisition" checkbox is deliberately a simple comparison
model, not production controller logic.

Run synthetic tests before changing the field controller:

1. normal straight path;
2. ordinary 90-degree corner;
3. missed waypoint behind the vehicle;
4. keyhole/turn overshoot matching waypoints 3041-3044;
5. GPS position jump;
6. self-near or crossing path where unrestricted nearest-point search could
   jump to the wrong branch; and
7. endpoint passed laterally but still far away.

Only after those cases pass should the revised controller be replayed over the
entire 2026-08-30 log, then tested blades off at low physical speed with direct
supervision.
