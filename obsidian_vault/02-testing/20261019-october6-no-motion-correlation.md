# 2026-10-19 October 6 no-motion correlation review

## Evidence reviewed

Archive:
`C:\Users\al532\Downloads\tractor01_master_manual_phone_rerun_complete_20261006.tgz`

Verified SHA-256:
`e863e39d4c7a2851209fe2aa6d3ad69781704026696b69bebfabdd925ea7e707`

The CSV analysis ignored the documented terminal NUL padding.

## What the 10:13:34–10:14:02 interval proves

All times below are UTC; subtract four hours for October 6 local time.

- At `14:13:34.932`, the HTTPS server accepted guarded Auto.
- The pure-pursuit log first reports `driving=True` at Unix time
  `1791296020.108`, or approximately `14:13:40.108`. Its speed command is
  `1.000 m/s`.
- At `14:13:40.270`, the mission logger first records the JRK target changing
  from neutral `2836` to forward `2300`.
- JRK feedback moved from about `2811` toward the demand and reached `2296`.
  Peak recorded JRK motor current in the reviewed window was `5249 mA`.
- The last non-neutral JRK target record is `14:14:00.948`.
- GPS ground speed never exceeded `0.02 m/s`. Logged latitude was unchanged;
  longitude changed only from `-80.33248750` to `-80.33248733`, consistent with
  stationary GNSS jitter rather than visible travel.
- The pursuit log changes from waypoint 12127 to 12134 and path progress from
  1762.545 m to 1763.332 m. Because GPS position remained effectively fixed,
  this route-index change is not proof of wheel motion.
- Phone commands were regular through sequence 157 at `14:14:00.951`. Sequence
  158 arrived at `14:14:01.754`, an approximately 803 ms server-arrival gap,
  with a client-reported RTT of 729 ms. That gap exceeds the independent 500 ms
  Teensy watchdog and explains the low-level return to Pause/neutral even
  though the HTTP stream subsequently resumed.

The evidence therefore proves that mission Auto, a 1.0 m/s controller command,
the low-level forward transmission target, JRK actuator motion, and later
watchdog neutralization all occurred. It does not prove that the engine was
running, that the drivetrain was engaged, or that a wheel turned.

## Missing evidence

The archive has no independent measurement of:

- engine running/RPM;
- ignition and engine-kill relay electrical state;
- brake or hydrostatic bypass state;
- axle or wheel rotation;
- actual ground motion independent of GNSS at near-zero speed; or
- a shared command/cycle identifier spanning the controller, bridge, Teensy,
  JRK record, and dashboard.

JRK target and feedback describe the transmission actuator position, not wheel
rotation. They cannot distinguish an engine-off tractor from a running tractor
whose drivetrain, brake, or hydrostatic mechanism did not produce motion.

## Required correlation for future tests

Record one unified monotonic correlation identifier through every layer:

1. phone session and sequence;
2. server receive monotonic time and accepted mode;
3. controller cycle, speed command, waypoint, and route progress;
4. UDP 6004 command and source;
5. bridge receive time and serial record sent;
6. Teensy accepted sequence, command age, mode, and watchdog transition;
7. JRK target, feedback, current, and error state;
8. engine-kill request plus an independent engine-running/RPM input;
9. independent left/right wheel or axle motion; and
10. GPS speed and position with fix-quality fields.

Use the Pi monotonic clock for local causal ordering. Phone wall time is
diagnostic only unless clock offset and uncertainty are measured. A synchronized
video showing the engine/tachometer, wheel marker, and test display is useful
supporting evidence but should not replace machine-readable engine and wheel
sensors.

Until engine state and wheel motion are logged independently, a future repeat
can conclusively say whether commands and the JRK actuator moved, but cannot
conclusively attribute an absence of tractor motion.
