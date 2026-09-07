# Tractor01 field-test handoff: steering recovery and 1.5 m/s speed calibration

Date: 2026-09-07  
Status: steering restored and safety monitor validated; speed-calibration mission is the next task

## Purpose of the next session

Build and run a supervised, blades-off speed-calibration test that:

1. establishes a repeatable baseline at the known-good full-forward production target;
2. incrementally extends the forward JRK actuator command until tractor01 achieves approximately 1.5 m/s actual GPS ground speed;
3. uses repeated laps of the same inner ring to include uphill, downhill, side-slope, and pitch-dependent loading;
4. calculates both lap-level and combined speed statistics; and
5. documents and preserves every newly approved maximum actuator command, feedback position, current peak, and measured ground speed.

The lower the JRK target number, the farther forward the transmission actuator moves. Target 1880 has been reached by the guarded stationary actuator test, but it is **not yet an approved production speed setting** and it is not known to be the mechanical stop.

## Repository and deployed-software state

At the end of this session, the C-drive checkout, GitHub `main`, and tractor01 were clean and synchronized at:

```text
e63da59  Revert "Temporarily roll back steering response watchdog"
```

Production Teensy firmware was compiled and flashed from:

- [`tractor_teensy/src/teensy_main_20260804.cpp`](../../tractor_teensy/src/teensy_main_20260804.cpp)
- [`tractor_teensy/platformio.ini`](../../tractor_teensy/platformio.ini)

The active Raspberry Pi bridge, including ntfy delivery and the field-tested 8192-byte GPS receive buffer, is:

- [`tractor_rpi/teensy_serial_bridge_20260728.py`](../../tractor_rpi/teensy_serial_bridge_20260728.py)

The 20 Hz telemetry verifier is:

- [`tractor_rpi/testing/verify_steering_telemetry_20260804.py`](../../tractor_rpi/testing/verify_steering_telemetry_20260804.py)

The current field logger is:

- [`tractor_rpi/field_test_logger_20260828.py`](../../tractor_rpi/field_test_logger_20260828.py)

Before changing or running anything in the next session, recheck all three repositories and preserve any tractor-only files before pulling.

## Steering failure: diagnosis and resolution

The original symptom was an apparently healthy IBT-2 power supply and enable inputs, but no voltage at the motor outputs and no steering movement. Replacing the IBT-2 and the DROK 12-to-5 V converter did not initially restore steering.

Multimeter testing then showed:

- Teensy pin 5 is `RPWM`.
- Teensy pin 6 is `LPWM`.
- Pin 6 measured about 3.2 V during the expected Manual steering command, proving that the Teensy and firmware were issuing the command.
- Steering returned after unseating and reseating the Teensy-to-IBT control connection.

The most likely cause was corrosion or high resistance on a Teensy-to-IBT connector pin, probably in the `LPWM` path. The replacement IBT-2 and DROK converter remain installed. The connector should be inspected, cleaned with an appropriate electrical-contact product, mechanically secured, and protected from moisture after field testing.

## Steering watchdog and ntfy validation

The previously tested steering-response watchdog was restored. Its current qualifying thresholds are defined in the production Teensy source:

- minimum steering error: 25 pot counts;
- minimum commanded PWM: 150;
- required feedback movement: 5 counts toward the command; and
- response timeout: 750 ms.

On a fault, the Teensy blocks the failed steering drive direction, latches the fault, and immediately commands JRK neutral. Auto cannot resume until the operator selects Pause and then demonstrates successful steering response in Manual while the Manual transmission command is neutral.

Validation completed on 2026-09-07:

- Steering telemetry: 20.00 Hz, median interval 50.0 ms, 101 unique sequences, zero malformed packets.
- Normal Manual motion: 26 response attempts accumulated without a false fault.
- Deliberate engine-off failure test with the steering fuse removed: one fault latched and one ntfy notification delivered.
- Recovery: Pause was acknowledged, steering fuse was restored, Manual steering response succeeded, and the latch cleared without rebooting.
- Final steering status: pot 443 (nominal center 447), `fault_latched=0`, `fault_count=1`, `recovery_pause_seen=0`, and `drive_blocked=0`.

The retained fault count of one is the expected historical record of the deliberate test.

## Important Manual transmission-control detail

Pause always commands JRK neutral target 2836. In Pause, the reported `bucket` can remain the last remembered Manual bucket and must not be treated as the active target.

The handheld transmission potentiometer's middle indent was observed near raw value 500. Under the current zero-based Manual bucket mapping:

- raw 562 through 653 selects bucket 4 and target 2836, the Manual-neutral range;
- raw 469 through 561 selects bucket 5 and target 2616, the first forward range.

Therefore, the middle indent near raw 500 is **not Manual neutral**. Before starting the engine, position the handheld transmission control in the known neutral range of 562-653 while still in Pause. Do not depend on the physical middle indent during a Manual recovery or mission abort.

## Existing speed evidence

The current production `SPEED_CAL` table is in the production Teensy source. It maps an Auto command of 1.25 m/s to JRK target 2288. Commands above 1.25 m/s currently clamp to the same final table entry, so merely putting 1.5 m/s in a mission file will **not** increase tractor speed.

Previous field evidence found:

- JRK target 2288 produced approximately 1.001 m/s median actual GPS speed under the observed field conditions.
- The result varies with slope and direction, so a single straight run is insufficient for the final calibration.

Existing Ring 13 speed-test materials:

- [Test notes](../../field_testing/sites/62_Collins_polygon_1/mission_plans/20260831_speed_and_skip_turn_tests/REVIEW_TEST_NOTES_20260831.md)
- [Four-speed Ring 13 mission](../../field_testing/sites/62_Collins_polygon_1/mission_plans/20260831_speed_and_skip_turn_tests/01_speed_settings/62_Collins_ring13_four_speed_settings_REVIEW_TEST_20260831.txt)
- [Launcher](../../field_testing/sites/62_Collins_polygon_1/mission_plans/20260831_speed_and_skip_turn_tests/01_speed_settings/speed_settings_20260831.sh)
- [Build report](../../field_testing/sites/62_Collins_polygon_1/mission_plans/20260831_speed_and_skip_turn_tests/01_speed_settings/speed_settings_ring13_report_20260831.json)
- [Validation report](../../field_testing/sites/62_Collins_polygon_1/mission_plans/20260831_speed_and_skip_turn_tests/01_speed_settings/speed_settings_validation_20260831.json)
- [Route preview](../../field_testing/sites/62_Collins_polygon_1/mission_plans/20260831_speed_and_skip_turn_tests/01_speed_settings/speed_settings_ring13_preview_20260831.png)

Ring 13 characteristics:

- lap length: approximately 73.39 m;
- lookahead: 2.0 m;
- waypoint spacing: 0.5 m;
- previous start: latitude 40.485616704, longitude -80.332356671;
- previous start heading: 163.81 degrees compass; and
- previous contained start transit: approximately 15.86 m.

## Guarded actuator evidence and present forward floor

The guarded stationary tests used 40-count pulses, stopped the motor after each pulse, and returned to neutral in 40-count stages. They did not change JRK settings.

Summary:

- lowest successful commanded target: 1880;
- lowest resulting feedback: 1888;
- successful staged forward probes: 60;
- maximum successful staged forward peak: 2.759 A;
- maximum successful staged return peak: 2.277 A; and
- target 1880 was reached without a high-current stall.

Large direct target changes previously produced approximately 3.9-4.0 A startup peaks, and a large direct return-to-neutral produced a 5.067 A transient. New forward settings must therefore be approached incrementally; the guarded stationary result does not authorize a direct production jump from neutral to 1880.

Evidence and analysis:

- [JRK analysis summary](../../field_testing/jrk/20260906/analysis/jrk_analysis_summary_20260906.json)
- [Movement summary](../../field_testing/jrk/20260906/analysis/jrk_movement_summary_20260906.csv)
- [Position performance map](../../field_testing/jrk/20260906/analysis/jrk_position_performance_map_20260906.csv)
- [Guard samples](../../field_testing/jrk/20260906/analysis/jrk_guard_samples_20260906.csv)
- [Original logs and settings snapshot](../../field_testing/jrk/20260906/source_logs/)
- [Performance workbook](../../outputs/jrk_20260906/jrk_actuator_performance_20260906.xlsx)
- [`jrk_limit_guard_test` firmware](../../tractor_rpi/testing/jrk_limit_guard_test/src/main.cpp)

## Required next mission design

Create a new dated `REVIEW_TEST` mission and launcher rather than editing the archived 2026-08-31 mission in place. Reuse the validated Ring 13 geometry so each lap samples the same changing slope and pitch conditions.

The new mission should begin near the previous start pose, but provide practical staging tolerance in both position and heading. Do not simply widen the old 1.5 m / 20 degree start gate without reviewing the resulting approach path. The next session should:

1. choose a larger field-practical start allowance with Al;
2. simulate starts at the distance and heading extremes;
3. confirm that acquisition and the complete start transit remain inside the reviewed boundary; and
4. record the final position and heading tolerances in the build report and launcher.

A reasonable candidate for evaluation is a start radius around 4 m and a heading allowance around 45 degrees, but those values are not approved until the start-transit geometry is validated.

## Recommended test sequence

### Phase 1: repeatable 1.0 m/s baseline

Build a three-lap Ring 13 mission at JRK target 2288. In the current firmware that corresponds to a 1.25 m/s Auto command and approximately 1.0 m/s observed ground speed.

Run all three laps in one continuous test if steering, GPS, transmission current, and tracking remain healthy. This establishes repeatability and captures different slope and pitch sectors.

For each lap:

- exclude the contained start transit;
- exclude acceleration into the lap and approximately the first and last five seconds around lap boundaries;
- calculate mean, median, standard deviation, minimum, maximum, and sample count for actual GPS speed;
- calculate the same statistics by uphill, downhill, and side-slope/heading sector where the field data supports that classification;
- preserve commanded speed, requested JRK target, actual JRK target, feedback, duty-cycle target, applied duty cycle, and JRK error fields; and
- preserve steering command, feedback, error, PWM, response time, response movement, and watchdog state.

Use the median of each lap as the primary robust value, then report the mean of the three lap medians as the baseline. Also report a sample-weighted combined median and mean; do not hide slope-dependent spread inside one average.

### Phase 2: approach 1.5 m/s incrementally

Do not relabel target 2288 as 1.5 m/s. Add a clearly documented temporary calibration mechanism that lets the Auto mission command an exact candidate JRK target while retaining production steering and watchdog behavior.

Suggested candidate sequence, one reviewed step at a time:

```text
2288  established baseline
2200  first extension
2120  second extension
2040  third extension
1960  only if still below 1.5 m/s and all evidence is healthy
1880  guarded-test floor; use only if prior steps justify it
```

Stop lowering the target as soon as the slope-aware average reaches approximately 1.5 m/s, or earlier for abnormal current, failure to reach feedback, poor controllability, steering faults, tracking degradation, or insufficient stopping margin. The exact final target should be interpolated from the two safe measurements bracketing 1.5 m/s and then confirmed with a separate three-lap Ring 13 run.

Avoid combining several unproven lower targets into one uninterrupted mission. Review the log and physical behavior after every new maximum-forward target before authorizing the next one.

### Phase 3: preserve the selected maximum

For every newly tested forward extension, record:

- commanded JRK target;
- minimum and steady-state feedback;
- peak and typical current;
- time to reach target;
- actual GPS speed by lap and slope sector;
- engine throttle setting, mower-deck state, tire condition, and notable ground conditions;
- whether steering remained responsive and watchdog fault count remained unchanged; and
- whether the target is rejected, test-only, or approved as the new production maximum.

After the final three-lap confirmation:

1. update `SPEED_CAL` with measured `{actual_mps, jrkTarget}` anchors;
2. make 1.5 m/s map to the confirmed safe target rather than a placeholder;
3. document the new approved actuator floor separately from the guarded-test floor and mechanical limit;
4. regenerate and validate affected missions;
5. update the JRK performance map/workbook with GPS speed data;
6. archive raw field and pursuit logs under a new dated run directory;
7. commit and push the code, mission, reports, plots, and notes; and
8. pull and verify the same clean commit on tractor01.

## Safety and readiness gate before moving

- Blades disengaged for all calibration runs.
- Operator directly supervising with immediate Pause/E-stop access.
- Steering connector inspected and steering response tested in Manual.
- Steering fuse installed; watchdog active; `fault_latched=0` and `drive_blocked=0`.
- Handheld Manual transmission control placed in the true raw 562-653 neutral range, not assumed neutral from the middle indent.
- RTK Fixed and valid heading carrier.
- `teensy-bridge.service` and `rtcm-server.service` active.
- Mission preflight passes.
- No existing field logger or Pure Pursuit process running.
- Engine throttle setting recorded and held constant across comparison runs.
- Test area clear, adequate stopping distance available, and path preview reviewed.
- Stop immediately for any ntfy steering fault, unexpected transmission response, loss of RTK/heading, abnormal JRK current, or tracking that approaches the reviewed boundary.

## Suggested opening request for the next chat

> Continue from `obsidian_vault/02-testing/20260907-steering-recovery-and-1p5mps-speed-test-handoff.md`. Verify the repository and tractor state first. Then build and statically validate a dated, blades-off Ring 13 speed-calibration mission beginning near the previous start, with a reviewed practical position/heading allowance. Start with three laps at JRK target 2288 to measure the repeatable 1.0 m/s baseline across slope and pitch. After reviewing that log, help increment the exact JRK target toward a measured 1.5 m/s without exceeding the guarded target-1880 floor, and preserve all raw logs, analysis, actuator-current evidence, and the resulting approved maximum.

