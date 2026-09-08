# Ring 13 Four-Value Calibration Retest — 2026-09-08

This folder defines the approved follow-up to the completed 2026-09-08 Ring 13
field run. It uses `teensy_main_20260908.cpp` and preserves the same reviewed
transit and three-lap geometry while testing revised transmission targets.

## Evidence for the new values

The preceding run is archived under:

`field_testing/sites/62_Collins_polygon_1/runs/20260908_ring13_three_speed_110712`

Its steady median results were:

| Command | JRK target | Actual GPS median |
|---:|---:|---:|
| 0.75 m/s | 2429 | 0.361 m/s |
| 1.00 m/s | 2288 | 1.041 m/s |
| 1.25 m/s | 2240 | 1.223 m/s |
| 1.50 m/s | 2200 | 1.344 m/s |

The operator approved the following four-value retest schedule:

| Section | Mission rows | Command speed | Teensy JRK target |
|---|---:|---:|---:|
| Transit | 1–66 | 0.75 m/s | 2350 |
| Lap 1 | 67–212 | 1.00 m/s | 2300 |
| Lap 2 | 213–359 | 1.20 m/s | 2246 |
| Lap 3 | 360–506 | 1.50 m/s | 2160 |

Target 2160 is a cautious intermediate step. The two upper results predicted
approximately 2149 for a 1.50 m/s median, but the preceding 1.50 m/s lap was
interrupted by a long radio-loss safety stop and already reached 1.586 m/s on
some downhill samples.

Lower JRK numbers move the transmission farther forward. The guarded
stationary actuator test previously reached target 1880 with feedback near
1888 without a confirmed physical limit or high-current stall. That value is
only a guarded-test floor—not a known mechanical limit and not an approved
driving target. The new 2160 target remains 280 counts above it.

## Geometry source and guards

The path geometry comes from the already reviewed and field-run mission:

`../20260831_speed_and_skip_turn_tests/01_speed_settings/62_Collins_ring13_four_speed_settings_REVIEW_TEST_20260831.txt`

The builder requires that source to retain SHA-256:

`7546efab90c6d6a1045342ed9a3b30ec306a82e404c68d1314bc3f31c907fa60`

The builder preserves latitude, longitude, heading, and lookahead exactly. It
keeps the original 66-row transit and first three complete Ring 13 laps, then
drops the historical fourth lap. Transit lookahead remains 1.50 m; Ring 13
lookahead remains 2.00 m. Approximate route length remains 236.0 m.

## Files

- `build_ring13_four_value_mission_20260908.py` regenerates and validates the
  506-row mission.
- `62_Collins_ring13_four_value_settings_20260908.txt` is generated at run
  time and intentionally not treated as the geometry source.
- `preview_ring13_four_value_20260908.py` creates the route preview.
- `ring13_four_value_20260908.sh` rebuilds the mission, checks all four speed
  sections, runs stationary pre-flight and the known-start gate, starts the
  field logger, and launches Pure Pursuit with a 1.50 m/s maximum.
- `tractor_teensy/src/teensy_main_20260908.cpp` contains the matching Auto
  speed-to-JRK table. `tractor_teensy/platformio.ini` selects this file.
- `field_testing/tools/analyze_ring13_four_value_retest_20260908.py` contains
  the matching expected-target checks for analyzing the completed retest.

## Build without driving

```bash
./ring13_four_value_20260908.sh --build-only
```

This must report 506 rows and the approved four-value schedule.

## Run

```bash
./ring13_four_value_20260908.sh
```

The launcher requires RTK Fixed, fixed heading carrier, valid heading, a start
within 1.5 m of the reviewed position, and heading within 20 degrees of the
reviewed direction. It automatically starts and stops the field logger; do not
start a separate logger.

This remains a directly supervised, blades-off calibration test. Remain ready
to select Pause or use the e-stop immediately. Investigate or correct the
radio-loss behavior observed in the preceding run before treating the new
higher-speed result as a production calibration.
