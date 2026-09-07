# Ring 13 Three-Speed Calibration Mission — 2026-09-07

This folder defines the three-lap Ring 13 speed-calibration mission used with
`teensy_main_20260907.cpp`.

## Geometry source

The path geometry is inherited from the already reviewed and field-run mission:

`../20260831_speed_and_skip_turn_tests/01_speed_settings/62_Collins_ring13_four_speed_settings_REVIEW_TEST_20260831.txt`

The builder requires the historical file to retain SHA-256:

`7546efab90c6d6a1045342ed9a3b30ec306a82e404c68d1314bc3f31c907fa60`

No latitude, longitude, heading, or lookahead value is changed. The new mission
keeps the original 66-row transit and the first three complete Ring 13 laps,
then drops the old fourth lap.

## New mission schedule

| Section | 1-based mission rows | Command speed | Teensy 20260907 JRK target |
|---|---:|---:|---:|
| Transit | 1–66 | 0.75 m/s | interpolated from calibration table |
| Lap 1 | 67–212 | 1.00 m/s | 2288 |
| Lap 2 | 213–359 | 1.25 m/s | 2240 |
| Lap 3 | 360–506 | 1.50 m/s | 2200 |

Transit lookahead remains 1.50 m. Ring 13 lookahead remains 2.00 m.
Approximate route length is 236.0 m: 15.86 m transit plus three 73.39 m laps.

## Files

- `build_ring13_three_speed_mission_20260907.py` — deterministically creates
  `62_Collins_ring13_three_speed_settings_20260907.txt` from the reviewed source.
- `ring13_three_speed_20260907.sh` — rebuilds and validates the mission, runs
  the existing preflight/start gate, starts the field logger, and launches Pure
  Pursuit with `--max-speed 1.50`.

## Build without driving

```bash
./ring13_three_speed_20260907.sh --build-only
```

This creates the normal five-column mission `.txt` in this folder and exits
after validating row counts, speed segments, and lookahead values.

## Run

```bash
./ring13_three_speed_20260907.sh
```

The launcher requires the same known start gate used by the prior Ring 13
speed-settings test: RTK Fixed, fixed heading carrier, valid heading, within
1.5 m of the known start, and within 20 degrees of the expected heading.

This remains a supervised blades-off calibration test. The operator should be
ready to select Pause or use the e-stop immediately if steering, transmission,
RTK/heading, or route tracking is abnormal.
