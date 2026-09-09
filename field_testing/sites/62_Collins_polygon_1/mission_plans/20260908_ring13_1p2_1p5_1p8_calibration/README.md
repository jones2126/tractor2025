# Ring 13 1.2/1.5/1.8 m/s calibration — 2026-09-08

This folder prepares an optional supervised follow-up to the completed
four-value Ring 13 run. It preserves the reviewed transit and three-lap
geometry while testing revised 1.2 and 1.5 m/s settings and a first 1.8 m/s
setting.

## Calibration evidence

The completed four-value run is archived under:

`field_testing/sites/62_Collins_polygon_1/runs/20260908_ring13_four_value_123620`

| Command | Tested JRK target | Actual steady median | Result |
|---:|---:|---:|---:|
| 0.75 m/s | 2350 | 0.820 m/s | 9.3% fast |
| 1.00 m/s | 2300 | 1.017 m/s | 1.7% fast |
| 1.20 m/s | 2246 | 1.164 m/s | 3.0% slow |
| 1.50 m/s | 2160 | 1.620 m/s | 8.0% fast |

The prior three-speed run also measured JRK 2200 at 1.344 m/s. Interpolation
inside the directly measured 2200-to-2160 bracket gives approximately 2178
for 1.50 m/s. Interpolation between the current 1.00 and 1.20 points gives
approximately 2233 for 1.20 m/s. Extending the local 2200-to-2160 response
slope from the measured 1.620 m/s point gives approximately 2135 for the first
1.80 m/s test.

## Mission schedule

| Section | Mission rows | Command speed | Teensy JRK target |
|---|---:|---:|---:|
| Transit | 1–66 | 0.75 m/s | 2368 |
| Lap 1 | 67–212 | 1.20 m/s | 2233 |
| Lap 2 | 213–359 | 1.50 m/s | 2178 |
| Lap 3 | 360–506 | 1.80 m/s | 2135 |

The transit is not a calibration lap. Its target is revised because JRK 2350
produced 0.820 m/s in the completed run.

## Soft mechanical limit and current evidence

Lower JRK numbers move the transmission farther forward. Target 1880, with
feedback around 1888, is the lowest value previously achieved. This was a
guarded, non-moving tractor test. Treat 1880 as a soft mechanical limit, not
an approved driving target and not proof that the actuator contacted its
physical stop. Target 2135 remains 255 counts above it.

The dedicated stationary JRK tests near the proposed targets measured:

| Nearby tested target | Average movement rate | Average moving current | Peak current evidence |
|---:|---:|---:|---:|
| 2236 | 219 counts/s | 1.256 A | 2.251 A maximum across trials |
| 2196 | 215 counts/s | 1.484 A | 2.189 A maximum across trials |
| 2156 | 209 counts/s | 1.345 A | 1.905 A maximum across trials |
| 2116 | 206 counts/s | 1.596 A | 2.431 A maximum across trials |
| 1880 | 186 counts/s | 2.165 A | 2.383 A peak |

These are short, approximately 40-count stationary actuator steps. They are
not current measurements from the completed moving mission. That mission's
`jrk_current` column is position feedback, despite its historical name. The
new `teensy_main_20260908_1p8_test` firmware adds actual JRK motor-current and
recent-peak-current fields in mA for the next field log.

## Geometry and safeguards

The geometry source remains the validated 2026-08-31 mission with SHA-256:

`7546efab90c6d6a1045342ed9a3b30ec306a82e404c68d1314bc3f31c907fa60`

The builder preserves latitude, longitude, heading, and lookahead exactly.
Transit lookahead remains 1.50 m and ring lookahead remains 2.00 m. The route
contains 506 points and is approximately 236 m long.

The launcher requires firmware identity
`teensy_main_20260908_1p8_test`, validates current telemetry during preflight,
checks the known starting position and heading, starts the field logger, and
caps Pure Pursuit at 1.80 m/s.

`field_testing/tools/analyze_ring13_1p2_1p5_1p8_20260908.py` checks the exact
speed-to-target mapping and summarizes GPS speed, JRK feedback, actual motor
current, recent peak current, duty, errors, timeouts, and radio-loss mode.

## Build without driving

```bash
./ring13_1p2_1p5_1p8_20260908.sh --build-only
```

## Run

```bash
./ring13_1p2_1p5_1p8_20260908.sh
```

This is an optional, directly supervised, blades-off test. The 1.80 m/s lap is
a first moving test. Remain beside Pause and the e-stop, and abort for abnormal
current, transmission response, heading loss, tracking, or stopping margin.
