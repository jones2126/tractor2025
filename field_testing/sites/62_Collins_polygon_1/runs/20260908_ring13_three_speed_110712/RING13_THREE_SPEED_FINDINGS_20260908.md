# Ring 13 three-speed calibration findings — 2026-09-08

## Result

The mission reached its goal and both loggers closed cleanly. The 1.00 m/s and
1.25 m/s settings were close to their commanded speeds. The 1.50 m/s setting
did not reach 1.50 m/s: JRK target 2200 produced a steady median GPS speed of
1.344 m/s.

The 0.75 m/s transit was also much slower than commanded. JRK target 2429
produced a steady median of 0.361 m/s. This point is near a strongly nonlinear
part of the transmission response and should be repeated before changing the
low-speed calibration.

## Data integrity and method

- Tractor repository revision during the run: `7545260`
- Field log: `ring13_three_speed_20260908_110712.csv`
  - 5,849 data rows
  - SHA-256: `422096e2155f35c26a70b4a5fd7e014cf6db9ec1874147005ee454989d777202`
- Pursuit log: `pursuit_log_20260908_110714.csv`
  - 5,813 data rows plus two descriptive/header rows
  - SHA-256: `50cef9fd1e6afd7780eeb7f7da9678e2e0e4a036cec8d0631a315d8644721961`
- Exact generated mission: `62_Collins_ring13_three_speed_settings_20260907.txt`
  - SHA-256: `b6784262a5e5096937faa0c945c2738b5068cc2e24fcff5092c974eb9041dc09`

For each commanded-speed section, the analysis retained only samples where:

- the Teensy was in Auto;
- the requested and JRK-reported actual targets both equaled the intended
  target;
- JRK telemetry was valid; and
- GPS speed was finite.

Five seconds were removed from both ends of every remaining continuous section
to exclude actuator movement, acceleration, deceleration, and safety-stop
recovery. The 0.75 m/s section is the start transit; the other three sections
are the three Ring 13 laps.

## Commanded speed versus actual GPS speed

| Mission section | Command (m/s) | JRK target | Steady GPS median (m/s) | Mean (m/s) | 5th–95th percentile (m/s) | Median error | Steady data |
|---|---:|---:|---:|---:|---:|---:|---:|
| Transit | 0.75 | 2429 | 0.361 | 0.355 | 0.266–0.424 | −51.9% | 31.5 s |
| Lap 1 | 1.00 | 2288 | 1.041 | 1.035 | 0.924–1.144 | +4.1% | 45.0 s |
| Lap 2 | 1.25 | 2240 | 1.223 | 1.214 | 1.080–1.328 | −2.2% | 37.8 s |
| Lap 3 | 1.50 | 2200 | 1.344 | 1.339 | 1.196–1.472 | −10.4% | 30.1 s |

The 1.50 m/s lap briefly exceeded 1.50 m/s on some downhill samples, reaching
1.586 m/s, but its steady median and mean remained near 1.34 m/s.

## JRK target, feedback, and PWM interpretation

| Command (m/s) | Requested target | JRK actual target | Median scaled feedback | Feedback minus target | Steady applied duty |
|---:|---:|---:|---:|---:|---:|
| 0.75 | 2429 | 2429 | 2424 | −5 | 0 |
| 1.00 | 2288 | 2288 | 2284 | −4 | 0 |
| 1.25 | 2240 | 2240 | 2236 | −4 | 0 |
| 1.50 | 2200 | 2200 | 2199 | −1 | 0 |

The Teensy does not command a continuous propulsion PWM. It sends the JRK a
position target. The JRK briefly applies PWM to the linear actuator to move the
transmission linkage, then the applied duty returns to zero after the feedback
position is reached. The linkage mechanically holds the selected transmission
position while the tractor continues moving.

Observed initial actuator movements were:

- target 2429: applied duty reached −600 (100% in that configured direction);
- target 2288: applied duty reached −600;
- target 2240: applied duty reached −460 (approximately 76.7%);
- target 2200: the 5 Hz JRK snapshot missed the short initial PWM pulse, but
  feedback moved from 2236 to 2202 in approximately 2.2 seconds. Later
  safety-return and recovery movements reached both −600 and +600.

The JRK's `duty cycle target` can exceed ±600 because it is the raw PID demand.
The applied `duty cycle` is limited to ±600, where magnitude 600 is 100%.
These definitions follow the
[Pololu Jrk G2 variable reference](https://www.pololu.com/docs/0J73/10).

The field named `jrk_current` is not electrical current; it is raw JRK feedback
position. This run therefore does not contain actuator current in amperes.

## Safety and data-quality observations

- JRK telemetry was valid in all 5,849 field samples.
- JRK halting-error bits remained zero.
- The JRK timeout counter did not increase.
- No steering fault latched and no steering drive block occurred.
- RTK position remained `RTK Fixed` throughout the field log.
- Heading was invalid in 61 field samples, about 3.05 seconds total. Pure
  Pursuit withheld driving commands during its corresponding invalid-heading
  samples, and the mission resumed when heading recovered.
- The Teensy entered radio-loss safety mode and commanded neutral for about
  0.6 seconds during the 1.25 m/s lap and about 31.6 seconds during the 1.50
  m/s lap. The latter is the long stop visible from approximately 225 to 256
  seconds. The operator/system recovered and the mission reached its goal.

The radio-loss events are separate from the brief heading-invalid stops and
deserve investigation before relying on an uninterrupted higher-speed run.

## Calibration implication

Using only the two new upper points, JRK 2240 → 1.223 m/s and JRK 2200 →
1.344 m/s, a linear extrapolation estimates that approximately JRK 2149 would
produce 1.50 m/s under the same conditions. This is an estimate, not an
approved production setting. Because the 1.50 m/s lap was interrupted and the
route has meaningful slope effects, the next supervised test should use a
smaller reviewed step, such as JRK 2160, before considering approximately
2149.

Do not update the production calibration from this single interrupted lap.
First investigate the radio interruption, then confirm the next target with
the same blades-off Ring 13 procedure.

## Generated analysis files

- `analysis/ring13_three_speed_summary_20260908.csv`
- `analysis/ring13_three_speed_analysis_20260908.json`
- `analysis/ring13_three_speed_speed_vs_command_20260908.png`

The analysis is reproducible with
`field_testing/tools/analyze_ring13_three_speed_20260908.py`.
