# Straight-strip steering diagnosis: scanlines 12 and 13

## Result

The IBT-2 was not receiving a continuously increasing correction based on path error. It only received PWM when the steering-pot error exceeded the firmware's 10-count deadband. On the settled straight portions, the controller requested pot setpoints near 373–378 counts and the measured pot was already near 369–371, so the firmware normally commanded PWM 0.

The stronger result is a likely steering-center calibration mismatch. The firmware defines 447 pot counts as straight. During both nearly heading-aligned traversals the settled observed pot median was about 370.0. If that observed value represents physical straight ahead, the present mapping has an implicit normalized bias of -0.308, equivalent to about -11.0 degrees in the controller's ±35.7-degree model. That almost exactly consumes the steady right-steering request seen on these strips.

## Settled straight portions (3 m removed at each end)

| Metric | Scanline 12 | Scanline 13 |
|---|---:|---:|
| Mean geometric CTE | 0.564 m | 0.627 m |
| Mean heading error | -1.38° | -0.13° |
| Mean pursuit `yt` | -0.645 m | -0.638 m |
| Mean requested steering angle | -10.32° | -10.21° |
| Mean normalized steering command | -0.289 | -0.286 |
| Reconstructed mean pot setpoint | 374.7 | 375.5 |

## Available low-level observations

| Metric | Scanline 12 | Scanline 13 |
|---|---:|---:|
| Field-log rows in strip window | 119 | 117 |
| 20 Hz samples with a telemetry match within 150 ms | 102 / 561 | 127 / 536 |
| Settled observed setpoint median | 375.0 | 373.0 |
| Settled observed pot median | 369.0 | 371.0 |
| Settled observed pot-error median | 4.0 | 2.0 |
| Settled matched samples with PWM = 0 | 100.0% | 74.8% |
| Settled matched samples with PWM = 150 | 0.0% | 25.2% |

## Important telemetry limitation

The pursuit calculation is genuinely present at 20 Hz. The July 28 Teensy source is intentionally a 10 Hz steering controller and telemetry producer (`controlSteeringInterval = 100 ms`), so even the intended firmware would provide low-level PID telemetry at 10 Hz rather than 20 Hz.

The field-test hardware was not emitting the July 28 telemetry format. The available rows report only one steering sequence value (`q=0`), and advanced fields such as P/I/D terms and applied left/right PWM remain zero even when the basic PWM field reports 150. The saved bridge journal also shows fresh steering messages arriving only about once every 2.4–2.6 seconds. The July 28 firmware would increment `q` and print a complete record every 100 ms. Therefore the Teensy was almost certainly still running an older compiled program; updating the Raspberry Pi bridge service did not flash the Teensy. The advanced values in this CSV were defaults supplied by the July 28 bridge while it parsed the older Teensy format, so they cannot be used as measured PID internals for this run.

The trustworthy low-level fields here are the basic setpoint, pot current, pot error, direction, and PWM magnitude. The exact current firmware calculation is: setpoint from normalized command → error = setpoint − pot → PWM 0 inside ±10 counts; otherwise `abs(Kp × error)` is raised to the 150 minimum and capped at 255, with Kp = 1.0.

## Interpretation

1. Pure pursuit did see the lateral displacement and requested roughly 10° right steering on the settled sections.
2. The low-level loop reached the requested pot value and then entered its deadband, so PWM correctly dropped to zero under its current rules.
3. Tractor heading nevertheless stayed almost parallel to the strips while the geometric CTE stayed near 0.6 m.
4. The simplest explanation consistent with all four facts is that pot ≈370 was physically near straight, while the firmware/controller mapping treated 447 as straight. The controller's normal correction was spent canceling that offset instead of curving back to the line.

This is a strong inference from the logs, not a substitute for a stationary wheel-center measurement. Confirm the pot reading with the front wheels physically straight before changing firmware constants.
