# Combined REVIEW_TEST Transmission-Speed Analysis

Source field log:
`combined_14ring_21stripe_20260830_150229.csv`

Source controller log:
`pursuit_log_20260830_150231.csv`

## Result

The controller continuously requested 0.75 m/s while driving. In AUTO mode,
the Teensy velocity calibration mapped 0.75 m/s to JRK target 2429. The JRK
reached that position, but the tractor was too slow on the test slope.

When the operator switched to Manual and selected full forward, the radio
selected bucket 9 and JRK target 2288. The JRK also reached that position, and
the tractor maintained approximately 1.0 m/s actual ground speed.

| Measure | AUTO | Manual full forward |
|---|---:|---:|
| Logged interval | 49.18-246.92 s | 251.00-310.50 s |
| Controller `speed_cmd_mps` | 0.75 m/s | AUTO command ignored in Manual |
| Radio transmission raw | approximately 501 | 1 |
| Manual bucket | not applicable | 9 |
| JRK requested target | 2429 | 2288 |
| JRK feedback median | 2422 | 2283 |
| Actual speed median | 0.332 m/s | 1.001 m/s |
| Actual speed p10 | 0.088 m/s | 0.918 m/s |
| Actual speed p90 | 0.495 m/s | 1.084 m/s |
| Actual speed maximum | 0.832 m/s | 1.162 m/s |

Near the end of the AUTO interval, actual median speed fell progressively:

- 216.59-224.50 s: 0.155 m/s;
- 225.59-234.12 s: 0.132 m/s;
- 234.39-236.31 s: 0.057 m/s; and
- 236.60-246.92 s: 0.091 m/s.

This confirms that the AUTO target, rather than failure of the JRK actuator to
reach its target, was insufficient for the slope.

## Interpreting PWM and bucket fields

The transmission is positioned by a Pololu JRK-controlled linear actuator.
`jrk_duty_cycle` is the PWM applied temporarily to move that actuator to the
requested position. It normally returns to zero after the target is reached;
it is not a continuous traction-motor PWM or a reliable proxy for ground
speed.

During steady AUTO target 2429, JRK duty was nonzero in 8.48% of rows. During
steady Manual target 2288, it was nonzero in only 0.34% of rows. In both cases,
the feedback proves the actuator reached the requested position.

The logged AUTO `bucket=5` must not be interpreted as Manual neutral. Firmware
sets the bucket diagnostic to 5 in AUTO while directly interpolating a
continuous JRK target. The authoritative AUTO command was target 2429.

## AUTO command required to reproduce Manual full forward

The Teensy `SPEED_CAL` table maps an AUTO request of 1.25 m/s to JRK target
2288. That is the same target produced by Manual bucket 9. Therefore the next
blades-off REVIEW_TEST should command 1.25 m/s at every waypoint and set the
controller cap to 1.25 m/s.

The 1.25 m/s value is a command/calibration-table value, not the expected
physical speed. Based on this field log, expected actual ground speed is about
1.0 m/s, with observed p10-p90 of 0.918-1.084 m/s over the full-forward Manual
interval.

## Other controller evidence

The pursuit log contained 7,205 driving cycles, all commanding 0.75 m/s. It
also recorded 401 wait cycles for `headValid=False` and 12 wait cycles for RTK
Float. Those heading/fix interruptions should be reviewed separately from the
transmission-speed correction.
