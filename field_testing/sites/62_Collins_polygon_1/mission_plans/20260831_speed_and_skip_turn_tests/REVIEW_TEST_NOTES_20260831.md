# 2026-08-31 speed and skip-row REVIEW_TEST notes

These are blades-off, directly supervised test missions. Both missions pass the
independent static mission validator, but neither is approved for unattended or
blades-on operation.

## 1. Ring 13 speed-settings mission

Mission:

`01_speed_settings/62_Collins_ring13_four_speed_settings_REVIEW_TEST_20260831.txt`

The route is four clockwise laps on a smoothed version of ring 13, the ring
immediately outside the innermost ring used in the 14-ring plan.

| Lap | Zero-based waypoints | AUTO command | Current firmware JRK target |
|---:|---:|---:|---:|
| 1 | 0-146 | 0.75 m/s | 2429 |
| 2 | 147-293 | 0.94 m/s | 2404 |
| 3 | 294-440 | 1.08 m/s | 2370 |
| 4 | 441-587 | 1.25 m/s | 2288 |

- Lap length: approximately 73.39 m
- Lookahead: 2.0 m
- Start: `40.485547965, -80.332483547`
- Start heading: approximately 134 degrees compass
- Validation: `PASS`; minimum sampled radius 2.36 m
- Mission SHA-256: `78f78e33def337c316d808d5b8f7d0a140bc6888459e634ad605e29c173c4e6c`

The four commands deliberately exercise JRK targets 2429, 2404, 2370, and
2288. Record median actual speed separately for each lap after excluding the
first and last five seconds of the lap. Also record uphill/downhill medians if
the slope creates a meaningful difference.

JRK target 2288 is the confirmed full-forward endpoint and produced about
1.0 m/s in the 2026-08-30 run. A physical 1.25 m/s target is therefore not
currently available without changing the confirmed transmission range. Do not
command beyond JRK 2288 during this calibration.

After the run, fit the firmware `SPEED_CAL` table using measured
`{actual_mps, jrk_target}` pairs. The desired operating points are physical
speeds of approximately 0.50, 1.00, and, only if the hardware safely supports
it, 1.25 m/s.

## 2. Four-row-skip combined mission

Mission:

`02_four_row_skip/62_Collins_combined_14ring_four_row_skip_REVIEW_TEST_20260831.txt`

This shortened mission starts at the same field position and heading used on
2026-08-30. A contained 16.45 m RSL transit takes the tractor directly to the
beginning of ring 14. It drives ring 14 once, omits rings 1-13, then enters the
21 core stripes in this order:

`1, 5, 9, 13, 17, 21, 4, 8, 12, 16, 20, 3, 7, 11, 15, 19, 2, 6, 10, 14, 18`

Four 38-inch row spacings equal 3.8608 m, slightly more than the 3.80 m
diameter of a 1.90 m-radius turn. That is why this pattern skips three
intermediate rows rather than one or two. All 20 stripe connectors are simple
two-arc LSL/RSR paths; none is an RLR/LRL keyhole.

- Waypoints: 1,208
- Route length: 553.6 m
- Original spiral source index for ring 14: 2,859
- Ring 14 source waypoints retained: 117
- Original-start-to-ring-14 transit: 16.45 m RSL, contained in the site
- Turn radius: 1.90 m
- Turn lookahead: 1.5 m
- Stripe lookahead: 2.0 m
- Current command speed: 1.25 m/s throughout, producing about 1.0 m/s actual
  in the latest field evidence
- Start: same as the 2026-08-30 combined mission
- Validation: `PASS`; minimum sampled radius 1.89 m; route contained in site
- Mission SHA-256: `5a3346b8887449876883b244d134a324819127dd8b29ee9bdfac269614a13dc1`

Three modulo-wrap relocations are intentionally longer: rows 21-to-4,
20-to-3, and 19-to-2. The first two cross a portion of the core. That is
acceptable for a blades-off steering test, but it would leave diagonal cut
marks during a production mowing run. Review those relocations after the field
test before converting this into a blades-on mission.

Do the speed-settings run first. Once the 0.50 m/s physical operating point is
known, rebuild the skip-row mission with that calibrated command on the turns.

Launch the shortened test on the tractor with:

`02_four_row_skip/four_row_skip_20260831.sh`
