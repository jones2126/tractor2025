# Ring 13 four-value calibration findings — 2026-09-08

## Sources and scope

- Field log: `ring13_four_value_20260908_123620.csv` — 4,585 samples at
  approximately 20 Hz; SHA-256
  `67873b91e2e87ca8b4ff5a5bc3b5f76ca4c234ab620c65f164088bf69d234668`.
- Pursuit log: `pursuit_log_20260908_123622.csv` — 4,548 usable controller
  cycles at 20 Hz; SHA-256
  `4416b8dd5609303495dcee51016aa2daa53c9f2d5325505d3cb350b2e70d65dd`.
- Flashed firmware identity: `teensy_main_20260908`.
- The 0.75 m/s portion is the approach to Ring 13. The 1.00, 1.20, and
  1.50 m/s portions are consecutive laps of the same ring and form the fair
  tracking and heading comparison.
- Speed estimates below use settled, actively driven intervals with matching
  requested/read-back JRK targets. Five seconds were removed from each end of
  each eligible continuous interval.
- Cross-track statistics use controller cycles when the controller was driving.
  Navigation-validity statistics include both driving and safety-wait cycles.

## 1. Actual speed versus target

| Target speed | JRK target | Steady median | Median error | Target achieved | 5th–95th percentile |
|---:|---:|---:|---:|---:|---:|
| 0.75 m/s | 2350 | 0.820 m/s | +0.070 m/s | 109.3% | 0.755–0.903 m/s |
| 1.00 m/s | 2300 | 1.017 m/s | +0.017 m/s | 101.7% | 0.909–1.105 m/s |
| 1.20 m/s | 2246 | 1.164 m/s | −0.036 m/s | 97.0% | 1.039–1.286 m/s |
| 1.50 m/s | 2160 | 1.620 m/s | +0.120 m/s | 108.0% | 1.509–1.723 m/s |

The 1.00 m/s calibration was the closest. The 1.20 m/s point was modestly
slow. The 0.75 and 1.50 m/s settings were both about 8–9% fast. This confirms
that one linear JRK-target conversion is not adequate across the complete
range.

JRK position feedback was stable: the median feedback differences from the
requested targets were −6, 0, −6, and −4 counts respectively. There were no
JRK halting errors, no timeout increase, and no Teensy radio-loss mode during
this mission. The steady JRK duty was zero after each actuator move, which is
expected for a settled position target and does not represent tractor drive
PWM.

## 2. Cross-track error versus speed

The 0.75 m/s approach is not comparable to the completed ring laps. It began
off the route and had a 0.900 m median cross-track error with steering
saturated for 61.1% of its driving samples. That is an acquisition-geometry
result rather than evidence that 0.75 m/s tracks poorly.

| Intended speed | Actual steady median | Median CTE | Mean CTE | 95th percentile | Maximum | Above 0.50 m |
|---:|---:|---:|---:|---:|---:|---:|
| 1.00 m/s | 1.017 m/s | 0.345 m | 0.386 m | 0.803 m | 1.172 m | 18.3% |
| 1.20 m/s | 1.164 m/s | 0.337 m | 0.387 m | 0.752 m | 1.102 m | 20.1% |
| 1.50 m/s | 1.620 m/s | 0.344 m | 0.390 m | 0.789 m | 1.260 m | 22.3% |

Typical tracking error did not materially worsen with speed. From the 1.00 to
1.50 m/s laps, mean error increased by only 0.004 m, median error decreased by
0.002 m, and the 95th percentile decreased by 0.014 m. After dividing each lap
into 20 matched progress bins, the median 1.50-minus-1.00 m/s difference was
−0.003 m; exactly 10 bins were worse and 10 were better at 1.50 m/s.

The percentage above 0.50 m rose from 18.3% to 22.3%, and the single maximum
was highest at 1.50 m/s. Those tail changes merit watching, but this one run
does not show a general speed-driven loss of path tracking.

All three laps had a persistent negative signed lateral error with medians
near −0.34 m. This common directional bias is much larger than the differences
between speeds and should be investigated separately from speed calibration.

## 3. Heading validity versus speed

| Segment | Heading valid | Invalid time | Invalid events | Longest event | RTK Fixed | Driving cycles |
|---:|---:|---:|---:|---:|---:|---:|
| 0.75 m/s approach | 99.86% | 0.05 s | 1 | 0.05 s | 99.86% | 99.86% |
| 1.00 m/s ring | 96.02% | 3.05 s | 12 | 0.90 s | 97.58% | 94.77% |
| 1.20 m/s ring | 98.36% | 1.00 s | 5 | 0.20 s | 100.00% | 98.36% |
| 1.50 m/s ring | 93.23% | 3.60 s | 7 | 2.45 s | 96.62% | 92.76% |

The 1.50 m/s lap had the weakest heading availability and the longest outage.
The controller responded correctly by withholding drive commands while data
was invalid. The 1.20 m/s lap was better than the 1.00 m/s lap, so the result
is not a monotonic degradation with increasing speed.

The heading receiver's median mean C/N0 was 43.2, 43.4, and 43.1 dB-Hz on the
1.00, 1.20, and 1.50 m/s laps. Median satellites used were 32, 31, and 31.
There is no accompanying signal-strength or satellite-count decline at the
higher speeds. RTK availability also fell on the 1.00 and 1.50 m/s laps, which
suggests that time/location-dependent GNSS conditions remain a plausible
cause.

Some heading-invalid samples appeared in similar middle portions of all three
laps, while the dominant 1.50 m/s outage was around 85–90% of that lap. This
partial spatial clustering argues against assigning the entire effect to
speed or vibration.

## Conclusions

1. JRK 2300 is already a good 1.00 m/s setting. JRK 2246 is slightly slow for
   1.20 m/s. JRK 2350 and 2160 are too fast for 0.75 and 1.50 m/s.
2. The repeated ring laps do not show a meaningful increase in typical
   cross-track error as speed rises. The persistent approximately 0.34 m
   directional bias is the larger tracking issue.
3. Heading availability was worst at the fastest setting, but this run does
   not establish a speed correlation. Only one lap was recorded at each speed,
   the result is non-monotonic, and GNSS availability changed at the same time.
4. A controlled follow-up should repeat multiple laps per speed and alternate
   or randomize the speed order. That would separate speed/vibration effects
   from location and elapsed-time effects.

## Recommended next targets

| Command | Recommended JRK target | Basis |
|---:|---:|---|
| 0.75 m/s transit | 2368 | Correction from 2350 producing 0.820 m/s |
| 1.00 m/s | 2300 | Retain the accurate measured setting |
| 1.20 m/s | 2233 | Interpolation from the measured 1.00/1.20 response |
| 1.50 m/s | 2178 | Interpolation inside the measured 2200/2160 bracket |
| 1.80 m/s | 2135 | Cautious extrapolation of the local 2200/2160 response |

The proposed 1.80 m/s target remains 255 counts above the JRK 1880 soft
mechanical limit. That limit was established with feedback around 1888 during
a guarded, non-moving tractor test.

The completed moving mission did not record JRK motor current in amps. Its
historically named `jrk_current` column is actuator position feedback. The
separate stationary guard tests measured roughly 206–219 feedback counts/s
and approximately 1.26–1.60 A average moving current near the proposed
2233-to-2135 range. Observed peaks at the neighboring tested targets were
1.91–2.43 A. The new follow-up firmware adds actual current and recent peak
current in mA to the field log so those values can be measured during the
next moving mission.
