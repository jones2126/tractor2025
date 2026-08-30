# AT340 Heading F9P perimeter analysis — 2026-08-30

## Selected perimeter

- Source: `field_test_20260830_heading_at340_perimeter_1.csv`
- Start: 184.27 seconds, immediately after the deliberate start pause
- End: 401.31 seconds, at the beginning of the deliberate end pause
- Duration: 217.04 seconds
- CSV data rows: 3,686 through 8,027 (header not counted)
- Start/end pause centroids were approximately 0.70 m apart
- Position fixes: 98.64% RTK Fixed, 1.27% RTK Float, 0.09% DGPS

## Carrier result

| Heading carrier | Rows | Share |
|---|---:|---:|
| Fixed | 4,146 | 95.49% |
| Float | 0 | 0.00% |
| None | 196 | 4.51% |

The four non-Fixed episodes totaled approximately 9.54 seconds. They were all
`none`; the AT340 run contained no Float carrier episode.

## Non-Fixed episodes

| Episode | Elapsed time (s) | Duration (s) | Approx. local position (m) | Median speed (m/s) | Heading C/N0 median (dB-Hz) | Heading SV used median | Position state | Head valid | Straight-sample median / p95 heading error |
|---|---:|---:|---:|---:|---:|---:|---|---|---:|
| 1 | 258.40–260.92 | 2.52 | (-5.2, -59.9) | 0.634 | 43.7 | 32 | 100% RTK Fixed | 44/52 rows | 7.00° / 8.02° |
| 2 | 261.87–264.12 | 2.25 | (-7.2, -60.2) | 0.666 | 42.8 | 32 | 44 RTK Fixed, 2 DGPS | 42/46 rows | 1.33° / 2.72° |
| 3 | 290.87–293.72 | 2.85 | (-27.6, -63.7) | 1.004 | 41.1 | 32 | 100% RTK Fixed | 54/58 rows | 9.38° / 15.97° |
| 4 | 302.80–304.72 | 1.92 | (-36.1, -56.1) | 1.002 | 41.6 | 32 | 100% RTK Fixed | 40/40 rows | 3.07° / 3.86° |

All four episodes occurred within a 46-second portion of the southern perimeter.
They did not coincide with weak averaged signal strength, low satellite count,
slow/zero motion, or a differential-correction outage. Median differential age
was 1.3–1.4 seconds and the maximum was 1.8 seconds.

## Straight-motion heading comparison

The comparison uses a 3 m position baseline, at least 0.25 m/s, no more than
12° of course change over the baseline, RTK Fixed position, and valid F9P
heading.

| Carrier | Samples | Median absolute error | p95 absolute error | Maximum absolute error |
|---|---:|---:|---:|---:|
| Fixed | 1,606 | 2.05° | 8.98° | 17.04° |
| None | 74 | 3.69° | 14.35° | 16.80° |

The `none` state modestly degraded the heading comparison but did not reproduce
the very large reversals seen with the original antenna's Float solution.

## Satellite comparison during the selected perimeter

| Metric | Base-Link median | AT340 Heading median |
|---|---:|---:|
| Satellites used | 31 | 32 |
| Satellites visible | 49 | 50 |
| Mean C/N0 | 40.0 dB-Hz | 41.3 dB-Hz |

During straight samples with carrier `none`, Heading still used a median of 32
satellites and had a median mean C/N0 of 41.6 dB-Hz. This is evidence against
weak aggregate reception as the cause of the remaining brief dropouts.

## Comparison with original Heading antenna run

| Metric | Original antenna, 2026-08-29 | AT340, 2026-08-30 |
|---|---:|---:|
| Carrier Fixed share | 43.6% | 95.49% |
| Carrier Float share | 48.6% | 0.00% |
| Carrier None share | 7.8% | 4.51% |
| Heading satellites used, median | 23 | 32 |
| Heading mean C/N0, median | 18.7 dB-Hz | 41.3 dB-Hz |
| Fixed median absolute heading error | 2.74° | 2.05° |
| Float median absolute heading error | 33.26° | no Float samples |

## Interpretation

The AT340 removed the persistent Float behavior and closed the approximately
19 dB signal-quality deficit between Heading and Base-Link. The original
Heading antenna is therefore the leading cause of the 2026-08-29 behavior.

The remaining four `none` episodes are short, geographically clustered, and
occurred despite strong satellite measurements. Possible causes include local
multipath/obstruction, a brief carrier-phase cycle slip, or movement of the
temporary antenna/adapter/cable during that portion of the route. The present
log does not distinguish those mechanisms. Secure the AT340 and its adapter,
repeat the same route, and check whether any new episodes recur at the same
southern locations.
