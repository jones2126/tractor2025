# Heading carrier history comparison — 2026-09-16

This report uses time-weighted carrier state from the field telemetry CSVs. It excludes derived analysis CSVs and exact duplicate copies. Large logger gaps are not attributed to the last state. `Moving` means `abs(speed_mps) >= 0.10 m/s`.

## Finding

Today's whole-run carrier was Fixed 97.37% of known time, versus 99.18% across the nine AT340-era runs from 2026-08-30 through 2026-09-15.

While moving, today was Fixed 96.03%, versus 99.26% historically. The moving non-Fixed rate was therefore about 5.4 times the historical aggregate. Today ranks 2 of 10 from worst to best by moving Fixed percentage.

Today's two non-Fixed episodes totaled 35.11 seconds and were both Float. The longest was 30.60 seconds, compared with a previous AT340-era maximum of 5.01 seconds. Only 5.66 seconds of today's non-Fixed time occurred while moving.

Conclusion: today was noticeably worse than the normal post-AT340 baseline, especially because of one sustained Float interval while mostly stationary. It was not unprecedented: the 2026-08-30 AT340 perimeter had a lower moving Fixed percentage.

## Same-day raw receiver audits

These two 120-second stationary observations were recorded in the investigation handoff. They were made under trees, and the full raw JSON files are not present on this computer, so individual post-change episode lengths cannot be reconstructed here.

| Stage | Frames | Carrier Fixed | Carrier Float | Other heading-invalid frames | Approx. Float time |
|---|---:|---:|---:|---:|---:|
| Before Heading USB NMEA cleanup | 599 | 599 (100%) | 0 | 2 Fixed-carrier, zero-baseline frames | 0.00 s |
| After Heading USB NMEA cleanup | 598 | 502 (83.946%) | 96 | not reported | 19.27 s |

The after-cleanup carrier snapshot was clearly worse than the before-cleanup snapshot, but this pair does not prove that disabling NMEA caused the change: the tests were sequential rather than simultaneous, the tractor was under trees, and carrier performance can change quickly with multipath and satellite geometry. The field mission CSV analyzed above was recorded before the NMEA-only change.

## Comparable recent field runs

| Date | Run | Era | Observed (s) | Fixed | Non-fixed (s) | Episodes | Longest (s) | Moving fixed | Moving non-fixed (s) |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| 20260829 | `field_test_20260829_heading_perimeter_132809` | original heading antenna | 553.9 | 55.16% | 248.37 | 13 | 81.67 | 49.02% | 155.73 |
| 20260830 | `combined_14ring_21stripe_20260830_150229` | AT340 heading antenna | 382.9 | 100.00% | 0.00 | 0 | 0.00 | 100.00% | 0.00 |
| 20260830 | `combined_14ring_21stripe_20260830_152252` | AT340 heading antenna | 1900.1 | 99.63% | 7.06 | 3 | 3.99 | 99.88% | 2.00 |
| 20260830 | `field_test_20260830_heading_at340_perimeter_1` | AT340 heading antenna | 502.8 | 97.10% | 14.57 | 6 | 2.92 | 95.44% | 14.57 |
| 20260831 | `ring13_speed_settings_20260831_171507` | AT340 heading antenna | 883.2 | 100.00% | 0.00 | 0 | 0.00 | 100.00% | 0.00 |
| 20260831 | `ring14_four_row_skip_20260831_163136` | AT340 heading antenna | 893.9 | 99.65% | 3.12 | 3 | 1.52 | 99.59% | 2.60 |
| 20260908 | `62_collins_polygon_2_20260908` | AT340 heading antenna | 612.7 | 99.05% | 5.83 | 4 | 5.01 | 99.79% | 0.82 |
| 20260908 | `ring13_four_value_20260908_123620` | AT340 heading antenna | 229.2 | 96.88% | 7.16 | 3 | 3.98 | 97.35% | 5.44 |
| 20260908 | `ring13_three_speed_20260908_110712` | AT340 heading antenna | 292.6 | 99.73% | 0.79 | 1 | 0.79 | 100.00% | 0.00 |
| 20260915 | `field_test_20260915_133923` | AT340 heading antenna | 1689.2 | 98.71% | 21.71 | 11 | 3.39 | 98.70% | 17.31 |
| 20260916 | `partial_rings_master_20260916_124854` | 2026-09-16 field session | 1333.8 | 97.37% | 35.11 | 2 | 30.60 | 96.03% | 5.66 |

## Earlier runs (different antenna/coax investigation period)

These are retained for history, but they are not a clean baseline for today's AT340/current-hardware comparison.

| Date | Run | Observed (s) | Fixed | Non-fixed (s) | Episodes | Longest (s) |
|---|---|---:|---:|---:|---:|---:|
| 20260727 | `62_Collins_polygon_1_20260727_144657` | 3540.1 | 96.83% | 112.28 | 54 | 9.22 |
| 20260803 | `62_Collins_polygon_1_20260803_124324` | 273.7 | 98.41% | 4.35 | 5 | 1.26 |
| 20260803 | `62_Collins_polygon_1_20260803_135310` | 437.9 | 84.29% | 68.80 | 25 | 39.19 |
| 20260804 | `00_boundary_log_20260804_115620` | 525.1 | 93.64% | 33.39 | 13 | 8.55 |
| 20260804 | `north_steering_1E_20260804_122428` | 282.1 | 96.39% | 10.19 | 5 | 5.79 |
| 20260804 | `north_steering_1W_20260804_123208` | 119.6 | 100.00% | 0.00 | 0 | 0.00 |
| 20260804 | `north_steering_2E85_20260804_131151` | 218.5 | 100.00% | 0.00 | 0 | 0.00 |
| 20260804 | `north_steering_2E_20260804_124248` | 192.3 | 92.39% | 14.64 | 8 | 3.20 |
| 20260804 | `north_steering_2W85_20260804_131735` | 132.5 | 100.00% | 0.00 | 0 | 0.00 |
| 20260804 | `north_steering_2W_20260804_124806` | 143.9 | 92.59% | 10.66 | 6 | 2.15 |
| 20260804 | `polygon_1_inner_stripes_20260804_144018` | 1213.1 | 88.38% | 140.99 | 36 | 34.99 |
| 20260805 | `headvalid_straight_15m_20260805_114856` | 83.2 | 63.18% | 30.62 | 4 | 18.28 |
| 20260805 | `headvalid_straight_20m_mower_20260805_120253` | 58.4 | 87.73% | 7.16 | 4 | 2.47 |
| 20260805 | `manual_drive_after_heading_coax_tighten_20260805` | 149.5 | 100.00% | 0.00 | 0 | 0.00 |
| 20260805 | `polygon_1_inner_stripes_20260805_111715` | 312.0 | 94.38% | 17.54 | 3 | 12.80 |
| 20260805 | `stationary_engine_on_20260805` | 59.5 | 95.07% | 2.93 | 1 | 2.93 |
| 20260805 | `stationary_engine_transition_20260805` | 89.4 | 96.79% | 2.87 | 2 | 1.52 |
| 20260805 | `stationary_mower_off_jrk_recovery_20260805` | 29.5 | 100.00% | 0.00 | 0 | 0.00 |
| 20260805 | `stationary_mower_off_on_120s_after_coax_tighten_20260805` | 119.5 | 100.00% | 0.00 | 0 | 0.00 |

## Exclusions and caveats

Exact duplicate telemetry copies excluded from comparisons:

- `field_testing\sites\62_Collins_polygon_1\runs\20260727_144659\field_test_20260727_144659.csv` duplicates `field_testing\sites\62_Collins_polygon_1\runs\20260727_144659\62_Collins_polygon_1_20260727_144657.csv`

- `heading_comparison_samples_20260828.csv` files are derived samples, not independent runs.
- The pure-pursuit log is a second view of today's same session, not another run.
- Fixed percentage is computed over known carrier time (`fixed`, `float`, or `none`).
- Non-fixed time combines `float` and `none`; the detailed CSV keeps those durations separate.
- A full-run percentage can be affected by where the tractor was parked and acquisition time. The moving columns are the fairer measure of field performance.

Detailed machine-readable results: `heading_carrier_history_20260916.csv`
