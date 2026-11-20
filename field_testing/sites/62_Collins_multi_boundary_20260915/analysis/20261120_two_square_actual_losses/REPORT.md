# Two-square real position-loss replay

Offline development check, November 20, 2026. Uses the all-eight operator-selected scales; no runtime changes.

Prediction starts at the last Fixed position before a non-Fixed episode and ends at the first recovered Fixed position. Heading is anchored to the last healthy independent heading decode before prediction, then advanced with recorded fixed-bias gyro yaw. Both signed wheel streams are integrated over common host-time intervals with midpoint heading. No GNSS position or heading correction is applied inside a prediction interval, even if heading remains healthy. This is a conservative no-correction comparison, not the full proposed selective-correction estimator.

| Last Fixed / prediction start (EST) | First non-Fixed | First recovered Fixed (EST) | Prediction duration | Recovery disagreement |
|---|---|---|---:|---:|
| 11:08:14.093 | 11:08:14.254 | 11:08:14.898 | 0.805 s | 0.025 m |
| 11:08:21.262 | 11:08:21.444 | 11:08:26.899 | 5.636 s | 0.118 m |
| 11:08:27.676 | 11:08:28.068 | 11:08:30.892 | 3.216 s | 0.022 m |

These durations include the last-good-to-first-bad and last-bad-to-first-good sampling brackets; they are not exact receiver outage durations. See JSON for first/last non-Fixed decode times and sample counts. Position non-Fixed episodes and heading-only losses are different; this table inventories position loss only.

## Nearby healthy combined-loss masks

| Start (EST) | Actual prediction duration | Endpoint disagreement |
|---|---:|---:|
| 11:07:35 | — | reference_not_healthy |
| 11:07:40.074 | 3.001 s | 0.034 m |
| 11:07:45.075 | 5.010 s | 0.059 m |

Real losses let us check that integration continues and compare against recovered Fixed GPS before a correction is applied. They cannot reveal the true path error while GPS is non-Fixed. Healthy masks retain a trustworthy reference throughout; this first check reports endpoint disagreement, with full trajectory scoring still to implement.

All results are same-run development checks. The scales use all eight selected sections; section 8 overlaps actual position loss, so recovery closure is partly calibration-dependent and is not independent validation. Recorded antenna positions are compared against a wheel/heading prediction without measured lever-arm correction, especially important in turns. The estimator is an offline calculation, not a commissioned production-model implementation. Raw evidence, transport, safety watchdogs and motion authorization remain unchanged.

Reproduce from the repository root:

```powershell
python field_testing/tools/replay_two_square_position_losses_20261120.py --evidence-root field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261119_manual --fit field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_user_selected_encoder_fit/fit_results.json --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_two_square_actual_losses
```
