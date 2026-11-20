# Encoder fit using Al’s selected sections

November 20, 2026. REVIEW ONLY; no runtime changes.

All eight sections are retained exactly as selected. RTK endpoint displacement is accepted as the travel reference for these basically straight sections, per Al’s instruction. The 16,384 ticks per magnet revolution constant is unchanged. No physical tire circumference or gearing is inferred.

| Side | Fitted ticks/metre | Metres/tick | Effective metres/magnet revolution | Fit RMSE | Leave-one-section-out RMSE |
|---|---:|---:|---:|---:|---:|
| L | 11324.463 | 0.000088304 | 1.446779 | 0.241 m | 0.282 m |
| R | 11303.422 | 0.000088469 | 1.449473 | 0.239 m | 0.274 m |

Total reference distance: 140.782 m across eight sections.

Fit `ticks = ticks_per_metre * GPS_distance` through zero, separately for each side: `ticks_per_metre = SUM(distance * ticks) / SUM(distance^2)`. This treats the accepted GPS distances as predictors and minimizes squared tick residuals, rather than averaging section ratios equally. Longer sections have more influence. Signed ticks remain positive for forward; the existing firmware has already applied the left sensor sign.

Use `distance_m = signed_ticks / ticks_per_metre` and `speed_mps = distance_m / elapsed_seconds`. The equivalent metres/magnet-revolution value can be used in an older formula, but should not be called a measured wheel circumference.

## Per-section distance checks using the all-eight fit

| Section | EST start–end | RTK distance | Left error | Right error |
|---|---|---:|---:|---:|
| 1 | 11:04:47–11:05:07 | 20.109 m | -0.344 m | -0.234 m |
| 2 | 11:05:12–11:05:25 | 13.645 m | -0.108 m | -0.309 m |
| 3 | 11:05:29–11:05:50 | 20.463 m | +0.356 m | +0.338 m |
| 4 | 11:05:53–11:06:08 | 15.052 m | +0.208 m | +0.004 m |
| 5 | 11:07:13–11:07:28 | 15.792 m | -0.188 m | -0.175 m |
| 6 | 11:07:32–11:07:52 | 21.522 m | -0.246 m | -0.150 m |
| 7 | 11:07:56–11:08:12 | 15.980 m | +0.217 m | +0.354 m |
| 8 | 11:08:17–11:08:35 | 18.220 m | +0.153 m | +0.126 m |

Errors are encoder-predicted minus GPS distance. All-eight fit errors are in-sample.

## Whole-section validation

- L: fit first four at 11342.846 ticks/m, then predict the later four: RMSE 0.209 m, maximum absolute error 0.280 m. Fitting the later four alone gives 11306.986 ticks/m.
- L: leave each whole section out, fit the other seven, predict the omitted section: RMSE 0.282 m, maximum absolute error 0.427 m. Refit scale range 11285.545–11361.251 ticks/m.
- R: fit first four at 11285.643 ticks/m, then predict the later four: RMSE 0.226 m, maximum absolute error 0.379 m. Fitting the later four alone gives 11320.325 ticks/m.
- R: leave each whole section out, fit the other seven, predict the omitted section: RMSE 0.274 m, maximum absolute error 0.406 m. Refit scale range 11266.531–11328.343 ticks/m.

The final candidate uses all eight. The first-four/later-four test checks chronological separation; leave-one-out checks section influence but its training sets overlap. These are same-run forward sections, not independent runs or reverse validation. No population confidence interval or short-outage pose accuracy is established by this fit.

Recorded heading/intermediate-position flags are preserved in fit_results.json rather than used to discard the selected sections. All start/end position brackets were previously checked as RTK Fixed, including section 8; interior loss does not automatically invalidate endpoint displacement. Exact receiver epochs remain unavailable, but this fit uses the agreed host-time alignment. Wheel boundary fractions, source counts, position interpolation and original flags are preserved in the input extraction.

This package is separate from the earlier strict-window fit. No services, firmware, raw transport, reviewed-calibration flag or motion authorization are changed. Next use is offline wheel/IMU pose prediction with these empirical constants.

Reproduce from the repository root (the package retains the exact extraction input as selected_sections.json):

```powershell
python field_testing/tools/fit_user_encoder_sections_20261120.py --input field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_user_selected_encoder_fit/selected_sections.json --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_user_selected_encoder_fit
```
