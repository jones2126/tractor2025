# November 19 two-square run: encoder ticks-per-metre calibration

Recorded November 20, 2026. All run clock times below are **November 19, Eastern Standard Time (UTC−05:00)**.

## Decision and current working constants

Al selected eight basically straight sections from the actual two-square manual run and accepted their RTK endpoint distances as the travel reference. The objective is empirical wheel distance/speed to support pose prediction for a few seconds when primary observations degrade. Explaining the physical circumference discrepancy is not a prerequisite for proceeding with offline pose work.

Fit each side independently using **all eight selected sections**:

| Quantity | Left | Right |
|---|---:|---:|
| Working ticks/metre | **11324.463159335863** | **11303.42226070444** |
| Metres/tick | 0.00008830440665751138 | 0.00008846878201449046 |
| Equivalent metres/magnet revolution | 1.4467793986766664 | 1.4494725245254116 |
| Encoder USB serial | 5879620 | F5CB3C7E51514A4E43202020FF142006 |

The physical **16,384 ticks per magnet revolution** remains unchanged. The previous `1.59593` multiplier was nominal 20-inch circumference, equivalent to about 10266.114 ticks/metre. The empirical calibration estimates ground travel per magnet revolution; it does not establish a measured tire circumference or sensor-to-wheel gear ratio. The left firmware already reverses the magnetic sensor sign; both recorded streams are positive forward. Do not reverse the left sign again.

Use:

```text
left_distance_m  = signed_left_ticks  / 11324.463159335863
right_distance_m = signed_right_ticks / 11303.42226070444
speed_mps       = distance_m / elapsed_seconds
```

These are the working empirical constants for the next offline pose implementation. **Not installed or commissioned**: the saved fit retains `reviewed_calibration=false`, `valid_for_control=false`; no producer service, firmware, controller input or motion authority was changed. Al accepted the direct ticks/metre approach; this is not an approval to deploy or move the tractor.

## Actual run and evidence lineage

The November 19 run used the consolidated producer and continuous recorder, with both rear encoder streams, IMU, GNSS and actuator status. Operator reported clockwise and counterclockwise square-like paths, approximately ten-second stop markers and later hard turns. The eight selections below cover the sides of the two squares; they do not define exact corner/whole-maneuver boundaries. First four and later four are the chronological square groups used for the validation split. Do not import the November 17 drive percentages or maneuver timestamps into this run.

Operator notes: 10:59 engine-off manual steering check, 11:01 engine running after two start attempts, approximately 11:14 parked/engine off/Pause. These notes have minute-level precision. The capture session begins earlier and ends later than motion.

Site root: `field_testing/sites/62_Collins_multi_boundary_20260915/`.

- Preserved raw copy: `runs/20261119_manual/` (git-ignored; originals retained).
- Archive: `manual_run_20261119.tgz`, 40,090,094 bytes.
- Archive SHA-256: `eb531f2c0504785f5c034073360533cef2157912524818e82a5b1bb50c0d2d19`.
- Segments: `telemetry_154324_711636Z`, `telemetry_155824_744702Z`, `telemetry_161324_745916Z`.
- Selected drive producer session: `7295481ca757417ead5487b37862ce3e`, source IMU 15:57:13.153124–16:18:28.987171 UTC. Earlier preparation session `f64aaef9015544be816bf9caae4a28e1` is separate; no integration bridges that restart.
- All archive/closed-segment hashes and metadata counts were verified. Across the three segments: 161106 IMU, 27742 encoder, 29970 GNSS, 42087 status, 29970 pose, 1872 event and 441 router rows. These counts include preparation, not only the squares.
- Original replay: `analysis/20261119_manual_sensor_replay/replay_20261119_manual.html`. Its elapsed-time zero is 10:57:13.153124 EST. Calibration uses full-rate raw observations, not the replay's 100 ms display samples.

## Al's exact selected sections

Boundaries are the whole-second times in Al's screenshot. Signed ticks below include fractional allocation of the two boundary-straddling encoder increments. Fractions are an alignment calculation; the recorded raw increments are integers.

| Section | Square group | Start EST | End EST | Seconds | RTK distance (m) | Left signed ticks | Right signed ticks |
|---|---|---|---|---:|---:|---:|---:|
| 1 | First | 11:04:47 | 11:05:07 | 20 | 20.108627257 | 223820.109163 | 224654.844000 |
| 2 | First | 11:05:12 | 11:05:25 | 13 | 13.644899865 | 153296.130578 | 150739.306271 |
| 3 | First | 11:05:29 | 11:05:50 | 21 | 20.463079657 | 235759.599294 | 235119.442360 |
| 4 | First | 11:05:53 | 11:06:08 | 15 | 15.051608563 | 172806.911429 | 170183.181416 |
| 5 | Second | 11:07:13 | 11:07:28 | 15 | 15.791602036 | 176705.524100 | 176521.511163 |
| 6 | Second | 11:07:32 | 11:07:52 | 20 | 21.521968026 | 240942.150058 | 241573.450565 |
| 7 | Second | 11:07:56 | 11:08:12 | 16 | 15.980422838 | 183427.345380 | 184629.703807 |
| 8 | Second | 11:08:17 | 11:08:35 | 18 | 18.219806309 | 208056.538991 | 207367.393105 |

Total reference distance **140.782014551 m**. Displayed digits preserve computational lineage, not a claim of millimetre physical accuracy.

All position fixes bracketing the 16 selected start/end boundaries were RTK Fixed. Section 2 has some heading-quality failure; section 8 has interior position/heading degradation. Those interior flags were retained, but **none of Al's eight sections was removed from this fit**. Al considers these basically straight and their endpoint distances reliable. Heading extrema/curvature diagnostics remain available as observations rather than grounds to silently replace the selections.

## How the numbers were constructed

1. Read native IMU and encoder JSONL from verified closed segments. Restrict to the identified producer session; retain device clocks, raw signed `delta_ticks`, errors and continuity state.
2. Recover each independent position **host decode time** from `imu.monotonic_s − gnss_position_age_s`, grouping within 1 microsecond. A new heading or repeated 20 Hz GNSS broadcast is not a new position fix. Identical coordinates at distinct decode times remain separate fixes. Precise receiver measurement epochs are not available in this recording.
3. Map selected EST times to source monotonic time using the selected session's median UTC-minus-monotonic offset. Interpolate start/end coordinates between the neighboring position decode updates.
4. Convert WGS84 coordinates to local east/north using the WGS84 curvature radii at the selected session origin. Reference distance is the antenna's endpoint displacement, `sqrt(delta_east² + delta_north²)`, accepted as travel for these selected sections. Do not sum absolute noisy GPS point distances.
5. Allocate each raw encoder increment to its overlap with the requested interval; assume uniform travel only within its approximately 100 ms source interval. Add signed increments. No gap, invalid increment, device reset or producer restart may be interpolated through. All eight selected sections had continuous, valid encoder evidence on both sides.
6. Fit `N = k D` through zero, independently left/right, where D is the accepted GPS distance and N is the signed tick total:

```text
k = SUM(D * N) / SUM(D * D)       # ticks/metre
predicted_distance = N / k
equivalent_metres_per_magnet_revolution = 16384 / k
```

This minimizes squared tick residuals with the accepted GPS distances as predictors. It is not an unweighted average of eight per-section ratios; longer distances have greater influence. Do not replace the constants with a different fit convention without recording the method and reason.

As a cross-check, endpoint distance/time gives approximately 0.974–1.076 m/s across these sections. Time-weighted distinct GNSS speed updates agree within approximately 1%; the historical `1.59593` conversion gives mean-wheel speeds approximately 8.5–12.2% higher. That supports the empirical conversion adjustment without identifying the physical cause.

## Validation and authoritative derived files

All-eight fit distance RMSE: **0.241 m left, 0.239 m right**. These are in-sample residuals.

Fit sections 1–4, predict sections 5–8:

| Side | First-four fit (ticks/m) | Later-four RMSE (m) | Maximum absolute error (m) |
|---|---:|---:|---:|
| Left | 11342.845967 | 0.209368 | 0.280196 |
| Right | 11285.642912 | 0.225811 | 0.379275 |

Leave one whole section out, fit the other seven and predict the omitted section: RMSE **0.282 m left, 0.274 m right**; maximum absolute error **0.427 m left, 0.406 m right**. These are same-run forward checks, not independent-run or reverse validation, and not a demonstrated short-outage pose error bound.

Authoritative derived package: [operator-selected fit report](../../field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_user_selected_encoder_fit/REPORT.md).

- `fit_results.json`: full-precision constants, per-section residuals, validation and review status.
- `selected_sections.json`: retained extraction input, exact boundaries/coordinates, signed ticks, flags and original raw hashes; independent of temporary spreadsheet support storage.
- `source_sha256.json`, `SHA256SUMS`: input/analyzer/raw-source provenance and output hashes.
- Reproducible fitter: `field_testing/tools/fit_user_encoder_sections_20261120.py`; the report gives the exact rebuild command.
- Spreadsheet: `outputs/20261120_encoder_slices_01a1f90b/Selected_time_slices_20261119.xlsx`. Permanent raw extractor: `field_testing/tools/extract_two_square_encoder_sections_20261120.py`; its exact JSON snapshot is retained in the authoritative fit package above. The initial spreadsheet-support copy is preserved under that output directory as well.

To repeat raw extraction without modifying evidence, from the repository root:

```powershell
python field_testing/tools/extract_two_square_encoder_sections_20261120.py --output-dir outputs/two_square_calibration_rebuild
```

Then pass the generated `slices.json` to the fitter's `--input` argument and use a separate derived output directory. The extractor is intentionally frozen to this capture/session and these eight selections; for future tires, make a new dated extraction/selection revision rather than silently changing this historical source.

The earlier strict-screen exploration is separate: `analysis/20261120_encoder_rtk_calibration_review/` fitted seven shorter windows and produced 11319.197 left / 11332.277 right ticks/m using a different fit orientation (distance against ticks). **Use the all-eight operator-selected values above for the next offline work**, not the earlier exploratory values. Preserve both packages for provenance.

## Future tires, pressure and calibration revisions

Tire make/model/size, measured pressure and load were **not recorded quantitatively** for this capture. Al reports no physical encoder installation change relative to the historical constants. Do not invent a pressure, exact rolling circumference or gear ratio.

For a future tire/configuration change, retain this calibration and its evidence. Create a new dated revision with tire identification, left/right measured pressures, relevant load and surface conditions, unchanged/changed sensor coupling, USB identities, raw capture/archive hashes, exact selected sections, method, validation and new constants. Compare the old and new conversions on the new evidence. Preserve the old revision rather than overwriting it; record the effective software/calibration ID if a later reviewed deployment occurs. Reassess after a material pressure or coupling change. Calibration selection, deployment and motion approval remain distinct.

## What remains for pose integration

Ticks/metre fitting is complete for this iteration. Runtime **time alignment does not refit those constants**. The two USB devices both report about 10 Hz, but their increments can span offset intervals. For example, L may describe 10.000–10.100 seconds and R 10.030–10.130. Position prediction must combine distances for the same interval; it should not average arbitrary latest reports as though they covered identical time. Interpolation/allocation must not cross a missing sample. This also matters where a trusted GPS correction falls partway through a wheel interval.

Encoders supply distance; the bias-corrected gyro predicts change in body heading from the last trusted GNSS heading. For a small common interval, average the two signed calibrated wheel distances and advance along the midpoint heading:

```text
distance = (left_distance + right_distance) / 2
delta_east  = distance * sin(compass_heading)
delta_north = distance * cos(compass_heading)
```

Angles must be converted to radians for these functions. Compass north is 0 degrees, east is 90 degrees; gyro-positive CCW yaw decreases compass heading in the current convention. At 0.3 m/s for 0.1 s, travel is 0.03 m: heading east adds 0.03 m east, while heading northeast adds approximately 0.0212 m east and north. No acceleration double integration is needed. A measured physical reference transform is still needed before treating antenna-based comparisons as rear-axle control geometry.

Real non-Fixed intervals in this same run can be replayed now: start from the last trusted position/heading, advance using wheel distance and gyro, then compare **before correction** with recovered Fixed GPS. Interior non-Fixed coordinates cannot establish the true error while the outage is in progress. Healthy intervals with GPS deliberately withheld provide a reference throughout. Both tests are useful; actual-loss recovery checks alone do not replace healthy masked-interval evaluation.

See the [actual-loss development replay](../../field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_two_square_actual_losses/REPORT.md), using these constants. It includes nearby healthy masks and states the same-run/calibration overlap. The existing earlier masking report used the earlier strict-window scales and must not be described as validation of this all-eight fit. Current pose remains shadow-only, raw UDP feeds remain unchanged, and the proposed degraded-motion envelope is not commissioned.

First actual position-loss checks, calculated November 20 with no GPS position or heading corrections during each prediction:

| Last Fixed / prediction start EST | First non-Fixed EST | First recovered Fixed EST | Prediction duration | Position disagreement at recovery |
|---|---|---|---:|---:|
| 11:08:14.093 | 11:08:14.254 | 11:08:14.898 | 0.804874 s | 0.025159 m |
| 11:08:21.262 | 11:08:21.444 | 11:08:26.899 | 5.636234 s | 0.118408 m |
| 11:08:27.676 | 11:08:28.068 | 11:08:30.892 | 3.216032 s | 0.021773 m |

Duration includes sampling brackets; it is not an exact receiver outage duration. The 5.636 s prediction accumulated about 5.761 m of mean signed wheel travel. These are historical predictions, not approval of that distance under a future degraded-motion policy. Section 8 contributed to scale fitting and overlaps the latter losses, so these closure checks are partly calibration-dependent, not independent validation.

Nearby healthy combined-loss masks over 3.001 s and 5.010 s gave endpoint disagreements of 0.033972 m and 0.059164 m. A proposed 1 s comparison beginning near 11:07:35 was rejected because the reference did not remain healthy throughout; it is retained in the replay audit. Full intermediate-trajectory error scoring and broader fault tests remain subsequent work. This first replay uses a standalone offline predictor, not an enabled production-model change.
