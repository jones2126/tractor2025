# Hard-turn radii and IMU observations: November 17 and 19

Review dated November 20, 2026. **Observed GPS position-antenna paths, not commissioned planning limits.** All November 19 times below are EST (UTC−05:00).

## What was measured

Both manually driven two-square sessions included subsequent hard-left and hard-right circles. Al supplied the following November 19 intervals from the replay. In this chat he corrected left circle 1 to 11:11:19–11:11:27 and confirmed left circle 2 as 11:11:27–11:11:34, replacing the zero-duration/overlapping entries in the initial screenshot. Keep these confirmed selections; do not revert to the screenshot's initial entries.

| Direction / circle | Selected interval EST | Fixed position fixes / all independent fixes | Antenna-path radius (m) | Diameter (m) | Radial RMS (m) |
|---|---|---:|---:|---:|---:|
| Right 1 | 11:09:48–11:09:55 | 35 / 35 | 1.067 | 2.134 | 0.022 |
| Right 2 | 11:09:55–11:10:03 | 40 / 40 | 1.079 | 2.157 | 0.034 |
| Right 3 | 11:10:03–11:10:10 | 35 / 35 | 1.077 | 2.154 | 0.026 |
| Left 1 | 11:11:19–11:11:27 | 40 / 40 | 1.190 | 2.381 | 0.042 |
| Left 2 | 11:11:27–11:11:34 | 25 / 35 | 1.226 | 2.453 | 0.028 |
| Left 3 | 11:11:34–11:11:42 | 36 / 40 | 1.174 | 2.349 | 0.031 |
| Left 4 | 11:11:42–11:11:49 | 35 / 35 | 1.176 | 2.353 | 0.032 |

The millimetre display precision supports comparisons, not a millimetre accuracy claim. Left circle 2 includes position-fix losses; only about **235 degrees** of the circle is covered by accepted Fixed points. Its fit is less well constrained, despite a small residual. Left circle 3 also loses Fixed briefly but retains about 347 degrees of sampled circle coverage. No lost/non-Fixed positions were invented or interpolated to fill these circle fits.

For the observed November 19 selections, right radii span **1.067–1.079 m**, left **1.174–1.226 m**. The later left circles cluster at **1.174 and 1.176 m**. These are observed variations, not confidence bounds or guaranteed minimum steering capability. Centres drift between laps; exact selected boundaries are not exact 360-degree lap boundaries.

## How the radius was calculated

1. Verify the November 19 archive and every closed telemetry segment checksum and channel count before analysis.
2. Select producer session `7295481ca757417ead5487b37862ce3e` and the confirmed EST intervals. Native source time, not replay display bins, controls selection.
3. Identify independent position host-decode updates from `IMU monotonic_s − gnss_position_age_s`, grouped within 1 microsecond. The repeated 20 Hz GNSS publications are not independent fixes. Identical coordinates at distinct native decode times remain separate observations.
4. Retain RTK Fixed, nonfatal positions within each selected interval. Heading quality is not a prerequisite to fit valid position coordinates. Preserve excluded non-Fixed points separately for review.
5. Convert WGS84 latitude/longitude to a local tangent east/north metre frame using the WGS84 curvature radii at the reported origin. Fit a circle by first obtaining an algebraic least-squares centre/radius, then iteratively minimizing **geometric radial residuals**: `sqrt((east−centre_east)^2 + (north−centre_north)^2) − radius`.
6. Give each independent accepted position equal weight. Do not smooth positions, delete outliers to improve the result, assume a complete lap, or use circumference divided by time to determine the fitted radius. Export radial RMS/p95/max, centres, angular coverage and original algebraic fit as diagnostics.

The radius is measured directly from the GPS antenna's spatial trace. The newly fitted encoder scales do **not** determine this GPS circle radius. Encoder distance is used only for an additional wheel-distance/IMU-angle comparison in the machine-readable results.

## What the IMU reported

Gyro Z is corrected with the recorded stationary bias, without recalibrating while moving. Positive rotation is counterclockwise/left; negative is clockwise/right. Mean rate below is the integrated recorded gyro yaw change divided by the exact selected duration. Native instantaneous rates fluctuate with body motion/vibration and turn-rate variation.

| Circle | Mean angular rate (deg/s, CCW +) | Integrated rotation (deg) | Approximate revolution period at mean rate |
|---|---:|---:|---:|
| Right 1 | -47.95 | -335.7 | 7.51 s |
| Right 2 | -48.12 | -384.9 | 7.48 s |
| Right 3 | -48.51 | -339.5 | 7.42 s |
| Left 1 | +48.26 | +386.1 | 7.46 s |
| Left 2 | +48.99 | +342.9 | 7.35 s |
| Left 3 | +49.84 | +398.8 | 7.22 s |
| Left 4 | +50.41 | +352.9 | 7.14 s |

There is no >50 ms IMU sampling gap or yaw-segment reset within any selected interval. The integrated angles need not equal 360 degrees: the user-selected boundaries are whole seconds rather than measured revolution crossings.

The corrected gyro Z native standard deviation is approximately **6.7–8.5 deg/s**. Gyro X/Y standard deviations are approximately **5.6–6.7 / 3.7–4.5 deg/s**. The report includes each circle's p05/p95/min/max, raw and corrected gyro statistics, other axes and native samples.

Acceleration magnitude averages approximately **1.08–1.10 g**, with native standard deviations about **0.19–0.24 g** and individual peaks approximately **1.8–2.25 g**. It includes gravity, mounting tilt, body motion and vibration; it is not a gravity-removed measurement of lateral acceleration, and sensor X/Y cannot be labelled tractor lateral/longitudinal without a reviewed mounting transform. These peaks alone do not identify wheel slip, impact, engine vibration or IMU failure. Acceleration was not double-integrated into position.

Recorded reported-ground-speed means are approximately **0.886–0.920 m/s right** and **1.029–1.070 m/s left**. As a cross-check, `reported speed / absolute angular speed in radians/s` gives approximately **1.059–1.096 m right** and **1.216–1.246 m left**. This is a consistency check, not the circle-fit method; decode timing, nonuniform motion and the shared GNSS source affect agreement.

Actual steering pot medians were **195, 195, 190** for right circles and **880, 879, 884, 882** for left circles, close to the commissioned right191/left885 endpoints. This supports the recorded hard-turn intent without claiming an independent mechanical stop measurement.

## Comparison with the November 17 session

The [November 17 observed-hard-turn record](../03-design/03-3-tractor/03-3-10-frame-chassis/20261117-observed-hard-turn-dimensions.md) documents the earlier two-square session and subsequent circles:

| Run / selection | Left antenna-path radii | Right antenna-path radii |
|---|---:|---:|
| November 17 settled full laps | 1.172–1.197 m | 1.049–1.057 m |
| November 19 Al-selected intervals | 1.174–1.226 m | 1.067–1.079 m |

November 17 selected left 11:34:08–11:34:26 and right 11:34:32–11:35:01, with two accepted settled laps each. Its first right lap was strongly noncircular (0.361 m radial RMS) and was excluded from the settled summary but retained in its audit. The earlier analysis used algebraic circle fits and its replay map projection; this analysis refines geometric radial residuals in a WGS84 tangent frame. Interval selection, speed, surface and sampling also differ. This is repeated observational evidence, not a controlled experiment identifying the cause of small radius differences.

Older planning notes also contain approximately 1.05 m left / 1.63 m right, with opposite asymmetry. Preserve that historical evidence; do not silently replace a mission's established planning geometry with a new observed minimum.

## Interpretation for the tractor and pose work

These radii belong to the **position antenna**, not the rear-axle midpoint, tire/deck outer edge or full vehicle swept envelope. A surveyed antenna lever arm and body/deck dimensions are required to convert this evidence into collision clearances or a rear-axle kinematic radius. The repeated data supports using these turns to evaluate heading prediction and wheel/gyro integration offline. It does not authorize field motion or change a route's clearance margin.

## Reproduction and evidence

Site root: `field_testing/sites/62_Collins_multi_boundary_20260915/`.

- Raw archive: `runs/20261119_manual/manual_run_20261119.tgz`, SHA-256 `eb531f2c0504785f5c034073360533cef2157912524818e82a5b1bb50c0d2d19`.
- Closed segments: `telemetry_154324_711636Z`, `telemetry_155824_744702Z`, `telemetry_161324_745916Z`.
- [Detailed report and figures](../../field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_selected_hard_turns/REPORT.md).
- Derived `results.json`: every radius, coverage/residual, gyro/acceleration statistic, speed cross-check and steering feedback distribution.
- `position_samples.json`, `imu_samples.json`: selected independent position and native IMU audit observations.
- `source_sha256.json`, `SHA256SUMS`: raw/source and derived provenance.
- Builder: `field_testing/tools/analyze_selected_hard_turns_20261120.py`.

```powershell
python field_testing/tools/analyze_selected_hard_turns_20261120.py --evidence-root field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261119_manual --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_selected_hard_turns
```

No production source, service, firmware, motion authority or raw evidence was changed by this analysis.
