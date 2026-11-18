# First IMU manual-drive analysis — November 17, 2026

**Historical evidence analysis. DRAFT — UNTESTED AS A NAVIGATION OR PLANNING POLICY.** No firmware, controller, route, safety gate or service was changed. Original evidence remains unchanged.

## Main findings

1. The expected yaw sign is supported: all 22 detected turn candidates have matching IMU/F9P turn signs, including reverse maneuvers. Among 18 candidates with all displayed GPS checks healthy throughout, median absolute turn-angle disagreement is 0.48°, maximum 3.12°. Endpoints and source timing matter; this is not independent truth.
2. Stationary engine-on gyro-Z noise is 40.8 times the selected engine-off reference. It mostly averages down: engine-on Z standard deviation is 1.336°/s at sample rate, versus 0.0152°/s for one-second arithmetic means.
3. Moving prediction windows show useful short-term agreement, but degree-scale outliers remain, especially in turns. A -100 ms timing sensitivity trial improves the 3-second turning comparison; it is not applied to the headline result and does not establish a calibrated delay.
4. The final left/right maneuvers contain repeated tight loops, allowing per-lap antenna-path measurements. Settled lap radii are recorded below. They are not whole-tractor clearance limits or approved minimum planning radii.

## Driving sequence

Operator follow-up: two square-ish paths, CW first at approximately 84% drive, then CCW at 100%; backed up before the second square; hard left followed by hard right. Percentages and steering intent are operator reports, not measured commands. No steering-pot or actuator-command telemetry was captured in this IMU dataset.

| Data-inferred phase | Eastern time | GPS median speed while >0.15 m/s | Net F9P CCW change |
|---|---|---:|---:|
| First square: CW | 11:27:45–11:31:05 | 0.437 m/s | -356.0° |
| Second square: CCW | 11:32:15–11:34:00 | 1.042 m/s | 361.4° |

The likely reverse interval immediately before the second square is 11:31:46–11:32:12, with median inferred signed forward speed about -0.27 m/s. Earlier reverse-like intervals also appear during positioning. Classification uses a 1.2-second GPS position chord projected onto chassis heading, not unsigned ground speed alone. No reverse motion model or turn-radius calibration is inferred from those intervals.

## Stationary bias and vibration

Intervals were selected from the known engine periods after inspecting speed/path, rather than searching for minimum gyro noise. Each selected interval has GPS speed below 0.05 m/s and approximately 1–2 cm endpoint displacement. Small chassis rocking and sensor noise remain possible.

| Interval (Eastern) | Corrected Z mean (°/s) | Z sample std (°/s) | Z one-second-mean std (°/s) | Accel-Z std (g) | Integrated yaw change |
|---|---:|---:|---:|---:|---:|
| engine_off_before 11:19:00–11:20:00 | -0.00332 | 0.03276 | 0.00427 | 0.00055 | -0.200° |
| engine_on_stopped 11:23:00–11:24:00 | +0.00123 | 1.33580 | 0.01520 | 0.08222 | +0.067° |
| engine_off_after 11:38:00–11:38:50 | -0.00895 | 0.03283 | 0.00337 | 0.00052 | -0.448° |

The post-run corrected Z mean is -0.00895°/s (about -0.537°/minute if held constant). It differs from the selected pre-run engine-off mean by -0.00562°/s. This supports per-run bias calibration and ongoing bias assessment; it is not a permanent calibration constant. Temperature was not recorded, so its contribution cannot be isolated.

One-second means are descriptive averaging, not an implemented control filter. Vibration at frequencies above the capture bandwidth and aliasing cannot be ruled out from this recording.

## Individual square turns

| Candidate | Start–end Eastern | F9P turn, CCW degrees | IMU turn | IMU minus F9P | GPS healthy throughout? |
|---|---|---:|---:|---:|---|
| 5 | 11:28:30–11:28:35 | -91.06 | -90.73 | +0.33° | Yes |
| 6 | 11:29:04–11:29:11 | -101.24 | -101.13 | +0.11° | Yes |
| 7 | 11:30:10–11:30:17 | -85.95 | -84.60 | +1.35° | Yes |
| 8 | 11:30:53–11:30:58 | -100.99 | -100.27 | +0.72° | Yes |
| 14 | 11:32:38–11:32:43 | +79.91 | +92.64 | +12.73° | No — comparison only |
| 15 | 11:33:03–11:33:09 | +85.76 | +84.92 | -0.84° | Yes |
| 16 | 11:33:24–11:33:29 | +94.61 | +91.49 | -3.12° | Yes |
| 17 | 11:33:47–11:33:55 | +86.51 | +87.41 | +0.90° | No — comparison only |

The +12.73° comparison in the first CCW square corner crosses a degraded GPS interval. It cannot be counted as measured IMU error against reliable truth. The healthy CCW corner with about -3.12° disagreement remains a real item to investigate. See `turn_comparisons.csv` for all candidates, including approach/reverse/return maneuvers.

## Short GPS-free prediction windows

Each trial starts from a healthy F9P heading, then advances using the recorded fixed-bias gyro integration alone. No intervening heading correction is applied. Both endpoints and every intervening raw/heading sample must pass displayed GPS checks; windows crossing an IMU reset, calibration, missing data, or >50 ms IMU gap are excluded. Actual endpoint spacing must be within 0.12 seconds of the nominal horizon.

The headline set has disjoint source intervals within each horizon. Horizons share evidence; even non-overlapping windows from one run are not independent validation runs. Rolling one-second starts are exported separately for inspection, not used to inflate the headline count.

| Horizon | Moving windows | Median absolute disagreement | 95th percentile absolute | Largest observed absolute |
|---:|---:|---:|---:|---:|
| 1 s | 449 | 0.347° | 1.780° | 10.167° |
| 3 s | 166 | 0.468° | 2.605° | 5.483° |
| 5 s | 101 | 0.473° | 2.686° | 7.068° |
| 10 s | 47 | 0.546° | 2.991° | 8.346° |

Results are descriptive disagreements with F9P, not guaranteed IMU error bounds. Turn-heavy and straight-travel subsets differ; see `prediction_summary.csv`. The small number of long straight windows limits their interpretation. No result establishes safe navigation through a loss or validates 0.3 m/s continuation.

### Time alignment is a priority

The baseline comparison aligns IMU yaw to the GNSS `relposned_timestamp` (Pi decode arrival) using the recorded UTC/monotonic relationship. It does not blindly compare the latest GPS value at IMU-read time. However, decode time is still not the receiver measurement epoch.

| IMU integration time shift | 3-second turning windows | Absolute p95 | Absolute maximum |
|---:|---:|---:|---:|
| -0.2 s | 91 | 4.357° | 6.550° |
| -0.1 s | 91 | 2.079° | 2.666° |
| +0.0 s | 91 | 3.639° | 5.483° |
| +0.1 s | 91 | 6.425° | 11.572° |
| +0.2 s | 91 | 9.715° | 17.887° |

The fractional offset `(decode UTC - iTOW seconds) modulo 1 second` has median 114.4 ms (5th–95th percentiles 101.1–134.2 ms). This is consistent with roughly a tenth-second timing effect, but modulo arithmetic cannot prove total latency, integer-second alignment, GNSS/host clock synchronization or a correction sign for production use. The -100 ms trial was exploratory, not an independently calibrated improvement; headline numbers remain unshifted.

## Observed steering capability for future planning

Settled portions of the last left/right circles were selected by time, retaining the complete maneuver results and per-lap fits. A complete revolution with radial RMS <=0.10 m is used as an exploratory circularity check. Two left laps and two right laps pass; the first right lap is strongly noncircular (0.361 m radial RMS) and is excluded from the settled-lap summary. These few laps share the same field run; ranges are observed variation, not confidence bounds.

| Maneuver | Selected interval | Settled full laps | Fitted radius range | Diameter range | Settled lap radial RMS |
|---|---|---:|---:|---:|---:|
| hard_left | 11:34:08–11:34:26 | 2 | 1.172–1.197 m | 2.344–2.393 m | 0.030–0.031 m |
| hard_right | 11:34:32–11:35:01 | 2 | 1.049–1.057 m | 2.097–2.115 m | 0.016–0.018 m |

These are circles fitted to the logged position antenna, using RTK Fixed / healthy-heading points, at roughly 1 m/s. Centers drift between laps. Whole-interval fits include entry/center drift and are substantially less circular; both are retained in `results.json`, not hidden.

The logger captured no steering command, pot position or full-lock acknowledgement. The operator's hard-turn intent is the steering evidence. Exact antenna-to-rear-axle offset and a surveyed body/deck footprint are not established by this dataset. Do not substitute these radii for a rear-axle kinematic limit or collision-clearance envelope.

Historical planning documentation reports different older left/right values (approximately 1.05/1.63 m). Today's apparent asymmetry is reversed. Different calibration, speed, surface, reference point and test conditions have not been separated. Preserve both datasets; do not silently replace established planning radii. Repeated instrumented full-lock tests and a body/deck swept-envelope check are needed before planning changes.

## Actual GPS-health losses

These are combined displayed-health losses on the deduplicated ~5 Hz heading-source timeline. Durations have roughly 0.2-second granularity, may combine several causes, and are not motion interruptions. A new offline prediction is anchored at the last healthy sample before each loss; disagreement is evaluated only when the reference recovers.

| Start–recovery Eastern | Loss duration | Heading reference available inside loss | Median speed | Recovered heading disagreement |
|---|---:|---:|---:|---:|
| 11:24:54–11:25:53 | 58.80 s | 90.5% | 0.009 m/s | +1.707° |
| 11:30:50–11:30:51 | 1.20 s | 0.0% | 0.413 m/s | -0.333° |
| 11:32:43–11:32:46 | 3.40 s | 20.0% | 1.203 m/s | +3.551° |
| 11:33:43–11:33:44 | 0.60 s | 0.0% | 1.052 m/s | -0.810° |
| 11:33:54–11:33:55 | 0.81 s | 100.0% | 1.034 m/s | +0.221° |
| 11:34:05–11:34:07 | 1.61 s | 0.0% | 1.099 m/s | -3.118° |
| 11:34:56–11:34:57 | 0.82 s | 100.0% | 0.687 m/s | -0.188° |

The long approximately 59-second interval is mainly loss of position RTK Fixed and is mostly stationary (median speed 0.009 m/s); it is not a 59-second driven heading-outage validation. The approximately 3.4-second moving combined loss ends with about +3.55° disagreement. Recovered agreement does not prove the intermediate heading or position was correct when no healthy reference existed.

## Capture integrity and next questions

Reintegration of every recorded segment agrees with stored yaw to 0°. The two >50 ms host sampling gaps occurred before the identified squares/hard circles and were excluded from prediction windows. Near-full-scale raw rows: gyro 0, acceleration 0. Host polling still cannot exclude aliasing or recover missed samples.

1. Resolve measurement-epoch versus host-decode timing before interpreting fast-turn disagreement as gyro drift or scale error.
2. Repeat stationary engine-on/off and instrumented left/right full-lock maneuvers to assess repeatability, surface dependence, thermal bias and clearance.
3. Build any next heading estimator offline/shadow-only. Evaluate it on held-out runs, including deliberately varied speeds and turns. Keep present local watchdogs and health gates unchanged.
4. Translate demonstrated heading/position uncertainty into route-corridor clearance only in a separately reviewed planning study. This analysis makes no continuation or field-motion authorization.

## Figures and reproducibility

- `maneuver_path.png`: phase-labeled path.
- `stationary_bias_and_vibration.png`: engine effect and residual mean.
- `heading_prediction_windows.png`: moving/turning/idle prediction distributions.
- `hard_turn_dimensions.png`: fits and discarded transition effects.
- `candidate_turns.json`, `turn_comparisons.csv`, `prediction_windows.csv`, `prediction_summary.csv`, `gps_loss_comparisons.csv`, `reverse_candidates.csv`, `timing_sensitivity.csv`, `results.json`: audit data.
- `operator_followup.json`: driving description supplied after replay review.
- `source_provenance.json`: exact source paths and SHA-256 hashes. `SHA256SUMS`: derived artifact hashes.

```powershell
python field_testing/tools/analyze_tractor01_imu_manual_20261117.py --evidence-root field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261117_imu_manual/imu_20261117T161449_639816Z --operator-notes field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261117_imu_manual/operator_notes.json --operator-followup field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261117_imu_manual_analysis/operator_followup.json --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261117_imu_manual_analysis
```
