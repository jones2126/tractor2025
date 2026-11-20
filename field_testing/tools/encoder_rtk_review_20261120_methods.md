## Interpretation and calibration decision

The measured straight-window ratios support approximately **1.45 metres per sensor shaft revolution**, with positive reported ticks for forward motion on both sides. One tick is approximately 0.088 mm under these candidates. The left firmware already negates its sensor counts; do not negate the recorded left stream again. A sensor revolution is 16,384 ticks. The mechanical gearing remains unverified, so this is not a measured tire circumference or wheel revolution.

The old approximately 7% side difference does **not** persist as a constant difference in these selected forward windows: the fitted scales differ by about 0.12%. Individual windows still vary. This does not isolate gearing, tire deformation, slip, steering, speed dependence or historical push-test geometry. Every accepted calibration window is forward; the short reverse block fails the minimum distance. Reverse calibration has therefore not been validated.

The candidate JSON must remain `reviewed_calibration=false` and `valid_for_control=false`. Training distances span only about 6.34–8.07 m, with three windows on two neighboring-leg groups from one run. The free-intercept diagnostics are approximately -0.91 m left and -0.79 m right. Those large intercepts, narrow distance range, residual curvature and unmeasured latency prevent treating the fitted decimal places as physical accuracy. A through-origin fit is reported because zero ticks should represent zero rolling travel on an uninterrupted constant-direction interval; it is a descriptive candidate, not evidence that all systematic errors vanish at zero.

`encoder_scale_candidates.json` records a seeded 2,000-replicate whole-leg bootstrap interval. With just two training leg groups, that interval has extremely limited meaning; it is resampling sensitivity, not a defensible population confidence interval. Per-window scale ranges and held-out residuals are more useful than a confidence claim. The four held-out windows belong to the later same-day maneuver and share surface, installation and environmental conditions. They are not independent field runs. Mean-wheel residuals and side ratios are included in the audit CSV.

## Evidence, timing and exclusions

The archive SHA-256 is pinned to `eb531f2c0504785f5c034073360533cef2157912524818e82a5b1bb50c0d2d19`. Each closed segment is verified with the repository exporter, and every JSONL channel count, including router/events, is checked against its metadata. All raw bytes are read-only inputs. `source_sha256.json` records each source file, archive and analyzer dependency; `SHA256SUMS` covers derived files. Git baseline at the start of work was clean `c6db1bc`; no running tractor was contacted.

The preserved data contains a preparation producer session from 15:43:24–15:47:12 UTC and a later session from 15:57:13–16:18:28 UTC. `producer_sessions.json` records both, including source clocks and row counts. Calibration explicitly selects the second session, `7295481ca757417ead5487b37862ce3e`, containing the drive. It does not integrate through the restart. `continuity_audit.csv` retains first samples, failed continuity checks, encoder events, calibration and IMU gaps. Firmware I2C errors produce no ENC record and rebaseline after recovery; neither interpolated distance nor the next suppressed increment can repair those losses. WARN can replace reported speed while raw ticks remain; producer validity and event records are retained. Device clocks are checked as unsigned 32-bit differences, allowing a genuine wrap but rejecting resets and long gaps.

Independent position decode times are reconstructed as `imu.monotonic_s - gnss_position_age_s`, grouped within **1 microsecond** within a producer session. `apply_gps()` advances its position clock only when a GGA patch contains latitude; heading updates can advance the generic timestamp but do not advance that position clock. Identical successive coordinates are retained if their position decode times differ. This is host **decode time**, not receiver measurement time, and it does not establish how much serial/USB latency preceded decoding. First IMU observations after each decode supply the associated position/fix state. Position epochs absent from all IMU observations cannot be recovered. Heading counts and RMC speed counts are not used as position sequence numbers.

Heading decode times are reconstructed independently using `gnss_heading_age_s`; `independent_headings.csv` preserves those observations. Prediction comparisons interpolate unwrapped heading between independent host-decode observations as an offline reference. Position-only masking uses that retained heading reference; it is an idealized interpolation, not a proven causal live correction algorithm. Heading-only masking retains GNSS position at the endpoint, so its zero position residual is structural, not validation of an estimator. These distinctions are explicit in the mask mode and must survive future implementation.

The local metre frame uses WGS84 meridional/prime-vertical curvature radii at the first position of the selected session; east is positive x and north is positive y. Height is omitted. On this small site a local tangent linearization is adequate for the exploratory comparison, but the origin and physical antenna reference remain explicit. Display UTC is mapped using the recorded UTC-minus-monotonic offset of the first preserved IMU record; calculations use host monotonic time. No device-to-host clock fit is needed because increments are allocated in host source time. Unmeasured transport delay remains a systematic uncertainty.

The final screening rules are:

- Every position link must be Fixed, nonfatal, with native decode spacing 0.10–0.35 s and independently supported heading: fixed carrier, valid/fix/ref flags, age at the associated IMU row <=0.30 s, accuracy <=1 degree, baseline 0.8–1.3 m.
- Moving speed of 0.2–1.8 m/s and heading change <=3 degrees/s are screened over approximately one second, using surrounding position endpoints rather than noisy 0.2-second differences. Actual preceding steering feedback must be within 35 pot counts of commissioned centre 525 and status receipt no older than 0.3 s. Status arrival and hardware source time are not equated.
- Contiguous qualifying links form blocks, cut at at most 20 seconds. Accepted blocks require >=5 s and >=3 m, <=0.15 m maximum endpoint-line lateral deviation, <=6 degrees heading spread, and no projected position backtrack >0.02 m. Direction comes from displacement versus body heading, within 30 degrees forward or reverse; tick signs must agree. Short remainders and all excluded links remain in CSVs.
- Both wheel streams must bracket the endpoints with consecutive valid producer records, positive host spacing <=0.2 s, positive device interval <=200 ms and unchanged producer session. Only the two endpoint-straddling increments are fractionally allocated, assuming uniform travel inside that approximately 100 ms interval. No allocation spans a discontinuity. Both signed totals and provisional/fitted distances are exported.
- Training precedes 11:06 EST; later accepted windows are held out as whole sections. Neighboring training fragments separated by <=5 s share a bootstrap unit. No duplicated snapshots or random point-level split are counted as independent validation.

Exploratory screening history matters: the initial native-interval heading-rate screen fragmented otherwise usable travel through measurement noise; a strict 4-degree/0.10 m screen left only one window. The final broader 6-degree/0.15 m criteria were chosen after inspecting that coverage, before fitting final scales. The resulting selection is exploratory and has not been independently pre-registered or validated. It must not be advertised as a confirmatory calibration experiment.

Distance is **endpoint displacement**, checked against displacement projected onto a separately fitted PCA track direction. Lateral RMS/max and heading spread quantify residual curvature. No noisy GNSS point-distance sums or absolute raw tick sums are used to fit scale. Stationary signed jitter is exported separately and cancels rather than accumulating as travel. `aligned_distance.csv` preserves intermediate signed wheel displacement and RTK displacement from the same start. The plots summarize these values; they do not replace full-rate source data.

The per-window +100 ms endpoint shift is a timing sensitivity, not a fitted latency correction. `timing_sensitivity.csv` also refits the training scales at -200/-100/0/+100/+200 ms and validates each on the same held-out windows. It shifts both encoder endpoints and leaves the reference distance fixed; it cannot identify receiver epoch delay or account for arbitrary variable latency. The source gaps and decode spacings do not establish a calibrated time offset. A position-specific receiver epoch/sequence and acquisition/decode timestamps should be recorded in a future reviewed schema.

## Geometry and missing measurements

The current Pure Pursuit controller converts compass heading to ENU and uses `gps_offset_x=0`, `gps_offset_y=0`, with a comment saying the antenna is at base_link; its wheelbase default is 1.27 m. This is a software convention, not a surveyed antenna-to-rear-axle measurement. The chassis notebook records 49.5 inches wheelbase (1.2573 m) and 28.5 inches rear tire centre spacing (0.7239 m), but the applicability and exact installed sensor reference geometry need confirmation.

For a provisional rear track of 0.7239 m, a 6-degree net turn changes each wheel's arc relative to axle centre by about 0.038 m. An antenna lever arm of length L metres can introduce endpoint displacement differences up to approximately `0.105 * L` metres over a 6-degree net rotation. L has not been established here, so that term cannot be assigned a numeric uncertainty bound. Oscillatory steering, changing slip and antenna/body heading offset can add effects not bounded by net rotation. Low curvature reduces those problems; it does not eliminate them.

The preserved evidence cannot answer these physical questions:

1. How many sensor shaft revolutions occur for one marked full rear-wheel revolution on each side? Record rolled distance as well; separate the coupling ratio from effective rolling circumference.
2. What are the position antenna's forward/left/up offsets from rear-axle midpoint, the heading baseline/body-axis offset, and confirmed rear contact-centre track width?
3. What repeatability is obtained on longer centered straight forward **and reverse** sections at several speeds, with independent native position epochs and measured geometry?

These are the next operator measurements, not prerequisites for reading the existing results. No additional field motion is authorized by this report.

## Prediction masks, anchoring and limitations

`mask_predictions.csv` evaluates position-only, heading-only and combined masking at nominal 1/5/10/30 s. Starts are disjoint within each healthy block and horizon. The actual endpoint duration is exported because the next independent position epoch may be up to about 0.2 s later. Horizons share observations and are not independent of each other. Only later held-out maneuver data is used; scale fitting uses earlier data. Training has not been extended to improve validation results.

There are two overlapping reporting strata, which must **not** be pooled: the four accepted calibration-style straight windows, and all later continuously healthy blocks up to approximately 11:14 EST. The latter allows long horizons and turns, provided both encoders and the IMU remain continuous. Trials with mean signed-increment travel below 0.15 m/s are omitted. Here absolute **synchronized mean increments** are used only to screen whether a trial moves, not to calibrate scale or claim true total travel. Fine jitter and reversals limit that descriptive travel metric.

The straight subset has 1 s and 5 s coverage but no qualifying 10 s or 30 s window. Long-horizon results therefore include turns and may mix straight, stop and reverse portions. `mask_maneuver_summary.csv` separates low-turn/turning and forward/reverse/mixed increment signs; 15 degrees heading range is the descriptive turning threshold. Do not infer validated reverse scale from counterfactual reverse trials. Every prediction uses fixed recorded gyro bias, signed candidate wheel increments averaged across sides, and compass/ENU integration. Acceleration is never double-integrated. Both wheel series must be present and continuous; there is no one-wheel fallback in this evaluator.

Healthy GNSS endpoints and intermediate support flags are required, so these are deliberate masks of healthy data, not reconstructions of real outages. Coverage excludes unhealthy reference intervals and stationary trials; the denominators in `mask_summary.csv` describe only healthy candidate blocks, not all drive time. Rejected candidate masks and their reasons are in `mask_exclusions.csv`. Near-block endpoint rounding, GNSS noise, heading decode latency, antenna lever arms and tire slip all contribute to residuals. Counts within one day are correlated. Empirical p95 values are descriptive disagreement quantiles and do not establish a 95% predictive guarantee for another run, surface or outage. No trustworthy calibrated prediction interval is claimed.

`heading_anchor_comparison.csv` separately compares one anchor per healthy block against periodic 5-second anchors, using independent heading decode alignment and the same fixed-bias yaw. Bias is never recalibrated while moving. The periodic version may use GNSS while it is available; the masked combined-loss predictor cannot secretly use those corrections during a loss. The table is an anchoring-policy comparison, not evidence for an IMU-only 5-second reset during GNSS loss. Each block has its own duration, p95 and maximum; long and short blocks should not be given equal meaning without reviewing duration.

## Proposed pose contract and staged implementation

**Proposal only — no producer, controller, raw UDP schema, service, firmware or safety-selection change is made in this package.** Preserve the existing threaded sensor acquisition/RTCM worker and explicit `ingest_gps()`, `ingest_imu()`, `ingest_encoders()` main-loop boundaries. The recorder stays the sole UDP 6009 owner. Reuse parsing, quality checks and gyro helpers where their semantics match, but do not promote the existing experimental wheel propagation to control by changing a flag.

Proposed `supported_pose_v1` publication should carry:

| Field group | Required contract |
|---|---|
| Identity | schema/version, producer boot/session, monotonically increasing sequence, continuity ID, calibration IDs and source evidence hashes |
| Reference | local ENU x/y metres, WGS84 origin/ellipsoid, rear-axle midpoint as proposed kinematic point; measured antenna lever arm and body heading offset required before transforming |
| Heading | compass degrees [0,360), true-north reference and forward body axis; ENU yaw radians explicitly derived as pi/2 minus compass; independent support flag |
| Time | state monotonic time and clock identity, publication UTC/monotonic time, position and heading receiver epochs when available, host decode/acquisition times and source ages; never freshen held data with publication time |
| Motion | signed forward speed m/s, direction-supported flag, signed displacement distinct from observed nonnegative travel; null travel if continuity or calibration cannot support it |
| Source support | independent position and heading modes: GNSS_CORRECTED, WHEEL_IMU_PREDICTED, HELD_UNSUPPORTED or UNAVAILABLE; source IDs/ages and rejection reasons per component |
| Uncertainty | quantified flag, position covariance/error model and heading variance only when validated; otherwise null/unquantified, with the empirical validation domain named |
| Anchors | independent trusted position/heading anchor epochs, source IDs and ages, continuous supported travel since each anchor, pending reacquisition state |
| Corrections | innovation/residual, accepted/rejected reason, correction magnitude, continuity change and any blend state; no concealed snap or freshness reset |
| Authority | `valid_for_control=false` throughout offline/shadow stages; estimator support never grants motion |

For the present offline antenna comparison, label the reference **position antenna**, not rear axle. Do not publish converted rear-axle coordinates until the transform has measured provenance. A future adapter must explicitly reconcile the controller's current zero-offset convention with the new reference point; never apply the lever arm twice.

Stage 1 pure functions should validate/order inputs within clock/session continuity, reject nonfinite values, identify unique eligible GNSS epochs, and align both wheel increments to one integration interval. Stage 2 predicts from bias-corrected gyro and synchronized signed wheel distances, using midpoint heading and measured reference geometry. Stage 3 applies independently eligible position and heading corrections, with reasoned innovation gates, timing uncertainty and correction auditing. Stage 4 assesses support/uncertainty and publishes a snapshot that distinguishes supported prediction from held values. These are proposed responsibilities, not implemented production functions.

Position Fixed eligibility and heading fixed-carrier/accuracy/baseline eligibility must remain independent in the estimator: healthy heading may help while position is unavailable, and vice versa. Do not require both merely to report the surviving component. Safety policy may impose additional joined conditions. A gyro gap, scale/clock reset, encoder error or missing wheel breaks the appropriate prediction chain; unsupported held coordinates remain explicitly unsupported/null with their original state time. Do not substitute acceleration integration or an unvalidated one-wheel yaw model.

On reacquisition, require unique, fresh, trusted GNSS observations and assess innovations in the stated reference frame. Large discrepancies must enter a rejected/pending-reanchor state and be logged; they must not silently snap a control pose or reset a degraded incident. Any later reviewed smoothing must expose both target correction and delivered state/uncertainty; an attractive smooth line is not stronger evidence. A new continuity/calibration ID invalidates downstream cached deltas.

The estimator reports support, ages, correction evidence and trustworthy travel; the **safety supervisor** separately owns any reviewed GNSS-loss incident. Al's proposed envelope is at most **3 m OR 30 s, whichever comes first, at <=0.3 m/s**; 3 m at that speed is 10 s. It is not commissioned. First loss starts an incident, reconnects do not reset its origin, and rearm needs a separately specified stopped-Pause operator acknowledgement. Missing time/travel/support cannot authorize continuation. Existing 500 ms command freshness/Teensy lease and local actuator checks remain unchanged, as does the distinct phone-only 30 s/30 m Auto exception.

## Focused verification and rollout gates

The accompanying analyzer tests exercise repeated position snapshots, identical-coordinate independent fixes, fractional endpoint allocation, reverse signs, stationary signed-jitter cancellation, missing samples, invalid producer increments, device reset/wrap, producer-session change, ENU axes and heading wrap. Hash/count verification and full raw replay are additional integration checks. These tests validate analysis mechanics, not a live estimator or a fault-safe robot.

Before a production implementation, add tests against the proposed contract for: position-only and heading-only loss, stale/frozen samples in fresh envelopes, IMU missing/large dt, one-wheel/both-wheel loss, ERR/rebaseline and WARN, out-of-order/duplicate receiver epochs, missing/nonfinite geometry, session/calibration changes, mixed-direction windows, large/small reacquisition corrections, and unchanged first-loss incident budgets through intermittent recovery. Missing support must invalidate only the components it actually supports; authorization remains fail-closed in the supervisor. No live implementation of these proposed behaviors is claimed here.

Rollout remains: **offline review and repeated held-out runs → reviewed live shadow validation → explicit versioned Pure Pursuit adapter → separately approved supervised field package**. The present results justify reviewing scale candidates and improving timestamp/geometry evidence; they do not justify selecting Auto or extending watchdogs.

Keep the network distinction: disabling Pixel mobile data allowed the warm-system Starlink-loss test to recover and support Manual steering after a brief approximately 1500 ms acknowledgement gap. Exact phone/router cause and gap-level actual output remain unproven; no cold offline-boot result was supplied. Navigation work must not add an internet gate to local Manual control.

## Reproduce

From `C:\Repos\tractor2025`:

```powershell
python field_testing/tools/test_analyze_encoder_rtk_20261120.py
python field_testing/tools/analyze_encoder_rtk_20261120.py --evidence-root field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261119_manual --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_encoder_rtk_calibration_review
```

Requires NumPy and Matplotlib already available in the development environment. Raw evidence is local and git-ignored; analysis can be rebuilt without a tractor connection or any serial/UDP consumer. The analyzer, tests and methods template are pinned in the source manifest. Output manifests deliberately exclude their own bytes.
