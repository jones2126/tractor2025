# Pose integration: context and next-chat prompt

November 19,2026. **DRAFT — NOT VALIDATED FOR CONTROL OR DEGRADED MOTION.**

Successor prepared November 20: [detailed pose-fusion context](20261120-pose-fusion-context.md)
and [continuation prompt](20261120-pose-fusion-continuation-prompt.md). Use those
for the next chat; this file preserves the earlier checkpoint.

## Context to retain

Use `C:\Repos\tractor2025` on Windows. GitHub main is the source of truth;
Tractor01 `/home/al/tractor2025` pulls reviewed commits. Read AGENTS.md and current
field context. Source synchronization does not require a service restart when
only documentation/offline tools changed. Do not start a second hardware reader
or UDP 6009 logger. Preserve raw evidence and exact source hashes.

The installed producer `tractor_rpi/calc_publish_position.py` already has
independent GPS/IMU/encoder ingestion functions in a20 Hz loop and background
readers/RTCM forwarding. `position_model.py` records raw GNSS and shadow pose;
`imu_sensor.py` supplies inertial helpers. Recorder/preflight are separate modules.
Legacy GNSS ports 6002/6009/6010/6013 are unchanged and Pure Pursuit still consumes
raw GNSS on 6010. Shadow pose is never control-authoritative and its uncertainty
is not quantified. No calibrated wheel-distance fallback is installed by default.

Healthy RTK position provides the best available distance reference for this
experiment, not an independent guarantee of ground truth. Wheel/RTK disagreement
can reflect scale, signs, turns, lever arms, timing or GNSS errors as well as slip.
Do not label disagreement slip without testing those alternatives.

Bias calibration requires actual stationary engine-off startup; heading anchoring
is a separate operation. Experimental shadow heading uses healthy position AND
heading anchors every 5 s. The replay's once-anchored comparison is different from
that periodically corrected shadow output; evaluate them separately.

Verified evidence:

- November 17 standalone capture: site
  `runs/20261117_imu_manual/imu_20261117T161449_639816Z`; previous replay/analysis
  under `analysis/20261117_imu_manual_replay/` and `20261117_imu_manual_analysis/`.
- November 19 segmented capture: site `runs/20261119_manual/`, three verified
  telemetry segments and preflight; archive hash in the session summary.
- November 19 replay: site `analysis/20261119_manual_sensor_replay/`; builder
  `field_testing/tools/build_tractor01_sensor_replay_20261119.py`. Raw files are
  ignored by Git. This new format is not the November 17 standalone CSV schema.
- November 19 operator markers:10:59 engine-off steering check,11:01 engine running
  after two attempts,~11:14 parked/off/Pause. Ten-second stops between squares and
  hard turns are candidate segmentation aids. Drive percentages are not supplied
  for this day; November 17 percentages must not be copied.

Open reliability issues: cold-boot left USB silence recoverable by targeted USB
reauthorization without reflashing; intermittent GNSS Float despite fresh
corrections; delayed ntfy. None has an isolated root cause. Do not fold unrelated
boot fixes into a navigation estimator without a separate review.

## Ready-to-use continuation prompt

Continue the tractor2025 pose-integration investigation from the November 19
handoff. First read AGENTS.md, `field_testing/TRACTOR01_CURRENT_FIELD_CONTEXT.md`,
the November 19 accomplishments note, both replay reports and the current
`calc_publish_position.py`, `position_model.py`, `imu_sensor.py`, recorder and
Pure Pursuit input adapter. Do not change live services, firmware, the frozen
mission package, legacy UDP schemas or motion authority during this analysis.

Our intended design is: a pose producer ingests GPS, IMU and both wheel encoders
and publishes the best supported position AND heading; Pure Pursuit calculates
steering from that statement; the RPi supervisor separately decides whether
motion is allowed. We want to evaluate at most 3 m OR 30 s of GNSS-degraded travel,
whichever limit occurs first, at up to 0.3 m/s. These are proposed limits, not a
commissioned policy. At that speed 3 m is 10 s. Keep existing 500 ms local freshness/
lease limits and all other independent stops unchanged.

Begin with preserved-data analysis and a design proposal before production code:

1. Verify hashes, source clocks, producer sessions, resets and native measurement
   counters. Identify preparation, engine-on vibration, straight runs, squares,
   reverse segments and hard turns. Distinguish operator notes from measured
   boundaries. Do not count repeated20 Hz GNSS snapshots as independent5 Hz fixes.
2. Confirm wheel direction/sign and inspect raw delta ticks, device clock wraps,
   discontinuities and read errors. A failed read need not imply lost firmware;
   inspect actual firmware baselining/gap semantics. Fit per-wheel distance scales
   from healthy straight RTK windows, account for antenna reference offset and
   reverse, and validate on held-out windows. Report sample selection, uncertainty
   and remaining unexplained disagreement before claiming slip detection.
3. Confirm IMU axes/mounting and clockwise/counterclockwise sign conventions;
   quantify stationary bias before/after engine start, vibration, gaps and
   saturation. Compare one-time and5 s GNSS anchoring and assess heading/position
   prediction over 1/5/10/30 s with deliberate offline GNSS masking, held-out tests
   and position-only, heading-only and combined degradations. Clearly separate
   what-if results from an actual recorded outage reconstruction.
4. Define the pose reference point/frame (GNSS antenna, rear axle or other point),
   antenna/IMU lever arms, compass vs ENU yaw, timestamp/freshness, monotonic age,
   provenance/source mode, quality/uncertainty, continuity/reset identifiers,
   supported position vs supported heading, and signed distance vs total incident
   travel. Decide when pose is unsupported instead of carrying stale values as
   apparently fresh. Estimate and authorization must remain separate.
5. Propose sensor handling for pristine GNSS, position degraded only, heading
   degraded only, both degraded, encoder/IMU faults, and GNSS reacquisition.
   Define trusted-anchor eligibility and residual checks, correction/smoothing
   effects, reset visibility and recovery without sudden unreviewed steering.
   Do not repeatedly reset gyro bias while moving.
6. Propose budget ownership: pose producer supplies anchor age/travel and support
   evidence; final RPi safety supervisor owns any authorized incident timer and
   distance budget. First loss starts the incident, position/heading recovery
   must not quietly reset it; define the stopped-Pause acknowledgement explicitly.
   No motion without trustworthy budget inputs. Explain what can be evaluated
   offline and what requires separately approved stationary/manual field work.
7. Measure hard-left/right antenna-path radius, diameter and steering limits from
   healthy windows; report fit residuals and explain what tractor footprint/deck
   dimensions are still needed for whole-vehicle planning clearance.
8. Present results and a reviewable staged plan: offline estimator replay,
   shadow live validation, explicit supported-pose adapter for Pure Pursuit and
   separately reviewed motion-policy integration. Keep current raw feeds until
   the new contract and all consumers are reviewed. Provide focused tests for
   gaps, stale inputs, restarts, limits and reacquisition before proposing rollout.

Keep the explanation understandable to Al, with concrete measured examples.
Ask only for decisions or measurements the evidence cannot answer. Do not claim
the manual replay validates autonomous control or a degradation policy.
