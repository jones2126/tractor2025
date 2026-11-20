# Continuation prompt: calibrate ticks against RTK, then build supported pose

Copy the following prompt into the next tractor2025 chat:

---

Continue the GPS/IMU/encoder integration work in `C:\Repos\tractor2025`.
Read `AGENTS.md`, `field_testing/TRACTOR01_CURRENT_FIELD_CONTEXT.md` and
`obsidian_vault/02-testing/20261120-pose-fusion-context.md` first. That context
supersedes the older November 19 handoff and includes function-level Mermaid
diagrams, verified dataset paths, design decisions, unknowns and field findings.
Review the linked November 17/19 replay reports and current producer/model,
recorder, encoder firmware and Pure Pursuit input/geometry code. Check Git state
before editing. GitHub main is software source of truth; do not make Tractor01
the development master or alter unrelated uncommitted work.

Our goal is one best-supported navigation-pose statement from GPS, IMU and both
encoders, ultimately consumed by Pure Pursuit. It is the agreed navigation input,
not perfect physical ground truth. Estimation, route tracking and motion
authorization remain separate. Current pose is shadow-only and invalid for
control; raw UDP 6002/6009/6010/6013 stays unchanged until a reviewed migration.

**Begin with actual quantitative encoder-ticks-versus-distance analysis**, using
the preserved, verified November 19 manual-run raw data. Do not stop after a plan
if the local evidence is available. Verify archive/segment hashes and raw counts.
Use independent trustworthy RTK Fixed position updates and multiple continuous,
straight moving windows. Fit and independently validate each side's signed
ticks/metre, metres/tick and metres/sensor-revolution. 16,384 ticks is a sensor
shaft revolution; confirm the mechanical coupling before calling it a wheel
revolution. Do not fit from existing provisional distance/speed or assume the
1.59593 m scale is calibrated. Do not sum absolute jitter or raw noisy GPS point
distances as true travel. Keep forward/reverse, turns, timing, geometry and wheel
slip hypotheses separate. Reject errors/discontinuities explicitly.

The 20 Hz GNSS stream repeats slower measurements. Heading count and speed count
are not a guaranteed GGA position sequence; general timestamp can be refreshed
by heading. Inspect source code and the IMU-recorded position decode age to
reconstruct independent position updates if supported, documenting tolerance
and decode-versus-measurement-time limits. Account for endpoint alignment,
producer/device-clock resets and antenna/reference-point offsets. Use held-out
whole windows/maneuvers, not interleaved correlated snapshots, for validation.

Create a reproducible analyzer and separate derived package at site
`analysis/20261120_encoder_rtk_calibration_review/`, preserving raw files.
Provide REPORT.md, per-window audit CSV, candidate-scale JSON marked REVIEW ONLY,
plots/residuals/uncertainty, exclusions and source/output SHA-256 manifests.
If geometry or timing prevents a defensible calibration, report the measured
ratios, uncertainty and exact missing information rather than force precision.
Do not set reviewed_calibration=true or install scales just because fitting ran.

Then propose the pose contract and staged prediction/correction implementation:
reference point/frame, sensor signs and lever arms, independent position/heading
support, source timestamps/ages, uncertainty, calibration/continuity IDs,
unsupported held values, trusted correction eligibility and reacquisition
behavior. Preserve the existing explicit GPS/IMU/encoder main-loop ingestion
functions and threaded acquisition/RTCM forwarding. Avoid duplicate hardware
owners or private UDP 6009 consumers. Reuse current pure helpers where appropriate;
do not assume the existing experimental wheel propagation is ready for control.

Evaluate one-time and 5 s GNSS heading anchoring separately, and use healthy
held-out data with offline position-only/heading-only/combined masking over
1/5/10/30s horizons. Report trajectory and heading errors, coverage and prediction
interval limitations. Do not repeatedly recalibrate gyro bias while moving or
silently substitute acceleration double integration for wheel distance.

Al's proposed degraded-motion envelope is at most 3 m OR 30 s, first limit reached,
at up to 0.3 m/s; 3 m at that speed is 10 s. This is not commissioned. The pose producer
reports source support/age/travel; the safety supervisor separately owns any
reviewed incident timer/distance budget and rearm rules. Preserve 500 ms freshness,
Teensy lease and all independent actuator safety checks. Recovery must not quietly
restart an incident budget. Do not confuse this with the existing phone-only
30s/30m Auto communications envelope.

Present the encoder findings first, followed by the pose-design proposal and
focused offline/fault tests. Proposed rollout is offline replay, reviewed live
shadow validation, then an explicit Pure Pursuit input adapter and separately
approved supervised field package. Do not change running services, firmware,
frozen mission packages, motion authority or raw transport schemas in this
analysis phase. Ask only for decisions/measurements the preserved evidence cannot
answer. Explain results plainly for Al, with concrete units and examples.

Retain the network-test distinction: disabling Pixel mobile data allowed a
warm-system Starlink-loss test to recover and support Manual steering after a
brief ~1500 ms acknowledgement gap. Exact phone/router cause is unproven; cold
offline boot result has not yet been supplied. Do not add internet dependency to
local Manual control while working on navigation.

---
