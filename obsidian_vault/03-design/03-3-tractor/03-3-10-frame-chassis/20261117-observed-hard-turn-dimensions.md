# November 17, 2026 — observed hard-turn dimensions

**Evidence reference for future planning — not an approved planning radius.**

Al manually drove two square-ish paths (CW at approximately 84% drive, CCW at
100%), backed up before the second square, then made hard-left and hard-right
maneuvers. The latter include repeated tight loops in the recorded GNSS trace.
Percentages and steering intent are operator reports; steering command, pot
position and full-lock acknowledgement were not captured by this logger.

Source: `imu_20261117T161449_639816Z`, logger commit
`d18cc6088916460aaeb7bc244f662a9aad66868e`. Analysis verifies the preserved source
SHA-256 manifest before reading it.

| Observed maneuver | Settled full laps | GPS antenna-path radius | Fitted diameter |
|---|---:|---:|---:|
| Hard left, 11:34:08–11:34:26 Eastern | 2 | 1.172–1.197 m | 2.344–2.393 m |
| Hard right, 11:34:32–11:35:01 Eastern | 2 accepted of 3 full laps | 1.049–1.057 m | 2.097–2.115 m |

Both selected maneuvers were approximately 1 m/s. Complete-lap circle fits with
radial RMS <=0.10 m were retained for the settled summary. Left-lap RMS was
approximately 0.030–0.031 m; retained right-lap RMS was 0.016–0.018 m. The first
right lap was strongly noncircular (RMS 0.361 m), so its 1.251 m fit is retained
in the audit but excluded from the settled-lap summary. This quality threshold
and interval selection are exploratory, not a pre-registered acceptance test.

The observed ranges are not confidence intervals or guaranteed minimums.
Centers moved between laps. Fits use the same approximate local map projection
as the replay and should be interpreted at centimetre scale, not millimetre
accuracy. Whole-interval fit/extent figures include entry and center drift.

These measurements describe the **position antenna's centerline trace**, not the
outer tires, front overhang, mower deck or entire swept footprint. A surveyed
antenna-to-rear-axle offset and verified current body/deck dimensions are still
needed for a kinematic or collision-clearance model.

Older site-planning guidance records approximately 1.05 m left / 1.63 m right,
which differs from this run and has the opposite asymmetry. The tests do not
isolate calibration, surface, speed or reference-point differences. Preserve
both records; do not replace existing mission planning radii from this result.

Before adopting a new planning limit: repeat instrumented full-lock tests in
both directions, record steering pot/command and surface/speed conditions,
measure the swept body/deck envelope, and review a clearance margin. No route,
controller, firmware, watchdog, or motion authority changed with this analysis.

Detailed report and provenance:
`field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261117_imu_manual_analysis/REPORT.md`.
Full circle fits, rejected lap and per-lap centers/residuals: `results.json`
and `hard_turn_dimensions.csv` in that analysis directory.
