# November 19 manual sensor replay

Open replay_20261119_manual.html locally; no network or tractor connection required.

Archive and all three closed segment manifests independently verified on Windows.
All consumed channel counts match closed metadata. Raw originals are preserved.
This replay includes 120,742 full-rate IMU rows, displayed in 100 ms bins
with gyro/acceleration extrema and extra health/segment transition boundaries.
Timeline starts 2026-11-19T15:57:13.153124+00:00.

GPS positions and IMU comparisons are recorded observations. Direct-serial position
decode age replaces the old standalone logger's UDP packet age in the displayed
health test. Position Fixed, heading validity/freshness/carrier/accuracy and baseline
checks follow the preceding replay; these labels do not reconstruct motion authority.
Detailed fractions and overlapping condition episodes are in summary.json.
Snapshot rates are not independent GNSS measurement rates (heading is about 5 Hz).

Wheel speeds and delta ticks use the latest preceding source record within 0.5 s;
no future samples are joined. Scale is provisional, not calibrated. Status is joined
the same way, using recorder arrival time where no source UTC exists. Drive chart
shows joined Manual-percent output, Pause zero, other/missing modes blank.
Steering is actual pot feedback. JRK VIN alone cannot determine engine state.

Heading comparison is the recorded once-anchored IMU output, not the separately
five-second-reanchored shadow pose and not a navigation estimate. The shadow pose
remains invalid for control. Timing gaps break integration; yaw across segments
cannot be subtracted. Host read intervals do not reconstruct missed sensor samples.

Operator notes: 10:59 engine off/manual steering check; 11:01 engine running after
two start attempts; approximately 11:14 parked/off/Pause. Unknown seconds are
shown as one-minute bands; the parking time has additional unspecified uncertainty.
Ten-second-stop events are automatic candidates, not confirmed maneuver boundaries.
CW/CCW squares and hard turns were reported; dimensions still require analysis.

Full source SHA-256 provenance is embedded in the HTML and summary.json.
Derived artifacts are separate from raw recordings. Nothing here changes control.
