# November 17 IMU experiment analysis

Start with [REPORT.md](REPORT.md). Figures and CSV/JSON audits are adjacent.

The operator's square directions, drive percentages and reverse maneuver are preserved in `operator_followup.json`. Phase times are inferred. Steering radii are observations of the GPS antenna path, not approved planning values.

The durable planning reference is `obsidian_vault/03-design/03-3-tractor/03-3-10-frame-chassis/20261117-observed-hard-turn-dimensions.md` from the repository root.

Numerical validation: `python -m unittest tractor_rpi.testing.test_imu_manual_analysis_20261117 tractor_rpi.testing.test_imu_replay_20261117` (15 tests). Source hashes are in `source_provenance.json`; output hashes are in `SHA256SUMS`. The build transcript is diagnostic rather than source evidence.
