# November 17 manual-drive IMU / GPS historical replay

Observation only. No motion authority, reconstructed controls, route or fused heading.

Capture: `imu_20261117T161449_639816Z`. Commit: `d18cc6088916460aaeb7bc244f662a9aad66868e`.
Verified 154,769 raw rows; 1444.5 seconds; 14,468 playback samples.
Median host dt 9.506 ms; maximum 82.613 ms.

## How to use

Open `replay_20261117_imu_manual.html` locally. No server or internet needed. Play/pause, scrub, select a chart time window, click a chart, or jump/search events. Map zoom controls match the shared mission replay.

## Evidence and limitations

- Shared lineage: November 9 mission replay; September 19 digest/numeric/boolean helpers; current mission template CSS and map projection/zoom/drawing helpers are embedded at build time.
- Playback uses approximately 100 ms bins, with raw per-bin min/max envelopes for gyro/acceleration. Phase, yaw-segment and health transitions force bin boundaries. Raw counts/timing/health episodes are computed before decimation. Point values are the last recorded row of each bin, not an average.
- No GPS smoothing, position correction, heading fusion or turn classification. GPS coordinates are plotted as recorded, including degraded fixes. An offline local east/north grid replaces map tiles.
- The health overlay includes the unchanged preflight baseline range 0.80–1.30 m in addition to packet, RTK and heading checks. It can be amber when the logger's narrower GPS=valid flag is true. These are observation labels, not reconstructed arbitrator decisions.
- Packet age and heading-source age are separate. The producer's general timestamp is not a dedicated position-measurement timestamp; exact position age is unavailable here.
- Wrapped compass heading plots break at north and at yaw-segment boundaries. Calibration has no corrected gyro/yaw. Two recorded gaps reset relative yaw and its anchor; never subtract yaw across segments.
- Operator engine-on/off notes have minute precision: shaded 60-second uncertainty intervals. Engine-on does not establish exact movement start/end.
- Angular disagreement is logged only when the logger considered GPS usable. It is not an independent absolute heading-error measurement. Heading/yaw remain experimental, not navigation.
- Health episode durations may overlap across conditions. Do not sum them as independent stopped time. No motion interruptions can be inferred without command/status logs.
- Near-full-scale raw rows: gyro 0, acceleration 0. Raw min/max envelopes are retained; host polling cannot recover missed samples.

## Sampling gaps

- 2026-11-17T16:20:30.365176+00:00: 82.613 ms; new yaw segment 1.
- 2026-11-17T16:24:23.405006+00:00: 61.780 ms; new yaw segment 2.

## SHA-256 source provenance

- `samples.csv`: `5F2185F48F40996A3D3D5A5B6DADF4A822E01153DB70C151780AC3CE5817CDEB`
- `gnss.jsonl`: `C4926B40F2147C2F5D34517E25A09F5FCC13537EC06E076B93DBBE3461829EB5`
- `metadata.json`: `4F6A205367D8CDF02FA76E85A8C88ABD80191FB3450DD6410875A402A5504BE9`
- `SHA256SUMS`: `16C4AE53F8DAAD8540A4CA038D15923936CCD725EDAE6502B220DCA5B9C71BAF`
- `operator_notes.json`: `D3795C00D1D1213D62A7F2A490755949228714F63C758DC79F0B651A2646F520`
- `build_tractor01_imu_replay_20261117.py`: `8F0EF88EF99F1E632A407241F627FCDDB6FD769E6B3FF97E03C7886E044CCC4C`
- `tractor-imu-replay-template.html`: `A8CE72D8270644C81DD4E2B080174920B02C430815683E8398508CDACCE80A69`
- `tractor-mission-replay-template.html`: `9047B9DC0C665E0FDBBC180A4045810BE094D84B23952135272AF3AB95AF7E82`
- `build_tractor01_mission_replay_20261109.py`: `6DA42BF53663CB972D9211A9C1BA3CA133ED69BC812B3D9D1F76058E9A830376`
- `build_tractor_mission_replays_20260919.py`: `108F2EA19FEBA28D11C19D37E7485DA535C13E06321E37161CB4BB3C05334D93`
- `check_imu_replay_ui_20261117.cjs`: `E79BCE830191AFBF82CE6F09A6FBA81B1918B46EF4AE4AFA712F5252B2198A37`
