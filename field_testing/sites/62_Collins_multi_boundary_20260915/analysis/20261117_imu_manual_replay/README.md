# November 17 IMU / GPS replay

Open `replay_20261117_imu_manual.html` in a browser. It is a self-contained historical replay; no network, server, robot connection or external map tiles are used.

Play/pause and real-time speed control share a timeline with map, health strip and six signal charts. Search/jump to engine events, sampling gaps or GPS-health changes. Click a chart to seek. Choose a local chart window to examine vibration or turns.

Engine times are operator reports with unknown seconds. The shaded event-minute bands express that uncertainty. Timing gaps reset the recorded yaw into separate segments; final yaw is not whole-run drift. No planned mission or command-state reconstruction is shown.

See `REPORT.md` for source hashes, definitions and limitations; `summary.json` for capture statistics; `gps_health_episodes.csv` for raw-resolution condition episodes; and `operator_notes.json` for the preserved engine-event notes.

## Rebuild from the preserved raw Windows copy

```powershell
python field_testing/tools/build_tractor01_imu_replay_20261117.py --evidence-root field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261117_imu_manual/imu_20261117T161449_639816Z --operator-notes field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261117_imu_manual/operator_notes.json --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261117_imu_manual_replay
```

The builder verifies all original capture hashes before reading the CSV. Derived files remain separate from the unchanged raw evidence.
