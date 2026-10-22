# October 21, 2026 master-manual phone replay

This directory contains the self-contained replay of the October 21
Tractor01 field run. The mission used the immutable September 23 route and
ultimately reached the goal, but phone-freshness expiries make it a failed
qualification run. Mission 2 was not authorized.

The verified raw evidence archive is stored locally in the ignored run-data
directory:

`field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261021_133807_master_manual_phone/`

Archive SHA-256:

`a316fcd031e60afdba5cddd6d9baf29377956043f5b2a772e74bd04344387574`

Key observations:

- First mission stop: phone freshness expired at 14:18:14 EDT near waypoint
  15,362 in `garden_left_ring_2`.
- The tractor rolled backward approximately one yard on a small hill.
- Manual repositioning enabled guarded forward reacquisition.
- The controller later reached the goal, sent stop, and closed both mission
  telemetry logs cleanly.
- Approximate gasoline consumption: 2.5 US quarts (0.625 US gal / 2.37 L).
- The 40-minute phone-control analysis failed: longest uninterrupted armed
  interval 38.7 minutes, RTT p95 85.0 ms, RTT maximum 520.6 ms, maximum armed
  command gap 423.7 ms, and 23 heartbeat/freshness expiry events across the
  collected control log. Event-by-event investigation remains necessary.

Rebuild the replay with:

```bash
python3 field_testing/tools/build_tractor01_mission_replay_20261021.py
```
