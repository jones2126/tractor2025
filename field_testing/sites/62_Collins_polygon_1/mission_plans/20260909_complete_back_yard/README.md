# Complete 62 Collins back yard mission — 2026-09-09

This is a supervised, blades-off first test of the complete back yard at a
constant **1.00 m/s** command.

Route order:

1. Polygon 1 — 14 clockwise inward rings.
2. Polygon 1 — 21 reviewed stripes and contained keyhole turns.
3. A contained 1.90 m-radius connector to the recorded travel route.
4. The RTK-Fixed route driven to Polygon 2 on 2026-09-08, including the short
   final approach to Polygon 2's recorded `(0, 0)` pause corner.
5. Polygon 2 — clockwise outer boundary and one inner ring.
6. The RTK-Fixed Polygon 2-to-Polygon 3 route driven on 2026-09-08.
7. Polygon 3 — clockwise outer boundary and two inner rings.

The route is approximately 2.37 km and requires about 39.5 minutes of movement
at 1.00 m/s. Allow additional time for acceleration and controller settling.

Run from any directory on tractor01:

```bash
/home/al/tractor2025/field_testing/sites/62_Collins_polygon_1/mission_plans/20260909_complete_back_yard/run_complete_back_yard_mission_20260909.sh
```

The launcher rebuilds and validates both mission sections, runs preflight,
checks the live start position and heading, requires a typed blades-off
confirmation, starts the field logger, and starts Pure Pursuit.

## Live dashboard

The live mission dashboard adds a replay-style map, target and actual values,
mission progress, a guarded Start button, and software Pause/Resume controls.
The handheld Pause remains the independent safety override.

```bash
cd /home/al/tractor2025 && sudo -v && python3 tractor_rpi/pure-pursuit/mission_dashboard_20260910.py
```

Open the temporary operator-key URL printed in the terminal. Full instructions
are in `tractor_rpi/pure-pursuit/MISSION_DASHBOARD_20260910.md`.
