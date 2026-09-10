# Polygon 2 to Polygon 3 ring test — 2026-09-09

Supervised, blades-off mission commanded at **1.00 m/s** throughout:

1. Polygon 2 outer boundary and one inner ring.
2. The RTK-Fixed Polygon 2-to-Polygon 3 transition driven on 2026-09-08.
3. Polygon 3 outer boundary and two inner rings.

The first waypoint is the exact `(0, 0)` local-frame position recorded during
the Polygon 2 pause. Direction arrows on the preview confirm clockwise travel.
The contained connector leaving Polygon 2 begins southward and meets the
original recorded transition, which is then followed toward Polygon 3.

Planning assumptions: 0.9652 m (38 inch) ring spacing, 1.90 m minimum turn
radius, 0.50 m maximum waypoint spacing, 2.00 m lookahead, and clockwise
rings. Ring connectors are forward-only Dubins paths checked against the
appropriate polygon drive area. The outer ring follows the finalized driven
boundary after inward corner rounding.

Run from any directory on tractor01:

```bash
/home/al/tractor2025/field_testing/sites/62_Collins_polygon_2/mission_plans/20260909_polygon2_polygon3_ring_test/run_polygon2_polygon3_ring_test_20260909.sh
```

The launcher rebuilds and validates the mission, runs preflight, checks the
live start position and heading, requires a typed confirmation, starts the
field logger, and then starts Pure Pursuit. Keep the mower deck disengaged and
remain ready to select Pause.
