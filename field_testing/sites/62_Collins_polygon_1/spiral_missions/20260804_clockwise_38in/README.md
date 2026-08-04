# Polygon 1 clockwise inward spiral — 2026-08-04

This is a continuous clockwise inward spiral derived from the complete manually
driven lap in `00_boundary_log_20260804_115620.csv`.

- Start: `40.485562833, -80.332340333`
- Start heading: approximately 162° compass (south-southeast)
- Spacing: 0.9652 m (38 inches)
- Speed: 0.85 m/s
- Lookahead: 2.0 m
- Waypoints: 2,846 at approximately 0.50 m spacing
- Route length: 1,422.2 m
- Estimated runtime: 27.9 minutes
- Sampled minimum radius: 2.13 m
- Centerline outside boundary, using 2 cm tolerance: 0.0 m

The route continuously moves inward to the tractor’s right instead of closing
one polygon and making an abrupt lateral jump to the next. It stops after 14
revolutions, before the inner geometry becomes too tight. Presumed remaining
uncut area using a 42-inch deck is approximately 254.7 m² (15.2%), primarily
the central region.

## Source-data warning

The selected lap contains 4,226 RTK Fixed rows, 579 RTK Float rows, 146 DGPS
rows, and two Invalid rows. The complete lap was required to obtain a closed
boundary. This is therefore a supervised test mission, not an unattended
production mission.

## Running the mission

Manually position tractor01 at the listed start and point south-southeast. Put
the switch fully UP in Pause. Then run:

```bash
cd /home/al/tractor2025/field_testing/sites/62_Collins_polygon_1/spiral_missions/20260804_clockwise_38in
bash ./run_polygon_1_clockwise_inward_20260804.sh
```

The launcher runs preflight and refuses to continue if tractor01 is more than
1.5 m from the start or more than 20° from the required heading. Remain in
Pause while the logger and controller initialize. Select Auto only after the
controller reports RTK Fixed, the route is clear, and its output has been
reviewed. Keep the deck disengaged for the first supervised run.
