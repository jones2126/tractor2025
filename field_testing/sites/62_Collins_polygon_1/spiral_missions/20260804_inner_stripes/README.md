# Polygon 1 inner-core stripe completion — 2026-08-04

This mission fills the inner core left by the clockwise spiral.

- 20 east/west strips at 0.9652 m (38-inch) spacing
- Straight speed: 0.85 m/s
- Keyhole speed: 0.50 m/s
- Keyhole radius: 1.90 m
- Pure Pursuit lookahead: 1.00 m (changed from 2.00 m for the 2026-08-05 retest)
- Route length: 472.2 m
- Estimated runtime: 12.8 minutes
- 1,010 waypoints
- Presumed core remaining uncut: 3.3 m² (1.5%)

Each straight strip reaches the modeled inner-core edge before the keyhole
begins. The initial 14.9 m transit starts at the outer-ring start pose and stays
in already-cut space. It and all 19 strip-to-strip keyholes have zero modeled
path length inside the core interior and remain inside the complete site
boundary.

One 2.3 m strip at the narrow bottom tip was excluded because connecting it
required an elongated path through the core. The 42-inch deck coverage model
shows that only small edge slivers remain.

## Starting point

- Latitude: `40.485562833`
- Longitude: `-80.332340333`
- Heading: approximately `162°` compass (south-southeast)

This is the same starting pose used by the reviewed clockwise outer-ring
mission. Reposition manually and return the switch fully UP to Pause.

## Running

```bash
cd /home/al/tractor2025/field_testing/sites/62_Collins_polygon_1/spiral_missions/20260804_inner_stripes
bash ./run_polygon_1_inner_stripes_20260804.sh
```

The launcher verifies the reviewed mission hash, runs mission preflight, and
requires tractor01 to be within 1.5 m and 20° of the start pose. Remain in Pause
until the controller output has been reviewed. Keep the deck disengaged for the
first supervised run.
