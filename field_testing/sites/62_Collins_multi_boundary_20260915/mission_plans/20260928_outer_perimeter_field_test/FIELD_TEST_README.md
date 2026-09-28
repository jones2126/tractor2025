# 62 Collins outer-perimeter field test

This package is the first supervised validation of the three outer perimeter paths. It is **not yet a mowing mission**.

## Route

- Start at the recorded base point.
- Follow the recorded route to the backyard.
- Drive the backyard perimeter clockwise.
- Follow the recorded route to the front yard.
- Drive the front-yard perimeter clockwise.
- Follow the recorded route to the over-road area.
- Drive the over-road perimeter clockwise.
- Return by the recorded front-yard and base routes.
- The left edge of the 42-inch deck is the perimeter-side edge.

After the first field attempt exposed turns that were too tight, the path was rebuilt conservatively with a 2.00 m turn-softening radius. It deliberately stands farther inside narrow or sharply notched portions of the surveyed perimeter.

The revised mission has 3,462 waypoints, is 689.949 m long, and takes about 11.50 minutes at the nominal 1.00 m/s command. Lookahead is 2.00 m throughout.

## First-run rules

1. Keep the mower deck disengaged for the entire run.
2. Run the normal preflight and do not proceed unless it passes.
3. Keep the legacy handheld ready; select Pause for excessive cross-track error, an unsuitable access connector, or any unexpected behavior.
4. Closely supervise the newly generated short access connectors. The longest is 2.88 m between the front-yard boundary and its recorded over-road transition route.
5. Use the dashboard's spoken left/right and distance guidance to approach and align with the mission's initial point.

## On tractor01

After pulling the commit, first verify without starting anything:

```bash
cd /home/al/tractor2025 && python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260928_outer_perimeter_field_test/verify_outer_perimeter_field_20260928.py
```

Then start the voice-guidance dashboard:

```bash
cd /home/al/tractor2025 && python3 tractor_rpi/pure-pursuit/mission_dashboard_outer_perimeter_field_20260928.py
```

The dashboard shows its URL in the terminal. Open it on the phone, use **Drive to start** and follow the spoken steering directions. The mission launcher will require the exact confirmation `RUN OUTER PERIMETER BLADES OFF`.

For terminal-only operation, use:

```bash
cd /home/al/tractor2025 && bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260928_outer_perimeter_field_test/run_62_Collins_outer_perimeter_field_20260928.sh
```
