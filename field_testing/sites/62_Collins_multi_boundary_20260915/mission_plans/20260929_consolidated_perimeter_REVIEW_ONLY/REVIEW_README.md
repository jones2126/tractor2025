# 62 Collins consolidated perimeter — REVIEW ONLY

This is the dated candidate generated from the owner-reviewed consolidated centerline. It is **not approved for Tractor01 execution**. The launcher verifies the package but blocks live operation until Al reviews the replay and static preview and explicitly approves promotion.

## Candidate summary

- 3,989 waypoints at approximately 0.20 m spacing
- 790.077 m total length
- 1.00 m/s at every waypoint
- 2.0 m normal lookahead, reduced to 1.5 or 1.0 m near reviewed tight turns
- Blades off and direct supervision required for the first field validation
- Start at local (0.000, 0.000); finish at parking near (-7.310, 0.811)

## Material geometry changes

G1 changes only submitted segment P6 near OBS-NEW-011. The source centerline brought the 42-inch deck envelope into the revision-3 candidate obstacle exclusion. The replacement follows the nearest 0.90 m centerline-clearance arc, leaving approximately 0.362 m between the nominal deck edge and the exclusion. This change is purple in both previews and requires owner review.

G2 and G3 remove two centimeter-scale near-180° join backtracks. G4 removes four sub-0.10 m duplicate join points. G5 rounds seven instantaneous heading changes over 40° within short local windows; its maximum centerline shift is 0.230 m. The gray source overlay makes all of these differences reviewable.

The other 12 turn-review markers retain the submitted geometry. Their radii are judged against successful field evidence at obstacles 004, 006, and 013 rather than rejected solely for being below 1.63 m.

## Review files

- `62_Collins_consolidated_perimeter_1mps_20260929_REVIEW_ONLY_full_route_REVIEW.png` — full-route static preview
- `62_Collins_consolidated_perimeter_1mps_20260929_REVIEW_ONLY_INTERACTIVE_REVIEW.html` — interactive Play/Pause replay with timeline, heading, obstacles, turn warnings, and deck reference
- `62_Collins_consolidated_perimeter_1mps_20260929_REVIEW_ONLY_validation.json` — validation and every turn decision
- `62_Collins_consolidated_perimeter_1mps_20260929_REVIEW_ONLY_audit.csv` — waypoint lineage and commands
- `62_Collins_consolidated_perimeter_1mps_20260929_REVIEW_ONLY.txt` — five-column mission file
- `mission_dashboard_consolidated_perimeter_20260929.py` — matching phone dashboard and voice guidance adapter (live launch remains gated)

## Safe verification

On Tractor01, after pulling the future approved commit, the review package can be checked without starting anything:

```bash
cd /home/al/tractor2025
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_REVIEW_ONLY/verify_consolidated_perimeter_20260929.py
```

or:

```bash
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_REVIEW_ONLY/run_62_Collins_consolidated_perimeter_20260929.sh --verify-only
```

Running the launcher without `--verify-only` currently stops with exit code 3. It does not configure GPS, start logging, start the controller, or send motion commands.

## Approval-time field sequence

After replay/preview approval, promote the package in a separate explicit edit. The promoted launcher must retain all of these safeguards:

1. Blades disengaged; direct supervision; handheld and physical e-stop ready.
2. Reconfigure the heading F9P with the guarded repository tool before starting rtcm-server.
3. Allow startup time for NRF, Teensy bridge, RTK corrections, and fixed heading.
4. Run preflight expecting `teensy_main_20260926` and require RTK corrections, RTK Fixed, valid fixed-carrier heading, 0.80–1.30 m baseline, heading accuracy ≤1.0°, healthy JRK, and stationary Pause.
5. Use dashboard voice guidance to reach and align with the initial waypoint.
6. Start field logging before Pure Pursuit and preserve neutral shutdown traps.
7. Run the first validation supervised and blades off; Pause immediately for unexpected clearance or tracking behavior.
