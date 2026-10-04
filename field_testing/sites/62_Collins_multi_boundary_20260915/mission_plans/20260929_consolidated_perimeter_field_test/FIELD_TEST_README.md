# 62 Collins consolidated perimeter — approved field test

Al visually approved the static preview and interactive replay on 2026-09-29. This package is approved only for the initial **directly supervised, blades-off** field test. It is not a mowing or unattended mission.

## Candidate summary

- 3,989 waypoints at approximately 0.20 m spacing
- 790.077 m total length
- 1.00 m/s at every waypoint
- 2.0 m normal lookahead, reduced to 1.5 or 1.0 m near reviewed tight turns
- Blades off and direct supervision required for the first field validation
- Start at local (0.000, 0.000); finish at parking near (-7.310, 0.811)

## Material geometry changes

G1 changes only submitted segment P6 near OBS-NEW-011. The source centerline brought the 42-inch deck envelope into the revision-3 candidate obstacle exclusion. The replacement follows the nearest 0.90 m centerline-clearance arc, leaving approximately 0.362 m between the nominal deck edge and the exclusion. This owner-approved change is purple in both previews.

G2 and G3 remove two centimeter-scale near-180° join backtracks. G4 removes four sub-0.10 m duplicate join points. G5 rounds seven instantaneous heading changes over 40° within short local windows; its maximum centerline shift is 0.230 m. The gray source overlay makes all of these differences reviewable.

The other 12 turn-review markers retain the submitted geometry. Their radii are judged against successful field evidence at obstacles 004, 006, and 013 rather than rejected solely for being below 1.63 m.

## Review files

- `62_Collins_consolidated_perimeter_1mps_20260929_full_route_APPROVED.png` — full-route static preview
- `62_Collins_consolidated_perimeter_1mps_20260929_INTERACTIVE_APPROVED.html` — interactive Play/Pause replay with timeline, heading, obstacles, turn warnings, and deck reference
- `62_Collins_consolidated_perimeter_1mps_20260929_validation.json` — validation and every turn decision
- `62_Collins_consolidated_perimeter_1mps_20260929_audit.csv` — waypoint lineage and commands
- `62_Collins_consolidated_perimeter_1mps_20260929.txt` — five-column mission file
- `mission_dashboard_consolidated_perimeter_20260929.py` — matching phone dashboard and voice guidance adapter

## Hands-free voice notes

Starting **Voice Guidance** also starts Chrome speech recognition. Accept the microphone permission before moving the tractor. During the run, begin each comment with **“Tractor note”**, for example:

- “Tractor note, move one foot left.”
- “Tractor note, clearance is too close.”

The dashboard says “Note saved” and writes both CSV and JSONL under `/home/al/field_logs/20260929_consolidated_perimeter_1mps/`. Each record includes timestamps, transcript, waypoint, route distance, GPS position, heading, speed, cross-track error, lookahead, fix quality, and a telemetry snapshot. Notes never alter the live mission.

Chrome may use an online recognition service, so internet access can be required. The dashboard displays `LISTENING`, `MICROPHONE PERMISSION BLOCKED`, `VOICE NOTES NEED INTERNET`, or another explicit status. Because guidance speech temporarily pauses recognition to prevent self-transcription, wait until the dashboard finishes speaking before saying “Tractor note.” Plain-HTTP microphone policy can vary by Chrome release; if Chrome blocks the tractor dashboard origin, use an HTTPS/trusted-origin setup before relying on voice notes.

## Safe verification

On Tractor01, after pulling the future approved commit, the review package can be checked without starting anything:

```bash
cd /home/al/tractor2025
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/verify_consolidated_perimeter_20260929.py
```

or:

```bash
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/run_62_Collins_consolidated_perimeter_20260929.sh --verify-only
```

## Field sequence

The approved launcher retains all of these safeguards:

1. Blades disengaged; direct supervision; Wi-Fi phone control page open in Pause and physical e-stop ready.
2. Reconfigure and verify the Heading F9P with the non-interactive RAM-only 5 Hz startup profile before starting rtcm-server; this enables USB UBX NAV-RELPOSNED and disables targeted USB NMEA output.
3. Allow startup time for the Wi-Fi phone heartbeat, Teensy bridge, RTK corrections, and fixed heading.
4. Run preflight expecting `teensy_main_20261003_wifi_v3` and require a released Wi-Fi E-stop, RTK corrections, RTK Fixed, valid fixed-carrier heading, 0.80–1.30 m baseline, heading accuracy ≤1.0°, healthy JRK, and stationary phone Pause.
5. Keep the Wi-Fi control page in the phone's foreground, and open the mission dashboard on the laptop. Use guarded phone Manual to reach and align with the initial waypoint. Dashboard voice guidance is optional and may remain off.
6. Start field logging before Pure Pursuit and preserve neutral shutdown traps.
7. Run the first validation supervised and blades off; Pause immediately for unexpected clearance or tracking behavior.
