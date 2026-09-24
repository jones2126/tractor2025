# Master/manual revision — review only (2026-09-20)

This is a separate candidate. The 19,250-point archived mission is unchanged. Do **not** copy this draft to tractor01 or select Auto from it yet. No launcher is provided.

## Requested changes, using original W numbers

- Keep W1–W12652. Join to recorded M18, then follow every recorded M point through M2860. A new, approximately 21-m curved return joins W12845. W12653–W12844 are omitted.
- Keep W13642, use a short direct join to W13704, and omit W13643–W13703.
- Keep W13907, use a short direct join to W13941, and omit W13908–W13940.
- The center of `garden_right_ring_3` is **not** filled in this candidate.

The candidate contains 21,917 rows. New connector IDs are E (entry), R (return), G (garden shortcut), and X (exit shortcut). The audit CSV maps each revised row to its original W, recorded M, or new connector ID. All output speeds are 1.00 m/s, matching the prior all-1.0-m/s request, but this speed is **not approved for these new connectors or tight garden geometry**.

## Safety and data issues to resolve before a field run

- The W12652→M18 entry covers only 0.82 m while changing desired heading by about 28 degrees. Check steering feasibility and the physical entry position.
- The M2860→W12845 return is approximately 21.3 m point-to-point. It has a smooth candidate curve (minimum sampled radius approximately 4.0 m), but its terrain, obstacles, mower clearance, and actual tracking have not been field-verified.
- W13642→W13704 is inside the ring-1 outline, but its arrival heading differs from W13704's stored heading by roughly 34 degrees. It may need a different join or slower validated controller behavior.
- The selected recorded manual path includes 22 DGPS samples among 2,843 retained M points. Inspect those portions of the trace before relying on centimeter-level precision.
- The original planning report rejected the next garden-right headland ring because it was too narrow for the configured 1.63-m right-turn radius, with no margin. Autonomous stripe U-turns remain deferred. A manually captured center-coverage path is the preferred next input; it can then be reviewed and spliced separately.
- As with the original mission, a fresh preflight, RTK-fixed heading, reliable radio, E-stop, blades-off trial, and low-risk supervised test are prerequisites for any later approved build.

The interactive HTML compares the revised route with the archived original. Search by original W, recorded M, connector E/R/G/X, or new row number. Play/Pause moves by 1 or 5 revised waypoints every 120 ms; Previous/Next use the selected step size, and the slider scrubs the whole draft. It works offline; the original route and omitted detours can be toggled. The PNG is a route overview. The audit CSV and JSON report provide exact lineage and checks. Rebuild with `python field_testing/tools/build_master_manual_revision_20260920.py` and then `python field_testing/tools/build_master_manual_revision_viewer_20260920.py` from the repository root; these regenerate only the files in this review directory.

## Manual-trace spacing before a run

The current M18–M2860 section intentionally contains all 2,843 unique recorded positions for route inspection, not a final navigation sampling density. Its median spacing is about 0.04 m, versus about 0.16 m on the remaining original ring-19 segment (other planned phases vary). After the splice geometry is accepted, generate a **new** mission candidate by distance-resampling the M trace to approximately 0.15 m spacing. Keep M18 and M2860 exact, preserve curves and turns to a measured maximum deviation, check for point gaps and heading discontinuities, and retain an audit mapping resampled points back to M numbers. Do this before building a launchable version; do not silently replace this exact-trace review artifact.
