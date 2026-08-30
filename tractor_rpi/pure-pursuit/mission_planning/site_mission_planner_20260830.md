# Site Mission Planner: Heading Qualification, Inward Spiral, and Keyhole Stripes

Date: 2026-08-30
Status: workflow definition and local geometry test; **not yet approved for autonomous field use**

## Purpose

This workflow turns one reviewed perimeter drive into a site-coverage mission in
three deliberately separate stages:

1. qualify the GPS/heading data and select exactly one perimeter lap;
2. generate clockwise, inward rings spaced 38 inches (0.9652 m) apart until the
   remaining turns become too tight; and
3. cover the remaining core with straight stripes and contained keyhole
   (Dubins RLR/LRL) turns.

It follows the review-gate approach in
`SITE_COVERAGE_MISSION_WORKFLOW_20260724.md`. It is not intended to be a
one-button mission generator. Each major geometry decision produces a CSV,
plot, and machine-readable report that an operator must review before the next
stage.

The 2026-08-30 test first demonstrated the two coverage geometries separately.
`build_combined_spiral_stripe_mission_20260830.py` now also produces a single
`REVIEW_TEST` route with a contained forward-only transition from the actual
spiral endpoint to the first core stripe. Static validation passes; supervised
blades-off field behavior is not yet demonstrated.

## Safety boundary

- Mission generation on the development machine does not authorize a tractor
  run.
- Keep the tractor in Pause with blades disengaged during transfer, preflight,
  and controller dry-run work.
- Never use a perimeter merely because a script accepted it. Review the
  original driven path, the simplified polygon, nearby hazards, slopes,
  ditches, property limits, and physical clearance.
- Require `RTK Fixed` base position for production boundary points.
- Treat heading carrier interruptions as a vehicle-readiness issue even when
  the base position remains RTK Fixed.
- A validator result of `REVIEW` is not field approval.
- First execution of any new route must be supervised, low speed, blades off,
  with immediate Pause/E-stop access.

## Current files

| Purpose | File |
|---|---|
| Heading and satellite analysis | `field_testing/tools/analyze_heading_f9p_20260828.py` |
| Boundary extraction and finalization | `tractor_rpi/pure-pursuit/mission_planning/site_boundary_from_log_20260724.py` |
| Clockwise inward spiral builder | `field_testing/tools/build_inward_spiral_mission_20260804.py` |
| Inner stripe/keyhole builder | `field_testing/tools/build_inner_stripe_mission_20260804.py` |
| Combined review/test builder | `field_testing/tools/build_combined_spiral_stripe_mission_20260830.py` |
| Independent mission validator | `tractor_rpi/pure-pursuit/mission_planning/validate_site_mission_20260724.py` |

The two 20260804 builders are useful prototypes, but retain hard-coded Collins
site names and 20260804 output filenames. They also read the raw logger CSV
instead of the reviewed `01_boundary_final.csv`. Those limitations must be
removed before this becomes the normal production workflow.

## Standard planning parameters

These are starting values, not universal tractor limits:

| Parameter | Initial value | Review requirement |
|---|---:|---|
| Lane/ring spacing | 0.9652 m (38 in) | Confirm desired cut overlap and GPS reference point |
| Mower deck width | 1.0668 m (42 in) | Confirm effective cutting width |
| Waypoint spacing | 0.50 m | Independent validator maximum gap <= 0.80 m |
| Spiral lookahead | 2.0 m | Dry-run before field use |
| Stripe lookahead | 1.0 m | Prior stripe execution was not fully proven; retest |
| Spiral/straight speed | 0.85 m/s | Reduce for the first supervised run |
| Keyhole-turn speed | 0.50 m/s | Reduce if steering tracking is poor |
| Planning turn radius | 1.90 m | Must reflect a measured, repeatable tractor radius plus margin |
| Boundary simplification | 0.10 m | Plot must preserve corners and topology |

## Artifact layout

Create a dated plan directory beneath the site:

```text
field_testing/sites/<site>/mission_plans/<date_and_test>/
  00_source/
  01_boundary/
  02_spiral/
  03_inner_stripes/
  04_validation/
  05_release/
```

Do not overwrite an earlier reviewed plan. Preserve source file name, selected
elapsed-time interval, selected data-row interval, parameters, plots, reports,
and SHA-256 hashes.

## Gate 0 - Capture one interpretable perimeter lap

1. Run mission preflight and require the RTCM server and Teensy bridge to pass.
   The LED controller may remain intentionally stopped.
2. Start `field_test_logger_20260828.py` before moving.
3. Drive to the intended perimeter start.
4. Remain stationary for at least 20 seconds.
5. Drive one ordered perimeter without doubling back.
6. Stop at the same point for at least 20 seconds.
7. Return to the parking position only after the end pause.
8. Stop the logger cleanly with `Ctrl+C` and copy the CSV to the development
   machine.

The travel before the first pause and after the second pause is evidence, but
is not part of the boundary lap.

## Gate 1 - Qualify heading and select the lap

Run the heading analyzer. Use explicit elapsed times when pause detection does
not select exactly the intended lap:

```powershell
$repo = 'C:\Repos\tractor2025'
$inputCsv = Join-Path $repo 'field_testing\sites\<site>\runs\<run>\field_test.csv'
$headingOut = Join-Path (Split-Path $inputCsv) 'heading_analysis'

python "$repo\field_testing\tools\analyze_heading_f9p_20260828.py" `
  --input $inputCsv `
  --output-dir $headingOut `
  --start-elapsed <lap-start-seconds> `
  --end-elapsed <lap-end-seconds>
```

Review:

- the pause-selected or manually selected lap limits;
- carrier episodes and their durations;
- `heading_numSV_used`, `heading_numSV_visible`, and heading mean C/N0;
- the same satellite measures for the base-link receiver;
- F9P heading versus course calculated from positions on sufficiently straight,
  moving samples; and
- whether non-fixed carrier episodes correspond to poorer heading agreement.

### Heading qualification policy

For a boundary to advance to mission planning:

- the selected lap must be unambiguous;
- base-position samples used for the polygon must be `RTK Fixed`;
- no fatal GPS/heading device error may be present;
- satellite and C/N0 values must be plausible and free of receiver-wide
  collapse; and
- every heading carrier `float` or `none` episode must be reported, mapped, and
  understood as far as the evidence permits.

Carrier `none` does not automatically corrupt the RTK-fixed latitude/longitude
boundary, but it does prevent the log from proving continuously reliable
moving-baseline heading. A route can be planned from the position data while
vehicle readiness remains on hold.

**Operator checkpoint 1:** record the accepted elapsed times and CSV data-row
range. Do not let a later stage silently choose different limits.

## Gate 2 - Extract and review the official boundary

Use `auto-extract` when the pauses are uniquely detected. Otherwise use the
explicit data rows accepted at Gate 1:

```powershell
$planner = Join-Path $repo 'tractor_rpi\pure-pursuit\mission_planning'
$plan = Join-Path $repo 'field_testing\sites\<site>\mission_plans\<date_and_test>'
New-Item -ItemType Directory -Force "$plan\01_boundary"

python "$planner\site_boundary_from_log_20260724.py" extract $inputCsv `
  --output "$plan\01_boundary\01_boundary_candidates.csv" `
  --plot "$plan\01_boundary\01_boundary_candidates.png" `
  --start-row <data-row-start> `
  --end-row <data-row-end> `
  --min-spacing-m 0.50 `
  --fix-quality 'RTK Fixed'

python "$planner\site_boundary_from_log_20260724.py" finalize `
  "$plan\01_boundary\01_boundary_candidates.csv" `
  --output "$plan\01_boundary\01_boundary_final.csv" `
  --plot "$plan\01_boundary\01_boundary_final.png" `
  --simplify-m 0.10
```

Inspect the original driven path, candidate points, and numbered final polygon.
Confirm:

- one non-crossing closed perimeter;
- correct point order and preferred start location;
- plausible area, perimeter length, and closing-edge length;
- no corner cut across a hazard or outside the intended mowing area; and
- conservative clearance for the GPS reference point, tractor, and deck.

**Operator checkpoint 2:** edit `include`, `sequence`, and notes in the
candidate CSV if needed, then rerun `finalize`. The reviewed
`01_boundary_final.csv` must become the only boundary geometry consumed by both
coverage stages.

## Gate 3 - Choose where the spiral must stop

Build clockwise inward rings at 0.9652 m spacing. Do not choose the number of
revolutions solely because a previous site used 14.

The production planner should sweep candidate revolution counts and stop before
the first candidate that violates any of these conditions:

1. the route or mower envelope leaves the safe polygon;
2. the inward offset becomes empty, splits unexpectedly, or changes topology;
3. sampled turn radius falls below the approved tractor radius plus margin;
4. adjacent rings pinch together or cross;
5. the residual core cannot support useful stripes; or
6. no contained transition to the core plan can be constructed.

The operator should compare at least the last passing candidate and the first
failing candidate. Review the spiral route plot, deck-coverage plot, minimum
and percentile turn radius, outside-boundary length, remaining core shape, and
mission start/end headings.

Current prototype command:

```powershell
python "$repo\field_testing\tools\build_inward_spiral_mission_20260804.py" `
  $inputCsv `
  --output-dir "$plan\02_spiral" `
  --start-row <data-row-start> `
  --end-row <data-row-end> `
  --lane-spacing-m 0.9652 `
  --revolutions <reviewed-count> `
  --waypoint-spacing-m 0.50 `
  --lookahead-m 2.0 `
  --speed-mps 0.85 `
  --simplify-m 0.10 `
  --deck-width-m 1.0668 `
  --smoothing-iterations 60
```

**Prototype warning:** this command currently rebuilds the polygon from the raw
log. It must be changed to accept `01_boundary_final.csv` before production
use.

**Operator checkpoint 3:** approve the chosen revolution count and the exact
residual core. Save the rejected alternative as evidence.

## Gate 4 - Plan straight core stripes and keyhole turns

The stripe planner must use the exact residual core produced at Gate 3, not
independently approximate it with another inset calculation.

Screen several stripe orientations. East/west (`0 degrees` math-frame) is only
a candidate. Consider number and length of stripes, keyhole containment,
slope, drainage, mower discharge, sun/glare, start/end location, and steering
tracking.

For each included stripe:

- trim endpoints so the tractor can enter the connector without leaving the
  safe polygon;
- connect adjacent stripes with a forward-only Dubins keyhole, alternating
  RLR and LRL as required;
- place the keyhole in the already cut ring area, outside the uncut core;
- require the full connector and mower envelope to remain inside the safe
  polygon; and
- exclude or separately review stripes shorter than the configured minimum.

Current prototype command:

```powershell
$spiralMission = "$plan\02_spiral\polygon_1_clockwise_inward_38in_20260804.txt"

python "$repo\field_testing\tools\build_inner_stripe_mission_20260804.py" `
  $inputCsv `
  $spiralMission `
  --output-dir "$plan\03_inner_stripes" `
  --start-row <data-row-start> `
  --end-row <data-row-end> `
  --lane-spacing-m 0.9652 `
  --core-inset-passes <same-reviewed-count> `
  --turn-radius-m 1.90 `
  --waypoint-spacing-m 0.50 `
  --straight-speed-mps 0.85 `
  --turn-speed-mps 0.50 `
  --lookahead-m 1.0 `
  --minimum-stripe-m 3.0 `
  --deck-width-m 1.0668
```

**Operator checkpoint 4:** review every connector, excluded short stripe,
outside-site length, inside-core connector length, residual deck-coverage gaps,
and the first/last pose.

## Gate 5 - Decide whether release uses one mission or two

The current prototypes produce two separate missions:

1. an outer-to-inner spiral; and
2. a stripe mission that starts at the original outer start and transits to the
   first stripe.

That is not a combined continuous mission. Choose explicitly:

### Two-mission release (recommended until integration is tested)

- Run the spiral, stop, Pause, and inspect position/heading.
- Reposition only by an independently reviewed route.
- Run the core-stripe mission as a separate supervised operation.

### One-mission review/test (implemented 2026-08-30)

- The combined builder removes the stripe prototype's outer-start transit.
- It constructs a contained forward-only Dubins transition from the **actual
  spiral endpoint and heading** to the chosen first stripe pose.
- Assign conservative transition speed and lookahead.
- Validate the joined mission, including the join gap, yaw, curvature, safe
  area, and mower envelope.
- Plot the transition separately and as part of the full route.

Do not concatenate waypoint files directly. A coordinate-continuous join can
still demand an impossible heading or turn radius.

## Gate 6 - Independent validation

Run validation independently of the builders:

```powershell
python "$planner\validate_site_mission_20260724.py" `
  <mission-file> `
  --turn-radius-m 1.90 `
  --expected-spacing-m 0.50 `
  --max-speed-mps 0.85 `
  --report <validation-report.json> `
  --plot <validation-plot.png>
```

Production validation should also receive a settings file that reconstructs
the reviewed safe polygon and obstacles. Without it, the independent validator
checks spacing, speed, yaw, duplicates, and curvature but does not independently
check boundary containment.

Release criteria:

- validator status `PASS`;
- zero errors and resolved warnings;
- zero duplicate or sub-centimeter consecutive points;
- maximum gap within the configured limit;
- minimum sampled radius at or above the approved threshold;
- zero route and mower-envelope length outside the safe area;
- all keyhole connectors contained and outside the uncut core;
- plausible speed/lookahead transitions; and
- matching SHA-256 hashes for the reviewed and transferred mission.

**Operator checkpoint 5:** a `REVIEW` or `FAIL` result stops release.

## Gate 7 - Package, transfer, and dry-run

Only after all preceding gates pass:

1. copy final missions, settings, reports, plots, and hashes into `05_release`;
2. use dated, site-specific mission names rather than the prototype 20260804
   names;
3. commit the immutable planning evidence;
4. pull the reviewed commit on tractor01;
5. verify hashes and service versions;
6. run mission preflight;
7. perform a controller dry run with blades off; and
8. conduct the first field test at reduced speed under direct supervision.

Archive the resulting field log with the exact mission hash and planner
parameters. Compare F9P heading with position-derived course again, especially
through all keyhole turns and any non-fixed carrier intervals.

## 2026-08-30 AT340 perimeter test

### Source and lap selection

Source:

```text
field_testing/sites/62_Collins_polygon_1/runs/20260830_heading_at340/
field_test_20260830_heading_at340_perimeter_1.csv
```

Accepted perimeter:

- elapsed time: 184.27 through 401.31 seconds;
- CSV data rows: 3686 through 8027;
- accepted boundary positions: 4,283 `RTK Fixed` rows;
- rejected within the row interval: 55 `RTK Float` and 4 `DGPS` rows.

Heading analysis during the lap:

- heading carrier fixed: 95.49%;
- heading carrier float: 0%;
- heading carrier none: 4.51%, in four episodes totaling 9.54 seconds;
- carrier-fixed heading satellites used: median 32;
- carrier-fixed heading satellites visible: median 50;
- carrier-fixed heading mean C/N0: median 41.3 dB-Hz;
- carrier-none heading satellites used: median 32;
- carrier-none heading satellites visible: median 48;
- carrier-none heading mean C/N0: median 41.6 dB-Hz; and
- position-derived versus F9P heading absolute median error: 2.05 degrees
  during carrier fixed and 3.69 degrees during carrier none.

The non-fixed carrier intervals therefore were **not explained by a collapse in
satellite count or mean C/N0**. They were associated with worse heading
agreement and remain a moving-baseline reliability issue to investigate.

Detailed report:

```text
field_testing/sites/62_Collins_polygon_1/runs/20260830_heading_at340/
AT340_HEADING_ANALYSIS_20260830.md
```

### Boundary result

Test directory:

```text
field_testing/sites/62_Collins_polygon_1/mission_plans/
20260830_at340_combined_test/
```

- 278 spatially thinned candidate points;
- 36 final points after 0.10 m simplification;
- area: approximately 1,770.0 m2;
- perimeter: approximately 155.3 m; and
- closing edge: approximately 0.71 m.

The numbered final-boundary plot was visually coherent and had no apparent
self-crossing.

### Spiral result

The prototype was tested with 14 revolutions:

- final nominal inset: 13.5128 m;
- 2,976 waypoints;
- route length: 1,486.9 m;
- estimated run time at configured speed: 29.15 minutes;
- minimum sampled radius: 2.30 m;
- outside-boundary length with 2 cm tolerance: 0 m; and
- estimated uncut area: 298.05 m2 (16.84%).

Independent validator: `PASS`.

### Inner-stripe result

Using a 14-pass core inset, 0-degree east/west stripes, and 1.90 m keyholes:

- 21 included stripes;
- one excluded 2.50 m short stripe;
- all 20 adjacent-stripe RLR/LRL connectors were reported contained in the
  site and outside the core;
- the prototype's separate outer-start transit was also reported contained;
- 1,163 waypoints;
- route length: 544.5 m;
- estimated run time: 14.34 minutes;
- minimum independently sampled radius: 1.88 m; and
- estimated core residual: 4.80 m2 (1.76% of the stripe planner's core).

Independent validator: `REVIEW`, because one pair of consecutive waypoints is
less than 1 cm apart. This must be removed and the route regenerated before any
field release.

### Combined review/test result

The dated combined builder used the reviewed `01_boundary_final.csv`, retained
the 2,976-point spiral, rebuilt the same 21-stripe core, removed the obsolete
outer-start transit, and added a transition from the actual spiral endpoint:

- transition: forward-only LRL;
- transition length: 12.30 m;
- contained inside reviewed boundary: yes;
- transition length inside the uncut core: 0 m;
- combined waypoints: 4,124;
- combined route length: 2,028.5 m;
- configured-speed estimate: 43.40 minutes;
- sub-centimeter consecutive gaps: 0;
- maximum waypoint gap: 0.50 m;
- minimum independently sampled radius: 1.88 m; and
- independent validator: `PASS`, with no warnings.

The supervised launcher caps the first run at 0.50 m/s, so practical duration
will be approximately 68 minutes rather than 43 minutes. The first run remains
`REVIEW_TEST`, blades off.

### Combined status

| Gate | Result |
|---|---|
| Lap and heading analysis | Complete; four carrier-none episodes documented |
| RTK-fixed boundary | PASS for local planning review |
| Clockwise 38-inch spiral geometry | PASS at 14 test revolutions |
| Original standalone stripe file | REVIEW; retained as historical evidence |
| Combined stripe/keyhole geometry | PASS; sub-centimeter join removed |
| Shared reviewed boundary/core geometry | IMPLEMENTED in combined builder |
| Spiral-end to first-stripe transition | PASS; 12.30 m contained LRL |
| Single combined static validation | PASS; 4,124 waypoints |
| Supervised blades-off field test | READY TO TEST, not yet run |
| Autonomous production release | HOLD |

The combined mission is a real joined route rather than a direct concatenation.
Its reviewed SHA-256 is
`386aa4913ce758a4829ef9809d34ea242cfda6deefc5a33c98316819187a3bb8`.

## Required implementation work

Before making this the standard site planner:

1. Change both coverage builders to consume the reviewed
   `01_boundary_final.csv` and an optional reviewed obstacle file.
2. Replace hard-coded site names, dates, and filenames with command-line
   parameters and a dated manifest.
3. Make the spiral builder emit its exact residual core as a reviewed artifact;
   the combined builder currently derives the core from the reviewed boundary
   and the same 14-pass inset.
4. Add a candidate sweep that selects the last spiral revolution count passing
   radius, containment, topology, spacing, and usable-core checks.
5. Add stripe-angle comparison with rendered keyhole geometry, not just line
   count.
6. Apply the combined builder's sub-centimeter waypoint suppression to the
   standalone stripe builder as well.
7. Promote the combined builder from `REVIEW_TEST` only after the supervised
   blades-off run demonstrates the transition and keyhole tracking.
8. Write a settings JSON and geometry audit CSV so the independent validator
   can reconstruct containment and obstacle clearance.
9. Make geometry warnings data-driven. The spiral prototype incorrectly says
   the 20260830 source contained two `Invalid` rows even though its reported
   source counts show none.
10. Refuse to emit a release artifact when any required gate is `REVIEW` or
    `FAIL`.

## Why this workflow can stop

Planning must stop when the evidence is ambiguous: uncertain lap limits,
non-RTK boundary points, an implausible polygon, an unreviewed hazard, turn
radius below the approved limit, a disconnected or unusable core, a keyhole
outside the safe area, an invalid spiral-to-stripe join, or any unresolved
validator warning. Stopping is the safe and expected result; change the
geometry or gather better field evidence rather than bypassing the gate.
