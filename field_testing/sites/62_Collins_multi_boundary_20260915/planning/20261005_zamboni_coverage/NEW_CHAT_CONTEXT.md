# New-chat context — plan complete Zamboni-style cutting coverage

## Status of this document

**CONTEXT ONLY — NO COVERAGE ROUTE OR LAUNCHABLE MISSION HAS BEEN GENERATED.**

Use this only after the approved 2026-09-29 consolidated perimeter mission has
run successfully and its field log is available. A successful perimeter run
is evidence to analyze; it does not automatically approve a blades-on or
unattended coverage mission.

## Paste-ready prompt for the future coverage chat

> The approved consolidated perimeter field test has completed successfully.
> Read
> `obsidian_vault/02-testing/20261005-wifi-control-boundary-mission-handoff.md`,
> the boundary run context and results, and this coverage context before doing
> any planning. First locate and analyze the new perimeter field log and ask me
> about any observed clearance or tracking concerns. Then build a REVIEW-ONLY
> complete cutting-coverage map for all mowable areas inside the validated
> boundary while excluding every obstacle with the full 42-inch deck envelope.
> Use 1.00 m/s speed commands. Use forward, soft Zamboni-style row transitions:
> skip the immediately adjacent row, cover it on a later pass, and ensure every
> planned row and all safely coverable area are ultimately covered. Avoid the
> tight, roughly 1.2 m-radius hard U-turn style. Reconcile the revision-2 site
> inventory with the larger review-only obstacle set used by the approved
> perimeter package before generating geometry. Produce auditable static and
> interactive review artifacts, coverage-gap and deck-clearance validation,
> and clearly label everything REVIEW ONLY. Do not create or enable a field
> launcher and do not call the result approved until I visually review it.

## User requirements

- Cover the cutting area inside the validated boundary.
- Avoid every obstacle, including the mower deck footprint rather than only
  the antenna/tractor centerline.
- Include all three mowable areas and safely cover all reachable portions.
- Use speed commands of 1.00 m/s.
- Prefer forward-only soft turns.
- Use a Zamboni-style ordering that skips the immediately adjacent row and
  returns to skipped rows later.
- Do not use adjacent-row hard U-turns near the approximately 1.2 m-radius
  style observed in earlier geometry.
- Do not silently reduce the requested speed or change the skip-next-row
  concept. If geometry or field evidence makes either unsafe, stop and discuss
  the conflict with Al.
- First output is review-only. No deployment, approval flag, or launchable
  mission until Al approves the visual replay and validation.

## Required evidence before geometry generation

1. Confirm the consolidated perimeter run completed successfully.
2. Locate its field log under
   `/home/al/field_logs/20260929_consolidated_perimeter_1mps/`.
3. Analyze actual path error, heading quality, controller pauses, speed,
   steering saturation, and clearance-related operator observations.
4. Compare the driven path with the approved mission and identify any boundary
   section that should not yet be treated as validated.
5. Record the exact Git revision and hashes of all geometry inputs.

If the perimeter run stopped early or exposed a clearance/tracking problem,
do not build full coverage. First update the boundary evidence and return for
owner review.

## Geometry authority must be reconciled

The repository currently contains two different levels of site information:

### Revision-2 site inventory

Authoritative file for the existing coverage-planning model:

`field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/62_Collins_site_inventory.geojson`

It currently records:

- 3 mowable polygons;
- 2 obstacle assets;
- 8 named access points;
- 5 directional between-area transition routes;
- 3,389.877 m² total mowable area.

Its status is `reviewed_not_field_validated`, and it explicitly contains no
launchable mission.

### Perimeter package obstacle source

The approved perimeter package was validated against:

`site_inventory/revisions/rev_003_20260926_REVIEW_ONLY/62_Collins_site_inventory_rev003_REVIEW_ONLY.geojson`

Its validation report checks fourteen `OBS-NEW-*` exclusions plus the pole and
tree exclusions—sixteen obstacle exclusions in total. Some have little nominal
deck-edge clearance. The revision-2 inventory's two obstacles are therefore
not sufficient by themselves for the requested coverage guarantee.

Before generating coverage, create an explicit reconciliation report that:

- identifies which boundary and obstacle geometries are supported by the
  successful perimeter run;
- retains every relevant exclusion used by the approved perimeter package;
- prevents double-applying an exclusion adjustment already embedded in an
  obstacle polygon;
- proposes a new reviewed inventory revision without overwriting revision 2;
- requires Al's review before it becomes the source for coverage.

## Vehicle and cutting geometry

- Mower deck width used by the perimeter validator: 1.0668 m (42 inches).
- Historical 38-inch row spacing: 0.9652 m, providing approximately 0.1016 m
  nominal overlap across a 42-inch deck. Treat this as historical context and
  confirm the desired effective cutting width before finalizing stripes.
- Historical weaker-right-turn planning radius: 1.63 m with no margin.
- Historical four-row-skip review test used a 1.90 m radius and skipped three
  intermediate 38-inch rows. It demonstrated useful soft-turn geometry but is
  not the requested skip-one-row ordering and must not be copied unchanged.
- That historical route also had long modulo-wrap relocations crossing the
  core, which would create unwanted diagonal cut marks in production.
- The future planner should test 1.90 m or larger soft-turn candidates, or
  another radius supported by field evidence, and report the margin. It must
  not assume that 1.2 m is an acceptable production turn radius merely because
  the tractor has driven some tight boundary features.

## Zamboni-style planning intent

Interpret “skip the row next” as avoiding an immediate adjacent-row reversal.
A simple conceptual ordering is one parity of rows followed by the other, for
example `1, 3, 5, ...` and later `2, 4, 6, ...`, adjusted per decomposed cell
and row count. The actual ordering must be optimized so that:

- every stripe is covered exactly as intended;
- no narrow sliver is silently omitted;
- each connector remains inside the permitted tractor-center region;
- the full deck envelope remains inside the mowing boundary and outside all
  obstacle exclusions;
- turn curvature changes smoothly and avoids hard U-turn/keyhole behavior;
- long relocations do not cut diagonal scars across finished rows;
- transitions between cells and mowable areas use the named access points and
  recorded directional routes;
- forward-only operation is preserved unless Al explicitly approves another
  maneuver.

Where a two-row separation cannot fit the chosen soft radius, do not tighten
the turn automatically. Consider cell decomposition, a different stripe
orientation, additional headland space, a larger skip pattern for that local
cell, or leaving a clearly reported manual-only pocket.

## Required review-only outputs

The future work should produce, at minimum:

- source/input manifest with hashes and inventory revision;
- reconciled boundary and obstacle GeoJSON marked REVIEW ONLY;
- row/cell ordering report showing how skipped rows are recovered;
- turn report with radius, direction, curvature, containment, and clearance;
- waypoint audit with lineage, speed, lookahead, row/cell, connector, and
  obstacle-clearance fields;
- static full-site preview with deck swath, boundaries, obstacles, access
  routes, row numbers, and turn markers;
- interactive replay/map with layer controls and route sequence;
- quantitative deck-coverage union showing uncovered and over-covered areas;
- per-area and whole-site coverage percentages;
- minimum centerline and deck-edge clearance for every obstacle;
- validation of gaps, duplicate points, reversals, self-intersections,
  curvature discontinuities, and maximum waypoint spacing;
- estimated route length and run time at 1.00 m/s.

Every output must say `REVIEW ONLY — NOT APPROVED FOR FIELD USE`. Do not create
an enabled launcher, approval flag, or production mission until after Al's
visual review and an explicit later request.

## Useful historical references

- `field_testing/sites/62_Collins_polygon_1/mission_plans/20260831_speed_and_skip_turn_tests/REVIEW_TEST_NOTES_20260831.md`
- `field_testing/tools/build_skip_row_coverage_mission_20260831.py`
- `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/README.md`
- `field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/generated/62_Collins_consolidated_perimeter_1mps_20260929_validation.json`
- `field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20260923_combined_coverage_review/README.md`
