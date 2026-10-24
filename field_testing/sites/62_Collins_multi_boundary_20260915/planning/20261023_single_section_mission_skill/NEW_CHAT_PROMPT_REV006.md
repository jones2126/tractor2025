# New chat prompt — build a draft mission from the approved AREA-OVER-ROAD asset (Revision 6)

Copy the text below into a new Codex chat started in the `tractor2025`
repository. The boundary-capture and inventory-promotion work is complete. Do
not restart that investigation unless a verification check fails.

---

I want to build and ultimately field test a mission for one mowing section at
62 Collins:

- asset ID: `AREA-OVER-ROAD`
- friendly name: `Over the road`
- scope: this section only

The workflow developed in this chat must be reusable for other selected mowing
areas later. Keep generic procedure in a Codex skill and keep 62 Collins paths,
coordinates, asset IDs, approvals, and operating parameters in a separate site
project file.

Use this exact status text on every newly generated map, report, replay,
mission draft, and derived artifact until a physical field test supports a
different status:

`DRAFT - UNTESTED IN THE FIELD`

Do not rename or rewrite historical evidence merely because it uses older
status wording.

## Approved starting point

The authoritative site inventory is now Revision 4:

- pointer:
  `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/current_inventory.json`
- authoritative geometry:
  `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/62_Collins_site_inventory.geojson`
- expected authoritative GeoJSON SHA-256:
  `703a6b03845598cc4ef6596369fa13ebef43c28b2c7df176aa77d1de7571817b`
- formatted derived index:
  `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/62_Collins_site_inventory.xlsx`

The current focused asset pointer is:

`field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/assets/AREA-OVER-ROAD/current.json`

It names this approved draft-planning package:

`field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/assets/AREA-OVER-ROAD/revisions/rev_001_20261023_DRAFT_UNTESTED/`

Use this file as the direct geometry input for draft mission planning:

`AREA-OVER-ROAD_mowable_area.geojson`

Expected SHA-256:

`692514bc11c25340e1424f79d87e0e29b3da6aea44920a031d10115ba50a2a93`

Verify the pointer and hashes before planning. If they match, accept the
boundary and obstacle decisions below as settled inputs. Do not ask me to
choose between Revision 2 and Revision 3 again.

## Decisions already made by Al on 2026-10-23

1. The exact Revision 3 `AREA-OVER-ROAD` shape was visually reviewed and is
   approved as the input for building a draft mission.
2. `OBS-POLE-001` is a real telephone-pole obstacle. Its active mowing
   exclusion is confirmed.
3. `OBS-NEW-012` was a turnaround maneuver, not an obstacle. Retain its stable
   ID and location as rejected provenance, but create no keep-out.
4. During the perimeter circuit, the edge of the mower deck followed the
   actual grass edge.
5. The promoted planning polygon was not expanded outward after approval. It
   is exactly the reviewed Revision 3 geometry.
6. The direct planning polygon already contains the `OBS-POLE-001` interior
   hole. Do not subtract or buffer the pole exclusion a second time.
7. This approval authorizes draft mission design. It is not evidence that the
   boundary or mission has been field tested.

## Operating assumptions approved during mission review

Treat these as the current site-project values for this draft. Keep them out of
the generic skill defaults; another area must confirm its own values.

1. Purpose: create a draft mission that can be replayed and then tested.
2. Use the recorded line named by `F-out` and `O-out` as the single access
   path in both directions.
3. The physical deck is 42 inches. Plan cutting width and lane spacing at
   38 inches. The discharge chute extends approximately 5 inches on the right.
4. The source boundary was recorded with the tractor's left deck edge on the
   intended cutting boundary.
5. Use 1.0 m/s for straight, headland, turn, and access planning speeds.
6. Begin with 2.0 m lookahead on straights and 1.5 m on U-turns. These are
   test values to monitor, not field-validated constants.
7. Use a 1.4 m planning turn radius. Retain the approximately 1.1 m observed
   hard-left pole turn only as supporting capability evidence.
8. Treat the polygon as one complete mowing area. Use one clockwise outer pass,
   then best-fit long-axis stripes; north and south ends are turn zones.
9. Forward-only travel is required while mowing because selecting reverse with
   the deck engaged triggers the tractor's safety shutoff.
10. Do not preclassify pole-side or edge-sliver ground as manual-only. Plan
    around the pole and include safe edge slivers when forward-only geometry
    allows it.
11. The first physical feasibility test is blades off, with Al alongside the
    tractor and ready to select MODE+PAUSE, MANUAL, or E-STOP.
12. This work changes coverage geometry only. Do not change Teensy firmware or
    the pure-pursuit controller.
13. Crossing or recutting the already-cut perimeter/headland is allowed when it
    creates a smoother forward-only turn.
14. Limited physical mower-envelope overhang beyond the mapped grass boundary
    may be shown for blades-off review, but centerline, deck, chute, obstacle,
    and boundary margins must be reported separately.

Use these plain maneuver names:

- **leave-edge pole loop**: leave the outer pass, make the full pole circle,
  and rejoin from the wider side;
- **minimum-offset stripe detour**: on a later stripe, turn only enough to clear
  the pole and then return to the stripe; and
- **edge-sliver transition**: a smooth forward-only maneuver that includes a
  short boundary stripe without declaring it manual-only.

The direct planning geometry has approximately:

- net mowable area: `440.557 m²`
- perimeter including its pole hole: `168.665 m`
- one exterior ring
- one interior ring for `OBS-POLE-001`

The retained boundary-observation sequence contains 1,215 points from raw
source rows 63,388 through 68,286. It closes within approximately `0.301 m`.

## Read first

Read these completely before creating mission artifacts:

1. `AGENTS.md`
2. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/README.md`
3. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/current_inventory.json`
4. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/62_Collins_site_inventory.geojson`
5. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/62_Collins_site_inventory.xlsx`
6. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/asset_decision_ledger.jsonl`
7. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/assets/AREA-OVER-ROAD/current.json`
8. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/assets/AREA-OVER-ROAD/revisions/rev_001_20261023_DRAFT_UNTESTED/README.md`
9. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/assets/AREA-OVER-ROAD/revisions/rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_manifest.json`
10. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/assets/AREA-OVER-ROAD/revisions/rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_validation.json`
11. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/assets/AREA-OVER-ROAD/revisions/rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_boundary_points.csv`
12. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/assets/AREA-OVER-ROAD/revisions/rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_boundary.geojson`
13. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/assets/AREA-OVER-ROAD/revisions/rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_obstacles.geojson`
14. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/assets/AREA-OVER-ROAD/revisions/rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_mowable_area.geojson`
15. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/assets/AREA-OVER-ROAD/revisions/rev_001_20261023_DRAFT_UNTESTED/AREA-OVER-ROAD_perimeter_evidence.geojson`
16. `field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/FIELD_TEST_README.md`
17. `obsidian_vault/02-testing/20261021-master-manual-phone-field-session-summary.md`
18. `tractor_rpi/pure-pursuit/mission_planning/SITE_COVERAGE_MISSION_WORKFLOW_20260724.md`

Treat the focused asset package and the authoritative Revision 4 inventory as
current inputs. Treat older Revision 2, Revision 3 review, proposed Revision 4,
and October whole-site coverage files as provenance or comparison material.
They do not override the promoted asset.

## Inventory protocol to preserve

- GeoJSON is authoritative geometry.
- `current_inventory.json` points to the current site inventory.
- `assets/<ASSET-ID>/current.json` points to the current revision of one asset.
- Asset revision directories are immutable after promotion.
- CSV, XLSX, HTML, and PNG files are derived indexes or views.
- The append-only decision ledger records promotion, rejection, retirement,
  and later field-test results.
- Stable asset IDs are never reused or renumbered.
- A later field test may promote, revise, or reject this draft asset, but a
  mission generator must not silently edit the source-of-truth geometry.

Use this same contract when another cutting area is selected in the future.

## Checkpoint rules

At every checkpoint:

1. show a compact visual or artifact I can inspect;
2. explain what changed and what remains uncertain in plain language;
3. list the exact decisions needed from me;
4. append my comments and decision to a durable checkpoint ledger;
5. stop for my explicit `approve`, `adjust`, `reject`, or `go back`;
6. do not infer approval from silence; and
7. reopen affected downstream checkpoints when an upstream decision changes.

8. preserve every reviewed revision and create a new revision for adjustments;
9. timestamp replay questions at the actual point of concern and ingest the
   downloaded response file into the project and ledger; and
10. do not treat a visually smooth moving marker as proof of feasible
    curvature—validate segment joins and heading continuity numerically.

Maintain this table in checkpoint responses:

| Checkpoint | Artifact revision | Status | Blocking decision |
|---|---|---|---|

Use only `not started`, `in review`, `adjustment requested`, `approved`,
`rejected`, or `reopened` in the Status column.

## Completed boundary checkpoint

The boundary lineage, telephone-pole decision, turnaround decision, and
inventory promotion are approved. Record them as completed prerequisites. Do
not repeat the old boundary-selection checkpoint unless hash or geometry
verification fails.

## Checkpoint 1 — access evidence and mission-purpose assumptions

Begin here.

Verify the two named over-road access points and the route connection from the
front yard only to the extent needed for this one mission. Then present the
historical operating values as history, not automatic defaults, and ask me to
confirm the intended field-test purpose and the required planning inputs:

- start and end location;
- travel direction;
- entry and exit point;
- deck width;
- lane spacing or intended overlap;
- operating speed;
- pure-pursuit lookahead;
- turn-radius evidence and chosen planning value;
- headland strategy;
- forward-only expectations;
- manual-only pockets;
- blade state;
- supervision level; and
- abort and recovery expectations.

Do not ask me to reapprove `AREA-OVER-ROAD`, `OBS-POLE-001`, or
`OBS-NEW-012`. Stop for approval of the access and operating assumptions.

## Checkpoint 2 — necessary decomposition

Treat the approved polygon as one complete mowing area unless geometry proves
that a split is necessary. Do not label the pole-side strip or an edge sliver
manual-only merely because it is awkward. Explain the narrow shape, pole
clearance, turn limitations, and any ground that remains unreachable after
testing smooth forward-only maneuvers. Stop before generating stripe
alternatives.

## Checkpoint 3 — strategy alternatives

Show two or three meaningfully different high-level alternatives. For each,
show arrows and numbered phases plus approximate route length, time, coverage,
uncovered area, turns/connectors, minimum boundary and obstacle clearance, and
known risks or manual-only areas.

At least one alternative should test stripe ordering as a variable. A short
edge lane can be moved earlier in the stripe sequence when that avoids a long
perimeter relocation. Explain what area every relocation serves. Use plain
maneuver names and distinguish the 1.4 m planning radius from the approximately
1.1 m observed capability.

Do not create an executable mission. Stop for selection or adjustment.

## Checkpoint 4 — detailed draft and replay

Generate only the selected alternative as auditable draft geometry. Provide a
static overview, interactive replay, waypoint/segment lineage, and validation
of containment, pole clearance, spacing, gaps, overlap, curvature, reversals,
connectors, and self-intersections. Preserve reviewed revisions instead of
overwriting them.

The replay must:

- default to a true top-down plan view with the long mowing edge vertical;
- use one identical pixels-per-meter scale for both map axes; never fit the
  horizontal and vertical extents independently;
- verify the top-down projection with a known circular feature: its rendered
  width and height must match within a stated tolerance before review;
- state when the display is rotated from north-up and avoid perspective or
  pseudo-3D presentation;
- provide Restart, Play, Pause, 1x, 5x, and 10x controls;
- provide zoom in, zoom out, reset view, pointer/mouse-wheel zoom, and drag
  panning; provide a focus-current-position control when practical;
- maintain playback position in a floating-point accumulator independent of
  the HTML range input so 1x visibly advances;
- be checked for a nonblank route, question list, Restart behavior, and
  movement at 1x, 5x, and 10x;
- show numbered timestamped questions and export the owner's responses; and
- keep centerline geometry, 38-inch modeled coverage, 42-inch physical deck
  coverage, and conservative deck-plus-chute envelope conceptually distinct.

Before presenting a revised replay, specifically inspect:

- entry into the first stripe for a continuous forward-only path;
- each edge-sliver entry and exit for abrupt heading changes;
- any relocation longer than a normal headland turn and the exact area it
  serves;
- whether stripe reordering removes unnecessary already-cut travel;
- every segment join with a numeric heading-continuity check; and
- playback at real-time 1x, not only accelerated speeds.
- the SVG viewBox and responsive layout preserve aspect ratio at the actual
  review-window size; rotation alone does not establish a top-down view.

For this area, cleanup passes remain deferred until after the first supervised
field run unless Al explicitly changes that decision. A manually driven and
recorded cleanup path may later become repeatable evidence.

Every artifact must display:

`DRAFT - UNTESTED IN THE FIELD`

Stop for adjustment cycles and design approval.

Treat an owner's approval of the route as `approved route concept`. It does
not convert unresolved joins into controller-ready geometry and does not
authorize field motion. Record the approval scope explicitly in the project
file and append-only ledger.

## Checkpoint 5 — freeze the design and create the reusable skill

After I approve the detailed design:

1. freeze input hashes, parameters, ledger entries, and artifact hashes;
2. create and validate a generic Codex skill named
   `plan-mower-section-mission`;
3. keep the skill's generic method separate from the 62 Collins project file;
4. implement the inventory pointer and decision-ledger contract above; and
5. demonstrate the skill with a non-mutating dry run.

Stop for skill approval.

## Checkpoint 6 — supervised field-test package

Prepare the over-road field-test package using the latest control stack and
current safety requirements. Include mission format, launcher/dashboard
integration, hashes, preflight, blades and supervision rules, abort criteria,
logging, recovery expectations, and post-run evidence collection.

Before live preflight, follow the established Tractor01 Heading-F9P startup
sequence from `field_testing/FIELD_COMMANDS_20260930.md`: stop the service that
owns the serial port, apply the 5 Hz moving-base profile to volatile RAM with
`configure_dual_f9p_5hz_profile_20260923.py --heading-startup`, require the
independent readback to pass, restart and require `rtcm-server` active, wait for
reacquisition, and only then run `mission_preflight_20261002.py`. Include this
step in the site launcher documentation and printable field checklist.

The first package revision must be review-only and fail closed:

- set `APPROVED_FOR_FIELD=false` until a separate explicit field-package
  approval;
- disable the dashboard Start action and display the blocking reason;
- provide a launch script that starts the web dashboard without starting
  tractor motion;
- preserve the approved route concept while separately listing every
  controller-readiness blocker, especially sharp segment joins;
- provide a post-run collection script that copies mission, controller,
  phone-control, recovery, service-journal, repository, and network evidence
  into a timestamped archive with SHA-256 records; and
- provide an end-of-session checklist that requires downloading and preserving
  the Archer AXE75 system log from the RTK base station that provides the
  `TractorField` network, including its clock/time-zone context.

Do not automate router login or a machine power-off unless those actions have
their own reviewed and safely configured procedure. Do not change Teensy
firmware or pure-pursuit behavior as part of mission geometry packaging.

Do not label the package as field-tested before it is physically run. Stop for
an explicit go/no-go decision. After the test, ingest the logs and record what
was actually validated.

Begin with Checkpoint 1 only.

---




