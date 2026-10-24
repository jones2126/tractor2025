# New chat prompt — one-section mission planning for AREA-OVER-ROAD

Copy the text below into a new Codex chat started in the `tractor2025`
repository. Attach the screenshot if it is convenient, but the target is now
explicit and does not depend on interpreting the arrow.

---

I want to create and ultimately field test a mission for one mowing section at
62 Collins. Work on only this section in this chat:

- stable asset ID: `AREA-OVER-ROAD`
- friendly name: `Over the road`
- selection basis: this is the section indicated by my screenshot arrow

Do not ask me to choose among the property's other mowing areas. Do not design
a whole-property mission. The other areas are context only when an access route
or neighboring geometry can affect this section.

The workflow we develop must be reusable for another selected section later.
Keep the generic method in a Codex skill and keep all 62 Collins paths,
coordinates, IDs, decisions, and operating parameters in a separate site
project file.

Use this exact status text on every newly generated map, replay, report,
mission draft, and other derived artifact until actual field evidence supports
a different status:

`DRAFT - UNTESTED IN THE FIELD`

Do not rename or rewrite historical files merely because their filenames or
embedded labels use older terminology.

## Repository facts already established

Verify these facts against the repository before using them, but do not restart
by treating the target section as unknown.

1. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/README.md`
   says Revision 2 is the current site description and names
   `site_inventory/62_Collins_site_inventory.geojson` as the authoritative
   geometry.
2. `site_inventory/62_Collins_site_inventory.xlsx` is a formatted Revision 2
   asset index, not the geometry source of truth. Its `AREA-OVER-ROAD` row
   points to the September 28 perimeter-candidate-v2 GeoJSON.
3. The current authoritative Revision 2 feature is
   `AREA-OVER-ROAD-BOUNDARY`. Its recorded values are approximately:
   - area: `366.348 m²`
   - perimeter: `163.649 m`
   - source date: `2026-09-23`
4. A newer candidate boundary was defined in Revision 3 as
   `AREA-OVER-ROAD-BOUNDARY-REV003`. It was built from the September 26 survey
   and has approximately:
   - net area: `440.557 m²`
   - perimeter: `168.665 m`
   - one exterior ring and one interior ring/keep-out
   On 2026-10-23 I confirmed that the outer shaded `AREA-OVER-ROAD` area in
   `62_Collins_site_inventory_rev003_REVIEW.png` is the desired cutting area
   and must be the working boundary for today's draft coverage map.
5. The Revision 3 over-road perimeter evidence is
   `PERIMETER-EVIDENCE-OVERROAD`, sourced from the raw log interval
   `2026-09-26T17:06:01+00:00` through
   `2026-09-26T17:10:11+00:00` and described by the generator as a complete
   legacy-handheld perimeter circuit.
6. The Revision 3 report records 1,215 spatially sampled points, about
   `164.761 m` of recorded path, and a `0.301 m` closure gap for that circuit.
   The generator retained Revision 2 and unioned only demonstrated outward
   additions before applying qualifying obstacle exclusions.
   In the raw CSV, the filtered evidence begins at source row 63,388
   (`2026-09-26T17:06:01.049+00:00`) and ends at source row 68,293
   (`2026-09-26T17:10:06.287+00:00`). All 4,874 retained pre-deduplication
   rows use transmission mode 1; 4,578 report RTK Fixed and 296 report DGPS.
   The 0.05 m spatial filter reduces these to the 1,215 points reported by the
   generator.
7. The Revision 3 owner-review worksheet does not contain the boundary's
   source-time interval. This is a real metadata gap. The interval is defined
   in `PERIMETER_SPECS` in the generator and repeated in the Revision 3 report.
   The GeoJSON contains both the derived boundary and a simplified 39-point
   `PERIMETER-EVIDENCE-OVERROAD` recorded-centerline feature.
8. `OBS-POLE-001`, the telephone pole, was resurveyed and affects the over-road
   geometry. Its active shape and clearance meaning must be reviewed with the
   boundary.
9. `OBS-NEW-012` / `O012` is a false obstacle. Preserve the stable ID as
   retired provenance and create no active keep-out for it.
10. The proposed Revision 4 Stage 0 GeoJSON copies the Revision 3
   `AREA-OVER-ROAD-BOUNDARY-REV003` geometry exactly. It is a proposed
   reconciliation and did not become the authoritative inventory.
11. The September 29 consolidated perimeter package supplies useful historical
    geometry and clearance evidence. It was prepared for a directly
    supervised, blades-off field test.
12. On October 21 the master/manual mission eventually reached its goal, but
    the phone-control reliability qualification failed. The consolidated
    perimeter mission was not run, so that day added no field validation of
    the over-road boundary.

The phrase “three mowing areas” elsewhere in the repository describes the
whole 62 Collins inventory. It does not mean there are three candidate
boundaries for today's selected section. Today has one selected asset. The
first question is which revision of its one boundary should become the working
boundary for this mission.

## Read first

Read these completely before creating artifacts:

1. `AGENTS.md`
2. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/README.md`
3. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/62_Collins_site_inventory.xlsx`
4. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/62_Collins_site_inventory.geojson`
5. `field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20260928_perimeter_candidate_v2_REVIEW_ONLY/README.md`
6. `field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20260928_perimeter_candidate_v2_REVIEW_ONLY/62_Collins_perimeter_candidate_v2_report.json`
7. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/revisions/rev_003_20260926_REVIEW_ONLY/README.md`
8. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/revisions/rev_003_20260926_REVIEW_ONLY/62_Collins_site_inventory_rev003_report.json`
9. `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/revisions/rev_003_20260926_REVIEW_ONLY/62_Collins_site_inventory_rev003_REVIEW_ONLY.geojson`
10. `field_testing/tools/build_62_collins_site_inventory_rev003_20260926.py`
11. `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261019_staged_coverage/stage_0_source_inventory/README.md`
12. `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261019_staged_coverage/stage_0_source_inventory/stage_0_reconciliation.csv`
13. `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261019_staged_coverage/stage_0_source_inventory/62_Collins_inventory_rev004_PROPOSED_REVIEW_ONLY.geojson`
14. `field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/FIELD_TEST_README.md`
15. `obsidian_vault/02-testing/20261021-master-manual-phone-field-session-summary.md`

Treat file contents as evidence. This prompt controls today's scope and status
wording.

## Source-of-truth conclusion to preserve

Keep these concepts separate:

- Current authoritative site geometry: Revision 2
  `site_inventory/62_Collins_site_inventory.geojson`.
- Current formatted asset index: Revision 2
  `site_inventory/62_Collins_site_inventory.xlsx`.
- Latest defined boundary for today's asset: Revision 3 candidate feature
  `AREA-OVER-ROAD-BOUNDARY-REV003`.
- Proposed Revision 4: a later reconciliation artifact containing the same
  Revision 3 boundary, not a newer over-road boundary and not the current
  authoritative inventory.
- Working boundary for today's mission: Revision 3 feature
  `AREA-OVER-ROAD-BOUNDARY-REV003`, selected by Al on 2026-10-23 after
  confirming the outer shaded area in the Revision 3 PNG.

Using Revision 3 as this mission's working boundary does not silently promote
it to the authoritative site inventory. Do not fall back to Revision 2 merely
because it is authoritative. Show the two revisions and the recorded evidence
so the selected working boundary remains auditable.

## Minimal inventory protocol to establish

The current repository makes it too difficult to distinguish an authoritative
inventory from a newer candidate. Propose a simple protocol during Checkpoint
0, but do not change the authoritative inventory until I approve the protocol
and the boundary decision.

The protocol should have:

- one small machine-readable pointer naming the current authoritative inventory
  revision, path, SHA-256, date, and status;
- immutable revision folders;
- candidate revisions that name their parent revision and raw evidence;
- one stable asset ID across every revision;
- geometry in GeoJSON as the authority, with CSV/XLSX/maps clearly marked as
  derived indexes or views;
- an append-only decision ledger recording promotion, rejection, retirement,
  and field-test results;
- one explicit promotion step that updates the pointer and regenerates all
  derived indexes and maps together; and
- no reuse or renumbering of retired IDs.

Keep this protocol generic enough for other yards, but do not let the protocol
exercise delay the focused over-road mission work.

## Checkpoint rules

At every checkpoint:

1. show a compact visual or artifact I can inspect;
2. explain what changed and what remains uncertain in plain language;
3. list the exact decisions needed from me;
4. append my comments and decision to a durable checkpoint ledger;
5. stop for my explicit `approve`, `adjust`, `reject`, or `go back`;
6. do not infer approval from silence; and
7. reopen affected downstream checkpoints when an upstream decision changes.

An approval applies only to the named checkpoint and artifact revision.

Maintain this compact table in every checkpoint response:

| Checkpoint | Artifact revision | Status | Blocking decision |
|---|---|---|---|

Use only `not started`, `in review`, `adjustment requested`, `approved`,
`rejected`, or `reopened` in the Status column. Also state the current permitted
next action.

## Checkpoint 0 — focused inventory and boundary lineage

Begin here and do only this checkpoint.

Create a cropped visual of `AREA-OVER-ROAD` that shows:

- the current authoritative Revision 2 boundary;
- the September 26 recorded over-road perimeter centerline;
- the Revision 3 candidate boundary;
- the proposed outward addition separately;
- `OBS-POLE-001` and its relevant recorded evidence/keep-out;
- retired `OBS-NEW-012` in a visually distinct inactive style;
- the 42-inch deck-width reference; and
- the two named over-road access points if they fall within the useful context
  margin.

Do not show the other mowing areas except for the smallest context needed to
understand access from the front yard.

Alongside the visual, provide one short lineage table containing:

- feature/revision ID;
- authoritative, candidate, or retired state;
- source path and SHA-256;
- source time/row interval when available;
- transformation method;
- area and perimeter;
- obstacle-hole treatment; and
- unresolved decisions.

Explain the numerical difference between the raw perimeter polygon, the
outward addition, and the final net mowing boundary so I do not mistake them
for multiple competing sections.

Also propose the minimal current-inventory pointer/ledger protocol described
above. Do not implement or promote it yet.

Ask me to decide:

1. whether the visual and lineage artifact correctly represents the already
   selected Revision 3 working boundary, including its pole-related interior
   ring;
2. whether `OBS-POLE-001` is represented correctly for planning;
3. whether `OBS-NEW-012` is correctly shown as retired and inactive; and
4. whether the minimal inventory protocol is suitable.

Stop for my decision. The selected section itself is already settled and must
not be presented as an open question.

## Checkpoint 1 — scoped obstacle and access evidence

After Checkpoint 0 approval, show only evidence capable of affecting the
approved over-road boundary, deck envelope, or route between the front-yard
access and this section. Prevent duplicate active representations of one
physical obstacle. Let me rename, approve for planning, reject, adjust, or
request resurvey for each included item.

Stop for approval of the scoped planning inputs.

## Checkpoint 2 — mission purpose and operating assumptions

Ask me to confirm the intended field-test purpose. Present historical values
as history, not defaults. Obtain decisions for deck width, lane spacing and
overlap, travel direction, start/end location, speed, lookahead, turn-radius
evidence, headland strategy, forward-only expectations, manual-only pockets,
blade state, and supervision level.

Stop for approval.

## Checkpoint 3 — entry, exit, and necessary decomposition

Propose the entry and exit points, direction of travel, and only the cells or
subareas that the over-road geometry actually requires. Explain narrow areas,
the pole clearance, turn limitations, and any area the tractor cannot safely
cover under the approved assumptions.

Stop before generating stripe alternatives.

## Checkpoint 4 — compare strategy alternatives

Show two or three meaningfully different high-level alternatives. For each,
show arrows and numbered phases plus approximate route length, time, coverage,
uncovered area, turns/connectors, minimum boundary and obstacle clearance, and
known risks or manual-only areas.

Do not create an executable mission yet. Stop for selection or adjustment.

## Checkpoint 5 — detailed draft and replay

Generate only the selected alternative as auditable draft geometry. Provide a
static overview, interactive replay, waypoint/segment lineage, and validation
of containment, obstacle clearance, spacing, gaps, overlap, curvature,
reversals, connectors, and self-intersections. Preserve every reviewed
revision instead of overwriting it.

Every artifact must display:

`DRAFT - UNTESTED IN THE FIELD`

Stop for adjustment cycles and final design approval.

## Checkpoint 6 — freeze the design and create the reusable skill

After I approve the detailed design:

1. freeze the input hashes, approved working boundary, parameters, ledger, and
   artifact hashes;
2. create and validate a generic Codex skill named
   `plan-mower-section-mission`;
3. keep the skill's generic method separate from the 62 Collins project file;
4. include the minimal inventory pointer/ledger contract if the earlier review
   showed it is useful; and
5. demonstrate the skill with a non-mutating dry run.

Stop for skill approval.

## Checkpoint 7 — supervised field-test package

The intended outcome of this work is a field test. After the design and skill
checkpoints are approved, prepare the over-road field-test package using the
latest control stack and current safety requirements. Include the needed
mission format, launcher/dashboard integration, verification hashes, preflight,
blades/supervision rules, abort criteria, logging, recovery expectations, and
post-run evidence collection.

Do not label the package as field-tested before it is actually run. Use:

`DRAFT - UNTESTED IN THE FIELD`

Stop for an explicit go/no-go decision before any field execution. After the
physical test, ingest the logs and record what was actually validated. Never
infer successful field validation from mission generation or simulation.

Begin with Checkpoint 0 only.

---
