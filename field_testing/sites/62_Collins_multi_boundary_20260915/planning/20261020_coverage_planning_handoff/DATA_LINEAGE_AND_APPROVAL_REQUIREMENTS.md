# Coverage data lineage and approval-tool requirements

## Why this document exists

The current files contain enough information to reproduce the geometry, but
important choices are embedded in Python constants and prior conversations.
That makes the process feel like a black box and makes it unsuitable as the
repeatable process for six yards.

The next tool must expose those choices, show how the raw evidence changes at
each step, and record Al's decision. A generated map is an output of that
process—not a substitute for it.

## What actually happened to the September 26 latitude/longitude data

The raw source is:

`field_testing/sites/62_Collins_multi_boundary_20260915/runs/20260926_phone_manual_perimeter_extension/field_test_20260926_121311.csv`

The transformation is implemented in:

`field_testing/tools/build_62_collins_site_inventory_rev003_20260926.py`

### 1. Rows were filtered

The script parsed `time`, `lat`, `lon`, `speed_mps`, GPS quality, and heading
fields. It retained rows only when:

- `trans_mode` was `0` or `1`; and
- reported speed was at least `0.12 m/s`.

Points less than `0.05 m` from the previously retained point were then skipped
inside each selected interval.

### 2. Latitude/longitude was converted to a local metre grid

The origin was taken from the first row of:

`runs/20260915_133923/boundary_review/segment_02_candidate_path.csv`

The `Frame` conversion from the earlier combined-coverage review converted each
latitude/longitude pair to local east/north metres. Derived geometry was later
converted back to longitude/latitude for GeoJSON output.

### 3. Obstacle tracks were selected by hard-coded time windows

`OBSTACLE_SPECS` in the generator assigns time intervals from the raw log to
`OBS-NEW-001` through `OBS-NEW-014`. A separate interval refines
`OBS-POLE-001`. Four obstacles use shorter `CIRCLE_FIT_INTERVALS` to remove
straight ingress/egress portions from the circle fit.

This is the largest lineage gap: the time windows are reproducible in code, but
most physical obstacle identities were not recorded. That is why the output
contains names such as `Unclassified surveyed obstacle 1`. The geometry can be
traced to source times, but the real-world name still needs owner verification.

### 4. Most obstacle keep-outs used a fitted-circle method

For most obstacle loops, the generator:

1. fitted a least-squares circle to the selected local XY points;
2. treated that fitted circle as the tractor/GPS centerline;
3. subtracted half of the 42-inch deck width (`0.5334 m`) from the fitted
   radius; and
4. created a circular mowing exclusion using the resulting radius, with a
   minimum radius of `0.15 m`.

In plain language: the tractor was driven around the object, and the inner deck
edge inferred from that drive was used to estimate the obstacle keep-out.

### 5. Three irregular obstacles used a path-based method

`OBS-NEW-003`, `OBS-NEW-005`, and `OBS-NEW-011` did not use a circle as their
active candidate. Their recorded points were divided into 72 angular bins
around the fitted center. The closest point in each bin formed an approximate
minimum recorded loop; missing bins were interpolated when at least 75% of the
angular bins contained evidence. That loop was buffered outward by `0.3048 m`
(12 inches).

### 6. A second polygon alternative was also calculated

For comparison, the script created a convex hull around each recorded loop and
shrunk it inward by half the deck width. Revision 3 therefore contains both an
active candidate exclusion and a polygon alternative. A later consumer must
select one; it must not apply both as independent obstacles.

### 7. The nearest mowing area was assigned automatically

Each derived obstacle center was assigned to the closest candidate mowing-area
geometry. This was a geometric association, not a separately recorded field
statement.

### 8. Later owner information changed one item

`OBS-NEW-012` was originally a low-confidence single loop. It was later
identified as a false obstacle/turnaround. The Stage 0 reconciliation retains
the identifier as rejected provenance but creates no active keep-out. Other
obstacle identifiers were not renumbered.

## What is repeatable today, and what is missing

Repeatable from the repository:

- the exact raw source file;
- source time interval for each obstacle candidate;
- the filtering rules;
- the local coordinate conversion;
- the fitted centers, radii, quality statistics, and keep-out polygons;
- the generator code and output reports.

Not adequately recorded yet:

- why every particular time interval was declared an obstacle survey;
- the physical identity of most obstacles;
- a durable owner decision for every candidate boundary, obstacle, access
  point, and route;
- a clear distinction between “reviewed on screen,” “approved as source data,”
  “field validated,” and “approved for mission generation”; and
- a reusable configuration format that removes site-specific time windows and
  assumptions from Python constants.

## Required input and verification tool

The first deliverable in the dedicated chat should be a local, user-facing
tool. A browser-based HTML application with JSON import/export is a good
default because it can show maps and evidence while keeping the saved decisions
machine-readable. Another format is acceptable if it provides the same
visibility and audit trail.

### Tab 1 — Site and sources

User-selectable or editable fields should include:

- site ID and friendly name;
- boundary waypoint file or files;
- raw GPS/tractor log used for boundary and obstacle evidence;
- existing obstacle inventory, if any;
- transition-path source file or files;
- coordinate-frame origin/source;
- mower deck width;
- candidate lane spacing/overlap;
- turn-radius assumptions, clearly marked as later-stage constraints;
- output directory; and
- notes about how the field survey was performed.

The tool should show file existence, file hashes, detected columns, coordinate
ranges, and basic validation before permitting the next tab.

### Tab 2 — Boundary evidence

For each cutting area, show:

- the raw selected path;
- the source file and time/row interval;
- the proposed polygon;
- additions/removals relative to any previous inventory;
- GPS quality summary;
- editable name and area assignment; and
- decision: `needs review`, `approved as source geometry`, `reject`, or
  `needs resurvey`, with notes, reviewer, and date.

### Tab 3 — Obstacle evidence

For each candidate, show:

- recorded GPS loop and a replay/focus control;
- source row/time interval;
- real-world obstacle name and type entered by Al;
- cutting-area assignment;
- geometry method and all parameters;
- fitted-circle, path-based, and polygon alternatives when applicable;
- numerical fit/closure/RTK quality information;
- proposed active keep-out; and
- decision: `approve`, `reject`, `needs resurvey`, or `needs adjustment`, with
  comments, reviewer, and date.

The tool must make it impossible to approve two active keep-out geometries for
the same obstacle accidentally.

### Tab 4 — Entry/exit points and transitions

For every point and route, show:

- map geometry;
- source file/segment;
- start and end areas or locations;
- recorded travel direction;
- whether reverse travel was actually validated;
- editable friendly name; and
- owner decision and comments.

“Confirm this named route” should become a concrete record showing exactly
what was confirmed.

### Tab 5 — Approval summary and export

The tab title should display progress, for example:

- `1 Sources — complete`
- `2 Boundaries — 2/3 approved`
- `3 Obstacles — 9/15 approved`
- `4 Transitions — needs review`
- `5 Dataset approval — blocked`

Export should be blocked while required items are unresolved. When complete,
the tool should produce:

- a versioned approved site-input JSON or GeoJSON;
- a human-readable approval report;
- an immutable decision/audit ledger;
- hashes of all source files;
- the exact transformation parameters;
- explicit rejected/retired records without identifier reuse; and
- a status that distinguishes source approval from field validation and mission
  approval.

## Reusable six-yard design

Site-specific values must live in a saved project/configuration file, not in
Python constants. The application and schema should be reusable so a new yard
starts by selecting new source files and answering the same review questions.

Suggested high-level project structure:

```text
site-project.json
raw_sources/
derived_review/
approval_ledger.json
approved_site_inventory.geojson
planning_stages/
```

Each derived feature should carry:

- a stable feature ID;
- source file hash;
- source row/time selection;
- transformation method and parameters;
- parent feature or prior revision;
- decision state;
- reviewer/date/comments; and
- field-validation and mission-approval states kept separate.

## Gate before coverage planning

Do not begin section/cell planning until the tool exists and the 62 Collins
dataset has been reviewed through it. The first run through 62 Collins is also
the usability test for the workflow that will later serve the other five yards.
