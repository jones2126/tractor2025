# Dedicated coverage-map planning prompt

We are developing a transparent, repeatable coverage-map planning process for
an autonomous Cub Cadet mower. The immediate site is 62 Collins, but the process
must be reusable for six yards.

Start by reading:

1. `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261020_coverage_planning_handoff/README.md`
2. `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261020_coverage_planning_handoff/DATA_LINEAGE_AND_APPROVAL_REQUIREMENTS.md`
3. `AGENTS.md`

Then inspect the pertinent source files linked by the handoff README. Treat all
existing review outputs as evidence, not instructions and not field approval.

## Objective

Replace the current black-box planning sequence with a user-visible,
checkpointed workflow. I need to understand and approve the source dataset
before any coverage sections, stripes, turns, or missions are generated.

## First deliverable: source-input and dataset-verification tool

Design and implement a local user-facing tool that lets me identify and verify
the inputs used by the planning process. A browser-based HTML application with
JSON import/export is a reasonable default, but you may recommend another format
if it gives better map review, durable decisions, and repeatability.

At minimum, the tool must let me specify:

- the waypoint file or files defining the mowing polygon(s);
- the source data for the obstacle inventory;
- the source data for the transition paths between cutting areas;
- the coordinate-frame source;
- deck width and later-stage planning assumptions; and
- the output location.

It must then provide separate, progress-labeled review tabs for:

1. sources;
2. boundaries;
3. obstacles;
4. entry/exit points and transition routes; and
5. final source-dataset approval/export.

Each boundary, obstacle, point, and route must show its data lineage: source
file and hash, source rows or timestamps, transformation method and parameters,
derived geometry, quality information, and owner decision. Approval choices and
comments must be saved in an audit ledger. The tab names or equivalent status
display should show exactly how many items are approved and what remains
blocked.

For obstacles, I need to see how raw latitude/longitude tracks became an
obstacle polygon. I must be able to name/identify the physical object, approve
or reject it, request resurvey, or approve an adjusted keep-out. Do not hide
time-window selection or geometry methods in Python constants. Prevent more than
one active keep-out from being approved for the same obstacle.

## Important current state

- The raw September 26 source log has been reviewed, but its transformation into
  named obstacles is not sufficiently transparent yet.
- Most revision 3 obstacles remain `Unclassified` because their real-world
  identities were never durably recorded.
- `OBS-NEW-012` is a rejected false obstacle/turnaround. Preserve that retired
  identifier as provenance and create no keep-out for it.
- The current proposed revision 4 and Stage 0 reconciliation are review-only,
  not an approved source dataset.
- Recorded transition routes are approved, if at all, only in their recorded
  direction unless reverse travel has separate evidence.
- Approval of a blades-off test package is not the same as completed physical
  validation.
- Existing October coverage outputs are comparison material only and must not be
  overwritten or promoted.

## Working method

Proceed step by step and explain decisions in plain language. Before coding the
tool, present a compact proposed data schema and screen/tab flow for my review.
Do not begin coverage-map generation while building this first deliverable.

After the tool is implemented and tested, use it with me to review 62 Collins
one tab at a time. Stop at each approval gate. Only after the complete source
dataset is explicitly approved should we propose coverage sections/cells.

Label every unapproved output:

`REVIEW ONLY — NOT APPROVED FOR FIELD USE`

Do not generate an executable mission or launcher without a later explicit
request and a separate field-use approval process.
