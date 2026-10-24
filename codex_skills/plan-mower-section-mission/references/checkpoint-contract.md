# Checkpoint and artifact contract

Use the smallest checkpoint sequence that keeps decisions auditable. Merge
adjacent checkpoints when the user already supplied the required decision, but
never merge route approval with field authorization.

## Recommended checkpoints

| ID | Output | Required decision |
|---|---|---|
| CP0 | scope, history, skill/project boundary | approve scope |
| CP1 | exact section and input lineage | approve selected outline |
| CP2 | section-affecting boundary, obstacle, and access evidence | approve or disposition inputs |
| CP3 | operating assumptions and permitted maneuvers | approve assumptions |
| CP4 | high-level strategy comparison | select, combine, adjust, or reject |
| CP5 | detailed route, validation, and replay | approve route concept |
| CP6 | frozen hashes and reusable-process update | approve process baseline |
| CP7 | optional disabled blades-off field package | explicit field-package approval |

## Project file minimum fields

- project, site, section, and stable asset IDs;
- status text;
- authoritative input paths, revisions, and SHA-256 values;
- approved access and operating assumptions;
- obstacle decisions and maneuver policies;
- current checkpoint state;
- current artifact names and revisions;
- validation metrics and unresolved blockers; and
- ledger path.

## Ledger event minimum fields

- checkpoint ID and name;
- artifact and revision;
- status and status text;
- recorded date/time and reviewer;
- comments and decision;
- approval scope when approved; and
- permitted next action.

Append only. Supersede mistakes with a correction event instead of editing history.

## Validation minimums

- route continuity and non-zero segment lengths;
- heading change at every segment join;
- minimum constructed turn radius and forward-only compliance;
- effective-cut and physical-envelope coverage/gaps;
- obstacle clearance and boundary overhang for centerline, deck, and asymmetric chute;
- access endpoint continuity;
- speed and lookahead ranges; and
- waypoint audit mapping every output point back to a reviewed segment.

State whether each metric is geometric, modeled, observed, or field validated.

## Approval wording

Prefer `approved planning input`, `approved route concept`, `controller-ready
geometry validated`, `field package approved, blades off`, `authorized for
supervised blades-off run`, or `field tested`. Avoid ambiguous machine-readable
`approved` state.
