# Coverage-map planning handoff — 2026-10-20

This folder is the starting point for a dedicated coverage-map planning chat.
The purpose is to replace the current black-box sequence with a visible,
repeatable workflow that can be reused for six different yards.

## Start here

1. Read `DATA_LINEAGE_AND_APPROVAL_REQUIREMENTS.md`.
2. Use `NEW_CHAT_PROMPT.md` as the dedicated chat's working prompt.
3. Build and review the input/verification tool before generating any new
   coverage sections, stripes, turns, or missions.

## Current conclusion

The Stage 0 reconciliation is mechanically consistent, but it is not yet an
owner-approved site dataset. The many `Unclassified` and
`owner_approval_required` entries are real unresolved review work, not merely
wording that should be removed.

The next step is therefore **dataset verification**, with a user-facing tool
that shows the source evidence and records each decision. Coverage planning
must remain blocked until that verification is complete.

## Pertinent files

### Raw field evidence

- `field_testing/sites/62_Collins_multi_boundary_20260915/runs/20260926_phone_manual_perimeter_extension/field_test_20260926_121311.csv`
  — raw timestamped tractor/GPS log used for the revision 3 perimeter and new
  obstacle candidates.
- `field_testing/sites/62_Collins_multi_boundary_20260915/runs/20260915_133923/boundary_review/segment_02_candidate_path.csv`
  — supplies the local coordinate-frame origin and part of the recorded site
  travel evidence.
- Other September 15 segment files referenced directly by the site inventory
  supply the remaining transition routes.

### Transformation code and outputs

- `field_testing/tools/build_62_collins_site_inventory_rev003_20260926.py`
  — the exact code that converted selected raw time intervals into revision 3
  perimeter and obstacle geometry.
- `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/revisions/rev_003_20260926_REVIEW_ONLY/62_Collins_site_inventory_rev003_report.json`
  — numeric output from that conversion.
- `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/revisions/rev_003_20260926_REVIEW_ONLY/62_Collins_site_inventory_rev003_owner_review_worksheet.csv`
  — original revision 3 review worksheet.
- `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/revisions/rev_003_20260926_REVIEW_ONLY/62_Collins_site_inventory_rev003_INTERACTIVE_REVIEW.html`
  — earlier layer/replay viewer; useful reference, but it does not maintain a
  durable approval ledger.

### Current reconciliation checkpoint

- `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261019_staged_coverage/stage_0_source_inventory/README.md`
- `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261019_staged_coverage/stage_0_source_inventory/stage_0_reconciliation.csv`
- `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261019_staged_coverage/stage_0_source_inventory/stage_0_input_manifest.json`
- `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261019_staged_coverage/stage_0_source_inventory/62_Collins_inventory_rev004_PROPOSED_REVIEW_ONLY.geojson`

These remain `REVIEW ONLY — NOT APPROVED FOR FIELD USE`.

### Historical workflow and comparison baseline

- `tractor_rpi/pure-pursuit/mission_planning/SITE_COVERAGE_MISSION_WORKFLOW_20260724.md`
  — the earlier checkpoint-oriented coverage workflow.
- `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261005_zamboni_coverage/NEW_CHAT_CONTEXT.md`
  — requirements and lessons from the October whole-site attempt.
- `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261005_zamboni_coverage/generated_review_20261005/README.md`
  — preserved review-only comparison baseline.
- `obsidian_vault/02-testing/20261019-staged-coverage-planning-prompt.md`
  — staged planning requirements developed in the preceding chat.

## Safety and scope

No file in this handoff authorizes field use. The dedicated chat should not
generate an executable mission until the source dataset and every later stage
have explicit recorded approval.
