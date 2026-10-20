# 62 Collins staged coverage planning — Stage 0

> **REVIEW ONLY — NOT APPROVED FOR FIELD USE**

Stage 0 reconciles the site sources before any new coverage geometry is planned.
It deliberately stops here for Al's review.

## What is in this checkpoint

- 3 candidate mowable-area boundaries from revision 3.
- 15 proposed active obstacle exclusions, with one active polygon per obstacle.
- `O012` / `OBS-NEW-012` retired as a false obstacle. Its identifier remains in
  the provenance record and is not reused, but it creates no keep-out.
- 8 entry/exit points and 5 recorded between-area routes. Route direction remains
  the recorded direction only.
- Exact SHA-256 hashes for 24 boundary, obstacle,
  transition, deck, turn-radius, field-evidence, workflow, and baseline inputs.

The proposed revision 4 file is a review artifact. It does **not** replace the
current authoritative inventory.

## Important evidence limits

- Revision 3 boundaries are still candidate boundaries. Approval of the
  September 29 package for a supervised blades-off test did not prove that every
  boundary was physically completed and validated.
- The 1.63 m weaker-right history and 1.90 m soft-turn review target are recorded
  for later stages only. Stage 0 approves no turn radius.
- The 1.0668 m deck width and historical 0.9652 m lane spacing are recorded, not
  applied to new coverage geometry.
- The October 5 whole-site coverage package remains unchanged as a comparison
  baseline; it is not promoted by this checkpoint.

## Review files

- `62_Collins_stage_0_inventory_map_REVIEW_ONLY.png` — human-readable map.
- `stage_0_reconciliation.csv` — one row per asset plus planning/evidence rows.
- `62_Collins_inventory_rev004_PROPOSED_REVIEW_ONLY.geojson` — proposed clean
  inventory with one active keep-out per real obstacle.
- `stage_0_input_manifest.json` — exact input provenance and hashes.
- `stage_0_validation.json` — mechanical checks (`PASS`).
- `staged_coverage_workflow.json` — the stage gates and current stop point.

## Approval gate

Please review the map and reconciliation table. Stage 1 must not begin until Al
confirms the boundaries, obstacles (especially the unclassified obstacles),
entry/exit points, and recorded route directions are suitable for section/cell
planning.

No cells, stripes, turns, executable mission, or launcher were generated.
