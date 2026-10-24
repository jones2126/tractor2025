# Checkpoint 0 — history and scope confirmation — revision 001

> REVIEW ONLY — NOT APPROVED FOR FIELD USE

Date: 2026-10-23

## Scope

Plan and review exactly one user-selected mowing section at a time. The target
section has not been selected. The screenshot arrow is not treated as a
selection, and no geometry will be inferred from it.

This checkpoint creates no coverage stripes, turns, mission waypoints,
launcher, dashboard adapter, or field authorization.

## Truth table

| Evidence or event | What is established | What is not established | Reuse in this workflow |
|---|---|---|---|
| Revision 3 site inventory and replay | Three candidate mowing-area boundaries; recorded perimeter evidence; reproducible candidate obstacle geometry; replay of the September 26 source log | Owner approval of the site dataset; physical names for most obstacles; field validation; mission approval | Use as review evidence and the initial visual source for identifying one section |
| Proposed revision 4 / Stage 0 reconciliation | Mechanically consistent review artifact; 3 candidate areas; 15 proposed active obstacle exclusions; 8 access points; 5 recorded-direction routes; source hashes; `OBS-NEW-012` retained as rejected provenance with no active keep-out | Approved site inventory; approval of unresolved boundaries, obstacles, access points, or routes | Use as a review index and lineage aid, never as an approved replacement inventory |
| `OBS-NEW-012` / `O012` decision | Owner identified it as a false obstacle/turnaround | Nothing authorizes reviving it as an active keep-out | Preserve the stable ID as retired/rejected provenance and render it inactive |
| September 29 consolidated perimeter package | Static preview and replay were owner-approved for one directly supervised, blades-off field test; it includes useful clearance and geometry decisions | Mowing approval; unattended operation; proof that all candidate boundaries or obstacles were field validated | Reuse only as historical geometry, clearance, and review evidence |
| October 5 whole-site Zamboni package | A preserved comparison package: 3 areas, 15 active obstacles, 97.099% estimated coverage, 8,095.071 m route, 90 radius-validated turns | Field approval or a viable completed design; 42 contained connectors still needed smoothing and 1 connector was uncontained/unresolved | Compare assumptions and validation methods only; do not copy or promote its geometry |
| October 21 master/manual field session | The immutable master mission eventually reached its goal after Manual repositioning and guarded reacquisition; fail-closed phone behavior worked | Passing phone-control reliability qualification; a clean uninterrupted mission; authorization to run Mission 2 | Reuse as control-stack and operational evidence only |
| October 21 consolidated-perimeter outcome | It was not run because the preceding qualification failed | Any new field validation of candidate boundaries, obstacle inventory, or the consolidated perimeter | Record as not executed; make no new geometry claims |
| Later phone Auto-recovery software work | May improve control/recovery behavior | Approval of mowing geometry, review design, or field mission | Keep separate from geometry and mission approvals |

## Proposed reusable skill

Name: `plan-mower-section-mission`

Proposed activation description:

> Plan and review one mower section from traceable boundary, obstacle, access,
> and operating evidence using explicit visual approval checkpoints. Use when a
> user wants to identify one section, review its planning inputs, compare route
> strategies, and freeze an auditable review-only design. Do not use for
> whole-property coverage or direct field-mission authorization.

The skill will not be created until Checkpoint 7 is explicitly reached and the
review design has been approved.

## Proposed skill/project boundary

Reusable skill content:

- checkpoint state machine and explicit approval rules;
- generic evidence-scoping, lineage, visualization, alternative-comparison,
  validation, revision, and reopening rules;
- generic ledger and project-file contracts;
- reusable blank project template only if the completed workflow shows it is
  materially useful;
- no 62 Collins paths, coordinates, IDs, obstacle names, or chosen parameters.

62 Collins project content:

- repository-relative source paths and SHA-256 hashes;
- site, area, boundary, obstacle, access, and route IDs;
- selected-section geometry and lineage;
- site-specific parameters, assumptions, comments, decisions, and artifacts;
- approval ledger and downstream checkpoint dependency state.

Proposed locations after their relevant gates:

- reusable personal skill: `%CODEX_HOME%/skills/plan-mower-section-mission/`;
- site project and checkpoint artifacts: this planning directory;
- one site project file, proposed as `62_collins_section_mission_project.json`;
- one append-only decision ledger, currently started as
  `checkpoint_ledger.jsonl`.

## Proposed checkpoint sequence

| Checkpoint | Purpose | Gate before continuing |
|---|---|---|
| 0 | Confirm history, scope, skill shape, and project boundary | Approve this artifact revision |
| 1 | Identify one exact section by outline, friendly name, and stable ID | Approve the selected outline |
| 2 | Review only the boundary and obstacle evidence capable of affecting that section | Approve or disposition scoped planning inputs |
| 3 | Select mission intent and operating assumptions | Approve the intent and geometry-affecting parameters |
| 4 | Review entry, exit, permitted travel, and only necessary decomposition | Approve access and decomposition |
| 5 | Compare two or three meaningfully different high-level strategies | Select, combine, adjust, or reject alternatives |
| 6 | Review and iteratively adjust one detailed, auditable, review-only design | Explicitly approve one artifact revision |
| 7 | Freeze inputs and hashes; create, validate, and dry-run the generic skill | Approve the skill as the standard process |
| 8 | Optional later conversion to field-test form under a separate safety gate | Requires a new explicit request after Checkpoint 7 |

## Approval semantics and dependencies

- Approval is limited to the named checkpoint and artifact revision.
- `adjust`, `reject`, or `go back` is recorded without advancing silently.
- A changed upstream decision reopens every affected downstream checkpoint.
- Source-data approval, planning-input approval, review-design approval,
  field validation, and field authorization remain separate states.
- Checkpoint 8 is outside the present review-planning authorization.

## Decisions requested now

1. Approve or adjust the single-section, review-only scope.
2. Approve or adjust the skill name and activation description.
3. Approve or adjust the reusable-skill versus 62 Collins project-file boundary
   and proposed locations.
4. Approve or adjust the checkpoint sequence and approval semantics.

No target-section decision is requested yet. That visual selection belongs to
Checkpoint 1.

## Status

| Checkpoint | Artifact revision | Status | Blocking decision |
|---|---|---|---|
| 0 — history and scope | `checkpoint_0_scope_rev001.md` | in review | Al must `approve`, `adjust`, `reject`, or `go back` on revision 001 |
| 1 — exact section | — | not started | Checkpoint 0 approval |
| 2 — scoped evidence | — | not started | Checkpoint 1 approval |
| 3 — intent and assumptions | — | not started | Checkpoint 2 approval |
| 4 — entry/exit and decomposition | — | not started | Checkpoint 3 approval |
| 5 — strategy alternatives | — | not started | Checkpoint 4 approval |
| 6 — detailed review plan | — | not started | Checkpoint 5 selection |
| 7 — freeze and create skill | — | not started | Checkpoint 6 approval |
| 8 — later field conversion | — | not started | Separate explicit request after Checkpoint 7 |

Current permitted next action: record Al's explicit decision on Checkpoint 0
revision 001. Do not begin Checkpoint 1 before approval.

## Evidence snapshot

The files below were read for this checkpoint. Hashes are SHA-256.

| Repository-relative source | SHA-256 |
|---|---|
| `AGENTS.md` | `a862e1dfd3582e950e7f8ee3664ea80e82ec175f400d1cb4eeae1d590dac4d5b` |
| `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261020_coverage_planning_handoff/README.md` | `4b3785182451c69835afe3b765704bda6f3c4d558115165f9bc4cf4011adad4b` |
| `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261020_coverage_planning_handoff/DATA_LINEAGE_AND_APPROVAL_REQUIREMENTS.md` | `58c85d13f6dfe9e20f9cab577f74a053c363037b6caa08b7a5973a4234e81bdc` |
| `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261019_staged_coverage/stage_0_source_inventory/README.md` | `d95dbcffb1f030bf270fdb23438b2947630d6b9ab421c7622a3cfe8f6e380668` |
| `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261019_staged_coverage/stage_0_source_inventory/stage_0_reconciliation.csv` | `0673af7da3bc297e5e7694fa78504274b821fc0f150aae8f0182d987aa257d11` |
| `field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/revisions/rev_003_20260926_REVIEW_ONLY/62_Collins_site_inventory_rev003_INTERACTIVE_REVIEW.html` | `2b68c238479dbfec634b2cd85272508fd245cd5f4411c18cdc91ff629d0d258d` |
| `field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/FIELD_TEST_README.md` | `bab4a185bb70d3a57fc2b68cc9b6c4c2053d5ec13a5f98e84ef63e1498452e40` |
| `obsidian_vault/02-testing/20261021-master-manual-phone-field-session-summary.md` | `395b0e49f16091a94d0ad8c1b5dde8ec7db673b81f528ad036e927446e724149` |
| `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261005_zamboni_coverage/generated_review_20261005/README.md` | `db591e03abf7b447f05805324c38d0d85da871eba679f813fefdecc1e6930495` |
| `field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261005_zamboni_coverage/generated_review_20261005/coverage_validation_report.json` | `0012a1e7eccc3a9684fffa9aac090f73bb7550a5df379308c718a5d74b0331cf` |

