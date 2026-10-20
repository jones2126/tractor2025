# 2026-10-19 Staged coverage-planning prompt

## Ready-to-paste prompt

```text
I want to replace the current 62 Collins coverage-map black box with a staged,
owner-reviewable planning process. Work in the tractor2025 repository, but do
not alter or delete the existing uncommitted coverage work.

Start by reading:

- obsidian_vault/02-testing/20261019-tractor01-testing-regrouping.md
- field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261005_zamboni_coverage/NEW_CHAT_CONTEXT.md
- field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261005_zamboni_coverage/generated_review_20261005/README.md
- field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261005_zamboni_coverage/generated_review_20261005/coverage_validation_report.json
- field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/README.md
- tractor_rpi/pure-pursuit/mission_planning/site_coverage_planner_20260724.py
- tractor_rpi/pure-pursuit/mission_planning/site_planner_common_20260724.py
- field_testing/tools/build_skip_row_coverage_mission_20260831.py

Inspect `git status` first. The existing October 5 generator and
`generated_review_20261005` directory are uncommitted user work; preserve them
as a baseline. Put new work in a new dated directory. No `.ipynb` file is
currently tracked in this repository, so search git history and older archives
for the notebook Al remembers, but do not assume it still exists.

The October 5 full-route review is not satisfactory. It produced an
approximately 8,095 m / 135 minute route, 43 unresolved or unsmoothed turns,
one uncontained connection, fixed north-south stripes, and only one large final
review. Treat it as diagnostic input, not an approved design.

The new process must stop for my explicit review after each stage:

Stage 0 — source and inventory reconciliation
- Identify and hash every boundary, obstacle, transition, deck, turn-radius,
  and field-evidence input.
- Preserve historical provenance and retire false obstacle O012 /
  OBS-NEW-012 without renumbering other obstacles.
- Produce a human-readable inventory map and reconciliation table.
- STOP and ask for my approval.

Stage 1 — section/cell proposal
- Propose which physical areas and decomposed cells should constitute mowing
  sections, with rationale based on obstacles, terrain, access, headland room,
  and turn feasibility.
- Make section boundaries editable/reviewable rather than implicit geometry.
- Show area and estimated cutting length per section.
- STOP and ask for my approval or edits.

Stage 2 — entry, exit, and travel paths
- Propose named entry/exit points and directional travel paths for every
  section. Show approach heading, clearance, and whether blades are cutting.
- Prevent long diagonal relocations across completed stripes.
- STOP and ask for my approval or edits.

Stage 3 — stripe-angle comparison
- For each approved section, compare several stripe angles rather than forcing
  north-south. Show stripe count, short/sliver rows, cutting length, non-cutting
  travel, turn demand, slope/terrain considerations when known, and coverage.
- Support a documented rotation of stripe angles between mowing sessions to
  satisfy the landowner's desire for changing patterns.
- STOP and let me choose the angle per section.

Stage 4 — stripe ordering and Zamboni alternatives
- Generate and compare adjacent, skip-one, larger-cycle, and locally adaptive
  ordering where appropriate. Prove that every intended row is eventually
  covered.
- Report cutting distance, deadhead distance and ratio, number of turns,
  longest relocation, cross-finished-lawn relocations, and estimated runtime.
- Visualize sequence numbers and animate/replay each candidate.
- STOP and let me choose the ordering per section.

Stage 5 — turn synthesis
- Build curvature-continuous, forward-only turns using field-supported radii.
  Do not silently substitute straight chords or tight keyholes.
- For each turn report mode, minimum radius, containment, deck-edge clearance,
  curvature discontinuity, length, and excess travel.
- Any unresolved, uncontained, or unsmoothed connector is a hard blocker.
- STOP for visual and quantitative review.

Stage 6 — combined coverage validation
- Combine only approved stage outputs. Compute deck-swept union, uncovered and
  over-covered areas, per-section and whole-site coverage, obstacle and boundary
  clearances, duplicates, reversals, self-intersections, waypoint gaps, route
  length, and runtime.
- Produce static and interactive artifacts with layer controls and lineage back
  to every approved decision.
- STOP for final review. Do not create a launcher or executable mission.

The interface may be a notebook, a small staged CLI plus HTML review pages, or
another maintainable approach. Favor explicit intermediate files that I can
inspect and edit. Reuse the useful `compare-angles`, `preview`, and reviewed-CSV
ideas from the July planner. The process must be reproducible from a manifest,
but it must not regenerate downstream stages automatically after an upstream
decision changes without showing me the consequences.

For this chat, implement the framework and complete only Stage 0. Present its
artifacts for my review and wait. Label everything `REVIEW ONLY — NOT APPROVED
FOR FIELD USE`.
```
