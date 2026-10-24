# Checkpoint 3 — selected strategy — revision 002

> **DRAFT - UNTESTED IN THE FIELD**

Date: 2026-10-23  
Scope: `AREA-OVER-ROAD` / `Over the road` only

Revision 002 records Al's selection and clarification. Revision 001 remains
preserved as the three-strategy comparison.

## Approved strategy

Use **Alternative A — adjacent-lane snake**, with the end-turn behavior Al
described:

- shorten each interior stripe before the end of the area;
- continue forward into the already-cut headland created by the clockwise
  outer pass;
- use a forward-only multi-arc/omega-style turn to line up with the adjacent
  stripe;
- allow the turn to cross or recut grass already cut by the outer pass; and
- do not model an adjacent-lane turn as a simple semicircle.

This is the preferred strategy because it is the shortest of the three
high-level alternatives and most closely matches normal mowing. Checkpoint 4
must construct and measure each north and south turn rather than assuming one
generic turn fits both ends.

The planning radius remains **1.4 m**. The observed approximately 1.1 m
hard-left circle around the pole is useful evidence, but it does not yet prove
the same radius in both steering directions or in every headland condition.

## Plain names for the pole maneuvers

The umbrella term `progressive obstacle bypass` is retired from the active
design vocabulary because it did not help Al understand the route. Use these
plain descriptions instead:

1. **leave-edge pole loop** — on the clockwise outer cut, leave the grass
   edge, travel around the pole, and return to the edge;
2. **minimum-offset stripe detour** — on a later stripe, deviate only enough
   to clear the pole, then return to the same straight stripe.

These names do not change the approved behavior from Checkpoint 2.

## What the 0.044 m number means

The conceptual 1.4 m-radius tractor **centerline** around the pole remains
inside the mapped outer boundary. At its closest point, that centerline is
only about **0.044 m (1.7 in)** inside the boundary.

That number is not the amount by which the tractor center leaves the area.
The important warning is that the 42-inch deck extends about 0.533 m from the
centerline, and the right chute can extend about another 0.127 m depending on
heading. Therefore some of the physical mower envelope can extend beyond the
mapped grass edge while its centerline remains inside. Al accepts limited
overhang beyond the original grass boundary for this maneuver. Checkpoint 4
must display the swept cut and physical-envelope overhang; the blades-off
field test must verify the real terrain and nearby hazards before mowing.

## Coverage-gap decision

Checkpoint 4 will produce a detailed coverage map. It will measure and show
any residual gaps rather than pre-classifying them as manual-only or silently
forcing them closed. Al will decide how to handle them after inspecting that
map.

## Checkpoint status

| Checkpoint | Artifact revision | Status | Blocking decision |
|---|---|---|---|
| Completed prerequisites — boundary, obstacle decisions, inventory promotion | focused asset revision `001` | approved | none |
| 1 — access evidence and mission-purpose assumptions | `checkpoint_1_access_and_operating_assumptions_rev001.md` | approved | none |
| 2 — necessary decomposition | `checkpoint_2_necessary_decomposition_rev002.md` | approved | none |
| 3 — strategy alternatives and selection | `checkpoint_3_strategy_selection_rev002.md` | approved | none |
| 4 — detailed draft and replay | — | not started | generate selected Alternative A only |
| 5 — freeze design and create reusable skill | — | not started | Checkpoint 4 approval |
| 6 — supervised field-test package | — | not started | Checkpoint 5 approval |

