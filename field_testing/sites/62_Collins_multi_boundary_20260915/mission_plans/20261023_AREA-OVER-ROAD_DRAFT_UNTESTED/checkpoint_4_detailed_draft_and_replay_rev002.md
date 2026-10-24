# Checkpoint 4 — detailed draft and review replay, revision 002

**DRAFT - UNTESTED IN THE FIELD**

This is non-executable design geometry. It incorporates Al's responses from
replay revision 002 and is presented for another review cycle.

## Adjustments made

1. The first-stripe entry now continues on the perimeter departure tangent for
   2.0 m and then uses a continuous forward-only 1.4 m Dubins right-turn path.
2. The short edge stripe is moved earlier in the stripe sequence.
3. Every between-stripe maneuver is now a continuous forward-only 1.4 m path.
4. The prior approximately 102 m perimeter relocation is eliminated by working
   back across the remaining stripes after the edge sliver.
5. The exit uses the shorter direction on already-cut perimeter ground.
6. No cleanup pass was added; that decision remains deferred until field
   testing or a manually captured cleanup path.

## Current draft metrics

- segments: 21
- total centerline length: 707.18 m
- estimated motion time at 1.0 m/s: 11.79 minutes
- forward-only: yes
- 1.4 m modeled turn segments: 8, all passing the sampled radius check
- centerline outside the exterior polygon: 0.000 m
- physical 42-inch deck overlap with the pole exclusion: 0.000 m²
- conservative 38-inch modeled coverage: 97.75%, leaving 9.92 m²
- modeled 42-inch physical-deck coverage: 98.81%, leaving 5.23 m²
- heading joins over 10 degrees: 6

Compared with revision 001, route length is reduced by approximately 150.17 m,
and the 102-second edge-sliver relocation no longer appears.

## Remaining non-executable findings

- The access-to-perimeter transition still contains placeholder heading joins.
- The full pole loop is intentionally non-simple because it includes a full
  circle followed by the wide-side rejoin arc.
- The exit-to-O-out joins still require smoothing before any executable mission.
- Six joins exceed 10 degrees; they remain explicit validation findings.
- Cleanup gaps are measured but intentionally not resolved in this revision.

## Replay review questions

| Time | Review point |
|---|---|
| 03:09.3 | revised first-stripe entry |
| 04:17.3 | reordered transition to the edge sliver |
| 04:38.1 | smoothed exit from the edge sliver |
| 09:54.4 | location where the former long relocation is now absent |
| 11:13.4 | shorter exit relocation |
| 11:34.7 | confirmation that cleanup remains deferred |

The replay defaults to a true top-down plan view with the long mowing edge
vertical and one identical pixels-per-meter scale on both axes.
Playback now uses a floating-point mission-position accumulator so 1×, 5×, and
10× advance independently of the range control's display precision.

## Decision needed

Review replay revision 004 and respond to its timestamped questions. Then reply
with `approve`, `adjust`, `reject`, or `go back` for this Checkpoint 4 revision.

