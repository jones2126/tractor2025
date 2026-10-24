# AREA-OVER-ROAD Pure Pursuit mission — review revision 001

**Status: DRAFT - UNTESTED IN THE FIELD**

The approved route concept has been converted into a five-column waypoint file that the current Pure Pursuit controller can read. This is a review artifact, not authorization to move the tractor. The dashboard and launcher remain fail-closed until Al separately approves a supervised, blades-off field test.

## Controller file

- File: `generated/62_Collins_AREA_OVER_ROAD_1mps_DRAFT_UNTESTED_20261024.txt`
- Format per line: `latitude longitude yaw_radians lookahead_m speed_mps`
- Waypoints: 3,463
- Route length: 690.216 m
- Estimated motion time at 1.0 m/s: 11 minutes 30 seconds
- Speed: 1.0 m/s throughout
- Lookahead: 2.0 m on straight/access/perimeter portions; 1.5 m on entry, exit, headland, pole-loop, and obstacle-bypass turns
- Direction: forward only
- SHA-256: `150e70a2be00eed9cf0e5aedc0bc26ba3c8a122a2b37a7c666aab7e3a7859979`

The audit CSV beside the mission file maps every waypoint back to its phase, segment, local projected position, and source sequence.

## How the downloaded responses were applied

- The entry contains an explicit continuous left turn rather than depending on lookahead to invent the maneuver.
- The perimeter-to-pole-loop entry and exit are tangent transitions.
- The full pole loop and three stripe obstacle bypasses are represented by waypoint geometry.
- The prior sharp corners inside the stripe obstacle bypasses were replaced with tangent 1.4 m planning-radius curves.
- The final stripe has a continuous exit transition aligned with the shared O-out/F-out access path.
- Lookahead may make the measured tractor track differ from the waypoint polyline, as Al noted, but it is not being used as a substitute for defining the requested turns.

## Numerical review

- All 20 segment-to-segment position gaps: 0.0 m
- Largest segment-join heading change: 4.516 degrees
- Sampled constructed-turn minimum radius: approximately 1.4 m
- Modeled physical deck intersection with pole exclusion: 0.0 m²
- Modeled conservative deck-plus-right-chute intersection with pole exclusion: 0.0 m²
- Planned 38-inch covered area: 428.206 m²
- Planned 38-inch uncovered area: 12.352 m²
- Modeled 42-inch covered area: 433.787 m²
- Modeled 42-inch uncovered area: 6.771 m²

The source perimeter still contains some coarse recorded-boundary corners. Pure Pursuit is expected to round these according to lookahead and steering saturation. The 1.4 m radius is a planning/test assumption, based on the reported approximately 1.1 m demonstrated pole turn; it has not yet been validated with this complete route.

## Replay review timestamps

- `00:12.4` — explicit left entry turn
- `02:26.2` — tangent entry to the pole loop
- `02:34.6` — full forward-only pole loop
- `05:19.6` — first stripe obstacle bypass
- `07:16.2` — remaining stripe obstacle bypasses
- `11:06.2` — explicit exit turn to O-out/F-out

Use `checkpoint_6_pure_pursuit_review_replay_rev001.html` for the true top-down, equal-scale replay with zoom and 1× playback.

## Verification completed

- Every nonblank mission row has exactly five numeric columns.
- The current `pure_pursuit_controller_20261022.py` loader accepted all 3,463 waypoints in a non-motion load test.
- All waypoint speeds are 1.0 m/s and all lookaheads are either 1.5 m or 2.0 m.
- The replay's embedded JavaScript passed a syntax check.
- The dashboard is pointed at this mission and audit file, but its Start action remains disabled.

## Remaining gate

Al should review the new replay and controller route. Enabling a field run requires a separate, explicit approval of a supervised blades-off test. No Teensy firmware or Pure Pursuit controller code was changed.
