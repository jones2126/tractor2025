# Manual recovery design review — 2026-09-15

Status: prototype; not approved for an unattended or mowing run.

## Intended behavior

1. In AUTO with RTK Fixed and valid heading, Pure Pursuit advances monotonically
   along the master path.
2. Loss of RTK Fixed immediately commands zero speed and marks path
   reacquisition as required. RTK recovery alone cannot restart motion; the
   controller requires an observed Manual/Pause state followed by AUTO.
3. Switching the handheld to Manual or Pause freezes mission progress. The
   controller continues to observe GPS and handheld status but does not select
   new targets.
4. The operator drives in Manual using knowledge of the mission.
5. When LED4 is green again and the handheld returns to AUTO, the controller
   searches only the not-yet-completed part of the master path.
6. AUTO resumes only when a path point is within 2.0 m, the path direction is
   within 60 degrees of tractor heading, and the closest choice is unambiguous.
7. The target is interpolated ahead by the configured lookahead distance. Path
   progress cannot move backward.

## Deliberate safe refusal

The recorded master contains closed loops, retraced connectors, and nearby
passes. At some locations, two future branches can be nearly the same distance
from the tractor. The prototype refuses to guess when candidates are within
0.35 m of one another but differ by at least 8 m of mission progress. It keeps
commanding zero speed. The operator remains in Manual and moves or aligns until
only one forward branch is a valid match.

The all-waypoint replay simulated a 20 m Manual interval before each AUTO
request. It acquired 2,548 of 2,642 tested locations and deliberately blocked
94 ambiguous locations. See `generated/forward_recovery_validation_report_20260915.json`.

## Required validation before use

- Replay against recorded GPS/status timing, including RTK Fixed → Float →
  Fixed and Pause → Manual → AUTO transitions.
- Confirm UDP 6003 is delivered independently to the controller, logger, and
  dashboard on the tractor Pi.
- Jack-stand or blades-disengaged test: verify no motion command is acted upon
  in Manual or Pause.
- Very-low-speed open-area test with a simple non-crossing path.
- Low-speed field test at a deliberately chosen unambiguous resume point.
- Review every phase boundary and every ambiguous range before enabling the
  776.9 m master replay.

No field launcher is provided until these checks pass.
