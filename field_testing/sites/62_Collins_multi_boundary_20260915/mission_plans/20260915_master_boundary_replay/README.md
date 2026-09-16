# 62 Collins master boundary replay — 2026-09-15

Status: **review only; do not run on the tractor yet.**

This package converts the labeled manual drive into one continuous replay in
the original order. Segment 1 is omitted as requested. Segments 2–17 are kept,
including every transition path that Al deliberately drove around obstacles.

The generated master is a boundary/transition replay, not a mowing coverage
mission. It has no launcher on purpose. Before field use, the new recovery
controller must pass recorded-data replay and a stationary/low-speed tractor
test of Pause → Manual → AUTO transitions.

## Telephone pole

Segments 13 and 15 are the two pieces of the over-road outer boundary.
Segment 14 is retained between them in the replay because it is the recorded
route around the telephone pole; directly connecting 13 to 15 would create an
unverified 5.95 m shortcut. A closed 51-point subset of Segment 14 is also
exported as the telephone-pole obstacle geometry. Its measured closure gap is
approximately 0.07 m.

## Generated contents

- `62_Collins_master_boundary_replay_REVIEW_ONLY_20260915.txt` — master replay
- `62_Collins_master_boundary_replay_audit_20260915.csv` — waypoint/phase index
- `62_Collins_boundaries_and_obstacle_REVIEW_20260915.geojson` — boundaries,
  pole obstacle loop, and full Segment 14 survey line
- `individual_paths/` — each labeled segment and the requested polygon groups
- `62_Collins_master_boundary_replay_report_20260915.json` — build metrics and
  explicit limitations
- `62_Collins_master_boundary_replay_REVIEW_20260915.png` — labeled geometry
- `forward_recovery_validation_report_20260915.json` — resume-location replay

Rebuild from the labeled candidate files with:

```bash
python3 build_master_boundary_replay_20260915.py
```

## Rings-only coverage candidate

`build_rings_only_coverage_20260915.py` applies the field decisions recorded
after the boundary review:

- use inner rings only; stripe coverage is deferred;
- command 1.0 m/s on ordinary coverage and 0.5 m/s on all planned connectors,
  all garden-left geometry (review features A-C), and each other field's
  innermost ring (including review features D-E);
- reject any planned connector requiring 300 degrees or more of net turning;
- preserve the deliberately driven inter-field transitions;
- treat the closed subset of Segment 14 as the telephone-pole obstacle and
  expand it outward by 24 inches (0.6096 m), plus a 0.02 m numerical margin;
- use a 1.63 m review-only planning radius, corresponding to the previously
  measured weaker right turn. This has no safety margin and is not yet an
  approved operational limit. Tighter stripe U-turn planning remains deferred
  until the recalibrated steering limits have a repeatable measured radius.

The Segment 14 pole-loop subset contains almost two turns. Its fitted radius is
approximately 1.12 m (1.17 m from path length divided by heading change).
Heading decreased by approximately 684 degrees, and the raw telemetry held a
steering setpoint of 885 for approximately 96% of the interval. The 2026-09-14
firmware defines 885 as the operating hard-left limit. This measurement
therefore supports a tighter left-turn radius; it does not validate a matching
right-turn radius.

The generated candidate contains 19 main-backyard rings, 3 garden-right
rings, 6 garden-left rings, and 5 front-yard rings. One over-road inner ring is
geometrically possible, but a contained entry-and-exit chain could not be
formed while simultaneously honoring the 24-inch pole exclusion and the
1.63 m right-turn constraint. The manually driven over-road boundary remains
in the master route: Segments 13 and 15 are retained, while the Segment 14 pole
portion is shifted outward to the expanded exclusion. The report marks the
inner-ring coverage as incomplete, not the boundary pass as missing.

The first generated version contained two nearly complete right-hand connector
circles between main-backyard rings 1-2 and 9-10. They were not coverage rings.
The connector search now rejects turns of 300 degrees or more and selects
contained forward alternatives. The replacements are 7.60 m with 6.9 degrees
of net turn and 4.18 m with 40.6 degrees of right turn. The A-E review features
remain because they are intentional innermost rings or contained transition
geometry; each is commanded at 0.5 m/s. Feature A is a 10.08 m closed inner
ring, so the field launcher reduces the controller's normal forward tracking
window from 12 m to 6 m. This prevents a nearby point on the far side/end of
that ring from being selected during ordinary tracking.

The files under `generated_rings_only/` retain `REVIEW_ONLY` or
`PARTIAL_REVIEW_ONLY` in their names so the omitted over-road inner ring is not
mistaken for complete coverage. Rebuild them with:

```bash
python3 build_rings_only_coverage_20260915.py
```

## Supervised field-test launcher

After reviewing the route image, Al authorized a supervised field test for
2026-09-16. `run_62_Collins_partial_rings_field_test_20260916.sh` verifies the
exact mission checksum and report limitations, runs the mission preflight,
checks RTK/heading and the starting pose, requires an explicit blades-off
confirmation, starts field logging, and launches the guarded 2026-09-15 Pure
Pursuit controller on its dedicated UDP 6010 navigation feed.

After the first attempt stopped beneath tree cover, the active launcher and
dashboard were moved to the clear-sky resume mission generated by
`build_resume_from_waypoint_0091_20260916.py`. It trims source waypoints 1-90
and begins at source waypoint 91 (19,250 waypoints remain). The selected start
was 0.67 m from the field position recorded before rebuilding. Re-run the
builder only if the reviewed source mission or selected start waypoint changes.

Recovery is now limited to 30 m of forward path and to the current audit phase,
preventing a nearby later ring from being selected. Driving also requires five
continuous seconds of RTK Fixed position and healthy heading: fixed carrier,
0.80-1.30 m baseline, and no more than 1.0 degree estimated heading error.

After an intermittent `headValid=False` stopped the first clear-sky attempt,
the controller, dashboard, and field logger were extended to retain the full
RELPOSNED flag set, baseline, accuracy, frame count/time, and heading-receiver
satellite signal data. The controller still stops immediately on an invalid
heading; its wait reason now states the individual receiver flags needed to
diagnose the event.

The launcher does not convert the partial route into complete coverage. The
over-road boundary is included, but its one geometrically possible inner ring
remains omitted. The mower deck must remain disengaged for this first test.

The real-time web interface in
`tractor_rpi/pure-pursuit/mission_dashboard_20260910.py` is configured for this
mission. Its **START MISSION** action calls the launcher with `--dashboard`,
retaining all checksum, preflight, start-pose, logging, RTK, and handheld-mode
gates while enabling the dashboard Pause/Clear-Pause and telemetry ports.

Validate files without touching hardware services:

```bash
bash ./run_62_Collins_partial_rings_field_test_20260916.sh --verify-only
```

Run the supervised field test:

```bash
bash ./run_62_Collins_partial_rings_field_test_20260916.sh
```
