# 62 Collins continuation after the main backyard — 2026-09-18

Status: **review before the next supervised blades-off run.**

This is a separate continuation package. It does not modify or replace the
2026-09-16 waypoint-91 mission used for the 2026-09-18 run.

## Why the cut is at transition 05

The worksheet's `main backyard 1` and `main backyard 2` are recorded boundary
segments 3 and 4. Together they define the main-backyard field used to generate
19 inner rings. The 2026-09-18 Pure Pursuit log reached controller waypoint
12,668, inside generated `main_backyard_ring_19`.

The controller did not traverse the final tail of ring 19 or the generated
main-backyard exit connector in AUTO. They are nevertheless omitted because
the operator declared the main-backyard worksheet area complete and will
manually return to the reviewed waypoint-91 start.

The existing recorded `transition_05` begins only **0.19 m** from that start,
with a starting-heading difference of **8.0 degrees**. No new shortcut or
connector geometry is introduced.

## Continuation contents

- 6,468 waypoints and approximately 1,073 m of reviewed path;
- estimated moving time 17.9 minutes;
- every waypoint commands **1.00 m/s**;
- every waypoint retains the original 2.00 m lookahead;
- all main-backyard phases omitted;
- garden right, garden left, front yard, and the reviewed over-road boundary
  retained unchanged geometrically;
- stripes remain omitted;
- the over-road inner ring remains omitted;
- the 24-inch-expanded telephone-pole exclusion remains preserved.

The all-1.00 m/s change deliberately includes connectors, garden-left features,
innermost rings, and the over-road boundary that previously commanded 0.50
m/s. This is a speed-policy change, not a geometry change. Keep the mower deck
disengaged for the first continuation test.

The archived 2026-09-18 controller log supports the operator's speed report:

- at a 0.50 m/s command, median GPS speed was 0.14 m/s and 41.9% of driving
  samples were at or below 0.10 m/s;
- at a 1.00 m/s command, median GPS speed was 0.96 m/s and only 0.7% of
  driving samples were at or below 0.10 m/s.

These measurements explain the uniform command change but do not guarantee
the same response in every turn or terrain condition. The first run remains a
supervised, blades-off test.

## Generated files

Under `generated_continuation_20260918/`:

- `62_Collins_continuation_after_main_backyard_1mps_20260918.txt`
- `62_Collins_continuation_after_main_backyard_1mps_audit_20260918.csv`
- `62_Collins_continuation_after_main_backyard_1mps_report_20260918.json`
- `62_Collins_continuation_after_main_backyard_1mps_REVIEW_20260918.png`

Rebuild with:

```bash
python3 build_continuation_after_main_backyard_20260918.py
```

Verify without touching services or hardware:

```bash
bash ./run_62_Collins_continuation_all_1mps_20260918.sh --verify-only
```

After review and deployment, start its dedicated voice-guidance dashboard:

```bash
cd /home/al/tractor2025 && python3 tractor_rpi/pure-pursuit/mission_dashboard_continuation_20260918.py
```

The dashboard and launcher retain strict preflight, start-position, heading,
handheld-Pause, phase-locked recovery, and blades-off confirmation gates.
