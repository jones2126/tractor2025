# AREA-OVER-ROAD field-package review — 2026-10-24

> **DRAFT - UNTESTED IN THE FIELD**

## Current decision state

Al approved the route concept and ordering on 2026-10-24. This approval is not
field authorization. Controller route revision 003 replaces the previously
reported sharp joins with continuous 1.4 m planning geometry. Al approved all
six controller-route review maneuvers on 2026-10-24. This package remains
deliberately review-only until one supervised blades-off run is explicitly
authorized and an enabled launcher is built and verified.

The five-column mission candidate can be loaded by the current Pure Pursuit
controller. Every audit row is marked `CONTROLLER_FORMAT_DRAFT_UNTESTED`. The
dashboard Start action and launcher still fail closed until separate field-test
authorization is recorded.

## Review the route

Open `checkpoint_6_pure_pursuit_review_replay_rev001.html`. It is a true
top-down, equal-scale view and now supports:

- Zoom in / Zoom out;
- mouse-wheel zoom;
- drag to pan;
- Focus tractor; and
- Reset view.

The replay runs at 1x, 5x, 10x, and 20x and shows the exact controller-route
geometry used to generate the mission file.

## Start the web dashboard from the Windows development computer

Run this PowerShell script from the repository:

```powershell
& .\field_testing\sites\62_Collins_multi_boundary_20260915\mission_plans\20261023_AREA-OVER-ROAD_DRAFT_UNTESTED\start_area_over_road_dashboard_20261024.ps1
```

It connects to Tractor01 over ZeroTier and runs the Linux dashboard starter.
The terminal prints the dashboard URL. Keep that terminal open and use
`Ctrl+C` to close the dashboard.

The equivalent Tractor01 command is:

```bash
cd /home/al/tractor2025
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED/start_area_over_road_dashboard_20261024.sh
```

This starts only the web dashboard. It cannot start tractor motion.

## Printable field sequence

The two-page printable sequence is:

`output/pdf/AREA-OVER-ROAD_blades-off_field_test_checklist_rev001.pdf`

It includes a hold point because the current dashboard Start action is still
disabled. Do not take the checklist to the field as authorization to move the
tractor; the field gate and launcher must first be separately approved and
verified.

## Package verification

On either machine after pulling the same commit:

```bash
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED/verify_area_over_road_review_package_20261024.py
```

Expected result: the controller file, continuity, turning geometry, and pole
clearance checks pass; live start remains disabled pending owner review.

## Evidence collection after a future supervised run

The package is not yet authorized for that run. When a later controller-ready
revision is approved and run, the collector can be invoked on Tractor01:

```bash
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED/collect_area_over_road_postrun_20261024.sh
```

It copies relevant logs, service journals, controller data, phone-control
events, recovery records, repository state, network state, package metadata,
and checksums into a timestamped `/home/al/tractor01_area_over_road_*.tgz`
archive. It does not delete source logs.

For the end-of-session checklist, use:

```bash
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED/finish_area_over_road_session_20261024.sh
```

That script refuses to proceed while a pure-pursuit controller is running,
collects the archive, and reminds the operator to:

1. copy and hash-check the archive off Tractor01; and
2. download the system log from the Archer AXE75 at the RTK base station that
   provides `TractorField`, recording the router clock/time zone and preserving
   the original file unchanged.

It intentionally does not issue a Linux shutdown command.

## Next required review

Review the controller mission at the timestamps listed in
`checkpoint_6_pure_pursuit_mission_rev001.md`. A supervised blades-off field
test still requires a separate explicit approval before changing
`APPROVED_FOR_FIELD=false`.
