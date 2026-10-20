# 2026-10-19 September master-route rerun prompt

## Ready-to-paste prompt

```text
Prepare—but do not prematurely start—the exact historical September master
route for another directly supervised, blades-off Tractor01 field run.

Read these files completely:

- obsidian_vault/02-testing/20260923-master-manual-field-session-summary.md
- obsidian_vault/02-testing/20261006-master-manual-phone-rerun-network-failure.md
- obsidian_vault/02-testing/20261019-tractor01-testing-regrouping.md
- field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/FIELD_TEST_README.md
- field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/verify_master_manual_phone_20261006.py
- field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/collect_master_manual_phone_run_20261006.sh

Inspect `git status` and preserve all unrelated dirty work.

The immutable route identity is:

- `62_Collins_master_manual_resampled_1mps_20260920.txt`
- 19,825 waypoints
- 1.00 m/s
- 2.00 m lookahead
- SHA-256 `0276fa22f2c7def0a516b2dcd5516bfd3b05fa647f3b50607a6ca5ec218436b4`

This is the route shown in the completed September 23 historical replay. The
October 6 rerun stopped around 78%. Do not regenerate, smooth, reorder, splice,
or replace the route. Keep this work separate from the new coverage map.

Before declaring the package field-ready:

1. Read the outputs of the separate ZeroTier-primary/page-hide recovery work
   and Manual Recovery to Parked Position work. Do not carry the old
   `192.168.10.x only` assumptions forward if the new architecture has changed,
   and do not invent changes if those tasks are incomplete.
2. Verify the mission and audit hashes and run existing automated tests.
3. Ensure automatic dual-F9P 5 Hz heading configuration, RTK Fixed, fixed
   carrier, baseline, heading-accuracy, start-distance, and start-heading gates
   remain fail-closed.
4. Confirm regulated network power, loaded low-voltage alarm, physical E-stop,
   phone Pause, and exactly one authoritative control session.
5. Require a completed stationary page-hide/recovery test and a completed
   Manual Recovery exercise before using the long route.
6. Add a pre-run network soak appropriate to the selected primary path and
   explicit go/no-go thresholds for heartbeat gap, RTT/jitter, packet loss,
   route/interface changes, and dashboard telemetry age.
7. Confirm the dashboard cannot display stale green Auto and that COPY ALL
   MESSAGES includes timestamps and path/session information.
8. Confirm the collector captures the phone flight recorder, every control
   service instance, network/path transitions, NetworkManager journals, mission
   and pursuit logs, and hashes. Preserve originals even if parser-safe CSV
   copies trim terminal NUL padding.
9. Prepare a step-by-step field checklist and shutdown/recovery card. Do not
   combine intentional link-failure testing with this full mission.
10. If I later say I am physically at the tractor and ready to run, guide me
    one consequential step at a time and wait for my actual output at every
    gate. Never infer PASS from old results.

First deliver a readiness matrix listing satisfied, failed, and pending gates.
If the network-control or recovery tasks are incomplete, stop at preparation
and clearly identify the blockers rather than authorizing motion.
```
