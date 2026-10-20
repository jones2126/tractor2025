# 2026-10-19 Manual Recovery to Parked Position prompt

## Ready-to-paste prompt

```text
Create a production-quality, guarded "Manual Recovery to Parked Position"
process for Tractor01. This is an emergency/recovery workflow that abandons an
Auto mission and lets me drive the tractor slowly back to parking through the
onboard travel router.

Read these files completely first:

- obsidian_vault/02-testing/20261006-master-manual-phone-rerun-network-failure.md
- obsidian_vault/02-testing/20261003-wifi-primary-control.md
- obsidian_vault/02-testing/20261019-tractor01-testing-regrouping.md
- field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/FIELD_TEST_README.md

Inspect `git status` before editing and preserve unrelated dirty work.

October 6 proof point: a temporary service accepting the onboard-router LAN
`192.168.8.0/24` at `https://192.168.8.20:8765` delivered 1,875 commands, zero
freshness expirations, 27.2 ms mean RTT, 59.4 ms p95 RTT, and a 425 ms maximum
command gap. It successfully returned the tractor to parking. Turn that result
into a preinstalled, audited process rather than another improvised command.

Requirements:

1. Recovery must start and remain in Pause until a deliberate guarded Manual
   selection. It must never offer or accept Auto.
2. Preserve the physical E-stop, software E-stop latch, transmission neutral,
   server freshness timeout, independent 500 ms Teensy watchdog, session
   invalidation, page-hidden Pause, and no automatic re-arm.
3. Bind/authorize only the intended onboard-router interface and subnet. Show
   the numeric URL, expected SSID, client IP/subnet, and selected interface.
4. Prevent simultaneous normal mission-control and recovery services. Use a
   clear mutual-exclusion/ownership mechanism, fail closed on ambiguous state,
   and provide an idempotent start/stop/status workflow.
5. Make the normal mission controller stop or remain held without advancing
   route progress while recovery Manual is active. Preserve its logs and
   checkpoint; do not pretend recovery driving is mission progress.
6. Provide a prominent RECOVERY MANUAL ONLY interface with guarded Pause,
   steering, low-speed forward/reverse limits, link age, RTT, command gap,
   session state, and software E-stop. Use conservative recovery speed limits
   justified from existing firmware/calibration evidence.
7. Log every command, mode transition, source address, session change,
   heartbeat expiry, actuator target/feedback, and service transition. Add the
   recovery log to the standard evidence collector.
8. Publish an ntfy message that clearly says RECOVERY MANUAL ONLY and includes
   the local numeric URL. Do not publish an Auto-capable link.
9. Produce focused tests for wrong subnet, multiple clients, stale session,
   service conflict, page hide, link loss, E-stop, restart, and attempted Auto.
10. Produce a wallet-sized recovery card and exact operator sequence: secure
    tractor and blades, select Pause, stop/hold mission, connect phone to the
    onboard router, start recovery, inspect route, guarded Manual drive, park,
    Pause, shut down, collect evidence.
11. Provide a physical validation ladder: stationary engine-off, wheels-raised,
    then separately approved low-speed recovery exercise. Do not treat software
    tests as field approval.

Also assess—but do not yet implement without explicit approval—a hardwired
last-resort controller. It must not bypass the Teensy watchdog, physical
E-stop, transmission neutral, or control ownership. Deliver a short hazard and
interface analysis for that future Plan C.
```
