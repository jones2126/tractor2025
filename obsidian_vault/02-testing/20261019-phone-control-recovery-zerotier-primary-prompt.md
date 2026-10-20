# 2026-10-19 Phone-control recovery and ZeroTier-primary prompt

## Ready-to-paste prompt

```text
I am preparing Tractor01 for renewed testing after the October 6 phone-control
incident. Work in the tractor2025 repository. Begin by reading these files
completely:

- obsidian_vault/02-testing/20261006-master-manual-phone-rerun-network-failure.md
- obsidian_vault/02-testing/20261006-next-field-test-network-reliability-prompt.md
- obsidian_vault/02-testing/20261003-wifi-primary-control.md
- obsidian_vault/02-testing/20261019-tractor01-testing-regrouping.md
- field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/FIELD_TEST_README.md

Inspect `git status` before editing and preserve all unrelated dirty boundary
and coverage-map work.

Current test-environment limitation: Al is at home and does not have access to
Tractor01, the RTK base station, either field router, the Teensy/JRK hardware,
or other field components. The real mobile phone is available. Prepare code,
automated tests, documentation, and real-phone tests against a non-tractor test
endpoint, but distinguish those results from end-to-end Tractor01 validation.
Do not describe mocked telemetry or home-network measurements as field
qualification. Keep unqualified motion paths disabled and list the deployment,
engine-off, wheels-raised, and moving checks that remain deferred.

My desired future control hierarchy is different from the October 6
recommendation:

1. ZeroTier from the phone should be evaluated as the desired primary path.
2. The phone's hotspot, with Tractor01 joining it, should be evaluated as the
   first fallback.
3. The onboard travel router should remain a local Manual-only recovery path.
4. A future hardwired device that can safely request steering and transmission
   commands may be evaluated as a last resort, but do not implement an
   unreviewed command-injection bypass.

Do not simply remove the current source-subnet restriction or declare ZeroTier
safe because it is reachable. Motion authority requires bounded latency,
heartbeat freshness, deterministic fail-closed behavior, guarded re-arming,
and a threat model. Lead with an evidence-backed assessment of whether
ZeroTier over cellular can safely satisfy those requirements. If it cannot,
say so plainly and propose the safest architecture that still advances my
goal.

Important October 6 evidence:

- Page hide at 10:11:09 intentionally caused Pause.
- Guarded Auto reached Tractor01 at 10:13:34.
- After the five-second freshness gate, drive commands resumed at 10:13:40.
- The JRK target moved from neutral 2836 to forward near 2300 and feedback
  reached about 2296. Do not assume the JRK lacked a forward command. Determine
  what additional evidence is needed to explain the absence of visible motion.
- At 10:14:01 the heartbeat expired and the JRK returned to neutral.
- Later TractorField failures included high latency, packet loss, and DHCP
  churn, while the wired RTK-base Pi remained healthy.
- The dashboard could retain a stale green Auto display after telemetry stopped.

Please do the following:

1. Trace the current phone client, HTTPS control server, source-address gate,
   session handling, ntfy links, ZeroTier interface, bridge, mission controller,
   dashboard telemetry, watchdogs, and systemd services. Document the actual
   end-to-end paths.
2. Define measurable safety requirements for each candidate path: maximum
   heartbeat gap, RTT/jitter, packet-loss window, telemetry age, route changes,
   IP/interface changes, and recovery-lockout behavior. Keep the independent
   500 ms Teensy watchdog and server-side fail-closed Pause.
3. Design and implement a configurable but explicit network-authority policy.
   It must never accept motion simultaneously from multiple networks or stale
   sessions. A path change must invalidate the session, force Pause, require a
   stability soak, and require a new guarded operator selection.
4. Add a hard stale-telemetry dashboard state. A disconnected dashboard must
   not continue displaying green Auto. Show telemetry age, last-good time,
   authoritative phone mode, selected network path, and recovery lockout.
5. Add a bounded phone-side flight recorder for visibility/page-hide events,
   wake-lock state, send attempts and failures, RTT, sequence/session changes,
   online/offline events, and path changes. Preserve it across reconnect and
   include it in the archive collector.
6. Add phone Wake Lock API support if available, while preserving immediate
   Pause when the page becomes hidden or the device locks.
7. Build an automated stationary test for the exact page-hidden sequence:
   start in guarded authority, hide/lock the page, verify Pause within the
   required bound, verify session invalidation and neutral JRK, restore the
   page, verify it remains in Pause, wait through the stability gate, then
   require a deliberate guarded re-arm. No automatic resumption is permitted.
8. Prepare a physical test ladder: software-only tests; engine-off stationary;
   engine-off/wheels-raised actuator test; separately approved very-low-speed
   test on a short clear route. Do not use the 19,825-waypoint mission as the
   initial fault-injection test.
9. For ZeroTier-primary testing, separately test Wi-Fi-backed ZeroTier and
   cellular-backed ZeroTier. Detect handoff between them and force Pause. Do
   not use NAS reachability as the motion-link health test.
10. For phone-hotspot fallback, document exact addressing and prove that the
    phone browser can reach Tractor01 while the phone is the hotspot. A fallback
    path must begin in Pause and must not depend on hostname discovery.
11. Review the 10:13:40–10:14:01 logs and identify how to conclusively correlate
    mission speed command, JRK target/feedback, engine state, wheel motion, GPS
    speed, and route progress in future tests.
12. Produce code, focused automated tests, an architecture decision record,
    and a step-by-step bench procedure. Clearly list physical checks still
    required before any moving field test.

Do not weaken a safety interlock to make reconnection easier. Do not authorize
field motion merely because unit tests pass.
```
