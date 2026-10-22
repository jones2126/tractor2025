# 2026-10-21 Master Manual Phone Field Session Summary

## Outcome

Tractor01 completed the immutable September 23 master mission with blades off,
but only after a phone-freshness failure stopped Auto near 77.5% and a later
Manual reposition allowed guarded forward reacquisition. The route completion
is useful engineering evidence; it is **not a passing field-day qualification
run**. The controlling procedure prohibited Mission 2 after any unexplained
phone heartbeat/freshness expiry, so the consolidated-perimeter mission was not
run.

The tractor was ultimately parked, the gasoline engine was stopped, phone
control was returned to Pause, the dashboard was closed cleanly, and Tractor01
was fully powered down. The operator estimated that the mission consumed about
2.5 US quarts (0.625 US gal / 2.37 L) of gasoline.

This record combines the field chat, the verified evidence archive, the
generated replay, Al's personal notes, and two Archer AXE75 system-log exports.

## Evidence and derived artifacts

- Verified raw archive:
  `field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261021_133807_master_manual_phone/tractor01_master_manual_endurance_20261021_151604.tgz`
- Archive SHA-256:
  `a316fcd031e60afdba5cddd6d9baf29377956043f5b2a772e74bd04344387574`
- Historical replay:
  `field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261021_master_manual_phone_replay/replay_20261021_133807_master_manual_phone.html`
- Replay notes:
  `field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261021_master_manual_phone_replay/README.md`
- Supplemental evidence copied into the ignored run-data directory:
  `field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261021_133807_master_manual_phone/supplemental_evidence/`
- Original personal notes:
  `C:\Users\al532\Downloads\20261021\notes.txt`
- Beginning-of-day AXE75 export:
  `C:\Users\al532\Downloads\20261021\Archer-AXE75-Log-2026-10-21-105200.log`
- End-of-day AXE75 export:
  `C:\Users\al532\Downloads\20261021\Archer-AXE75-Log-2026-10-21-153200.log`

The raw run directory is ignored by Git by design. The compact replay,
replay-builder changes, and durable Markdown notes are suitable for version
control.

## Software preparation and defects corrected in the field

### systemd ownership-lock permission failure

Tractor01 initially reported commit `01cfeea` and a clean tracked branch, with
two unrelated untracked PlatformIO configuration files preserved. The normal
phone-control service then entered an automatic restart loop. Its process ran
as user `al` but attempted to use a lock directly under `/run`, where it could
not create the file.

The persistent repair, delivered in commit `0b20f23`, added a systemd runtime
directory shared by the mutually exclusive normal and recovery services:

- `RuntimeDirectory=tractor-phone-control`
- `RuntimeDirectoryMode=0700`
- ownership lock `/run/tractor-phone-control/ownership.lock`

After Tractor01 pulled the change and both services were reinstalled, the
normal service remained active and Manual Recovery remained static/inactive
until explicitly requested.

### phone-soak analyzer field-name error

The first stationary soak analysis falsely reported zero armed samples even
though fresh JSONL records showed Manual, zero drive, zero steering, 5 Hz
heartbeats, and `zerotier-primary`. The logger wrote the mode as `mode`, while
the analyzer only read `phone_mode`.

Commit `24feab4` added the `mode` fallback and a regression test. After
Tractor01 pulled it, the same sample correctly reported:

- 10.6 minutes continuously armed;
- RTT p95 68.7 ms and maximum 151.9 ms;
- maximum armed command gap 328.0 ms;
- zero heartbeat expiries and zero rejected commands; and
- a provisional timing PASS.

The useful debug pattern was to inspect the raw command events before changing
the runtime system:

```bash
grep '"event":"command".*"mode":"manual"' \
  /home/al/field_logs/wifi_primary_control_20261021_113348.jsonl | tail -n 3
```

## Stationary qualification and recovery work

The combined verifier passed 40 tests and confirmed the exact mission
packages, control software, stale-dashboard behavior, network-readiness logic,
and Manual Recovery behavior. It explicitly did not grant field authorization.

Confirmed stationary results included:

- normal control URL `https://192.168.193.76:8765`;
- authority policy `zerotier-primary`;
- current Teensy firmware `20261003_wifi_v3`;
- phone session ACTIVE, initially Pause, with zero drive and steering;
- normal service bound to the shared runtime ownership lock;
- Tractor01 `eth0` at `192.168.8.20/24`;
- onboard GL-SFT1200 SSIDs `GL-SFT1200-9f4` and
  `GL-SFT1200-9f4-5G` (the older `TractorField` recovery instruction was stale);
- hiding/locking the phone caused Pause, reopening began a fresh Pause session,
  and there was no automatic re-arm;
- Manual Recovery started only on deliberate command, showed
  `Tractor01 RECOVERY MANUAL ONLY`, removed Auto, and displayed the configured
  reverse/forward limits;
- recovery traffic used phone `192.168.8.190` to Tractor01
  `192.168.8.20`; and
- stopping Manual Recovery did not automatically restart normal control.

Engine-off Manual Recovery moved steering and the transmission actuator.
Whether its forward limit supplies enough real ground motion to return to
parking remains a future engine-running recovery check.

Battery/solar samples during bright sun were 14.2-14.7 V. Al declined to stop
and build a new real-time voltage monitor. The documented loaded low-voltage
alarm requirement was therefore not demonstrated and must be recorded as an
operator deviation, not a pass. A fresh physical tractor E-stop result was
also not captured in the chat; the later guarded phone E-stop is a different
mechanism and must not be substituted for it.

## Mission chronology

Times below are EDT on October 21, 2026 unless stated otherwise.

| Time | Event |
|---|---|
| 11:14-11:35 | Updated and repaired the phone-control services; normal control reached `CONNECTED - PAUSE`. |
| 11:42 | Two-mission verifier passed all 40 tests. |
| 12:08 | Confirmed the two actual onboard GL router SSIDs. |
| About 12:22-12:33 | Stationary Manual-zero soak recorded healthy 5 Hz command traffic. The analyzer bug was diagnosed and corrected. |
| 12:42 onward | Manual Recovery acceptance was exercised through the onboard GL router. Normal ZeroTier-primary control was then deliberately restored. |
| 13:18 | Dashboard/voice-note session was opened; automatic Heading-F9P 5 Hz RAM configuration passed. |
| 13:38:07-13:38:10 | Combined field logger and pure-pursuit controller started. |
| 13:39:29 | Guarded Auto was selected for the mission. |
| **14:18:14.028** | The phone heartbeat exceeded the 500 ms freshness limit. The service invalidated the session with `phone_freshness_expired`, forced Pause/neutral, and claimed a new session about 388 ms later. Controller elapsed time was approximately **2404.2 s**. Progress froze near waypoint **15,362 / 19,825 (77.5%)** in `garden_left_ring_2`. |
| 14:18-14:20 | Al deliberately attempted guarded Auto recovery, selected Pause, tried Auto again, then exercised the guarded phone E-stop and reset. These later mode transitions were operator actions, not the initiating failure. |
| After the stop | The tractor rolled backward approximately one yard on a small hill. With progress frozen, its changed pose/heading no longer met guarded forward-reacquisition geometry. |
| 14:42:21 | A new phone session entered Manual for repositioning. |
| 14:42:53 | Guarded Auto was selected after repositioning. The controller later reported successful forward reacquisition. |
| 14:46-15:01 | Additional phone freshness expiries and guarded re-arms occurred. These require event-by-event browser/server/network correlation rather than treatment as one outage. |
| About 15:02:19 | The controller printed `Goal reached -- sending stop`, closed the pursuit and combined field logs cleanly, and the launcher exited with status 0. |
| After completion | The dashboard correctly showed COMPLETED, but its controller telemetry age increased because the mission controller had exited. The phone-control link age remained live and separate. Phone Pause was selected, the tractor was parked, and the gasoline engine was stopped. |
| 15:16 | A complete evidence archive was created on Tractor01. It was copied to the development machine and its SHA-256 matched. |

## What the first stop established

The immediate cause of the 14:18:14 stop was a control-stack freshness expiry,
not a route completion, RTK loss, heading invalidity, physical E-stop, or
operator Pause. The relevant normal-control log sequence was:

1. Auto remained continuously armed from 13:39:29.
2. At 14:18:14.028 the server emitted `heartbeat_expired` with a 500 ms limit.
3. The same instant invalidated the active session with reason
   `phone_freshness_expired`.
4. A new session was claimed at 14:18:14.416 and began in Pause.
5. Subsequent Auto/Pause/E-stop events were deliberate recovery actions.

Setting the phone screen timeout to 30 minutes and moving the steering slider
once per minute did not prevent this. The browser already sends periodic
heartbeats while Auto is active, and the Manual steering value is not the Auto
steering command. The important question is why a continuously expected
browser-to-server request/response interval exceeded the 500 ms safety limit.

The fail-closed behavior worked: session authority was removed, transmission
returned to neutral, and Auto did not re-arm by itself.

## Reacquisition learning

While phone control was in Pause, the dashboard's green `READY FOR GUARDED
AUTO` banner only proved that telemetry and safety/control freshness gates were
ready. It did **not** evaluate forward route-reacquisition geometry.

The controller's offline calculation using the current pose and frozen mission
progress found:

- route distance 1.339 m, within the 2.0 m limit;
- tractor heading 293.157 degrees;
- eligible route heading 358.482 degrees;
- heading error 65.325 degrees, exceeding the 60-degree limit; and
- the relevant route point within the same `garden_left_ring_2` phase and the
  5 m forward-progress window.

The rollback therefore left the tractor close enough to the route but pointed
about 5.3 degrees outside the permitted heading error. A short controlled
forward-right Manual reposition brought it into the guarded recovery envelope.
This demonstrated why a stopped vehicle on a slope must be physically secured:
fail-closed neutral prevents powered motion but cannot prevent gravity-driven
rollback.

## End-of-run analyzer result

The archived normal-control log produced this 40-minute-gate result:

- longest continuous armed duration: 38.7 minutes;
- command intervals in that segment: 19,740;
- RTT p95 85.0 ms;
- RTT maximum 520.6 ms;
- maximum armed command gap 423.7 ms;
- 23 heartbeat/freshness expiries across the collected service session; and
- zero rejected commands.

The result is a qualification FAIL. The count of 23 spans the original mission,
recovery attempts, later mission operation, and post-mission maneuvering; it
must not be described as 23 identical mission stops without classifying each
event against the controller, browser flight recorder, and operator timeline.

## AXE75 system-log observations

The beginning export was captured at 10:52 with five connected clients (four
Wi-Fi). The end export was captured at 15:32 with four connected clients (three
Wi-Fi). Both identify Archer AXE75 hardware 1.0, firmware
`1.5.6 Build 20260623 rel.56854`, LAN `192.168.10.1`, and WAN
`100.81.44.171`.

The exports are dominated by the router's own WAN DHCP client. They show
successful 300-second-lease acknowledgements roughly every 154 seconds. Normal
ACK sequences at 14:16:49 and 14:19:23 bracket the 14:18:14 tractor Pause.
There is no logged WAN renewal failure at that time.

However, these exports contain no useful per-client association,
deauthentication, roaming, retry, signal, packet-loss, queueing, or latency
events. They therefore argue against an obvious router-wide WAN lease failure,
but they cannot determine whether the timeout originated in the Pixel browser,
Android scheduling, Wi-Fi radio path, ZeroTier overlay, AXE75 forwarding, or
Tractor01 server path. Absence of a client event in this limited system log is
not evidence that the client path was healthy.

Input hashes:

- personal notes SHA-256:
  `26ab468b395abe921e2c02194af2f1e478e891e9d10cb1a488725e219480fc66`
- 10:52 AXE75 log SHA-256:
  `088478fb28d96d42d184c61eaf02d4b9408044498bf5b81d16ffb7d74fae6c16`
- 15:32 AXE75 log SHA-256:
  `edd6ce33cad3ef2a179ae6ce182b748ce18ae6d305df234d04a369318baadc75`

## Key tools and debug patterns

- `systemctl status/show` and `journalctl` separated unit-installation,
  permissions, and runtime failures.
- Raw JSONL inspection prevented an analyzer defect from being mistaken for a
  logging or network defect.
- `verify_two_mission_field_day_20261020.py` checked immutable missions and the
  prepared control/recovery software before motion.
- `analyze_phone_control_log_20261020.py` provided reproducible timing gates;
  its results remain provisional and do not override physical gates.
- The dashboard's read-only `/api/state` endpoint exposed live pose, heading,
  mission progress, control freshness, and actuator state without changing
  authority.
- The actual `PurePursuit.reacquire_forward` calculation was reused offline to
  decide whether the stopped pose met the same 2 m, 60-degree, 5 m, and
  same-phase constraints as the running controller.
- `collect_field_day_run_20261020.sh` produced a non-destructive archive with
  internal hashes and diagnostics.
- The archive and copied hash record were verified before Tractor01 was powered
  down.
- `build_tractor01_mission_replay_20261021.py` generates a self-contained
  historical replay from the verified archive without checking the large raw
  CSV/JSONL files into Git.

## Lessons and follow-up

1. Route completion and control-link qualification are separate outcomes. The
   route completed, but the control session failed its reliability gate.
2. Do not weaken the 500 ms fail-closed behavior to make the mission appear
   more available. Diagnose the latency source first.
3. Phone screen-timeout settings and occasional slider interaction are not a
   heartbeat guarantee.
4. Dashboard telemetry age and phone-control link age describe different
   processes. Controller telemetry naturally becomes stale after a completed
   controller exits; the UI should label that terminal state clearly rather
   than invite a reset.
5. A green pre-Auto banner is not proof that route reacquisition will succeed.
   Reacquisition geometry should be made visible before an operator re-arms.
6. A neutral drivetrain can still roll on a slope. Parking-brake/terrain
   handling belongs in the recovery procedure.
7. The AXE75 consumer system log lacks the client-level observability needed
   for this investigation. The browser flight recorder and server log are the
   best available evidence, supplemented by controller and service telemetry.
8. The next work should be offline analysis. Tractor01 is shut down and no
   additional field action is required.

## Related notes

- [[20261006-master-manual-phone-rerun-network-failure]]
- [[20261019-phone-control-home-test-procedure]]
- [[20261019-manual-recovery-to-parked-position]]
- [[20261019-phone-network-authority-adr]]
- [[20261022-phone-heartbeat-timeout-telemetry-deep-dive-prompt]]
