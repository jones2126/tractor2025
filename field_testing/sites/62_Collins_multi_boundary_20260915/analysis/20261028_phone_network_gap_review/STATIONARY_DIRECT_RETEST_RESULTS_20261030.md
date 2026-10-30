# DRAFT - UNTESTED IN THE FIELD

# 2026-10-30 clean direct-router stationary retest results

Reviewed: 2026-10-30

This is an Approval A continuation result. It does not authorize physical
movement, blades, COMMUNICATIONS DEGRADED, Approval B, Approval C, or a mission
rerun. Server acceptance timestamps are authoritative. Phone/server one-way
wall-clock subtraction is not used.

## Evidence identity

Archive:
`20261030_stationary_direct_retest_evidence.tgz`

Archive SHA-256:
`4be13100bba1046c3f52246cb39e94d511f245614955ad442dc92df8243ab017`

All 13 files in the internal relative-path `SHA256SUMS` manifest independently
verified after extraction. The Tractor01 source revision was
`a417162cf0a541d0a038acae72cb2c0a7f01fa5e`. The only reported working-tree
items were the two pre-existing untracked PlatformIO `.ini` files.

The physical hold point recorded engine off, blades off, rear hydrostatic
transmission lever pulled out, physical Pause selected, and wheels chocked.
The dry-run server used simulated bridge telemetry and had UDP 6004 disabled.

## Authoritative interval

The marked controlled interval was exactly:

- local: 2026-10-30 15:16:12–15:26:12 EDT;
- UTC: 2026-10-30 19:16:12–19:26:12Z; and
- duration: 600.000 seconds.

The server actually entered `auto_running` at 15:16:09.975 and returned to
`pause_latched` at 15:26:38.226, providing 628.251 seconds of continuous
dry-run AUTO around the conservative marked analysis window.

## Measured results for the marked ten minutes

| Measurement | Result |
|---|---:|
| Accepted AUTO commands | 3,000 |
| Accepted rate | 5.000 Hz |
| Accepted gap p50 / p95 / p99 | 200.2 / 213.005 / 228.701 ms |
| Accepted gap p99.9 / maximum | 255.206 / 281.8 ms |
| Accepted gaps over 400 / 500 / 1,000 ms | 0 / 0 / 0 |
| Phone send attempts / successes / failures | 3,000 / 3,000 / 0 |
| Phone RTT p50 / p95 / p99 | 19 / 63 / 82.01 ms |
| Phone RTT p99.9 / maximum | 120.024 / 154 ms |
| Network RTT p50 / p95 / p99 / maximum | 19 / 62.05 / 82.01 / 154 ms |
| Scheduler delay p50 / p95 / p99 / maximum | 0 / 1 / 1 / 9 ms |
| Coalesced wait p99.9 / maximum | 0 / 0 ms |
| Server processing p50 / p95 / p99 | 0.14 / 0.18 / 0.66 ms |
| Server processing p99.9 / maximum | 0.77 / 41.85 ms |
| Simulated bridge age p50 / p95 / p99 / maximum | 24.8 / 47.5 / 49.601 / 50.1 ms |
| Sessions | 1 |
| Freshness expiries / invalidations / rejections | 0 / 0 / 0 |
| Safety-state transitions or stopped episodes | 0 |

The one 41.85 ms server-processing outlier did not create a problematic
accepted-command gap. All other tail measures retained substantial margin
inside the 500 ms current freshness policy.

## Supporting evidence

### Direct-path and ZeroTier sampling

The phone control requests came only from direct-LAN address `192.168.8.190`
under `tractorfield-mission-custom-subnet`. ZeroTier was status-only and could
not carry motion authority in this server policy.

During the marked interval, the two-second sampler captured 288 phone-peer
observations for node `82a60e38ec`. Every sample was `DIRECT`, every path was
`192.168.8.190/9994`, and reported peer latency was 11 ms minimum, p95, and
maximum. The unrelated `d3509d6b53 RELAY` peer in the peer table was not the
phone.

### Pi and wired interface

During the marked interval:

- 293 Pi-health samples were recorded;
- temperature ranged from 42.2°C to 46.1°C;
- maximum one-minute load average was 1.24; and
- `eth0` RX errors, RX drops, TX errors, and TX drops remained zero.

These results provide no evidence of Pi overload, thermal stress, or wired-link
failure during the trial.

### Packet capture

The capture ran from 15:12:59.892 to 15:23:59.286 EDT. It covered the first
467.286 seconds of the marked interval, not the final 132.714 seconds. It
recorded 27,438 packets total and tcpdump reported zero packets dropped by the
kernel. Because the capture used interface `any` and no full TCP expert
analysis is bundled here, repeated sequence observations must not be promoted
to proven radio or TCP retransmission counts. The complete server and phone
logs cover the entire marked interval.

### ntfy and software fault evidence

The server recorded `ntfy.last_result=acknowledged`, with no pending notice.
This proves server acknowledgement for the test notice, not delivery timing to
the phone. ntfy remained informational and did not affect authority.

The 53-test deterministic motion-free suite passed. It covers current
freshness fail-closed behavior, session recovery rules, reconnects not
extending a degraded incident, independent time/distance limits, every defined
local-health input failing closed, stopped-Pause acknowledgement before reset,
diagnostic publication, ntfy retry state, and simulated/live bridge isolation.

## Gate assessment

| Approval A continuation condition | Assessment |
|---|---|
| Zero accepted gaps at or above 500 ms | PASS |
| Accepted-gap p99.9 no more than 350 ms | PASS — 255.206 ms |
| Zero freshness expiries | PASS |
| Zero unexplained aborts | PASS |
| Zero session invalidations | PASS |
| Direct control path known | PASS — local `192.168.8.190` |
| ntfy state measured without authority effect | PASS — acknowledged |
| Pi resource and wired-interface stability | PASS for this stationary sample |
| Real Pi-to-Teensy and Pure Pursuit watchdog qualification | NOT TESTED — simulated bridge, no mission |

The direct-router stationary continuation passes. Together with the earlier
409-second clean pre-walk portion of Trial B, the evidence now contains two
independent direct-path samples with zero 500 ms gaps, zero freshness expiry,
and no unexplained failures. Only one sample is a complete controlled ten
minutes; this is not the original five-repetition full statistical
qualification.

## Decision boundary and recommendation

Approval A continuation is complete. The evidence supports a separate request
to prepare an Approval B package for one short, directly supervised, blades-off
bounded-route run using the direct onboard GL-router phone path.

That future package must:

- keep COMMUNICATIONS DEGRADED disabled;
- preserve the current 500 ms phone freshness behavior and every immediate
  local Pure Pursuit, bridge, Teensy, physical Pause, and e-stop watchdog;
- begin stationary in physical Pause with blades off and rear transmission
  lever pulled out;
- apply and independently read back the current-session Heading-F9P moving-base
  profile, restart and require `rtcm-server` active, wait for heading
  reacquisition, and require a fresh `MISSION PREFLIGHT PASS`;
- require fresh real bridge, Teensy, GPS/RTK, heading, and Pure Pursuit evidence
  before the distinct lever-in movement hold point;
- use the operator-confirmed Manual-to-start, Pause, lever-out, dashboard-start,
  ready, lever-in, ground-beside-tractor, handheld-AUTO sequence;
- define immediate Pause/Manual/e-stop criteria and stop on the first phone
  expiry, session invalidation, local-health failure, or unexplained warning;
  and
- collect and hash server, phone, bridge, Pure Pursuit, mission, preflight,
  service, network, and operator-note evidence.

This recommendation is permission to request/prepare Approval B only. It is not
field authorization. The known valid mission remains Approval C and requires a
later, separate decision after Approval B evidence is reviewed.
