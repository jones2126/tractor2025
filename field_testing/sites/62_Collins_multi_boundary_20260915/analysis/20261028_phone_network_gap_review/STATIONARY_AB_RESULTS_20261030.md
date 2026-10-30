# DRAFT - UNTESTED IN THE FIELD

# 2026-10-30 stationary phone-network A/B results and next gate

Reviewed: 2026-10-30

This document reviews Approval A evidence only. It does not authorize tractor
motion, blades, COMMUNICATIONS DEGRADED, a bounded-route test, or a mission
rerun. Server acceptance records are authoritative. Phone/server one-way
wall-clock subtraction is not used.

## Evidence identity and scope

The supplied archive is
`20261030_stationary_network_ab_evidence.tgz`, SHA-256
`33a7af61c3c5df029e8677dcd2b7731d190ae59596ea1ec10ed166cbc883e502`.
All 16 entries in `SHA256SUMS_NAS.txt` independently verified after extraction.
The run directory retains the planned `20261029` name, but the evidence and
operator notes are dated 2026-10-30.

Only these intervals are treated as the intended trials:

| Trial | Server log | Controlled interval |
|---|---|---|
| A, HTTPS over ZeroTier | `wifi_primary_control_20261030_131635.jsonl` | 13:18:11–13:28:11 EDT |
| B, direct onboard-router HTTPS | `wifi_primary_control_20261030_133500.jsonl` | 13:37:11–13:47:11 EDT |

The 12:50 and 13:01 starts are setup attempts. They are excluded because the
first lacked the visible dry-run banner and the second mixed simulated and live
bridge telemetry. Commits `a774971` and `87b0144` corrected those defects before
the intervals above.

The operator note says the walk-away began at approximately 13:44. Trial B is
therefore reported in two parts: a controlled direct-Wi-Fi baseline ending at
13:43:59.999 and the subsequent intentional coverage-loss fault. The latter is
useful fault evidence but must not be presented as ordinary direct-path jitter.

## Measured results

| Measurement | Trial A: ZeroTier, full 10 min | Trial B: direct, before walk | Trial B including walk fault |
|---|---:|---:|---:|
| Controlled duration | 600 s | 409 s | 600 s |
| Accepted commands | 2,542 | 2,044 | 2,461 |
| Accepted rate | 4.237 Hz | 4.998 Hz | 4.102 Hz |
| Accepted gap p50 / p95 / p99 | 209.2 / 307.5 / 480.1 ms | 200.2 / 218.6 / 231.5 ms | 200.2 / 219.8 / 233.9 ms |
| Accepted gap p99.9 / max | 3,561.2 / 8,609.4 ms | 291.9 / 398.3 ms | 429.3 / 107,091.7 ms |
| AUTO accepted gaps over 500 ms | 18 | 0 | 1 |
| Phone freshness expiries | 15 | 0 | 2 |
| Effective mission-stop episodes | 15 | 0 | 2 |
| Stopped time | 128.920 s | 0 s | 119.734 s |
| Sessions observed in accepted commands | 18 | 1 | 3 |
| Phone send failures | 19 | 0 | 1 |
| Phone RTT p50 / p95 / p99 / max | 196 / 298 / 424 / 732 ms | 18 / 63 / 97 / 384 ms | 19 / 65 / 107 / 560 ms |
| Phone scheduler-delay p95 / max | 0 / 19 ms | 1 / 4 ms | 1 / 4 ms |
| Server processing p95 / max | 0.18 / 3.05 ms | 0.17 / 1.61 ms | 0.18 / 1.61 ms |
| Simulated bridge-age p95 / max | 48.1 / 51.3 ms | 47.9 / 50.1 ms | 47.9 / 50.1 ms |

The reproducible analyzer is `analyze_stationary_ab_20261030.py`. It filters
phone records by each embedded event timestamp because flight-recorder uploads
can contain older locally buffered events.

## What Trial A would have done to a moving mission

Trial A crossed the 500 ms freshness policy often enough to enter a stopped
link-wait state 15 times. The individual stop durations were:

`5.673, 5.878, 19.254, 5.341, 5.775, 13.290, 5.513, 12.393, 5.938,
13.988, 6.185, 7.514, 6.262, 6.247, and 9.670 seconds`.

Total stopped time was 128.920 seconds; median was 6.247 seconds and maximum
was 19.254 seconds. Eleven episodes completed automatic recovery. Four returned
to AUTO through a fresh operator AUTO selection. There were 27 session
invalidations in the full server log and 18 distinct sessions in accepted
commands during the timed window. The resulting churn is amplification after
the initiating delay, not proof of slow server processing.

## Trial B fault timeline

- 13:44:00.106: server phone freshness expired and immediate link-wait/stop
  began.
- 13:44:05.994: automatic recovery completed after 5.888 seconds.
- 13:44:07.583: freshness expired again and the second stop began.
- 13:44:37.592: the existing 30-second AUTO-recovery claim window expired. This
  window did not permit continued motion; the stop had already occurred.
- 13:44:37: GL router logged the Pixel client disconnected, followed by brief
  reconnect/disconnect churn.
- 13:45:41: GL router logged a durable reconnect.
- 13:46:01.429: a new operator AUTO selection ended the second 113.846-second
  stopped interval.

The router associated randomized client MAC `5a:55:ca:3f:5f:95` with
`192.168.8.190`. It logged the initial association at 13:30:24, disconnect at
13:44:37, a two-second reconnect, another disconnect, and durable reconnect at
13:45:41. The exported diagnostic snapshot did not include station RSSI, radio
retry, failed-transmission, or rate history, so the exact RF degradation before
disassociation remains unmeasured.

## Layer attribution

### Confirmed

1. The poor Trial A behavior was not caused by ordinary Pi server processing.
   Its 0.18 ms p95 processing time is negligible beside the 298 ms phone RTT
   p95 and multi-second accepted-command gaps.
2. The Trial A phone browser was visible and reported online. Its local timer
   scheduler delay was small (0 ms p95, 19 ms maximum). The large coalesced wait
   and session churn followed long in-flight network requests rather than a
   long browser timer stall.
3. Three Trial A failures were explicit `api_timeout_1000ms` aborts. Sixteen
   HTTP 409 failures were stale-session consequences after invalidation. This
   matches the October 28 finding: the formerly unexplained aborts were the
   page's application timeout, while 409s amplified recovery churn.
4. The ZeroTier phone peer was `DIRECT`, not `RELAY`, in the before and after
   snapshots. Before Trial A its underlay endpoint was public address
   `153.66.141.241:29605`; afterward, once the phone was on the GL LAN, it was
   `192.168.8.190:9994`. No continuous peer sampler was running, so direct
   transport at every individual failure instant is highly supported but not
   proven.
5. Direct onboard-router HTTPS was materially better before the intentional
   walk: one session, no failures, no freshness expiry, no gap over 500 ms, and
   a 291.9 ms accepted-gap p99.9. This controlled portion meets the timing and
   churn criteria for that one sample.
6. The walk-away fault was a local coverage/path-loss event. It produced the
   required immediate stop under the present policy and did not silently keep
   AUTO running.

### Not established by this test

- Trial A changed both physical Wi-Fi path and ZeroTier use, so it cannot split
  TractorField radio/router/WAN delay from ZeroTier overlay delay. The AXE75
  log and per-phone radio statistics are still needed.
- Simulated bridge telemetry was used. This test does not qualify the real
  Pi-to-Teensy serial bridge, Teensy command watchdog, or Pure Pursuit
  `cmd_vel` freshness.
- The GL export proves association churn but lacks radio retry statistics.
- One 409 was recorded in the direct fault segment. Request-ID correlation
  makes it attributable to session invalidation, not an unexplained transport
  abort.
- ntfy was intentionally disabled. Queueing, server acknowledgement, delayed
  delivery, and outage behavior remain to be tested separately.

## Strategy recommendation — requires separate approval

Use the direct onboard GL router as the candidate real-time phone authority for
the next stationary qualification and later blades-off validation. Keep
ZeroTier for dashboard/status, maintenance, log transfer, and a separately
tested stop path; do not use the present ZeroTier fetch path as the primary
real-time heartbeat for motion.

Do not enable COMMUNICATIONS DEGRADED yet. Do not extend the existing 500 ms
local watchdog. Preserve immediate local command, Pure Pursuit, bridge,
Teensy, physical Pause, and e-stop authority. The candidate degraded envelope
remains disabled until real local-health signals and fault tests prove that it
fails closed. If later approved, it must use the original first-loss time and
distance origins, end at the first of 30 seconds or 30 m, and require a stopped
physical-Pause operator acknowledgement before reset. Reconnects and ntfy
notices cannot reset or extend it.

## Recommended staged next actions

### Decision 1 — strategy approval only

Approve or reject direct onboard-router control as the candidate real-time
path. This decision does not authorize another test or motion.

### Decision 2 — Approval A continuation, stationary only

If the strategy is approved, run one replacement clean ten-minute direct trial
without walking or changing networks. The current B evidence has only 409
controlled seconds, not the planned ten minutes. Also run the automated
motor-off fault matrix with ntfy enabled for notification-state observation;
ntfy must not influence authority. Collect continuous ZeroTier peer state,
router station/retry data if available, and bounded packet/Pi scheduling data.

Required result before requesting motion: zero accepted gaps at or above
500 ms; accepted-gap p99.9 no more than 350 ms; zero unexplained aborts,
freshness expiries, or session invalidations in the clean direct run; and exact
expected immediate-stop reasons for every injected local safety fault.

### Decision 3 — Approval B, separately authorized blades-off bounded route

Only after Decision 2 evidence passes: prepare and review a short bounded-route
package. Begin stationary in physical Pause with blades off; apply and
independently read back the current-session Heading-F9P moving-base profile;
restart and require `rtcm-server` active; wait for heading reacquisition; and
require a fresh `MISSION PREFLIGHT PASS`. Keep the rear transmission lever
pulled out through controller start and `READY FOR GUARDED AUTO`; pushing it in
is a distinct physical-movement hold point. Preserve the operator's confirmed
Manual-to-start, Pause, lever-out, dashboard-start, ready, lever-in, ground-
beside-tractor, handheld-AUTO sequence.

### Decision 4 — Approval C, known valid mission rerun

Only after Approval B evidence is collected and reviewed: select the exact
known valid mission revision, freeze its hash, choose blades state explicitly,
and issue a new checklist and evidence collector. A successful bounded-route
test does not itself authorize that mission. The rerun must retain the same
current-session Heading-F9P proof, preflight PASS, direct-router control,
physical Pause/e-stop authority, and post-run hashed evidence collection.

## Verification performed on the development workspace

- The analyzer reproduced the metrics above from the supplied archive.
- Python compilation passed.
- 53 focused phone-control, degraded-guard, dashboard, and URL-publication
  tests passed with zero failures.
- `git diff --check` passed.

## Decision record

On 2026-10-30, Al approved the recommendations in this review. The recorded
scope is:

- direct onboard GL-router control is the selected candidate real-time path;
- one clean stationary direct-path replacement run and the motor-off software
  fault suite may proceed as an Approval A continuation; and
- COMMUNICATIONS DEGRADED, physical movement, Approval B, Approval C, blades,
  and a mission rerun remain separately gated and are not authorized by this
  decision.
