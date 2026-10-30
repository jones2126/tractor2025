# DRAFT - UNTESTED IN THE FIELD

# October 28 phone-network gaps: incident and design review

Date reviewed: 2026-10-29

This review covers the October 28 master/manual phone run only. It does not
authorize motion, a blades-off run, or mowing. Server acceptance records are
authoritative. Phone/server one-way wall-clock differences are not used.

## Evidence identity

| Evidence | SHA-256 |
|---|---|
| `master_manual_phone_20261028_153941.csv` | `5b1799500d361394aacc9c3e3c1a314de40d0968fa4722c70e1f7459350aaa2e` |
| `pursuit_log_20261028_153944.csv` | `f96a4fd7fa6a1a6f9cce90a72aa7b0b65cda03d0d0141d1e201843630ce3d5c1` |
| `phone_flight_recorder_20261028_144314.jsonl` | `5b1af5fc1c830b132e00ee7f0ac9a29e90e6960a9a6d90fb683aceb8e3fa1f90` |
| `wifi_primary_control_20261028_144314.jsonl` | `7503a49801f93efff5667eebb8e4d3589401c3a5e7f21eeefa6a6aff53883503` |

The master log spans 4,590.804 seconds. The Pure Pursuit log spans 4,589.222
seconds. Small count differences result if the master logger boundary rather
than the controller boundary is used; the context document's phone counts use
the controller mission window.

## Verified metrics

| Metric | Result | Assessment |
|---|---:|---|
| Server `phone_freshness_expired` | 93 | confirmed |
| AUTO link-wait entries | 75 | confirmed |
| AUTO recoveries completed | 65 | confirmed |
| Link losses during recovery stabilization | 65 | confirmed |
| Server session invalidations | 178 | confirmed |
| Pure Pursuit stale-heartbeat episodes | 106 | confirmed |
| Stale-heartbeat total / median / maximum | 100.089 s / 0.350 s / 8.813 s | confirmed |
| Stabilization episodes / total | 107 / 412.318 s | confirmed |
| Accepted-command gap p50 / p95 / p99 / max | 220.2 / 315.0 / 517.2 / 17,184.2 ms | confirmed |
| Accepted-command gaps over 500 ms | 204 | confirmed using master-log boundary |
| Phone RTT p50 / p95 / p99 / max | 206.8 / 299.1 / 447.3 / 982.5 ms | confirmed within rounding |
| Phone failures | 71 HTTP 409 + 37 client aborts | confirmed |

The server handled accepted commands in 0.19 ms median, 0.25 ms p95, 0.78 ms
p99, and 37.94 ms maximum. This rules out ordinary server command processing
as the source of the hundreds-of-milliseconds acceptance gaps.

## Confirmed causes

1. **`send_queued` is a timer-collision counter, not packet queue depth.** The
   phone starts a timer every 200 ms while allowing one command request in
   flight. When the timer fires during a request, it records `send_queued` and
   sets one Boolean pending flag. After completion it sends only the newest
   state. With a 206.8 ms median RTT, collisions are the normal case. In the
   strict controller window, 15,302 of 16,580 reconstructed logical attempts
   had at least one collision; the median positive collision count was one,
   p95 two, and maximum six.
2. **The raw phone recorder includes duplicate uploads.** It had no event ID or
   idempotency check. If the server appended a batch but its response was lost,
   the phone retried the same batch. In the master window 1,082 records were
   exact duplicate event payloads; 364 were duplicate `send_queued` records.
   Therefore 18,587 is the number of stored records, not the number of unique
   logical collisions and certainly not network packets waiting to send.
3. **The freshness threshold had almost no timing margin.** Median RTT was
   already slightly longer than the 200 ms cadence; p99 accepted gap exceeded
   the 500 ms freshness limit. This made normal tail latency sufficient to
   trigger the server and Pure Pursuit safety gates.
4. **The 37 unexplained aborts were application timeouts.** The page creates an
   `AbortController` and aborts every API request after 1,000 ms without a
   reason. Thirty-six failures pair to attempts at 1,004-1,016 ms. At least
   eight of those command requests were accepted by the server before the
   phone abandoned the response, explaining subsequent client/server state
   disagreement.
5. **Session recovery amplified the initiating delay.** Of 203 consecutive
   accepted gaps over 500 ms inside the strict pursuit window, 132 crossed a
   session change. Stale requests from an invalidated session then produced
   HTTP 409 responses, claims invalidated older sessions, and five-second
   stabilization gates added 412.318 seconds of stopped time. The 409s are
   consequences, not the initiating network fault.

## Remaining unknowns

- The logs prove that commands arrived from phone overlay address
  `192.168.193.206` under `zerotier-primary`. They do **not** contain
  `zerotier-cli peers` path data, so direct versus relay transport is unknown.
  The NAS TCP reachability probe only proves reachability and briefly failed at
  20:48; it cannot identify relay use.
- Existing evidence cannot allocate tail delay among Android/browser
  scheduling, phone-to-router radio, router/Ethernet, and ZeroTier. The page
  stayed visible and online according to browser hints, but those hints do not
  measure radio retries, roaming, or overlay path changes.
- Router station RSSI/rate/retry logs, bounded packet capture, interface-route
  changes, TLS/connection reuse, and Pi scheduling telemetry were not recorded.
- Eight aborts were demonstrably server-accepted; the remaining aborts cannot
  be classified as request loss versus accepted-with-unobserved-response from
  the old schema.

## Safety design decision

Do not increase the existing 500 ms watchdog. Phone supervisory loss is a
different fault from stale local motion commands.

The candidate `COMMUNICATIONS DEGRADED` state is disabled pending stationary
approval. It may eventually preserve AUTO only if all of these independent
local checks remain healthy: physical Pause released, physical E-stop released,
fresh Teensy status, healthy Teensy command watchdog, fresh Pure Pursuit
`cmd_vel`, fresh controller telemetry, fresh RTK Fixed position, valid fixed
heading, a defined AUTO route, corridor compliance, and speed at or below the
approved limit. Any failed or missing check requires immediate Pause.

The absolute envelope is:

```text
allowed = preauthorized
          AND elapsed_from_first_loss < 30 s
          AND max(path_progress_delta, locally_integrated_travel) < 30 m
          AND every local safety check is healthy
```

At 1.0 m/s, 30 seconds is 30 m or 98.4 ft (often rounded informally to about
90 ft). Both time and distance are hard limits. The incident begins at the
first loss. A reconnect does not reset either origin; reset requires explicit
operator acknowledgement while physically in Pause and confirmed stopped.
ntfy is best-effort information only: it can be delayed by the same outage and
must never grant or extend motion authority.

## Implemented candidate changes

- Phone flight events now have unique IDs; server ingestion deduplicates
  retried events.
- The phone reports scheduler delay, coalesced wait, collision totals, pending
  depth, longest request, explicit abort reason, request ID, corrected
  request/response clock-offset estimate, server processing, and network RTT
  estimate.
- The noisy per-collision flight event was replaced by a five-second scheduler
  summary while preserving newest-state coalescing.
- The server publishes a secret-free atomic diagnostics snapshot for the
  mission dashboard.
- The Messages view now separates exact stop/wait reason, accepted-command
  gap, session claims/invalidations, server-to-bridge age, Pi/Teensy modes,
  Pure Pursuit status, ZeroTier path measurement state, ntfy pending/ack state,
  and degraded-mode gate state.
- A tested, motion-free guard implements the sticky time-and-distance envelope.
  It is not connected to motion authority and is explicitly reported disabled.

## Recommended priority

1. Deploy instrumentation only and run stationary A/B trials.
2. Qualify direct onboard-router control; keep ZeroTier for maintenance/status
   unless ZeroTier independently meets the real-time acceptance criteria.
3. Replace repeated independent fetch control with a persistent local transport
   after A/B evidence, while retaining immediate Pause/E-stop paths.
4. Add router station/retry collection, ZeroTier direct/relay sampling, bounded
   packet capture, and Pi resource/scheduling telemetry.
5. Only after stationary fault tests pass, review integration of the disabled
   degraded guard with independent local health producers.
6. Only after a separate blades-off approval, run a short bounded route test.
7. Only after that evidence is reviewed, request separate approval to rerun the
   known valid mission. No approval in this document implies the next one.

