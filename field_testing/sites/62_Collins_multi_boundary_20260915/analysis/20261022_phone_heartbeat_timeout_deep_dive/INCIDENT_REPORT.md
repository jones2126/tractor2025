# 2026-10-21 phone-freshness timeout telemetry deep dive

## Finding and confidence

**Finding:** The 14:18:14.028 EDT Auto stop was triggered by a delayed
request/response completion, amplified by the page's single-flight fetch design.
The browser initiated command sequence 14115 on time. Tractor01 received it at
18:18:13.492Z, accepted it at 18:18:13.493Z, and spent only 0.38 ms in the
instrumented command handler. The browser did not record the successful response
until 710 ms after it initiated the request. While that fetch remained outstanding,
the 200 ms heartbeat timer continued to run but only recorded `send_queued`; it did
not start another request. The browser's 500 ms acknowledgement watchdog failed
closed, and the server independently expired the session 535 ms after its last
accepted command.

**Confidence:** High for this application-level sequence and for excluding a slow
command-handler or stale bridge as the immediate cause. Medium that the delay was
on the response/return side rather than inside the browser after network delivery.
Low for assigning the delay to one of HTTP/TLS response writing, ZeroTier, either
underlay, AXE75 forwarding, or Android/Chromium delivery because the archive has no
packet trace, server response-write timestamp, ZeroTier path telemetry, or Android
network/power trace.

This analysis does **not** support weakening the 500 ms fail-closed threshold or
authorizing another field mission. The safety response operated as designed.

## Evidence integrity and method

The outer archive SHA-256 matched the expected value:

`a316fcd031e60afdba5cddd6d9baf29377956043f5b2a772e74bd04344387574`

All nine files listed in `diagnostics/collected_file_hashes.sha256` matched their
recorded hashes. Both original AXE75 exports also matched the hashes in the field
summary:

- 10:52 export: `088478fb28d96d42d184c61eaf02d4b9408044498bf5b81d16ffb7d74fae6c16`
- 15:32 export: `edd6ce33cad3ef2a179ae6ce182b748ce18ae6d305df234d04a369318baadc75`

The archive was extracted only to a temporary directory. Existing staged and
unstaged repository work was preserved. The analysis script writes only the
derived files in this directory.

## Clock normalization

| Source | Native clock | Normalization and limitation |
|---|---|---|
| Control-server JSONL | Tractor01 UTC ISO-8601, millisecond display | Primary clock domain. Safety uses server monotonic time, not these wall-clock strings. |
| Combined mission CSV | UTC ISO-8601 | Directly aligned with the server wall clock. |
| Pursuit CSV | Unix epoch seconds | Interpreted as UTC; 20 Hz samples. |
| Browser event payload | Phone `Date` UTC plus `performance.now()` | Phone wall clock was ahead of the server. Server-receive minus phone-send ranged from -1673 ms to +6568 ms; median -1607 ms. The minimum, -1673 ms, is used only as a diagnostic upper bound on clock offset because it still includes outbound transit. |
| Browser wrapper `server_time` | Server UTC when a later batch was uploaded | **Not an occurrence time.** Median upload delay was 3.777 s, p95 12.384 s, and maximum 1692.685 s. |
| Service journal | Tractor01 local EDT, one-second display | Converted with UTC-04:00. Useful for service continuity, not millisecond ordering. |
| AXE75 export | Router local EDT, one-second display | Converted with UTC-04:00. The header and mission-era entries agree with local time; boot entries initially use firmware-build or epoch dates before time sync. |

The normalized 60-second, full-resolution join is in
`critical_window_timeline.csv`. Browser rows use the documented -1673 ms alignment
bound; their original phone timestamps remain in each row's JSON details.

## 14:18:14 reconstruction

The most useful sequence is below. Browser-aligned times are approximate within
the clock-offset bound; server, mission, and pursuit times are direct.

| UTC | EDT | Observation |
|---|---|---|
| 18:18:13.160 | 14:18:13.160 | Browser initiated sequence 14114. |
| 18:18:13.230 | 14:18:13.230 | Server received and accepted 14114; command gap 121.7 ms, processing 0.17 ms, bridge age 25.6 ms. |
| 18:18:13.359 | 14:18:13.359 | Browser completed 14114 after 199 ms RTT. |
| **18:18:13.360** | **14:18:13.360** | Browser initiated sequence **14115**, Auto/heartbeat, page visible, `navigator.onLine=true`, connection hint `type=wifi`, wake lock last reported acquired. |
| 18:18:13.492 | 14:18:13.492 | Server received 14115 from `192.168.193.206`. |
| 18:18:13.493 | 14:18:13.493 | Server accepted 14115 in session `s8LtPKTARq0-uhVMGhxZ3g`; command gap 262.2 ms, prior client RTT 198.7 ms, processing 0.38 ms, bridge age 46.3 ms. |
| 18:18:13.497-13.897 | 14:18:13.497-13.897 | Later 200 ms timer callbacks ran, but `sending=true`; the page logged three `send_queued` events instead of starting another fetch. |
| about 18:18:13.900 | about 14:18:13.900 | Browser acknowledgement age reached about 540 ms. It locally changed to Pause, cleared the session, and logged `ack_watchdog_expired`. It did not log `offline`, a connection-hint change, a visibility change, or a fetch error. |
| **18:18:14.028** | **14:18:14.028** | Server monotonic freshness exceeded 500 ms (535 ms since the accepted command), logged `heartbeat_expired`, invalidated the session, and began a five-command Pause burst. |
| about 18:18:14.071 | about 14:18:14.071 | The outstanding 14115 fetch finally completed successfully in the browser. Browser RTT was **710 ms**. This was too late to preserve either watchdog. |
| 18:18:14.129 | 14:18:14.129 | Server logged completion of the stop burst. |
| 18:18:14.149 | 14:18:14.149 | Combined telemetry first showed transmission mode Pause and JRK target neutral 2836, 121 ms after expiry. |
| 18:18:14.214 | 14:18:14.214 | Pursuit first observed phone Pause and stopped driving, 186 ms after expiry. It froze recovery progress at waypoint **15,362** in `garden_left_ring_2`; RTK remained Fixed and heading remained valid. |
| 18:18:14.416 | 14:18:14.416 | Server claimed fresh Pause session `qHGnRIwpY6uWXkugLih5cA`, 388 ms after expiry. |
| 18:18:14.755 | 14:18:14.755 | First new-session Pause command arrived; it echoed the preceding 710.2 ms browser RTT and had a 1262.6 ms cross-session command gap. |
| 18:18:15.335 | 14:18:15.335 | JRK feedback first reached neutral within 10 counts (2838 versus target 2836), 1.307 s after expiry. |
| 18:18:38.429 | 14:18:38.429 | Operator deliberately selected guarded Auto. Later Pause/Auto/E-stop transitions are recovery actions, not the initiating event. |

Immediately before expiry, pursuit still showed Auto, 1.0 m/s command, about
0.84 m/s actual speed, RTK Fixed, valid heading, and active waypoint 15,374-15,375.
When the controller observed Pause it intentionally reset/froze recovery progress
to waypoint 15,362. This reconciles the apparently different waypoint numbers.

### What kind of failure was 14:18?

It was **not a request that started late**: the browser's 200 ms timer was running
and sequence 14115 began on schedule. It was **not a request that arrived after the
server deadline**: the server accepted it 535 ms before expiry. It was a request
whose response became visible to JavaScript too late, while the page's single-flight
guard prevented subsequent timer ticks from starting requests.

The evidence cannot distinguish among:

- delay while the threaded HTTPS server serialized/wrote the response or while the
  kernel/TLS stack transmitted it (the 0.38 ms metric ends before response writing);
- loss, retransmission, relay, or queueing on the ZeroTier/underlay return path; and
- prompt network receipt followed by delayed Chromium/Android delivery or task
  scheduling.

There was no `send_failed` at this incident; sequence 14115 eventually logged
`send_ok`. That contradicts a lost request followed immediately by reconnection.

## Flight-recorder schema: what is measured

The browser JavaScript creates each nested `event` payload. Every event contains a
phone wall-clock time, rounded `performance.now()` value, event name, page
visibility, `navigator.onLine`, a Boolean indicating whether the JavaScript variable
currently holds a session, the current local sequence, and optional Network
Information API hints (`type`, `effective_type`, `downlink`, and `rtt_hint_ms`).

Browser-originated event types include page load and visibility, wake-lock
acquire/release/failure, claim attempt/success/failure, send attempt/queued/OK/fail,
ack-watchdog failure, browser online/offline, connection-hint change, and local
session clearing. `send_ok.rtt_ms` is measured in JavaScript around `fetch`.
`send_ok.phone_mode` and `session_claimed.phone_mode/authority_path` are values
returned by the server. Error status/message may come from an HTTP response or from
the Fetch/Abort APIs.

The outer wrapper is added later by Tractor01 when a batch is uploaded. Its
`server_time` is batch receipt time and `client_ip` is the source address of the
diagnostic upload. Uploading diagnostic events does not grant, refresh, or change
motion authority.

The recorder can prove that timer callbacks executed, that the single-flight guard
queued them, when JavaScript initiated fetches, when fetch promises resolved or
rejected, browser-reported page/online hints, local session state, and approximate
monotonic intervals within one page lifetime. It cannot prove packet transmission
time, TCP/TLS acknowledgement time, response arrival at the phone kernel, radio
state, Android Doze/power state, actual Wi-Fi association quality, ZeroTier peer
path/relay state, or whether a long fetch completed late in the network versus late
in browser task delivery. `navigator.onLine` and Network Information fields are
hints, not path-health evidence. Wake-lock state is event-derived rather than
recorded on every row.

## All 23 freshness expiries

The detailed table is `expiry_events.csv`. It includes both clock forms, session,
sequence, preceding/following RTT and gaps, browser events, controller/mission
state, RTK/heading, phase, recovery claim/mode, and neutralization timing.

| # | EDT | Context | Application signature | Powered Auto interrupted |
|---:|---|---|---|:---:|
| 1 | 13:31:11.315 | Pre-mission Manual/control preparation; controller CSV not active | 524 ms delayed successful response; queued sends | Unknown/not demonstrated |
| 2 | 14:18:14.028 | Primary mission, `garden_left_ring_2` | 710 ms delayed successful response; queued sends | Yes |
| 3 | 14:46:15.249 | Continued mission, `garden_left_ring_5` | Outstanding request failed/reconnected | Yes |
| 4 | 14:49:29.849 | `front_yard_ring_1` | 929 ms delayed successful response | Yes |
| 5 | 14:49:39.313 | `front_yard_ring_1` | 626 ms delayed successful response | Yes |
| 6 | 14:50:08.260 | `front_yard_ring_1` | 593 ms delayed successful response | Yes |
| 7 | 14:50:22.131 | `front_yard_ring_1` | 629 ms delayed successful response | Yes |
| 8 | 14:50:42.062 | `front_yard_ring_1` | Outstanding request failed/reconnected | Yes |
| 9 | 14:50:52.679 | `front_yard_ring_1` | Outstanding request failed/reconnected | Yes |
| 10 | 14:52:02.038 | `front_yard_ring_2` | Outstanding request failed/reconnected | Yes |
| 11 | 14:53:07.038 | `front_yard_ring_2` | Outstanding request failed/reconnected | Yes |
| 12 | 14:54:12.040 | `front_yard_ring_3` | Outstanding request failed/reconnected | Yes |
| 13 | 14:56:39.669 | `transition_11_12` | Outstanding request failed; long reclaim delay | Yes |
| 14 | 14:57:53.482 | `over_the_road_24in_clearance_bypass` | Outstanding request failed/reconnected | Yes |
| 15 | 14:58:35.647 | Same bypass phase | Outstanding request failed/reconnected | Yes |
| 16 | 14:58:49.869 | Same bypass phase | Outstanding request failed/reconnected | Yes |
| 17 | 14:59:10.050 | Same bypass phase | Outstanding request failed/reconnected | Yes |
| 18 | 14:59:25.023 | Same bypass phase | Outstanding request failed/reconnected | Yes |
| 19 | 14:59:40.046 | Same bypass phase | Outstanding request failed/reconnected | Yes |
| 20 | 15:00:51.155 | Same bypass phase | 778 ms delayed successful response | Yes |
| 21 | 15:00:57.915 | Continued-mission recovery/wait state | Outstanding request failed/reconnected | No powered command at sample |
| 22 | 15:05:51.650 | Post-mission Manual maneuvering | Outstanding request failed; long reclaim delay | No mission Auto |
| 23 | 15:07:40.260 | Post-mission Manual maneuvering | Outstanding request failed/reconnected | No mission Auto |

Thus the count is not 23 identical mission stops: one was pre-mission, one was the
primary long Auto interruption, 18 occurred while later powered Auto was active,
one occurred in a later controller wait/recovery state, and two occurred during
post-mission Manual maneuvering. None coincided with page hide/reload, an
`offline` event, or a connection-hint-change event. The deliberate hide/restore at
about 14:41:59/14:42:16 happened before continued operation and correctly caused
Pause; it is not one of the 23 expiries.

## Why 14:46-15:01 became frequent

This interval is a real cluster, not one continuous outage. Every expiry forced a
fresh Pause session and required a guarded re-arm. Five powered-Auto events in the
cluster have the same delayed-successful-response signature as 14:18; the other 13
powered-Auto events show the next outstanding fetch failing or arriving only after
session invalidation. Event 21 occurred while the controller was already waiting.

The browser remained visible, reported online/Wi-Fi, and had no wake-lock release.
The server and bridge stayed responsive. The strongest new contextual signal is an
independent Tractor01-to-NAS ZeroTier readiness probe becoming intermittent:

- transient failures at 14:44:14, 14:45:45, 14:49:07, 14:50:23, and 14:50:45;
- a declared loss at 14:50:55 followed by recovery at 14:51:16;
- another failure/recovery series at 14:53:59, 14:57:51-14:58:29, and
  14:58:37-14:59:01.

Those probes use a different peer/destination from the phone control request, so
they do not prove that ZeroTier itself caused every expiry. They do show that the
later cluster occurred during broader overlay-or-underlay instability visible from
Tractor01. That signature is absent at 14:18. The cluster therefore probably shares
the same immediate fetch/ack mechanism but has stronger evidence for a network-path
contribution than the isolated 14:18 event.

## Healthy baseline versus expiry windows

Server command metrics below compare armed accepted commands more than 10 seconds
from any expiry with armed accepted commands within two seconds of an expiry.

| Metric | Healthy p50 / p95 / p99 / max | Expiry-window p50 / p95 / p99 / max |
|---|---:|---:|
| Command gap ms | 200.7 / 212.9 / 238.0 / 408.9 | 201.9 / 295.2 / 409.7 / 445.3 |
| Prior browser RTT echoed by next command ms | 35.7 / 83.0 / 134.7 / 337.9 | 52.0 / 194.4 / 366.6 / 520.6 |
| Server processing ms | 0.17 / 0.31 / 0.64 / 24.66 | 0.17 / 0.49 / 0.73 / 1.76 |
| Bridge age ms | 12.1 / 81.5 / 91.4 / 118.7 | 11.5 / 85.1 / 93.7 / 94.9 |

Browser-measured successful fetch RTT near expiries was p50 56 ms, p95 396 ms,
p99 745 ms, and maximum 929 ms, versus 36/86/154/883 ms outside 10-second expiry
guards. The healthy maximum is a rare outlier; the distribution and repeated
watchdog events, not maxima alone, separate the failure windows.

Within the combined two-second expiry windows the recorder captured 26 local ack
watchdog events, 97 queued sends, 17 failed command fetches, 17 claim attempts, and
15 successful claims. Counts can exceed 23 because windows overlap and flight
events from more than one active page/history may coexist. Failure details were 16
Fetch aborts (`signal is aborted without reason`) and two HTTP 409 stale-session
responses (one of the 18 failures was a claim failure). There were no browser
`offline` events in the archive.

Accepted server sequences had only four within-session discontinuities across the
full collected log. These occurred at 14:23:48, 14:50:59, 14:51:11, and 15:08:48;
they are consistent with failed/aborted attempts incrementing the browser sequence.
Sequence gaps are a symptom, not evidence of server compute delay.

## Analyzer audit

The existing analyzer's main qualification numbers reproduce exactly:

- longest continuous armed interval: 38.733 minutes (reported 38.7);
- armed command intervals: 19,740;
- RTT p95 85.0 ms and maximum 520.6 ms;
- maximum armed command gap 423.7 ms; and
- 23 `heartbeat_expired` records.

The reported **zero rejected commands is not supported by this log schema**. The
analyzer searches for `accepted:false` on `command_received`, but the server never
writes that field. It logs `command_received` before validation and writes a
separate `command` record only after acceptance. There are 13 received requests
without a matching accepted `(session, sequence)`, all around stale-session/recovery
activity, plus one separately logged `nonlocal_control_rejected`. This diagnostic
correction does not alter the qualification result or redefine its gate; the run
already fails on duration, expiry, RTT maximum, and command-gap maximum.

## Causal-layer assessment

| Layer | Supporting observations | Contradicting observations | Missing telemetry / assessment |
|---|---|---|---|
| Browser timer throttling or lifecycle | Timer behavior can be affected by browser scheduling in general. | At 14:18 the 200 ms callbacks continued and logged `send_queued`; page was visible. No expiry coincided with hide/reload. | Scheduler-task latency trace. **Unlikely as the 14:18 initiating mechanism.** |
| Android screen/power/wake lock | Android may still schedule networking/radio independently of a screen wake lock. | Wake lock was last logged acquired, page visible, no release before 14:18 or the later cluster. | `adb` power, Doze, Wi-Fi, radio, and Chromium process traces. **Not demonstrated; not ruled out below the page API.** |
| Browser fetch queue / HTTP/TLS | Strong: single outstanding fetch blocked later sends; 710 ms successful RTT at 14:18; many later aborts/409s. | Browser code itself continued running; server handler was fast. | Request ID, response-ready/write-complete times, Resource Timing, socket/TLS trace. **Immediate application-level mechanism.** |
| Phone Wi-Fi underlay | Phone reported `type=wifi`; AXE75 is on the phone side. Browser online hints can remain true through transient loss/queueing. | No browser offline or connection-change event. | Association/BSSID/RSSI/retry/roam logs and phone packet capture. **Possible, unresolved.** |
| AXE75 forwarding or WAN/CGNAT | Phone path traversed the consumer router/WAN; its logs cannot expose transient forwarding latency. | Successful WAN DHCP ACKs bracket 14:18 and continue regularly. No lease failure/reboot then. | Per-client/AP counters, queue/latency probes, WAN loss, conntrack, packet capture. **Router-wide lease failure contradicted; transient client/forwarding issue unresolved.** |
| ZeroTier tunnel | Control used phone overlay address. Later cluster overlaps independent Tractor01-to-NAS ZeroTier probe failures. | No readiness transition at 14:18; another peer probe is not the same flow. | Peer path, direct/relay state, underlay endpoint changes, overlay ping/loss, packet capture. **Plausible for later cluster; unproven at 14:18.** |
| Tractor01 interface/network | Later ZeroTier probes failed; final snapshot alone cannot represent incident-time counters. | Default route and interfaces were up at collection; control server continued; bridge journal continued. | Continuous routes, carrier, gateway probes, interface counters, ZeroTier state. **Possible network-path contributor, not app-server failure.** |
| Python server processing/event loop | A response-write stall outside current timing instrumentation remains possible. | Handler processing at 14:18 was 0.38 ms; expiry-window p95 0.49 ms; ThreadingHTTPServer and service stayed alive. | Response serialization/write start/write completion and thread/CPU scheduling. **Core command handler strongly contradicted; response-write path unresolved.** |
| Bridge/Teensy | None as initiating evidence. | Bridge age 46.3 ms at anchor; failure-window bridge-age distribution matches baseline; Pause/neutral propagated promptly. | Wire-level timestamps would refine actuation latency. **Contradicted as initiating cause.** |

## AXE75 assessment

Both exports identify Archer AXE75 hardware 1.0.0, firmware
`1.5.6 Build 20260623 rel.56854`, LAN `192.168.10.1/24`, WAN
`100.81.44.171/10`, gateway `100.64.0.1`, and a 300-second DHCP lease. The 10:52
header reports five clients/four Wi-Fi; the 15:32 header reports four clients/three
Wi-Fi. Across unique entries, successful WAN ACK spacing is p50 154 s, range
154-155 s, consistent with half-lease renewal.

At 14:18 the nearest entries are successful DHCP ACK sequences at 14:16:49 and
14:19:23. The same normal cadence continues across the later expiry cluster. This
narrows away an obvious router reboot or WAN lease-renewal failure. It does not
address phone association, deauthentication, roaming, retries, signal, packet loss,
queueing, latency, forwarding, or ZeroTier behavior; the exports contain none of
those records. Absence of those client events is not evidence that the path was
healthy.

## Minimum next stationary instrumentation/bench test

1. Keep the 500 ms server and Teensy fail-closed limits unchanged. Run a 60-minute,
   engine-off, wheels-secured 5 Hz test matrix: ZeroTier over phone Wi-Fi, ZeroTier
   over phone cellular, and direct onboard-router LAN. Change only one path per run.
2. Add one request ID and four server timestamps to every command response:
   socket/handler receive, command decision complete, HTTP response write start,
   and write completion. Echo them to the browser and record them with
   `performance.now()`. This is the minimum change needed to separate server write,
   network return, and post-receipt browser delay.
3. Capture bounded rotating packet traces on Tractor01's ZeroTier interface and its
   selected physical underlay, with synchronized gateway, ZeroTier-peer, and NAS
   probes. Record ZeroTier direct-versus-relay path and endpoint changes.
4. During the same stationary runs, collect Pixel `adb` power/Doze, Wi-Fi
   association/RSSI/retry, connectivity, and Chromium process/network logs. Record
   wake-lock state on every flight event, not only transitions.
5. Extend the flight recorder with scheduled-tick time versus actual callback time,
   request ID, session identifier hash, fetch phase, and browser Resource Timing
   where available. Prevent duplicate shared-localStorage upload ambiguity by
   assigning a page-instance ID.
6. Add continuous Tractor01 route/interface/carrier/counter logging and independent
   probes to the local gateway, public underlay target, ZeroTier peer, and control
   client. The present single end-of-run network snapshot is insufficient.
7. Correct the analyzer's rejected-request accounting in a separate reviewed change;
   retain the existing qualification semantics and show both received and accepted
   counts explicitly.

## Reproduction

From the repository root on a machine with Python 3.11 or newer:

```powershell
python field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261022_phone_heartbeat_timeout_deep_dive/analyze_phone_timeout_telemetry.py `
  field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261021_133807_master_manual_phone/tractor01_master_manual_endurance_20261021_151604.tgz `
  'C:\Users\al532\Downloads\20261021\Archer-AXE75-Log-2026-10-21-105200.log' `
  'C:\Users\al532\Downloads\20261021\Archer-AXE75-Log-2026-10-21-153200.log'

python -m unittest discover `
  -s field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261022_phone_heartbeat_timeout_deep_dive `
  -p 'test_*.py'
```

Generated artifacts and their SHA-256 values are listed in
`artifact_hashes.sha256`. The script also records input and internal evidence hashes
in `metrics.json`.
