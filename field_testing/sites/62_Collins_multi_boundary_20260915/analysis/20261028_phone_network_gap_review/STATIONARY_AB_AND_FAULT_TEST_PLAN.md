# DRAFT - UNTESTED IN THE FIELD

# Stationary communications A/B and fault-test plan

This plan is motor-off, blades-off, wheels-chocked work. It does not authorize
field motion. Tractor01 is offline, so the commands below are preparation for a
later separately approved session.

## Fixed setup

- Tractor stationary in physical Pause; engine off; PTO/blades off; drive
  wheels chocked.
- Apply and independently verify the Heading-F9P moving-base profile in volatile
  RAM, restart `rtcm-server`, require it active, wait for heading reacquisition,
  then require `MISSION PREFLIGHT PASS` before collecting comparable data.
- Same phone, browser version, screen on, page visible, wake lock held, battery
  optimization settings, phone position, Pi load, TLS certificate, and 5 Hz
  desired command cadence. Hold each router's position and radio configuration
  fixed across all repetitions that use that router; Trials A and B necessarily
  use different Wi-Fi routers.
- Full qualification means ten runs total: five randomized 10-minute runs per
  transport (five A and five B). Include a 2-minute idle interval between runs
  and record the ten-run order before starting. The operator selected only one
  A run and one B run for the separately approved first test-day shakedown;
  review that evidence before deciding whether to request the remaining runs.

## A/B matrix

| ID | Phone path | Application transport | Purpose |
|---|---|---|---|
| A | ZeroTier | current HTTPS fetch | incident baseline |
| B | onboard GL router direct | current HTTPS fetch | candidate direct-path comparison |
| C | onboard GL router direct | persistent WebSocket candidate | isolates transaction churn |
| D | ZeroTier | persistent WebSocket candidate | separates overlay from application transport |

The direct route must use a numeric onboard-router address and verified active
interface. Do not infer the path from a hostname. Capture `ip route get`, active
interfaces, phone SSID/BSSID/band, and ZeroTier peer state at trial start, every
five seconds, on any path change, and at trial end.

A versus B compares the two complete candidate operating paths. Because it
changes both the Wi-Fi router and use of ZeroTier, it cannot alone assign a
difference specifically to ZeroTier. Layer attribution must use the synchronized
phone, router, ZeroTier, server, bridge, and controller measurements below. A
later same-Wi-Fi control run using both a direct address and a ZeroTier address
may be added only after its routing is verified; it is not part of the initial
two-run shakedown.

## Measurements by leg

1. **Phone scheduling:** scheduled deadline, actual start, scheduler delay,
   visibility, wake-lock state, in-flight count, coalesced state age, request ID,
   explicit abort initiator, and response-complete time.
2. **Local Wi-Fi/router:** phone association/BSSID, band/channel, RSSI,
   negotiated rate, retries, roaming, gateway RTT/loss, router CPU/power events,
   and Ethernet error counters.
3. **ZeroTier:** peer ID, direct/relay status, physical endpoint, latency, path
   changes, and link-local route. A NAS reachability result is not a substitute.
4. **Server:** receive, validation, acceptance, response-start/finish,
   processing time, HTTP connection identity/version, TLS handshakes, accepted
   gap, rejection reason, and session generation.
5. **Pi-to-Teensy bridge:** bridge packet age, serial write/ack age, command
   sequence, Teensy command age, watchdog state, firmware, serial timeouts and
   discarded bytes.
6. **Pure Pursuit:** controller loop lateness, `cmd_vel` production and bridge
   receipt ages, exact wait/stop reason, route progress, cross-track error, GPS
   and heading gate status.
7. **ntfy:** event ID, queued time, attempt count, HTTP acknowledgement time,
   delivery-observed time when available, and pending/failed state. ntfy never
   affects the safety decision.

Use a rotating packet capture capped at 100 MiB total and 10 files. Protect the
capture because URLs/headers may contain credentials. Collect copies; do not
delete source logs during the test.

## Acceptance criteria

For each 10-minute stationary trial report p50, p95, p99, p99.9 and maximum for
RTT, corrected network RTT, scheduler delay, server processing, accepted gap,
bridge age, and Pure Pursuit command age; plus expiries, abort reasons, session
claims/invalidations, retransmissions, path changes, and coalescing.

An application/path combination passes the initial stationary gate only if all
five trials have:

- zero accepted-command gaps at or above 500 ms;
- p99.9 accepted gap no more than 350 ms (150 ms safety margin);
- zero freshness expiries, unexplained aborts, and session invalidations;
- zero Pi-to-Teensy or Pure Pursuit watchdog violations;
- known direct/relay state for every ZeroTier sample;
- every injected local safety fault producing Pause for the exact expected
  reason; and
- ntfy state visible as queued, acknowledged, or failed without influencing
  motion authority.

## Automated fault matrix

Run first against the dry-run/simulated bridge only. Never inject faults into a
live moving mission.

| Fault | Expected result |
|---|---|
| 250, 400, 550, 1,000 ms phone-path delay | attribution to network leg; current production policy pauses at freshness loss |
| 1%, 5%, 20%, 100% loss bursts | no stale replay; newest state coalesced; exact loss reason |
| response-only loss after server acceptance | request ID proves acceptance; no ambiguous abort |
| request reordering/duplicate delivery | stale sequence rejected and counted; no authority extension |
| browser timer stall and page hide | scheduler delay/visibility reported; fail-closed behavior retained |
| server 250/750 ms processing stall | server processing separated from network RTT |
| bridge telemetry stale / firmware mismatch | immediate Pause; degraded guard denied |
| Pure Pursuit `cmd_vel` stale | immediate local watchdog Pause; degraded guard denied |
| GPS stale, RTK Float, heading invalid | immediate Pause; degraded guard denied |
| physical Pause or E-stop | immediate local authority; cannot be overridden |
| repeated reconnects during one incident | original time/distance origin retained |
| time reaches 30 s or distance reaches 30 m | Pause required at the first limit |
| ntfy endpoint unavailable | notice remains pending/failed; safety behavior unchanged |

## Three separate approval gates

1. **Approval A — deploy instrumentation and perform stationary motor-off A/B.**
   No motion and degraded mode remains disabled.
2. **Approval B — supervised blades-off bounded route test.** Requested only
   after Approval A evidence is reviewed and local-health integration is proven.
3. **Approval C — rerun the known valid mission.** Requested only after the
   bounded route test is reviewed. Begin stationary in Pause with blades off;
   perform Heading-F9P volatile-RAM configuration/readback, restart and verify
   `rtcm-server`, wait for heading, and require `MISSION PREFLIGHT PASS`. Use
   Manual only to reach the start, return to Pause, start the controller in
   Pause, wait for the ready gate, then select Auto. On completion select Pause
   before Manual, park, stop the engine, collect and hash telemetry, and record
   router logs separately.

