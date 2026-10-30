# October 28 Network Gaps — Deep Root-Cause Plan, Resilience Design, and Fresh-Chat Prompt

## Purpose

This document provides focused context for diagnosing the October 28, 2026 phone-control network gaps and designing a safer long-term response. It includes the operator's requested investigation of an ntfy-notification-based degraded-communications mode and improved dashboard diagnostics.

This engineering investigation and design brief must lead to another field test in the same area includeing conducting an autonomous field test, blades off, using the same mission.

## Operator requirements and observations

- Numerous network-related safety stops disrupted the mission.
- The operator is typically within 100 ft of the tractor and visually confirms that no obstacles are in its planned path.
- The operator would prefer prompt ntfy notices at the start of the event and 15 second intervals plus detailed dashboard messages instead of the current logic which is an immediate mission pause for every brief phone-network interruption.
- The operator is willing to consider up to 45 seconds of communications degradation only while all other safety checks remain healthy.
- The dashboard Messages tab should explain where latency is accumulating so the operator can diagnose the link during a mission.
- The phone recorded 18,587 `send_queued` events, which requires explanation and correction.

## Incident data

Mission command:

```text
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/mission_dashboard_master_manual_phone_20261006.py
```

Downloaded logs:

```text
C:\Users\al532\Downloads\20261028 - logs for analysis\master_manual_phone_20261028_153941.csv
C:\Users\al532\Downloads\20261028 - logs for analysis\pursuit_log_20261028_153944.csv
C:\Users\al532\Downloads\20261028 - logs for analysis\phone_flight_recorder_20261028_144314.jsonl
C:\Users\al532\Downloads\20261028 - logs for analysis\wifi_primary_control_20261028_144314.jsonl
```

Relevant code and service configuration include:

```text
tractor_rpi/pure-pursuit/mission_dashboard_20260910.py
tractor_rpi/pure-pursuit/pure_pursuit_controller_20261022.py
tractor_rpi/setup/tractor-wifi-control.service
tractor_rpi/testing/webrtc/wifi_primary_control_20261022.py
```

Confirm exact current paths before editing because filenames may have evolved.

## Evidence summary

The mission ran for approximately 4,589 seconds, or 76.5 minutes.

### Server/control events

- 93 `phone_freshness_expired` events;
- 75 entries into AUTO link-wait;
- 65 completed AUTO recoveries;
- 65 link losses during recovery stabilization;
- 178 control-session invalidations.

### Pure Pursuit observations

- 106 stale-heartbeat episodes;
- approximately 100.1 seconds total stale-heartbeat time;
- median stale episode approximately 0.35 seconds;
- longest stale episode approximately 8.81 seconds;
- 107 stabilization episodes totaling approximately 412.3 seconds, or 6.9 minutes.

### Accepted-command timing

```text
median gap:  220.2 ms
p95 gap:     315.0 ms
p99 gap:     517.2 ms
maximum gap: 17,184.2 ms
gaps >500 ms: 204
```

### Phone-reported request round-trip time

```text
median RTT:  206.8 ms
p95 RTT:     299.1 ms
p99 RTT:     447.3 ms
maximum RTT: 982.5 ms
RTT samples >500 ms: 115
```

### Phone events during the mission window

```text
send attempts:          16,918
send_ok:                16,822
send_failed:               108
send_queued:            18,587
ack_watchdog_expired:       34
session_invalidated:        69
```

The phone log may contain buffered or duplicated event records, so the server log should remain authoritative for accepted commands and session state.

Of the 108 send failures:

- 71 were HTTP 409 responses stating that the control session was not current;
- 37 reported that the request signal was aborted without a reason.

The 409 responses are probably consequences of heartbeat expiry, session invalidation, and reclaim attempts, rather than the original network fault.

## Leading explanation for 18,587 queued sends

The phone attempts to send about every 200 ms, or 5 Hz. The median request round-trip time was approximately 206.8 ms, and the median accepted-command gap was approximately 220.2 ms.

The client uses a single-flight pattern: while one request is active, `sending` is true; if the next timer fires before that request completes, the client records `send_queued` and sets a queued-send flag. Because median RTT is already slightly longer than the nominal 200 ms period, overlapping timer firings are expected during at least roughly half the requests. Normal jitter makes this worse.

This explains how queued-send events can outnumber successful sends without requiring 18,587 network packets to be waiting in an operating-system queue. The counter appears to record timer attempts that arrived while the previous request was still in flight, not a literal queue depth.

Likely feedback cycle:

```text
200 ms timer cadence
    -> previous HTTPS fetch often still active
    -> send is marked queued
    -> latency jitter delays accepted heartbeat beyond 500 ms
    -> server expires freshness and invalidates session
    -> phone reclaims/retries
    -> old or racing requests receive HTTP 409
    -> recovery and stabilization add more churn
```

This hypothesis must be confirmed by inspecting the current phone JavaScript and correlating request sequence numbers with server acceptance records.

## Important measurement limitation

The existing estimated phone-to-server one-way latency is not trustworthy. Phone and server wall clocks were offset, producing impossible negative values (median approximately -323 ms). Use round-trip timing or synchronized monotonic exchanges. Do not infer one-way network latency until clock offset and drift are measured.

## Likely contributing causes to investigate

The evidence presently points to a combination of architecture, latency, and jitter rather than simple total packet loss.

1. **Request cadence too close to observed RTT.** A new 5 Hz HTTPS request is scheduled about as quickly as the median request can finish.
2. **Independent request/response transactions.** Repeated fetches can incur browser scheduling, TCP/TLS, connection-pool, and overlay-network delays even when connections are reused.
3. **ZeroTier in the real-time control path.** The phone address was `192.168.193.206` under the `zerotier-primary` policy. Determine whether peers were communicating directly or through a relay and whether the physical phone path was Wi-Fi, cellular, or transitioning between them.
4. **Phone power and browser lifecycle behavior.** Android battery optimization, the ZeroTier/VPN app, page visibility, screen state, wake lock, or browser scheduling may suspend or delay requests.
5. **Wi-Fi roaming or mobile-data handoff.** A phone may briefly prefer another network, roam between access points/bands, or change the underlying route while the overlay remains addressed the same way.
6. **Abort behavior.** The 37 unexplained aborted requests may come from an AbortController timeout, page lifecycle event, network transition, browser behavior, or application recovery code.
7. **Session-recovery races.** In-flight requests from an invalidated session may arrive after a new claim and produce 409 responses and misleading secondary noise.
8. **Router/radio conditions.** RSSI, retries, congestion, band selection, interference, router load, DHCP events, Ethernet errors, or power instability require direct measurement.

The browser's `effective_type` value such as `4g` is only a performance category; it is not proof that the phone was using cellular service.

## Deep root-cause investigation plan

### 1. Establish a common event timeline

Correlate phone, Wi-Fi-control server, Pure Pursuit, mission field logger, and dashboard events using explicit sequence numbers and monotonic times.

Add a lightweight clock-offset exchange so reports can estimate phone/server offset and drift. Preserve raw local timestamps, server receive timestamps, and round-trip timing. Do not overwrite raw evidence with adjusted time.

### 2. Instrument each request stage

For every phone command, record:

- monotonically increasing request sequence;
- scheduled time, actual start time, and scheduler delay;
- time spent waiting because another send is active;
- fetch start, response headers, response completion, and failure/abort time;
- timeout/abort initiator and reason;
- HTTP status;
- session identifier/revision in a non-secret form;
- server receive, validation, acceptance, response-start, and response-finish times;
- last accepted command age;
- page visibility, screen/wake-lock state, and navigator connectivity hints.

Replace the ambiguous cumulative `send_queued` signal with at least:

- current queue/coalesced-pending depth;
- cumulative timer collisions;
- longest single-flight duration;
- oldest unsent command age;
- commands coalesced or superseded;
- number of in-flight requests.

For control semantics, stale queued commands should normally be superseded by the newest desired state rather than replayed later. Preserve immediate delivery semantics for PAUSE/e-stop paths.

### 3. Measure each network leg independently

During tests, collect:

- phone to local GL router/gateway latency and loss;
- phone to the tractor's direct local address;
- phone to the tractor through ZeroTier;
- tractor to its gateway;
- ZeroTier peer status, direct-versus-relayed path, and path changes;
- active interface and route on both ends;
- router station RSSI, negotiated rate, retransmissions, channel, band, and roaming events;
- Ethernet link/error counters between router and Tractor01;
- CPU, memory, temperature, and service scheduling delays on the tractor Pi.

Use a bounded rotating packet capture on Tractor01 during a stationary diagnostic run to identify TCP retransmissions, connection churn, TLS handshakes, long server-response delays, and ZeroTier UDP path changes. Establish storage limits and remove or protect sensitive capture data after analysis.

### 4. Inspect phone/browser behavior

Determine whether:

- HTTPS connections are actually reused and which HTTP version is used;
- the JavaScript timer fires late or bunches callbacks;
- the page loses visibility or wake lock;
- Android battery optimization affects the browser or ZeroTier app;
- Wi-Fi remains associated during gaps;
- mobile-data preference changes the underlying path;
- `signal is aborted without reason` maps to application timeouts, navigation, recovery code, or OS/browser cancellation.

### 5. Run controlled A/B tests

Use identical, timed, trials:

1. current HTTPS/fetch over ZeroTier;
2. current HTTPS/fetch directly through the onboard GL router;
3. a persistent local WebSocket design;
4. optionally, the same WebSocket over ZeroTier for comparison.

For each trial, report p50, p95, p99, and p99.9 RTT and accepted-command gaps, maximum gap, expiry count, session churn, retransmissions, timer collisions, and queue/coalescing behavior. The pass criterion should be justified against the safety threshold, not merely described as “better.”

## Recommended long-term communications architecture

### Keep the real-time control path local

Prefer a direct onboard-router mission path:

```text
phone -> qualified GL router Wi-Fi -> Ethernet -> Tractor01
```

Use a stable direct local URL/address and a mission-qualified phone-facing band, preferably 5 GHz when field measurements support its range and reliability. Avoid making the Pi's own Wi-Fi radio a mandatory part of the sub-second motion-authority heartbeat.

Define and test an `onboard-router-mission` policy that can support AUTO only after it passes the safety and reliability qualification. ZeroTier remains valuable for maintenance, remote access, monitoring, and log transfer.  The Web mission dashboard, which will be connected via ZeroTier can be an additional means for pausing or stopping the mission.

### Use a persistent, observable control connection

After establishing the local network path, evaluate a persistent authenticated WebSocket instead of independent HTTPS fetches every 200 ms. A persistent channel can reduce scheduling and transaction overhead and make sequence, acknowledgement, and connection-state semantics clearer.

The design must still have:

- explicit sequence numbers and freshness;
- newest-state coalescing rather than stale-command replay;
- immediate PAUSE handling;
- bounded reconnect behavior;
- server-side authorization/session ownership;
- a local low-level watchdog independent of the phone UI.

## Proposed ntfy and communications-degraded behavior

The operator's request should be implemented as a separately reviewed state-machine change, not by changing the existing 500 ms value to 15 seconds.

### Essential safety distinction

Separate two concepts:

1. **Local motion-control health:** the onboard navigation controller is continuously producing fresh commands and the Teensy/bridge, steering, transmission, GPS/heading, path/boundary checks, and physical radio safety controls remain healthy.
2. **In the field supervisory link health:** the phone dashboard control page acknowledgement or supervisory lease is delayed.  For a window of time, this can be a notification event.

The low-level onboard command watchdog and physical PAUSE/e-stop must remain immediate and independent. A lost or stale local motion command must never be treated like a mere notification event.

### Candidate state machine for investigation

When only the remote phone link becomes stale, and every explicitly enumerated local safety gate remains healthy, consider entering `COMMUNICATIONS DEGRADED` rather than immediately pausing.

In that state:

- immediately record a high-priority local event;
- show a persistent dashboard warning and detailed Messages-tab entry;
- enqueue an ntfy alert through a nonblocking worker;
- use the mission dashboard voice commands to notify the operator;
- continue only under the already active onboard plan and within a separately approved time/distance envelope;
- pause immediately if any other safety signal degrades;
- pause when an absolute communications-degraded limit is reached;
- prevent repeated short reconnections from resetting the limit indefinitely.

The operator proposes 30 seconds as the absolute maximum. At 1.0 m/s, 30 seconds is about 30 m or 90 ft of travel. Being within 100 ft does not by itself guarantee that a person can intervene before the tractor reaches a hazard. But it should be noted the tractor is otherwise healthy and following a defined path. The final envelope should therefore be derived from speed, stopping distance, route clearance, boundary margin, available physical e-stop/radio response, and test evidence. 

### ntfy limitations

ntfy delivery may depend on the same phone, Wi-Fi, overlay, or internet path. It must not be treated as proof that the operator received the warning.

Record separate events for:

- notice created;
- notice queued locally;
- delivery request attempted;
- ntfy server acknowledged, if such acknowledgement is available;
- phone receipt, if such acknowledgement is available.

If the network is unavailable, retain a bounded local queue and send the notice when connectivity returns. The ntfy worker must never block the control loop.

### Safety gates to define explicitly

Before any degraded-link continuation can be considered, define and implement affirmative checks for at least:

- fresh local Pure Pursuit output;
- fresh Pi-to-Teensy bridge communication;
- valid RTK/GPS and heading quality required by the mission;
- steering tracking within limits;
- transmission response within limits;
- route, geofence, and boundary margin;
- physical handheld radio/e-stop health and immediate PAUSE authority;
- maximum speed appropriate for the degraded state;
- no other active safety stop or controller fault;
- bounded elapsed time and distance since phone-link loss.


## Dashboard Messages-tab improvements

Add structured, human-readable events that identify which layer is slow or stale. If a new page with a graphic showing the components and blinking red idicators is possible that would aid the operator to know which link is the cause.  Each network incident should show:

- local and server timestamps plus clock-offset quality;
- request/command sequence;
- selected handheld mode and effective controller mode;
- phone scheduler delay;
- request RTT;
- time since last server-accepted command;
- server processing time;
- session invalidation, reclaim, and revision changes;
- bridge/Teensy status age;
- Pure Pursuit command age;
- active phone network hints, page visibility, and wake-lock state;
- direct local path health;
- ZeroTier direct/relay state and path changes when available;
- exact safety-stop source and threshold;
- `ntfy queued`, `ntfy server acknowledged`, or `notification delivery unknown`;
- start, continuation, recovery, and final duration of each gap.

Use clear categories such as:

```text
PHONE SCHEDULER DELAY
PHONE <-> SERVER RTT
SERVER SESSION REJECTED
ZEROTIER PATH CHANGED
LOCAL ROUTER LINK DEGRADED
PI <-> TEENSY BRIDGE STALE
PURE PURSUIT COMMAND STALE
COMMUNICATIONS DEGRADED
COMMUNICATIONS RECOVERED
SAFETY PAUSE — ABSOLUTE LIMIT REACHED
```

The dashboard must not claim that latency came from a particular layer unless that layer was measured.

## Staged verification plan

1. Analyze and instrument the existing system.
2. Reproduce queueing and expiry behavior in a motor-off, stationary test.
3. Complete direct-local versus ZeroTier A/B tests.
4. Fix request scheduling/coalescing and session-recovery races; retest stationary.
5. Prototype Messages-tab diagnostics and nonblocking ntfy delivery; induce network loss while stationary.
6. Design the communications-degraded state and complete a written hazard review.
7. Test the state machine with simulated time and network faults.
8. Bench test with engine off before initiating the next live field test.
9. Only after review and authorization, re-run the previous mission and capture results.

## Completion criteria

The investigation is complete when:

- each major October 28 gap can be assigned to a measured layer or explicitly labeled unknown;
- the meaning and cause of `send_queued` are confirmed in code and controlled testing;
- unexplained aborts and session-invalidating races are understood or bounded;
- local-direct and ZeroTier paths have documented reliability measurements with appropriate uses understood;
- the Messages tab identifies the failing layer using measured evidence;
- ntfy is asynchronous, but never treated as a safety acknowledgement;
- the proposed degraded-communications state has explicit gates, time and distance bounds, a non-resetting cumulative rule;
- local motion watchdogs and physical PAUSE/e-stop remain independent and immediate;
- all changes pass stationary tests before any moving trial.
