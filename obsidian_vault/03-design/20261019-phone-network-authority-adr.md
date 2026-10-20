# ADR: Phone network authority after the October 6 incident

Date: 2026-10-19
Status: **Code-ready design; not qualified on Tractor01**

## Decision

Motion authority is selected once when the HTTPS control service starts. One
named policy owns one IPv4 source network. Runtime requests cannot add a second
network, and changing the selected path requires a service restart. Startup and
shutdown both send Pause, and a restart destroys the old in-memory session.

The implemented policies are:

| Policy | Source network | Manual | Auto | Present status |
|---|---:|---:|---:|---|
| `zerotier-primary` | `192.168.193.0/24` | yes | yes | **Production target**; real phone/home endpoint verified, Tractor01 acceptance deferred |
| `tractorfield-mission` | `192.168.10.0/24` | yes | yes | Historical current path; failed October 6 reliability qualification |
| `onboard-router-recovery` | `192.168.8.0/24` | yes | **no** | Recovery candidate supported by October 6 evidence; still needs installed-service and hardware acceptance tests |

The shipped service now selects `zerotier-primary`. This records the intended
production architecture; it is not a claim that Tractor01 has been deployed or
physically accepted. A custom subnet override still cannot smuggle ZeroTier
authority into another named production policy. Phone-hotspot motion authority
remains absent.

## Current end-to-end paths

### Phone motion commands

1. The phone loads the keyed HTTPS page on TCP 8765.
2. `POST /api/claim` creates a random in-memory session in Pause.
3. The visible page sends sequential `POST /api/command` requests at 5 Hz.
4. The server checks the bearer key, client source subnet, owner/session,
   increasing sequence, guarded mode-change reason, expected fresh firmware,
   and selected policy.
5. Accepted commands are published as JSON to loopback UDP 6004.
6. `teensy_serial_bridge_20261002.py`, inheriting the dated 20260728 bridge,
   converts the phone command to the `WIFI` serial record at 460800 baud.
7. The Wi-Fi-primary Teensy firmware applies Manual demands or permits mission
   `CMD` motion only in guarded Auto. The Teensy independently expires commands
   after 500 ms.

The server and Teensy both use a 500 ms freshness target. The Teensy remains
the independent final authority if Linux, the bridge, or the network server
stalls; the server latch invalidates the web session and prevents later
automatic resumption.

### Mission Auto commands

The pure-pursuit controller sends navigation `CMD` data through UDP 6004 and
the serial bridge. The Teensy applies it only while the phone-selected mode is
Auto. Reconnecting a phone creates a new session in Pause and cannot restore
Auto automatically.

### Telemetry and dashboard

Teensy/bridge state is broadcast on UDP 6003. The mission controller publishes
dashboard telemetry on UDP 6012 and the dedicated dashboard GPS feed uses UDP
6013. The dashboard now replaces any green Auto presentation when its HTTP poll
fails and treats controller telemetry at least one second old as stale. It
shows the browser's dashboard access path, reported phone mode, recovery
lockout, telemetry age, and last fresh controller time.

### Notifications and services

`tractor-wifi-control.service` listens on all interfaces but claim/command
authorization is restricted to the one selected policy network. Its normal
unit selects `tractorfield-mission`. ntfy currently publishes local-control and
ZeroTier status links. The ZeroTier readiness probe checks the overlay address
and TCP reachability to the NAS; it is not a motion-link health test.

## ZeroTier decision and residual limitation

The HTTPS server normally sees the phone's ZeroTier overlay address. That
address can remain constant while the underlay changes between Wi-Fi and
cellular. The browser Network Information API provides only optional hints and
cannot be treated as authoritative interface attestation. The phone now forces
local Pause and discards its session on an observable connection-hint change,
and its independent acknowledgement watchdog changes the display to
`RECONNECTING · TRACTOR PAUSED` after 500 ms without an acknowledged command.
An entirely seamless underlay change that produces neither a command gap nor a
browser hint remains indistinguishable to the web application. In that case
the encrypted ZeroTier session and command acknowledgements continue without
an observed service interruption.

The 2026-10-19 home test verified the real Pixel 6 against a dry-run endpoint:
Wi-Fi-backed and 5G-backed ZeroTier, guarded zero-output Manual, page-hide
recovery, forced Airplane-mode loss, a 506 ms phone acknowledgement timeout,
automatic ZeroTier recovery over 5G, and fresh-session recovery in Pause. It
also exposed and corrected a stale `MANUAL LIVE` display. These results qualify
the browser/overlay behavior only; they do not qualify Tractor01, the serial
bridge, or physical watchdog and actuator behavior.

## Provisional measurable requirements

These are bench qualification gates, not claims already met:

| Measurement | Provisional gate |
|---|---|
| Phone command rate | 5 Hz nominal (200 ms period) |
| Accepted-command gap while armed | warn at 300 ms; fail the soak at 400 ms; the independent 500 ms Teensy timeout is unchanged |
| Any watchdog/freshness expiry | fail; session must be invalidated and a new guarded choice required |
| Command RTT | p95 no greater than 100 ms and observed maximum no greater than 250 ms during the qualification soak |
| Sequence continuity | no unexplained duplicate, reversal, or accepted stale sequence; loss is judged primarily by command inter-arrival gap |
| Dashboard controller telemetry | hard stale at 1.0 s; no green Auto or actionable controls while stale/disconnected |
| Address, route, interface, or observable connection change | force Pause and invalidate the session before requalification |
| Stability after a change | at least 120 seconds engine-off with no expiry/change, followed by the existing five-second controller gate and a new guarded operator selection |
| Page hidden or phone locked | page attempts Pause immediately; physical acceptance must prove neutral/no steering output no later than the unchanged 500 ms Teensy bound |

Phone and server wall clocks are not assumed synchronized. Safety decisions use
server monotonic inter-arrival age, sequence continuity, client-observed RTT,
and Teensy command age. Phone-to-server wall-clock subtraction is diagnostic
only.

## Threat model requirements

Qualification must address a lost phone, disclosure of the keyed URL through
ntfy or browser history, an unauthorized ZeroTier member, request replay,
multiple or restored tabs, stale sessions, service restart, and access to the
read-only state endpoint. Key rotation and a new guarded selection are required
after suspected credential disclosure.

## Deferred acceptance evidence

The following require Tractor01 or field hardware and are not verified by the
current Windows/home tests:

- actual Teensy 500 ms timeout and serial-bridge behavior;
- JRK target, feedback, neutralization, and engine/wheel response;
- real Tractor01 routes, addresses, DHCP, services, and interface transitions;
- onboard-router and phone-hotspot addressing;
- Wi-Fi-backed versus cellular-backed ZeroTier performance to Tractor01;
- RTK/controller/dashboard integration; and
- engine-off, wheels-raised, and separately approved short moving tests.
