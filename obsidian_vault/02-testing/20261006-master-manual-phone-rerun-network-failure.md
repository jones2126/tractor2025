# 2026-10-06 Master Manual Phone Rerun and Network Failure

## Outcome

The September 23 master mission was rerun with the new Wi-Fi phone handheld.
The mission reached approximately 78% before repeated loss of the local phone
control heartbeat made continued Auto operation unsafe. The tractor was not
stranded mechanically: after the mission was abandoned, a temporary guarded
Manual service was placed on the onboard travel-router LAN and the tractor was
driven back to its parking position. The engine was then turned off.

The dominant failure was the `TractorField` local WLAN path
(`wlan0`, `192.168.10.0/24`), not steering, the JRK actuator, GPS heading, the
mission route, the Pi as a whole, or the remote ZeroTier path. The safety layers
did their job by expiring motion authority and returning the transmission to
neutral, but the dashboard could retain a stale green `AUTO ACTIVE` display
after its telemetry stream stopped. That stale presentation made diagnosis and
recovery more confusing than necessary.

Do not repeat a moving field test with the present network arrangement. The
onboard travel router's wired LAN (`eth0`, `192.168.8.20`) should be evaluated
and made into a deliberate, preconfigured primary phone-control path before the
next run. The improvised recovery service used today was successful evidence,
not a finished operating procedure.

## Evidence archive

- Tractor archive: `tractor01_master_manual_phone_rerun_20261006_131734.tgz`
- Development-machine copy:
  `C:\Users\al532\Downloads\tractor01_master_manual_phone_rerun_complete_20261006.tgz`
- SHA-256:
  `e863e39d4c7a2851209fe2aa6d3ad69781704026696b69bebfabdd925ea7e707`
- Archive manifest:
  `tractor01_master_manual_phone_rerun_complete_20261006.txt`
- The copied archive hash matches the tractor manifest.

The archive contains the controller CSV, pursuit CSV, four phone-control JSONL
logs, NetworkManager incident journal, normal and recovery control-service
journals, final network snapshot, service state, repository state, and file
hashes. It is sufficient for continued offline analysis. The archive contains
private temporary control-link material and should not be published.

Both large CSVs end with a short run of NUL padding after their final complete
record: 277 bytes in the mission CSV and 2,791 bytes in the pursuit CSV. The
last complete records remain readable, ending at about 12:13:51 local time just
before the reboot. Analysis tools should trim this terminal padding. The
collector should eventually make a consistent closed-file copy or sanitize the
copy so ordinary CSV readers do not interpret the padding as an extra record.

## Mission sequence

| Local time | Event and evidence |
|---|---|
| About 09:39 | Dashboard and mission controller started. |
| 09:41:07 | First guarded Auto selection was accepted. |
| 10:11:09 | The phone screen/page became hidden. The phone deliberately sent `page_hidden` Pause, and motion authority was removed. This was expected safety behavior. |
| 10:13:34 | Guarded Auto was accepted again. After the five-second freshness gate, the controller commanded motion. |
| 10:13:40–10:14:01 | JRK target changed from neutral `2836` to forward near `2300`; feedback reached about `2296` and held for about 21 seconds. Recorded ground speed stayed near zero, consistent with the engine already being off. This disproves the concern that no command reached the JRK after 10:13:34. |
| 10:14:01 | Phone heartbeat expired and the JRK was returned to neutral. |
| 10:14:05 | The phone-control service stopped and restarted cleanly. The two 10:14 ntfy messages were service-ready links, not proof of an external network outage. |
| 10:47:44 | Auto was deliberately resumed. The route advanced from about 61.2% to about 78%. |
| 10:55:44–10:56:47 | Four heartbeat expirations occurred, interleaved with deliberate guarded Auto re-arms. The final driving period ended fail-closed at about 10:56:47. |
| About 11:05 | The laptop dashboard still displayed green `AUTO ACTIVE`, approximately `1.00 / 0.85 m/s`, and JRK `2300 / 2297`. Those values were stale, frozen telemetry from the last live period; the controller had already paused and neutralized. |
| 11:21–11:24 | Additional guarded Auto attempts repeatedly expired. Reported round-trip times included about 0.9 s, 7.1 s, and 56.7 s. Continuing the mission was no longer reasonable. |
| About 12:14 | Tractor01 was rebooted. Guarded Manual attempts on `TractorField` still expired, including attempts with multi-second latency. |
| 12:51 onward | NetworkManager repeatedly acquired and lost `192.168.10.120`, reporting DHCP `no lease` and `ip-config-unavailable` cycles. |
| 13:05:37 | A temporary guarded control service was started for the onboard travel-router LAN, `192.168.8.0/24`; the normal `192.168.10.0/24` service was stopped. |
| 13:11:18 | Guarded Manual was selected through `https://192.168.8.20:8765`. |
| 13:13:54–13:14:21 | The tractor reached the parking position, returned to neutral, selected Pause, exercised the guarded software E-stop, and returned to Pause. |
| 13:16:45 | The temporary recovery service stopped cleanly. The tractor was parked with its engine off. |
| 13:17 | Final evidence archive was produced and later copied and hash-verified on the development machine. |

## What the network evidence says

### TractorField router evidence recovered after the run

The TP-Link router export `syslog-2026-10-06.tar.gz` was copied to the
development machine and verified with SHA-256
`690246e4c56976a1130cb6163bcb8d5b6ddc55adb5b4b0ee701e727dbfa5025f`.
The export identifies the unit as an Archer AXE75 hardware version 1.0,
marketed as AXE5400, running firmware `1.5.6 Build 20260623 rel.56854`.

The log contains a complete boot sequence with the unset clock initially using
the firmware-build date, followed by correct local time and network-ready events
at about 11:39. The router's web status at 13:49 showed only about two hours and
nine minutes of online time. Together these establish that the router restarted
or was power-cycled shortly before 11:39. The operator confirmed that this was a
deliberate troubleshooting restart intended to improve stability, not an
uncommanded router reset. Because the router log is volatile, the restart removed
the events surrounding the original mission failures before 11:00.

After the restart, the export shows successful WAN DHCP acknowledgements every
approximately 154 seconds for `100.81.44.171`. It contains no error, deauthentication,
crash, watchdog, or LAN DHCP-client entries. This shows the WAN lease was stable
after the restart, but it does not clear the LAN access point or explain
Tractor01's later `192.168.10.120` lease failures.

The router client page at about 13:43 identified the three remaining clients:

- field laptop: `192.168.10.48`, 5 GHz Wi-Fi;
- Pixel 6 phone: `192.168.10.232`, 5 GHz Wi-Fi; and
- `rtkbase-RPi3`: `192.168.10.114`, wired Ethernet.

### Independent RTK-base Pi 3 evidence

The Pi 3 evidence archive
`tractorfield_rpi3_incident_20261006_final.tgz` was copied to the development
machine and verified with SHA-256
`1ff5ac18ea4041882c540a8fcca739d0bf4c2f42c1cc311d0137623c764373b4`.
It contains the Pi's journal from 08:00 through collection, kernel log, interface
snapshot, and final gateway and Tractor01 pings.

The wired Pi 3 provides an independent view of the TractorField router:

- `eth0` retained `192.168.10.114` and recorded normal lease renewals at 08:52,
  09:46, and 10:42;
- it recorded no Ethernet carrier or DHCP interruption during the mission's
  repeated phone-heartbeat failures at 10:55–10:57;
- the RTK service continued producing broadcasts at approximately 5.6 Hz
  throughout that failure period;
- its Ethernet carrier dropped at 11:36:17 and returned at 11:38:10, exactly
  covering the operator's intentional router restart;
- it reacquired `192.168.10.114` at 11:38:16, six seconds after carrier return;
- the Pi's health helper then deliberately restarted ZeroTier. The outgoing
  ZeroTier process exited with signal 11 during that stop, but the replacement
  started immediately and the NAS was reachable again by 11:38:52. This is a
  separate service-shutdown defect to investigate, not the cause of the earlier
  phone-control loss;
- at 13:59, 100 wired pings to the router completed with zero loss and
  0.769 ms average latency; and
- the failed ping to `192.168.10.120` is expected because Tractor01 had already
  been powered down. It is not additional evidence of network failure.

The Pi reported no current undervoltage or throttling. Its `0x20000` historical
flag means ARM frequency capping had occurred at some unspecified point during
the ten-day uptime; it does not identify an event during this mission. Ethernet
counters showed zero errors and zero carrier errors. The cumulative 16,533 RX
drops cover the full ten-day boot and cannot be assigned to this incident.

This independent evidence argues against a complete LAN-router or RTK-base
failure at 10:55. It strengthens the conclusion that the safety-critical failure
was specific to Tractor01's `wlan0` association/data/DHCP path, or to the access
point behavior affecting that wireless client, while the wired LAN remained
operational.

### Failed mission-control path

The normal service allowed motion only from `192.168.10.0/24`. Tractor01 used
`wlan0` at `192.168.10.120` to join `TractorField`, and the phone used the same
SSID. Symptoms included:

- 20% packet loss from Tractor01 to the router, later 100% `Destination Host
  Unreachable`, and later 75% loss;
- a nominal signal near -60 dBm but link rates as low as 6.5 to 19.5 Mbit/s;
- local SCP and FileZilla transfers that timed out or fell to roughly 14
  KiB/s;
- repeated DHCP lease loss and reacquisition in the NetworkManager journal;
- phone command gaps long enough to trip both the server freshness limit and
  the Teensy watchdog; and
- high and sometimes extreme command round-trip times after the reboot.

Clearing the stored BSSID and disabling Wi-Fi power saving improved association
but did not make the path reliable. A stronger 2.4 GHz `TractorField` access
point was visible on channel 6. Forcing 2.4 GHz initially failed to obtain a
lease and later associated, but NetworkManager continued to record lease churn.
Changing bands alone therefore cannot be accepted as the fix.

### Successful parking-recovery path

The onboard travel router was already connected to Tractor01 by Ethernet:

- Tractor01 `eth0`: `192.168.8.20/24`
- travel-router gateway: `192.168.8.1`
- phone connected directly to that router's Wi-Fi
- control URL used by the phone: `https://192.168.8.20:8765`

The recovery log recorded 1,875 commands: 818 Manual, 1,023 Pause, no Auto, and
no phone freshness expirations. Average command round-trip time was 27.2 ms,
p95 was 59.4 ms, and the largest command gap was 425 ms. That kept the command
stream under the Teensy's 500 ms independent watchdog and allowed an uneventful
return to parking.

ZeroTier status fluctuated during this recovery while local travel-router
control stayed stable. Likewise, PuTTY and archive transfer over ZeroTier
worked while the `TractorField` local path was failing. ZeroTier or NAS
reachability must not be used as a proxy for the health of the safety-critical
phone motion link.

## Safety behavior that worked

- Hiding the phone page caused an immediate deliberate Pause.
- A stale phone heartbeat latched Pause, invalidated the old session, sent a
  stop burst, and required a new guarded mode selection.
- The Teensy independently expired stale commands.
- The JRK target and feedback returned to neutral after loss of authority.
- No automatic re-arm occurred after reconnection.
- The temporary parking recovery used Manual only; Auto was never selected.

These protections should remain. Reliability improvements must not weaken the
500 ms Teensy watchdog, guarded mode changes, session invalidation, or
fail-closed Pause behavior.

## Problems exposed

1. **The safety-critical path used the less reliable interface.** Phone motion
   authority depended on a Pi Wi-Fi client connection even though the onboard
   router already had a wired Ethernet connection to the Pi.
2. **The dashboard could look live while frozen.** It retained green Auto state
   and old actuator/speed values after dashboard telemetry stopped.
3. **The phone-side evidence is incomplete.** Server logs show received
   commands and client-reported timing but do not show every browser send
   attempt, transport error, visibility event, reconnect, or Wi-Fi transition.
4. **Readiness notifications were ambiguous.** ntfy published ZeroTier and
   hostname links but did not consistently identify the numeric, motion-authorized
   local URL and required SSID/subnet.
5. **Recovery depended on an improvised command.** The successful `.8` recovery
   path was not a preinstalled, tested, one-step recovery mode.
6. **Network readiness was a snapshot, not a soak test.** Good signal bars and a
   momentarily fresh heartbeat did not reveal the later packet loss and DHCP
   instability.

## Required work before another moving field run

### Priority 0 — prevent another near-stranding

1. Select one authoritative local motion network. Evaluate the onboard travel
   router's wired `192.168.8.0/24` LAN as the primary design and remove the
   dependence on Tractor01 joining a second Wi-Fi network for control.
2. Make the selected subnet, expected SSID, numeric control URL, interface, and
   route explicit in configuration and on both user interfaces. Do not rely on
   hostname discovery in the field.
3. Install a guarded, least-privilege parking-recovery service in advance. It
   must start in Pause, permit Manual only, use the independent watchdogs, and
   be tested engine-off before it is needed. Do not make an ad hoc transient
   service the standard procedure.
4. Add a hard dashboard stale state. After no fresh controller telemetry for a
   short fixed interval, cover the route/status area with `TELEMETRY LOST —
   TRACTOR PAUSED`, remove all green Auto indications, freeze values only as
   explicitly timestamped historical data, and prevent dashboard actions until
   fresh state is re-established.
5. Add a mandatory two-to-five-minute engine-off network soak. Fail preflight
   on any phone heartbeat expiration, DHCP/association change, unexpected
   interface or subnet, dashboard stream loss, or command gap approaching the
   500 ms Teensy watchdog. Establish final numerical thresholds from bench
   testing; the successful recovery's p95 RTT below 60 ms is a useful baseline.

### Priority 1 — make the next failure diagnosable

6. Add a bounded browser flight recorder using local storage. Record command
   send attempts, responses/errors, RTT, sequence number, online/offline,
   visibility, wake-lock state, network type when available, reconnects, and
   session changes. Upload it after reconnection and include it in the archive.
7. Add independent health channels for phone-to-control-server,
   laptop-to-dashboard, Pi-to-local-gateway, and remote maintenance. Display
   them separately instead of one generic network state.
8. Persist interface, route, DHCP lease, association/BSSID, frequency, link
   rates, signal, retries/drops, and gateway probes throughout a run. Add a
   small rotating packet capture around an incident if storage permits.
9. Make `COPY ALL MESSAGES` include timestamps, telemetry age, session ID or
   sequence, interface/subnet, and explicit stale/recovery transitions.
10. Update the archive collector to include the browser flight recorder,
    router/AP diagnostics when available, complete NetworkManager journal,
    control-service logs from every boot, and a collection manifest with hashes.
    It should also copy active logs consistently and remove only terminal NUL
    padding from CSV copies while preserving and hashing the originals.

### Priority 2 — field usability

11. Request and display a browser wake lock while the phone control page is in
    use, while preserving the existing immediate Pause on page hide or lock.
12. Publish an ntfy message that clearly labels the one motion-authorized local
    link, its expected SSID, and its subnet. Label ZeroTier as maintenance or
    status-only when it is not authorized for motion.
13. Persist safe Wi-Fi settings rather than applying them manually. Keep BSSID
    pinning disabled unless a specific tested design requires it. Do not declare
    either 2.4 or 5 GHz acceptable until sustained tests pass.
14. Continue the regulated router-power, loaded low-voltage alarm, and physical
    E-stop checks from [[20261005-field-stop-recovery-analysis]]. Network
    evidence dominates this incident, but power protection remains mandatory.
15. Keep automatic dual-F9P 5 Hz heading-profile configuration in the default
    launch process and retain the heading/start gates.

## Mission state preserved for later planning

The route was not completed. The best logs place the end of meaningful forward
progress around waypoint 15,453–15,470 of 19,825, approximately 78%, in or near
`garden left ring 2`. Any continuation mission must be derived and reviewed
from the logs rather than treating the stale dashboard screenshot as an exact
resume state. This continuation work should remain separate from the newer
boundary and coverage-map development.

## Related notes

- [[20261005-field-stop-recovery-analysis]]
- [[20261003-wifi-primary-control]]
- [[20260923-master-manual-field-session-summary]]
- [[20261006-next-field-test-network-reliability-prompt]]
