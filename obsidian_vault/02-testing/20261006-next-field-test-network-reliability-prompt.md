# 2026-10-06 Next Field Test Network-Reliability Prompt

> [!note] Revised direction on 2026-10-19
> This file preserves the immediate post-incident recommendation. Al's revised
> goal is to evaluate ZeroTier as the primary path while retaining the onboard
> router for Manual recovery. Use
> [[20261019-phone-control-recovery-zerotier-primary-prompt]] for the current
> task rather than silently rewriting this historical handoff.

Use the prompt below to begin a focused new chat before the next moving field
test. It intentionally keeps reliability remediation separate from the newer
boundary and coverage-map work.

## Ready-to-paste prompt

```text
I need you to prepare Tractor01 for another field test after the 2026-10-06
master-manual phone mission suffered repeated local network failures and nearly
left the tractor stranded.

Start by reading these repo notes completely:

- obsidian_vault/02-testing/20261006-master-manual-phone-rerun-network-failure.md
- obsidian_vault/02-testing/20261005-field-stop-recovery-analysis.md
- obsidian_vault/02-testing/20261003-wifi-primary-control.md

The verified evidence archive is on my development machine at:

C:\Users\al532\Downloads\tractor01_master_manual_phone_rerun_complete_20261006.tgz

Its SHA-256 is:
e863e39d4c7a2851209fe2aa6d3ad69781704026696b69bebfabdd925ea7e707

Important facts already established:

- The mission reached about 78% before repeated phone-heartbeat expirations.
- The normal safety-critical path used TractorField through Tractor01 wlan0 on
  192.168.10.0/24. It showed severe packet loss, very low link rates, local SCP
  timeouts, multi-second command latency, and repeated DHCP lease failures.
- Steering, JRK neutralization, GPS heading, and the mission controller's
  fail-closed logic were not the primary cause.
- The laptop dashboard could retain a stale green AUTO ACTIVE display after
  its telemetry stream stopped. This must be fixed.
- Tractor01 is wired to its onboard travel router on eth0 at 192.168.8.20.
  A temporary Manual-only recovery service on 192.168.8.0/24 delivered 1,875
  commands with zero freshness expirations, 27.2 ms mean RTT, 59.4 ms p95 RTT,
  and a 425 ms maximum command gap. It allowed a safe return to parking.
- ZeroTier continued to work while the TractorField local path failed, and it
  fluctuated while the successful local recovery remained stable. It is a
  maintenance path, not proof that the motion-control link is healthy.
- The independently logged wired RTK-base Pi 3 retained its DHCP lease and
  continued broadcasting RTCM at about 5.6 Hz during the 10:55–10:57 failures.
  Its only Ethernet carrier loss matched an intentional router restart at
  11:36–11:38. This further isolates the incident to Tractor01's wireless-client
  path or access-point behavior affecting that client, not the entire LAN.
- The 500 ms Teensy watchdog, server freshness Pause, guarded mode selections,
  session invalidation, page-hidden Pause, and no automatic re-arm must remain.
- Never copy temporary operator keys into documentation or chat output.

Work from the repository and inspect the current dirty worktree before editing.
Do not modify or discard unrelated boundary/coverage-map work. Do not operate
the tractor or assume a field test is authorized. Hardware validation must be
presented as engine-off, wheels-raised where appropriate, and then a separately
approved low-speed field step.

Please do the following:

1. Trace the complete phone-command, control-server, bridge, dashboard, service,
   and network configuration paths in the current code. Confirm every subnet,
   bind/listen address, source-address gate, ntfy link, and systemd dependency.
2. Propose and implement a coherent primary-control architecture using the
   onboard travel router's wired 192.168.8.0/24 LAN unless repository evidence
   reveals a concrete safety reason not to. Avoid a permissive multi-network
   shortcut; keep motion authority deliberately limited and explicit.
3. Add a preinstalled guarded parking-recovery mode that starts in Pause,
   permits Manual but never Auto, retains both watchdog layers, and cannot be
   confused with the normal mission service.
4. Make the dashboard fail visibly closed when telemetry is stale. Remove or
   cover green Auto status, show telemetry age and last-good timestamp, and
   require confirmed fresh state before actions can resume.
5. Add a phone-side bounded flight recorder for send attempts, failures,
   response timing, sequence numbers, online/offline, page visibility,
   wake-lock state, reconnections, and session changes. Preserve and upload it
   after reconnection.
6. Add separate live health indicators and logs for phone-to-server,
   laptop-to-dashboard, Pi-to-local-gateway, and remote maintenance. Do not
   collapse them into one network-good flag.
7. Add a mandatory engine-off network soak gate and recommend justified pass/
   fail thresholds. It must catch heartbeat expiry, DHCP/association churn,
   telemetry staleness, wrong SSID/subnet/interface, and command gaps that
   approach the Teensy's 500 ms watchdog.
8. Update ntfy so it publishes and labels the numeric motion-authorized local
   URL, expected SSID, and subnet. Clearly label ZeroTier links as maintenance
   or status-only when they cannot carry motion authority.
9. Expand the end-of-run collector to include the browser flight recorder,
   NetworkManager and control-service journals across restarts, interface and
   route history, gateway probe results, router/AP information when available,
   and hashes. Preserve the existing one-command collection workflow. The two
   large 2026-10-06 CSVs have short terminal NUL-padded tails, so make active-log
   snapshots consistent and produce parser-safe copies without altering the
   archived originals.
10. Keep the automatic dual-F9P 5 Hz heading-profile setup, all existing mission
    start gates, physical E-stop check, regulated-router-power check, loaded
    low-voltage alarm check, and COPY ALL MESSAGES recovery guidance.
11. Add focused automated tests for stale dashboard state, source-subnet
    rejection, restart/session behavior, Manual-only recovery, no automatic
    re-arm, phone flight-recorder recovery, and the network soak gate.
12. Produce a step-by-step next-field checklist with explicit stop/go gates and
    a short parking-recovery card. The first validation stages must be bench,
    engine-off, and wheels-raised. Do not recommend a moving field run until
    all earlier gates pass.

Lead with your evidence-backed architecture recommendation. Then implement the
safe changes, run the available tests, document exactly what changed, and list
anything that still requires physical verification. Do not weaken a safety
interlock merely to improve availability.
```

## Related incident record

See [[20261006-master-manual-phone-rerun-network-failure]].
