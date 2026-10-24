# Tractor01 eth0-primary routing field context

Date prepared: 2026-10-24

Intended field use: 2026-10-25

Status: **NETWORK CONFIGURATION ONLY - NOT FIELD-MOTION AUTHORIZATION**

## Objective

Configure Tractor01 so its persistent normal underlay route is:

```text
default via 192.168.8.1 dev eth0 src 192.168.8.20
```

The Raspberry Pi 5 must reach the wider field network through its physical
Ethernet cable to the onboard GL-SFT1200 router. The Pi's onboard `wlan0`
interface must not independently associate with the AXE75 `TractorField` SSID
during mission operation.

This task configures and verifies networking only. It does not authorize the
tractor to move, start a mission, engage Auto, or engage the cutting deck.

## Operator and equipment state

- Al will be in the field with a Windows 10 laptop and PuTTY.
- Keep Tractor01 parked, the engine off, the transmission physically
  disengaged, the cutting deck off, and phone control in Pause while changing
  or testing network configuration.
- Do not start the pure-pursuit controller for this work.
- Do not weaken the 500 ms phone/server/Teensy fail-closed protections.
- Do not change Teensy firmware, pure-pursuit behavior, or mission geometry.
- Never put an operator key into a transcript, repository file, or chat.

## Known physical and logical topology

```text
Windows laptop / phone
        |
        | AXE75 TractorField network (192.168.10.0/24)
        |
Archer AXE75 gateway: 192.168.10.1
        |
        | Wi-Fi uplink
        |
GL-SFT1200 sta0: previously 192.168.10.203
GL-SFT1200 br-lan: 192.168.8.1
        |
        | physical Ethernet cable
        |
Tractor01 eth0: 192.168.8.20/24
```

An unwanted parallel path was present:

```text
AXE75 TractorField -> Tractor01 onboard wlan0: previously 192.168.10.120/24
```

The `eth0` interface is the physical Ethernet connection between the Pi and
the GL router. The `wlan0` interface is the Pi's onboard Wi-Fi radio. The GL
router's upstream Wi-Fi client is `sta0`; it is not the Pi's `wlan0`.

The GL router supplied or reserved `192.168.8.20` for Tractor01 on its LAN.
Earlier Pi output described that address as dynamic, so verify both the GL
DHCP reservation and the Pi profile rather than assuming that it is static.

## Evidence from 2026-10-24

During the failed `TractorField` test, Tractor01's onboard `wlan0` was directly
associated with the AXE75:

- Pi `wlan0`: `192.168.10.120`
- AXE75 gateway: `192.168.10.1`
- signal: approximately -58 dBm
- transmit bitrate: 6.5 Mbit/s
- receive bitrate: 13.0 Mbit/s
- cumulative transmit failures: 11,024 of 61,193 packets at the captured time
- gateway test: 1,889 transmitted, 1,451 received, 23.1869% packet loss
- round-trip time: 1.051/21.513/2998.863/129.759 ms min/average/max/mdev

Disabling Pi Wi-Fi power saving did not correct the instability.

The GL-router recovery path used the cable instead:

- Pi `eth0`: `192.168.8.20/24`
- GL LAN/gateway: `192.168.8.1`
- phone while connected to the GL LAN: `192.168.8.190`
- the recovery page reached `https://192.168.8.20:8765`
- the page reached `CONNECTED PAUSE`, and Manual recovery subsequently worked

This supports making `eth0` the Pi's normal physical underlay. It does not by
itself qualify a future Auto mission.

## ZeroTier precision

PuTTY connects to Tractor01's ZeroTier address `192.168.193.76`. On the Pi,
that connection appears on a ZeroTier virtual interface. ZeroTier's encrypted
UDP transport still leaves the Pi through a physical underlay selected by the
Linux routing table.

The desired physical underlay is:

```text
ZeroTier virtual interface -> eth0 192.168.8.20 -> GL 192.168.8.1 -> GL sta0
```

Do not claim that ZeroTier uses `eth0` merely because PuTTY reaches
`192.168.193.76`. Inspect ZeroTier peer endpoints and the Linux route to the
active physical endpoint, or capture interface traffic, to prove the underlay.

## Why route metrics alone are insufficient

A lower default-route metric on `eth0` can make Internet-bound traffic prefer
the GL router. It cannot prevent Linux from using a more-specific directly
connected route on `wlan0`, such as `192.168.10.0/24`. ZeroTier may also learn
a directly reachable peer endpoint on that LAN.

Therefore the mission configuration must include both:

1. a persistent `eth0` default route through `192.168.8.1`; and
2. no automatic Pi `wlan0` association with `TractorField` during mission use.

A separate maintenance Wi-Fi profile may be retained, but it must not silently
become an active mission underlay. If a fallback is retained, document its
activation and make it deliberate rather than automatic.

## Known service-state hazard from 2026-10-24

A temporary runtime systemd override was created at:

```text
/run/systemd/system/tractor-wifi-control.service.d/10-tractorfield-mission.conf
```

It selected the historical `tractorfield-mission` authority policy. That path
failed prior reliability qualification and must not be mistaken for the final
normal configuration. A file under `/run` may disappear after reboot, so first
inspect the installed unit and all drop-ins; do not infer current state from
this note.

At the last reported check, Manual Recovery was active and the normal service
was inactive. Re-establish the actual current state before changing anything.
Do not start a motion-authorized service merely to test routing.

## Safe configuration workflow

The field assistant should present and evaluate one short command group at a
time. Do not give Al a long blind script.

### 1. Capture the untouched state

Collect, without changing state:

- timestamp, hostname, OS, kernel, and repository commit/status;
- IPv4 addresses, main routes, policy rules, and route metrics;
- active and stored NetworkManager connection profiles;
- the network renderer actually managing `eth0` and `wlan0`;
- current `wlan0` association, SSID, BSSID, power-save state, and signal;
- GL gateway reachability from `eth0`;
- ZeroTier status, peers, virtual address, and service status;
- route/service unit state, including `/run` and `/etc` drop-ins;
- `rtcm-server`, Teensy bridge, normal phone control, and recovery service
  states; and
- enough journal evidence to explain any current link failure.

Treat repository documentation as context, not proof of the live Pi state.
Do not assume NetworkManager connection names; discover them first.

### 2. Prove the wired uplink before touching wlan0

Require all of the following while `wlan0` is still available as a fallback:

- `eth0` owns `192.168.8.20/24`;
- `192.168.8.1` answers reliably over `eth0`;
- Internet reachability works when explicitly bound to `eth0` or source
  `192.168.8.20`;
- DNS works through the wired path;
- ZeroTier remains online through the GL-router uplink; and
- required remote endpoints, including the RTK base at `192.168.193.88`, are
  reachable without depending on Pi `wlan0`.

The current RTCM client connects to the ZeroTier RTK source
`192.168.193.88:6001`. Its code also reports Pi `wlan0` RSSI as telemetry.
Loss of that RSSI field after intentionally disconnecting `wlan0` is not proof
that RTCM transport failed; verify RTCM and heading streams independently.

### 3. Protect against remote lockout

Before changing the active route or disconnecting `wlan0`:

- keep physical access to Tractor01;
- keep the GL-router local recovery path available;
- open a second PuTTY session where practical;
- save exact before-state and rollback commands;
- arrange a short timed automatic rollback that restores the prior network
  profile unless explicitly cancelled after verification; and
- do not cancel rollback merely because one PuTTY window remains visible.

If the wired uplink cannot independently carry ZeroTier and maintenance
traffic, stop and diagnose it. Do not disconnect `wlan0`.

### 4. Apply a persistent, minimal configuration

After identifying the actual connection manager and exact profile names:

- make the wired `eth0` profile autoconnect;
- ensure it installs the default route through `192.168.8.1` with a preferred
  metric;
- preserve `192.168.8.20/24` using the already established reservation or a
  deliberately coordinated static address;
- disable autoconnect for the Pi's `TractorField` Wi-Fi profile;
- prevent that Wi-Fi profile from installing a default route if it is retained
  for explicit maintenance;
- leave unrelated home/maintenance profiles unchanged unless Al approves a
  broader change; and
- remove the temporary `tractorfield-mission` runtime override if it still
  exists, reload systemd, but do not automatically start the normal control
  service.

Do not solve this with an ephemeral `ip route` command alone. The desired state
must survive link renewal and reboot.

### 5. Verify before cancelling rollback

Require evidence of all of these conditions:

```text
eth0 has 192.168.8.20/24
default via 192.168.8.1 dev eth0 src 192.168.8.20
wlan0 is not connected to TractorField
the GL gateway is stable and has no material packet loss
Internet and DNS work over eth0
ZeroTier is ONLINE and PuTTY reconnects to 192.168.193.76
ZeroTier's physical transport is shown to use eth0
the RTK-base ZeroTier endpoint remains reachable over the wired uplink
```

Use `ip route get 1.1.1.1` as one check. Its expected routing portion is:

```text
via 192.168.8.1 dev eth0 src 192.168.8.20
```

`ip route get 192.168.193.88` will normally show the ZeroTier virtual
interface; by itself it does not prove whether the encrypted underlay uses
`eth0` or `wlan0`.

### 6. Persistence and soak

- Save all before/after evidence before rebooting.
- Ask Al before performing the controlled reboot.
- After reboot, repeat address, route, `wlan0`, ZeroTier-underlay, DNS,
  gateway, and RTK reachability checks.
- Run a stationary network soak long enough to reveal route changes, DHCP
  churn, packet loss, or ZeroTier loss. Report actual duration and statistics.
- Any unexplained route change, meaningful packet loss, loss of PuTTY,
  ZeroTier underlay ambiguity, or RTCM reachability failure is a test failure.
- Preserve logs and exact rollback instructions.

## Acceptance boundary

Successful completion means only that the persistent network path is verified
and documented. It does not mean Tractor01 is ready to move.

Before any later mission, Tractor01 still requires, in order:

1. stationary Pause with blades off;
2. application and independent readback of the established Heading-F9P
   moving-base profile in volatile RAM;
3. restart and verification of `rtcm-server`;
4. heading reacquisition; and
5. a fresh full mission preflight ending in `MISSION PREFLIGHT PASS`.

The phone-control path must then pass its separate stationary qualification.
Do not weaken watchdogs or treat successful SSH/ZeroTier access as proof that
the safety-critical phone command path is healthy.

## Durable references

- `obsidian_vault/03-design/03-1-rtk-base/Outdoor_Field_Network_CPE210_Implementation_Runbook.md`
- `obsidian_vault/02-testing/20261006-master-manual-phone-rerun-network-failure.md`
- `obsidian_vault/02-testing/20261019-manual-recovery-to-parked-position.md`
- `obsidian_vault/03-design/20261019-phone-network-authority-adr.md`
- `obsidian_vault/02-testing/20261021-master-manual-phone-field-session-summary.md`
- `tractor_rpi/rtcm_server_20260828.py`
