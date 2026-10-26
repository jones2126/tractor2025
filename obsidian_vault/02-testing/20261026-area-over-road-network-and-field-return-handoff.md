# AREA-OVER-ROAD network findings and field-return handoff — 2026-10-26

## Purpose and status

This note preserves the network-path work, boot/preflight changes, field-log
evidence, and the exact startup command for the next supervised field session.
It does not approve a new route, blades-on operation, or unattended motion.

The AREA-OVER-ROAD mission completed, followed by a separately logged manual
cleanup pass. Both archives and the 41,607-row manual log were copied to the
development repository and their SHA-256 values were verified.

## Qualified Tractor01 network path

The intended normal path observed on Tractor01 was:

```text
Phone browser
  -> phone 4G/cellular connection
  -> ZeroTier client 192.168.193.206
  -> Tractor01 ZeroTier 192.168.193.76
  -> tractor-wifi-control.service (zerotier-primary)
  -> Teensy bridge and fail-closed motion controls

Tractor01 physical upstream
  eth0 192.168.8.20/24
  -> GL router 192.168.8.1
  -> TractorField / AXE75 upstream
```

Tractor01's own `wlan0` was intentionally disconnected. The RTK base route to
`192.168.193.88` used the ZeroTier interface with source `192.168.193.76`.
The older `192.168.10.x` phone-control instructions are retired and must not be
used for normal control. The onboard GL-router path remains relevant as the
tractor's upstream and for the separately guarded Manual-only recovery mode.

## Preflight network gates added and field-verified

The full preflight grew from 29 to 35 checks; no checks were removed. The six
new gates require:

1. proof that the current boot applied and independently read back all 22
   Heading-F9P RAM-profile values;
2. `eth0` address `192.168.8.20/24`;
3. default route via `192.168.8.1` on `eth0`, source `192.168.8.20`;
4. Tractor01 `wlan0` not directly associated with `TractorField`;
5. zero loss in the preflight GL-gateway probe; and
6. the RTK-base address routed over ZeroTier from `192.168.193.76`.

The corrected automatic service completed all 35 checks and ended in
`MISSION PREFLIGHT PASS` on Tractor01.

## Boot ordering and network-readiness behavior

The permanent startup order is:

1. `rtcm-server.service` invokes
   `configure_dual_f9p_5hz_profile_20260923.py --heading-startup` as its
   mandatory `ExecStartPre` action.
2. That action writes the established 5 Hz moving-base configuration to the
   Heading F9P's volatile RAM and independently reads back all 22 values.
3. `rtcm-server` cannot start if Heading-F9P configuration or verification
   fails.
4. `tractor-startup-preflight.service` waits for the core services and the
   qualified network path.
5. Its wrapper requires the expected `eth0` address and route, a successful
   GL-gateway probe, the expected ZeroTier route/source, the normal phone
   control service active, and HTTPS reachability to `ntfy.sh`.
6. Only then does it publish the "kicked off" notice and run the 35-check
   non-motion preflight.
7. It publishes PASS or FAILED. A FAILED notice includes the exact command for
   displaying all PASS/FAIL lines.

If the network is not ready within five minutes, preflight and ntfy publishing
are skipped. The service returns a temporary-failure status and systemd retries
after 30 seconds. A completed preflight failure returns a non-retrying status
so the operator can inspect it without repeated failure notifications.

### Manual preflight rerun and full report

After correcting the failed condition:

```bash
sudo systemctl restart tractor-startup-preflight.service
journalctl -u tractor-startup-preflight.service -b --no-pager -o cat
```

The second command is also embedded verbatim in the FAILED ntfy message.

## Network events observed during the mission

In the analyzed ten-minute controller window ending at 11:51:28 EDT:

- stale or missing phone heartbeat: **266.320 seconds** in three events;
- controller network-blocked time including recovery holds:
  **276.985 seconds**;
- estimated dashboard `NETWORK UNSTABLE` display time:
  **286.821 seconds**; and
- every stale-heartbeat sample reported `driving=False`.

The three stale intervals were:

| Interval | Duration | Interpretation |
|---|---:|---|
| 11:42:11–11:42:40 | 29.393 s | Beginning of the long control-session loss |
| 11:42:41–11:46:35 | 234.173 s | Same effective outage; the 0.6 s apparent recovery was not meaningful |
| 11:48:24–11:48:27 | 2.754 s | Short browser/transport request stall |

At 11:42:10.580 the control server recorded `phone_freshness_expired`,
`auto_link_wait_entered`, and `session_invalidated`. No valid stream returned
until the phone reclaimed ownership at 11:46:35.360. Mode selections made by
the visible phone page during the gap recorded `session_present=False`.

The later short event aligned with a 3.168-second server command gap and three
phone requests failing with `signal is aborted without reason`, even though the
phone reported visible, online, and `effective_type=4g`. Observed request RTT
was 40 ms median, 135 ms at p95, and 498 ms maximum. The 498 ms maximum nearly
consumed the entire 500 ms heartbeat-freshness allowance.

The controller failed closed: it froze path progress, issued stop bursts,
reported no driving samples during stale intervals, and enforced its recovery
holds. GPS speed reached 0.928 m/s at the beginning of the first stop while
`driving=False`, consistent with physical coast/deceleration after command
inhibition rather than continued commanded motion.

## Evidence against a Tractor01-to-router outage

The Tractor01/GL-router checks showed:

- correct `eth0` address and default route;
- 0% loss in the live gateway sample, approximately 1.2 ms average RTT;
- GL uplink signal median -52 dBm and minimum -66 dBm during the analyzed
  period;
- no NetworkManager or kernel link-down event; and
- RTCM, Teensy bridge, and normal phone-control services active.

The AXE75 system log downloaded at 13:00 covered 09:24:58–12:58:11. All 252
records were routine DHCP-client lines. It contained 84 successful WAN lease
ACKs spaced 154–155 seconds apart, always retaining WAN address
`100.81.44.171`. Successful renewals occurred at 11:43:41 and 11:46:15 during
the long control-session outage. No reboot, WAN-down, DHCP failure, timeout,
deauthentication, or disconnect was recorded.

AXE75 log SHA-256:

```text
ca1964c1c161e47adcf3a2d866a14c62653e506f436fa8e885c5d82941d4a473
```

The AXE75 log is limited: it records only WAN DHCP activity, not client
association, signal, latency, or packet loss. It supports WAN continuity but
cannot independently prove that the AXE75 radio never stalled. The combined
evidence points primarily to phone browser/session behavior and cellular/
ZeroTier latency rather than the Tractor01 wired path or an AXE75 WAN reset.

## Rebuild coverage

The permanent rebuild sources now contain both required behaviors:

| Requirement | Rebuild source |
|---|---|
| Heading F9P configured before RTCM | `tractor_rpi/setup/rtcm-server.service` and the generated unit in `install_services.sh` |
| Automatic network-gated preflight | `tractor_rpi/setup/tractor-startup-preflight.service` |
| Preflight wrapper and ntfy text | `tractor_rpi/testing/run_startup_mission_preflight_20261026.sh` |
| Installation and boot enablement | `tractor_rpi/setup/install_services.sh` and `install_tractor_startup_preflight_service.sh` |
| Rebuild/operator instructions | `tractor_rpi/setup/tractor01_rpi_setup.md` |

Thus a rebuild using the checked-in Tractor01 installer restores the Heading
F9P `ExecStartPre`, installs and enables automatic preflight, waits for real
network readiness, and preserves the manual rerun/log commands.

## Next field session: September master/manual route

The October 6 package reruns the immutable September master/manual mission:

- 19,825 waypoints;
- approximately 3,019 m;
- 1.00 m/s with 2.00 m lookahead; and
- route SHA-256
  `0276fa22f2c7def0a516b2dcd5516bfd3b05fa647f3b50607a6ca5ec218436b4`.

It is a **directly supervised, blades-off field-test package**. The attached
startup checklist's blade-engagement step is incompatible with this launcher
and must remain omitted unless a separate reviewed package explicitly approves
blades-on operation.

After boot startup preflight passes, start its dashboard from PuTTY with:

```bash
cd /home/al/tractor2025
sudo -v
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/mission_dashboard_master_manual_phone_20261006.py
```

That dashboard startup again stops RTCM, applies and verifies the Heading-F9P
RAM profile, restarts RTCM, and requires it active. Pressing Start Mission then
runs another fresh full preflight before the controller can report
`READY FOR GUARDED AUTO`.

Because the 2026-10-26 run contained a long phone-session loss and a separate
transport stall, the next session should first keep the tractor stationary and
run the established network soak. Any heartbeat expiry, session loss, path
change, or failed timing gate ends the moving-test attempt until reviewed.
