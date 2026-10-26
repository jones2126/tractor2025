# Tractor01 automatic startup preflight — 2026-10-26

## Status

`INSTALLED AND FIELD-EXERCISED ON TRACTOR01`

This is a stationary startup/preflight improvement for the already-built
AREA-OVER-ROAD mission. It does not change route geometry, firmware, Pure
Pursuit behavior, field authorization, or blade authorization.

## Permanent startup and mission-start behavior

1. `rtcm-server.service` starts automatically at boot.
2. Before the RTCM process starts, its mandatory `ExecStartPre` action writes
   the established 5 Hz moving-base profile to the Heading F9P's volatile RAM
   and independently reads back all 22 targeted values.
3. If that action fails, `rtcm-server.service` does not start.
4. `tractor-startup-preflight.service` runs one non-motion preflight after the
   boot services and network are available.
5. The startup service sends a best-effort ntfy notice when the preflight is
   kicked off, followed by a PASS or FAILED notice.
6. A startup PASS is informative only. The mission launcher still runs a fresh
   full preflight and must end in `MISSION PREFLIGHT PASS` before guarded Auto.

The wrapper now waits before step 4 until the qualified network path is
actually usable. It requires the expected `eth0` address/default route, a live
GL-router gateway, the expected ZeroTier route and source address, the normal
phone-control service, and HTTPS reachability to `ntfy.sh`. Therefore it does
not start preflight merely because systemd declared `network-online.target`,
and it does not knowingly attempt an ntfy publish before ntfy is reachable.
Temporary network-readiness failures are retried by systemd. A completed
preflight failure is not automatically retried or spammed; it requires review
and an explicit manual restart.

## New fail-closed preflight gates

The current `mission_preflight_20261002.py` now also requires:

- proof that the current `rtcm-server` invocation successfully completed the
  Heading-F9P startup configuration action;
- `eth0` address `192.168.8.20/24`;
- default route through `192.168.8.1` on `eth0`, source `192.168.8.20`;
- the Pi's own `wlan0` not connected directly to `TractorField`;
- zero packet loss in the short preflight sample to the GL router at
  `192.168.8.1` over `eth0`; and
- the RTK base address `192.168.193.88` routed through the Tractor01 ZeroTier
  interface with source `192.168.193.76`.

The existing RTCM journal and live RTK/heading checks remain responsible for
proving that corrections are actually flowing and producing valid solutions.

## Live read-only evidence before installation

Observed on Tractor01 on 2026-10-26:

- repository was initially at `240c60c`, then pulled implementation commit
  `f8df26b`;
- `rtcm-server.service` was enabled and active/running;
- its boot-time Heading action ran at approximately 10:08 EDT and reported an
  ACK, independent readback PASS for 22 values, and final RAM-profile PASS;
- `teensy-bridge.service` and `tractor-wifi-control.service` were active;
- `eth0` had `192.168.8.20/24`;
- the default route was via `192.168.8.1 dev eth0 src 192.168.8.20`;
- Pi `wlan0` reported `Not connected`;
- three GL-gateway probes had 0% packet loss and approximately 1.276 ms average
  round-trip time;
- `192.168.193.88` routed through `ztuze7ml6g` from `192.168.193.76`; and
- all six new Heading-startup/network gates passed when exercised on the Pi.

## Files

- `tractor_rpi/testing/mission_preflight_20261002.py`
- `tractor_rpi/testing/run_startup_mission_preflight_20261026.sh`
- `tractor_rpi/setup/tractor-startup-preflight.service`
- `tractor_rpi/setup/install_tractor_startup_preflight_service.sh`
- `tractor_rpi/testing/test_mission_preflight_20261002.py`

## Installation and verification

Installation requires Tractor01 sudo authority:

```bash
cd /home/al/tractor2025
sudo bash tractor_rpi/setup/install_tractor_startup_preflight_service.sh --start-now
```

The service was installed, enabled, and exercised on 2026-10-26. The first run
correctly ran the full preflight but exposed a false negative in the new
Heading-startup proof: `systemctl daemon-reload` had cleared the completed
`ExecStartPre` status fields. The proof gate was corrected to use the
successful current-boot independent-readback journal when those fields are no
longer retained.

The corrected run completed 35 checks and ended in `MISSION PREFLIGHT PASS`.
The service state was `active (exited)` with `status=0/SUCCESS`. The earlier
2026-10-24 preflight had 29 checks; none were removed. The six added checks are
the Heading-startup proof plus the five network-path gates listed above.

If an automatic run fails, remain in Pause with blades off, do not select Auto,
and view the complete preflight report with:

```bash
journalctl -u tractor-startup-preflight.service -b --no-pager -o cat
```

The ntfy failure notice includes this command.

To rerun after correcting a failed gate:

```bash
sudo systemctl restart tractor-startup-preflight.service
journalctl -u tractor-startup-preflight.service -b --no-pager -o cat
```
