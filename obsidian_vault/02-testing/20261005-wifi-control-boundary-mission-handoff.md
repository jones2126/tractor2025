# 2026-10-05 Wi-Fi control and boundary mission handover

## Purpose

This note preserves the working state at the end of the long NRF24, Wi-Fi
handheld, preflight, and consolidated-perimeter preparation chat. Use it as
the dated source of truth when a new chat is opened. Older firmware tables in
`AGENTS.md` and `00-project-overview.md` describe earlier project snapshots.

No boundary mission was launched while preparing this handover.

## Current Tractor01 control state

- Tractor Teensy firmware: `teensy_main_20261003_wifi_v3`.
- PlatformIO selection: `tractor_teensy/platformio.wifi-primary.ini`.
- Teensy bridge service uses
  `tractor_rpi/teensy_serial_bridge_20261002.py`, which inherits the current
  Wi-Fi command support.
- The NRF24 handheld and its radios are not required for current control. The
  soldered NRF24 link failed the isolated HELLO/ACK test and has been set aside.
- The physical tractor E-stop remains installed, unchanged, and authoritative.
- The phone Wi-Fi E-stop latches the engine-stop relay. Guarded
  **MODE SELECT + E-STOP** resets it and leaves control in Pause.
- Wi-Fi command loss exceeding the Teensy's 500 ms watchdog pauses steering
  and transmission. The field server then latches Pause and invalidates the
  session. Reconnection establishes a fresh session in Pause; a new guarded
  Manual or Auto selection is required and motion never resumes automatically.
- Manual steering and transmission were tested in both directions with the
  engine off. Auto was armed while stationary and did not create motion without
  a navigation command.
- The engine-stop relay was tested: the engine stopped on the phone E-stop and
  could be restarted after the guarded reset.

## Automatic phone-control service

`tractor-wifi-control.service` is installed, enabled, and starts automatically
after the network, ZeroTier daemon, and Teensy bridge are ordered at boot.

Relevant tracked files:

- `tractor_rpi/setup/tractor-wifi-control.service`
- `tractor_rpi/setup/install_tractor_wifi_control_service.sh`
- `tractor_rpi/testing/webrtc/wifi_primary_control_20261003.py`
- `tractor_rpi/testing/webrtc/wifi_primary_control_20261003.html`
- `tractor_rpi/testing/test_wifi_primary_control_20261003.py`

Network behavior:

- Local control URL uses `https://raspberrypi.local:8765/`.
- ZeroTier control URL uses `https://192.168.193.76:8765/`.
- Separate local-ready and ZeroTier-ready notices are sent through the usual
  Tractor01 ntfy topic.
- ZeroTier readiness requires three successful checks to the always-on RPi5NAS
  at `192.168.193.217:22`; a 90-second window is informational, not a retry
  deadline.
- The URL bearer key is stored outside the repository at
  `/home/al/.config/tractor-wifi-control/operator.key` with mode 0600.
- The key and URL now remain stable across service restarts and reboots.
- Hidden or old browser tabs stop heartbeats and cannot fight the visible tab
  for ownership. Keep only one visible phone-control tab anyway.
- The mission dashboard discovers the tractor address used to reach the field
  laptop at `192.168.10.48`. Its ntfy notice prefers that local
  `http://192.168.10.x:8088/` URL and falls back to ZeroTier only if no usable
  tractor LAN address is found.

Deployment acceptance on 2026-10-05:

- Tractor01 pulled commit `c8b24e7`.
- The installer reported that it preserved the existing phone-control URL.
- The service stopped through its graceful Pause/neutral shutdown path and
  restarted active.
- Local and ZeroTier readiness notifications were both sent.
- With the current ZeroTier page left open, a second service restart showed
  `RECONNECTING · TRACTOR PAUSED` for about 0.5 seconds and returned to
  `CONNECTED · PAUSE` without opening a new URL.
- The development test suite passed all 16 Wi-Fi-control tests; the page's
  JavaScript also passed a syntax check.

The repeated ntfy notices are readiness notices. Their access URL should now
be identical. A new page does not need to be opened after each notice.

## GPS, heading, and preflight state

- `rtcm-server.service` applies and independently verifies the Tractor01 5 Hz
  moving-base Heading-F9P profile in volatile RAM during startup.
- The standalone recovery/configuration path is
  `tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py
  --heading-startup`.
- The current preflight is
  `tractor_rpi/testing/mission_preflight_20261002.py`.
- Heading satellites used: PASS above 25, WARNING at 24-25, FAIL at 23 or
  fewer.
- The last complete reported preflight passed every check with RTK Fixed,
  fixed carrier heading, 31 heading satellites used, healthy JRK telemetry,
  phone Pause, and firmware `teensy_main_20261003_wifi_v3`.
- A fresh full preflight is still mandatory immediately before the field run.

## Approved boundary mission

Package:

`field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/`

Verified route:

- 3,989 waypoints.
- 790.077 m.
- Every speed command is 1.00 m/s.
- Nominal motion time is about 13.2 minutes; allow roughly 20 minutes for the
  supervised field activity.
- Mission SHA-256:
  `35ea19776283ef415518fd57997e418b966ea823741bcc6a6ca135cee7b504d9`.
- Audit SHA-256 after the phase-compatibility correction:
  `1f44364e9dc8443e98631e11fbf23e6cae76a88d7383bca1a107f7c03d6d4eee`.
- The audit now contains 77 unique contiguous route phases required by the
  controller's phase-locked recovery. The correction changed no mission
  coordinate, heading, lookahead, or speed command.
- Approved only for the initial directly supervised, blades-off field test.
- Keep the physical E-stop immediately available.

The launcher and dashboard were updated for the Wi-Fi phone controller. The
generated validation report still contains historical references to the NRF
handheld and older firmware; those strings are provenance from the original
approval report, not the current launch procedure. Do not edit the approved
mission or audit file to clean up those historical strings.

Primary operating references:

- `field_testing/FIELD_COMMANDS_20260930.md`
- `field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/FIELD_TEST_README.md`
- `field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/BOUNDARY_MISSION_NEW_CHAT_CONTEXT_20261005.md`

The mission dashboard belongs on the Windows laptop. The single Wi-Fi control
page belongs in the phone foreground. Voice guidance and voice notes are
optional; Al currently prefers to leave voice off.

## What the next chat should do

1. Read the boundary-mission new-chat context named above.
2. Guide Al one command or screen action at a time and interpret each result.
3. Verify the immutable package before starting any service or controller.
4. Require a fresh Heading-F9P verification and `MISSION PREFLIGHT PASS`.
5. Use guarded phone Manual to reach and align at the start, then return to
   Pause.
6. Start the mission from the laptop dashboard and select guarded phone Auto
   only when the controller is live and the route is clear.
7. Preserve the field log and dashboard/controller outcome after completion or
   any early stop.
8. Do not modify or rebuild the approved route during the run.

## Deferred coverage work

Coverage planning must wait for the boundary run result and field log. The
prepared context is:

`field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261005_zamboni_coverage/NEW_CHAT_CONTEXT.md`

That document is context only. No coverage mission or route was generated in
this handover.
