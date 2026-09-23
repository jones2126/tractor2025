# Live mission dashboard — 2026-09-10

The dashboard is a local web interface modeled after the Pure Pursuit replay
tool. It now loads the reviewed 2026-09-15 partial rings master mission. It
displays the complete planned path, the most recent 30 seconds of the
actual tractor trail, heading,
active target, mission progress, controller calculations, GPS state, steering
telemetry, and JRK transmission telemetry.

Before the controller starts, a dedicated GPS feed shows the tractor's live
position, its distance from the mission start, and the east/west and
north/south movement needed to reach the start. **OPEN MESSAGES** opens the
complete launcher output in a separate browser tab with a copy button.

## Voice guidance to the start

Open the dashboard on a phone connected to the field network (or to ZeroTier),
turn up its media volume, and press **START VOICE GUIDANCE**. The button press
is required because phone browsers do not permit a page to begin speaking on
its own. Keep the dashboard in the foreground and keep the phone awake.

While the tractor is driven manually, the phone announces the distance to the
mission start and its direction relative to the tractor as a clock position.
Announcements occur every 10 seconds when far away, then every 6 or 4 seconds
as the tractor approaches. Within the launcher's 1.50 m position tolerance,
the guidance changes to the left/right correction needed to reach the mission
start heading. It reports ready only when the launcher's strict RTK Fixed,
heading-valid, fixed-carrier, baseline, heading-accuracy, position, and heading
limits are all satisfied. After the handheld is moved to Pause, it announces
that the start checks are ready. When the mission becomes active, the same
enabled voice guidance switches from positioning directions to safety-stop
monitoring.

After the tractor has begun driving, the phone announces RTK-position loss,
heading-solution loss, or handheld-radio loss when one of those gates stops
the controller. It then announces when GPS and heading return, when the
five-second stability timer is running, and when the required handheld
Pause-to-Auto acknowledgement may be completed. A new state is spoken
immediately, then the current safety or recovery instruction repeats every
10 seconds until the state changes or driving resumes.

Voice guidance is advisory and sends no motion commands. Drive only in Manual,
keep the handheld with you, watch the route and obstacles, and stop whenever an
announcement disagrees with the physical situation. Browser speech may stop if
the phone locks or the dashboard moves to the background.

## Safety behavior

- The handheld Pause remains the independent safety override and must stay
  within reach.
- The web **PAUSE** button immediately sends repeated zero-speed commands and
  places Pure Pursuit in a software hold. GPS and telemetry continue updating,
  but the pursuit waypoint does not advance.
- **CLEAR PAUSE** releases only the software hold, and the dashboard rejects
  this command unless fresh telemetry confirms that the handheld is in Pause.
  The tractor therefore remains physically paused until the operator selects
  Auto on the handheld.
- Moving the handheld from Manual to Auto does not clear a dashboard software
  Pause. This prevents an unexpected restart while the operator is moving the
  tractor manually.
- Closing the dashboard server with Ctrl+C software-pauses and terminates the
  mission controller so its normal cleanup stops the tractor and logger.
- The Start button is rejected unless fresh Teensy telemetry reports both
  steering and transmission in handheld Pause.
- When voice guidance is enabled, an in-mission safety stop is announced with
  its broad cause and recovery instructions. These announcements are advisory;
  they do not weaken any controller gate or automatically resume motion.

The dashboard is an additional operator control. It is not an emergency stop
and does not replace the handheld.

## Field startup

On tractor01, update the repository and flash the expected
`teensy_main_20260914` firmware first. Then run this one line:

```bash
cd /home/al/tractor2025 && python3 tractor_rpi/pure-pursuit/mission_dashboard_20260910.py
```

The terminal prints a private, temporary URL containing an operator key, for
example:

```text
ZeroTier: http://192.168.193.76:8088/?key=...
Local:    http://192.168.1.151:8088/?key=...
```

From a remote development machine, open the complete ZeroTier URL. Use the
local URL only from a device on the tractor's local network. Keep the terminal open.
The key changes whenever the dashboard restarts and prevents another device
on the field network from operating the buttons without the URL.

At startup, the dashboard also posts the complete ZeroTier URL to
`https://ntfy.sh/rpi-tractor01-jones2126`. The notification body contains the
URL and its click action opens the dashboard directly. Failure to reach ntfy
is reported as a warning and does not prevent the local dashboard from
starting. Because the URL contains the temporary operator key, anyone who can
read that ntfy topic can use the dashboard controls for that server session.

## Starting and running

1. Keep the mower deck disengaged and the handheld in Pause.
2. Open the dashboard URL and confirm live Teensy status is visible. If the
   tractor still needs positioning, open the same URL on a phone, select Manual,
   press **START VOICE GUIDANCE**, and drive to the announced pose. Stop and
   return the handheld to Pause when instructed.
3. Confirm the map and mission note identify the **62 Collins partial rings
   master**, then press **START MISSION** and accept the blades-off confirmation.
4. Watch the launcher output on the lower-right, or open **OPEN MESSAGES** in a
   separate tab. It verifies the exact reviewed mission, runs preflight, and
   checks the starting position and heading.
5. Wait until controller telemetry is live, then select Auto on the handheld.
6. Use **PAUSE** for a software hold. The dashboard keeps showing the complete
   mission while limiting the cyan actual trail to its most recent 30 seconds.
7. To continue, first put the handheld in Pause, then press **CLEAR PAUSE**.
   Confirm the dashboard changes to **HANDHELD PAUSE**, then select Auto on the
   handheld. Manual-to-Auto alone deliberately does not clear software Pause.
8. At mission completion, the controller sends stop and the launcher closes
   the field logger normally.

For this supervised partial-rings test, the runtime GPS/heading gate matches
the successful 2026-08-30 controller behavior: RTK Fixed and `headValid` remain
mandatory, but carrier, baseline, and heading-accuracy values are diagnostic
rather than independent stop conditions. A healthy packet permits automatic
recovery without a five-second timer or a required handheld switch cycle.
Handheld Manual/Pause gating, dashboard software Pause, frozen progress outside
AUTO, and bounded phase-locked path reacquisition remain enabled.

## Loaded mission

- 19,250 remaining waypoints with 2.0 m lookahead (resume at source waypoint 91)
- 1.0 m/s cruise; 0.5 m/s for planned connectors, garden-left A-C, and the
  other fields' innermost rings including D-E
- the two near-360-degree main-backyard connector loops are prohibited and
  have been replaced by contained forward connectors
- the launcher uses a 6 m forward tracking window so the controller cannot
  jump across the 10.08 m closed A ring
- inner rings for the main backyard, both gardens, and front yard
- recorded safe connectors between fields
- over-road boundary included with the 24-inch-expanded pole exclusion
- over-road inner ring omitted because no contained 1.63 m entry/exit chain
  was found
- stripes omitted

The dashboard starts
`run_62_Collins_partial_rings_field_test_20260916.sh --dashboard`. As of the
2026-09-16 clear-sky restart, it loads the resume route beginning at source
waypoint 91 rather than the original route beginning. The launcher
still verifies the exact mission checksum, report limitations, preflight,
RTK/heading state, and starting pose before it starts Pure Pursuit.

## Network ports

| Port | Purpose |
|---|---|
| TCP 8088 | Dashboard page and local API |
| UDP 6011 | Local-only Pure Pursuit Pause/Clear-Pause control |
| UDP 6012 | Local-only controller telemetry for the dashboard |
| UDP 6013 | Dedicated live GPS feed for pre-start map positioning |
| UDP 6003 | Existing Teensy bridge status |
| UDP 6004 | Existing `cmd_vel` command path |

The dashboard control and controller telemetry sockets bind or transmit only
through localhost. TCP 8088 is reachable from the field network and protected
by the temporary operator key.

## Heading-drop diagnostics

The 2026-09-16 diagnostic update keeps the mission fail-closed while exposing
the reason for an intermittent dual-F9P heading rejection. The dashboard now
shows carrier state, baseline length, heading accuracy, satellites/C/N0, and
the RELPOSNED `fixOK`, `diff`, `relPosValid`, `moving`, `refPosMiss`, and
`refObsMiss` flags. Controller wait messages include the same values whenever
`headValid` clears.

Both the Pure Pursuit CSV and `field_test_logger_20260828.py` CSV retain these
values per cycle. This distinguishes an RF/satellite problem from missing
moving-base observations, an invalid relative position, or a receiver flag
transition without weakening any drive gate.
