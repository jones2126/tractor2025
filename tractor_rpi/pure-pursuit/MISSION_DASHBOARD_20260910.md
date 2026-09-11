# Live mission dashboard — 2026-09-10

The dashboard is a local web interface modeled after the Pure Pursuit replay
tool. It displays the complete planned path, the most recent 30 seconds of the
actual tractor trail, heading,
active target, mission progress, controller calculations, GPS state, steering
telemetry, and JRK transmission telemetry.

Before the controller starts, a dedicated GPS feed shows the tractor's live
position, its distance from the mission start, and the east/west and
north/south movement needed to reach the start. **OPEN MESSAGES** opens the
complete launcher output in a separate browser tab with a copy button.

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

The dashboard is an additional operator control. It is not an emergency stop
and does not replace the handheld.

## Field startup

On tractor01, update the repository and flash the expected
`teensy_main_20260908_1p8_test` firmware first. Then run this one line:

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

## Starting and running

1. Keep the mower deck disengaged and the handheld in Pause.
2. Open the dashboard URL and confirm live Teensy status is visible.
3. Press **START MISSION** and accept the blades-off confirmation.
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
