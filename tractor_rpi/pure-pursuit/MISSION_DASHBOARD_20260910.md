# Live mission dashboard — 2026-09-10

The dashboard is a local web interface modeled after the Pure Pursuit replay
tool. It displays the complete planned path, actual tractor trail, heading,
active target, mission progress, controller calculations, GPS state, steering
telemetry, and JRK transmission telemetry.

## Safety behavior

- The handheld Pause remains the independent safety override and must stay
  within reach.
- The web **PAUSE** button immediately sends repeated zero-speed commands and
  places Pure Pursuit in a software hold. GPS and telemetry continue updating,
  but the pursuit waypoint does not advance.
- **RESUME** releases only the software hold. If the handheld remains in Pause,
  the tractor remains physically paused.
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
cd /home/al/tractor2025 && sudo -v && python3 tractor_rpi/pure-pursuit/mission_dashboard_20260910.py
```

The terminal prints a private, temporary URL containing an operator key, for
example:

```text
http://192.168.1.151:8088/?key=...
```

Open that exact URL from the field laptop or phone. Keep the terminal open.
The key changes whenever the dashboard restarts and prevents another device
on the field network from operating the buttons without the URL.

## Starting and running

1. Keep the mower deck disengaged and the handheld in Pause.
2. Open the dashboard URL and confirm live Teensy status is visible.
3. Press **START MISSION** and accept the blades-off confirmation.
4. Watch the launcher output on the lower-right. It rebuilds the mission, runs
   preflight, and checks the starting position and heading.
5. Wait until controller telemetry is live, then select Auto on the handheld.
6. Use **PAUSE** for a software hold. Use **RESUME** when ready, or retain Pause
   on the handheld for the independent physical hold.
7. At mission completion, the controller sends stop and the launcher closes
   the field logger normally.

If Start reports that non-interactive sudo is unavailable, return to the
tractor01 terminal, run `sudo -v`, and press Start again promptly.

## Network ports

| Port | Purpose |
|---|---|
| TCP 8088 | Dashboard page and local API |
| UDP 6011 | Local-only Pure Pursuit Pause/Resume control |
| UDP 6012 | Local-only controller telemetry for the dashboard |
| UDP 6003 | Existing Teensy bridge status |
| UDP 6004 | Existing `cmd_vel` command path |

The dashboard control and controller telemetry sockets bind or transmit only
through localhost. TCP 8088 is reachable from the field network and protected
by the temporary operator key.
