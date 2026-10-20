# Manual Recovery to Parked Position

Date: 2026-10-19
Status: software prepared; Tractor01 installation and physical validation deferred

## Purpose

This is a guarded way to abandon or hold an Auto mission and drive Tractor01
slowly to parking through the onboard travel router. It is a separate,
Manual-only authority path. It is not a way to continue or advance a mission.

The October 6 recovery established the network proof point on `eth0` at
`192.168.8.20/24`: 1,875 commands, no freshness expirations, 27.2 ms mean RTT,
59.4 ms p95 RTT, and 425 ms maximum command gap. The productionized path uses:

- numeric URL `https://192.168.8.20:8765` (the ntfy link adds the access key);
- permitted phone subnet `192.168.8.0/24` only;
- expected phone-facing SSID `TractorField`;
- Manual and Pause only; Auto is hidden and rejected by the server;
- drive bounds of 25% reverse and 60% forward;
- full steering range, used at low speed;
- the server and Teensy 500 ms freshness/watchdog protections.

The speed bounds are conservative relative to the current firmware mapping.
Positive 60% maps to approximately JRK target 2507, less aggressive than the
lowest current Auto calibration target (2428). Reverse 25% maps to approximately
2912, the first reverse-side calibration bucket. These calculations justify a
starting test envelope; they are not field approval.

## One-time Tractor01 installation (deferred)

From the Tractor01 shell after pulling the reviewed commit:

```bash
cd /home/al/tractor2025
sudo bash tractor_rpi/setup/install_tractor_manual_recovery_service_20261019.sh
```

The installer updates both service definitions, enables normal control, and
explicitly leaves recovery disabled at boot. Before accepting the installation:

1. Confirm `eth0` is `192.168.8.20/24`.
2. Confirm the phone-facing onboard-router SSID. Repository evidence suggests
   `TractorField`, but this must be checked on the router or its label in person.
3. Confirm `systemctl is-enabled tractor-wifi-recovery.service` reports
   `disabled` or `static`, never `enabled`.
4. Run the stationary validation ladder below.

## Exact operator sequence

1. Stop in a clear area. Disengage blades/PTO. Keep people clear and be ready
   to use the physical E-stop.
2. Select physical **Pause** and verify transmission neutral/no motion.
3. On Tractor01 run `sudo tractor-manual-recovery start`.
4. If a mission controller is active, the workflow sends software Pause and
   will start recovery only after telemetry confirms `software_paused=true`.
   It saves a checkpoint containing the route progress and leaves the controller
   held so its progress and logs are preserved.
5. On the phone, connect to the verified onboard-router SSID. Disable automatic
   switching to mobile data for this exercise if Android offers it.
6. Open the ntfy notification titled **Tractor01 RECOVERY MANUAL ONLY**. Confirm
   the page says the same, shows `192.168.8.20`, `eth0`, `192.168.8.0/24`, and
   begins at **CONNECTED · PAUSE**.
7. Inspect the entire route to parking. Do not proceed near people, drop-offs,
   traffic, obstacles, or any location without a clear escape path.
8. Hold **MODE SELECT** and press **MANUAL**. Begin at the smallest practical
   demand. Use the physical E-stop whenever stopping behavior is uncertain.
9. At parking, set drive to Neutral, then guarded **PAUSE**. Verify no motion.
10. Run `sudo tractor-manual-recovery stop`. This intentionally does not restart
    normal control. Shut down/power down according to the ordinary tractor
    procedure.
11. Collect evidence:
    `sudo bash tractor_rpi/recovery/collect_manual_recovery_20261019.sh`.

Useful commands:

```bash
sudo tractor-manual-recovery status
sudo tractor-manual-recovery start
sudo tractor-manual-recovery stop
```

Repeated start and stop requests are safe. If service ownership, interface
identity, fresh Teensy telemetry, or mission Pause is ambiguous, the workflow
fails closed. Normal and recovery services have systemd conflicts and also
share `/run/tractor-phone-control.lock`.

## Physical validation ladder

Each level requires a separately recorded go/no-go decision. A pass does not
authorize the next level automatically.

### Level 0 — home/software (completed 2026-10-19)

- Focused automated tests and dry-run behavior only.
- No claim of Tractor01, router, actuator, or field acceptance.

### Level 1 — stationary, engine off

- Tractor secured, blades disengaged, drive unable to create motion.
- Verify SSID, numeric URL, interface/subnet, access key, recovery-only ntfy.
- Verify start in Pause, Auto absent/rejected, second phone rejected, page hide
  latches Pause, network loss invalidates the session, restart returns to Pause,
  and both software and physical E-stops stop/retain safe state.
- Verify logs contain client IP, session changes, command gaps, actuator
  target/feedback, heartbeat expiry, and service transitions.

### Level 2 — wheels raised, engine/drive system enabled as approved

- Use rated supports and the established mechanical safety procedure.
- Check neutral first, then minimum forward and reverse demand.
- Confirm measured direction, bounded demand, stop latency, link-loss behavior,
  page-hide behavior, E-stop behavior, and no automatic re-arm.
- Confirm the mission checkpoint does not advance during recovery commands.

### Level 3 — separately approved low-speed recovery exercise

- Clear, level route; blades off; spotter and physical E-stop plan.
- Start near parking and use the minimum useful speed.
- Intentionally test Pause and one controlled link interruption before relying
  on the workflow for a real recovery.
- Review the collected archive before granting operational acceptance.

## Known deferred item

The exact phone-facing SSID cannot be confirmed from home. `TractorField` is the
evidence-based current value, but it is a mandatory on-site verification item.
