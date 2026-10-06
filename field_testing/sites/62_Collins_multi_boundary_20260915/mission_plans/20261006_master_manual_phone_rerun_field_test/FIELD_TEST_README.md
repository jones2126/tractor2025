# September 23 master mission rerun with Wi-Fi phone handheld

This package reruns the completed mission shown in the historical replay
`20260923_124933_master_manual_5hz`. It deliberately reuses the immutable route
from the September 20 master/manual package and adds only the current Wi-Fi phone
control, preflight, dashboard, and logging workflow.

It does not use or modify the newer consolidated boundary mission or the October
coverage-map work.

## Exact mission identity

- Route: `62_Collins_master_manual_resampled_1mps_20260920.txt`
- Waypoints: 19,825
- Speed: 1.00 m/s at every waypoint
- Lookahead: 2.00 m
- SHA-256: `0276fa22f2c7def0a516b2dcd5516bfd3b05fa647f3b50607a6ca5ec218436b4`
- Historical evidence: the September 23 pursuit log reports 19,825 waypoints

## Safety scope

This remains a directly supervised, blades-off field test. Keep the physical
E-stop immediately available. The phone control page must stay open in the
foreground and use the tractor router's local `192.168.10.x` network; ZeroTier is
for maintenance/status, not motion authorization.

The current phone-control stack fails closed to Pause, invalidates the phone
session after heartbeat loss, requires a new guarded choice, and prevents Auto
until the heartbeat has been continuously healthy for five seconds. The mission
dashboard also displays a ten-second network-recovery lockout after a dropout.

Do not run until the router has regulated power and the loaded low-voltage alarm
is active. Yesterday's 11.7 V observation was a failed-power condition, not an
approved cutoff.

## Tractor01 deployment and verification

After this package is committed and pushed, on Tractor01:

```bash
cd /home/al/tractor2025
git pull --ff-only origin main
sudo systemctl stop tractor-wifi-control.service
sudo bash tractor_rpi/setup/install_tractor_wifi_control_service.sh
sudo systemctl start tractor-wifi-control.service
systemctl is-active tractor-wifi-control.service
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/run_62_Collins_master_manual_phone_20261006.sh --verify-only
```

Verification must end with both PASS messages and must report 19,825 waypoints.
It does not start the controller or logger.

The normal dashboard startup now performs the complete Heading-F9P preparation
automatically: stop `rtcm-server`, apply and verify the RAM-only 5 Hz startup
profile, restart `rtcm-server`, and require the service to be active. A failure
stops dashboard startup with `DO NOT SELECT AUTO`. The launcher performs the
same preparation when it is run directly rather than through the dashboard.

## Field sequence

1. Keep the mower deck disengaged. Confirm regulated router power, the loaded
   low-voltage alarm, and physical E-stop operation.
2. After Tractor01 boots, wait for the ntfy notification titled
   `Tractor01 local control ready`. The enabled `tractor-wifi-control` service
   sends this when the local control hostname is ready. Connect the phone to the
   tractor router's local Wi-Fi, open that ntfy link, and leave the phone in
   Pause. A separate ZeroTier status notification may also arrive, but it cannot
   authorize motion and is not the phone-control link.
3. On the field laptop, start the mission dashboard:

   ```powershell
   ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 "cd /home/al/tractor2025 && sudo -v && python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/mission_dashboard_master_manual_phone_20261006.py"
   ```

   Before printing the dashboard URLs, this command automatically stops
   `rtcm-server`, applies and verifies the Heading-F9P 5 Hz RAM profile, restarts
   `rtcm-server`, and confirms it is active. Wait for
   `PASS: Heading profile verified and rtcm-server is active`. If this step
   fails, do not select Auto; copy the terminal output into the Codex chat.

4. Open the dashboard's preferred local URL on the laptop and select
   `START VOICE GUIDANCE`. The mission dashboard—not the phone-control page—
   provides spoken distance, direction, and heading guidance to the start. This
   action also enables hands-free voice notes. Keep the dashboard browser open.
5. Use phone Manual to move to the historical mission start and align with the
   initial heading while listening to the dashboard guidance. Return the phone
   to Pause before starting the mission.
6. Start the mission from the dashboard while the phone remains in Pause. The
   launcher requires fresh phone control,
   RTK Fixed, fixed-carrier heading, a 0.80-1.30 m baseline, heading accuracy no
   worse than 1 degree, start distance no more than 1.50 m, and heading error no
   more than 20 degrees.
7. If preflight fails or another message needs analysis, select `OPEN MESSAGES`
   on the dashboard. Copy the relevant message text and paste it into the Codex
   chat. Future field checklists should retain this troubleshooting step.
8. When the dashboard says `READY FOR GUARDED AUTO` and the route is clear, use
   the phone's guarded Auto action. Maintain direct supervision.
9. For any unexpected tracking, clearance, power, network, GPS, or heading
   behavior, select phone Pause immediately. After a link dropout, remain in
   Pause through the visible recovery lockout and deliberately re-arm Auto only
   after inspecting the tractor and route.
10. At completion, select phone Pause, then press Ctrl+C in the dashboard
    terminal. Confirm the tractor is stopped. Package the run from Tractor01:

    ```bash
    cd /home/al/tractor2025 && sudo -v && bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/collect_master_manual_phone_run_20261006.sh
    ```

    The collector makes timestamped snapshots of the mission logs, pursuit logs,
    Wi-Fi-control logs, service journal, service status, repository revision, and
    network state. It does not stop services or delete source logs. It prints the
    archive's SHA-256 and refreshes this stable copy path:

    `/home/al/tractor01_master_manual_phone_rerun_latest.tgz`

Logs and voice notes are written under
`/home/al/field_logs/20261006_master_manual_phone_rerun_1mps/`.

## Copy the completed archive off Tractor01

From a terminal on the RPi5NAS development machine, use this one line:

```bash
scp al@192.168.193.76:/home/al/tractor01_master_manual_phone_rerun_latest.tgz /home/al/repos/tractor2025/field_testing/
```

Or, from PowerShell in this Windows development workspace, use:

```powershell
scp al@192.168.193.76:/home/al/tractor01_master_manual_phone_rerun_latest.tgz C:\Repos\tractor2025\field_testing\
```

The collector also writes the timestamped archive path and expected SHA-256 to
`/home/al/tractor01_master_manual_phone_rerun_latest.txt`. Verify the copied file
with `sha256sum` on Linux or `Get-FileHash -Algorithm SHA256` in PowerShell before
shutting Tractor01 down.
