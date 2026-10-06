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

If the Heading F9P lost its RAM-only 5 Hz profile after power-off, stop
`rtcm-server`, run the launcher's `--configure-heading` mode, then restart
`rtcm-server` and allow RTK/heading to settle.

## Field sequence

1. Keep the mower deck disengaged. Confirm regulated router power, the loaded
   low-voltage alarm, and physical E-stop operation.
2. Connect the phone to the tractor router's local Wi-Fi. Open the local phone
   control link and leave it in Pause.
3. Use phone Manual to move to the historical mission start and align with the
   initial heading. Return the phone to Pause.
4. On the field laptop, start the mission dashboard:

   ```powershell
   ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 "cd /home/al/tractor2025 && sudo -v && python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/mission_dashboard_master_manual_phone_20261006.py"
   ```

5. Open the dashboard's preferred local URL on the laptop. Start the mission
   while the phone remains in Pause. The launcher requires fresh phone control,
   RTK Fixed, fixed-carrier heading, a 0.80-1.30 m baseline, heading accuracy no
   worse than 1 degree, start distance no more than 1.50 m, and heading error no
   more than 20 degrees.
6. When the dashboard says `READY FOR GUARDED AUTO` and the route is clear, use
   the phone's guarded Auto action. Maintain direct supervision.
7. For any unexpected tracking, clearance, power, network, GPS, or heading
   behavior, select phone Pause immediately. After a link dropout, remain in
   Pause through the visible recovery lockout and deliberately re-arm Auto only
   after inspecting the tractor and route.
8. At completion, select phone Pause, then press Ctrl+C in the dashboard
   terminal. Preserve the field log, pursuit log, and relevant service journals.

Logs and voice notes are written under
`/home/al/field_logs/20261006_master_manual_phone_rerun_1mps/`.
