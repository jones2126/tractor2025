# AREA-OVER-ROAD authorized blades-off field package

**DRAFT - UNTESTED IN THE FIELD**

This package is authorized for one directly supervised blades-off feasibility run. It does not authorize mowing.

## Mandatory Heading-F9P preparation before preflight

After every Heading-F9P power cycle, apply and independently read back the
established 5 Hz moving-base profile in volatile RAM before running the live
preflight. Keep the tractor stationary in Pause with blades disengaged:

```bash
cd /home/al/tractor2025
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED/prepare_heading_and_preflight_20261024.sh
```

The helper stops `rtcm-server`, applies the RAM-only profile, independently
verifies its targeted settings, restarts the service, waits 60 seconds for
reacquisition, and runs the live mission preflight. Require both the Heading
configuration PASS and final `MISSION PREFLIGHT PASS`. It does not start the
logger, Pure Pursuit, or motion.

## What happens when Start Mission is pressed

1. The dashboard requires fresh Teensy and navigation telemetry and phone Pause.
2. The dashboard launches `run_area_over_road_field_20261024.sh`.
3. The launcher verifies the exact mission, audit, validation hashes, authorization state, and required programs.
4. It checks the Wi-Fi phone heartbeat, Pause state, released Wi-Fi E-stop, firmware preflight, RTK Fixed, fixed-carrier heading, heading geometry, start distance, and start heading.
5. It starts `field_test_logger_20260828.py` and `pure_pursuit_controller_20261022.py`.
6. Pure Pursuit waits while the phone remains in Pause. The tractor must not move.
7. When the dashboard says `READY FOR GUARDED AUTO`, selecting Auto on the phone gives Pure Pursuit motion authority.
8. Phone Pause, Manual, and E-stop remain operator overrides.

## Windows 10 PowerShell - update and verify Tractor01

After this package has been committed and pushed, open PowerShell on the field laptop:

```powershell
ssh -t al@192.168.193.76 "cd /home/al/tractor2025 && git pull --ff-only origin main && python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED/verify_area_over_road_field_package_20261024.py"
```

Require all verifier lines to pass. This command does not start the controller.

## Windows 10 PowerShell - start the dashboard

If the field laptop has the repository at `C:\Repos\tractor2025`:

```powershell
Set-Location C:\Repos\tractor2025
git pull --ff-only
& .\field_testing\sites\62_Collins_multi_boundary_20260915\mission_plans\20261023_AREA-OVER-ROAD_DRAFT_UNTESTED\start_area_over_road_field_dashboard_20261024.ps1
```

The PowerShell starter updates Tractor01, verifies the exact mission package, and then starts the dashboard. The PowerShell window remains attached to the dashboard session. Keep it open. Open the dashboard URL printed in that window.

If the local repository is unavailable, run the remote starter directly:

```powershell
ssh -t al@192.168.193.76 "cd /home/al/tractor2025 && git pull --ff-only origin main && python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED/verify_area_over_road_field_package_20261024.py && bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED/start_area_over_road_field_dashboard_20261024.sh"
```

## Field control sequence

1. Blades off; open phone control and confirm Pause.
2. Run the mandatory Heading-F9P preparation and require `MISSION PREFLIGHT PASS`.
3. Start the dashboard from PowerShell and open its printed URL.
4. Start voice guidance.
5. Select Manual and drive to the start.
6. Stop and select Pause.
7. Press Start Mission. The launcher performs all gates and starts Pure Pursuit, but Pause still prevents motion.
8. Wait for `READY FOR GUARDED AUTO`.
9. With the route clear, select Auto. Pure Pursuit now controls the tractor.
10. On completion, select Pause and then Manual; drive to parking, select Pause, set the parking brake, and turn off the engine.

## Post-run evidence

After the controller has stopped:

```powershell
ssh -t al@192.168.193.76 "cd /home/al/tractor2025 && bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261023_AREA-OVER-ROAD_DRAFT_UNTESTED/finish_area_over_road_session_20261024.sh"
```

Copy the reported telemetry archive off Tractor01 and verify its SHA-256. Also download the original system log from the Archer AXE75 at the RTK base station for the `TractorField` network, recording its clock/time zone and download time.
