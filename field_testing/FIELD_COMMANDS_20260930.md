# Tractor01 field commands — 2026-09-30

These commands are for the approved 2026-09-29 consolidated-perimeter mission.
The initial test remains directly supervised and blades-off. Keep the handheld
and physical e-stop immediately available. Do not select Auto unless preflight
ends with `MISSION PREFLIGHT PASS`.

## Before leaving: install the approved package on Tractor01

The approved package is tracked in the main GitHub workflow. Run this command
in Windows PowerShell while Tractor01 is reachable.

Update Tractor01 and display the installed revision and working-tree status:

```powershell
ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 'cd /home/al/tractor2025 && git pull --ff-only origin main && git rev-parse --short HEAD && git status --short'
```

Verify the pulled package without starting services or motion:

```powershell
ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 'cd /home/al/tractor2025 && python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/verify_consolidated_perimeter_20260929.py'
```

Expected result: `PASS: exact approved supervised blades-off field package verified.`

## Field Window 1 — configure Heading F9P and run preflight

Before running: tractor stationary, handheld in Pause, blades disengaged.
This stops `rtcm-server`, runs the guarded Heading-F9P configuration, restores
`rtcm-server` even if configuration fails, waits for startup, and runs the
preflight expecting firmware `teensy_main_20260926`.

```powershell
ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 'cd /home/al/tractor2025 || exit 1; sudo systemctl stop rtcm-server.service || exit 1; bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/run_62_Collins_consolidated_perimeter_20260929.sh --configure-heading; config_rc=$?; sudo systemctl start rtcm-server.service; start_rc=$?; if [ $config_rc -ne 0 ] || [ $start_rc -ne 0 ]; then echo "HEADING CONFIGURATION OR RTCM RESTART FAILED — DO NOT SELECT AUTO"; exit 1; fi; echo "Waiting 30 seconds for corrections and heading startup..."; sleep 30; systemctl is-active rtcm-server.service teensy-bridge.service; sudo python3 tractor_rpi/testing/mission_preflight_20260804.py --expected-firmware teensy_main_20260926'
```

When prompted by the guarded tool, type exactly:

```text
CONFIGURE HEADING
```

If RTK or heading has not settled yet, rerun preflight with:

```powershell
ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 'cd /home/al/tractor2025 && sudo python3 tractor_rpi/testing/mission_preflight_20260804.py --expected-firmware teensy_main_20260926'
```

## Field Window 2 — launch the approved dashboard

Keep this PowerShell/SSH window open for the entire dashboard session:

```powershell
ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 'cd /home/al/tractor2025 && sudo -v && python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/mission_dashboard_consolidated_perimeter_20260929.py'
```

Open the printed URL beginning with:

```text
http://192.168.193.76:8088/?key=
```

For drive-to-start: keep blades off, use handheld Manual, start voice guidance,
and drive toward the announced direction. Stop and select handheld Pause when
the dashboard reports ready. Press `START MISSION` only while stationary in
Pause; the launcher runs its safety gates again. Select Auto only after the
controller is live and the route is clear.

## Optional status/log window

```powershell
ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 'systemctl is-active rtcm-server.service teensy-bridge.service led-controller.service; journalctl -u rtcm-server.service -u teensy-bridge.service --since "10 minutes ago" --no-pager -n 120'
```

## NoMachine diagnostics on either Windows computer

User-profile NoMachine errors and warnings:

```powershell
if (Test-Path "$env:USERPROFILE\.nx\nxserver.log") { Select-String -Path "$env:USERPROFILE\.nx\nxserver.log" -Pattern "ERROR","WARNING","failed","disconnect","timeout","reset","encrypt","socket" -CaseSensitive:$false | Select-Object -Last 100 } else { Write-Output "NoMachine user log not found: $env:USERPROFILE\.nx\nxserver.log" }
```

Most recently changed user-profile NoMachine files:

```powershell
if (Test-Path "$env:USERPROFILE\.nx") { Get-ChildItem "$env:USERPROFILE\.nx" -Recurse -File | Sort-Object LastWriteTime -Descending | Select-Object -First 20 LastWriteTime,Length,FullName } else { Write-Output "NoMachine user folder not found: $env:USERPROFILE\.nx" }
```

System-wide NoMachine server log tail:

```powershell
if (Test-Path "$env:PROGRAMDATA\NoMachine\var\log\nxserver.log") { Get-Content "$env:PROGRAMDATA\NoMachine\var\log\nxserver.log" -Tail 100 } else { Write-Output "NoMachine system log not found: $env:PROGRAMDATA\NoMachine\var\log\nxserver.log" }
```

Most recently changed system-wide NoMachine log files:

```powershell
if (Test-Path "$env:PROGRAMDATA\NoMachine\var\log") { Get-ChildItem "$env:PROGRAMDATA\NoMachine\var\log" -Recurse -File | Sort-Object LastWriteTime -Descending | Select-Object -First 20 LastWriteTime,Length,FullName } else { Write-Output "NoMachine system log folder not found: $env:PROGRAMDATA\NoMachine\var\log" }
```
