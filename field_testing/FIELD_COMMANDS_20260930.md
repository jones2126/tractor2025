# Tractor01 field commands — 2026-09-30

These commands are for the approved 2026-09-29 consolidated-perimeter mission.
The initial test remains directly supervised and blades-off. Keep the Wi-Fi
phone control page open and the physical e-stop immediately available. Do not select Auto unless preflight
ends with `MISSION PREFLIGHT PASS`.

## 1. Establish the field network

Turn on Starlink or the field router and wait for a working network. Power up
Tractor01 and allow the Raspberry Pi and GPS receivers to boot. The installed
`tractor-wifi-control.service` starts the HTTPS phone control server in Pause.
Use the local ntfy URL while connected to the tractor router, or wait for the
separate ZeroTier-ready notice after RPi5NAS reachability is verified.

## 2. Open PuTTY

Connect PuTTY to Tractor01 as user `al`:

```text
192.168.193.76
```

If ZeroTier is unavailable but the laptop is connected directly to the
tractor's local network, use `192.168.1.151`.

All commands below are single Linux shell lines pasted directly into PuTTY.

## 3. Pull the approved mission from GitHub

This fetches GitHub, shows the local and GitHub revisions, performs only a
fast-forward pull, displays the installed revision and local changes, and
verifies the exact approved mission package without starting services or
motion:

```bash
cd /home/al/tractor2025 && git fetch origin && echo "Before pull: local=$(git rev-parse --short HEAD) GitHub=$(git rev-parse --short origin/main)" && git pull --ff-only origin main && echo "Installed revision: $(git rev-parse --short HEAD)" && git status --short && python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/verify_consolidated_perimeter_20260929.py
```

Expected result: `PASS: exact approved supervised blades-off field package verified.`

## 4. Push and verify the non-interactive Heading-F9P profile

Before running: tractor stationary, phone control in Pause, blades disengaged.

This command stops `rtcm-server`, writes the known 5 Hz Heading-F9P profile to
volatile RAM, independently reads back every targeted value, and restarts
`rtcm-server`. It enables UBX `NAV-RELPOSNED` on USB and disables the USB NMEA
protocol and all targeted USB NMEA messages. It does not prompt and does not
write flash. The service startup repeats the same verified RAM-only recovery.

```bash
cd /home/al/tractor2025 || exit 1; sudo systemctl stop rtcm-server.service || exit 1; sudo python3 -u tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py --heading-startup --device-wait-seconds 30; config_rc=$?; sudo systemctl start rtcm-server.service; start_rc=$?; systemctl is-active rtcm-server.service; if [ "$config_rc" -ne 0 ] || [ "$start_rc" -ne 0 ]; then echo "FAIL: Heading configuration or rtcm-server restart failed — DO NOT SELECT AUTO"; false; else echo "PASS: Heading profile verified and rtcm-server is active"; fi
```

Require both the configurator's independent-readback PASS and active
`rtcm-server`. If either fails, do not select Auto.

## 5. Run preflight

RTK Fixed and fixed-carrier heading may need time to settle. Rerun this same
command as needed while the tractor remains stationary in Pause:

```bash
cd /home/al/tractor2025 && sudo python3 tractor_rpi/testing/mission_preflight_20261002.py --expected-firmware teensy_main_20261003_wifi_v3
```

This dated preflight also requires more than 30 Heading-F9P satellites used
for a normal PASS. Counts of 29-30 produce a non-blocking warning; 28 or fewer
fail closed. Do not continue until the final result is `MISSION PREFLIGHT PASS`.

## 6. Start the approved dashboard

Open a second PuTTY window and keep it open for the entire dashboard session:

```bash
cd /home/al/tractor2025 && sudo -v && python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/mission_dashboard_consolidated_perimeter_20260929.py
```

Open the printed URL on the Windows laptop, not in another phone tab. Keep the
Wi-Fi control page in the phone's foreground so its 5 Hz safety heartbeat is
not throttled. The dashboard URL begins with:

```text
http://192.168.193.76:8088/?key=
```

For drive-to-start: keep blades off and use guarded Manual on the Wi-Fi phone
control page. Voice guidance is optional and may remain off. Stop and select
phone Pause when the dashboard reports ready. Press `START MISSION` only while
stationary in Pause; the launcher runs its safety gates again. Select guarded
Auto on the phone only after the controller is live and the route is clear.

## Optional Tractor01 status/log command

```bash
systemctl is-active rtcm-server.service teensy-bridge.service led-controller.service tractor-wifi-control.service; journalctl -u rtcm-server.service -u teensy-bridge.service -u tractor-wifi-control.service --since "10 minutes ago" --no-pager -n 160
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
