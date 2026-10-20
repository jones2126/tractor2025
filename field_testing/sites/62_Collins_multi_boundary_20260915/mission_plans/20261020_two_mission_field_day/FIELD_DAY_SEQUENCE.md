# 62 Collins two-mission field-day sequence

> **PREPARATION ONLY — EACH MOVING RUN REQUIRES A FRESH GO/NO-GO DECISION**

The field-day order is fixed:

1. qualify the updated phone/control/recovery stack while stationary;
2. run the immutable September master/manual route as the endurance test;
3. stop, archive, copy, hash-check, and review Mission 1 evidence;
4. only if Mission 1 completed without a control-stack failure, run the
   previously approved consolidated perimeter mission;
5. archive and copy Mission 2 evidence before considering later coverage work.

Both moving missions are directly supervised and blades off. The physical
E-stop must remain immediately available. Do not combine intentional network
failure testing with either moving mission.

## Route identities

### Mission 1 — master/manual endurance rerun

- Exact immutable September route; no regeneration or resume splice.
- 19,825 waypoints, approximately 3,019 m.
- 1.00 m/s and 2.00 m lookahead.
- Nominal uninterrupted travel time is about **50.3 minutes**, not 40 minutes;
  positioning, pauses, and gates make the field session longer.
- SHA-256: `0276fa22f2c7def0a516b2dcd5516bfd3b05fa647f3b50607a6ca5ec218436b4`.

### Mission 2 — consolidated perimeter

- Exact previously approved package; do not regenerate it.
- 3,989 waypoints, 790.077 m, 1.00 m/s.
- Approved only for an initial supervised, blades-off test.
- SHA-256: `35ea19776283ef415518fd57997e418b966ea823741bcc6a6ca135cee7b504d9`.

## Before traveling to the field

These files currently exist in the Windows working tree. Tractor01 cannot pull
them until the reviewed changes are committed and pushed. After that has been
done, run this on Windows PowerShell:

```powershell
ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 "cd /home/al/tractor2025 && git pull --ff-only origin main"
```

On Tractor01, install the normal ZeroTier-primary control service and the
Manual-only onboard-router recovery service:

```bash
cd /home/al/tractor2025
sudo systemctl stop tractor-wifi-control.service tractor-wifi-recovery.service
sudo bash tractor_rpi/setup/install_tractor_wifi_control_service.sh
sudo bash tractor_rpi/setup/install_tractor_manual_recovery_service_20261019.sh
sudo systemctl start tractor-wifi-control.service
systemctl is-active tractor-wifi-control.service
systemctl is-enabled tractor-wifi-recovery.service
```

Required results are normal control `active` and recovery `disabled` or
`static`. Then run the read-only package/software verification:

```bash
cd /home/al/tractor2025
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261020_two_mission_field_day/verify_two_mission_field_day_20261020.py
```

Require the final preparation PASS. It is not permission to move.

## Gate 0 — physical and power setup

- Tractor stationary in a clear area; deck/PTO disengaged.
- Regulated router power and loaded low-voltage alarm working.
- Physical E-stop tested and immediately reachable.
- Expected Wi-Fi-primary firmware confirmed by fresh telemetry.
- Phone has ZeroTier connected and exactly one control tab open.
- Normal page identifies `zerotier-primary`, begins in Pause, and does not show
  stale green Auto if dashboard telemetry is interrupted.
- Confirm the onboard travel-router SSID and `eth0` address `192.168.8.20` for
  recovery; do not assume the SSID from documentation alone.

## Gate 1 — stationary phone and recovery acceptance

With the engine off and the tractor secured:

1. Claim normal phone control and confirm Pause.
2. Select guarded Manual with drive and steering at zero.
3. Hide/lock the phone. Require the page and tractor telemetry to return to
   Pause with no automatic re-arm.
4. Reopen the page. Require a new session in Pause.
5. Stop normal control and exercise `sudo tractor-manual-recovery start`.
6. Confirm recovery uses `192.168.8.20`, starts in Pause, hides/rejects Auto,
   and limits Manual demand. Then run `sudo tractor-manual-recovery stop`.
7. Deliberately restart normal control; confirm a fresh Pause session.

Do not proceed directly from untested software to the long mission. Complete
the established wheels-raised neutral, minimum-forward/reverse, page-hide,
link-loss, and E-stop checks before accepting the new stack for moving use.

## Gate 2 — two-minute network soak

Keep the tractor stationary and physically unable to move. Hold guarded Manual
at zero drive/steering for at least two continuous minutes with the phone page
visible. Any expiry, path change, stale display, or automatic re-arm is a fail.
Then evaluate the newest control log:

```bash
cd /home/al/tractor2025
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261020_two_mission_field_day/analyze_phone_control_log_20261020.py --latest-dir /home/al/field_logs --require-minutes 2
```

The provisional timing gates are no heartbeat expiry, maximum armed command
gap no greater than 400 ms, RTT p95 no greater than 100 ms, and maximum RTT no
greater than 250 ms. Review any reported network events separately.

## Mission 1 — master/manual endurance route

Record the local start time with `date '+%Y-%m-%d %H:%M:%S'`. On Windows
PowerShell start the dashboard:

```powershell
ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 "cd /home/al/tractor2025 && sudo -v && python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/mission_dashboard_master_manual_phone_20261006.py"
```

Require automatic Heading-F9P preparation to pass. Use Manual only to reach
and align with the start, return to Pause, start from the dashboard, and select
guarded Auto only after `READY FOR GUARDED AUTO`. Supervise continuously.

At completion or any abort: phone Pause, verify neutral/no motion, stop the
dashboard with Ctrl+C, and keep the tractor powered while collecting evidence.
Substitute the recorded start time below:

```bash
cd /home/al/tractor2025
sudo -v
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261020_two_mission_field_day/collect_field_day_run_20261020.sh --run master --since 'YYYY-MM-DD HH:MM:SS'
python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261020_two_mission_field_day/analyze_phone_control_log_20261020.py --latest-dir /home/al/field_logs --require-minutes 40
```

From Windows, copy both the archive and its hash record:

```powershell
scp al@192.168.193.76:/home/al/tractor01_master_manual_endurance_latest.tgz C:\Repos\tractor2025\field_testing\
scp al@192.168.193.76:/home/al/tractor01_master_manual_endurance_latest.txt C:\Repos\tractor2025\field_testing\
Get-FileHash -Algorithm SHA256 C:\Repos\tractor2025\field_testing\tractor01_master_manual_endurance_latest.tgz
```

### Mission 1 to Mission 2 gate

Do not start Mission 2 unless all are true:

- Mission 1 completed rather than merely reaching the old 78% point.
- No phone heartbeat/freshness expiry, unexplained session loss, route/interface
  change, stale green Auto display, or physical watchdog failure occurred.
- The timing analyzer passed and the complete archive was copied off Tractor01.
- The Windows SHA-256 matches the hash in the copied `.txt` record.
- Logs and operator observations show no unresolved steering, transmission,
  power, RTK, heading, or safety issue.
- Battery, fuel, weather, daylight, people, and route conditions remain suitable
  for another supervised run.

If any item fails, stop the field day after preserving evidence.

## Mission 2 — approved consolidated perimeter

Record a new local start time. Start its dashboard from Windows PowerShell:

```powershell
ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 "cd /home/al/tractor2025 && sudo -v && python3 field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260929_consolidated_perimeter_field_test/mission_dashboard_consolidated_perimeter_20260929.py"
```

Again require Heading-F9P preparation and every fresh preflight gate. Reach and
align with the start in guarded Manual, return to Pause, start from the
dashboard, and select guarded Auto only on the green readiness instruction.
Pause immediately for unexpected boundary tracking or obstacle clearance.

After stopping safely, collect and copy the separate perimeter archive:

```bash
cd /home/al/tractor2025
sudo -v
bash field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261020_two_mission_field_day/collect_field_day_run_20261020.sh --run perimeter --since 'YYYY-MM-DD HH:MM:SS'
```

```powershell
scp al@192.168.193.76:/home/al/tractor01_consolidated_perimeter_latest.tgz C:\Repos\tractor2025\field_testing\
scp al@192.168.193.76:/home/al/tractor01_consolidated_perimeter_latest.txt C:\Repos\tractor2025\field_testing\
Get-FileHash -Algorithm SHA256 C:\Repos\tractor2025\field_testing\tractor01_consolidated_perimeter_latest.tgz
```

Mission 2 completion supplies evidence for later boundary reconciliation. It
does not itself approve blades-on operation, unattended operation, or a
Zamboni coverage mission. Coverage planning resumes only after its logs and
observations have been reviewed.
