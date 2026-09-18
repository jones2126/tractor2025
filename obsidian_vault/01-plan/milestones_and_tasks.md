# Tractor2025 milestones and tasks

> [!note] How this list is organized
> **Current priorities** is the short working queue. **Next field test** is the
> checklist for the next outing. Sections **A–G** are the canonical backlog by
> subsystem. The command-rich startup and shutdown runbooks remain here for
> field use.
>
> Last workflow review: **2026-09-17**

## Current priorities

1. **Run the 62 Collins partial-rings master mission** from the reviewed
   clear-sky resume point at source waypoint 91.
2. Use the **August 30-style runtime safety gate** for this supervised test:
   RTK Fixed and `headValid` remain mandatory, but carrier, baseline, and
   heading-accuracy values are diagnostic rather than independent runtime stop
   conditions. Recovery has no five-second delay and does not require a
   Manual/Pause-to-AUTO acknowledgement cycle.
3. Preserve both the field log and Pure Pursuit log, then compare tracking,
   heading interruptions, stops, recovery behavior, and cross-track error with
   the successful 2026-08-30 run.
4. Retain the current master's conservative **1.63 m** planning radius. The
   2026-09-16 manual full-lock data measured approximately **1.14 m left** and
   **1.04 m right**; use **1.20 m** only as a candidate for future planning
   after a supervised AUTO validation at 0.5 m/s.
5. After the 62 Collins master mission is proven, use the documented boundary
   workflow to map and build the first mission for **70 Collins Drive**.

## Next field test — 62 Collins partial-rings master

### Objective

Run as much of the reviewed rings-only master mission as conditions safely
allow. This remains a blades-off, closely supervised test. Stripes are deferred
until the inner-ring mission executes acceptably.

### Runtime behavior being tested

- The mission starts only after strict preflight and start-position checks.
- During AUTO, motion stops immediately for:
  - stale/no GPS data;
  - a fatal receiver error;
  - loss of RTK Fixed position; or
  - `headValid=False`.
- Carrier state, heading baseline, and estimated heading accuracy remain
  visible and logged, but they do not independently interrupt this test.
- If RTK Fixed and `headValid=True`, the AUTO mission may continue. If either
  condition is false, the controller immediately commands zero speed. This is
  a controller safety wait, not a dashboard software Pause.
- Once RTK Fixed and `headValid` recover, the controller may resume without a
  five-second timer or handheld acknowledgement cycle.
- Handheld Manual or Pause still prevents motion and freezes path progress.
- The operator may select Manual and drive forward along the understood mission
  route. On returning to AUTO, the controller attempts to reacquire the best
  matching forward path segment; it does not intentionally move mission
  progress backward.
- Dashboard software Pause still requires an explicit **CLEAR PAUSE**.
- Resumption remains subject to bounded, phase-locked forward reacquisition:
  within 2 m of the path, within 60 degrees of path heading, no more than 30 m
  ahead of previous progress, within the current mission phase, and without an
  ambiguous near-tie to another portion of the path. If these checks fail, the
  tractor remains stopped and reports that reacquisition is blocked.

### Success criteria

- Preflight passes with the tractor stationary in Pause.
- Heading USB reports no NMEA sentences during the preflight sample.
- The dashboard and both telemetry logs remain live for the complete test.
- The tractor enters AUTO, follows the expected forward route, and does not
  select a nearby later ring.
- Short heading interruptions recover as expected without creating repeated
  five-second waits or demanding a handheld switch cycle.
- Cross-track error remains acceptable relative to the 2026-08-30 baseline.
- Any stop or operator intervention can be matched to telemetry afterward.

### Stop conditions

Select handheld Pause immediately for unexpected route selection, steering,
speed, obstacle proximity, or recovery behavior. Use the e-stop when necessary.
Do not continue merely to finish the mission.

---

## Field start-up runbook

### RTK base and network

- [ ] Power the RTK base, base router, and Base Starlink Mini. Allow about three
  minutes for `TractorField` to appear.
- [ ] Connect the field laptop to `TractorField` and confirm the RTK base,
  laptop, and tractor on the [GL.iNet router page](http://192.168.10.1/webpages/index.html#/login).
- [ ] Treat the completed 24-hour base survey as the authoritative survey work.
  Do **not** start another survey during routine field startup.
- [ ] Confirm `rtcm_server.service` is active on the base and verify current
  RTCM output. Investigate any unexpected coordinate/configuration change
  before driving.
- [ ] Confirm the prior ESP32 download exists. Use the downloader's `status`
  action to verify the current source log is growing. Run `download_delete`
  only when a new verified download/reset is intended.

Connect to the RTK base from PowerShell:

```powershell
ssh al@192.168.193.88
```

Check the RTCM service and recent output at the RTK-base prompt:

```bash
systemctl is-active rtcm_server.service && journalctl -u rtcm_server.service --since '5 minutes ago' --no-pager | tail -40
```

Check the current ESP32 source log without deleting it:

```bash
python3 /home/al/tractor2025/RTKBase/Bridgeville/esp32_downloader_20260623.py status
```

When a new verified download and source-log reset is intended, run:

```bash
python3 /home/al/tractor2025/RTKBase/Bridgeville/esp32_downloader_20260623.py download_delete
```

Require `SUCCESS: Download and delete completed`, a non-zero line count, and a
saved filename. Then rerun `status` and confirm that the new source log is
growing.

### Tractor inspection and software revision

- [ ] Walk the route and remove new obstacles. Inspect antennas, connectors,
  tires, steering linkage, and visible fluid leaks.
- [ ] Keep the mower deck disengaged and keep people and animals clear.
- [ ] Turn on the handheld, select **Pause**, and confirm radio connectivity.
- [ ] Power tractor01 while it remains in Pause. Keep the e-stop accessible.
- [ ] Confirm LED1 is green and review LED4/RTK status.
- [ ] Confirm the deployed revision:

  ```bash
  cd /home/al/tractor2025 && git log -1 --oneline && git status --short
  ```

- [ ] Do not drive with unexpected local changes or an unreviewed revision.
- [ ] When an approved GitHub update must be installed on tractor01, use this
  complete one-line command and verify the reported revision:

  ```bash
  cd /home/al/tractor2025 && git pull --ff-only origin main && git rev-parse --short HEAD && git status --short
  ```

### Mission preflight

Connect to tractor01:

```powershell
ssh al@192.168.193.76
```

If ZeroTier is unavailable but the laptop is connected to the tractor's local
network, use:

```powershell
ssh al@192.168.1.151
```

Run preflight with the tractor stationary in Pause:

```bash
cd /home/al/tractor2025 && sudo python3 tractor_rpi/testing/mission_preflight_20260804.py --expected-firmware teensy_main_20260914
```

For a more visible field-console run, the following is the complete one-line
version retained from the earlier runbook:

```bash
echo "===== TRACTOR PREFLIGHT START ====="; echo "Computer: $(hostname)"; echo "Starting directory: $(pwd)"; sleep 2; cd /home/al/tractor2025 || { echo "FAIL: Could not open /home/al/tractor2025"; sleep 10; exit 1; }; echo "PASS: Repository opened"; echo "Current directory: $(pwd)"; sleep 2; test -f tractor_rpi/testing/mission_preflight_20260804.py || { echo "FAIL: Pre-flight script not found"; find tractor_rpi -iname '*preflight*' -print; sleep 10; exit 1; }; echo "PASS: Pre-flight script found"; ls -l tractor_rpi/testing/mission_preflight_20260804.py; sleep 2; echo "Running mission pre-flight..."; sudo python3 tractor_rpi/testing/mission_preflight_20260804.py --expected-firmware teensy_main_20260914; RESULT=$?; echo "===== PREFLIGHT FINISHED ====="; echo "Pre-flight exit code: $RESULT"; if [ "$RESULT" -eq 0 ]; then echo "PASS: Mission pre-flight passed"; else echo "FAIL: Do not select Auto"; fi; sleep 10
```

Continue only after `MISSION PREFLIGHT PASS`. The check includes services,
devices, RTCM corrections, RTK position, RELPOSNED rate/flags, heading quality,
Heading-F9P USB NMEA output, stationary ground speed, Teensy modes, steering,
JRK diagnostics, firmware identity, and motor-current telemetry.

The expected NMEA result is:

```text
[PASS] Heading USB NMEA output  none detected during sample
```

If the NMEA counter is absent after updating tractor01, restart the service:

```bash
sudo systemctl restart rtcm-server.service
```

Confirm the tractor services and inspect recent RTCM-server messages:

```bash
systemctl is-active rtcm-server.service teensy-bridge.service && journalctl -u rtcm-server.service --since '5 minutes ago' --no-pager | tail -40
```

### Dashboard and mission start

- [ ] Open the dashboard URL on a phone connected through the field network or
  ZeroTier. Turn up media volume and press **START VOICE GUIDANCE**. In Manual,
  drive toward the announced clock direction. Inside 1.50 m, follow the spoken
  heading correction; stop and select Pause when the dashboard reports ready.
- [ ] Start the dashboard from the field/development computer:

  ```powershell
  ssh -t -i "$env:USERPROFILE\.ssh\id_ed25519_tractor01" al@192.168.193.76 "cd /home/al/tractor2025 && sudo -v && python3 tractor_rpi/pure-pursuit/mission_dashboard_20260910.py"
  ```

- [ ] Open the printed `http://192.168.193.76:8088/?key=...` URL.
- [ ] Verify that the dashboard identifies the **62 Collins partial-rings
  master** and shows live GPS, heading, steering, transmission, and radio data.
- [ ] Keep the handheld in Pause and select **START MISSION**. The launcher
  reruns preflight and validates the reviewed mission, start position, and
  heading.
- [ ] Select AUTO only after the controller is live and the route is clear.
- [ ] If dashboard **PAUSE** was used, first select handheld Pause, press
  **CLEAR PAUSE**, confirm the dashboard shows handheld Pause, and only then
  return to AUTO. A Manual-to-AUTO cycle does not clear dashboard Pause.

---

## Field shut-down and data preservation

- [ ] Stop the mission normally and allow the field and pursuit loggers to
  close cleanly.
- [ ] Record the site, field-log filename, pursuit-log filename, and run start
  time shown in the filenames.
- [ ] While tractor01 remains powered, confirm both files exist, have non-zero
  size, and calculate SHA-256 hashes.
- [ ] Download both logs to the analysis computer and verify their hashes.
- [ ] Collect the RTK-base/ESP32 data while the base remains powered.
- [ ] Retain a verified recovery copy before resetting any ESP32 source log.
- [ ] Record the deployed Git revision with the run.
- [ ] Shut down tractor01 and the RTK base only after all expected files are
  verified.
- [ ] Turn off the handheld and tractor master power, disconnect the charger,
  and store/cover the tractor.

On the Windows analysis computer, open the collection menu from PowerShell:

```powershell
Set-Location C:\Repos\tractor2025
.\field_testing\tools\field_test_analysis_menu_20260726.ps1
```

Use menu option **2** to download the tractor logs and mission package while
tractor01 is still powered. Require non-zero field and Pure Pursuit log row
counts, a hash manifest, and a collection summary. Use option **1** to collect
the RTK-base/ESP32 data while the base remains powered. Option **3** may then be
used to analyze the run and open its generated HTML map.

Only after the downloads and hashes have been verified, shut down the computers
from PowerShell:

```powershell
ssh al@192.168.193.76 "sudo shutdown now"
ssh al@192.168.193.88 "sudo shutdown now"
```

---

## Canonical backlog

### A — Navigation and mission planning

- [ ] Complete the 62 Collins partial-rings master mission end-to-end.
- [ ] Complete A/B testing for the heading solution; see
  [`potential-Dual-F9PHeading-A-B-Test.md`](../02-testing/potential-Dual-F9PHeading-A-B-Test.md).
- [ ] Review the resulting cross-track error and recovery behavior against the
  2026-08-30 combined mission.
- [ ] Add cutting stripes only after the rings-only mission is satisfactory.
- [ ] Develop obstacle-aware polygon coverage, beginning with the telephone
  pole and its additional 24-inch exclusion distance.
- [ ] Build a general safe transition-path generator from the tractor's current
  pose to a mission start; evaluate Dubins-style paths where appropriate.
- [ ] Preserve recorded, intentionally driven connectors as preferred safe
  routes when available.
- [ ] Capture and build the 70 Collins Drive boundary/master mission.
- [ ] Define acceptance limits for cross-track error and use the same scoring
  method for every comparable run.
- [ ] Investigate the historical target-table error observed on 2026-07-22 if
  it remains reproducible in current code.

### B — GPS, RTK, and heading

- [x] Complete the permanent-base 24-hour survey data collection.
- [ ] Weatherproof and securely remount the permanent-base electronics and
  antenna without changing the surveyed antenna position.
- [ ] Run the dual-F9P clear-sky audit after the Heading USB NMEA cleanup.
- [ ] Confirm across repeated tests whether current heading performance is
  adequate or whether migration to the dual-antenna X20D should become the
  primary plan.
- [ ] If pursuing X20D, update its RTCM server with current diagnostics,
  logging, dashboard, and preflight fields before deployment.
- [ ] Improve `configure_heading_f9p_20260727.py` handling and reporting of
  missed UBX-CFG-VALSET acknowledgements while retaining verified readback as
  the authoritative result.
- [ ] Add direct 12 V monitoring at the RTK-base ESP32.
- [ ] Update the base boot notification to include all IP addresses and
  ZeroTier reachability.
- [ ] Add weather-resistant 12 V switches for the base components.

### C — Steering, radio, and transmission

- [x] Set the authoritative JRK neutral target to **2836** in current firmware,
  preflight, and field procedures.
- [x] Calibrate the steering physical center and guarded mechanical limits.
- [x] Analyze the 2026-09-16 manual full-lock telemetry: approximately 1.14 m
  left and 1.04 m right. Retain 1.63 m for the present master mission.
- [ ] Validate a candidate common 1.20 m planning radius in supervised AUTO at
  0.5 m/s before using it in future mission geometry.
- [ ] Tune steering control beyond the current `kp=1.0`, `ki=0`, `kd=0`
  behavior after the present navigation baseline is repeatable.
- [ ] Add radio pushbutton states to the field logger as boundary/segment
  breadcrumb markers.
- [ ] Evaluate a magnetic steering-angle sensor as a more robust alternative to
  the potentiometer.
- [ ] Add safe on-tractor recovery controls for left, right, forward, and
  reverse if the radio/potentiometer fails.
- [ ] Connect the wheel-odometry units and compare their data with RTK/heading
  interruptions.
- [ ] Keep the optional Ring 13 1.2/1.5/1.8 m/s firmware test deferred until
  the 1.0 m/s master mission is stable.

### D — Tractor computer, services, and web tools

- [x] Provide a live web dashboard that starts, pauses, monitors, and logs the
  partial-rings master mission.
- [x] Add consolidated stationary preflight with automatic GPS/heading failure
  diagnostics.
- [x] Add live Heading-F9P NMEA detection to `rtcm-server` and preflight.
- [x] Add phone-based spoken start-position and heading guidance to the mission
  dashboard without adding an external notification dependency or motion path.
- [ ] Copy the deployed tractor01 systemd unit files into
  `tractor_rpi/setup/` and verify them against the installed units.
- [ ] Install and verify the service set on tractor02 when tractor02 work
  resumes.
- [ ] Add tractor01 CPU utilization to the field telemetry.
- [ ] Complete browser teleoperation and OAK-D mounting while keeping DepthAI
  pinned to `2.30.0.0`.

### E — Data collection and analysis

- [x] Capture RELPOSNED flags, carrier, baseline, heading accuracy, frame
  counters/timestamps, and per-receiver satellite/C/N0 diagnostics.
- [x] Resolve historical `jrk_current` as position feedback and add explicit
  JRK motor-current/peak/current-valid telemetry.
- [x] Create reusable heading-carrier history analysis with fixed/non-Fixed
  duration and episode comparisons.
- [x] Analyze and archive the 2026-07-24 run as an early consolidated-workflow
  example.
- [ ] Validate the automated run-collection workflow during a complete field
  shutdown, including hashes, mission package, both tractor logs, and RTK-base
  ESP32 data.
- [ ] Standardize a single report containing RTK-loss, heading-validity,
  carrier, cross-track RMS/median/p95/max, speed, steering, and recovery events.
- [ ] Add radio breadcrumb inputs, CPU utilization, and wheel odometry to the
  logger.
- [ ] Decide whether TimescaleDB/Grafana remains useful after the file-based
  workflow is fully validated.
- [ ] Migrate RPiNAS to Ubuntu 26.04 LTS when appropriate.

### F — Electronics and fabrication

- [ ] Perform the permanent-board wiring dry run using real cable bend radii.
- [ ] Design the Teensy/NRF24 carrier PCB and RPi power/Andon interface PCB.
- [ ] Complete the IBT-2 Gen 2 PCB and current-sense/enable wiring plan.
- [ ] Add tractor battery-voltage telemetry.
- [ ] Print the JRK bracket and fabricate weather-resistant electronics
  enclosures.
- [ ] Create and cut the final electronics mounting board.
- [ ] Fabricate the cutting-deck bracket.

### G — Maintenance and documentation

- [ ] Service the mower: tune-up kit, spark plug, oil/filter, blades, grease,
  and general inspection.
- [ ] Write the next dated field-test summary after the master-mission retry.
- [ ] Prepare the next project video update.
- [ ] Keep confirmed code, mission artifacts, analysis, and documentation
  synchronized with GitHub after each test session.

---

## Recently completed

- [x] Completed the 24-hour permanent-base survey data collection.
- [x] Captured the 2026-09-15 multi-boundary field drive and labeled its
  segments, safe connectors, fields, and telephone-pole obstacle.
- [x] Built the rings-only 62 Collins master mission and removed the two unsafe
  near-360-degree connector loops.
- [x] Added the waypoint-91 clear-sky resume mission and bounded phase-locked
  path reacquisition.
- [x] Integrated the reviewed master mission into the live web dashboard.
- [x] Captured and preserved the 2026-09-16 field and pursuit telemetry.
- [x] Compared recent heading-carrier reliability across 30 unique telemetry
  runs.
- [x] Audited the dual-F9P moving-base configuration and isolated Heading USB
  NMEA cleanup from MSM/UART changes.
- [x] Added Heading USB NMEA detection to preflight.
- [x] Restored August 30-style runtime gating for the supervised partial-rings
  test while retaining strict startup checks and bounded reacquisition.
- [x] Calibrated current steering center/mechanical limits and measured the
  final manual full-lock radii at approximately 1.14 m left and 1.04 m right.
- [x] Confirmed `teensy_main_20260914` and JRK neutral target **2836** in
  preflight.

## Key references

- `obsidian_vault/02-testing/20260916-dual-f9p-heading-investigation-handoff.md`
- `field_testing/sites/62_Collins_multi_boundary_20260915/README.md`
- `field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20260915_master_boundary_replay/README.md`
- `tractor_rpi/pure-pursuit/MISSION_DASHBOARD_20260910.md`
- `field_testing/sites/62_Collins_multi_boundary_20260915/analysis/HEADING_CARRIER_HISTORY_20260916.md`
