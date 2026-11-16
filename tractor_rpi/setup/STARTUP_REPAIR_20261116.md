# Tractor01 boot dependency repair — prepared, not installed

**DRAFT — NOT VALIDATED ON TRACTOR01.** Installation and subsequent stationary
activation/reboot require Al's separate approval. No motion is authorized.

## Problem and prepared change

Two observed boots started the disabled legacy Teensy bridge because enabled
`tractor-startup-preflight.service` still requires it. Production RPi/phone
arbitrators are enabled but inactive. Its old wrapper additionally waits for the
retired Wi-Fi service and selects the October preflight/old firmware. See the
[investigation](../../field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261116_legacy_bridge_boot_investigation/REPORT.md).

The prepared canonical startup unit now requires the existing RTCM server and
both production arbitrators, and invokes the new November 16 startup wrapper.
The wrapper calls the unchanged canonical November 9 preflight with the exact
commissioned v5 firmware identity. All current preflight checks are preserved.
The old October wrapper is retained for historical reproducibility.

The network gate requires the current Ethernet/router/ZeroTier paths and all
three production services. It no longer waits for a retired control service or
ntfy reachability. Notices are attempted with bounded timeouts; failed delivery
does not suppress local checks. PASS requires both successful preflight exit
and its exact final `MISSION PREFLIGHT PASS` line. A complete failure remains
failed until inspection; a temporary network/service wait exits 75 and retries.
No notice grants motion authority or claims actual Pause before it is checked.

The phone-link notice still belongs to the unchanged phone service. This new
wrapper supplies startup-check notices; it does not create or send phone keys.
An acknowledgement from ntfy does not prove delivery to the phone.

## Exact installation scope

`prepare_startup_repair_20261116.sh --plan` prints the plan without changing files
or contacting hardware. Its separately gated `--apply` mode:

1. Requires root, explicit engine-off/stationary/blades-off confirmation, a clean
   main checkout and exact full commit. It refuses an active production pair,
   mismatched installed production units or unexpected unit drop-ins.
2. Saves installed unit files, drop-ins, effective definitions, current enablement
   and source commit under a unique `/var/backups/tractor-startup-repair-*` folder,
   with a SHA-256 manifest of the saved regular files.
3. Stops/disables the legacy startup preflight, then the retired bridge and three
   retired Wi-Fi control services. Stops no healthy RTCM or production service.
   Refuses to continue if another process still owns `/dev/teensy`.
4. Moves the old bridge unit into the retained backup and masks its name with a
   persistent `/etc/systemd/system/teensy-bridge.service -> /dev/null` link. This
   blocks remaining historical `Wants=` and manual starts from reviving it.
5. Installs only the migrated `tractor-startup-preflight.service`, reloads
   systemd and enables that unit plus both already-installed production units.
6. Verifies the mask and effective startup dependency names.

The installer **does not start** the production pair or preflight. It does not
install/alter their unit files, credentials, firmware, GPS configuration, IMU
logger, routes, phone/control code or watchdogs. No ntfy publish occurs during
installation. The disabled LED unit is left disabled; its historical bridge
`Wants=` cannot defeat the mask. Any future decision to enable it needs review.

## Review and local validation

```bash
python3 -m unittest tractor_rpi.testing.test_startup_phone_repair_20261116 tractor_rpi.testing.test_startup_preflight_service_20261026 tractor_rpi.testing.test_mission_preflight_20261002
bash -n tractor_rpi/testing/run_startup_phone_preflight_20261116.sh
bash -n tractor_rpi/setup/prepare_startup_repair_20261116.sh
bash tractor_rpi/setup/prepare_startup_repair_20261116.sh --plan
```

Behavior tests substitute all network, notification, service and preflight
commands, covering nominal PASS, ntfy outage, failed preflight with a misleading
PASS marker, zero exit without a marker, missing production service and invalid
wait settings. They also check dependency wiring, installer plan/refusal paths,
backup-before-mutation order and absence of service-start commands. Actual root
installation and boot behavior remain unvalidated until approved live checks.

Prepared validation on Windows: **166 tests passed**, including the existing
production package tests, startup/preflight regression tests and IMU tests.
Both new shell scripts passed Bash syntax checks; read-only installer plan
execution passed. No test contacted ntfy, opened robot hardware, or changed an
installed service. The root installer mutation path has not been run.

## Proposed operator installation — do not run until separately approved

Keep engine off, blades disengaged, tractor stationary, transmission mechanically
released and e-stop accessible. Stop any IMU/field logger before a startup
preflight uses UDP 6009. First review the exact prepared commit and diff.
After source approval, synchronize through GitHub; do not edit Tractor01 files.

```bash
cd /home/al/tractor2025
git status --short --branch
git pull --ff-only origin main
git rev-parse HEAD
bash tractor_rpi/setup/prepare_startup_repair_20261116.sh --plan
```

After confirming the full reviewed SHA, the **separately approved installation**
command is:

```bash
cd /home/al/tractor2025
sudo bash tractor_rpi/setup/prepare_startup_repair_20261116.sh --apply FULL_REVIEWED_COMMIT ENGINE_OFF_STATIONARY_BLADES_OFF
```

Record the printed backup path. Inspect mask, installed unit and enablement
before separately approved reboot/activation. If installation fails, keep the
engine off; do not assume configuration is complete and do not proceed to reboot
or movement. Inspect the error and retained backup before any repair action.

## Stationary reboot acceptance — separately approved

After Al reboots, inspect current-boot records before starting services manually:

- RTCM, production RPi arbitrator and phone arbitrator active/running.
- Old bridge masked/inactive and exactly one owner of `/dev/teensy` (RPi arbitrator).
- Retired Wi-Fi services disabled/static and inactive.
- Startup-preflight effective dependencies reference the production pair only.
- Startup wrapper completes with canonical preflight PASS, or reports the exact
  failed check. Network delay retries are distinguishable from a completed FAIL.
- Phone can open its authenticated local-router control page and request Pause;
  verify returned actual Pause, neutral transmission and zero steering output.
- A startup PASS may require opening the phone and establishing Pause first;
  inspect any completed failure and explicitly rerun the startup-preflight only
  after resolving it. It does not create a phone/operator session itself.
- ntfy notices are useful diagnostics, but local state and preflight are the
  authority. If notices fail with services healthy, investigate delivery separately.
- Require a fresh canonical `MISSION PREFLIGHT PASS` before any separately
  authorized manual IMU run. No logger may share UDP 6009 during that preflight.

## Recovery if the new startup automation needs to be withdrawn

Do not automatically roll back to the known conflicting boot state. Keep engine
off and obtain approval for recovery. The narrow fallback is to stop/disable the
new automatic startup-preflight, retain the old bridge mask, retain production
unit enablement, and use the unchanged canonical preflight manually. No motion
is granted by that fallback. Exact original files/enablement are retained in the
backup for reviewed restoration if required; restoring the retired bridge and
its legacy dependency together would recreate the defect.
