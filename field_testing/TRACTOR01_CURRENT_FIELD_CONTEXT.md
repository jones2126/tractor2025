# Tractor01 current field context

Last updated: 2026-11-19

This is durable operational context for assistants helping Al with Tractor01.
Read it before proposing field commands. It records established facts and
required command paths; it does not authorize motion, blades, firmware changes,
service changes, or a mission.

## Command-path rule

PuTTY normally opens in `/home/al`, while the Tractor01 repository is:

```text
/home/al/tractor2025
```

Never assume the shell is already inside the repository. Every proposed
repository command must either:

1. begin with `cd /home/al/tractor2025`; or
2. use an absolute path below `/home/al/tractor2025`.

## Current installed runtime — November 19

This section supersedes older dated preparation/installation checkpoints below.
Canonical stationary preflight, engine off, stopped in Pause, blades disengaged:

```bash
cd /home/al/tractor2025 && sudo python3 -m tractor_rpi.tractor_preflight --expected-firmware teensy_main_20261110_rpi_command_lease_v5 --confirm-engine-off --heading-wait-seconds 120
```

Require `MISSION PREFLIGHT PASS`; a FAIL never permits Auto. A PASS does not
itself authorize movement. Confirm current commit/clean tree before field work.

- Production producer: `calc-publish-position.service`, alias `rtcm-server.service`.
  Owns F9Ps, IMU, encoders and RTCM forwarding. Its ExecStartPre still writes the
  Heading moving-base profile to RAM and independently reads back 22 values.
- Production bridge: `tractor-teensy-bridge.service`, alias
  `tractor-rpi-arbitrator.service`, sole `/dev/teensy` owner, unchanged v5 command
  safety selection. Retired `teensy-bridge.service` remains masked.
- Phone: `tractor-phone-arbitrator.service`, local TLS `https://192.168.8.20:8765`.
  Depends on bridge alias; stopping bridge can stop phone. Local-control-ready
  ntfy does not mean full preflight or calibration passed.
- `tractor-telemetry-recorder.service`: independent boot recorder, sole UDP 6009
  owner; receives status UDP 6003, router TCP 6005 and native sensor/pose/event IPC.
  Daily Eastern folders `/home/al/field_logs/YYYY-MM-DD/telemetry_<UTC>/`, 15-minute
  segments with closed hashes. No separate legacy field or standalone IMU logger.
- `tractor-startup-preflight.service`: consolidated one-shot readiness/full test,
  300-second readiness wait, 600-second service timeout. An earlier failed unit
  state persists after a later successful manual preflight; it is not a live
  reflection of the last manually run test. Local reports survive ntfy failures.
- Four raw GNSS copies still publish at about 20 Hz to 6002/6009/6010/6013.
  Each unicast consumer gets its own port. Experimental pose is IPC-recorded,
  `valid_for_control=False`; Pure Pursuit still uses raw GNSS, not fused pose.
- Left Teensy 3.2 serial `5879620`; right encoder serial
  `F5CB3C7E51514A4E43202020FF142006`; tractor Teensy 4.1 serial `16883790`.
  Resolve USB identity, not ttyACM index. Left USB cold-boot silence remains an
  unresolved issue; targeted reauthorization restored output without reflashing.
- Commissioned steering: right191/centre525/left885; JRK neutral2836; firmware
  `teensy_main_20261110_rpi_command_lease_v5`.
- Ethernet192.168.8.20/24 gateway192.168.8.1; maintenance/dashboard via
  ZeroTier192.168.193.76 (dashboard normally8088). `0.0.0.0` is a listen address,
  never the address the operator enters. Review dashboard may intentionally show
  no controller before it is launched.
- Frozen November4 launcher uses a competing private logger/old preflight and
  must not run unchanged with the consolidated recorder. No Auto successor yet.

Read AGENTS.md's startup walkthrough for exact waits/notices/calibration gates.
Do not restart healthy services just to rediscover current-boot readback proof.

## Raspberry Pi throttling monitoring

Operator-confirmed live Tractor01 behavior as of 2026-11-11:

- Boot produces an ntfy notice containing `vcgencmd get_throttled` state.
- A healthy current boot reports `throttled=0x0`.
- If a power/throttling problem develops during operation, repeated ntfy
  notices are sent.
- Al reported that this state is also retained in operational telemetry.

Do not ask Al to rerun `vcgencmd get_throttled` merely to discover a value he
has already supplied for the current boot. It remains legitimate as an explicit
preflight gate or when diagnosing a new power alert.

The exact installed notifier/telemetry implementation was not found in GitHub
`main` during the 2026-11-11 review. Treat the behavior above as verified
live-system context, but do not invent a repository service or field name.
Trace the installed unit/script on Tractor01 before changing or documenting its
implementation details.

## Dashboard interpretation before controller start

The review dashboard intentionally cannot start Pure Pursuit. Before the
mission controller exists:

- `Last fresh controller telemetry: never` is expected.
- `Recovery lockout: waiting for stable control` is expected in the review
  dashboard because its legacy stability calculation requires controller
  telemetry that the disabled dashboard never starts.
- Those two labels alone do not indicate a phone, Teensy, GPS, or actuator
  fault.

Live bridge values must still show Pause, zero steering PWM, neutral
transmission, fresh RTK Fixed position, fixed/valid heading, acceptable heading
accuracy and baseline, and no actuator fault.

## Current outer-perimeter package

The current reviewed boundary package is:

```text
field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/
20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED
```

Route identity:

- Stable asset: `62COLLINS-OUTER-PERIMETER-20260928`
- 3,462 waypoints
- 689.949 m
- 1.00 m/s
- clockwise
- route SHA-256:
  `b47660b4c74d5e26ca5d9448f069fd7193c068cf62e3c0b62d5c083d9aed4d64`

Keep review/dashboard inspection, preflight approval, physical movement, and
blade authorization as separate decisions.

## Current 2026-11-11 commissioning evidence

- Firmware v5 compiled and uploaded successfully from GitHub `main` commit
  `755bde9`.
- Production services restarted successfully after reboot.
- Straight steering settled at pot `526` for center target `525`, with zero PWM
  in Pause and no steering fault.
- Full-left test: pot `881`, target `885`, zero PWM, no fault.
- Full-right test: pot `194`, target `191`, zero PWM, no fault.
- JRK supply telemetry: `12.57 V`, valid; handheld multimeter: `12.5 V`.
- Transmission actuator sweep behaved as expected with engine off and the
  transmission physically disengaged.
- Phone E-stop actuated and released the relay audibly.

These stationary commissioning observations do not themselves authorize a
mission or physical movement.

## November 11 outer-perimeter rerun and shutdown

- The production phone-controlled outer-perimeter mission was run with the
  cleaned Teensy firmware and stopped at approximately 380.857 m route
  progress / waypoint 1917.
- The complete evidence archive was copied to Windows and verified at SHA-256
  `5c101e8bdf5adb11e2aa4ae4e132440a70020712708868ebd9879fee18a491b4`.
- The historical replay is under
  `field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261111_phone_outer_perimeter_replay/`.
  It records 18 grouped interruptions and 22 distinct raw threshold-breach
  episodes. See `REPORT.md` and `safety_condition_loss_episodes.csv` there.
- Immediately before shutdown, Tractor01, GitHub, and the Windows checkout were
  clean on `main` at `e4c7f73`. Tractor01 was then shut down.
- Repository work prepared after shutdown has not been pulled to Tractor01.
  Reconfirm commit identity and a clean tree before the next field session.
- The continuous battery-voltage logger is a repository candidate only. It is
  **DRAFT - UNTESTED IN THE FIELD**, has not been installed, and has not changed
  any Tractor01 service.

## Prepared post-run collection change — not deployed

Local Windows work prepared on 2026-11-14 adds a general post-run evidence
collector and connects it to the outer-perimeter launcher's exit handler. The
candidate assigns the field and Pure Pursuit CSVs one shared run timestamp,
records their exact paths in a JSON run manifest, stops the logger before
collection, and creates an internally and externally hashed archive that
includes applicable `/var/log/pilog` evidence. The dashboard waits for the
collector and displays a clear pass or failure result.

On 2026-11-16, the prepared change passed 133 software tests, 10 arbitrator
fault simulations, and package verification. It is being synchronized through
GitHub to Tractor01 with this revision. No service restart, installation, or
live stationary collection validation is included. Automatic evidence collection
remains DRAFT - UNTESTED IN THE FIELD; verify collection manually until a
separately approved stationary validation succeeds.

## Historical checkpoints (dates matter)

The following entries preserve what was known at each checkpoint. Their temporary
service states and uninstalled labels do not override the current runtime above.

## ISM330DHCX observation logger — manual capture completed November 17

`tractor_rpi/testing/imu/ism330dhcx_field_logger_20261116.py` and its adjacent
README provide a standalone foreground IMU/GNSS recording experiment. It is
an observation-only logger, separately stationary-tested and manually exercised
as recorded below. It changes no production control code,
serial ownership, services, or watchdogs. The logger configures only the IMU on
I2C bus 1 at 0x6b and exclusively receives localhost GPS UDP 6009; do not run the
existing field logger concurrently. It logs under `/home/al/field_logs/imu/`
with per-run calibration, raw/scaled samples, GPS source/arrival ages, relative
yaw and once-anchored compass comparison, exact commit metadata and SHA-256
checksums. Stopping the logger does not send Pause or stop the tractor.

The original observation logger passed stationary validation on November 16 and
recorded the approved November 17 manual run. Capture:
`/home/al/field_logs/imu/imu_20261117T161449_639816Z`, 154,769 rows, logger commit
`d18cc6088916460aaeb7bc244f662a9aad66868e`. All original hashes and the independent
Windows copy were verified. Raw originals remain on Tractor01; local raw copies
under site `runs/20261117_imu_manual/` remain git-ignored. Replay and analysis are
in `analysis/20261117_imu_manual_replay/` and `analysis/20261117_imu_manual_analysis/`.
Those results validate only that observation experiment, not navigation fusion
or a degraded-motion policy. The newly prepared combined producer below is a
different, field-untested program.

## November 16 boot regression — repaired and reboot checked

Two inspected boots started the disabled legacy `teensy-bridge.service` while
both enabled production arbitrators remained inactive. The enabled installed
`tractor-startup-preflight.service` still requires the retired bridge, and its
October wrapper waits for retired `tractor-wifi-control.service` and selects
obsolete preflight/firmware defaults. See the evidence/report under
`field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261116_legacy_bridge_boot_investigation/`.

Al approved and installed the repair on November 16. Backup:
`/var/backups/tractor-startup-repair-20261116T201755Z`. The legacy bridge is now
masked/inactive. A reboot verified both production arbitrators active and the
RPi arbitrator as sole `/dev/teensy` owner. The migrated startup preflight uses
`run_startup_phone_preflight_20261116.sh`; it initially failed GPS convergence
checks rather than falsely passing. Al supplied a complete canonical preflight
PASS on November 17 with RTK Fixed, fixed/valid heading, baseline 1.0509 m,
actual Pause, zero steering PWM, neutral 2836 and exact v5 firmware. Do not
unmask the old bridge. Preserve the existing heading ExecStartPre/readback hook.
Do not infer a later boot's health from this record; verify current-boot state.

## November 17 combined pose-producer candidate — NOT installed

`tractor_rpi/calc-publish-position-20261117.py` is prepared for a separately
approved November 18 stationary check and manual data-flow run. Windows and
GitHub are synchronized at the end of preparation; Tractor01 is deliberately
NOT pulled or changed on November 17. Read
`tractor_rpi/setup/CALC_PUBLISH_POSITION_20261117.md` before proposing deployment.

The candidate uses the agreed background readers/main-loop ingestion split,
retains raw GNSS on the same localhost UDP 6002/6009/6010/6013 at 20 Hz, omits Pi
Wi-Fi polling, and records IMU, encoder and non-authoritative shadow-pose evidence
under `/home/al/field_logs/position/`. Raw position/fix/heading fields are never
replaced by the experiment. Wheel signs are operator-confirmed, but scales and
the approximately 7% left/right difference remain unverified. USB serial IDs
are used instead of assuming ttyACM numbers or unapplied udev aliases.

The optional service drop-in is only a prepared field-test override of the
existing `rtcm-server.service`, under `/run/systemd/system` for the current boot
only. It preserves the heading startup hook, requires engine-off stationary
calibration and disables automatic restart. Reboot clears the override; close
and verify evidence before ending the field session. It has NOT been installed.
Do not run both GPS owners
or two IMU acquisition programs concurrently. Installation, restart, stationary
validation, manual movement and blades remain separate approvals. No 3 m/30 s
continuation policy or safety watchdog change is included.

## November 18 field recovery and consolidation candidate

Al supplied a failed five-minute preflight with RTK position Float and Heading
Float, fresh corrections and adequate satellites. After one operator-executed
restart of the existing `rtcm-server.service`, the full preflight passed:
position RTK Fixed, Heading Fixed in 100% of samples, baseline 1.1472 m,
Heading accuracy 0.514 degrees, 29 Heading satellites, actual Pause and neutral
2836. This establishes recovery after restart, not an isolated root cause.
No replacement producer was installed during that recovery.

Al subsequently approved local implementation of the consolidated startup,
continuous recorder and preflight design. Al installed it November 18 with backup
`/var/backups/tractor-consolidation-20261118T181841Z` and began stationary checks.
It remains **DRAFT — STATIONARY VALIDATION INCOMPLETE; NOT MOTION VALIDATED**.
Read `tractor_rpi/setup/CONSOLIDATED_STARTUP_REVIEW.md` before proceeding.
Installed canonical service names are
`calc-publish-position.service`, `tractor-teensy-bridge.service` (same existing
command-selection implementation) and `tractor-telemetry-recorder.service`.
Old production names would become aliases; the retired `teensy-bridge.service`
must remain masked. The installer initially stopped phone control through its
bridge dependency; Al explicitly started it again and confirmed reconnection.

Candidate storage is `/home/al/field_logs/YYYY-MM-DD/telemetry_<UTC>/`; the recorder
starts at boot, owns UDP 6009, and supplies observations to canonical
`tractor_rpi/tractor_preflight.py` through local IPC. Do not start old private
field loggers or use the old UDP-binding preflight concurrently after deployment.
Frozen mission packages are not yet migrated and must not be launched unchanged
with the new boot recorder. No new controller, firmware or motion policy is included.

### Left encoder recovery in progress

The new position service repeatedly exited after five seconds with no left encoder
serial data, causing unwanted GPS reconfiguration on each automatic restart. The
recorder and right encoder/IMU were active. A follow-up source correction reports
silence as stale encoder health while keeping GNSS acquisition running; it is not
yet pulled/validated on Tractor01. Al stopped startup-preflight, position, phone and bridge
services and physically disconnected the main Teensy 4.1 (serial 16883790).
Recorder was left running. Do not restart the control stack or reboot until the
left encoder firmware is restored, output verified and main controller reconnected.

Left board is operator-confirmed Teensy 3.1/3.2, USB serial 5879620, physical USB
location observed 2-2.3. Its port changed from ttyACM4 to ttyACM0 after programming
without moving cables; resolve identity, never assume a fixed ttyACM index.
Al built and uploaded Blink with the left Program button, then successfully
uploaded changed double-flash Blink WITHOUT a button press while the 4.1 remained
disconnected. This establishes upload from responsive Blink, not firmware loss or
recovery from hung I2C code. Al then successfully uploaded the actual left-encoder
source without a button press and supplied five seconds of valid ENC,L data at
10 Hz with no ERR lines. Raw parked jitter was approximately +/-4 ticks per 100 ms.
The follow-up startup check permits bounded jitter (8 ticks per 100 ms, scaled by
actual sample interval), while still requiring fresh valid wheels, stationary GNSS,
Pause/neutral and the established IMU checks. All raw ticks remain recorded;
there is no wheel-distance calibration or motion-authority change. 247 software
regressions passed; live validation of these two producer corrections is pending.

Project `tractor_rpi/testing/encoders/left_teensy32/` contains preserved left
encoder source and separate Blink environments. Automatic upload refuses any
other connected Teensy; keep 4.1 disconnected for `left_encoder_auto` restore.
No left-encoder calculation, scale, filter or I2C logic was changed. Future
targeted reboot/USB isolation with both Teensys connected is not implemented.

### Later November 18 stationary preflight

After reconnecting the control Teensy, pulling a5afdf8 and starting the four
services, Al supplied a report with recorder error=None/losses=0, both encoder
streams near 10 Hz, but both GNSS solutions Float. A further position-service
restart did not immediately recover Fixed. A later 60-second preflight showed
position RTK Fixed, gyro bias calibrated=True, Heading still Float and no heading
anchor. No field movement is established by these reports.

That later report exposed a preflight timestamp race: freshness was assessed
against a time saved before fetching the observation windows, so newly arriving
records could appear future-dated. Source correction assesses age after fetching
the windows; 34 focused tests passed, including an observation-arrives-during-query
case. Pulling this preflight-only correction requires no service restart and does
not resolve the remaining genuine Heading Float/alignment failure.

### Approximately 15:12 November 18: post-reboot full PASS after base restart

Al restarted the external RTK base service, then supplied a full canonical
preflight PASS saved as
`/home/al/field_logs/2026-11-18/preflight_151148_057129.txt`.
Position RTK Fixed; Heading carrier Fixed in 100% of snapshots; baseline 1.2335 m;
Heading accuracy 0.466 degrees; 31 Heading satellites used; correction age 1.40 s.
Gyro calibration/alignment and both encoders passed (50 records/5 s each), actual
Pause/neutral 2836 and commissioned v5 firmware passed. Recorder session
`723efd1adc3e47048616d5197fb40e07`, current segment
`/home/al/field_logs/2026-11-18/telemetry_201128_507836Z`, error=None/losses=0,
approximately 32.8 GB free. Recorder rotates segments automatically; retain the
whole relevant daily session, not only this last directory.

This establishes eventual post-reboot readiness after intervention, not unattended
boot reliability. Left encoder had required USB power-cycling after reboot without
reflashing; its existing firmware resumed output. GNSS recovery followed the base
restart, but root cause is not isolated. Al reports the tractor remained unmoved;
the changing reported Fixed baseline remains an analysis question. No manual
engine start or field movement is established by this PASS alone.

### Second cold-start left-encoder isolation

Al reproduced left-encoder silence after another clean shutdown/ten-second Pi
power removal with cables left connected. Kernel enumeration was normal: serial
5879620 at USB 2-2.3 around 4.7 s, ttyACM4 by 8.1 s, producer startup around 30 s.
No USB disconnect/reset/enumeration error was found in the uploaded boot capture.
Linux cdc_acm interface unbind/rebind did not restore serial output.

With producer stopped and board still powered, guarded USB-device deauthorization
and reauthorization (2-2.3/authorized: 0, wait two seconds, 1) DID restore 10 Hz
ENC,L output. Device clock resumed around 763312 ms rather than zero, supporting
recovery without whole-MCU reset or reflashing. See
`obsidian_vault/02-testing/20261118-left-encoder-boot-investigation.md`.
No automatic startup recovery has been implemented. At this checkpoint the
position producer is still stopped; bridge/phone/recorder were left running.

## November 19 manual sensor run — collected and replayed

Al recovered the silent left encoder with guarded USB reauthorization, started
the producer and supplied a full canonical preflight PASS (10:57 EST). He reported
engine running at 11:01 after two start attempts, then CW/CCW squares and hard
turns with approximately ten-second pauses; parked/off/Pause around 11:14.
This establishes one manual capture, not reliable unattended boot or Auto fusion.

Three closed telemetry segments covering preparation and the drive were copied
to Windows. Archive SHA-256 and all inner manifests independently verified;
originals remain on Tractor01. Raw copy is ignored under site
`runs/20261119_manual/`; derived replay is
`analysis/20261119_manual_sensor_replay/`. See
`obsidian_vault/02-testing/20261119-manual-imu-gps-encoder-run.md` for provenance,
operator markers and initial stream-continuity observations. Encoder scale and
turn-clearance analysis remain pending. No control policy was changed.
