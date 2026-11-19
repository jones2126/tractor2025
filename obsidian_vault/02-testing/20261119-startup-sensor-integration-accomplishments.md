# November 16–19, 2026: startup, telemetry and pose preparation

Recorded November 19 in America/New_York. This summarizes operator reports,
preserved field evidence and repository implementation. A manual sensor capture
is not validation of fused navigation, autonomous driving or degraded motion.

## What was accomplished

### November 16: startup conflict identified and repaired

The retired standalone `teensy-bridge.service` was being started through an old
startup preflight dependency while the intended production bridge/phone services
were inactive. This was a systemd startup dependency issue, not evidence that
inherited old parser source was an independently running service. The repair
masked the retired bridge and restored the intended startup path. Operator backup:
`/var/backups/tractor-startup-repair-20261116T201755Z`. Subsequent reboot checks
and a November 17 full preflight established recovery for those tested boots.

The standalone ISM330DHCX observation logger was exercised and preserved raw
IMU/GNSS data with metadata and SHA-256 manifests. It was an experiment, not a
controller input. Steering calibration is right 191, centre 525, left 885; preserve
that order rather than ambiguous left/centre/right shorthand.

### November 17: first manual IMU run, replay and architecture discussion

Full preflight passed. Al reported engine start 11:22, parked/off 11:37 and stopped
the logger 11:38. Capture `imu_20261117T161449_639816Z` contains 154,769 IMU rows.
The raw Windows copy was independently verified. A self-contained offline HTML
replay and separate analysis package were built under the site's November 17
analysis directories, preserving source hashes and timing-gap limitations.

Al described two square-ish paths: CW at about 84% drive, CCW at 100%, backing
before the second square, then hard left/right turns. Those percentages belong
to November 17 only. Turn dimensions should inform future path planning, with
antenna path distinguished from the tractor/deck swept clearance.

Agreed design direction: Pure Pursuit should consume the best supported pose
from a dedicated producer rather than implement sensor fusion itself. Estimate
position and heading from available evidence; separately decide how long motion
may remain authorized. Healthy GNSS can periodically anchor IMU heading; a
five-second experimental interval was selected for evaluation. Gyro bias
calibration, heading alignment and recovery of GNSS position are different tasks.

### November 18: continuous recording and startup consolidation installed

Al requested that manual-driving telemetry be captured automatically at boot,
without starting several private loggers or running a second readiness procedure.
The implementation separates acquisition/estimation, continuous recording and
canonical preflight. The service names were changed to describe their roles:

| Current service | Role / compatibility alias |
|---|---|
| `calc-publish-position.service` | F9Ps, RTCM, IMU, encoder acquisition and shadow pose; alias `rtcm-server.service` |
| `tractor-teensy-bridge.service` | Same established control bridge/safety selection; alias `tractor-rpi-arbitrator.service` |
| `tractor-phone-arbitrator.service` | Existing local authenticated handheld control |
| `tractor-telemetry-recorder.service` | Independent boot recording |
| `tractor-startup-preflight.service` | Readiness wait and canonical full preflight |

Installation backup: `/var/backups/tractor-consolidation-20261118T181841Z`.
The installer did not start services; Al subsequently activated them while
stationary. Stopping the bridge also stopped dependent phone control; explicit
phone start restored the handheld. Retired `teensy-bridge.service` stays masked.

Raw telemetry lives outside Git in `/home/al/field_logs/YYYY-MM-DD/`, Eastern
daily folders and UTC segment names. Native IMU/encoders, one raw GNSS copy,
full bridge/JRK status, router telemetry, events and experimental pose are kept.
Fifteen-minute segments and manual seal produce closed files with checksums;
recording continues into the next segment. Preflight queries read-only recorder
windows rather than competing for UDP 6009. Ports 6002/6009/6010/6013 intentionally
retain duplicate raw GNSS snapshots for separate unicast consumers.

Implementation fixes during commissioning:

- Encoder silence marks readiness stale without repeatedly killing/restarting
  the GNSS producer and rewriting the Heading startup profile.
- Stationary encoder allowance accepts observed small parked jitter while
  preserving raw counts. This is not a distance calibration or motion deadband.
- Preflight computes freshness after collecting observation windows, avoiding
  a race that incorrectly labelled newly arrived rows future-dated.

### Encoder restoration and boot investigation

Left board confirmed Teensy 3.1/3.2, USB serial 5879620. The PlatformIO project
`tractor_rpi/testing/encoders/left_teensy32/` preserves the actual encoder source
and separate Blink environments. Blink, changed Blink and restored encoder
firmware were successfully built/uploaded on Tractor01. Automatic soft-reboot
upload succeeded without pressing Program while the tractor Teensy 4.1 was
physically disconnected. The guard refuses other connected Teensys; selective
automatic upload with both boards connected has NOT been established.

ttyACM numbers changed without moving cables. Resolve USB serial identity.
After cold Pi boot the left board enumerated normally but stayed silent. A
physical USB power cycle restored it without reflashing. cdc_acm interface
unbind/rebind failed to restore output; whole USB-device authorization0/wait2s/1
restored10 Hz output while the board's millisecond clock continued. This argues
against assuming lost firmware. USB/firmware/host startup interaction remains
unisolated; no automatic boot recovery was added. See the November 18 boot note.

### GNSS findings and limits

The Heading F9P receives its RAM-only moving-base startup profile and independent
22-key readback at producer service start. A separate profile review matched
the inspected targeted settings. Fresh forwarded correction bytes, recent
correction age and about 31 used satellites coexisted with Float position/heading.
Restarting the tractor producer or external RTK-base service preceded recovery
in different trials. These observations do not isolate the cause. A possible
base message-buffer issue remains a hypothesis, not a demonstrated diagnosis.

The former and replacement producers both achieved Fixed during comparison.
No evidence established that simply renaming/replacing the program cured GNSS.
On November 19 Al moved to an area with fewer perceived multipath concerns and
GNSS passed; the move is context, not proof of a root cause. Reported fixed
baseline variation warrants analysis against actual mounting/receiver evidence.

### November 19: complete manual sensor capture and replay

Fresh boot again required targeted left USB recovery. Full canonical preflight
passed at 10:57 EST: Fixed position/heading, calibrated/aligned IMU, both wheel
streams, Pause/neutral 2836 and exact commissioned v5 firmware. Report:
`preflight_105734_413335.txt`.

Operator markers:10:59 engine off/manual steering check;11:01 running after two
start attempts; approximately11:14 parked/off/Pause. CW/CCW squares and hard
turns had approximately ten-second pauses. Unknown seconds and inferred stop
markers must not be treated as exact maneuver boundaries.

The archive `manual_run_20261119.tgz` (40,090,094 bytes), SHA-256
`eb531f2c0504785f5c034073360533cef2157912524818e82a5b1bb50c0d2d19`, and all
three inner segment manifests were independently verified on Windows. Closed
segments are `telemetry_154324_711636Z`, `telemetry_155824_744702Z` and
`telemetry_161324_745916Z`. Originals remain on Tractor01; ignored raw copy is
under site `runs/20261119_manual/`. No cleanup is authorized by collection.

The new replay under site `analysis/20261119_manual_sensor_replay/` uses the prior
IMU replay layout and shared mission map/style. GPS health, inertial data, both
wheel speeds/ticks, recorded steering, selected manual drive and JRK VIN share
one timeline. It uses 120,742 full-rate IMU rows for the selected10:57–11:16 window,
displayed as 11,308 points with bin extrema. Controls passed offline JavaScript
checks and seven existing replay regression tests passed. Wheel scale remains
provisional. Health percentages include preparation and must not be relabelled
as drive-only percentages.

For the reported11:01–11:14 window, raw continuity checks found 83,574 IMU rows,
7,800 left-wheel rows and7,773 right-wheel rows. Maximum source gaps were0.01321s,
0.10345s and0.10448s respectively. GNSS recorder arrival maximum gap0.20846s;
status maximum0.18280s. These establish recorded continuity, not pose accuracy.

## Current boundaries and next work

The pose is recorded as shadow-only, `valid_for_control=False`. Ports 6002/6009/
6010/6013 still publish raw GNSS. Wheel signs/scales, timing, IMU mounting/sign,
lever arms, turn dimensions, short-horizon prediction error and recovery behavior
need measured review before feeding a supported pose to Pure Pursuit.

Al's proposed degraded allowance is 3 m or 30 s at no more than 0.3 m/s. Treat them
as separate maximum budgets with earliest expiry; 3 m takes 10 s at that speed.
The estimate producer reports evidence/support/age/travel; the RPi motion
supervisor separately enforces any approved continuation policy. Existing500ms
local freshness/lease checks are not extended. No such policy is commissioned.

Local-control-ready notification, sensor readiness, complete preflight PASS and
motion authorization are distinct. ntfy is best effort; local dated results
remain authoritative evidence. Failed startup one-shot state does not update
when a later manual preflight passes. Automatic boot recovery remains unfinished.

## November 19 synchronization audit

Before these documentation/replay changes: Windows HEAD, GitHub main and clean
Tractor01 main all matched `06e2fbe32dba7bc995e46a7331e6c2ec274eaf64`.
Read-only SSH confirmed producer/bridge/phone/recorder active, startup preflight
in its earlier failed state, retired bridge masked, and correct service aliases.
Installed producer/bridge/recorder/startup-unit hashes matched repository source
after LF normalization; the phone unit matched byte-for-byte on Tractor01.
No drop-ins were reported. No service restart, reinstall or firmware upload was
needed for this audit. The final synchronized commit is recorded in the Git
history/final response rather than embedded self-referentially in its own commit.

See `20261119-pose-integration-next-step-handoff.md` for the continuation prompt.
