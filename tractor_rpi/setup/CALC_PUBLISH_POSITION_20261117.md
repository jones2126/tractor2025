# November 17 pose-producer field candidate

> November 20 repository update: this candidate is archived under `tractor_rpi/archive/pose_producer_20261117/`. Commands and deployment status below describe the historical candidate, not current startup. Current entry point is `tractor_rpi.calc_publish_position`; see `../README.md`.

**DRAFT — UNTESTED IN THE FIELD.** Prepared for a separately approved stationary
data-flow check and manual run on November 18. No installation, restart, firmware
flash, movement or blade operation is authorized by this file. Tractor01 is not
updated during the November 17 preparation.

## What was built

Requested entry point: `tractor_rpi/calc-publish-position-20261117.py`.
Implementation: `calc_publish_position_20261117.py`; pure data/pose model:
`position_model_20261117.py`; reviewed wire parsers extracted from the existing
RTCM server: `position_gnss_protocol_20261117.py`.

The old RTCM server and production unit are unchanged. The new candidate has no
Wi-Fi polling and never opens the control Teensy, sends RPICMD, publishes on
command/proposal ports or grants degraded motion. It retains the correction TCP
source `192.168.193.88:6001`, both F9P serial owners and raw GNSS fields on the
four existing localhost ports 6002/6009/6010/6013 at nominal 20 Hz. Only the three
Pi-wlan0 diagnostic fields are omitted, as agreed. One encoded packet is copied
to all four ports. No fused/shadow position replaces legacy `lat`, `lon`,
`heading_deg`, `fix_quality` or validity flags. A 2048-byte guard protects the
existing field logger's receive buffer. Non-finite derived accuracy becomes
null rather than nonstandard JSON NaN.

## Execution split

Main loop, nominal 20 Hz:

1. `ingest_gps()`
2. `ingest_imu()`
3. `ingest_encoders()`
4. `update_pose()` — process the full batch in timestamp order
5. `assess_pose_quality()`
6. `publish_navigation_if_due()`
7. `queue_diagnostics_if_due()`

Seven background workers: base GNSS reader, heading GNSS reader, IMU reader,
left encoder reader, right encoder reader, RTCM forwarder, evidence/diagnostics
writer. Readers queue all acquired samples; IMU integration uses individual
sample intervals rather than a synthetic 20 Hz rate. Queues are bounded and
overflow is an explicit capture failure. RTCM and base diagnostic poll writes
share a serial write lock; heading polling has its own sole writer. Correction
reconnects cannot hold up IMU acquisition or the main loop. Publication/diagnostic
failures do not manufacture a healthy pose. Threading/load remain field-unvalidated.

## Sources and experimental estimator

- GNSS: checksum-checked mixed NMEA/UBX streams; current position, speed,
  heading, satellites, baseline, accuracy, counters and source timestamps.
- IMU: ISM330DHCX on I2C bus 1 at 0x6b, 104 Hz nominal, +/-2 g, +/-250 dps,
  block-data update/auto-increment and readback; same proven acquisition/scaling
  helper as the November 16 standalone logger. One-second settling then ten-second
  engine-off stationary gyro calibration; quality checks remain in place.
- Left encoder: USB serial `5879620`; right encoder:
  `F5CB3C7E51514A4E43202020FF142006`; both 115200 baud. Devices are selected by
  their USB serial identity, not unstable ttyACM numbers or unapplied udev aliases.
  `fuser`/exclusive opening and device-collision checks protect existing owners.
  Do not run another IMU test/logger at the same time; shared I2C bus access alone
  does not enforce ownership of one sensor's configuration.
- Encoder `ERR`, `WARN`, malformed lines, missing samples and device-clock resets
  invalidate continuity. A resumed stream cannot restore lost travel. Both raw
  delta ticks and firmware-filtered speed are preserved. A later healthy GNSS
  position can re-establish a reference; discontinuity records remain in evidence.
- Raw wheel scale is **unverified**. Default provisional values are clearly
  labeled; no uncalibrated wheel distance is used as supported position fallback.
  Optional reviewed per-side calibration requires matching device IDs, an evidence
  reference and a calibration-file hash retained in metadata. Do not fabricate
  this calibration for tomorrow's first flow test.
- Shadow heading integrates bias-corrected gyro and records an experimental
  healthy-GNSS anchor approximately every five seconds. A separate once-anchored
  relative-yaw record remains available to examine drift. Healthy anchors require
  RTK Fixed position plus fixed/valid/fresh heading, accuracy <=1 degree and
  baseline 0.80–1.30 m. A >50 ms IMU gap resets its segment, not an incident budget.
- The experimental model can propagate signed average wheel distance when
  reviewed scales and continuous/fresh streams exist. This is a simple local
  projection/current-heading model, not a validated probabilistic filter or
  surveyed axle/antenna model. Uncertainty is explicitly unquantified, not zero.
  GNSS decode-time alignment is still experimental. `valid_for_control` is always
  false; no 3 m/30 s policy, speed cap or authorization is implemented here.

## Evidence captured

Unique directories under `/home/al/field_logs/position/position_<UTC>/` contain:

- `samples.csv`: every acquired IMU sample, raw/scaled/bias-corrected channels,
  settling/calibration/observation phase, relative yaw and current GNSS snapshot.
  Two added columns identify `direct_serial` input and actual position-source age.
  UDP packet/socket-age columns are blank because this program reads GNSS directly.
- `encoders.csv`: both sides, arrival times, device clocks, raw positions/counts,
  reported filtered speed/delta, computed provisional/calibrated delta and continuity.
- `gnss.jsonl`: full legacy GNSS snapshot once per publication cycle.
- `shadow_pose.jsonl`: non-authoritative pose, modes, sources/quality, anchor ages,
  scale state and explicit unknown uncertainty.
- `events.jsonl`: calibration, gyro gaps, encoder warnings/errors, health, correction
  throughput and queue depths. Existing correction-throughput console text is kept
  for canonical preflight compatibility.
- `metadata.json`: exact clean Git commit, program source hashes, devices, scales,
  calibration, row counts, four-port send counts and completion status.
- `SHA256SUMS`: generated after the evidence writer closes. Logging failure,
  overflow or incomplete shutdown must not be treated as a successful capture.

This package has a distinct capture schema and directory name; do not run the
November 17 frozen replay/analyzer on it using their old dataset defaults.
The full raw records support a new explicitly targeted replay after the field run.

## Tonight's hardware-free checks

November 17 software validation passed: 214 unit/regression tests, including 26
candidate tests and a simulated multi-sensor capture through the four UDP sends,
closed evidence files, checksums and offline capture verifier. Entry-point help,
Python compilation and the candidate import audit passed. The frozen production
mission package still verifies unchanged. Existing replay JavaScript checks and
28 archived analysis/replay file checksums also passed.

These checks use simulated devices and do not prove Pi scheduling, physical
sensor acquisition, actual UDP consumer reception, scale accuracy or field pose
accuracy. Those remain tomorrow's stationary and separately approved manual
validation work.

```bash
python3 -m unittest tractor_rpi.testing.test_calc_publish_position_20261117
python3 tractor_rpi/calc-publish-position-20261117.py --help
```

## Tomorrow: inspection and separately approved installation

Start in `/home/al/tractor2025`, confirm clean main and pull the exact reviewed
GitHub revision. Read `field_testing/TRACTOR01_CURRENT_FIELD_CONTEXT.md`.
Keep phone in Pause, engine off, blades disengaged, transmission mechanically
released and e-stop accessible. First inspect installed service contents,
current-boot heading readback proof, USB identities and existing device owners.
Require the current canonical preflight PASS before changing software. Do not
restart a healthy service merely to discover its existing configuration.

Installation/replacement is a separate approval tomorrow. The prepared drop-in
`tractor_rpi/setup/calc-publish-position-20261117.conf` is for the **existing**
`rtcm-server.service` name. It overrides ExecStart only, preserving the installed
Heading-F9P configuration/readback ExecStartPre and current consumers/preflight
service checks. It sets Restart=no: a restarted process must never silently
calibrate with the engine running. Do not install this field-test override permanently; it asserts operator-confirmed stationary startup on every start.
The proposed override is installed under `/run/systemd/system` for this boot
only and disappears automatically on reboot, leaving the original definition.
Close and verify capture files before ending the session.

After explicit installation approval, back up `systemctl cat rtcm-server.service`
and the installed unit/drop-ins. Refuse an existing `position-candidate.conf`
unless it is reviewed first. The proposed commands, **not run tonight**, are:

```bash
cd /home/al/tractor2025
sudo install -D -m 0644 tractor_rpi/setup/calc-publish-position-20261117.conf /run/systemd/system/rtcm-server.service.d/position-candidate.conf
sudo systemctl daemon-reload
sudo systemctl restart rtcm-server.service
```

That restart is the explicit software replacement, not a diagnostic restart.
Never run the old server and replacement concurrently. No original program
file or base service unit is overwritten. No udev change or encoder flash is
included. Do not alter the phone/RPi arbitrator services or command watchdogs.

## Stationary validation before any manual movement

Inspect current-boot journals and actual serial owners. Expect all three sensor
families, passed IMU calibration, both wheel sample counters advancing, and
`READY FOR DATA REVIEW`. Confirm all four existing navigation consumers still
parse the raw GNSS packets. Run the unchanged canonical preflight while the
tractor remains stopped and require its exact final `MISSION PREFLIGHT PASS`.
GPS forwarding/heading startup checks must still pass; diagnose failures before
movement. Stream/file presence is not physical encoder scale calibration.

Do not run the old standalone IMU logger, another I2C test, or another GNSS
serial reader. After preflight, the existing field logger can optionally run
alongside the candidate: the candidate **publishes**, not binds, UDP 6009. This
would additionally capture actual steering/drive/status and router RSSI for
tomorrow's hard-turn review. Stop it before repeating a preflight that competes
for 6009. Store its exact separate filename and start time with the run notes.

## Manual route and closing the capture

Manual movement is a separate operator approval. Use established local phone
Manual controls, blades off, for the two squares and hard-left/hard-right
maneuvers, only after the stationary checks pass. Report engine start/off and
maneuver order/times; raw counts remain useful even with uncalibrated wheel scales.
This logger/estimator grants no motion authority. Stopping it does not send Pause.

At completion: park, phone Pause, engine off, stop any optional field logger,
then stop the candidate service normally so it closes its evidence:

```bash
sudo systemctl stop rtcm-server.service
```

Record the printed directory and require Saved STOPPED. Then verify that exact
directory (replace the placeholder; this verifier does not access hardware):

```bash
cd /home/al/tractor2025
python3 tractor_rpi/testing/verify_position_capture_20261117.py /home/al/field_logs/position/EXACT_PRINTED_DIRECTORY
```

Copy the entire directory and optional controller log off the Pi, independently
verify hashes and preserve originals. File verification does not prove physical
scale or that every recipient received each UDP datagram. Treat firmware warnings,
gaps and scale mismatch as evidence requiring review, not values to smooth away.

## Restore the original service — separate service-change approval

Keep stopped with engine off. Preserve/move the one candidate drop-in into the
recorded backup directory rather than deleting evidence, reload systemd and
start the original rtcm-server definition. Confirm ExecStart resolves to
`rtcm_server_20260828.py`, fresh heading readback proof, production owners,
actual Pause and canonical preflight PASS. Do not unmask the retired Teensy
bridge. A reboot also clears the /run override, but close/verify/copy the evidence first.
The candidate is a first data-flow experiment, not a boot deployment.
