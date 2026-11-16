# ISM330DHCX observation logger

**DRAFT - UNTESTED IN THE FIELD.** This foreground experiment reads only the IMU
and existing GPS UDP 6009. It sends no network data, owns no serial devices,
changes no control programs, and installs no service. Source/build approval,
stationary sensor validation, manual movement, and blade operation are separate.

## Before first use

Use a clean committed checkout at `/home/al/tractor2025` matching GitHub main.
The handoff reports `smbus2` already installed, I2C bus 1, address 0x6b, and axes
X forward / Y left / Z up. Verify mounting and signs from the recorded turns.
The logger verifies identity and reads back its configuration: 104 Hz,
accelerometer +/-2 g, gyro +/-250 dps, block-data update and auto-increment.
Register/scaling reference: [ST's official driver](https://github.com/STMicroelectronics/ism330dhcx-pid).

UDP 6009 is **localhost unicast**, despite older descriptions calling it a
broadcast. The IMU logger binds it exclusively without socket reuse. Do not run
`field_test_logger_20260828.py` concurrently: a second listener would compete
for the feed. Do not start a mission/dashboard launcher for this manual test.
Ports 6002, 6010, 6011, 6013, 6014 and 6015 are untouched. Phone control and its
existing safety checks continue independently. This first package does not
collect Teensy/status telemetry; the existing phone/server logs remain separate.

If the bind fails, IMU recording continues with missing GPS explicitly marked;
resolve this while stopped before a useful comparative drive. Stop/restart the
logger to retry the socket. Fresh packets do not prove fresh heading: the logger
also checks `relposned_timestamp`, validity flags, fixed carrier, RTK Fixed,
heading accuracy <=1 degree, and absence of fatal/ref-position/ref-observation errors.
These checks label observations only and grant no motion authority.

## Stationary validation (separate operator approval required)

Keep the tractor stopped, engine off, blades disengaged and phone in Pause.
Confirm no other IMU test program is running. In PuTTY:

```bash
cd /home/al/tractor2025
git status --short --branch
git rev-parse HEAD
python3 tractor_rpi/testing/imu/ism330dhcx_field_logger_20261116.py --stationary --duration 20
```

`--stationary` confirms the startup condition. A one-second settling period
precedes at least ten seconds of bias calibration. Raw calibration rows are
preserved. Mean/stddev of all gyro axes and acceleration noise/magnitude are
recorded; insufficient sampling, excessive noise/bias, or abnormal gravity fail
calibration. These are experiment-quality checks, not proof that the tractor is
stationary. They cannot detect a steady turn: the operator must ensure stillness.

Expect `READY`, roughly 104 samples/second, near-zero corrected stationary gyro,
and `GPS=valid`. Status is printed at about 5 Hz. After 20 observation seconds,
expect `Saved COMPLETED` and an evidence directory. Inspect metadata and CSV
before approving the driven run. Compare measured sensor ranges to the handoff;
never hard-code its preliminary biases.

## Manual-drive recording (after stationary validation and movement approval)

Run the canonical mission preflight while stationary in Pause and require its
final `MISSION PREFLIGHT PASS` before movement:

```bash
cd /home/al/tractor2025
sudo python3 tractor_rpi/testing/mission_preflight_phone_control_20261109.py --expected-firmware teensy_main_20261110_rpi_command_lease_v5
python3 tractor_rpi/testing/imu/ism330dhcx_field_logger_20261116.py --stationary
```

Calibrate with engine off and remain stationary until `READY`. The logger only
records; it does not start or control the tractor. With separate manual-motion
approval, use the established phone Manual controls and blades-off procedure.
Record a square clockwise, then counterclockwise, followed by circles in both
directions where clear. Include straight segments and stopped intervals.
Expected gyro Z: left positive, right negative. Pause for unexpected behavior.
Stop in phone Pause, engine off, then press Ctrl-C in the logger terminal.
Stopping the logger does **not** stop the tractor or send Pause.

## Evidence and interpretation

Each unique UTC-named directory under `/home/al/field_logs/imu/` contains:

- `samples.csv`: raw signed counts, g, deg/s, corrected gyro, host UTC/monotonic
  timestamps, actual dt, calibration/observation phase and latest GPS/age fields.
- `gnss.jsonl`: complete received GPS packets with local receive times.
- `metadata.json`: exact commit, source hash, configuration, measured bias/noise,
  timestamps and completion/failure state.
- `SHA256SUMS`: hashes of all three files, generated after files close.

Relative yaw is unwrapped CCW-positive trapezoidal integration of bias-corrected
gyro Z. The compass comparison is anchored once per segment to usable F9P
heading, then runs freely (compass = anchor minus relative-yaw change).
`imu_minus_gnss_deg` is wrapped to [-180,180). No continuous GNSS correction is
applied, preserving drift for analysis. Missing/invalid GPS never resets yaw.
Intervals over 50 ms start a new numbered yaw segment and discard the gap;
compare yaw only within a segment. This is not heading fusion or navigation.

This version polls data-ready and reads a coherent 12-byte burst; it does not
use hardware timestamps or FIFO. Linux scheduling can miss samples, and recorded
dt is host read time. Saturation at +/-250 dps or +/-2 g and engine vibration
must be reviewed from raw data; timing gaps and aliasing limit yaw accuracy.
An engine-off calibration may not remove engine-on vibration/temperature bias.
CSV/JSONL flush at least every second; abrupt power loss can lose buffered data.

Preserve the printed directory in full. Verify on Tractor01 while stopped:

```bash
cd /home/al/field_logs/imu/EXACT_PRINTED_RUN_DIRECTORY
sha256sum -c SHA256SUMS
```

Copy the whole directory off the Pi and verify its hashes again before analysis
or cleanup. Never delete original evidence simply because a copy exists.

## Hardware-free checks

```bash
python3 -m unittest tractor_rpi.testing.test_ism330dhcx_field_logger_20261116
```

Importing the logger opens no hardware, sockets, or files. Tests exercise signed
decoding, configuration/identity, calibration, heading freshness/sign/wrap,
integration gaps, socket conflict behavior, and a complete fake-sensor capture
including CSV schema, metadata, cleanup and checksums.
