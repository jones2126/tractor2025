# Polygon 1 inner-stripes 1.00 m lookahead retest context

Use this document as context for a new Codex chat supporting the field retest and subsequent analysis.

## Objective

Retest the same Polygon 1 inner-core stripe mission after reducing the Pure Pursuit lookahead from 2.00 m to 1.00 m. The primary acceptance target is geometric cross-track error within plus or minus 0.10 m on the straight stripe centerlines.

The mission geometry, waypoint order, starting pose, lane spacing, and speeds have not changed. Only the lookahead column and its associated report, documentation, and SHA-256 guard were changed.

## Mission files

Repository-relative directory:

```text
field_testing/sites/62_Collins_polygon_1/spiral_missions/20260804_inner_stripes
```

Important files:

```text
polygon_1_inner_stripes_38in_20260804.txt
polygon_1_inner_stripes_38in_preview.png
polygon_1_inner_stripes_38in_deck_coverage.png
polygon_1_inner_stripes_38in_report.json
polygon_1_inner_stripes_38in_20260804_validation.json
polygon_1_inner_stripes_38in_20260804_validation.png
run_polygon_1_inner_stripes_20260804.sh
README.md
```

Current mission characteristics:

- 20 east/west strips
- 1,010 waypoints
- 0.9652 m (38-inch) lane spacing
- 472.2 m route length
- 0.85 m/s straight speed
- 0.50 m/s keyhole and transit speed
- 1.90 m modeled keyhole radius
- 1.00 m Pure Pursuit lookahead on every waypoint
- Starting latitude: `40.485562833`
- Starting longitude: `-80.332340333`
- Starting heading: approximately `162 degrees` compass
- Mission SHA-256: `b9c9ff5b18be1cba7f725b63ba17f561c72769409ee59e2e72fd144b0181c028`

The independent validator returned `REVIEW` because one existing pair of consecutive mission points is only approximately 5.8 mm apart. This warning was present in the geometry and was not introduced by changing the lookahead. Maximum waypoint gap, speeds, route length, and minimum modeled radius otherwise remained acceptable.

## August 4 baseline run

Baseline run directory:

```text
field_testing/sites/62_Collins_polygon_1/runs/20260804_inner_stripes_interrupted
```

Primary baseline files:

```text
polygon_1_inner_stripes_20260804_144018.csv
pursuit_log_20260804_144020.csv
rtcm_server_inner_stripes_20260804.txt
teensy_bridge_inner_stripes_20260804.txt
four_strip_cross_track_diagnostic_20260805.png
four_strip_cross_track_summary_20260805.csv
```

The run used a 2.00 m lookahead. It was interrupted at waypoint 849 of 1,010 after transmission behavior became concerning. The controller reached waypoint 849 at approximately 698 seconds. Auto ended around 715 seconds, followed by radio-loss and Manual recovery.

## Baseline cross-track analysis

Strips 13 through 16 were selected because they were the last four fully completed strips before the interruption. They alternate west, east, west, east and correspond to these zero-based mission waypoint ranges:

| Strip | Direction | Mission indexes |
|---|---|---:|
| 13 | West | 654-679 |
| 14 | East | 707-731 |
| 15 | West | 759-781 |
| 16 | East | 809-830 |

The analysis used the 20 Hz pursuit log and RTK position. For each strip, a straight line was constructed from its mission endpoints. Each actual GPS position was projected onto that line to calculate signed geometric cross-track error. Positive error means left of the direction of travel. The first and last meter of each strip were excluded to reduce keyhole entry and exit effects.

This geometric error is the appropriate acceptance measurement. The controller's logged `cross_track_err_m` is the absolute lateral displacement of the active lookahead target in the vehicle frame; it is useful controller telemetry but is not independently calculated point-to-line error.

Baseline results:

| Strip | Direction | Mean absolute error | 95th percentile | Maximum | Samples within plus or minus 0.10 m |
|---|---|---:|---:|---:|---:|
| 13 | West | 0.293 m | 0.433 m | 0.488 m | 0.0% |
| 14 | East | 0.282 m | 0.298 m | 0.317 m | 0.0% |
| 15 | West | 0.330 m | 0.473 m | 0.511 m | 0.0% |
| 16 | East | 0.282 m | 0.332 m | 0.332 m | 0.0% |

All four paths remained approximately 0.28-0.33 m left of the direction of travel. Because the sign stayed left-relative-to-travel while geographic direction alternated, this was not simply a fixed north/south map displacement.

The controller recognized the error:

- Mean controller lateral target offset was approximately 0.30-0.32 m.
- Mean Pure Pursuit command was approximately `-0.30` normalized steering.
- Negative steering means right.
- This represented approximately 11 degrees right steering in the controller model.
- On strips 13-15, the Teensy steering setpoint and measured steering potentiometer position closely followed the Pure Pursuit request.
- Strip 16 contained a short interval where the measured steering position did not follow the requested position.

Strips 13 and 15 reduced large entry errors but settled near 0.28 m instead of converging to the line. Strips 14 and 16 traveled almost parallel to the target at that offset. The determination was that the system was correcting in the proper direction but was not aggressive enough to satisfy the plus or minus 0.10 m target. Possible contributors include the 2.00 m lookahead, 0.85 m/s speed, slope or tire slip, steering-angle calibration, and low-level steering response.

## Lookahead change

Every mission row was changed from:

```text
lookahead_m = 2.00
```

to:

```text
lookahead_m = 1.00
```

This includes the initial transit, all straight strips, and all keyhole connectors. Geometry, yaw, and speed columns were not changed. The report and launcher verify the new 1.00 m value and updated mission hash.

Reducing lookahead should cause Pure Pursuit to react more strongly to a given lateral displacement. It can also produce more steering activity or oscillation, particularly in keyholes. The retest must therefore remain supervised with immediate access to Pause.

## Current tractor software and telemetry

Current relevant files:

```text
tractor_teensy/src/teensy_main_20260804.cpp
tractor_rpi/teensy_serial_bridge_20260728.py
tractor_rpi/archive/field_test_loggers/field_test_logger_20260804.py
```

The updated firmware publishes 20 Hz steering PID telemetry and 5 Hz expanded JRK transmission diagnostics. Important transmission fields include:

```text
jrk_target
jrk_actual_target
jrk_current
jrk_scaled_feedback
jrk_duty_cycle_target
jrk_duty_cycle
jrk_errors_halting
jrk_errors_occurred
jrk_valid
jrk_read_latency_ms
jrk_timeouts
jrk_discarded_bytes
radio_transmission_raw
trans_cmd_vel_mps
trans_cmd_vel_age_ms
```

Radio mode convention:

```text
DOWN   = Auto   = mode 0
MIDDLE = Manual = mode 1
UP     = Pause  = mode 2
No radio        = mode 9
```

Do not run the mission unless the handheld radio is available and reliably changes the telemetry state to Pause and Manual as expected.

## Preparing tractor01

On tractor01:

```bash
cd /home/al/tractor2025
git pull
git status --short
git rev-parse --short HEAD
```

The status should be empty. Verify that the mission has the expected hash:

```bash
sha256sum field_testing/sites/62_Collins_polygon_1/spiral_missions/20260804_inner_stripes/polygon_1_inner_stripes_38in_20260804.txt
```

Expected:

```text
b9c9ff5b18be1cba7f725b63ba17f561c72769409ee59e2e72fd144b0181c028
```

With the tractor in Pause and blades disengaged, run preflight:

```bash
sudo python3 /home/al/tractor2025/tractor_rpi/testing/mission_preflight_20260804.py
```

Do not continue unless preflight passes. Confirm at minimum:

- `rtcm-server.service` active
- `teensy-bridge.service` active
- RTK Fixed
- heading valid and carrier solution fixed
- correction age acceptable
- steering telemetry approximately 20 Hz
- radio state is Pause
- transmission requested target and JRK actual target are neutral
- JRK diagnostic reads valid with no accumulating timeouts

## Starting the retest

Position tractor01 at:

```text
Latitude:  40.485562833
Longitude: -80.332340333
Heading:   approximately 162 degrees compass
```

Keep the radio switch UP in Pause and the mower deck disengaged. Run:

```bash
cd /home/al/tractor2025/field_testing/sites/62_Collins_polygon_1/spiral_missions/20260804_inner_stripes
bash ./run_polygon_1_inner_stripes_20260804.sh
```

The launcher verifies the hash, report, preflight, starting distance, and heading. Type the requested confirmation only after reviewing the controller output. Then select Auto using the DOWN switch position.

For the first portion of the retest:

- Keep the deck disengaged.
- Watch cross-track error, heading, steering command, and physical steering response.
- Expect more active steering than with the 2.00 m lookahead.
- Use Pause immediately if steering oscillates, the tractor approaches the boundary unexpectedly, RTK behavior becomes unstable, or either steering/transmission actuator moves without a corresponding command.
- Do not relax the RTK Fixed requirement to finish the mission.

## Stopping and preserving data

At completion or after an interruption, put the tractor in Pause. Allow the launcher to close both logs cleanly. Record the two paths printed by the launcher:

```text
/home/al/field_logs/polygon_1_inner_stripes_<timestamp>.csv
/home/al/repos/field-testing-data/pursuit_log_<timestamp>.csv
```

Also save the service journals covering the run:

```bash
sudo journalctl -u teensy-bridge.service --since "30 minutes ago" --no-pager -l > /home/al/teensy_bridge_1m_retest.txt
sudo journalctl -u rtcm-server.service --since "30 minutes ago" --no-pager -l > /home/al/rtcm_server_1m_retest.txt
```

Adjust the `--since` interval if the run lasted longer.

## Copying the results to Windows

In Windows PowerShell, create a run folder using the actual run timestamp:

```powershell
$site = 'C:\Repos\tractor2025\field_testing\sites\62_Collins_polygon_1'
$runId = '<YYYYMMDD_HHMMSS>'
$run = Join-Path $site "runs\${runId}_inner_stripes_1m_lookahead"
New-Item -ItemType Directory -Force $run
```

Copy the exact field and pursuit filenames printed when the launcher stops:

```powershell
scp "al@192.168.193.76:/home/al/field_logs/polygon_1_inner_stripes_<timestamp>.csv" $run
scp "al@192.168.193.76:/home/al/repos/field-testing-data/pursuit_log_<timestamp>.csv" $run
scp "al@192.168.193.76:/home/al/teensy_bridge_1m_retest.txt" $run
scp "al@192.168.193.76:/home/al/rtcm_server_1m_retest.txt" $run
```

Review the copied files before rebooting or deleting anything:

```powershell
Get-ChildItem $run | Sort-Object Name | Select-Object Name, Length, LastWriteTime
```

## Analysis requested after the retest

Ask Codex to compare the new 1.00 m run directly with the August 4 2.00 m baseline. At minimum, repeat the geometric analysis on strips 13-16 with identical selection rules:

- Use mission indexes 654-679, 707-731, 759-781, and 809-830.
- Include only `driving=True` samples.
- Project actual RTK position onto each mission endpoint line.
- Exclude the first and last 1.0 m of each strip.
- Preserve signed error; positive means left of travel.
- Calculate sample count, mean signed error, mean absolute error, RMS error, median, 95th percentile absolute error, maximum absolute error, and percentage within plus or minus 0.10 m.
- Split each strip into entry, middle, and exit thirds to measure convergence.
- Plot geometric error versus along-strip distance with a shaded plus or minus 0.10 m band.

Also evaluate control behavior:

- Controller `yt_m` and `cross_track_err_m`
- Pure Pursuit `steer_normalized`
- Requested steering setpoint versus measured steering potentiometer position
- Steering error, PWM, deadband, minimum-PWM clamping, and saturation
- Heading error relative to the straight-line direction
- RTK fix quality and heading validity
- Actual speed versus the 0.85 m/s command

Evaluate the expanded transmission data at the same timestamps:

- Confirm `jrk_target` equals `jrk_actual_target`.
- Identify any feedback jump while actual target remains stable.
- Compare raw/scaled feedback with duty-cycle target and applied duty.
- Report changes in JRK timeouts or discarded bytes.
- Correlate any anomaly with radio mode, raw radio transmission value, RTK quality, speed, and waypoint.

## Retest decision criteria

The 1.00 m lookahead is an improvement if it materially increases time within plus or minus 0.10 m and lowers mean and 95th-percentile geometric error without producing unsafe oscillation.

Suggested interpretation:

- **Pass candidate:** all four steady strip portions predominantly remain within plus or minus 0.10 m, with no sustained oscillation or actuator anomaly.
- **Improved but insufficient:** error decreases significantly from the 0.28-0.33 m baseline but remains predominantly above 0.10 m.
- **Too aggressive:** repeated sign-changing cross-track oscillation, increasing steering reversals, heading oscillation, saturation, or visibly unstable keyholes.
- **Low-level limitation:** Pure Pursuit requests an appropriate correction but measured steering does not follow it.
- **Vehicle/model limitation:** measured steering follows the request but the tractor remains parallel and offset, suggesting calibration, slope, tire slip, or controller-model limitations.

Do not tune solely from the controller's `cross_track_err_m`. Use the independently calculated geometric point-to-line error as the acceptance metric.
