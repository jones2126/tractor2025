# Pure Pursuit Controller Log Enhancements

**Status:** Action item — not yet implemented  
**Created:** 2026-07-23  
**Applies to:** `tractor_rpi/pure-pursuit/pure_pursuit_controller_*.py` and mission launchers  
**Goal:** Make every field-test log self-describing and suitable for reliable steering-configuration comparisons.

## Problem Found During the 2026-07-22 Analysis

The pursuit CSV contains actual GPS positions and controller calculations, but it
does not contain enough context to reproduce the complete test without locating
other files:

- `driving=True` means the controller sent `cmd_vel`; it does not prove that the
  handheld switch and Teensy were in AUTO.
- Only the current lookahead target is logged. The complete target path cannot be
  recovered reliably, so the original mission file is required.
- Controller and firmware settings are absent. A result cannot be tied confidently
  to the steering PID, lookahead, speed limit, offsets, or code version.
- `cross_track_err_m` is currently `abs(yt)`, the offset to the lookahead target in
  the tractor frame—not geometric distance to the mission polyline.
- The analyzed run began about 23 m from waypoint 0. That approach must be reported
  separately from active path-following accuracy.

## Desired Outcome

Every pursuit log should answer:

1. Which exact mission, code, firmware, and configuration produced it?
2. Was AUTO actually enabled, and did the Teensy accept each command?
3. Which samples are approach, active tracking, pause, completion, or fault?
4. What steering was requested and applied, and how did the actuator respond?
5. Can runs be compared fairly at the same path position, speed, and GPS quality?

## 1. Add a Metadata Sidecar

Create `pursuit_log_YYYYMMDD_HHMMSS.metadata.json` beside each CSV. This keeps
the CSV compatible with normal tools and eliminates the descriptive second row.

Record:

| Field | Purpose |
|---|---|
| `schema_version` | Supports old and new log formats |
| `run_id` and ISO-8601 UTC start time | Correlates all log sources |
| `tractor_id` | Identifies the hardware |
| `controller_file` and full Git commit | Identifies exact controller source |
| `controller_git_dirty` | Warns when uncommitted code was used |
| `teensy_firmware` | Identifies low-level behavior |
| `mission_filename`, SHA-256, and waypoint count | Proves exact mission contents |
| `max_speed_mps` and minimum fix quality | Records command-line safety limits |
| GPS offsets and wheelbase | Reproduces geometry |
| steering `kp`, `ki`, `kd`, deadband, and PWM limits | Records tuning |
| operator notes | Ground, weather, throttle, tire pressure, and anomalies |

Copy the mission file into the log directory at startup or embed its complete
contents in the metadata. Do not rely on its filename alone.

## 2. Add Per-Sample CSV Fields

| Field | Meaning |
|---|---|
| `radio_mode_raw` | Exact mode value observed from Teensy/bridge |
| `radio_mode_name` | `PAUSE`, `MANUAL`, `AUTO`, or `NO_SIGNAL` |
| `radio_packet_age_s` | Detects stale mode information |
| `cmd_accepted` | Teensy/bridge acknowledgement |
| `control_state` | `WAIT`, `ACQUIRE_PATH`, `TRACKING`, `PAUSED`, `GOAL_REACHED`, or `FAULT` |
| `mission_id` | Joins rows to metadata |
| `nearest_path_segment` | Segment used for geometric reporting |
| `mission_progress_m` | Along-path position |
| `geometric_cte_signed_m` | Nearest-polyline distance; positive left, negative right |
| `lookahead_yt_signed_m` | Clear name for the existing `yt_m` meaning |
| `steer_requested` and `steer_applied` | Requested versus accepted command |
| `steering_pot_raw` and `steering_setpoint_raw` | Actual feedback and target |
| `steering_error_raw` | Setpoint minus feedback |
| `steering_pwm` and `steering_saturated` | Actual output and authority limit |
| `transmission_command` and `transmission_feedback` | Requested/actual JRK state |
| `ground_speed_mps` | Measured speed, not only commanded speed |

Retain timestamps, position, heading, fix quality, GPS age, current target,
waypoint, lookahead, alpha, delta, speed command, `driving`, and wait reason.

## 3. Define Semantics in Code

- `geometric_cte_signed_m`: shortest tractor-reference-point distance to the
  mission polyline, signed using the segment direction.
- `lookahead_yt_signed_m`: pure-pursuit lateral displacement to its selected target.
- `control_state=TRACKING`: GPS gate passed, radio mode confirmed AUTO, command
  accepted, and initial path acquisition completed.
- Document whether position is the raw antenna coordinate or corrected tractor base.
- Use one radio-mode mapping everywhere. Some current project notes/scripts describe
  numeric modes inconsistently, so log and validate both numeric value and name.

## 4. Standard Analysis Metrics

Default headline metrics must use only `control_state=TRACKING`:

- signed bias, MAE, RMSE, median absolute error, P95, and maximum;
- percentages within ±0.10 m, ±0.25 m, and ±0.50 m;
- steering saturation percentage;
- error zero crossings per 10 m as a weaving indicator;
- metrics in 5 m path-distance bins;
- metrics grouped by actual speed and GPS fix quality;
- steering response lag using requested steering versus feedback.

Report acquisition separately: initial path distance, time/distance until tracking,
and engagement overshoot.

## Fair Steering-Comparison Procedure

1. Change only one tuning variable per comparison when practical.
2. Use the identical mission and verify its SHA-256.
3. Start at approximately the same position and heading.
4. Use the same throttle, speed, minimum fix quality, and tire pressure.
5. Record ground and weather conditions.
6. Perform at least three runs per configuration.
7. Compare aggregate median, MAE, P95, bias, saturation, and zero-crossing rate.
8. Do not select a configuration from one unusually good run.
9. Preserve raw logs, metadata, mission copy, summary, map, and calculated samples.

Suggested label:
`kp1.0_ki0_kd0_ld3.0_v0.4_deadbandX_YYYYMMDD`

## Acceptance Criteria

- [ ] CSV has one header row; descriptions live in metadata/documentation.
- [ ] Metadata records mission hash, Git commit, firmware, and tuning values.
- [ ] Exact mission contents are preserved beside the log.
- [ ] Every sample contains raw and named radio mode.
- [ ] AUTO is observed from Teensy/bridge state, not inferred from `driving`.
- [ ] Controller state separates acquisition from active tracking.
- [ ] Requested/applied steering, feedback, and PWM are logged.
- [ ] Geometric CTE is documented and tested with synthetic known geometry.
- [ ] Analyzer supports the new schema and remains compatible with July 2026 logs.
- [ ] A run can be analyzed without manually locating mission/configuration files.
- [ ] Three repeated runs can produce one configuration-comparison table.

## Implementation Order

1. Add schema-versioned metadata; copy and hash the mission.
2. Pass observed radio mode and acknowledgement from bridge to controller.
3. Add `control_state` and separate acquisition from tracking.
4. Add steering setpoint, feedback, PWM, and transmission feedback.
5. Add geometric CTE/progress, or calculate them using the preserved mission.
6. Update `analyze_pure_pursuit.py` for metadata and multi-run comparisons.
7. Capture three baseline runs with current settings before tuning.

## Related Files

- `field_testing/tools/analyze_pure_pursuit.py`
- `tractor_rpi/pure-pursuit/pure_pursuit_controller_20260714.py`
- `tractor_rpi/pure-pursuit/mission_manual_turn_test_20260722.txt`
- `tractor_rpi/pure-pursuit/run_manual_turn_test_20260722.sh`
