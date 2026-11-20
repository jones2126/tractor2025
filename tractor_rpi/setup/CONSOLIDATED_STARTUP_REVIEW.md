# Consolidated startup implementation — review before deployment

November 18 design/review record; status updated November 19, 2026.
**INSTALLED November 18; stationary preflight and manual sensor capture exercised
November 19. Unattended boot reliability and control-pose accuracy remain unvalidated.**

The preparation text below is historical. Al installed the consolidation with
backup `/var/backups/tractor-consolidation-20261118T181841Z`. Canonical runtime,
current limitations and startup details are in AGENTS.md and
`field_testing/TRACTOR01_CURRENT_FIELD_CONTEXT.md`. The four runtime services
were active in the November 19 audit; installed unit hashes matched repository
sources (LF-normalized). No additional install/restart is required for the
November 19 documentation/replay synchronization.
Al approved local implementation after reviewing the design. No hardware access,
service installation, firmware changes, engine start or field movement was performed.
The earlier November 17 one-boot candidate instructions are not the deployment
procedure for this new boot architecture.

## Decisions incorporated from Al's review

- Service `calc-publish-position.service` matches the position program's purpose.
  It starts module `tractor_rpi.calc_publish_position`; `rtcm-server.service`
  becomes an alias to the same unit, not a second process. Heading-F9P startup
  configuration and independent readback remain mandatory.
- `tractor-teensy-bridge.service` is the clearer operator-facing name, with
  `tractor-rpi-arbitrator.service` as an alias. The command-selection program,
  route hash, existing communications-degraded envelope and Teensy owner remain
  unchanged. This is a bridge that also performs safety selection. The retired
  `teensy-bridge.service` remains masked; never use that name for the new service.
- `tractor-telemetry-recorder.service` starts independently at boot. Raw telemetry
  is in `/home/al/field_logs/YYYY-MM-DD/telemetry_<UTC>/`, outside Git. Daily
  folders use America/New_York; raw timestamps retain UTC and monotonic times.
- Ten-second healthy-GNSS stops create **candidate** replay markers. Startup,
  ordinary pauses and intended markers cannot be distinguished by duration alone.
- Startup waits for source/network readiness and sensor preparation before full
  preflight. ntfy notices follow qualified service/network readiness and final
  results, with bounded retries. ntfy remains best effort; local reports always
  run and persist. App/web delivery latency is outside the preflight's authority.

## Production structure and source map

| Program/module | Purpose |
|---|---|
| `tractor_rpi/calc_publish_position.py` | GPS/IMU/encoder owners, RTCM forwarding, four unchanged raw GNSS feeds, non-authoritative pose, calibration state |
| `tractor_rpi/position_gnss_protocol.py` | Direct GNSS wire decoders; no device access on import |
| `tractor_rpi/imu_sensor.py` | Direct IMU acquisition/scaling and pure yaw/bias helpers |
| `tractor_rpi/position_model.py` | Raw GNSS state and observational pose; explicit calibration permission |
| `tractor_rpi/telemetry_ipc.py` | Bounded Unix transport, producer sequences and ACK/retry, read-only observation queries |
| `tractor_rpi/telemetry_recorder.py` | One continuous recorder; raw JSONL, compatible field CSV, bounded recent windows, daily/15-minute segmentation, closed-file hashes |
| `tractor_rpi/field_telemetry_schema.py` | Pure mapping for legacy CSV columns; raw status additionally preserves full arbitrator objects |
| `tractor_rpi/tractor_preflight.py` | One explicit check implementation; no dated preflight imports or monkey-patching |
| `tractor_rpi/tools/export_telemetry_run.py` | Verify closed segment manifests and export a time-bounded compatible field CSV with provenance |

There are three running programs, not one process for every module. Stable helper
modules do not start threads, open hardware or perform monkey-patching on import.
Frozen experiment/runtime sources are retained for provenance, not inherited into
the new preflight or sensor producer. The established F9P configuration hook and
unchanged Teensy control implementation still have their existing filenames.

## Recording and calibration behavior

Recorder owns one GPS feed (6009), status broadcast (6003) and router TCP 6005.
IMU/encoder/pose/events use bounded local Unix IPC. Publisher sequence ACKs allow
deduplication after uncertain delivery; gaps are explicit. Preflight reads sample
windows without consuming or stealing UDP packets. Slow responses use bounded
nonblocking writes. No new external UDP port is introduced.

Each segment holds `gnss.jsonl`, `status.jsonl`, `imu.jsonl`, `encoders.jsonl`,
`pose.jsonl`, `router.jsonl`, `events.jsonl`, `field.csv`, metadata and, after close,
`SHA256SUMS`. IMU/encoders retain native acquired observations. One GPS feed is
recorded, not four copies. Hashing closed segments runs independently of capture.
Low disk space/write failure stops successful recording and is reported; originals
are not deleted. Prior incomplete sessions are identified without modifying them.

Initial boot assumes the established **engine-off startup procedure**; fresh Pause,
neutral, zero output, Fixed stationary GNSS and fresh stationary wheels must also
be observed for three seconds before settling/gyro calibration. Raw IMU data is
recorded while waiting. Gyro calibration uses the established one-second settling
and ten-second stationary sample checks. Valid GNSS then aligns heading separately.
No automatic engine-off detector is claimed. An engine-on reboot is outside that
calibration prerequisite even if the sensors look quiet.

A runtime token survives producer service restarts but clears on actual reboot.
After a later restart, raw recording continues and calibration stays unarmed until
an explicit engine-off confirmation through canonical preflight. Existing bias
is never silently recalibrated by a routine preflight rerun. Encoder scales remain
unverified; no distance calibration/firmware work or slip classifier was added.

Canonical preflight adds recorder health, calibrated/aligned fresh IMU and both
live encoder message streams. Each encoder needs at least five distinct device
clocks over the recent five-second window, latest sample age <=0.5 s and a valid
current producer state. Zero delta while parked passes. Existing GPS, network,
Pause, firmware, transmission and applicable legacy conditional checks remain.
These new checks do not modify Manual authority or arbitrator watchdogs.

Optional `--restart-gps-on-failure --confirm-engine-off` permits one explicit
GPS-related recovery only after the existing stopped-Pause checks pass; it reruns
full preflight afterward. The default boot path does not restart GPS repeatedly.
Preflight stdout and dated reports remain available even if ntfy is unavailable.

## Exact caller migration boundary

New startup unit invokes `python3 -m tractor_rpi.tractor_preflight --startup`.
New manual command is the same module without `--startup`, with v5 explicitly
specified. Installer replaces the installed startup unit only after separately
approved stationary installation. The old scripts and installed services have
not been changed by local preparation.

**Frozen November 4 mission launcher/dashboard/verifier remain unchanged.** That
launcher starts a private logger, uses old preflight UDP bindings and expects a
mission-specific log. Do not run it unchanged after installing the boot recorder.
Historical packages also remain unchanged. No Auto mission successor is authorized
or silently substituted in this manual data-collection implementation.

For a future reviewed Auto successor, use the canonical preflight, use the existing
route/controller unchanged, and export continuous recorder segments/time bounds
for the collector instead of starting/killing a private logger. The export utility
implements the file contract; the frozen launcher's control/collection integration
requires a separately reviewed successor package. Existing collectors/replays can
read an exported field CSV; new raw IMU/encoder replay needs the new capture schema
and explicit source paths, not the November 17 defaults.

## Stationary installation review and rollback

The installer defaults to review-only:

```bash
cd /home/al/tractor2025
python3 -m tractor_rpi.setup.install_consolidated_startup
```

Do not apply until the reviewed diff is committed/synchronized and separate
stationary installation is authorized. Inspect actual units/owners; require clean
main/exact SHA, engine off, stopped Pause and blades disengaged. Confirm user al
can access GNSS/encoder serial and I2C, dependencies pyserial/smbus2 and system
timezone data exist, and `/home/al/field_logs` is writable with sufficient storage.
Stop standalone IMU/private field loggers first. Review all installed drop-ins;
the installer refuses unreviewed ones and the temporary November 17 override.

Explicit installer application backs up unit files/symlinks, effective definitions,
enablement and commit under `/var/backups/tractor-consolidation-<UTC>/`, stops the
old startup/position/bridge units, installs new names and compatibility aliases,
and enables boot units. **It does not start production services.** This is a
prepared installer, not permission to run it now.

Approved stationary activation starts recorder, then renamed bridge/producer,
with the existing phone service retained. Verify exactly one owner of each serial
device, raw file growth, no loss/error counters, Heading readback, calibration and
fresh full canonical preflight PASS. Inspect current-boot journals. Reboot testing
is separate; ensure actual engine-off startup conditions. Record exact directories.

Rollback while stopped/engine off: stop new startup/position/bridge/recorder, close
and copy/verify evidence, disable the new canonical units, restore the backed-up
files or symlink targets from the manifest (remove only newly installed paths
listed there), restore original enablement, daemon-reload and start the original
production stack. Keep the retired bridge masked. Verify original ExecStart,
single owners, original canonical preflight PASS. No automatic deletion/rollback
script is provided for unreviewed installed state.

## Validation and review artifacts

See the consolidation design's CHECK_PARITY.md and REFERENCE_INVENTORY.md for all
legacy predicates, CLI semantics and 83 direct-reference files plus five indirect
integration dependencies. Review the full local diff before committing, as requested
in the November 13 consolidation prompt. Existing tests and frozen files are unchanged.
New offline tests cover predicate comparisons, transport/queries, file hashes and
exports, duplicate records, gaps, raw pre-calibration samples, calibration/alignment,
zero-motion encoders, low storage, ownership refusal and notification failure.
Hardware acquisition, installed systemd alias behavior, Pi scheduling, real ntfy
arrival timing and field pose accuracy still require stationary/field validation.
