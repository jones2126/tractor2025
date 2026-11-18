# Local implementation review — November 18

**Implemented and reviewed for repository synchronization; not installed or field validated.**
Prepared after Al reviewed PLAN.md and authorized implementation with comments.
The November 13 consolidation reference asks for a full diff and migration report
before committing. The diff/report were presented before committing; Al then agreed to proceed.
Repository synchronization is the next step. Tractor01 service installation remains separate.

## Result

Three programs: a sensor/position producer, continuous recorder and canonical
preflight. Stable helper modules are imported directly, without dated preflight
inheritance or monkey-patching. A prepared installer defaults to a no-change review.
See `tractor_rpi/setup/CONSOLIDATED_STARTUP_REVIEW.md` for exact behavior,
installation prerequisites, migration limits and rollback procedure.

| Al's comment | Implementation |
|---|---|
| Match position service name to purpose/program | calc-publish-position.service invokes tractor_rpi.calc_publish_position; old rtcm-server name aliases the same unit |
| Prefer Teensy bridge wording | tractor-teensy-bridge.service; existing command-selection code/parameters retained; old arbitrator name aliases the same unit; retired teensy-bridge stays masked |
| Daily folders outside repo | /home/al/field_logs/YYYY-MM-DD/telemetry_<UTC>/; Eastern folder day, UTC observations; 15-minute/day-boundary segments |
| Ten-second stops as markers | Inferred healthy-GNSS stationary episodes recorded as candidate markers, not assumed operator intent |
| ntfy and pre-preflight readiness | Wait for services/network/observations/calibration, bounded ntfy retries, full local result retained regardless of notification delivery |

## What changes on reviewed deployment

- One boot recorder owns UDP 6009 and captures one raw GPS feed plus status,
  router, native IMU/encoder observations and non-authoritative pose.
- Preflight reads bounded recorder observation windows; its queries do not consume
  recorded observations or bind GPS sockets. Original check predicates and CLI
  defaults are retained; added checks cover actual recording and fresh calibrated/
  aligned IMU plus two live encoder message streams. Zero wheel motion passes.
- Producer owns sensors and forwards RTCM. Four existing raw GNSS UDP feeds and
  their meaning remain unchanged. Raw sensor/pose data is separate from those feeds.
- Initial engine-off boot procedure permits measured-stationary bias calibration;
  heading alignment is separate. Later producer restarts require explicit stopped
  engine-off confirmation before calibration; raw logging continues while waiting.
- A one-attempt optional GPS restart is explicit and requires stopped-Pause checks
  and engine-off confirmation. It is not an automatic boot restart loop.
- Dated local preflight reports, source/boot/session metadata, raw/CSV segments,
  loss diagnostics and closed-segment checksums support off-tractor review/export.

## Caller disposition

The new startup service calls the canonical module directly. The prepared installer
backs up and replaces installed units only under explicit stationary installation
approval; no local implementation action changed installed units. Current live
commands still reference the installed legacy stack until deployment.

Frozen current and historical mission packages, existing tests, controller,
arbitrator implementation and firmware are unchanged. The current Auto launcher
starts a private logger and runs a UDP-binding preflight; it is **incompatible with
the boot recorder unless migrated through a reviewed successor package**. The
manual recording workflow does not launch that package. The export utility verifies
closed source segments and supplies a compatible field CSV/time window for future
collector integration without stopping continuous capture.

The canonical sensor path reuses reviewed parsing/scaling/model logic by direct
stable modules. Dated observation experiments remain preserved for reproducibility.
No wheel scale calibration, slip detector or Pure Pursuit estimated-pose input was
implemented. No degraded-motion authorization or watchdog was changed.

## Validation performed locally

- **245 tests passed**, including 31 new consolidation tests and the existing
  production/control/startup/IMU/replay/analysis regressions.
- New comparisons cover the old GPS predicates, satellite WARN/FAIL behavior,
  steering/transmission/JRK/firmware predicates, startup-proof fallback and warning
  rendering. Independent original steering implementation avoids contamination
  from historical monkey-patching in the regression suite.
- Real local socket-pair tests cover framed ACK/query behavior and non-consuming
  repeated queries; local UDP test covers duplicate status suppression. No remote
  or hardware connection was used.
- Guarded synthetic boot calibration retains all 1,700 raw IMU observations and
  aligns only when trustworthy heading is available. Additional checks cover
  pre-calibration raw capture, unarmed restart and zero-motion encoder liveness.
- Closed-file checksums, compatible CSV generation, time-window export, tamper
  rejection, producer gaps, duplicate ACKs, bounded frames, threaded sequence
  allocation, low storage, live-socket ownership refusal and failed ntfy are tested.
- Python compilation, module import-use audit, CLI help and review-only installer
  passed. Frozen production mission package verification passed unchanged.

Unit tests do not establish installed systemd alias behavior, sensor acquisition,
actual engine-off detection, Pi timing/load/storage consumption or phone notification
arrival timing. Those are stationary commissioning/field measurements. System timezone
data, pyserial, smbus2, device permissions and journal access are deployment prerequisites.

## Diff and next checkpoint

Full review patch: `outputs/20261118_consolidation_review.patch` in the Windows
checkout. It includes new sources, units, tests, design/review documents and the
current-context updates. It is a review artifact, not a deployment command.

Review/commit/synchronization precedes a separately approved stationary installation.
No direct field activation commands have been issued during implementation.
