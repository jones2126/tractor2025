# Tractor01 startup, telemetry and preflight consolidation

November 18, 2026 — reviewed design; local implementation subsequently authorized.
The initial inspection changed documentation only. The subsequent implementation
is documented in `tractor_rpi/setup/CONSOLIDATED_STARTUP_REVIEW.md`; it remains
prepared for GitHub synchronization after the requested diff review; it is not installed or field validated.
No Tractor01 hardware access or service changes were performed.

## Intended operator experience

Boot Tractor01; telemetry capture starts automatically in dated files, including
the period before sensors become ready. With the tractor stationary, engine off,
blades disengaged and phone in Pause, startup prepares sensors and runs the same
canonical preflight available manually. A passing report includes recorder health,
IMU bias/alignment state and both live encoder streams. No second recorder or
manual activation command is needed for a manual data-collection run.

Boot readiness is not a motion command. Existing phone/arbitrator/Teensy authority,
500 ms watchdogs and current navigation gates remain unchanged. Pose estimates
remain observational; this work does not implement the proposed 3 m/30 s policy.

Al approved proceeding with this design and migration inspection in the November
18 conversation. The preceding proposal explicitly made design/inventory the next
step. Al subsequently reviewed this plan and approved implementation with comments
on service naming, daily storage, ten-second stop markers and ntfy/readiness.
Separate field installation/validation still follows local implementation review.

## What exists and why it needs consolidation

The actual preflight execution chain is:

1. `mission_preflight_phone_control_20261109.py` imports the October profile,
   replaces its underlying service list and invokes `profile.base.main()`.
2. `mission_preflight_20261002.py` imports the August module, retains its original
   functions and replaces `gps_checks`, `service_checks` and `print_checks` at
   runtime. It adds Heading satellite thresholds, startup proof, network checks
   and WARN rendering.
3. `mission_preflight_20260804.py` supplies the main implementation, UDP sampling,
   core checks, CLI, failure diagnostics and exit handling.

The standalone centered-input successor is a separate historical profile, not a
new production requirement. The current v5 preflight does not invoke it.

`field_test_logger_20260828.py` consumes GPS unicast UDP 6009, status broadcast
6003 and GL-router TCP 6005. It writes joined CSV rows at roughly 20 Hz, driven
by distinct steering sequences (GPS fallback when steering is absent). It does
not record every raw IMU/encoder sample. Preflight currently binds 6009 too,
using socket reuse: concurrent unicast listeners do not guarantee each sees every
packet. A boot logger therefore requires a preflight sampling migration.

The November 17 position candidate owns F9Ps, IMU and encoders, forwards RTCM,
publishes the four raw GNSS feeds and writes its own sensor/shadow files. It is
not installed. Its temporary drop-in asserts `--stationary` and disables restart;
it is unsuitable as the permanent boot design. Its hardware-free model/parsers
are useful code to reuse directly, not another dated inheritance chain.

The current outer-perimeter launcher starts/stops a private field logger and its
collector expects an exact mission CSV. Simply enabling boot logging would create
duplicate owners and break that evidence contract.

## Proposed production ownership

Three clear programs, with ordinary direct imports of small shared schemas and
parsers where needed. No dated import/monkey-patch chain. Proposed stable names:

| Owner | Proposed entry point/service | Responsibilities |
|---|---|---|
| Position/sensor producer | `tractor_rpi/calc_publish_position.py`; `calc-publish-position.service`, old name an alias | Sole owner of both F9P USB devices, IMU acquisition and both identified encoder serial devices; RTCM forwarding; sensor calibration state; observational pose; four unchanged raw GNSS feeds |
| Continuous recorder | `tractor_rpi/telemetry_recorder.py`; `tractor-telemetry-recorder.service` | Sole boot-session evidence writer; UDP 6009, broadcast 6003, router TCP 6005 and local IMU/encoder/pose subscription; bounded recent observation buffers for preflight |
| Preflight | `tractor_rpi/tractor_preflight.py`; existing startup-preflight service invokes it | One explicit production check implementation; reads recent observations/status, never owns sensor hardware or competes for 6009; same CLI for startup, manual and dashboard use |

Keep the Heading ExecStartPre independent readback hook. Al requested a matching
service name: use `calc-publish-position.service`, with `rtcm-server.service` as
a compatibility alias to the same unit. The Teensy process becomes
`tractor-teensy-bridge.service`, with `tractor-rpi-arbitrator.service` as an alias;
keep its existing command-selection behavior and sole `/dev/teensy` ownership.
Do not reuse the retired, masked `teensy-bridge.service`. No extra daemon
solely to supervise or proxy the three programs is proposed.

Use a versioned local Unix socket protocol rather than another network UDP port.
Producer streams raw IMU/encoder samples, shadow pose and sensor state to recorder;
recorder exposes read-only recent sample windows/status to preflight. Explicit
framing, size bounds, sequence numbers, monotonic arrival/source times, boot ID,
producer session IDs and dropped-record counters are required. Preflight takes a
new observation interval, not an old PASS or a cached single snapshot. Enforce
local permissions and bounded request/subscription queues; slow preflight clients
must not stall capture. Missing recorder, transport gaps or missing observations
fail the applicable new readiness checks, never manufacture valid sensor state.

GNSS is recorded from 6009 once, not from each of 6002/6009/6010/6013. Preserve all
four producers' destinations, existing raw field meanings and nominal 20 Hz.
6002 remains the arbitrator feed; 6010 navigation; 6013 dashboard. No unvalidated
pose replaces legacy raw GPS/heading. Keep 6003 broadcast semantics and deduplicate
status by source sequence; verify concurrent consumers do not steal status data.

## Startup, calibration and restarts

1. Start recorder independently of GNSS/network convergence; open a unique dated
   boot-session directory and report missing sources while waiting. Sensors not
   ready is evidence, not a reason to omit early recording.
2. Existing GPS startup hook applies/reads back the Heading RAM profile. Producer
   opens devices, forwards corrections, emits observations and health. Retain the
   agreed background acquisition/main-loop ingestion split.
3. Producer maintains explicit IMU states: waiting for stationary conditions,
   bias calibration, bias valid, waiting for heading alignment, aligned, invalid.
   Reuse established settling/calibration duration and quality checks initially.
4. Gyro-bias calibration needs stillness, not just a valid heading. Engine-off
   calibration remains the commissioned condition. No verified automatic engine
   state input exists: voltage cannot distinguish charger/alternator. Use fresh
   Pause/neutral/zero output, stationary GNSS/wheels and IMU stability as supporting
   evidence, and make the engine-off startup condition explicit in the runbook.
   These signals do not conclusively prove engine-off. Do not label it detected.
5. Align heading when fresh, Fixed/valid GNSS heading and RTK position meet the
   existing health conditions. Record alignment source and timestamp separately
   from bias. Five-second experimental re-anchors remain separate from raw yaw.
6. Startup preflight waits for readiness and records the results. Later preflight
   runs verify calibration state without recalibrating. They cannot open I2C or
   encoder serial devices. A producer restart invalidates its prior calibration;
   it must not blindly recalibrate on a moving or engine-running tractor.
7. Recorder restart creates a new segment with explicit missing-data interval.
   Producer restart does not stop/restart the recorder or discard previous files.
   Capture remains active through GPS recovery. No permanent `--stationary`
   assertion in an auto-restarting service.

Engine-off verification after an unexpected engine-on restart is a commissioning
decision: safest initial behavior is remain uncalibrated until stopped engine-off
confirmation, while raw logging continues. Normal planned engine-off boot should
need no extra acquisition commands. This decision must be resolved before deployment.

## Evidence contract

One dated directory per boot/session with append-only segments, source Git SHA,
firmware identity, source/device IDs, schemas, configuration and calibration events.
Record raw IMU at its native nominal 104 Hz, both encoder streams nominal 10 Hz,
one GNSS snapshot at 20 Hz, unique status frames, router observations and shadow
pose separately. Preserve firmware raw ticks, filtered values, WARN/ERR and gaps;
do not rescale or smooth away disagreements in the raw evidence.

Retain a joined field CSV compatible with existing analysis where practical,
alongside raw sensor files and recorder health. Preserve UTC and monotonic times,
source update times/counters and missing-data age; held 20 Hz snapshots must not
be counted as distinct 5 Hz heading observations. Alignment/antenna offsets and
timing uncertainty remain explicit for later RTK-versus-encoder comparisons.

Use periodic flush, atomic metadata, bounded queues, explicit disk/write failures,
free-space monitoring and time/size rotation. Hash closed segments; never hash an
actively changing file as final. Detect and label incomplete prior sessions after
power loss. No automatic deletion of originals or overwrite on restart. Retention
and capacity thresholds require a measured Pi storage budget; implement warnings
and new-capture failure reporting before any cleanup policy.

Manual runs need only an optional start/end or maneuver marker for replay focus;
absence of markers cannot prevent recording. Record clockwise/counterclockwise,
reverse and hard-turn events if supplied. Healthy RTK Fixed is the comparison
reference, not independent surveyed truth. Wheel disagreement can reflect scale,
turn geometry, timing or slip; do not declare slip from disagreement alone.

## One preflight: parity first, additions separately

The companion CHECK_PARITY.md inventories current checks, CLI and omissions.
Preserve those exact pass predicates, warning semantics, diagnostic conditions,
CLI defaults and exit status first. Then separately add reviewed checks for:

- Recorder alive and actually writing advancing records, adequate capacity,
  no unexplained loss, fresh producer session and complete recent sample window.
- IMU fresh samples, stationary bias calibration passed, heading alignment valid
  with recorded source age. Logging before calibration must still work.
- Both encoder identities present and sending fresh valid ENC records with
  advancing device/sample counters. Stationary zero delta/zero speed must pass.
  Proposed initial test: at least 5 distinct valid records per side over 5 s,
  latest age <=0.5 s, matching USB identity and no active stale/error state;
  nominal 10 Hz and WARN/ERR counts shown diagnostically. These are new proposed
  thresholds, not established production rules. No distance calibration, wheel
  movement test, wheel scale gate or firmware work in this scope.

Preflight report should distinguish recording readiness from existing GPS/control
checks, so sensor experiments do not silently change live Manual arbitrator gates.
Adding a preflight condition is not adding a low-level motion trigger.

Keep ntfy best-effort. Local results must exist without internet/notifications.
Preserve full timestamped failure and post-recovery reports. Optional RTCM recovery
is a separate explicit one-attempt option, only after confirmed stopped Pause,
engine-off conditions. Never restart on arbitrary failures or count restart as
success. No unattended restart loop; startup default remains diagnostic.

The November 18 field restart preceded RTK Fixed and heading Fixed recovery after
five minutes of Float; root cause was not isolated. See the future-improvement
note. Improving convergence wording/trends is separate from acceptance thresholds.
Heading NMEA absence only verifies that output condition, not all F9P configuration.

## Caller migration and historical preservation

REFERENCE_INVENTORY.md contains every tracked matching reference found during
inspection, its line, firmware context and proposed classification. Do not delete
dated files merely because they are old; imported historical profiles still need
their dependencies until all intended callers are handled.

| Caller group | Plan |
|---|---|
| Startup unit, production November 16 wrapper and repair installer | Update to one canonical preflight and recorder readiness; retain network gating, exits, ntfy and log commands; prefer direct Python startup mode after parity, retire duplicate shell logic only when fully replaced |
| Manual operational references, AGENTS/current field context, setup instructions | Migrate to same canonical command after implementation; distinguish current instructions from historical evidence |
| November 4 frozen outer-perimeter launcher/dashboard/verifier | Do not edit silently. Build a separately reviewed successor integration package: canonical preflight, no private logger start/kill, continuous-recorder run markers/export and matching collector/verifier contract; preserve exact route/controller/hashes |
| Collector and replay tooling | Consume exact closed recorder segments plus mission time bounds, hashes and manifest; create a compatible mission slice/export if old tools need a single CSV; never stop boot capture to satisfy a collector |
| Historical mission launchers, builders and tests | Preserve for reproduction. Templates/builders that may generate future launchers are explicitly review-required; do not globally search/replace historical scripts |
| Existing dated preflight tests | Leave unchanged as historical behavior reference; add independent parity tests against the new implementation without importing monkey-patching profiles in the same process |
| Three dated preflight entry points | Retain unchanged for historical callers initially; production uses only canonical implementation. Any later thin compatibility wrapper needs explicit scope and parity review, not another implementation |
| Standalone IMU logger and November 17 field candidate | Preserve as observation experiment lineage; do not install both acquisition paths; extract direct stable modules for production and clearly retire candidate installation instructions from current workflow only after deployment |

## Implementation sequence and verification

1. Approve this ownership/IPC design, calibration restart policy and caller scope.
2. Extract one canonical preflight with explicit functions and production profile;
   prove predicate/CLI/output/exit parity using offline fixtures. No new sensor
   gates in this first parity step. Inventory all imports for actual use.
3. Refactor candidate into stable sensor producer and continuous recorder with
   versioned local interfaces; test bounded memory, source identity/age, reconnects,
   slow clients, missing devices, partial writes, disk full and incomplete shutdown.
4. Integrate preflight sample windows via recorder. Prove full observed rates and
   all old checks with concurrent recorder/preflight; no socket competition, no
   hardware re-opening, no data loss during checks. Add new gates separately.
5. Prepare boot units/installer with backups and rollback, preserving Heading hook
   and production control owners. Update operational docs and successor mission
   integration/collection contracts. Do not install or modify frozen packages.
6. Run existing software regressions plus focused meaningful new tests, syntax
   checks and source-diff review. Deliver diff/migration report before committing;
   synchronize local/GitHub after review. No hardware tests in implementation.
7. Separately approve stationary installation: clean exact main SHA, installed unit
   inspection, known device owners, engine-off boot, sensor readiness/preflight,
   unique dated logs, four existing UDP consumer checks, continuous recording while
   preflight runs, closed-file checksum verification and rollback exercise.
8. Separately approve manual route after stationary acceptance. Record and compare
   RTK/IMU/encoder/status evidence; no estimated pose supplied to Pure Pursuit yet.

Acceptance: one production preflight; boot capture without manual acquisition
commands; calibration/alignment explicit; live encoders checked without needing
motion; all current checks traceable; recorder survives preflight/recovery; no
duplicate source owners; no new motion policy; raw evidence and frozen history
preserved. Unit-test success is not field validation.

## References

- `obsidian_vault/02-testing/20261113-prompt-to-Consolidate-preflight-scripts.txt`
- `field_testing/TRACTOR01_CURRENT_FIELD_CONTEXT.md`
- `tractor_rpi/setup/CALC_PUBLISH_POSITION_20261117.md`
- `obsidian_vault/05-future-ideas/20261118-preflight-optional-rtcm-restart.md`
- Current source SHA at inspection: `561d95c49996ea77feb8b371ae1403f63a87b49a`.
