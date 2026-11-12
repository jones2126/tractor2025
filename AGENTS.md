# Codex Context — Tractor Robot Project

## Working Directory
Use the permanent checkout for the machine running Codex:

- Windows: `C:\Repos\tractor2025`
- RPi5NAS: `/home/al/repos/tractor2025`
- Tractor01 runtime checkout: `/home/al/tractor2025`

Do not confuse a Codex worktree, temporary cleanup clone, or historical
`tractor2025-rpi-lease` path with the production checkout. GitHub `main` is the
source of truth. Tractor01 pulls from GitHub; it is not the development master.

For any live Tractor01 field work, read
`field_testing/TRACTOR01_CURRENT_FIELD_CONTEXT.md` before suggesting commands.
That file contains the current production paths, service owners, dashboard
semantics, preflight command, and operator-confirmed live-system automation.

## Who I Am
Al — hobbyist robotics developer. Not a professional software engineer.
Walk me through processes step by step when needed.

## The Robot Project
Autonomous outdoor robot on a **Cub Cadet XT1 LT42 EFI** riding mower chassis.
Full documentation: `obsidian_vault/00-project-overview.md`

### Hardware Summary
| Component | Details |
|-----------|---------|
| Low-level controller | Teensy 4.1 — steering PID, transmission, RPi command lease, e-stop relay |
| Primary compute | Raspberry Pi 5 — navigation, GPS, teleoperation |
| Steering | IBT-2 H-bridge, PID (kp=1.0, ki=0.0, kd=0.0), pot feedback A9 |
| Transmission | Pololu JRK G2 21v3 linear actuator, 10-bucket system, commissioned neutral=2836 |
| GPS | 2× ArduSimple ZED-F9P RTK moving-base pair; JSON published at 20 Hz and RELPOSNED heading at approximately 5 Hz |
| Retired hardware | NRF24 is no longer physically used on Tractor01; do not add it to the production build |
| Camera | OAK-D stereo (WebRTC teleoperation) — DepthAI must stay at 2.30.0.0 |
| LED tower | PCA9685 via I2C, channels 8–12 |

### Robot RPi Connection (onboard the tractor)
- Hostname: `raspberrypi` | User: `al`
- Production field Ethernet: `192.168.8.20/24`, GL router gateway `192.168.8.1`
- ZeroTier IP: `192.168.193.76`
- Serial bridge: `460800 baud` USB (`/dev/teensy`)
- Reachable from the dev NAS over ZeroTier (`192.168.193.x` network).

### RPi Command Modes
| Mode | Value | Behavior |
|------|-------|----------|
| Pause | 0 | Transmission neutral, steering hold |
| Manual | 1 | Signed drive percent and normalized steering from the RPi arbitrator |
| Auto | 2 | Calibrated m/s and normalized steering from the RPi arbitrator |
| E-stop | 3 | Latches the Teensy software e-stop relay output |
| Reset Pause | 4 | Clears the software e-stop only as a zero Pause command |

### UDP Ports
| Port | Content |
|------|---------|
| 6002 | GPS/RTK state for the bridge |
| 6003 | Combined Teensy, command-lease, and RPi-arbitrator status broadcast |
| 6004 | Legacy `cmd_vel` input; production arbitrator rejects/bypasses it as a motion authority |
| 6009 | Dedicated 20 Hz GPS feed for the field logger |
| 6010 | Dedicated 20 Hz navigation feed for Pure Pursuit |
| 6011 | Mission-dashboard Pause/resume control |
| 6012 | Pure Pursuit dashboard telemetry |
| 6013 | Dedicated GPS feed for the mission dashboard |
| 6014 | Authenticated phone intent to the RPi safety arbitrator |
| 6015 | Pure Pursuit proposal and health feed to the RPi safety arbitrator |

### Current Firmware
| Board | File |
|-------|------|
| Tractor Teensy 4.1 (commissioned on hardware) | [`tractor_teensy/src/teensy_main_20261110.cpp`](tractor_teensy/src/teensy_main_20261110.cpp), firmware identity `teensy_main_20261110_rpi_command_lease_v5` |
| Canonical build configuration | [`tractor_teensy/platformio.ini`](tractor_teensy/platformio.ini); it selects only the consolidated production source |
| Retired NRF handheld | Historical source remains under `radiocontrol_nrf24radio/`, but the radio is not physically used and has no production command authority |

The commissioned v5 firmware was compiled and uploaded from GitHub `main`
commit `755bde9` on 2026-11-11. It implements steering PID and response watchdog,
JRK transmission control and diagnostics, 500 ms RPi command lease, session and
sequence validation, Pause/Manual/Auto/software-E-stop/guarded-reset modes, and
telemetry including JRK VIN millivolts. Commissioned steering values are right
`191`, center `525`, and left `885`; JRK neutral is `2836`.

### Active Development Focus (as of 2026-11-11)
- Review a bounded 0.3 m/s continuation state for specified short RTK/heading
  degradations; inspection and plan come before implementation.
- Field-validate the independent continuous JRK VIN logger before installation.
- Improve route recovery/corridor guidance and continue evidence-backed replay
  analysis without weakening immediate local watchdogs.
- IBT-2 Gen2 wiring remains planned but not implemented.

### Known Issues
1. Voltage alone cannot distinguish the engine alternator from an external battery charger.
2. All grounds must be tied — previous Teensy damage from 328Ω between Teensy and JRK G2 grounds
3. DepthAI must stay at version `2.30.0.0` (3.0 is incompatible)
4. IBT-2 overcurrent latch — reset by pulling motor fuse 5 sec; R_EN/L_EN hardwired to 5V currently
5. Pixel hotspot MAC randomization can cause WiFi failures — clear `802-11-wireless.bssid` if needed.
6. The installed Pi throttling/5 V `pilog` automation is live-system evidence,
   but its exact source and installer are not yet represented in GitHub `main`.

---

## Current Tractor01 Production Software

Treat this as the current key-program inventory. Confirm installed unit contents
against these repository files before changing services because the RPi may not
yet have pulled the newest `main` commit.

### Boot services and persistent processes

| Service/process | Repository program or unit | Role |
|---|---|---|
| `rtcm-server.service` | [`tractor_rpi/setup/rtcm-server-tractor01.service`](tractor_rpi/setup/rtcm-server-tractor01.service), [`tractor_rpi/rtcm_server_20260828.py`](tractor_rpi/rtcm_server_20260828.py) | Owns both F9P serial devices, receives base corrections, decodes position/heading, and publishes the dedicated GPS feeds. Its `ExecStartPre` runs [`configure_dual_f9p_5hz_profile_20260923.py`](tractor_rpi/testing/configure_dual_f9p_5hz_profile_20260923.py) with `--heading-startup` and requires independent RAM readback. |
| `tractor-rpi-arbitrator.service` | [`tractor-rpi-arbitrator.service`](field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED/tractor-rpi-arbitrator.service), [`teensy_serial_bridge_rpi_arbitrator_20261104.py`](tractor_rpi/teensy_serial_bridge_rpi_arbitrator_20261104.py) | Sole production owner of `/dev/teensy`; joins phone intent, Pure Pursuit proposal, local health, route/corridor limits, and the command lease before emitting serial motion frames. Broadcasts combined status on UDP 6003. Its separately bounded phone-only `COMMUNICATIONS_DEGRADED` envelope is 30 seconds / 30 m and cannot override a failed local-health check. |
| `tractor-phone-arbitrator.service` | [`tractor-phone-arbitrator.service`](field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED/tractor-phone-arbitrator.service), [`wifi_rpi_arbitrator_control_20261104.py`](tractor_rpi/testing/webrtc/wifi_rpi_arbitrator_control_20261104.py) | Serves authenticated TLS phone control at `https://192.168.8.20:8765`, owns operator sessions, logs authoritative accepted commands, publishes diagnostics, sends phone intent on UDP 6014, and creates non-authoritative ntfy notices. It never sends legacy UDP 6004 motion commands. |
| `zerotier-one.service` | OS package, not repository-owned | Provides maintenance/SSH, RTK-base reachability, log transfer, and the mission-dashboard path. It is not the primary real-time phone-control authority. |
| `pilog.timer` / `pilog.service` | Installed Tractor01 automation; source not yet found in `main` | Approximately two-minute Pi power/resource log under `/var/log/pilog`: 5 V rail, `get_throttled`, load, ARM MHz, and temperature. Do not confuse its 5 V measurement with the 12 V JRK VIN telemetry. |

The production boot pair conflicts with and replaces the legacy
`teensy-bridge.service`, `tractor-wifi-control.service`,
`tractor-wifi-direct-mission.service`, and `tractor-wifi-recovery.service`.
Those legacy services must remain disabled/inactive. Exactly one process may
own `/dev/teensy`.

### Programs started for a mission

| Program | Role |
|---|---|
| [`mission_dashboard_production_20261109.py`](field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED/mission_dashboard_production_20261109.py) | Production dashboard normally exposed over ZeroTier on port 8088; displays authoritative phone, controller, GPS/heading, route, and actuator state. |
| [`run_outer_perimeter_20261104.sh`](field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED/run_outer_perimeter_20261104.sh) | Verifies the frozen package and preflight, starts the field logger, then launches the proposal-only controller for the exact outer-perimeter route. |
| [`mission_preflight_phone_control_20261109.py`](tractor_rpi/testing/mission_preflight_phone_control_20261109.py) | Canonical non-motion preflight for services, network routes, devices, correction stream, heading profile, telemetry, JRK health, safe Pause state, and exact firmware identity. |
| [`pure_pursuit_arbitrator_feed_20261031.py`](tractor_rpi/pure-pursuit/pure_pursuit_arbitrator_feed_20261031.py) | Wraps the frozen controller, preserves dashboard telemetry, hashes the exact route, and duplicates proposal/health data to UDP 6015. It does not own the Teensy. |
| [`pure_pursuit_controller_20261022.py`](tractor_rpi/pure-pursuit/pure_pursuit_controller_20261022.py) | Computes forward-only path tracking proposals and current GPS/heading/reacquisition health. The RPi safety arbitrator remains the final Pi-side motion authority. |
| [`field_test_logger_20260828.py`](tractor_rpi/field_test_logger_20260828.py) | Records synchronized UDP 6003 Teensy/arbitrator/JRK data and UDP 6009 GPS/heading data, including `jrk_vin_voltage_mV`. It normally runs only during a mission. |

### Prepared but not installed

[`battery_voltage_logger_20261111.py`](tractor_rpi/battery_voltage_logger_20261111.py)
and [`tractor-battery-logger.service`](tractor_rpi/setup/tractor-battery-logger.service)
are a **DRAFT - UNTESTED IN THE FIELD** candidate for independent 10-second
JRK VIN logging. They were not installed before Tractor01 shut down on
2026-11-11. Do not describe them as a running service until installation and a
stationary validation are explicitly approved and completed.

---

## Latest Tools for Developing Missions

Use the most recent applicable workflow, but do not select a script merely
because its filename has the newest date. Mission builders are often frozen to
one site, source dataset, route, or approval checkpoint.

### Current coverage-planning workflow

The current reusable workflow starts with traceable source-data approval, not
with waypoint generation:

1. Read the [coverage-planning handoff](field_testing/sites/62_Collins_multi_boundary_20260915/planning/20261020_coverage_planning_handoff/README.md)
   and its linked lineage/approval requirements.
2. Use the [Coverage Dataset Verifier](field_testing/tools/coverage_dataset_verifier/README.md)
   to inspect source evidence and record source decisions. Its append-only
   SHA-256 approval ledger is the first gate; it does not create a mission.
3. Use [`build_62_collins_staged_coverage_20261019.py`](field_testing/tools/build_62_collins_staged_coverage_20261019.py)
   only for the implemented Stage 0 reconciliation. Later cells, stripes,
   turns, waypoint generation, and launch integration remain blocked until the
   preceding owner-review gates are complete.
4. Use [`MULTI_BOUNDARY_TO_WEB_MISSION_WORKFLOW.md`](field_testing/MULTI_BOUNDARY_TO_WEB_MISSION_WORKFLOW.md)
   and [`SITE_COVERAGE_MISSION_WORKFLOW_20260724.md`](tractor_rpi/pure-pursuit/mission_planning/SITE_COVERAGE_MISSION_WORKFLOW_20260724.md)
   as process/reference guides. Their older example commands and runtime names
   must be reconciled with the current production stack before use.

For 62 Collins, the [site inventory](field_testing/sites/62_Collins_multi_boundary_20260915/site_inventory/README.md)
is the geometry/lineage starting point. Preserve raw captures, extracted
segments, inventory revisions, and approval records; never hand-edit a final
mission in a way that breaks regeneration or provenance.

### Building and reviewing a specific mission package

- Put each mission in a self-contained `mission_plans/<dated_name>/` package
  with the builder, generated waypoint file, matching waypoint-audit CSV,
  machine-readable validation/report, review visualization, verifier,
  launcher, dashboard integration, README, and pinned hashes.
- Use the builders under [`field_testing/tools/`](field_testing/tools/) and the
  prior site packages as reviewed implementations, not as generic inputs. For
  example, [`build_62_collins_consolidated_perimeter_mission_20260929.py`](field_testing/tools/build_62_collins_consolidated_perimeter_mission_20260929.py)
  is specific to that consolidated-perimeter package.
- Keep generated candidates labeled **REVIEW ONLY** or
  **DRAFT - UNTESTED IN THE FIELD** until the route image, geometry, audit,
  limitations, verifier, and exact launch package receive the required
  approvals. Building a route never authorizes field motion.
- The currently exercised outer-perimeter runtime is the frozen
  [2026-11-04 phone-arbitrator package](field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261104_PHONE_ARBITRATOR_OUTER_PERIMETER_DRAFT_UNTESTED/README.md).
  Do not silently substitute a new route or controller into that package.

---

## Tools for Gathering End-of-Mission Telemetry

The logger and runtime producers are part of the evidence chain:

| Evidence | Producer / normal location | What it establishes |
|---|---|---|
| Synchronized mission CSV | [`field_test_logger_20260828.py`](tractor_rpi/field_test_logger_20260828.py), mission-specific directory under `/home/al/field_logs/` | UDP 6003 controller/Teensy/JRK state plus UDP 6009 GPS/heading state, including JRK VIN telemetry |
| Pure Pursuit CSV | Current controller, normally `/home/al/repos/field-testing-data/pursuit_log_*.csv` | Route index, pose, target, tracking, GPS/heading health, proposal, and controller decisions |
| Authoritative phone/server JSONL | `tractor-phone-arbitrator.service`, under its configured `/home/al/field_logs/` directory | Accepted phone requests, mode/session changes, and server-side timing |
| Phone flight-recorder JSONL | Uploaded by the phone-control page beside the server log | Client-observed scheduler, request, acknowledgement, timeout, and network events; diagnostic rather than motion authority |
| Pi power/resource CSV | Installed `pilog.timer`, `/var/log/pilog/` | 5 V input, throttling flags, load, ARM frequency, and CPU temperature; not 12 V battery voltage |
| Runtime snapshot | `/run/tractor-phone-control/diagnostics.json` | Latest phone-arbitrator diagnostics at collection time |
| Service/network/repository state | `systemctl`, `journalctl`, `ip`, `zerotier-cli`, and Git read-only commands | Exact processes, current-boot events, network paths, commit, and working-tree state |

After a mission, keep the tractor stopped, engine off, blades disengaged, and
handheld in Pause. Stop the foreground dashboard/controller normally, then
copy evidence without modifying the source logs. Record the mission start time
and exact file paths before using time-based searches.

The most complete repository collector example is
[`collect_master_manual_phone_run_20261006.sh`](field_testing/sites/62_Collins_multi_boundary_20260915/mission_plans/20261006_master_manual_phone_rerun_field_test/collect_master_manual_phone_run_20261006.sh).
It demonstrates staged copying, Pi logs, diagnostics, an internal manifest, a
compressed archive, and an archive checksum. It is a historical template with
old mission paths and old service names; **do not run it unchanged for the
current production stack**. A new mission package should include an adapted
collector that targets `rtcm-server.service`,
`tractor-rpi-arbitrator.service`, and `tractor-phone-arbitrator.service` and
the package's exact log locations.

Every completed collection must contain, when available:

- mission, pursuit, phone-server, phone-flight-recorder, recovery, and Pi logs;
- dashboard/controller diagnostics, current-boot service journals and status,
  network state, repository SHA/status, collection metadata, and the exact
  mission/audit files or their hashes;
- a relative-path SHA-256 manifest created after collection; and
- a `.tgz` archive plus a separate SHA-256 file, both copied off Tractor01 and
  independently verified before any cleanup.

Never delete or truncate the original Tractor01 evidence merely because an
archive was created. Cleanup requires a verified second copy and explicit
scope. The independent battery logger is not yet a source until its prepared
but uninstalled status above changes through field validation.

---

## Tools for Analyzing End-of-Mission Telemetry

Use preserved copies for analysis and keep generated results separate from raw
evidence. The preferred current outer-perimeter replay tool is
[`build_tractor01_mission_replay_20261109.py`](field_testing/tools/build_tractor01_mission_replay_20261109.py).
Pass explicit `--evidence-root`, mission CSV, pursuit CSV, phone JSONL, mission
path, audit path, run ID, and output directory for a new run; do not rely on its
November 9 defaults. It produces:

- a self-contained historical replay HTML;
- `REPORT.md` with source filenames and SHA-256 hashes;
- `safety_pause_timeline.csv`, grouping complete motion interruptions;
- `pause_transition_details.csv`, retaining individual low-level Pause
  transitions; and
- `safety_condition_loss_episodes.csv`, measuring each continuous raw RTK,
  heading, GPS-feed, or corridor threshold breach and its duration.

The [November 11 analysis README](field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261111_phone_outer_perimeter_replay/README.md)
is the current example of these outputs and explains their different duration
semantics. Replay sample numbers and route waypoint numbers are not the same.
Overlapping condition-loss episodes must not be summed as independent stopped
time.

Shared replay support lives in
[`build_tractor_mission_replays_20260919.py`](field_testing/tools/build_tractor_mission_replays_20260919.py),
[`build_tractor01_mission_replay_20260923.py`](field_testing/tools/build_tractor01_mission_replay_20260923.py),
and [`tractor-mission-replay-template.html`](field_testing/tools/tractor-mission-replay-template.html).
Use focused analyzers only for the dataset named in the script/documentation;
older dated analyzers are preserved for reproducibility, not automatically the
best tool for a new run.

[`build_rtk_slow_crawl_counterfactual_20261110.py`](field_testing/tools/build_rtk_slow_crawl_counterfactual_20261110.py)
is a bounded historical what-if tool, not a reconstruction, controller change,
or motion authorization. Counterfactual outputs and any policy inferred from
them remain **DRAFT - UNTESTED IN THE FIELD** until separately implemented,
reviewed, and field validated.

---

## Durable Operational and Evidence Rules

### Field-command discipline

- PuTTY normally opens in `/home/al`, not the repository. Start commands with
  `cd /home/al/tractor2025` or use an absolute repository path.
- Before field work, confirm `main`, a clean tree, the exact expected commit,
  the exact firmware identity, and the installed service commands.
- Keep source/build approval, service installation, firmware flashing,
  stationary commissioning, engine-on movement, and blade operation as
  separate approvals. Never infer a later approval from an earlier one.
- Do not restart a healthy service merely to rediscover a fact already proven
  for the current boot. Use current-boot journals and the canonical preflight.
- Distinguish `Phone request` from `Actual output`. The RPi supervisor or
  Teensy may correctly hold Pause even when the phone requests Manual or Auto.
- A review dashboard intentionally reports no controller telemetry because it
  cannot start the controller. Do not diagnose those expected labels as a
  fault without checking live bridge/GPS state.

### October 29 phone-network investigation lessons

- Server acceptance records are authoritative for commands and session state.
  Phone logs can contain retried duplicate event batches.
- Never estimate phone-to-server one-way latency by subtracting unsynchronized
  wall clocks. Use RTT or a measured clock-offset exchange; safety freshness is
  based on local monotonic arrival time.
- Historical `send_queued` counted 200 ms timer collisions while one HTTPS
  request was in flight; it was not OS packet-queue depth. With median RTT near
  the send period, collisions were expected.
- Historical unexplained aborts were the page's 1,000 ms application timeout.
  HTTP 409 responses were usually consequences of session invalidation and
  recovery races, not the initiating transport fault.
- Direct GL-router phone control materially outperformed the former ZeroTier
  heartbeat path in stationary tests. Use the local Ethernet/router path for
  real-time phone authority and ZeroTier for dashboard, maintenance, RTK-base
  access, and log transfer.
- Keep the 500 ms phone/Pure-Pursuit/Teensy freshness and command-lease limits
  independent from any longer degraded-mode policy. Do not implement degraded
  behavior by extending a watchdog.
- A bounded degraded incident begins at the first loss. Reconnects cannot reset
  its time or distance origin; reset requires explicit stopped-Pause operator
  acknowledgement. Any missing local health input fails closed.
- ntfy is best-effort information only. Server acknowledgement does not prove
  timely phone delivery, and a notification must never grant, reset, or extend
  motion authority.
- Diagnose communications by measured layer: phone scheduler, phone/router
  link, server processing, ZeroTier direct/relay state, Pi scheduling,
  Pi-to-Teensy status, and Pure Pursuit freshness. Do not attribute a fault to
  an unmeasured layer.

### Evidence handling

- Preserve raw files and SHA-256 manifests before analysis. Derived replays and
  CSVs must identify their source files and hashes.
- Grouped motion interruptions and raw threshold-breach episodes are different
  measurements. Do not sum overlapping condition durations as if they were
  independent stopped time.
- Mark counterfactual replays and proposed controller policies
  **DRAFT - UNTESTED IN THE FIELD**. A replay is not field validation.
- The current durable live-state reference is
  [`field_testing/TRACTOR01_CURRENT_FIELD_CONTEXT.md`](field_testing/TRACTOR01_CURRENT_FIELD_CONTEXT.md).

---

## Repo Structure
```
tractor2025/
├── AGENTS.md                     ← this file
├── README.md
├── markdown_cross_reference.csv
├── obsidian_vault/               ← Obsidian engineering notebook (open as vault)
│   ├── 00-project-overview.md   ← hardware ref, key files, RPi connection
│   ├── 01-plan/milestones.md    ← project milestones
│   ├── 02-testing/              ← dated session logs (YYYYMMDD-topic.md)
│   ├── 03-design/               ← component notes, wiring, CAD prompts
│   ├── 04-reference/            ← laptop setup, git workflow, PDFs
│   ├── 05-future-ideas/         ← deferred work (IBT-2 Gen2, etc.)
│   └── attachments/             ← all images
├── tractor_teensy/              ← Teensy 4.1 firmware (PlatformIO)
│   ├── platformio.ini             ← canonical software build configuration
│   └── src/teensy_main_20261110.cpp
├── radiocontrol_nrf24radio/     ← Retired handheld NRF firmware retained as history
├── tractor_rpi/                 ← RPi 5 scripts and testing utilities
│   ├── f9p/                     ← dual F9P RTK scripts
│   ├── light-tower/             ← LED tower control
│   ├── oak-camera/              ← OAK-D WebRTC teleoperation
│   ├── pure-pursuit/            ← path following algorithm
│   ├── servo-board/             ← PCA9685 servo board scripts
│   ├── setup/                   ← RPi setup and install scripts
│   ├── teensy/                  ← Teensy serial bridge scripts
│   ├── testing/                 ← test sketches and scripts
│   └── webrtc/                  ← WebRTC streaming
└── BridgevilleRTKBase/          ← RTK base station (Bridgeville PA)
```

---

## Development Machines

The permanent Windows checkout used in the 2026-11-11 cleanup and replay work
is `C:\Repos\tractor2025`. RPi5NAS is also a development/NAS machine. Always
check the actual current working directory and synchronize through GitHub
`main`; do not assume one checkout already contains another checkout's work.

### RPi5NAS

- Hostname: `RPi5NAS` | User: `al`
- Hardware: Raspberry Pi 5 Model B Rev 1.0 (aarch64)
- OS: Ubuntu 25.04 (Plucky Puffin), kernel `6.14.0-raspi`
- Local IPs: `192.168.1.2`, `192.168.1.205` | ZeroTier IP: `192.168.193.217`
- Also runs as a NAS: 2× 1.8 TB NVMe (`nvme0n1`, `nvme1n1`, Crucial P3) + Docker.
- Repo lives on the SD card root (`/dev/mmcblk0p2`, `/`), not the NVMe pool.
- Role: development, evidence storage, and NAS services. Tractor01 pulls and
  runs reviewed GitHub revisions.
