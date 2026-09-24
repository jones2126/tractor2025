# Master/manual mission and dual-F9P field session — 2026-09-23

## Outcome

The reviewed 62 Collins master/manual mission completed all **19,825
waypoints** with the mower deck disengaged. The tractor retained the guarded
5 Hz dual-F9P profile and used the proven September 18 runtime policy: RTK
Fixed plus `headValid` were mandatory while driving, recovery had no added
five-second delay, and GPS/heading recovery did not require a handheld mode
cycle. Strict carrier, baseline, and heading-accuracy checks remained in the
stationary preflight and mission-start gate.

The session also included heading-F9P recovery, stationary 10 Hz and 5 Hz
audits, a moving 5 Hz manual test, a short initial mission attempt, the
completed master mission, and approximately 28 minutes of manually driven
stripes. All eleven September 23 logs and configuration/audit backups were
copied from tractor01 and verified locally with matching SHA-256 hashes before
shutdown.

Closeout revision: `fcec1e9c9163567b965f0bedaae8f962443faeb6`. Local,
GitHub, and tractor01 matched this revision at field closeout; tractor01's work
tree was clean.

## Deployed changes during the session

- Updated `SPEED_CAL_20260908_1P8_TEST[]` in
  `tractor_teensy/src/teensy_main_20260914.cpp`: the former 0.40 m/s entry was
  changed to **0.14 m/s at target 2428**, and the 0.75 m/s entry was removed.
  The firmware was deployed and preflight observed
  `teensy_main_20260914`.
- Added the reviewed/resampled 19,825-waypoint master/manual mission,
  verification script, dedicated launcher, and mission dashboard.
- Added repeating ten-second safety-stop voice announcements and ntfy delivery
  of the temporary ZeroTier dashboard URL.
- Corrected the mission dashboard's automatic-recovery wording and kept one
  voice-guidance session armed from start-position guidance into mission
  safety monitoring.
- Added and applied the guarded dual-F9P 5 Hz/MSM4 configuration tool with
  backup, independent readback, and restore support.
- Restored the September 18 basic runtime gate and automatic recovery in the
  dedicated master/manual launcher while retaining strict preflight and start
  checks.

## Measured run results

### Completed master/manual mission

Source files:

- `field_testing/sites/62_Collins_multi_boundary_20260915/runs/20260923_124933_master_manual_5hz/master_manual_field_20260923_124933.csv`
- `field_testing/sites/62_Collins_multi_boundary_20260915/runs/20260923_124933_master_manual_5hz/pursuit_log_20260923_124935.csv`

| Metric | Result |
| --- | ---: |
| Mission elapsed time | 60.90 min |
| Controller `driving=True` time | 51.69 min |
| Final waypoint | 19,824 of 19,825; goal reached |
| Cross-track median | 0.062 m |
| Cross-track RMS | 0.249 m |
| Cross-track p95 | 0.595 m |
| Cross-track p99 | 0.811 m |
| Cross-track maximum | 1.714 m |
| Recoverable safety-stop episodes | 109 |
| Safety-stop time | 468.53 s / 7.81 min |

Safety-stop episode counts were 66 position/RTK, 16 heading, and 27 NRF24
radio. The longest radio stops began at mission elapsed 451.66 s, 1,208.04 s,
and 1,703.12 s and lasted 79.76 s, 51.22 s, and 48.97 s respectively. Seven
safety episodes lasted at least ten seconds. Automatic recovery allowed the
mission to continue without converting every transient into an
operator-latched stop.

The maximum cross-track value requires spatial review rather than acceptance
in isolation. Build a route-colored map and inspect the p95/max locations,
especially around manual joins, turns, radio-loss recoveries, and GPS recovery
events.

### September 18 comparison

The archived September 18 autonomous run recorded approximately 40.69 minutes
of `driving=True` time. Its cross-track median was also 0.062 m, with 0.192 m
RMS, 0.465 m p95, 0.600 m p99, and 0.842 m maximum. September 23 therefore
matched the September 18 median but had a wider error tail: 0.249 m RMS,
0.595 m p95, 0.811 m p99, and 1.714 m maximum.

September 18 logged 61 GPS-related interruptions totaling 36.75 seconds; 44
lasted no more than 0.25 seconds and the median was 0.20 seconds. They were not
experienced as repeated latched stops because that launcher recovered
immediately. The routes and event exposure were not identical, so the
cross-track comparison is a baseline rather than a controlled A/B result.

### Other September 23 motion

| Activity | Logged duration | Moving/driving time | Notes |
| --- | ---: | ---: | --- |
| Initial master attempt | 21.34 min | 0.32 min | Stopped after heading loss; not a tracking baseline |
| 5 Hz manual moving test | 6.05 min | 4.78 min | Radio remained `GOOD`; exposed moving RTK/carrier degradation |
| Completed master mission | 60.90 min | 51.69 min | Completed all waypoints |
| Manual stripes | 32.91 min | 28.40 min | Maximum recorded speed 1.517 m/s; radio field remained `GOOD` in this standalone log |
| **Total** | **121.19 min** | **85.18 min** | Four non-overlapping logs |

The tractor was topped off with **5 US quarts / 1.25 gallons** after the
session. Dividing by logged motion gives a preliminary **0.88 gal per moving
hour**. Dividing by total logger time gives **0.62 gal per logged hour**. Neither
is yet an engine fuel-consumption specification: the fuel total includes an
unknown amount of idling and diagnostic time, and engine-on time was not
recorded independently. Future tests should record engine start/stop time or
an hour-meter value and use a repeatable fill level.

## GPS and heading work

### Startup failure and recovery

At the first preflight, device symlinks existed and `rtcm-server.service` was
active, but the service had latched a fatal heading-port open failure from
startup. The misleading September 18 service start timestamp after a September
23 boot was caused by the Pi starting with an old clock before time
synchronization; it was not evidence that the process survived a power-off.
A service restart after `/dev/gps-heading` existed cleared the fatal state.

The heading receiver then presented lost/default-like settings: USB NMEA was
enabled, RELPOSNED was disabled, the navigation rate was 1 Hz, and UART1 was
38,400 baud. A guarded configuration attempt reported no ACK, but independent
readback showed that the intended 10 Hz/115,200/UBX-only settings had actually
been written. Lesson: independent readback is authoritative; a missed ACK is
not by itself proof that the write failed.

Do not simply assume an antenna failure when preflight shows plentiful heading
satellites plus NMEA output and no RELPOSNED. Treat that combination first as a
configuration/readback problem. A future startup tool should perform an
idempotent readback and guarded repair rather than blindly factory-resetting or
rewriting receivers on every boot.

### 5 Hz profile

The guarded profile changed both receivers to 200 ms measurements, changed the
Base-Link UART output from MSM7 to MSM4, retained RTCM 1230 and 4072.0, disabled
4072.1, kept the UART at 115,200 baud, and retained Heading USB UBX/RELPOSNED
with USB NMEA disabled.

Stationary results:

- Initial 10 Hz/MSM7 audit in the first location: 421/568 valid fixed frames,
  **74.120%**, at 4.732 observed frames/s.
- Clearer-location 10 Hz/MSM7 audit recorded separately: **91.738%** valid.
- 5 Hz/MSM4 audit: 594/598 valid fixed frames, **99.331%**, at 4.982 frames/s.

The moving manual comparison was less favorable than the stationary audit.
Using the September 18 basic runtime gate, 94.562% of moving epochs were
drivable; the stricter carrier/baseline/accuracy gate would have accepted only
87.147%. This did not prove F9P hardware degradation. Satellite counts and
mean C/N0 were not worse than September 18, while the receiver profile,
satellite geometry, route, and environment differed.

Decision: retain the 5 Hz configuration for the completed master comparison.
Do not restore the prior profile solely because it produces diagnostic
imperfections. Compare tracking, recovery, and location-specific failures from
the completed mission before changing the receiver profile again.

## Runtime-gate lesson

The September 18 run used `--basic-runtime-heading-gate`,
`--resume-stable-seconds 0.0`, and
`--no-operator-cycle-after-safety-loss`. Most of its GPS interruptions were
about 0.2 seconds and recovered automatically, which is why the operator did
not experience them as repeated latched stops.

The first September 23 launcher instead treated carrier, baseline, and heading
accuracy as independent runtime gates, required five continuously healthy
seconds, and required a handheld acknowledgement cycle. This amplified a
short receiver imperfection into a long or indefinite operational stop.

The final launcher intentionally separates:

- **strict startup qualification**: RTK Fixed, valid fixed-carrier heading,
  0.80–1.30 m baseline, heading accuracy at most 1 degree, safe modes, and
  complete preflight; from
- **supervised runtime availability**: current data, no fatal receiver error,
  RTK Fixed, and `headValid=True`, followed by bounded forward path
  reacquisition.

This is not a request for perfect GNSS data. It is a documented risk decision
for a supervised, blades-off run in open space with adequate buffer.

## Dashboard and voice workflow

The mission-specific dashboard now:

- sends its temporary ZeroTier URL to
  `https://ntfy.sh/rpi-tractor01-jones2126`;
- creates a new random 18-byte operator key for every dashboard process;
- provides spoken clock-direction and heading guidance to W1;
- remains voice-enabled after reaching W1;
- announces mission safety monitoring when the controller starts; and
- repeats an active safety-stop message every ten seconds.

Al's preferred browser speech voice is **Microsoft Clara Online (Natural) —
English (Canada), `en-CA`**.

The safe sequence is Manual to approach W1, Pause before **START MISSION**,
remain in Pause through launcher preflight/controller startup, and select Auto
only after the controller is live and waiting on handheld Pause.

Voice worked for approximately the first 20 minutes and then became silent.
The pursuit log proves this was not an absence of events: after 1,200 seconds,
71 safety-stop episodes remained, including a 51.22-second radio loss at
1,208.04 seconds. Tractor01's journal showed no ZeroTier, network, process,
kernel, USB, or out-of-memory failure during the mission. The dashboard later
showed `COMPLETED`.

The remaining fault domain is the phone/browser. The current dashboard does
not persist browser heartbeat, voice-enabled state, speech start/end/error, or
page visibility. Likely causes include screen lock/background suspension,
mobile `speechSynthesis` stalling, lost audio focus, or an accidental voice
toggle. Add Wake Lock, visible voice/client heartbeat, browser-to-tractor event
logging, and a speech watchdog before relying on voice for a full-duration
mission. Voice remains advisory; the handheld and visual dashboard remain the
primary controls.

## NRF24 observations

The completed mission experienced 27 radio-loss episodes, including several
long interruptions. The operator suspects the nearby garden electric fence or
its grounding as a possible contributor. This is a hypothesis, not a finding.
Run controlled comparisons with the fence energized and de-energized while
recording route/location, ACK rate, radio state, and received-packet behavior.

Evaluate phone/Wi-Fi manual control as a deliberately slow fallback experiment.
It must remain bounded, dead-man controlled, easy to stop, and independent of
the autonomous mission start path. Do not replace the handheld until latency,
loss behavior, authentication, and stop behavior are measured in a supervised
blades-off test.

## Data preservation

Local run folders:

- `runs/20260923_105953_master_manual_initial/`
- `runs/20260923_dual_f9p_5hz_manual/`
- `runs/20260923_124933_master_manual_5hz/`
- `runs/20260923_135328_manual_stripes/`
- `runs/20260923_dual_f9p_configuration/`

The preserved set contains four field logs, two pursuit logs, the heading
configuration backup, the dual-F9P pre-5-Hz backup, and three stationary audit
reports. All eleven tractor files matched their local SHA-256 hashes. Large run
artifacts remain local and are not committed to Git by default.

## Follow-up actions

1. Build coverage maps that combine the earlier captured runs with the
   September 23 completed mission and manual stripes. Identify missed,
   duplicated, and unsafe coverage before replanning paths.
2. Produce a location-aware mission report with cross-track median/RMS/p95/max
   and overlays for RTK, heading, radio-loss, stop, and recovery events.
3. Define comparable acceptance limits for cross-track error and recovery
   behavior before approving revised coverage geometry.
4. Add guarded heading-F9P startup readback/repair and make `rtcm-server`
   tolerate late USB-device availability instead of latching a fatal startup
   state indefinitely.
5. Run controlled NRF24 interference tests around the electric fence and
   inspect radio power, grounding, antennas, and connectors.
6. Prototype supervised low-speed Wi-Fi manual control with dead-man and
   explicit stop behavior.
7. Add Wake Lock, client heartbeat, speech event/error logging, and a speech
   watchdog to the dashboard.
8. Cut and mount a permanent support board for the Cub Cadet EFI ignition
   module using the captured dimensions and photos; inspect strain relief and
   weather exposure before final wiring.
9. Add RTK-base connectivity recovery that detects restored internet/ZeroTier
   reachability and restarts or rejoins ZeroTier with rate limiting and logs.
10. Record engine-on time or hour-meter readings and repeatable before/after
    fuel level on future tests.
