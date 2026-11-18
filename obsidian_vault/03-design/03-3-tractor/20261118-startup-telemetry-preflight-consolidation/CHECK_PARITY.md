# Existing production preflight parity inventory

Inspection November 18, 2026, source SHA `561d95c`. No checks implemented or changed.
Production entry is the November phone profile, with expected firmware explicitly
supplied as `teensy_main_20261110_rpi_command_lease_v5` by current callers.

## Check ownership and predicates to preserve

| Check/group | Current contributor | Existing behavior |
|---|---|---|
| Required services | August implementation, November service list | rtcm-server, tractor-rpi-arbitrator, tractor-phone-arbitrator active/running; no retired standalone bridge requirement |
| Heading startup configuration | October | Installed ExecStartPre includes configuration tool and --heading-startup; successful exit fields or current-boot readback journal markers accepted |
| Wired mission address | October | eth0 192.168.8.20/24 |
| Wired default path | October | Route to 1.1.1.1 via 192.168.8.1, eth0, source 192.168.8.20 |
| Direct TractorField Wi-Fi exclusion | October | iw succeeds and Pi SSID is not TractorField |
| GL router gateway | October | Three eth0 pings, zero reported loss |
| RTK base route | October | Route to 192.168.193.88 via zt interface, source 192.168.193.76 |
| Device presence | August | /dev/gps-base-link, /dev/gps-heading, /dev/teensy present |
| RTCM stream | August | Recent service journal readable, no configured bad-pattern lines, latest forwarding report bytes/rate positive |
| GPS feed | August | UDP 6009 snapshot arrival rate >=15 Hz; no packets fails |
| GPS device health | August | Latest fatal_error not set |
| RELPOSNED stream | August | At least two finite counter observations; advancing at >=4 Hz |
| Heading USB NMEA | August | Counter available; no counter increase in sample window; cumulative count shown diagnostically |
| RELPOSNED flags | August | Each of fixOK, differential solution, relPosValid, isMoving and both reference-not-missing flags true in >=90% of snapshots; normalized fraction diagnostic only |
| RTK position | August | Latest fix_quality exactly RTK Fixed |
| Correction age | August | Finite values exist; median last 20 finite observations <= configured maximum, default 5 s |
| Ground speed stream | August | Finite speed in >=90% snapshots, at least two finite source-counter observations, counter rate >=4 Hz |
| Stationary speed | August | Median last 20 finite speed observations in 0..0.20 m/s by default |
| Heading output | August | Finite heading values exist; headValid true in >=90% snapshots |
| Heading carrier | August | Fixed in >=90% snapshots |
| Heading baseline | August | Median last 20 finite values within 0.80..1.30 m by default |
| Heading accuracy | August | Median last 20 finite values <=1.0 degree by default |
| Heading satellites used | October | >=90% valid count coverage; median last 20 counts >25 PASS; 24–25 WARN but passing; <=23/missing FAIL |
| Steering telemetry | August | Distinct positive steering sequences; >=2 required, rate >=18 Hz |
| Transmission fields | August | >=90% records contain finite target, current, actual_target, scaled_feedback, duty_cycle_target, duty_cycle, errors_halting, jrk_sequence, jrk_valid, jrk_read_latency_ms, jrk_timeouts, cmd_vel_mps |
| JRK diagnostic rate | August | >=2 finite sequence observations, advancing >=2 Hz |
| JRK diagnostic validity | August | Valid flag in >=90% records; timeout counter does not increase |
| JRK errors | August | Halting error fields present in >=90%; every observed value zero |
| Safe starting mode | August | Latest steering/transmission telemetry modes both 2, state PAUSE, finite zero steering PWM, requested/actual JRK targets convert to configured neutral integer (2836 default) |
| Firmware identity | August, caller argument | Exact latest system firmware string match when --expected-firmware supplied |
| Wi-Fi E-stop relay | August, legacy condition | Only expected firmware teensy_main_20261003_wifi_v3: wifi_control age <=1 s and latch zero; not selected for commissioned v5 |
| JRK current validity | August, legacy condition | Only enumerated older firmware identities: >=90% valid finite current values and peak sample exists; v5 is not in that enumerated set |

## Do not silently claim checks that do not exist

The present preflight is not the complete RPi safety supervisor. It does not
explicitly check every v5 lease/watchdog/steering-fault field, and does not have
a separate JRK VIN acceptance threshold. It records transmission fields named
`current` but that is not by itself a motor-current or battery-voltage gate.
Steering's current preflight health coverage is stream rate, zero Pause output
and joined diagnostic checks. Independent arbitrator/Teensy enforcement remains
in place. Any extra steering/lease/VIN checks need separate review and fixtures,
not an undocumented addition during parity consolidation.

Startup proof currently accepts matching markers anywhere in the current boot
when explicit successful ExecStartPre fields are unavailable; it does not bind
those journal markers to a service invocation ID. Preserve this in the parity
step; propose invocation-bound proof as a separately tested improvement, especially
after recovery restarts. Similarly, do not label the recent journal forwarding
report a receiver-side RTCM CRC/use test.

The centered-input preflight adds raw-input checks for a historical profile.
It is not in the current November production chain and should remain historical
unless explicitly migrated. The Andon hardware check is intentionally excluded.

## Command-line and result compatibility

| Argument | Current default |
|---|---|
| --sample-seconds | 5.0 |
| --heading-wait-seconds | 60.0 |
| --journal-seconds | 30 |
| --diagnostic-journal-seconds | 180 |
| --max-diff-age | 5.0 |
| --max-heading-error | 1.0 |
| --min-baseline | 0.8 |
| --max-baseline | 1.3 |
| --max-stationary-speed | 0.20 |
| --neutral-jrk-target | 2836 |
| --expected-firmware | Optional; production callers explicitly supply v5 |

Retain current argument validation and argparse help/error semantics. Preserve
zero on every applicable check passing (including passing WARN), one on completed
check failure and argparse error status two. Preserve final success marker exactly
`MISSION PREFLIGHT PASS`; failure marker and detailed diagnostics remain available.
Startup wrapper separately uses exit 75 for temporary network/service readiness,
1 for completed failure and 2 for invalid wrapper configuration; keep these
distinctions when consolidating startup behavior.

GPS collection retries sample windows until GPS checks pass, a fatal receiver
error appears or the heading-wait deadline ends. It does not require heading to
be Fixed for the entire wait interval. Steering is sampled after GPS collection;
transport migration must preserve actual per-window timing and rate denominators,
not fabricate simultaneous samples from different intervals. Preserve duplicate
sequence handling and conditional diagnostic output after GPS/heading failure.

GPS failure diagnostics include receiver device mappings, fix distribution,
satellites used/visible, signal averages, HDOP, correction-age presence/range,
Heading validity/carrier distribution, RELPOSNED rate and relevant recent journals.
They perform no configuration. Preserve the information while proposing more
precise convergence wording separately; absence of NMEA does not prove the entire
Heading configuration. An accuracy estimate cannot override Float/baseline failures.

## Offline parity verification plan

Keep old tests unchanged. Build deterministic recorded/synthetic fixtures for
all checks above: healthy v5, warning satellites, absent/stale source observations,
repeated counters, rates below boundary, each GPS/flag failure, neutral mismatch,
JRK missing/errors/timeouts, firmware mismatch, relevant legacy conditional checks,
network failures, journal proof fallback, completed failures, CLI errors and exits.
Evaluate old and new logic in isolated processes to avoid monkey-patch import
order contamination. Compare check names, applicability, PASS/WARN/FAIL and exit
status before any wording changes. Verify startup/manual/dashboard call the same
new implementation. Then test the recorder transport with concurrent sampling,
finite bounded windows, restart IDs, gaps, no packet stealing and no record loss.

New encoder/IMU/recorder readiness gates and optional recovery are tested as
separate additions after parity. Unit tests require no hardware, service changes,
network access to Tractor01 or firmware operations.
