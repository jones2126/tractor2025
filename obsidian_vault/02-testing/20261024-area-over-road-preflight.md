# AREA-OVER-ROAD Heading-F9P preparation and preflight — 2026-10-24

## Status

`DRAFT - UNTESTED IN THE FIELD`

The mission had not yet started when this note was recorded. This note captures
only the successful current-session Heading-F9P preparation and live preflight
for the already authorized supervised blades-off feasibility run.

## Permanent field-start order

Before every Tractor01 mission:

1. Keep the tractor stationary in Pause with blades disengaged.
2. Apply the established 5 Hz Heading-F9P moving-base profile to volatile RAM.
3. Require the independent configuration readback to pass.
4. Restart `rtcm-server.service` and require it to be active.
5. Wait for RTK position and fixed-carrier heading reacquisition.
6. Run the full mission preflight.
7. Do not start the mission unless the final result is `MISSION PREFLIGHT PASS`.

Heading configuration is mandatory before preflight; it is not merely a
recovery action after preflight fails.

## Successful preflight evidence

- Recorded from the live output at `2026-10-24T16:24:57Z`.
- Final result: `MISSION PREFLIGHT PASS`.
- `rtcm-server.service` and `teensy-bridge.service`: active/running.
- Position solution: `RTK Fixed`; median correction age 1.00 s.
- `NAV-RELPOSNED`: approximately 4.98 Hz.
- RELPOSNED fix, differential, valid, moving, and reference-present flags: 100%.
- Heading: 346.744 degrees; valid in 100% of samples.
- Heading carrier: fixed in 100% of samples.
- Heading baseline: 1.1029 m.
- Heading accuracy estimate: 0.541 degrees.
- Heading satellites used: 31.
- Starting state: Pause; steering PWM 0; JRK requested/actual neutral 2836/2836.
- Firmware: `teensy_main_20261003_wifi_v3`.
- Wi-Fi E-stop: released (`latched=0`).
- JRK diagnostics and motor-current telemetry: valid.

## Meaning

These results satisfy the current-session Heading-F9P and live preflight gates.
They do not by themselves start Pure Pursuit, authorize mowing, or establish
that the AREA-OVER-ROAD route has been field tested.
