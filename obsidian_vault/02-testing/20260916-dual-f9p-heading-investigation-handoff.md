# Dual-F9P heading investigation handoff — 2026-09-16

## Purpose

This document is the restart context for diagnosing intermittent heading loss
on Tractor01. Start the next session from this document instead of reconstructing
the long 2026-09-16 conversation.

The immediate next task is a stationary, clear-sky baseline. Do not run an
autonomous mission until the heading system remains reliably Fixed.

## Safety state and operating constraints

- Keep the blades disengaged.
- Use Manual only to move the tractor to the test location.
- Select Pause before running any commands.
- Do not select Auto during this investigation.
- `rtcm-server.service` was active after the last audit.
- Every audit command below restarts `rtcm-server.service`, even when the audit
  returns a failure exit code.

## Hardware and data path

- Two ArduSimple ZED-F9P receivers, both running:
  - `FWVER=HPG 1.32`
  - `PROTVER=27.31`
  - `MOD=ZED-F9P`
- Rear Base-Link receiver: `/dev/gps-base-link`
- Front Heading receiver: `/dev/gps-heading`
- Measured antenna baseline: approximately 1.06–1.10 m
- Physical moving-base link:

  ```text
  Base-Link UART1 TX -> Heading UART1 RX
  ```

- The Raspberry Pi receives position/NMEA from Base-Link USB and
  UBX-NAV-RELPOSNED heading from Heading USB.
- Base-Link USB also receives the external fixed-base RTCM stream from
  `rtcm-server.service`.

## Current receiver configuration

### Both receivers

- `CFG-RATE-MEAS=100` (nominal 10 Hz)
- `CFG-RATE-NAV=1`
- UART1 enabled at 115200 baud
- Time mode disabled

### Base-Link

- UART1 RTCM input disabled
- UART1 RTCM output enabled
- USB RTCM input enabled
- UART1 message set:
  - MSM7 1077, 1087, 1097, and 1127 enabled
  - RTCM 1230 enabled
  - RTCM 4072.0 enabled
  - RTCM 4072.1 enabled
- MSM4 1074, 1084, 1094, and 1124 disabled

### Heading

- UART1 RTCM input enabled
- UART1 RTCM output remains enabled; this was deliberately left unchanged in
  the NMEA-only experiment
- USB UBX output enabled
- UBX-NAV-RELPOSNED enabled on USB
- USB NMEA protocol disabled
- All audited standard NMEA USB message rates are zero

The Heading NMEA-only configuration was applied even though the configuration
utility missed the ACK. The subsequent independent readback and raw stream
proved that all requested values persisted.

## What happened in the field

The master mission repeatedly stopped because the Pure Pursuit controller is
fail-closed. It commands zero speed when heading becomes invalid or when the
heading carrier is not Fixed.

The mission preflight can pass during a healthy five-second window, but later
intermittent frames can still stop the mission.

The current preflight accepts a RELPOSNED stream at 4 Hz or higher. The
controller stale-data timeout is 0.5 seconds. Therefore, the observed 5 Hz raw
heading rate is adequate for the current controller; the blocking issue is
intermittent loss of a Fixed and valid heading solution, not 5 Hz by itself.

## Audit tools added

- `tractor_rpi/testing/audit_dual_f9p_moving_base_20260916.py`
  - Read-only configuration audit
  - Reads firmware, RAM/BBR/Flash configuration where available
  - Checks moving-base RTCM messages and Heading USB outputs
  - Counts raw RELPOSNED and actual NMEA sentences
  - Does not change receiver settings
- `tractor_rpi/testing/configure_heading_f9p_20260727.py`
  - Now has a guarded `--nmea-only` mode
  - `--nmea-only` changes only Heading USB NMEA protocol/message rates
  - It does not change UARTs, MSM messages, baud, rates, UBX, or RELPOSNED

Relevant Git commits on `main`:

- `39ae8dd` — initial dual-F9P moving-base audit
- `e510a0e` — firmware identification
- `a8988df` — configured and actual NMEA stream audit
- `d46c325` — complete Heading cleanup target
- `6a4787f` — isolated NMEA-only mode

Current development revision at handoff: `6a4787f`.

## Evidence collected

### Before disabling Heading USB NMEA

A 120-second raw audit, with `rtcm-server.service` stopped, measured:

- 599 RELPOSNED frames
- 4.991 Hz
- 597 Fixed-valid frames
- 2 invalid frames
- 99.666% Fixed-valid
- Invalid frames contained:
  - carrier reported Fixed
  - `fixOK=true`
  - `diffSoln=true`
  - `relPosValid=false`
  - `isMoving=false`
  - `headValid=false`
  - baseline 0.0 m
  - accuracy 0.0
  - no `refPosMiss` or `refObsMiss`

A separate ten-second raw capture found that Heading USB also emitted 1,661
NMEA sentences, dominated by multi-constellation GSV and GSA messages.

### NMEA-only experiment

Only Heading USB NMEA protocol and standard NMEA USB message rates were set to
zero. Base-Link, both UARTs, MSM7, 4072.0, 4072.1, baud, rates, UBX, and
RELPOSNED were deliberately unchanged.

The following 120-second audit measured:

- 598 RELPOSNED frames
- 4.981 Hz
- zero NMEA sentences
- 502 Fixed-valid frames
- 96 frames that failed the Fixed-only safety criterion
- 83.946% Fixed-valid
- The displayed non-Fixed examples were carrier Float while `headValid`,
  relative-position validity, moving flag, and approximately 1.06 m baseline
  remained valid

Conclusion: excess NMEA output was real and is now removed, but it did not
cause the approximately 5 Hz heading rate. The worse Fixed percentage is
confounded by the tractor being parked under tree canopy.

### Latest live satellite snapshot under the trees

- Base-Link: 32 used / 48 visible, mean C/N0 37.2 dB-Hz
- Heading: 31 used / 48 visible, mean C/N0 38.0 dB-Hz
- Heading carrier Fixed
- `headValid=true`

The aggregate satellite numbers are healthy and closely matched. Tree canopy
can still cause carrier-phase multipath or cycle slips without a large decrease
in satellite count or average C/N0.

## Official ArduSimple HPG 1.32 files reviewed

Official guide:

<https://www.ardusimple.com/simplertk2heading-hookup-guide/>

Official 5 Hz files:

- Moving base:
  <https://www.ardusimple.com/wp-content/uploads/2022/09/simpleRTK2Blite_FW132_HeadingKit_MovingBase_5Hz-00.txt>
- Rover/heading:
  <https://www.ardusimple.com/wp-content/uploads/2022/09/simpleRTK2B_FW132_HeadingKit_Rover_5Hz-00.txt>

The files were decoded rather than judged only by their filenames.

Important official Moving Base settings:

- 5 Hz (`CFG-RATE-MEAS=200`)
- UART1 at 460800 baud
- MSM7 1077, 1087, and 1097 enabled
- BeiDou MSM7 1127 disabled
- RTCM 1230 and 4072.0 enabled
- 4072.1 disabled

Important official Rover settings:

- 5 Hz
- UART2 at 460800 baud
- UART1 at 115200 baud
- RELPOSNED enabled on USB and UART1
- USB NMEA messages enabled

Do **not** flash the two stock files unchanged. The ArduSimple kit's board
interconnect/port mapping differs from the tractor's custom UART1-to-UART1
wiring. Flashing the stock pair could put the correction transmitter and
receiver on different UARTs or baud rates and break the heading correction
link. The Rover file would preserve USB RELPOSNED but would also restore the
large USB NMEA stream.

The official files establish that 5 Hz is ArduSimple's standard HPG 1.32
heading-kit profile. They do not establish that 10 Hz is required for this
tractor.

## Immediate next test: clear-sky unchanged baseline

Weather prevented this test on 2026-09-16.

1. With blades disengaged, drive manually to a clear-sky location.
2. Select Pause.
3. Wait approximately one minute for LED4 to remain green.
4. Do not select Auto.
5. Run:

```bash
cd /home/al/tractor2025 && sudo systemctl stop rtcm-server.service && sudo python3 tractor_rpi/testing/audit_dual_f9p_moving_base_20260916.py --observe-seconds 120 --json-output /home/al/dual_f9p_clear_sky_msm7_20260916.json; rc=$?; sudo systemctl start rtcm-server.service; systemctl is-active rtcm-server.service; echo "Audit exit code: $rc"
```

Paste the complete output into the new chat.

### Interpretation

- If the clear-sky audit has zero or nearly zero non-Fixed/invalid frames,
  canopy multipath is the leading cause of the Float interval.
- If Float or zero-baseline invalid frames continue, the moving-base link and
  message timing remain the leading suspects.
- Record the raw RELPOSNED rate, but do not treat approximately 5 Hz alone as a
  failure. Require stable Fixed carrier, valid relative position, plausible
  baseline, and good heading accuracy.

## Possible later configuration test — not yet authorized or implemented

If the clear-sky baseline still fails, build a tractor-specific adaptation of
the official ArduSimple 5 Hz profile. Do not flash the stock files.

The adapted profile must:

- Preserve the tractor's UART1-to-UART1 physical link.
- Keep the transmitter and receiver UART1 baud rates identical.
- Preserve Base-Link USB RTCM input for fixed-base corrections.
- Preserve Heading USB UBX and UBX-NAV-RELPOSNED output.
- Keep Heading USB NMEA disabled because the Pi does not need it.
- Prefer the official HPG 1.32 moving-base message set:
  - MSM7 1077, 1087, and 1097
  - RTCM 1230
  - RTCM 4072.0
  - no 1127
  - no 4072.1
- Use the official 5 Hz rate unless a separate controlled test justifies 10 Hz.

Before applying any adapted profile:

1. Capture a complete configuration backup from both receivers.
2. List every key that will change.
3. Confirm that the Heading USB RELPOSNED output remains enabled.
4. Confirm that both ends of the physical UART1 link use the same baud.
5. Make the change recoverable over each receiver's USB connection.
6. Obtain explicit operator approval.

## Ready-to-use prompt for a new chat

```text
Continue the Tractor01 dual-F9P heading investigation using
obsidian_vault/02-testing/20260916-dual-f9p-heading-investigation-handoff.md as
the authoritative context. Read the entire document before acting.

I have manually moved the tractor to a clear-sky location, the blades are
disengaged, the handheld is in Pause, and LED4 is [green/not green]. Help me
run and interpret the unchanged 120-second clear-sky audit. Do not select or
recommend Auto until the heading system is demonstrated to be stable. Do not
flash the stock ArduSimple configuration files; preserve the tractor's
UART1-to-UART1 moving-base link and Heading USB RELPOSNED output.
```

