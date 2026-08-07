# Tractor01 JRK repair and inner-stripes retest context

Use this document as context for a new Codex chat after the transmission actuator wiring, Teensy mounting, and associated harness work are complete.

## Current status

The August 5 testing isolated the unexpected transmission motion to the JRK feedback system. Al subsequently found a physically loose wire affecting the JRK controller. Components are being removed, re-soldered, and remounted more securely before any autonomous mission is attempted again.

This is now a confirmed hardware repair, not a Pure Pursuit tuning problem. Do not compensate for it with a large JRK feedback dead zone or software filtering until the repaired feedback signal has been tested.

The tractor should be treated as **not ready for Auto** while wiring or mounting work remains incomplete.

## Mission to be retested

Repository-relative directory:

```text
field_testing/sites/62_Collins_polygon_1/spiral_missions/20260804_inner_stripes
```

Mission: `polygon_1_inner_stripes_38in_20260804.txt`

- 20 east/west strips; 1,010 waypoints
- 0.9652 m (38-inch) lane spacing; 472.2 m route
- 0.85 m/s straight and 0.50 m/s keyhole/transit speeds
- 1.90 m modeled keyhole radius
- 1.00 m Pure Pursuit lookahead throughout
- Start: `40.485562833, -80.332340333`, heading about `162 degrees` compass
- Expected SHA-256: `b9c9ff5b18be1cba7f725b63ba17f561c72769409ee59e2e72fd144b0181c028`

The detailed geometric baseline and acceptance criteria remain in `LOOKAHEAD_1M_RETEST_CONTEXT_20260805.md`.

## August 5 findings

The first 1 m-lookahead inner-stripes start was aborted after only a few seconds because speed control was not apparent and the tractor was unexpectedly jerky.

The investigation showed:

- RTK Fixed and `headValid` could remain good during supervised driving.
- A slightly loose heading-antenna coax connection was found and tightened.
- Straight 15 m and 20 m tests completed at 0.50 m/s with RTK Fixed.
- Starting the mower produced large JRK feedback excursions while requested and actual JRK targets remained fixed.
- Stopping the mower returned JRK feedback to a narrow, stable range.

Stationary test directory:

```text
field_testing/sites/62_Collins_polygon_1/runs/20260805_stationary_mower_off_on_after_coax
```

Key files are `stationary_mower_off_on_120s_after_coax_tighten_20260805.csv`, `stationary_mower_off_jrk_recovery_20260805.csv`, and `jrk_mower_off_on_feedback_duty_20260805.png`.

Observed behavior:

- Requested and actual JRK target: approximately `2836`, steady.
- Initial mower-off scaled feedback range: approximately `2759-2912`.
- Mower-on scaled feedback range: approximately `2727-3326`.
- Ten large mower-on feedback excursions were observed.
- Each sufficiently large false feedback error produced applied duty `-600`, apparently the configured duty limit.
- After mower shutdown/recovery, feedback was `2831-2844` with no large excursions.
- The mower clutch is mechanical, so engagement has no electric clutch-coil transient.

Stable requested and read-back targets ruled against corrupted serial targets as the primary cause. The feedback excursions caused the JRK closed loop to move the actuator. The later discovery of a loose wire confirms a feedback connection fault consistent with vibration sensitivity.

## Relevant run directories

```text
field_testing/sites/62_Collins_polygon_1/runs/20260805_inner_stripes_1m_aborted_start
field_testing/sites/62_Collins_polygon_1/runs/20260805_headvalid_straight_15m
field_testing/sites/62_Collins_polygon_1/runs/20260805_headvalid_straight_20m_mower_off
field_testing/sites/62_Collins_polygon_1/runs/20260805_stationary_engine_gps_test
field_testing/sites/62_Collins_polygon_1/runs/20260805_manual_drive_after_heading_coax_tighten
field_testing/sites/62_Collins_polygon_1/runs/20260805_stationary_mower_off_on_after_coax
```

## Active Teensy firmware and repair pins

PlatformIO builds only `tractor_teensy/src/teensy_main_20260804.cpp`. See `tractor_teensy/TEENSY41_PIN_ASSIGNMENT_AUDIT_20260807.md` for the complete audit.

| Teensy pin | Active use |
|---|---|
| 5 | IBT-2 steering RPWM output |
| 6 | IBT-2 steering LPWM output |
| 9 | NRF24 CE |
| 10 | NRF24 CSN |
| 11 | NRF24 SPI MOSI |
| 12 | NRF24 SPI MISO |
| 13 | NRF24 SPI SCK |
| 14 | Serial3 TX to JRK serial RX |
| 15 | Serial3 RX from JRK serial TX |
| 23 / A9 | Steering potentiometer analog input |
| 30 | E-stop relay output, active LOW |

Pin 32 in old documentation is not the active E-stop pin. Pins 7, 8, 21/A7, and 22/A8 are planned IBT-2 Gen2 connections but are not used by active firmware.

## Before applying power

With battery and USB power removed:

1. Inspect and continuity-test every repaired joint.
2. Tug-test every crimp and conductor individually.
3. Confirm the intended common reference between Teensy ground, JRK logic ground, and JRK feedback ground. Previous damage occurred when Teensy and JRK grounds were not properly tied.
4. Check for solder bridges, loose strands, and conductive mounting hardware.
5. Add strain relief so frame vibration is not carried by solder joints.
6. Verify JRK reference, feedback wiper, and feedback ground by function, not wire color alone.
7. Cross Serial3 correctly: Teensy TX3 pin 14 to JRK RX; JRK TX to Teensy RX3 pin 15; share logic ground.
8. Verify the E-stop relay input is pin 30, not pin 32.
9. Keep the steering pot on A9/pin 23 and within the Teensy's 3.3 V input range.

## Staged recommissioning

Keep the transmission mechanically disengaged, wheels chocked, blades off, and linkage clear for the first tests.

### 1. Power and safe state

- Start with the handheld UP in Pause.
- Confirm mode `2`, steering PWM zero, requested/actual JRK target `2836`, valid JRK reads, and no accumulating timeouts/discarded bytes.
- Exercise the physical E-stop before driving.

### 2. Stationary JRK test, mower off

- Log at least 60 seconds at fixed neutral.
- Confirm target, actual target, feedback, duty target, and duty are stable.
- Gently manipulate one harness section at a time while watching feedback. Stop if the actuator moves.
- Any feedback jump with a steady target is a failure.

### 3. Stationary vibration test

- Record at least 60 seconds mower off, 60 seconds mower on, and 60 seconds mower off in one continuous log.
- Do not touch moving or vibrating linkage.
- Pass only if engagement creates no large feedback excursion, duty burst, or actuator twitch.
- Compare directly with the August 5 PNG and CSV.

### 4. Manual actuator test

- With transmission mechanically disengaged and mower off, exercise small forward/reverse commands.
- Confirm target, read-back target, feedback direction, and actuator direction agree.
- Return to Pause and verify repeatable neutral without twitching.

### 5. Short straight missions

- Re-engage transmission only after stages 1-4 pass.
- Repeat a short 0.50 m/s straight run mower off, then mower on, with expanded JRK logging.
- Do not run inner stripes until both pass.

### 6. Inner-stripes mission

After preflight and short tests pass:

```bash
cd /home/al/tractor2025/field_testing/sites/62_Collins_polygon_1/spiral_missions/20260804_inner_stripes
bash ./run_polygon_1_inner_stripes_20260804.sh
```

Stay ready to select Pause. Stop for unexplained actuator motion, feedback jumps with steady target, heading invalidity, loss of RTK Fixed, steering oscillation, or unexpected path behavior.

## Software notes

- The startup comment calling `Serial3.write(0x83)` "Exit safe start" is incorrect for the JRK G2 compact protocol; it requests a target variable byte. Correct it in a separate controlled firmware change after hardware validation.
- USB serial is 460800 baud; JRK Serial3 is 9600 baud.
- JRK diagnostics run at 5 Hz; steering control/telemetry at 20 Hz.
- Modes: DOWN/Auto=`0`, MIDDLE/Manual=`1`, UP/Pause=`2`, no signal=`9`.
- `tractor_teensy_main_documentation.md` is obsolete Teensy 3.5-era material and is not a wiring authority.

## Acceptance criteria

- No JRK feedback excursions during mower off/on/off testing.
- No actuator motion without a corresponding target change.
- Requested and actual targets agree.
- Feedback changes smoothly in the expected direction when commanded.
- Applied duty settles to zero.
- No accumulating JRK timeouts or discarded bytes.
- E-stop, Pause, Manual, and radio-loss safety work physically.
- RTK Fixed and valid heading during short drive tests.

If excursions remain, isolate the actuator's internal potentiometer/wiper, integral cable, JRK 5 V/reference and ground, and JRK FB input before changing controller parameters.
