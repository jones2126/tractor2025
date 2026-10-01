# 2026-09-30 Handheld Steering Input Diagnostic Handoff

## Purpose

Diagnose the handheld controller's steering potentiometer input without moving
the tractor steering in Manual mode. Observe and verify before changing or
flashing firmware.

## Safety state

- Tractor engine off.
- Blades off.
- Handheld mode switch remains in Pause (`mode=2` in the currently flashed
  tractor/handheld firmware).
- Do not select Manual or Auto during these diagnostics.
- The tractor steering-angle sensor has already been mechanically realigned;
  straight wheels now report approximately `525` and should not be adjusted
  again during the handheld investigation.

## Evidence collected on Tractor01

With the tractor wheels straight, UDP 6003 reported the tractor steering sensor
near `525`.

The handheld was left in Pause while the Teensy serial stream was read directly.
The operator performed this sequence:

1. Handheld steering pot at hard right before capture.
2. Wait five seconds.
3. Move to physical center and wait five seconds.
4. Move to hard left and wait five seconds.
5. Return to physical center.

Only these changing raw values were observed:

```text
handheld_raw=0      mode=2
handheld_raw=1024   mode=2
```

A second stationary test, with the handheld steering control physically
centered and still in Pause, repeatedly reported:

```text
handheld_raw=1024   mode=2
handheld_raw=1024   mode=2
```

The serial bridge restarted successfully after both tests.

## Interpretation so far

This is not a tractor steering-angle calibration problem. A healthy handheld
steering center is expected near raw `503`; hard right is near `1` and hard left
near `1024`. An exact rail-to-rail transition with physical center stuck at
`1024` points toward the handheld steering potentiometer, its mechanical
coupling, its wiper wiring, or the analog input connection.

Do not assume the pot itself has failed until its raw Teensy ADC input is
compared with another known pot and its wiring is checked.

## Checked-in handheld firmware reviewed at start

Tracked source at the start of this investigation:

`radiocontrol_nrf24radio/src/RadioControlNRF24_20260612.cpp`

Although its first source comment still names the older 20260517 firmware, Git
history shows this is the tracked 20260612 handheld source. Relevant executable
pin assignments are:

| Firmware field | Teensy analog pin | Radio payload field |
|---|---:|---|
| Steering | 16 | `steering_val` |
| Additional/throttle pot | 14 | `throttle_val` |
| Transmission pot | 15 | `transmission_val` |
| Spare fourth pot | 17 | `pot4_val` |
| Voltage input | 18 | `voltage_mv` |

The struct comments near the top contain stale pin labels for steering and
transmission, but the actual constants and the NRF documentation agree that
steering is pin 16 and transmission is pin 15.

Every 100 ms, the firmware reads each input with `analogRead()` and maps the ADC
range `0..1023` to transmitted values `0..1024`. Therefore a transmitted
steering value of exactly `1024` corresponds to a raw ADC reading at or very
near the upper rail; it is not a centering calibration offset.

The current USB serial output reports radio/ACK status but does **not** print the
four analog values. Simply attaching USB and opening a serial monitor will not
yet expose the pot readings.

## Recommended next steps

1. Keep the handheld and tractor in Pause.
2. First use the existing radio payload to compare steering pin 16 with the
   known additional pot on pin 14. Capture `s`, `t`, `x`, and mode directly from
   the tractor Teensy's serial stream. This requires no handheld firmware flash.
3. If pin 14 sweeps smoothly while steering pin 16 jumps between rails, focus on
   the steering pot, coupling, wiper lead, and pin-16 connection.
4. If both pins behave incorrectly, inspect common handheld 3.3 V/reference and
   ground wiring before blaming one pot.
5. If direct USB confirmation is still desired, prepare a deliberately separate
   diagnostic Teensy 3.2 build that prints raw ADC pins 14, 15, 16, and 17 at a
   readable rate. Do not overwrite the production source. Review and approve
   the diagnostic before flashing it.
6. Preserve an explicit, tested path for restoring
   `RadioControlNRF24_20260612.cpp` after any diagnostic flash.

## Constraints for the next chat

- Diagnose first; do not modify tractor or handheld control behavior without
  explicit approval.
- Do not select Manual or Auto merely to read the handheld pots.
- Do not alter the tractor steering sensor calibration; straight is now about
  `525`.
- Explain every connection/upload/monitor step for a hobbyist developer.
- Confirm the exact USB serial device before any upload.
- Never flash a diagnostic until the source, target board, and production
  recovery procedure have been reviewed with Al.

## Resolution

The potentiometers and their PCB signal paths were not the cause. With the
handheld powered from USB, the original Teensy 3.2 measured approximately:

- `5.0 V` on the 5 V input;
- `2.6 V` on the nominal 3.3 V rail;
- `0.9 V` from AREF to AGND; and
- about `0.7 V` on pin 14 when its raw ADC reading was already `1016`.

The pin-14 potentiometer itself swept normally at the Teensy pin: approximately
`0 V` at full clockwise, `1.6 V` at center, and `3.3 V` at full
counter-clockwise. All four ADC channels nevertheless saturated near the upper
rail. Replacing the Teensy restored normal operation, confirming a failed
Teensy analog supply/reference path rather than four failed potentiometers.

For future multi-channel ADC failures, measure the Teensy 3.3 V pin against GND
and AREF against AGND immediately after confirming the signal voltage at an
analog input. Shared rail/reference checks should precede individual
potentiometer replacement.

The firmware actually uploaded after the replacement is preserved in the
repository as:

`radiocontrol_nrf24radio/src/RadioControlNRF24_20260930.cpp`

Its active input mapping is steering pin 16, additional/throttle pin 14,
transmission pin 15, and spare pot pin 17. It explicitly selects the default
ADC reference and 10-bit resolution and prints all four raw ADC values at
0.5 Hz.
