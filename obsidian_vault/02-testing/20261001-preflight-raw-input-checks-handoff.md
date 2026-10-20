# 2026-10-01 Pre-Flight Raw Input Checks Handoff

## Purpose of the next chat

Update the most current tractor01 pre-flight workflow so it fails closed when
the tractor steering-angle sensor or any of the four handheld potentiometers
is not near its expected centered value.

Before pre-flight is run, the operator will:

- keep the tractor stationary with the engine and blades off;
- put the handheld mode switch in **Pause**;
- point the tractor wheels nearly straight; and
- place all four handheld potentiometers at their physical middle positions.

The pre-flight should collect live raw values, show the observed values to the
operator, and compare each channel with an expected value near `500` using a
clearly documented, configurable tolerance. It must not require selecting
Manual or Auto or moving the steering during this check.

## Safety and mode convention

For the firmware currently selected and used in the September field work:

| Mode | Value |
|---|---:|
| Auto | `0` |
| Manual | `1` |
| Pause | `2` |
| Radio-loss safety | `9` |

Some older documentation still says Pause is `0`. Do not use that stale
mapping for this work. The current pre-flight already requires steering mode
`2`, transmission mode `2`, steering state `PAUSE`, zero steering PWM, and the
neutral JRK target.

Do not change tractor or handheld control behavior as part of this task. Any
tractor or handheld firmware change must be limited to adding telemetry unless
Al explicitly approves more. Review the source, target board, upload device,
and recovery path before any firmware flash.

## Handheld Teensy issue and resolution

The handheld steering input appeared to have failed. With the physical
steering control centered, the received value was stuck at the upper rail:

```text
handheld_raw=1024 mode=2
```

Moving the control produced essentially a rail-to-rail transition instead of
a smooth sweep. Later, direct USB diagnostics showed all four handheld ADC
channels near `1023`, even though the potentiometers were physically centered.

The potentiometers and their PCB signal paths were not the cause. Measurements
on the original Teensy 3.2 showed:

- 5 V pin: approximately `5.0 V`;
- nominal 3.3 V pin: approximately `2.6 V`;
- AREF to AGND: approximately `0.9 V`; and
- pin 14: approximately `0.7 V` while the Teensy reported ADC `1016`.

The pin-14 potentiometer signal itself swept normally at the Teensy pin:
approximately `0 V` at full clockwise, `1.6 V` at center, and `3.3 V` at full
counter-clockwise. Replacing the Teensy restored normal readings. This
confirmed failure of the Teensy's analog supply/reference path rather than
four simultaneous potentiometer failures.

Diagnostic lesson: when several ADC channels fail together, check the Teensy
3.3 V pin against GND and AREF against AGND early, after confirming a signal
voltage directly at an analog input.

The complete diagnostic record is:

`obsidian_vault/02-testing/20260930-handheld-steering-input-diagnostic-handoff.md`

## Current checked-in files

- Current pre-flight implementation:
  `tractor_rpi/testing/mission_preflight_20260804.py`
- Current serial bridge:
  `tractor_rpi/teensy_serial_bridge_20260728.py`
- Tractor PlatformIO currently selects:
  `tractor_teensy/src/teensy_main_20260914.cpp`
- Handheld replacement-board firmware uploaded on 2026-09-30 and synchronized
  to Git:
  `radiocontrol_nrf24radio/src/RadioControlNRF24_20260930.cpp`

The dated filenames are retained by project convention. Create appropriately
dated successors rather than silently overwriting the historical identity of
a deployed script or firmware image.

## Known raw values and expected centers

### Tractor steering-angle sensor

The tractor steering sensor is read on the tractor Teensy and is already
published by the bridge as:

```text
UDP 6003 -> steering.current
```

The sensor was mechanically realigned and must not be adjusted during this
task. The selected tractor firmware defines:

| Position | Raw value |
|---|---:|
| Calibrated right | `191` |
| Straight/center | `525` |
| Calibrated left | `885` |

Straight wheels have been observed near `525` in live UDP 6003 data.

### Handheld potentiometers

The handheld reads 10-bit ADC values `0..1023` and maps them into transmitted
values `0..1024`. A healthy centered steering value was historically near
`503`; nominal center for all four channels is approximately `500..512`.

Executable pin assignments in
`radiocontrol_nrf24radio/src/RadioControlNRF24_20260930.cpp` are:

| Radio payload field | Executable analog pin | Intended role |
|---|---:|---|
| `steering_val` | `16` | Steering |
| `throttle_val` | `14` | Additional/throttle |
| `transmission_val` | `15` | Transmission |
| `pot4_val` | `17` | Spare fourth pot |

Important: comments and the `POT_RAW` print labels near `sendData()` currently
claim pin 14 is steering and pin 16 is the former steering input. Those labels
contradict the executable constants above. Do not infer physical pin identity
from the current diagnostic text. Either correct the labels in a separately
reviewed dated firmware version or name pre-flight fields by both payload field
and verified executable pin.

There had been an expressed intention during diagnosis to move steering control
to pin 14. The uploaded source does **not** implement that switch: only the
nearby diagnostic labels describe pin 14 as steering. Treat the executable
pin-16 mapping as the current behavior. Do not turn the pre-flight work into a
control-pin remapping; resolve any desired remapping as a separate, explicitly
approved firmware change.

## Existing pre-flight behavior

`mission_preflight_20260804.py` is the only current pre-flight script found in
the repository. It already:

- samples tractor status from JSON UDP port 6003;
- deduplicates the duplicate UDP broadcasts by steering sequence;
- checks the steering stream rate;
- requires the current Pause state;
- checks zero steering PWM and neutral JRK requested/read-back targets;
- checks transmission/JRK diagnostic completeness and validity; and
- can require a specified tractor firmware identity.

It does not currently validate centered raw input values.

## Telemetry gap that must be solved

The pre-flight script alone cannot currently check all four handheld pots.

The selected tractor firmware receives all four values in the 14-byte NRF24
payload, but its `RADIO` serial record currently:

- prints `s=steering_val`, `t=throttle_val`, and `x=transmission_val`;
- does not print `pot4_val`; and
- prints the values only once every five seconds.

The five-second interval is too sparse for the pre-flight's default five-second
sample window and may yield no fresh sample, depending on timer alignment.

The serial bridge parses and caches `RADIO` key/value pairs, but UDP 6003's
`radio` object currently publishes only signal state, ACK rate, current rate,
and age. It discards `s`, `t`, and `x` from the outward JSON message.

Therefore, to make the four checks reliable from tractor01, the next chat will
probably need a reviewed telemetry-only change through this path:

```text
handheld ADC
  -> NRF24 payload
  -> tractor Teensy RADIO telemetry, including all four fields
  -> serial bridge UDP 6003 radio object
  -> pre-flight centered-input checks
```

Do not have the pre-flight open `/dev/teensy` directly while the systemd serial
bridge owns it. Use the bridge's UDP output so there remains one serial reader.

## Requested pre-flight checks

Add fail-closed checks for these five centered inputs:

1. Tractor steering-angle sensor, using `steering.current` and a center near
   `525`.
2. Handheld `steering_val` from verified pin 16.
3. Handheld `throttle_val` from verified pin 14.
4. Handheld `transmission_val` from verified pin 15.
5. Handheld `pot4_val` from verified pin 17.

For each value, the output should show at least:

- payload/role name and verified pin where applicable;
- sample count;
- latest value;
- median value;
- observed minimum and maximum;
- expected center and accepted range; and
- PASS or FAIL.

Use medians over fresh samples rather than trusting one reading. Reject an
absent, nonnumeric, stale, or inadequately sampled field. A value at or close
to `0` or `1024` while centered must fail and should be described as a possible
rail/reference/wiring fault.

Make the center tolerance a command-line option and print it in the report. A
reasonable initial proposal for discussion is target `500` with tolerance
`100` for the four handheld pots, while retaining the more accurate tractor
target of `525`. Confirm the final default from healthy centered field samples
rather than tightening it arbitrarily. The script must continue to return a
nonzero exit status if any required check fails.

The checks must occur only after Pause is confirmed. If the mode or Pause state
is wrong, report that first and do not suggest moving a potentiometer or
selecting another mode.

## Suggested implementation sequence for the next chat

1. Re-read the files named above and confirm the Git worktree is clean.
2. Inspect a live UDP 6003 packet on tractor01 before editing anything.
3. Define unambiguous field names for the four handheld values. Avoid the
   ambiguous `p1`/`p2` labels unless their pins are also included.
4. Prepare and review a dated, telemetry-only tractor firmware successor that
   emits all four received pot values often enough for pre-flight. Do not flash
   it yet.
5. Prepare a dated serial-bridge successor that forwards the four fields into
   UDP 6003 while preserving existing keys and consumers. Review deployment and
   rollback before changing the systemd service.
6. Prepare a dated pre-flight successor with configurable centered-input checks
   and focused unit tests using synthetic UDP messages.
7. Verify Python syntax and tests locally.
8. Review the exact tractor firmware target, `/dev/teensy` identity, upload
   command, bridge deployment, service restart, and rollback procedure with Al.
9. Only after explicit approval, deploy telemetry changes and run the pre-flight
   with handheld Pause, wheels nearly straight, and all four pots centered.
10. Record the five healthy center distributions and adjust the tolerance only
    from that evidence.

## Acceptance criteria

- No Manual or Auto selection is needed to observe the values.
- No steering or transmission motion is commanded by the new checks.
- UDP 6003 exposes fresh, clearly named values for all four handheld pots.
- Pre-flight verifies the tractor steering sensor and all four handheld pots.
- Every check reports useful raw statistics and its accepted range.
- Missing, stale, rail-saturated, or out-of-range values fail closed.
- Existing pre-flight safety, GPS, heading, radio, steering, JRK, and firmware
  checks remain intact.
- The new scripts/firmware retain dated filenames and have documented rollback
  paths.

## Repository state at handoff

The repository was clean and synchronized after commit `7a21862` (`Add field
artifact retention policy and site records`) before this handoff document was
created. This new document should be reviewed and committed before starting the
separate implementation chat if it is to serve as the shared GitHub handoff.
