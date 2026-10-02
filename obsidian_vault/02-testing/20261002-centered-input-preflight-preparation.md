# 2026-10-02 centered-input preflight preparation

## Status

Prepared and locally verified. A field deployment attempt on 2026-10-02 was
rolled back and the feature is **not deployed**.

The telemetry firmware compiled and PlatformIO reported a successful upload,
but the tractor Teensy remained in HalfKay bootloader mode instead of returning
as USB serial `16c0:0483`. The first rollback write also failed. After a short
press of the Teensy Program button, the known-good firmware upload succeeded,
`/dev/teensy` returned, firmware identity `teensy_main_20260926` was observed on
UDP 6003, and the original `teensy_serial_bridge_20260728.py` service override
was restored. Do not retry the telemetry firmware on Tractor01 until its boot
behavior and the contemporaneous Raspberry Pi undervoltage/USB events have
been investigated away from a field mission.

## Added telemetry path

1. `tractor_teensy/src/teensy_main_20261002.cpp` inherits the deployed
   `teensy_main_20260926` control behavior and adds only a 4 Hz
   `RADIO_INPUTS` serial record.
2. `tractor_rpi/teensy_serial_bridge_20261002.py` forwards those four received
   handheld values into UDP 6003.
3. `tractor_rpi/testing/mission_preflight_20261002_centered_inputs.py` retains
   the 2026-10-02 Heading-satellite gate and adds centered-input checks.

## Center policy

- Tractor steering sensor: target 525, tolerance 50 (accepted 475-575).
- Each handheld input: target 500, tolerance 100 (accepted 400-600).
- Handheld fields: steering/pin 16, additional/pin 14,
  transmission/pin 15, and pot4/pin 17.
- All values use medians over fresh UDP samples.
- Missing, stale, radio-invalid, out-of-range, or non-Pause data fails closed.

## Verification completed

- Python compilation succeeded.
- 29 Python tests passed.
- The Teensy 4.1 firmware compiled successfully with PlatformIO.
- The attempted field upload was fully rolled back as described above.

## Deployment boundary

Deployment requires a new bench review: confirm the tractor Teensy USB
identity, preserve a rollback build, flash the telemetry-only firmware, point
`teensy-bridge.service` at the dated bridge successor, restart the bridge, and
inspect a live UDP 6003 packet before using the new preflight. Until then, keep
the approved mission launcher on `mission_preflight_20261002.py`.

After deployment, the expected firmware identity will be
`teensy_main_20261002` and the new command will be:

```bash
sudo python3 tractor_rpi/testing/mission_preflight_20261002_centered_inputs.py \
  --expected-firmware teensy_main_20261002
```
