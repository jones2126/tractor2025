# Left encoder Teensy 3.1/3.2 recovery

Prepared November 18 at Al's request; exact board confirmed from yesterday's
Arduino IDE selection. Target USB serial **5879620**, currently `/dev/ttyACM4`.
Never flash the main control Teensy (serial 16883790).

PlatformIO `teensy31` is the 3.1/3.2 board target. Platform version is pinned to
the installed Windows toolchain version 5.0.0. Windows build permission was
declined. Al subsequently built Blink successfully on Tractor01 and uploaded it
using the verified left-board Program button. LED blinking was confirmed.

- `left_blink`: updated onboard LED double-flash: 100 ms on, 100 ms off,
  100 ms on, 1700 ms off; serial diagnostic identifies DOUBLE firmware.
  This temporarily replaces encoder reporting. It has no sensor/control outputs.
- `left_blink_auto`: same updated Blink, explicit no-button reboot/upload test.
  Requires left encoder to be the only connected PJRC/Teensy USB device.
- `left_encoder_auto`: original left encoder source, same isolated-board
  no-button upload guard; use to restore encoder functionality after Blink.
- `left_teensy32`: exact firmware appendix from the November 17 handoff, with
  `WHEEL_LEFT` changed from 0 to 1 and explicit Arduino.h for PlatformIO C++.
  No added LED behavior, distance/scale/filter/timing changes or I2C timeout fix.

All environments specify upload_port in the ini. The custom uploader actually
checks that port's serial identity, pins its Linux USB location, refuses another
programming-mode Teensy and waits for a brief press of ONLY the left encoder's
Program button. It then requires the sole bootloader at that location and invokes
the loader for mk20dx256. Button mode never invokes generic soft reboot. The
explicit `left_blink_auto` environment uses the documented loader -s/-w options
only after verifying the left encoder is the sole connected Teensy. Keep the
control 4.1 disconnected throughout this test; do not reconnect it mid-upload.
The stock PlatformIO teensy-cli path invokes generic reboot rather than selecting
the serial upload_port; simply declaring upload_port is insufficient here.

Keep tractor stopped, engine off, phone Pause. Position producer must remain stopped;
recorder, phone and bridge may remain active. No reboot or control Teensy upload.

```bash
cd /home/al/tractor2025
pio run -d tractor_rpi/testing/encoders/left_teensy32 -e left_blink -t upload
```

Wait for the verified-left-board prompt, then briefly press only its Program
button within 90 seconds. Do not press the main controller's button. No physical
USB change is needed if the configured port still matches the verified serial.
The guard refuses mismatches; inspect current ports before editing the ini.

For the no-button test, build/upload together with:

```bash
pio run -d tractor_rpi/testing/encoders/left_teensy32 -e left_blink_auto -t upload
```

Do not press Program in that test. The expected change from the first deployed
one-second blink is a short double-flash followed by a long pause. The first
manual Blink upload succeeded with firmware HEX SHA256
`dd60d29ef7329a4a61a2992979c266c5da398196267eb9925ce5dd71b5cd253a`.
Al confirmed the no-button upload succeeded and the double-flash pattern ran on
November 18, after overriding the changed Linux port from ttyACM4 to ttyACM0.
USB cable placement had not changed. Success establishes automatic upload
from this running USB-responsive Blink; it does not guarantee recovery from hung
encoder/I2C firmware without pressing Program.

Confirm double-flash and the `LEFT_ENCODER_BLINK_DOUBLE_20261118` diagnostic.
Then restore encoder functionality using the same command with `-e left_teensy32`.
Alternatively use `-e left_encoder_auto` with the 4.1 still disconnected for a
no-button restore from responsive Blink. Supply the current left-encoder port
with `--upload-port`; Linux ttyACM numbers are not persistent device identities.
Require actual `ENC,L` or diagnostic `ERR,L` output before restarting the position
producer. Blink success shows the MCU/flash/LED path, not sensor/I2C health or
root-cause resolution. Preserve the raw serial capture; do not claim calibration.

Source handoff SHA256:
`5bb141fb99f9346c7e47edae5c024ad664dfd8713d110cc02577ea7d50a16e5b`.

References:
- https://docs.platformio.org/en/latest/boards/teensy/teensy31.html
- https://www.pjrc.com/teensy/loader_cli.html
