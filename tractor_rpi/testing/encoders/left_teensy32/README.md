# Left encoder Teensy 3.1/3.2 recovery

Prepared November 18 at Al's request; exact board confirmed from yesterday's
Arduino IDE selection. Target USB serial **5879620**, currently `/dev/ttyACM4`.
Never flash the main control Teensy (serial 16883790).

PlatformIO `teensy31` is the 3.1/3.2 board target. Platform version is pinned to
the installed Windows toolchain version 5.0.0. Windows build permission was
declined; compilation and hardware upload must be validated on Tractor01.

- `left_blink`: onboard LED 500 ms on/500 ms off, one serial diagnostic per second.
  This temporarily replaces encoder reporting. It has no sensor/control outputs.
- `left_teensy32`: exact firmware appendix from the November 17 handoff, with
  `WHEEL_LEFT` changed from 0 to 1 and explicit Arduino.h for PlatformIO C++.
  No added LED behavior, distance/scale/filter/timing changes or I2C timeout fix.

Both environments specify upload_port in the ini. The custom uploader actually
checks that port's serial identity, pins its Linux USB location, refuses another
programming-mode Teensy and waits for a brief press of ONLY the left encoder's
Program button. It then requires the sole bootloader at that location and invokes
the loader for mk20dx256. It never invokes teensy_reboot or the generic -s option.
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

Confirm LED blinking and the `LEFT_ENCODER_BLINK_20261118` serial diagnostic.
Then restore encoder functionality using the same command with `-e left_teensy32`.
Require actual `ENC,L` or diagnostic `ERR,L` output before restarting the position
producer. Blink success shows the MCU/flash/LED path, not sensor/I2C health or
root-cause resolution. Preserve the raw serial capture; do not claim calibration.

Source handoff SHA256:
`5bb141fb99f9346c7e47edae5c024ad664dfd8713d110cc02577ea7d50a16e5b`.

References:
- https://docs.platformio.org/en/latest/boards/teensy/teensy31.html
- https://www.pjrc.com/teensy/loader_cli.html
