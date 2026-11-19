# Left Teensy encoder cold-start silence — November 18

Investigation in progress. No firmware-loss conclusion or root-cause fix.
Left: confirmed Teensy 3.1/3.2, USB serial 5879620, USB location observed 2-2.3.
Main control Teensy 4.1: 16883790; must not be reset, rebound or flashed here.

## Established evidence

- Original encoder firmware became silent after Pi shutdown/startup.
- USB still enumerated as Teensyduino USB Serial, 16c0:0483, serial 5879620.
- Direct serial read after stopping the position producer returned no bytes,
  with DTR true. Opening the port again did not restore output.
- Disconnecting ONLY the left USB for five seconds and reconnecting restored
  approximately 10 Hz ENC,L messages without reflashing. Al confirmed the board
  has no separate power connection. Therefore its firmware was still present.
- A further clean Pi shutdown, ten-second power removal and startup, with USB
  cables left connected, reproduced zero left samples. Right encoder and IMU ran.

## Captured boot timeline

Source supplied by Al:
`C:/Users/al532/Downloads/20261118/left_encoder_boot_20261118.txt`.
The report was generated at 15:21:14 EST, using kernel/service monotonic timestamps.

| Seconds since boot | Observation |
|---|---|
| 4.571 | USB 2-2.3 detected |
| 4.698 | Left serial identity 5879620 reported |
| 8.122 | cdc_acm binds interface 2-2.3:1.0 as ttyACM4 |
| 28.322 | Position service begins startup hook |
| 30.007 | Heading profile independent readback passes |
| 30.050 | Position process starts |
| 31.385 | IMU 106 samples; left 0/right 9 |
| 35.401 | Explicit left-encoder silent event |
| 198.819 | Left still 0, right continuing; position process remains running |

No USB disconnect/reset/enumeration error was found in this captured boot. This
does not rule out power quality or a device-side stall; kernel enumeration is not
proof that the sketch's main loop or I2C acquisition is running. The serial device
exists roughly 22 seconds before the producer starts, so late port enumeration
does not explain this failure. A blind producer startup delay is not justified yet.

Firmware waits for host Serial for at most three seconds, then initializes Wire
and reads the AS5048B. A simple returned I2C read error should generate periodic
ERR rows. Complete silence does not identify where execution stopped. The local
Teensyduino 1.59 WireKinetis implementation already contains timeout/recovery paths;
do not claim all I2C waits are unbounded without further evidence.

## Next controlled isolation

Stop the position producer, preserve all logs and leave board power unchanged.
Verify sysfs serial 5879620 at USB 2-2.3 before unbinding/rebinding ONLY its
cdc_acm interface 2-2.3:1.0. Then discover the current serial port by identity and
repeat the read. This changes the Linux serial-driver attachment, not firmware,
and does not intentionally remove USB power. Do not touch the main control device.

If data resumes, investigate USB/CDC connection state and host interaction. If it
does not, that alone does not prove an I2C stall; continue with explicitly targeted
USB-device re-enumeration or firmware execution-stage diagnostics before choosing
a permanent remedy. Avoid automatic resets that hide the failing boot evidence.

GPS correction startup timeouts and unresolved heading are separate observations;
do not use them to explain left-encoder silence without measured linkage.

## Result: USB reauthorization restores output without reflashing

Al performed these interventions on the reproduced silent boot, with the position
producer stopped and the board left connected/powered:

1. Guarded cdc_acm unbind/rebind of interface 2-2.3:1.0: completed, but the following
   five-second exclusive serial read returned no bytes.
2. Guarded write of 0 to `/sys/bus/usb/devices/2-2.3/authorized`, two-second wait,
   then write of 1: completed. The next serial read immediately returned ENC,L
   at 100 ms intervals with no ERR lines in the supplied sample.

The first recovered device clock was **763312 ms**, increasing to 768312 ms;
it did not start near zero. That supports recovery without a whole-MCU reset.
USB reauthorization is not a deliberate VBUS power-cycle. Firmware was not
uploaded or changed. No main-controller USB interface was touched.

This distinguishes whole USB-device reconfiguration from a Linux serial-driver
reattachment, and supports investigating device/host USB configuration, endpoint
or connection state rather than treating the sketch as erased. It does not yet
identify which side has the defect or prove the I2C state was unaffected.

Potential follow-up: a bounded, identity-guarded startup recovery only if left
samples remain absent, followed by explicit successful sample validation. Such
automation is **not implemented or boot-validated**. It must not target a cached
ttyACM number, reset the main controller, repeatedly reset devices, or mask a
continued acquisition failure. Preserve pre-recovery diagnostics and report the
action. Exact wiring/firmware instrumentation or repeat controlled boots may still
be needed to establish the underlying cause.
