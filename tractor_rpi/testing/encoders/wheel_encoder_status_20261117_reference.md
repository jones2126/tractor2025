# Wheel Encoder Bring-up — Status Summary (2026-11-17)

Context for a future discussion. Covers the left and right rear wheel encoders on the tractor2025 platform: what exists, what was tested, what is still unverified.

## Goal

Use wheel movement plus IMU data as a fallback when RTK GPS or dual-antenna heading degrades (see the project notes on speed-correlated RTK dropout above ~1.6 m/s). Distance/scale calibration is deliberately **deferred** until a long, near-straight run with RTK Fixed is available; until then the wheel scale numbers below are provisional.

## Status at a glance

| Item | State |
|---|---|
| Left encoder (Teensy + AS5048B) | Working with the production serial firmware (`WHEEL_LEFT 1`), streaming `ENC,L` at 10 Hz, sign flip applied |
| Right encoder (Arduino Nano Every + AS5048B) | Working, streaming `ENC,R` at 10 Hz |
| ROS dependency | Removed. Firmware prints plain CSV lines over USB serial |
| Direction (forward = positive `total_rot`) | Confirmed positive on both sides with a short forward push |
| Scale (ticks to metres) | **Not verified** — push distance was not measured |
| Left vs right difference | ~7% unexplained (see below) |
| Stable device names (udev) | **Not yet applied** — rules drafted below |
| Pi-side reader / integration | Not started |

## Hardware

| | Left rear | Right rear |
|---|---|---|
| Board | Teensy, believed 3.2 (USB bcdDevice 2.75; repo README says two Teensy 3.2 for wheel odometry) | Arduino Nano Every (genuine, ATmega4809) |
| USB ID | `16c0:0483`, serial `5879620` | `2341:0058`, serial `F5CB3C7E51514A4E43202020FF142006` |
| Sensor | AS5048B magnetic angle sensor, I2C address `0x40` (14-bit, 16384 ticks/rev) | same |
| I2C pins | 18 (SDA) / 19 (SCL) | A4 (SDA) / A5 (SCL) |
| Logic level | 3.3 V | 5 V (check the sensor wiring voltage matches) |
| Port seen on 2026-11-17 | `/dev/ttyACM5`, USB path `2-2.3` | `/dev/ttyACM6`, USB path `2-2.2` |
| Sensor gear | In reverse, so ticks are negated on the left | Normal |

Other devices on the same Pi that day: main Teensy 4.1 (serial `16883790`, `ttyACM0`), Pololu JRK G2 (`ttyACM3/4`), u-blox GNSS (`ttyACM1`).

Port numbers change with USB hub layout. The old `udev_rules` project doc matches on USB path (`KERNELS=="8-1.x"`), which no longer matches this hub layout, so `/dev/odom_left` and `/dev/odom_right` did not exist.

## Source code

- Original firmware: GitHub `jones2126/lawn_tractor_embedded_code`, folders `leftSpeed2024_v2/` and `rightSpeed2024_v2/` (rosserial-based, topics `wheel_data_left` / `wheel_data_right`). The folders could not be opened from the assistant side; the user uploaded the two `main.cpp` files directly. They are identical except for the topic name and the commented-out `delta_ticks * -1` on the right.
- Current firmware: `wheel_encoder_serial.cpp` (full source in the appendix). Single file for both wheels; set `#define WHEEL_LEFT 1` for left, `0` for right.

## What happened

1. **Port discovery:** `dmesg` showed the left Teensy appear as `ttyACM5`. The old `odom_left`/`odom_right` symlinks did not resolve (path-based rules, different hub).
2. **Left encoder failure:** the original rosserial firmware repeatedly logged `I2C transmission failed` / `I2C sensor failed - reset Teensy`. That means the sensor did not ACK address `0x40` (the failure was at `endTransmission`, never at the read). The binary garbage in the capture was rosserial framing, not corruption.
3. **Recovery:** after the Teensy was pulled and reseated on the protoboard and flashed with a bench sketch, the sensor answered at `0x40` and read cleanly. The root cause is **not conclusively identified**; a poor header/connector contact is the most likely explanation. Watch for recurrence (wiggle test: gently flex the connector and Teensy while capturing, look for `ERR` lines).
4. **Nano Every:** initially silent at 9600–460800 baud (no encoder firmware on it). The first upload attempt failed with avrdude `programmer arduino ... not in sync`, which is the classic-Nano upload path; the Nano Every needs Board = Arduino Nano Every (megaAVR core). It was later uploaded successfully and streams correctly.
5. **ROS removed:** rewrote the firmware as plain serial CSV.

## Firmware behaviour (`wheel_encoder_serial.cpp`)

Output, one line per sample at 10 Hz:

```
ENC,<side>,<ms>,<pos>,<total_rot>,<delta_ticks>,<speed_mps>,<delta_m>
WARN,<side>,<ms>,<text>      G-force filter tripped
ERR,<side>,<ms>,<text>       I2C failure (at most once per second)
# ...                        banner/comment, skip when parsing
```

- Constants: 16384 ticks/rev, wheel circumference 1.59593 m (20" diameter assumed), 10 Hz sample, I2C 100 kHz, serial 115200.
- Speed = delta ticks / 16384 x circumference / measured time between samples. A G-force filter (>2 g) substitutes the average of the last three valid speeds.
- On an I2C failure it publishes **no** `ENC` line, prints a rate-limited `ERR` line, and re-baselines on the first good read afterwards (no spurious speed spike). The original rosserial firmware reported a failed read as position 0.
- Differences from the original rosserial code: ROS removed; `WHEEL_LEFT` switch for topic/sign; return-0-on-failure fixed; shadowed `currentPosition` removed; unused `positionSum` and `sampleIndex` removed; speed uses measured `deltaTime`; clock 400 kHz to 100 kHz; `abs` to `fabsf`; float printing via `Serial.print` (AVR `snprintf` does not support `%f`).
- Opening the port resets the Nano Every (first line at `t=100`); the Teensy does not reset on open.
- Compile-checked for both `WHEEL_LEFT` values against stub headers only. Hardware behaviour confirmed by the captures below.

## Test results (2026-11-17)

- **Stationary:** both streams steady. Left position about 5300–5306, right about 6724–6729. Jitter is +/-1 to 5 ticks (about +/-0.005 m/s at 10 Hz, since 1 tick is about 0.1 mm). Not a concern at tractor speeds; a small deadband could give a clean zero at rest if wanted.
- **Short forward push, engine off, distance NOT measured** (user estimated ~3 ft):
  - Left `total_rot` change **+0.4033**, right **+0.3776** (both positive, so the current signs are correct).
  - At the current constants that implies about 0.644 m (left) and 0.603 m (right), roughly 2 ft. If the push really was 3 ft (0.914 m), the counts would be about 30% low, pointing at a sensor gear ratio below 1:1. Cannot tell from this data.
  - Left and right differ by about 7%. Possible causes: slight steering angle (the rear axle has a differential), different sensor gearing or tire size per side, or wheel slip. Unresolved.

## Open items

1. **Gear ratio check (distance-independent):** mark a tire at the ground, roll exactly one full revolution straight, capture both sides. `total_rot` should be about 1.000 per side if the sensor gear is 1:1. Measure the rolled distance too (expect about 1.596 m, 5 ft 3 in, for a 20" tire).
2. **Scale calibration** against RTK on a long straight run (user's plan; deferred). Candidates to fit: effective circumference per side, and any gear ratio.
3. **Left/right discrepancy:** repeat with steering centred and straight.
4. **udev rules** (drafted, not applied; comment out the old path-based `odom_left`/`odom_right` lines):
   ```
   SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{serial}=="5879620", SYMLINK+="odom_left"
   SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0058", ATTRS{serial}=="F5CB3C7E51514A4E43202020FF142006", SYMLINK+="odom_right"
   ```
   Then `sudo udevadm control --reload-rules && sudo udevadm trigger` and `ls -l /dev/odom_left /dev/odom_right`.
5. **Pi-side reader:** parse `ENC` lines from both ports, publish speed/odometry (for example over the existing UDP JSON path). Not written yet.
6. **Right sensor voltage:** confirm the right-wheel AS5048B supply and I2C levels suit the Nano's 5 V logic.
7. **Intermittent I2C** on the left: keep the `ERR` lines visible in any logger so a recurrence is noticed.

## Notes for the wheel + IMU fallback (design questions, nothing decided)

- Speed and distance per wheel are now available at 10 Hz. Yaw rate from the left/right difference needs the rear track width (not recorded here) and is only approximate on a tractor with a rear differential and grass slip.
- The repo also contains `BNO085_ROS_IMU_Publisher/` (an IMU sketch, not examined in this session) and the project already carries a heading source (dual F9P / ZED-X20D). Which IMU is the fallback heading source, and how the fallback is arbitrated with RTK/heading quality, still needs to be defined.
- Decide a staleness rule: with `ERR` meaning no `ENC` lines, the consumer should time out rather than hold the last speed.

## Appendix: `wheel_encoder_serial.cpp` (current, `WHEEL_LEFT 0` = right wheel build)

```cpp
// wheel_encoder_serial.cpp - AS5048B wheel encoder -> plain USB serial (no ROS)
// One source for both wheels: set WHEEL_LEFT below (1 = left, 0 = right).
//
// Output, one line per sample at 10 Hz (comma separated, newline terminated):
//   ENC,<side>,<ms>,<pos>,<total_rot>,<delta_ticks>,<speed_mps>,<delta_m>
//   WARN,<side>,<ms>,<text>        G-force filter tripped
//   ERR,<side>,<ms>,<text>         I2C failure (rate limited)
//   # ...                          comment / banner lines - skip these when parsing
#include <Wire.h>
#include <math.h>

#define WHEEL_LEFT 0   // 1 = left (sensor gear reversed), 0 = right

#define AS5048_ADDRESS 0x40
#define AS5048B_ANGLMSB_REG 0xFE

#if WHEEL_LEFT
  const char WHEEL_ID = 'L';
  const int DIR_SIGN = -1;                  // left sensor gear is in reverse
#else
  const char WHEEL_ID = 'R';
  const int DIR_SIGN = 1;
#endif

const unsigned long SAMPLE_INTERVAL = 100;  // 100 = 10Hz sampling
const int MAX_RETRIES = 3;                  // Maximum number of retries for I2C read
const unsigned long I2C_CLOCK_HZ = 100000;  // 100 kHz is plenty at 10 Hz and forgiving on long wiring
const unsigned long ERR_INTERVAL_MS = 1000; // minimum time between ERR lines
const float TICKS_PER_REVOLUTION = 16384.0; // 2^14 ticks per full rotation
const float WHEEL_CIRCUMFERENCE = 1.59593;  // Wheel circumference in meters (20" diameter)
const int VALID_SPEED_BUFFER_SIZE = 3;

unsigned long lastSampleTime = 0;
uint16_t currentPosition = 0;
uint16_t lastPosition = 0;
bool positionValid = false;                 // false until a good read, and after any failed read
uint8_t lastI2cErr = 0;                     // last I2C error code (100 = short read)
unsigned long lastErrTime = 0UL - ERR_INTERVAL_MS;
int32_t totalTicks = 0;
float speed = 0; // in meters per second

float validSpeedBuffer[VALID_SPEED_BUFFER_SIZE] = {0};
int validSpeedIndex = 0;
float lastSpeed = 0;

float calculateGForce(float currentSpeed, float lastSpeed, float deltaTime) {
  float acceleration = (currentSpeed - lastSpeed) / (deltaTime / 1000.0); // m/s^2
  return acceleration / 9.81; // Convert to G-force
}

float averageValidSpeeds() {
  float sum = 0;
  for (int i = 0; i < VALID_SPEED_BUFFER_SIZE; i++) {
    sum += validSpeedBuffer[i];
  }
  return sum / VALID_SPEED_BUFFER_SIZE;
}

// Returns true and writes the 14-bit angle through 'position' on success.
// A failed read returns false (it is never reported as a position of 0).
bool readAngle(uint16_t &position) {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    Wire.beginTransmission(AS5048_ADDRESS);
    Wire.write(AS5048B_ANGLMSB_REG);
    lastI2cErr = Wire.endTransmission(false);
    if (lastI2cErr != 0) {
      delay(5);
      continue;
    }

    if (Wire.requestFrom((uint8_t)AS5048_ADDRESS, (uint8_t)2) != 2) {
      lastI2cErr = 100;   // our own code for "short read"
      delay(5);
      continue;
    }

    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    position = (highByte << 6) | (lowByte & 0x3F);
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);                 // Teensy ignores the rate (native USB); Nano Every needs it to match the reader
  while (!Serial && millis() < 3000) {} // wait up to 3 s for the USB host
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Serial.print("# wheel encoder start, side=");
  Serial.println(WHEEL_ID);
  if (readAngle(currentPosition)) {     // only baseline on a good read
    lastPosition = currentPosition;
    positionValid = true;
  }
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastSampleTime < SAMPLE_INTERVAL) return;

  unsigned long deltaTime = currentTime - lastSampleTime;
  lastSampleTime = currentTime;

  // Failed read: report (rate limited), skip this sample, re-baseline on recovery
  if (!readAngle(currentPosition)) {
    positionValid = false;
    if (currentTime - lastErrTime >= ERR_INTERVAL_MS) {
      lastErrTime = currentTime;
      Serial.print("ERR,"); Serial.print(WHEEL_ID); Serial.print(',');
      Serial.print(currentTime); Serial.print(",i2c_fail err=");
      Serial.print(lastI2cErr); Serial.print(" addr=0x");
      Serial.println(AS5048_ADDRESS, HEX);
    }
    return;
  }
  // First good read after startup or a failure: set the baseline, output nothing this cycle
  if (!positionValid) {
    lastPosition = currentPosition;
    lastSpeed = 0;
    positionValid = true;
    return;
  }

  int16_t delta_ticks = (int16_t)currentPosition - (int16_t)lastPosition;
  delta_ticks = delta_ticks * DIR_SIGN;  // -1 on left (reversed sensor gear), +1 on right
  if (delta_ticks > 8192) delta_ticks -= 16384;
  if (delta_ticks < -8192) delta_ticks += 16384;
  totalTicks += delta_ticks;
  float totalRotations = totalTicks / TICKS_PER_REVOLUTION;

  // Speed in meters per second, from the measured time between samples
  float rotations_per_second = (float)delta_ticks / TICKS_PER_REVOLUTION * (1000.0 / deltaTime);
  speed = rotations_per_second * WHEEL_CIRCUMFERENCE;

  float delta_meters = (float)delta_ticks / TICKS_PER_REVOLUTION * WHEEL_CIRCUMFERENCE;
  float gForce = calculateGForce(speed, lastSpeed, deltaTime);

  // A valid speed is when G-force is less than 2, likely even lower
  if (fabsf(gForce) < 2.0) {
    validSpeedBuffer[validSpeedIndex] = speed;
    validSpeedIndex = (validSpeedIndex + 1) % VALID_SPEED_BUFFER_SIZE;  // circular buffer of valid speeds
  } else {
    speed = averageValidSpeeds();
    Serial.print("WARN,"); Serial.print(WHEEL_ID); Serial.print(',');
    Serial.print(currentTime); Serial.print(",gforce>2 pos=");
    Serial.print(currentPosition); Serial.print(" delta=");
    Serial.print(delta_ticks); Serial.print(" g=");
    Serial.println(gForce, 2);
  }

  Serial.print("ENC,"); Serial.print(WHEEL_ID); Serial.print(',');
  Serial.print(currentTime); Serial.print(',');
  Serial.print(currentPosition); Serial.print(',');
  Serial.print(totalRotations, 4); Serial.print(',');
  Serial.print(delta_ticks); Serial.print(',');
  Serial.print(speed, 3); Serial.print(',');
  Serial.println(delta_meters, 4);

  lastPosition = currentPosition;
  lastSpeed = speed;
}
```
