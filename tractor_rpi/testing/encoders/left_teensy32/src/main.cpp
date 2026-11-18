#include <Arduino.h>
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

#define WHEEL_LEFT 1   // 1 = left (sensor gear reversed), 0 = right

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
