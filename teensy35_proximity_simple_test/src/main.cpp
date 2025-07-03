#include <Arduino.h>
int LJ18A_pin = 26;  // LJ18A sensor connected to pin 26 on Teensy 3.5
volatile byte bLED = LOW;  // Still used for sensor state tracking (optional)
volatile int ticks = 0;  // Encoder Ticks

void movement_detect() {
  bLED = !bLED;  // Toggle for sensor state (optional, not used for LED)
  ticks++;  // Increment tick count
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Onboard LED on pin 13
  pinMode(LJ18A_pin, INPUT_PULLDOWN);  // Sensor pin with internal pull-down
  attachInterrupt(LJ18A_pin, movement_detect, FALLING);  // Interrupt on falling edge
  Serial.begin(9600);  // USB serial for debugging
}

void toggleLED(unsigned long currentTime) {
  static unsigned long lastLEDToggle = 0;
  const unsigned long LEDInterval = 500;  // 1 Hz = 500 ms on, 500 ms off

  if (currentTime - lastLEDToggle >= LEDInterval) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Toggle LED state
    lastLEDToggle = currentTime;  // Reset timer
  }
}

void printTicks(unsigned long currentTime) {
  static unsigned long lastPrint = 0;
  const unsigned long printInterval = 500;  // 2 Hz = 500 ms interval

  if (currentTime - lastPrint >= printInterval) {
    Serial.println(ticks);  // Print tick count
    lastPrint = currentTime;  // Reset timer
  }
}

void loop() {
  unsigned long currentTime = millis();  // Set current time
  toggleLED(currentTime);
  printTicks(currentTime);
}