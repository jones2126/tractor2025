#include <Arduino.h>
int LJ18A_pin = 26;  // LJ18A sensor connected to pin 26 on Teensy 3.5
volatile byte bLED = LOW;
volatile int ticks = 0;  // Encoder Ticks

void movement_detect() {
  bLED = !bLED;
  ticks++;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Onboard LED on pin 13
  pinMode(LJ18A_pin, INPUT_PULLDOWN);  // Sensor pin with internal pull-down
  attachInterrupt(LJ18A_pin, movement_detect, CHANGE);  // Interrupt on pin change
  Serial.begin(9600);  // USB serial for debugging
}

void toggleLED() {
  static unsigned long lastLEDToggle = 0;
  const unsigned long LEDInterval = 1000;  // 1 Hz = 1000 ms interval

  unsigned long currentTime = millis();
  if (currentTime - lastLEDToggle >= LEDInterval) {
    digitalWrite(LED_BUILTIN, bLED);  // Update LED state
    lastLEDToggle = currentTime;  // Reset timer
  }
}

void printTicks() {
  static unsigned long lastPrint = 0;
  const unsigned long printInterval = 2000;  // 2 second interval

  unsigned long currentTime = millis();
  if (currentTime - lastPrint >= printInterval) {
    Serial.println(ticks);  // Print tick count
    lastPrint = currentTime;  // Reset timer
  }
}

void loop() {
  unsigned long currentTime = millis();  // Set current time
  toggleLED();  // Call LED toggle function (1 Hz)
  printTicks();  // Call print ticks function (2 Hz)
}