#include <Arduino.h>
int LJ18A_pin = 26;  // LJ18A sensor connected to pin 26 on Teensy 3.5
volatile byte bLED = LOW;
volatile int ticks = 0;  // Encoder Ticks
unsigned long currentTime = millis();

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

void toggleLED(unsigned long currentTime) {
  static unsigned long lastLEDToggle = 0;
  const unsigned long LEDInterval = 1000;  // 1 Hz = 1000 ms interval

  if (currentTime - lastLEDToggle >= LEDInterval) {
    digitalWrite(LED_BUILTIN, bLED);  // Update LED state
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
  currentTime = millis();  // Set current time
  toggleLED(currentTime);  // Call LED toggle function (1 Hz)
  printTicks(currentTime);  // Call print ticks function (2 Hz)
}