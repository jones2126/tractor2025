#include <Arduino.h>
int LJ18A_pin = 26;  // LJ18A sensor connected to pin 26 on Teensy 3.5
volatile byte bLED = LOW;  // For sensor state tracking (optional)
volatile int ticks = 0;  // Slot count
unsigned long currentTime;  // Global time variable

void movement_detect() {
  static unsigned long lastInterrupt = 0;
  if (millis() - lastInterrupt >= 10 && digitalRead(LJ18A_pin) == LOW) {  // Count falling edge, debounce 10 ms
    bLED = !bLED;  // Optional: track sensor state
    ticks++;
    lastInterrupt = millis();
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Onboard LED on pin 13
  pinMode(LJ18A_pin, INPUT_PULLDOWN);  // Sensor pin with internal pull-down
  attachInterrupt(LJ18A_pin, movement_detect, CHANGE);  // Interrupt on both edges
  Serial.begin(9600);  // USB serial for debugging
}

void toggleLED() {
  static unsigned long lastLEDToggle = 0;
  const unsigned long LEDInterval = 500;  // 1 Hz = 500 ms on, 500 ms off

  if (currentTime - lastLEDToggle >= LEDInterval) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Toggle LED state
    lastLEDToggle = currentTime;  // Reset timer
  }
}

void printTicks() {
  static unsigned long lastPrint = 0;
  const unsigned long printInterval = 500;  // 2 Hz = 500 ms interval

  if (currentTime - lastPrint >= printInterval) {
    Serial.println(ticks);  // Print slot count
    lastPrint = currentTime;  // Reset timer
  }
}

void loop() {
  currentTime = millis();  // Update global time
  toggleLED();  // Call LED toggle function (1 Hz)
  printTicks();  // Call print ticks function (2 Hz)
}