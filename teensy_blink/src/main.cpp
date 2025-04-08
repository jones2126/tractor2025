#include <Arduino.h>

void setup() {
  pinMode(13, OUTPUT);  // Onboard LED
}

void loop() {
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}