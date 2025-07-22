#include <Arduino.h>

void setup() {
  delay(1000); // Wait for serial monitor to connect
  Serial.begin(115200);
  Serial.println("ESP32 Test: Starting...");
}

void loop() {
  Serial.println("ESP32 Test: Loop running...");
  delay(1000);
}