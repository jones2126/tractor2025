#include <Arduino.h>

const int LED_PIN = 2;  // Onboard LED for most ESP32 boards

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (for native USB)
  }

  Serial.println("ESP32 Booting...");
  delay(1000);
}

void loop() {
  static bool ledState = false;

  // Toggle LED
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState ? HIGH : LOW);

  // Print to serial
  Serial.print("Heartbeat - LED is ");
  Serial.println(ledState ? "ON" : "OFF");

  delay(1000);  // Wait 1 second
}
