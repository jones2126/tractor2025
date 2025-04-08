#include <Arduino.h>

void setup() {
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB
    }
  
    pinMode(LED_BUILTIN, OUTPUT);
  }

void loop() {
  Serial.println("Hello from Teensy!");
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}