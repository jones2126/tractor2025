#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);  // Built-in LED
  delay(2000);  // Give serial monitor time to connect
  Serial.println("ESP32 Started - Simple Test");
}

void loop() {
  digitalWrite(2, HIGH);
  Serial.println("LED ON - Test Message");
  delay(1000);
  
  digitalWrite(2, LOW);
  Serial.println("LED OFF - Test Message");  
  delay(1000);
}