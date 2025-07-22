#include <Arduino.h>

// Common LED pins on different ESP32 boards
int ledPins[] = {2, 5, 16, 17, 18, 19, 21, 22, 23};
int numPins = sizeof(ledPins) / sizeof(ledPins[0]);

void setup() {
  Serial.begin(115200);
  delay(2000);  // Give serial monitor time to connect
  Serial.println("ESP32 LED Pin Test Started");
  Serial.println("=========================");
  Serial.println("Testing LED on different pins...");
  
  // Initialize all potential LED pins
  for (int i = 0; i < numPins; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);  // Start with LEDs off
  }
}

void loop() {
  // Test each pin one by one
  for (int i = 0; i < numPins; i++) {
    Serial.print("Testing LED on GPIO ");
    Serial.println(ledPins[i]);
    
    // Turn on LED
    digitalWrite(ledPins[i], HIGH);
    delay(1000);
    
    // Turn off LED  
    digitalWrite(ledPins[i], LOW);
    delay(500);
  }
  
  Serial.println("------------------------");
  Serial.println("Cycle complete. Watch for blinking LED!");
  delay(2000);
}