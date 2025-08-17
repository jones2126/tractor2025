/*
  4-Port Relay Test for Teensy 4.1
  Tests relay connections on pins 29-32
  Each relay cycles on for 2 seconds, then off for 1 second
*/
#include <Arduino.h>
// Define relay pins
const int relayPins[] = {29, 30, 31, 32};
const int numRelays = 4;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    // Wait for serial connection or timeout after 3 seconds
  }
  
  // Initialize all relay pins as outputs and turn them off
  for (int i = 0; i < numRelays; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);  // Turn relay OFF
  }
  
  Serial.println("Teensy 4.1 - 4-Port Relay Test Starting...");
  Serial.println("Each relay will cycle ON for 2 seconds, then OFF for 1 second");
  Serial.println();
  
  delay(1000);  // Initial delay
}

void loop() {
  static int cycle = 1;
  
  Serial.print("--- Cycle ");
  Serial.print(cycle);
  Serial.println(" ---");
  
  // Test each relay individually
  for (int i = 0; i < numRelays; i++) {
    Serial.print("Activating Relay ");
    Serial.print(i + 1);
    Serial.print(" (Pin ");
    Serial.print(relayPins[i]);
    Serial.println(")...");
    
    digitalWrite(relayPins[i], HIGH);  // Turn relay ON
    delay(2000);  // Keep on for 2 seconds
    
    digitalWrite(relayPins[i], LOW);   // Turn relay OFF
    Serial.print("Relay ");
    Serial.print(i + 1);
    Serial.println(" OFF");
    
    delay(1000);  // Pause between relays
  }
  
  Serial.println("All relays tested. Starting next cycle in 3 seconds...");
  Serial.println();
  delay(3000);  // Pause before next cycle
  
  cycle++;
}

// Optional: Function to turn off all relays (can be called from anywhere)
void allRelaysOff() {
  for (int i = 0; i < numRelays; i++) {
    digitalWrite(relayPins[i], LOW);
  }
  Serial.println("All relays turned OFF");
}