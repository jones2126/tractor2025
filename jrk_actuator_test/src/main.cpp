#include <Arduino.h>
unsigned long jrkTarget = 3500;
// Set up Serial3 for communication with the JRK controller
void setup() {
  delay(45000);  // delay to let RPi boot up so Serial will work
  Serial.begin(115200);      // Serial monitor for debugging
  Serial3.begin(9600);     // Ensure the baud rate matches the JRK configuration
  delay(1000);             // Small delay to let everything initialize
}

// Function to send the target position to the JRK controller
void setJrkTarget(uint16_t target) {
  Serial3.write(0xC0);                  // Command byte for setting target
  Serial3.write(target & 0x1F);         // Lower 5 bits of target (target & 0x1F)
  Serial3.write((target >> 5) & 0x7F);  // Upper 7 bits of target ((target >> 5) & 0x7F)
}

// Function to get the current position of the JRK actuator
uint16_t getJrkPosition() {
  Serial3.write(0xA5);                 // Command byte to request feedback from JRK
  
  // Wait for two bytes of data from the JRK
  while (Serial3.available() < 2);

  // Read the two bytes of feedback data
  uint8_t lowByte = Serial3.read();
  uint8_t highByte = Serial3.read();
  
  // Combine the two bytes into a 16-bit integer (feedback value)
  uint16_t position = (highByte << 8) | lowByte;
  
  return position;
}

void loop() {
  // Poll and print the current position before moving
  uint16_t currentPosition = getJrkPosition();
  Serial.print("Current Position: ");
  Serial.println(currentPosition);
  
  // Send command to move to position jrkTarget
  Serial.println("Moving to " + String(jrkTarget));
  setJrkTarget(jrkTarget);
  
  // Wait for 10 seconds
  delay(10000);

  // Poll and print the position after moving
  currentPosition = getJrkPosition();
  Serial.print("Current Pos after setJrkTarget(jrkTarget): ");
  Serial.println(currentPosition);
  
  // Send command to move to position 0
  Serial.println("Moving to 0");
  setJrkTarget(0);

  // Wait for another 10 seconds
  delay(10000);

  // Poll and print the position after moving to 0
  currentPosition = getJrkPosition();
  Serial.print("Current Pos after setJrkTarget(0): ");
  Serial.println(currentPosition);
}