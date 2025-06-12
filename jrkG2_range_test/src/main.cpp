#include <Arduino.h>
unsigned long jrkTarget = 0;
unsigned long jrkPause = 10000;

void setTarget(uint16_t target) {
  Serial3.write(0xAA);               // Start byte
  Serial3.write(11);                 // Device number
  Serial3.write(0x40);               // Set Target command
  Serial3.write(target & 0x1F);      // 5 LSBs
  Serial3.write((target >> 5) & 0x7F); // 7 MSBs
}

void setTargetCompact(uint16_t target) {
  if (target > 4095) target = 4095;  // Clamp range
  Serial3.write(0xC0 + (target & 0x1F));       // Command byte with 5 LSBs
  Serial3.write((target >> 5) & 0x7F);         // 7 MSBs
}

void setup() {
  delay(45000);  // delay to let RPi boot up so Serial will work
  Serial.begin(115200);      // Serial monitor for debugging
  Serial3.begin(9600); // Match JRK baud rate
}

void loop() {
  jrkTarget = 2048; // Midpoint
  Serial.println("Moving to " + String(jrkTarget));
  // setTarget(jrkTarget); delay(jrkPause); // Midpoint
  setTargetCompact(jrkTarget); delay(jrkPause); // Midpoint
  jrkTarget = 4095; // Fully Extend
  Serial.println("Moving to " + String(jrkTarget));
  // setTarget(jrkTarget); delay(jrkPause);  
  setTargetCompact(jrkTarget); delay(jrkPause); 
  jrkTarget = 0; // Retract
  Serial.println("Moving to " + String(jrkTarget));
  // setTarget(jrkTarget); delay(jrkPause);
  setTargetCompact(jrkTarget); delay(jrkPause); 
}
