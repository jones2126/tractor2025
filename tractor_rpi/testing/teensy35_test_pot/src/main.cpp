// Simple potentiometer test for Teensy 3.5 pin A9 (pin 23)
// Connect potentiometer:
// - Pin 1: 3.3V
// - Pin 2 (wiper): Teensy pin 23 (A9)
// - Pin 3: GND
#include <Arduino.h>
void setup() {
  delay(60000);  // 60 seconds, Added delay to wait for the Rpi to ensure serial comms is ready  
  Serial.begin(115200);
  while (!Serial && millis() < 5000);  // Wait up to 5 seconds for serial
  
  Serial.println("=================================");
  Serial.println("Potentiometer Test - Pin A9");
  Serial.println("=================================");
  Serial.println("Move the pot and watch the values");
  Serial.println("Expected range: 0-1023");
  Serial.println("Expected voltage: 0.00V - 3.30V");
  Serial.println("=================================");
}

void loop() {
  // Read the potentiometer
  int potValue = analogRead(A9);
  
  // Convert to voltage (assuming 3.3V reference)
  float voltage = (potValue * 3.3) / 1023.0;
  
  // Print the results
  Serial.print("Pot Value: ");
  Serial.print(potValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 2);
  Serial.println("V");
  
  // Wait 500ms before next reading
  delay(500);
}