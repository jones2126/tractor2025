#include <Arduino.h>

unsigned long jrkTarget = 0;
unsigned long jrkPause = 5000;

void setTargetCompact(uint16_t target) {
  if (target > 4095) target = 4095;
  Serial3.write(0xC0 + (target & 0x1F));
  Serial3.write((target >> 5) & 0x7F);
}

uint16_t readFeedback() {
  Serial3.write(0xE5);  // Command: Get variables
  Serial3.write(0x04);  // Offset for Feedback
  Serial3.write(0x02);  // Length = 2 bytes

  unsigned long start = millis();
  while (Serial3.available() < 2) {
    if (millis() - start > 100) {
      Serial.println("Timeout waiting for feedback");
      return 0xFFFF;  // Error code
    }
  }

  uint8_t low = Serial3.read();
  uint8_t high = Serial3.read();
  return (high << 8) | low;
}

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);  // Match JRK baud rate
  delay(5000);          // Optional startup delay
  Serial.println("Starting JRK test with feedback");
}

void loop() {
  uint16_t positions[] = {2048, 4095, 0};
  for (int i = 0; i < 3; i++) {
    jrkTarget = positions[i];
    setTargetCompact(jrkTarget);
    Serial.print("Sent Target: ");
    Serial.println(jrkTarget);

    delay(jrkPause);  // Wait for actuator to reach position

    uint16_t feedback = readFeedback();
    if (feedback != 0xFFFF) {
      Serial.print("Feedback: ");
      Serial.println(feedback);
    }

    delay(1000); // Small pause before next command
  }
}
