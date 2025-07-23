// Simple ESP32 Renogy connectivity test
#include <HardwareSerial.h>

HardwareSerial RenogySerial(2);  // Use UART2 on pins 16 (RX2) & 17 (TX2)

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Renogy test - after Serial.begin...");
  
  // These pins match your wiring: (RX2 = input from controller, TX2 = output to controller)
  RenogySerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("Renogy test start: send and receive raw data...");
}

void loop() {
  // Forward any data from controller to USB serial monitor
  while (RenogySerial.available()) {
    uint8_t b = RenogySerial.read();
    Serial.printf("%02X ", b);
  }

  // (Optional) Forward any typed in via USB serial to controller
  while (Serial.available()) {
    uint8_t b = Serial.read();
    RenogySerial.write(b);
  }
}
