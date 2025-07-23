#include <HardwareSerial.h>

HardwareSerial RenogySerial(2);  // UART2 on GPIO16 (RX2), GPIO17 (TX2)

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("RS232 loopback test ready.");
  
  // Begin Renogy UART2
  RenogySerial.begin(9600, SERIAL_8N1, 16, 17);  // Check for other baud rates if needed
  Serial.println("Type characters below. They should echo back:");
}

void loop() {
  // Read from USB terminal → send to RS232
  while (Serial.available()) {
    uint8_t b = Serial.read();
    RenogySerial.write(b);
  }

  // Read from RS232 (looped) → send to USB terminal
  while (RenogySerial.available()) {
    uint8_t b = RenogySerial.read();
    Serial.write(b);  // Echo it back
  }
}
