#include <HardwareSerial.h>

HardwareSerial RenogySerial(2);  // UART2 on GPIO16 (RX2), GPIO17 (TX2)

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== USB Serial connection confirmed ===");

  RenogySerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("=== RS232 test beginning ===");
}

void loop() {
  // Keep it empty or insert your test logic here
}