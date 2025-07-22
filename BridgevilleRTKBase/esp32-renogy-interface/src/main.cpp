#include <Arduino.h>

// RS232 Communication pins
#define RS232_RX 17
#define RS232_TX 16

// Create a hardware serial instance
HardwareSerial renogySerial(2); // Use UART2

void setup() {
  delay(1000); // Wait for serial monitor to connect
  Serial.begin(115200);
  Serial.println("ESP32 Test: Starting...");

  // Clear any residual data in the serial buffer
  while (Serial.available()) {
    Serial.read();
  }

  Serial.println("after Serial.available...");

  // Attempt to initialize RS232 communication
  renogySerial.begin(9600, SERIAL_8N1, RS232_RX, RS232_TX);

  Serial.println("renogySerial...");  
  // // Test if renogySerial is operational
  // renogySerial.write("Test"); // Attempt to write to renogySerial
  // renogySerial.flush();      // Wait for transmission to complete
  // delay(100);                // Short delay to allow any response

  // Check if renogySerial is receiving any unexpected data (indicating a wiring or device issue)
  if (renogySerial.available()) {
    Serial.println("Error: Unexpected data on renogySerial. Check RS232 wiring or device.");
    while (renogySerial.available()) {
      Serial.print((char)renogySerial.read()); // Print unexpected data for debugging
    }
    Serial.println();
  } else {
    Serial.println("renogySerial initialized successfully.");
  }
}

void loop() {
  Serial.println("ESP32 Test: Loop running...");
  delay(1000);
}