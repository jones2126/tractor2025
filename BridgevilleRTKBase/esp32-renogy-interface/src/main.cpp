#include <Arduino.h>
#include <ModbusMaster.h>

// RS232 Communication pins (matching your setup)
#define RXD2 17
#define TXD2 16

// Modbus address (try 255 as per GitHub script, fallback to 1)
#define MODBUS_ADDRESS 255

// Create Modbus instance
ModbusMaster node;

// Create a second serial interface for Modbus
HardwareSerial Serial2(2); // Use UART2

void setup() {
  delay(1000); // Wait for serial monitor to connect
  Serial.begin(115200);
  Serial.println("ESP32 Renogy Test: Starting...");

  // Clear residual data in Serial buffer
  while (Serial.available()) {
    Serial.read();
  }

  // Initialize Modbus serial
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial2 initialized for Modbus...");

  // Initialize Modbus communication
  node.begin(MODBUS_ADDRESS, Serial2);
  Serial.println("Modbus initialized with address: " + String(MODBUS_ADDRESS));
}

void loop() {
  // Read battery voltage register (0x101, single register)
  uint8_t result = node.readHoldingRegisters(0x101, 1);

  if (result == node.ku8MBSuccess) {
    uint16_t raw_value = node.getResponseBuffer(0);
    float battery_voltage = raw_value * 0.1; // Convert to volts (per GitHub script scaling)
    Serial.println("Battery Voltage: " + String(battery_voltage, 1) + " V");
  } else {
    Serial.print("Failed to read battery voltage. Error code: 0x");
    Serial.println(result, HEX);
    if (result == 0xE2) {
      Serial.println("Error: Modbus timeout. Check RS232 wiring or device power.");
    } else if (result == 0xE1) {
      Serial.println("Error: Invalid response. Check Modbus address or device compatibility.");
    }
  }

  Serial.println("---");
  delay(2000); // Wait 2 seconds before next read
}