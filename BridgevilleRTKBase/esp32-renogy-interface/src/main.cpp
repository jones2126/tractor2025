#include <Arduino.h>
#include <ModbusMaster.h>

// RS232 Communication pins
#define RXD2 17
#define TXD2 16

// Modbus settings
#define MODBUS_ADDRESS 255  // Based on GitHub example and Bluetooth module
#define BAUD_RATE 9600      // Standard for Renogy

// Create Modbus instance
ModbusMaster node;

void setup() {
  // Early serial output to confirm execution
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32 Renogy Test: Starting...");
  Serial.println("Time: 10:44 PM EDT, July 22, 2025");

  // Clear Serial buffer
  while (Serial.available()) {
    Serial.read();
  }
  Serial.println("Serial buffer cleared.");

  // Test UART2 loopback
  Serial.println("Testing UART2 loopback...");
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RXD2, TXD2);
  Serial2.write("TEST", 4);
  Serial2.flush();
  delay(100);
  if (Serial2.available()) {
    Serial.println("Loopback data received:");
    while (Serial2.available()) {
      Serial.print((char)Serial2.read());
    }
    Serial.println();
  } else {
    Serial.println("No loopback data. Check UART2 wiring if testing loopback.");
  }

  // Initialize Modbus
  Serial.println("Serial2 initialized for Modbus at address " + String(MODBUS_ADDRESS));
  node.begin(MODBUS_ADDRESS, Serial2);
  Serial.println("Modbus initialized.");
}

void loop() {
  // Test Modbus read
  Serial.println("Attempting Modbus read from 0x0101...");
  uint8_t result = node.readHoldingRegisters(0x0101, 1);

  if (result == node.ku8MBSuccess) {
    uint16_t raw_value = node.getResponseBuffer(0);
    float battery_voltage = raw_value * 0.1;
    Serial.println("Battery Voltage: " + String(battery_voltage, 1) + " V at address " + String(MODBUS_ADDRESS));
  } else {
    Serial.print("Failed to read 0x0101. Error code: 0x");
    Serial.println(result, HEX);
    if (result == 0xE2) {
      Serial.println("Error: Modbus timeout. Check RS232 wiring, device power, or configuration.");
    }
  }

  // Check for raw data
  if (Serial2.available()) {
    Serial.println("Data received after Modbus request:");
    while (Serial2.available()) {
      Serial.print("0x");
      Serial.print(Serial2.read(), HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  Serial.println("---");
  delay(2000);
}