#include <Arduino.h>
#include <ModbusMaster.h>

// RS232 Communication pins
#define RXD2 17
#define TXD2 16

// Modbus addresses to test
#define MODBUS_ADDRESS_1 255
#define MODBUS_ADDRESS_2 1

// Create Modbus instance
ModbusMaster node;

// Function prototype
void testModbusRead(uint8_t address);

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
}

void loop() {
  // Test with Modbus address 255
  Serial.println("Testing with Modbus address: " + String(MODBUS_ADDRESS_1));
  node.begin(MODBUS_ADDRESS_1, Serial2);
  testModbusRead(MODBUS_ADDRESS_1);

  delay(2000); // Wait before trying next address

  // Test with Modbus address 1
  Serial.println("Testing with Modbus address: " + String(MODBUS_ADDRESS_2));
  node.begin(MODBUS_ADDRESS_2, Serial2);
  testModbusRead(MODBUS_ADDRESS_2);

  Serial.println("---");
  delay(2000); // Wait before next loop
}

void testModbusRead(uint8_t address) {
  // Send a test write to wake up the device
  Serial2.write("TEST", 4);
  Serial2.flush();
  delay(100);

  // Check for any unexpected data before sending Modbus request
  if (Serial2.available()) {
    Serial.println("Unexpected data received before Modbus request:");
    while (Serial2.available()) {
      Serial.print("0x");
      Serial.print(Serial2.read(), HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // Read battery voltage register (0x101, single register)
  uint8_t result = node.readHoldingRegisters(0x101, 1);

  if (result == node.ku8MBSuccess) {
    uint16_t raw_value = node.getResponseBuffer(0);
    float battery_voltage = raw_value * 0.1; // Convert to volts
    Serial.println("Battery Voltage: " + String(battery_voltage, 1) + " V");
  } else {
    Serial.print("Failed to read battery voltage. Error code: 0x");
    Serial.println(result, HEX);
    if (result == 0xE2) {
      Serial.println("Error: Modbus timeout. Check RS232 wiring, device power, or Modbus address.");
    } else if (result == 0xE1) {
      Serial.println("Error: Invalid response. Check Modbus address or device compatibility.");
    }

    // Print any received data for debugging
    if (Serial2.available()) {
      Serial.println("Received data after failed request:");
      while (Serial2.available()) {
        Serial.print("0x");
        Serial.print(Serial2.read(), HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
}