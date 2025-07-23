#include <Arduino.h>
#include <ModbusMaster.h>

// RS232 Communication pins
#define RXD2 17
#define TXD2 16

// Modbus settings
#define MODBUS_ADDRESS 255  // Based on GitHub example and Bluetooth module compatibility
#define BAUD_RATE_1 9600    // Standard for Renogy
#define BAUD_RATE_2 19200   // Alternative
#define BAUD_RATE_3 38400   // Rare case

// Create Modbus instance
ModbusMaster node;

// Function prototype
void testModbusRead(uint8_t address, uint16_t reg, const char* regName, uint32_t baudRate);

void setup() {
  // Early serial output
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32 Renogy Test: Initializing...");

  // Clear Serial buffer
  while (Serial.available()) {
    Serial.read();
  }
  Serial.println("Serial buffer cleared.");

  // Test UART2 loopback
  Serial.println("Testing UART2 loopback...");
  Serial2.begin(BAUD_RATE_1, SERIAL_8N1, RXD2, TXD2);
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
  Serial.println("Serial2 initialized for Modbus...");
}

void loop() {
  // Test with different baud rates
  const uint32_t baudRates[] = {BAUD_RATE_1, BAUD_RATE_2, BAUD_RATE_3};
  const uint16_t registers[] = {0x01A, 0x0101, 0x00A};
  const char* regNames[] = {"Controller Address (0x01A)", "Battery Voltage (0x0101)", "Operating Parameters (0x00A)"};

  for (uint8_t b = 0; b < 3; b++) {
    Serial.println("Testing with baud rate: " + String(baudRates[b]));
    Serial2.begin(baudRates[b], SERIAL_8N1, RXD2, TXD2);
    node.begin(MODBUS_ADDRESS, Serial2);
    for (uint8_t r = 0; r < 3; r++) {
      testModbusRead(MODBUS_ADDRESS, registers[r], regNames[r], baudRates[b]);
      delay(250);
    }
  }

  Serial.println("---");
  delay(5000);
}

void testModbusRead(uint8_t address, uint16_t reg, const char* regName, uint32_t baudRate) {
  // Check for unexpected data
  if (Serial2.available()) {
    Serial.println("Unexpected data before Modbus request (baud " + String(baudRate) + "):");
    while (Serial2.available()) {
      Serial.print("0x");
      Serial.print(Serial2.read(), HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // Send Modbus request
  uint8_t result = node.readHoldingRegisters(reg, 1);

  if (result == node.ku8MBSuccess) {
    uint16_t raw_value = node.getResponseBuffer(0);
    if (reg == 0x0101) {
      float battery_voltage = raw_value * 0.1;
      Serial.println(String(regName) + ": " + String(battery_voltage, 1) + " V at address " + String(address));
    } else {
      Serial.println(String(regName) + ": " + String(raw_value) + " at address " + String(address));
    }
  } else {
    Serial.print("Failed to read " + String(regName) + " at address ");
    Serial.print(address);
    Serial.print(" (baud ");
    Serial.print(baudRate);
    Serial.print("). Error code: 0x");
    Serial.println(result, HEX);
    if (result == 0xE2) {
      Serial.println("Error: Modbus timeout. Check RS232 wiring, device power, or configuration.");
    } else if (result == 0xE1) {
      Serial.println("Error: Invalid response. Check address or device compatibility.");
    }
  }

  // Check for response data
  if (Serial2.available()) {
    Serial.println("Data received after Modbus request (baud " + String(baudRate) + "):");
    while (Serial2.available()) {
      Serial.print("0x");
      Serial.print(Serial2.read(), HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}