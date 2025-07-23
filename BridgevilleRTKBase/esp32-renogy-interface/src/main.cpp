#include <Arduino.h>
#include <ModbusMaster.h>

// RS232 Communication pins
#define RXD2 17
#define TXD2 16

// Default Modbus addresses to try for reading controller address
#define DEFAULT_ADDRESS_1 1   // From Node.js example
#define DEFAULT_ADDRESS_2 255 // From GitHub Arduino example

// Create Modbus instance
ModbusMaster node;

// Function prototype
void testModbusRead(uint8_t address, bool& found, uint8_t& workingAddress);

void setup() {
  // Early serial output to confirm execution
  Serial.begin(115200);
  delay(500); // Short delay for monitor sync
  Serial.println("ESP32 Renogy Test: Initializing...");

  // Clear residual data in Serial buffer
  while (Serial.available()) {
    Serial.read();
  }
  Serial.println("Serial buffer cleared.");

  // Test UART2 with loopback (optional, requires TX-RX shorted)
  Serial.println("Testing UART2 loopback...");
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
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
  static bool addressFound = false;
  static uint8_t workingAddress = 0;

  // Try reading controller's Modbus address from register 0x01A
  if (!addressFound) {
    Serial.println("Attempting to read controller Modbus address from 0x01A...");
    for (uint8_t addr : {DEFAULT_ADDRESS_1, DEFAULT_ADDRESS_2}) {
      Serial.println("Trying default address: " + String(addr));
      node.begin(addr, Serial2);
      uint8_t result = node.readHoldingRegisters(0x01A, 1);
      if (result == node.ku8MBSuccess) {
        workingAddress = node.getResponseBuffer(0);
        addressFound = true;
        Serial.println("Success! Controller Modbus address: " + String(workingAddress));
        break;
      } else {
        Serial.println("Failed to read 0x01A at address " + String(addr) + ". Error code: 0x" + String(result, HEX));
      }
      delay(500);
    }
  }

  // If address found, use it; otherwise, scan 1 to 247
  if (addressFound) {
    Serial.println("Using Modbus address: " + String(workingAddress));
    node.begin(workingAddress, Serial2);
    testModbusRead(workingAddress, addressFound, workingAddress);
    delay(2000);
  } else {
    Serial.println("No controller address found. Scanning Modbus addresses 1 to 247...");
    for (uint8_t address = 1; address <= 247 && !addressFound; address++) {
      Serial.print("Testing Modbus address: ");
      Serial.println(address);
      node.begin(address, Serial2);
      testModbusRead(address, addressFound, workingAddress);
      delay(250); // Short delay for faster scanning
    }
    if (!addressFound) {
      Serial.println("No working address found. Retrying in 5 seconds...");
      Serial.println("---");
      delay(5000);
    }
  }
}

void testModbusRead(uint8_t address, bool& found, uint8_t& workingAddress) {
  // Check for unexpected data
  if (Serial2.available()) {
    Serial.println("Unexpected data before Modbus request:");
    while (Serial2.available()) {
      Serial.print("0x");
      Serial.print(Serial2.read(), HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // Read battery voltage register (0x0101)
  uint8_t result = node.readHoldingRegisters(0x0101, 1);

  if (result == node.ku8MBSuccess) {
    uint16_t raw_value = node.getResponseBuffer(0);
    float battery_voltage = raw_value * 0.1; // Convert to volts
    Serial.println("Success! Battery Voltage: " + String(battery_voltage, 1) + " V at address " + String(address));
    found = true;
    workingAddress = address;
  } else {
    Serial.print("Failed at address ");
    Serial.print(address);
    Serial.print(". Error code: 0x");
    Serial.println(result, HEX);
  }
}