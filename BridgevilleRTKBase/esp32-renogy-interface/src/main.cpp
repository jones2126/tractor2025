#include <Arduino.h>
#include <ModbusMaster.h>

// RS232 Communication pins
#define RXD2 17
#define TXD2 16

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

  // Scan addresses if none found
  if (!addressFound) {
    Serial.println("Scanning Modbus addresses 1 to 247...");
    for (uint8_t address = 1; address <= 247 && !addressFound; address++) {
      Serial.print("Testing Modbus address: ");
      Serial.println(address);
      node.begin(address, Serial2);
      testModbusRead(address, addressFound, workingAddress);
      delay(500); // Short delay between addresses
    }
    if (!addressFound) {
      Serial.println("No working address found. Retrying in 5 seconds...");
      Serial.println("---");
      delay(5000);
    }
  } else {
    // Use working address to repeatedly query
    Serial.println("Using working Modbus address: " + String(workingAddress));
    node.begin(workingAddress, Serial2);
    testModbusRead(workingAddress, addressFound, workingAddress);
    delay(2000); // Query every 2 seconds
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
    found = true; // Stop scanning
    workingAddress = address; // Store working address
  } else {
    Serial.print("Failed at address ");
    Serial.print(address);
    Serial.print(". Error code: 0x");
    Serial.println(result, HEX);
    if (result == 0xE2) {
      Serial.println("Error: Modbus timeout. Check RS232 wiring, device power, or address.");
    } else if (result == 0xE1) {
      Serial.println("Error: Invalid response. Possible device mismatch.");
    }
  }
}