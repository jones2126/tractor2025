#include <Arduino.h>
#include <HardwareSerial.h>

// RS232 Communication pins (corrected from RS485)
#define RS232_RX 17
#define RS232_TX 16

// Renogy device address (default is usually 1)
#define RENOGY_ADDRESS 1

// Create a hardware serial instance
HardwareSerial renogySerial(2); // Use UART2

// Function declarations (forward declarations)
void queryRenogyRegister(uint16_t registerAddress, const char* description);
void sendRS232Message(uint8_t* message, int length);
uint16_t calculateCRC16(uint8_t* data, int length);

void setup() {
  // Initialize serial for debugging
  delay(1000);
  Serial.begin(115200);
  delay(4000);
  Serial.println("Renogy Wanderer Query Test");
  Serial.println("===========================");
  
  // Initialize RS232 communication
  renogySerial.begin(9600, SERIAL_8N1, RS232_RX, RS232_TX);
  
  delay(2000);
  Serial.println("RS232 initialized. Starting queries...");
}

void loop() {
  // Query basic status registers
  queryRenogyRegister(0x100A, "Battery Voltage");      // Battery voltage
  delay(1000);
  
  queryRenogyRegister(0x100B, "Charging Current");     // Charging current  
  delay(1000);
  
  queryRenogyRegister(0x100C, "Battery SOC");          // State of charge
  delay(1000);
  
  queryRenogyRegister(0x100D, "Battery Temp");         // Battery temperature
  delay(1000);
  
  queryRenogyRegister(0x1011, "Solar Voltage");        // Solar panel voltage
  delay(1000);
  
  queryRenogyRegister(0x1012, "Solar Current");        // Solar panel current
  delay(1000);
  
  Serial.println("------------------------");
  delay(5000); // Wait 5 seconds before next round
}

void queryRenogyRegister(uint16_t registerAddress, const char* description) {
  uint8_t query[8];
  
  // Build Modbus RTU query: [Address][Function][Register High][Register Low][Count High][Count Low][CRC Low][CRC High]
  query[0] = RENOGY_ADDRESS;    // Device address
  query[1] = 0x03;              // Function code: Read Holding Registers
  query[2] = (registerAddress >> 8) & 0xFF;  // Register address high byte
  query[3] = registerAddress & 0xFF;         // Register address low byte  
  query[4] = 0x00;              // Number of registers high byte
  query[5] = 0x01;              // Number of registers low byte (read 1 register)
  
  // Calculate CRC16
  uint16_t crc = calculateCRC16(query, 6);
  query[6] = crc & 0xFF;        // CRC low byte
  query[7] = (crc >> 8) & 0xFF; // CRC high byte
  
  // Send query
  sendRS232Message(query, 8);
  
  // Wait for response
  delay(100);
  
  // Read response
  if (renogySerial.available()) {
    uint8_t response[8];
    int bytesRead = 0;
    
    // Read available bytes (timeout after 500ms)
    unsigned long startTime = millis();
    while (bytesRead < 7 && (millis() - startTime) < 500) {
      if (renogySerial.available()) {
        response[bytesRead] = renogySerial.read();
        bytesRead++;
      }
    }
    
    if (bytesRead >= 7) {
      // Parse response: [Address][Function][Byte Count][Data High][Data Low][CRC Low][CRC High]
      if (response[0] == RENOGY_ADDRESS && response[1] == 0x03) {
        uint16_t value = (response[3] << 8) | response[4];
        
        Serial.print(description);
        Serial.print(": ");
        
        // Convert raw value based on register type
        if (strstr(description, "Voltage")) {
          Serial.print(value / 10.0, 1);
          Serial.println(" V");
        } else if (strstr(description, "Current")) {
          Serial.print(value / 100.0, 2);
          Serial.println(" A");
        } else if (strstr(description, "SOC")) {
          Serial.print(value);
          Serial.println(" %");
        } else if (strstr(description, "Temp")) {
          // Temperature might be in different format, adjust as needed
          Serial.print(value);
          Serial.println(" (raw)");
        } else {
          Serial.print(value);
          Serial.println(" (raw)");
        }
      } else {
        Serial.print(description);
        Serial.println(": Invalid response");
      }
    } else {
      Serial.print(description);
      Serial.println(": No response or timeout");
    }
    
    // Print raw response for debugging
    Serial.print("Raw response: ");
    for (int i = 0; i < bytesRead; i++) {
      Serial.print("0x");
      if (response[i] < 0x10) Serial.print("0");
      Serial.print(response[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    
  } else {
    Serial.print(description);
    Serial.println(": No data received");
  }
}

void sendRS232Message(uint8_t* message, int length) {
  // Send message (no direction control needed for RS232)
  renogySerial.write(message, length);
  renogySerial.flush(); // Wait for transmission to complete
  
  // Debug: print sent message
  Serial.print("Sent: ");
  for (int i = 0; i < length; i++) {
    Serial.print("0x");
    if (message[i] < 0x10) Serial.print("0");
    Serial.print(message[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

uint16_t calculateCRC16(uint8_t* data, int length) {
  uint16_t crc = 0xFFFF;
  
  for (int i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  return crc;
}