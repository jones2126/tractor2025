#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Counter variable for TTL messages
int counter = 0;

// TTL Serial pins
#define RXD2 16  // Define RXD2 as GPIO16
#define TXD2 17  // Define TXD2 as GPIO17

// Temperature sensor setup
const int oneWireBus = 32;     // GPIO where the DS18B20 is connected
OneWire oneWire(oneWireBus);   // Setup a oneWire instance
DallasTemperature sensors(&oneWire); // Pass oneWire reference to Dallas Temperature sensor

void setup() {
  // Start Serial for debugging/monitoring
  Serial.begin(115200);
  delay(2000); // Allow USB to stabilize
  
  // Wait for user input, printing message every 5 seconds
  unsigned long lastPrintTime = 0;
  const unsigned long printInterval = 5000; // 5 seconds
  while (Serial.available() == 0) {
    if (millis() - lastPrintTime >= printInterval) {
      Serial.println("Waiting for user input... Send any character to continue.");
      lastPrintTime = millis();
    }
    delay(100); // Small delay to prevent excessive CPU usage
  }
  
  // Clear the input buffer
  while (Serial.available()) {
    Serial.read(); // Read and discard any input
  }
  
  Serial.println("User input received! Starting setup...");
  delay(1000); // Brief delay for monitor to catch up
  
  Serial.println("Starting!");
  
  // Initialize TTL Serial communication
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("TTL Serial initialized on GPIO16 (TX) and GPIO17 (RX)");
  
  // Initialize DS18B20 temperature sensor
  sensors.begin();
  Serial.println("DS18B20 temperature sensor initialized on GPIO32");
  
  Serial.println("Setup complete! Starting main loop...");
  Serial.println("TTL Loopback Test Starting...");
}

void loop() {
  // TTL Loopback functionality
  Serial2.println("Hello-5 TTL Loopback! Count: " + String(counter));
  
  // Check for TTL response
  if (Serial2.available()) {
    String receivedData = "";
    while (Serial2.available()) {
      char c = Serial2.read();
      if (c == '\n' || c == '\r') {
        if (receivedData.length() > 0) {
          Serial.print("TTL Received: ");
          Serial.println(receivedData);
          receivedData = "";
        }
      } else {
        receivedData += c;
      }
      delay(1); // Small delay to allow more data to arrive
    }
    // Print any remaining data that didn't end with newline
    if (receivedData.length() > 0) {
      Serial.print("TTL Received: ");
      Serial.println(receivedData);
    }
  }
  
  // Temperature sensor reading
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  
  // Print temperature readings
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.print("°C / ");
  Serial.print(temperatureF);
  Serial.println("°F");
  
  // Increment counter
  counter++;
  
  // Wait before next iteration
  delay(1000); // 1 second delay for TTL, but temperature will be read every loop
}