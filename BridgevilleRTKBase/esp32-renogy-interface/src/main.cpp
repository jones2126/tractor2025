#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WebServer.h>

// Network credentials
const char* ssid = "cui_bono";
const char* password = "Andrew13";

// GPIO where the DS18B20 is connected
const int oneWireBus = 16;

// DS18B20 sensor addresses
DeviceAddress sensor1 = {0x28, 0xFF, 0x7C, 0x65, 0x66, 0x14, 0x02, 0x0A};
DeviceAddress sensor2 = {0x28, 0xFF, 0x21, 0x5D, 0x66, 0x14, 0x02, 0x27};
DeviceAddress sensor3 = {0x28, 0xFF, 0xA7, 0x06, 0x66, 0x14, 0x01, 0xDE};

// Function declarations
String formatTemperatureHTML(const char* sensorName, DeviceAddress address, float offset);
String formatTemperatureSerial(const char* sensorName, DeviceAddress address, float offset);
String formatAverageTemperatureSerial();

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
WebServer server(80);

// Hardcoded offsets (set to 0 for testing)
const float offset1 = 0.0;
const float offset2 = 0.0;
const float offset3 = 0.0;

void setup() {
  Serial.begin(115200);
  Serial.flush(); // Clear serial buffer
  delay(2000); // Allow Serial Monitor to stabilize, there will still be some unprintable characters at boot.
  Serial.println("Starting ESP32...");

  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount());
  Serial.println(" DS18B20 sensors");

  // Connect to WiFi
  WiFi.setHostname("ESP32-TemperatureSensor");
  Serial.println("Connecting to WiFi: " + String(ssid));
  WiFi.begin(ssid, password);
  unsigned long startAttemptTime = millis();
  const unsigned long connectionTimeout = 30000;

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < connectionTimeout) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi. Continuing without WiFi...");
  } else {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

  // Start the server
  server.on("/", HTTP_GET, [&]() {
    Serial.println("Received HTTP request");
    sensors.requestTemperatures();
    String readings;
    readings += formatTemperatureHTML("Sensor 1", sensor1, offset1);
    readings += formatTemperatureHTML("Sensor 2", sensor2, offset2);
    readings += formatTemperatureHTML("Sensor 3", sensor3, offset3);

    const char index_html[] PROGMEM = R"rawliteral(
      <!DOCTYPE html>
      <html>
      <head>
        <title>ESP32 Temperature</title>
        <meta http-equiv="refresh" content="5">
        <style>
          body { font-family: Arial, sans-serif; margin: 20px; text-align: center; background-color: #f7f7f7; }
          h1 { color: #333; }
          p { font-size: 18px; color: #555; }
          .temp { font-weight: bold; color: #007BFF; }
          .error { color: red; }
        </style>
      </head>
      <body>
        <h1>ESP32 Temperature Readings</h1>
        <p>%READINGS%</p>
      </body>
      </html>
    )rawliteral";

    String html = index_html;
    html.replace("%READINGS%", readings);
    server.send(200, "text/html", html);
  });

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  static unsigned long lastReadingTime = 0;
  const unsigned long readingInterval = 5000;

  if (millis() - lastReadingTime >= readingInterval) {
    lastReadingTime = millis();
    sensors.requestTemperatures();
    Serial.println("Temperature Readings (in Fahrenheit):");
    Serial.println(formatTemperatureSerial("Sensor 1", sensor1, offset1));
    Serial.println(formatTemperatureSerial("Sensor 2", sensor2, offset2));
    Serial.println(formatTemperatureSerial("Sensor 3", sensor3, offset3));
    Serial.println(formatAverageTemperatureSerial());
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.println("--------------------------");
  }
}

String formatTemperatureHTML(const char* sensorName, DeviceAddress address, float offset) {
  float tempC = sensors.getTempC(address);
  if (tempC == DEVICE_DISCONNECTED_C) {
    return String(sensorName) + ": <span class='error'>Error (Sensor not found)</span><br>";
  } else {
    float adjustedTempC = tempC + offset;
    float tempF = adjustedTempC * 9.0 / 5.0 + 32.0;
    return String(sensorName) + ": <span class='temp'>" + String(tempF, 2) + " &deg;F</span><br>";
  }
}

String formatTemperatureSerial(const char* sensorName, DeviceAddress address, float offset) {
  float tempC = sensors.getTempC(address);
  if (tempC == DEVICE_DISCONNECTED_C) {
    return String(sensorName) + ": Error (Sensor not found)";
  } else {
    float adjustedTempC = tempC + offset;
    float tempF = adjustedTempC * 9.0 / 5.0 + 32.0;
    return String(sensorName) + ": " + String(tempF, 2) + " °F";
  }
}

String formatAverageTemperatureSerial() {
  float tempC1 = sensors.getTempC(sensor1) + offset1;
  float tempC2 = sensors.getTempC(sensor2) + offset2;
  float tempC3 = sensors.getTempC(sensor3) + offset3;
  int validSensors = 0;
  float sumTempC = 0.0;
  if (tempC1 != DEVICE_DISCONNECTED_C) { sumTempC += tempC1; validSensors++; }
  if (tempC2 != DEVICE_DISCONNECTED_C) { sumTempC += tempC2; validSensors++; }
  if (tempC3 != DEVICE_DISCONNECTED_C) { sumTempC += tempC3; validSensors++; }
  if (validSensors > 0) {
    float avgTempC = sumTempC / validSensors;
    float avgTempF = avgTempC * 9.0 / 5.0 + 32.0;
    return "Average: " + String(avgTempF, 2) + " °F";
  } else {
    return "Average: Error (No valid sensors)";
  }
}