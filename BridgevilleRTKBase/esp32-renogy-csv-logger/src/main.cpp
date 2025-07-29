#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ModbusMaster.h>
#include <FS.h>
#include <SPIFFS.h>

// Temperature sensor setup
const int oneWireBus = 32;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Modbus setup for Renogy
ModbusMaster node;
#define RXD2 16
#define TXD2 17

// Timing variables
unsigned long lastTempRead = 0;
unsigned long lastRenogyRead = 0;
unsigned long lastCSVWrite = 0;
const unsigned long tempInterval = 15000;    // 15 seconds
const unsigned long renogyInterval = 20000;  // 20 seconds
const unsigned long csvInterval = 60000;     // 60 seconds (1 minute)

// Temperature averaging
float tempSum = 0;
int tempCount = 0;
float avgTemperatureC = 0;
float avgTemperatureF = 0;

// Renogy data structures
const uint32_t num_data_registers = 35;
const uint32_t num_info_registers = 17;
bool simulator_mode = false;

struct Controller_data {
  uint8_t battery_soc;
  float battery_voltage;
  float battery_charging_amps;
  uint8_t battery_temperature;
  uint8_t controller_temperature;
  float load_voltage;
  float load_amps;
  uint8_t load_watts;
  float solar_panel_voltage;
  float solar_panel_amps;
  uint8_t solar_panel_watts;
  float min_voltage_today;
  float max_voltage_today;
  float max_charging_amps_today;
  float max_discharging_amps_today;
  uint8_t max_charge_watts_today;
  uint8_t max_discharge_watts_today;
  uint16_t charge_amphours_today;
  uint16_t discharge_amphours_today;
  uint16_t charge_watthours_today;
  uint16_t discharge_watthours_today;
  uint16_t controller_uptime_days;
  uint16_t total_battery_overcharges;
  uint16_t total_battery_fullcharges;
  float battery_temperatureF;
  float controller_temperatureF;
  float battery_charging_watts;
  long last_update_time;
  bool controller_connected;
};
Controller_data renogy_data;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Wait for user input
  unsigned long lastPrintTime = 0;
  const unsigned long printInterval = 5000;
  while (Serial.available() == 0) {
    if (millis() - lastPrintTime >= printInterval) {
      Serial.println("Waiting for user input... Send any character to continue.");
      lastPrintTime = millis();
    }
    delay(100);
  }
  
  while (Serial.available()) {
    Serial.read();
  }
  
  Serial.println("User input received! Starting setup...");
  delay(1000);
  Serial.println("Starting!");
  
  // Initialize SPIFFS for CSV file storage
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }
  Serial.println("SPIFFS mounted successfully");
  
  // Initialize temperature sensor
  sensors.begin();
  Serial.println("DS18B20 temperature sensor initialized on GPIO32");
  
  // Initialize Modbus for Renogy
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  node.begin(255, Serial2);
  Serial.println("Modbus initialized for Renogy communication");
  
  // Initialize CSV file with headers
  initializeCSVFile();
  
  Serial.println("Setup complete! Starting data logging...");
}

void loop() {
  readTemperature();
  readRenogy();
  postResults();
  
  delay(100); // Small delay to prevent excessive CPU usage
}

void readTemperature() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastTempRead >= tempInterval) {
    sensors.requestTemperatures();
    float currentTemp = sensors.getTempCByIndex(0);
    
    if (currentTemp != DEVICE_DISCONNECTED_C) {
      tempSum += currentTemp;
      tempCount++;
      
      // Calculate running average
      avgTemperatureC = tempSum / tempCount;
      avgTemperatureF = (avgTemperatureC * 9.0 / 5.0) + 32.0;
      
      Serial.println("Temperature reading: " + String(currentTemp) + "°C (Avg: " + String(avgTemperatureC) + "°C)");
    } else {
      Serial.println("Error reading temperature sensor");
    }
    
    lastTempRead = currentTime;
  }
}

void readRenogy() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastRenogyRead >= renogyInterval) {
    Serial.println("Reading Renogy data...");
    renogy_read_data_registers();
    
    if (renogy_data.controller_connected) {
      Serial.println("Renogy - Battery: " + String(renogy_data.battery_voltage) + "V (" + 
                     String(renogy_data.battery_soc) + "%), Solar: " + 
                     String(renogy_data.solar_panel_watts) + "W");
    } else {
      Serial.println("Renogy controller not connected");
    }
    
    lastRenogyRead = currentTime;
  }
}

void postResults() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastCSVWrite >= csvInterval) {
    writeToCSV();
    
    // Reset temperature averaging for next period
    tempSum = 0;
    tempCount = 0;
    
    lastCSVWrite = currentTime;
  }
}

void initializeCSVFile() {
  File file = SPIFFS.open("/data_log.csv", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to create CSV file");
    return;
  }
  
  // Write CSV headers
  file.println("Timestamp,Avg_Temp_C,Avg_Temp_F,Battery_Voltage,Battery_SOC,Battery_Charging_Amps,Solar_Panel_Voltage,Solar_Panel_Amps,Solar_Panel_Watts,Controller_Temp_C,Battery_Temp_C,Load_Voltage,Load_Amps,Load_Watts");
  file.close();
  Serial.println("CSV file initialized with headers");
}

void writeToCSV() {
  File file = SPIFFS.open("/data_log.csv", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open CSV file for writing");
    return;
  }
  
  // Create timestamp (milliseconds since boot)
  unsigned long timestamp = millis();
  
  // Write data row
  file.print(timestamp);
  file.print(",");
  file.print(avgTemperatureC, 2);
  file.print(",");
  file.print(avgTemperatureF, 2);
  file.print(",");
  file.print(renogy_data.battery_voltage, 2);
  file.print(",");
  file.print(renogy_data.battery_soc);
  file.print(",");
  file.print(renogy_data.battery_charging_amps, 2);
  file.print(",");
  file.print(renogy_data.solar_panel_voltage, 2);
  file.print(",");
  file.print(renogy_data.solar_panel_amps, 2);
  file.print(",");
  file.print(renogy_data.solar_panel_watts);
  file.print(",");
  file.print(renogy_data.controller_temperature);
  file.print(",");
  file.print(renogy_data.battery_temperature);
  file.print(",");
  file.print(renogy_data.load_voltage, 2);
  file.print(",");
  file.print(renogy_data.load_amps, 2);
  file.print(",");
  file.println(renogy_data.load_watts);
  
  file.close();
  
  Serial.println("Data written to CSV - Temp: " + String(avgTemperatureC) + "°C, Battery: " + 
                 String(renogy_data.battery_voltage) + "V (" + String(renogy_data.battery_soc) + "%)");
}

void renogy_read_data_registers() {
  uint8_t j, result;
  uint16_t data_registers[num_data_registers];
  
  result = node.readHoldingRegisters(0x100, num_data_registers);
  if (result == node.ku8MBSuccess) {
    renogy_data.controller_connected = true;
    for (j = 0; j < num_data_registers; j++) {
      data_registers[j] = node.getResponseBuffer(j);
    }

    renogy_data.battery_soc = data_registers[0];
    renogy_data.battery_voltage = data_registers[1] * 0.1;
    renogy_data.battery_charging_amps = data_registers[2] * 0.1;
    renogy_data.battery_charging_watts = renogy_data.battery_voltage * renogy_data.battery_charging_amps;
    
    uint16_t raw_data = data_registers[3];
    renogy_data.controller_temperature = raw_data / 256;
    renogy_data.battery_temperature = raw_data % 256;
    renogy_data.controller_temperatureF = (renogy_data.controller_temperature * 1.8) + 32;
    renogy_data.battery_temperatureF = (renogy_data.battery_temperature * 1.8) + 32;

    renogy_data.load_voltage = data_registers[4] * 0.1;
    renogy_data.load_amps = data_registers[5] * 0.01;
    renogy_data.load_watts = data_registers[6];
    renogy_data.solar_panel_voltage = data_registers[7] * 0.1;
    renogy_data.solar_panel_amps = data_registers[8] * 0.01;
    renogy_data.solar_panel_watts = data_registers[9];
    renogy_data.min_voltage_today = data_registers[11] * 0.1;
    renogy_data.max_voltage_today = data_registers[12] * 0.1;
    renogy_data.max_charging_amps_today = data_registers[13] * 0.01;
    renogy_data.max_discharging_amps_today = data_registers[14] * 0.1;
    renogy_data.max_charge_watts_today = data_registers[15];
    renogy_data.max_discharge_watts_today = data_registers[16];
    renogy_data.charge_amphours_today = data_registers[17];
    renogy_data.discharge_amphours_today = data_registers[18];
    renogy_data.charge_watthours_today = data_registers[19];
    renogy_data.discharge_watthours_today = data_registers[20];
    renogy_data.controller_uptime_days = data_registers[21];
    renogy_data.total_battery_overcharges = data_registers[22];
    renogy_data.total_battery_fullcharges = data_registers[23];
    renogy_data.last_update_time = millis();
  } else {
    renogy_data.controller_connected = false;
    // Reset values on failure
    renogy_data.battery_voltage = 0;
    renogy_data.battery_charging_amps = 0;
    renogy_data.battery_soc = 0;
    renogy_data.controller_temperature = 0;
    renogy_data.battery_temperature = 0;
    renogy_data.solar_panel_amps = 0;
    renogy_data.solar_panel_watts = 0;
    renogy_data.battery_charging_watts = 0;
    
    if (simulator_mode) {
      renogy_data.battery_voltage = 13.99;
      renogy_data.battery_soc = 55;
    }
  }
}