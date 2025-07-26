/*

Reads the data from the Renogy charge controller via its RS232 port using an ESP32 or similar. Tested with Wanderer 30A (CTRL-WND30-LI) and Wanderer 10A

See my Github repo for notes on building the cable:
https://github.com/wrybread/ESP32ArduinoRenogy

Notes: 
- I don't think can power the ESP32 from the Renogy's USB port.. Maybe it's so low power that it shuts off?

To do:
- find out how much of a load the load port can handle... 
- test with an Arduino

*/

// https://github.com/syvic/ModbusMaster
#include <ModbusMaster.h>
ModbusMaster node;

#define RXD2 16
#define TXD2 17

void renogy_read_data_registers();
void renogy_read_info_registers();
void renogy_control_load(bool state);

/*
Number of registers to check. I think all Renogy controllers have 35
data registers (not all of which are used) and 17 info registers.
*/
const uint32_t num_data_registers = 35;
const uint32_t num_info_registers = 17;

// if you don't have a charge controller to test with, can set this to true to get non 0 voltage readings
bool simulator_mode = false;

// A struct to hold the controller data
struct Controller_data {
  
  uint8_t battery_soc;               // percent
  float battery_voltage;             // volts
  float battery_charging_amps;       // amps
  uint8_t battery_temperature;       // celcius
  uint8_t controller_temperature;    // celcius
  float load_voltage;                // volts
  float load_amps;                   // amps
  uint8_t load_watts;                // watts
  float solar_panel_voltage;         // volts
  float solar_panel_amps;            // amps
  uint8_t solar_panel_watts;         // watts
  float min_voltage_today;           // volts
  float max_voltage_today;          // volts
  float max_charging_amps_today;     // amps
  float max_discharging_amps_today;  // amps
  uint8_t max_charge_watts_today;    // watts
  uint8_t max_discharge_watts_today; // watts
  uint16_t charge_amphours_today;     // amp hours
  uint16_t discharge_amphours_today;  // amp hours
  uint16_t charge_watthours_today;    // watt hours
  uint16_t discharge_watthours_today; // watt hours
  uint16_t controller_uptime_days;    // days
  uint16_t total_battery_overcharges; // count
  uint16_t total_battery_fullcharges; // count

  // convenience values
  float battery_temperatureF;        // fahrenheit
  float controller_temperatureF;     // fahrenheit
  float battery_charging_watts;       // watts
  long last_update_time;             // millis() of last update time
  bool controller_connected;         // bool if we successfully read data from the controller
};
Controller_data renogy_data;

// A struct to hold the controller info params
struct Controller_info {
  
  uint8_t voltage_rating;            // volts
  uint8_t amp_rating;                // amps
  uint8_t discharge_amp_rating;      // amps
  uint8_t type;
  uint8_t controller_name;
  char software_version[40];
  char hardware_version[40];
  char serial_number[40];
  uint8_t modbus_address;  

  float wattage_rating;
  long last_update_time;           // millis() of last update time
};
Controller_info renogy_info;

void setup()
{
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
  
  // Initialize Serial2 for Modbus
  Serial.println("Initializing Serial2 for Modbus...");
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial2 initialized");
  
  // Initialize Modbus
  int modbus_address = 255;
  Serial.println("Initializing Modbus with address: " + String(modbus_address));
  node.begin(modbus_address, Serial2);
  Serial.println("Modbus initialized");

  // Test single register read
  Serial.println("Testing single register read (address 0x100)...");
  uint8_t result = node.readHoldingRegisters(0x100, 1);
  if (result == node.ku8MBSuccess) {
    Serial.println("Single register read successful! Value: " + String(node.getResponseBuffer(0)));
  } else if (result == 0xE2) {
    Serial.println("Single register read timed out!");
  } else {
    Serial.print("Single register read failed, error code: ");
    Serial.println(result, HEX);
  }
}

void loop()
{
  static uint32_t i;
  i++;
  
  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  node.setTransmitBuffer(0, lowWord(i));  
  // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
  node.setTransmitBuffer(1, highWord(i));

  renogy_read_data_registers();
  renogy_read_info_registers();

  Serial.println("Battery voltage: " + String(renogy_data.battery_voltage));
  Serial.println("Battery charge level: " + String(renogy_data.battery_soc) + "%");
  Serial.println("Panel wattage: " + String(renogy_data.solar_panel_watts));
  Serial.println("controller_temperatureF=" + String(renogy_data.controller_temperatureF)); 
  Serial.println("battery_temperatureF=" + String(renogy_data.battery_temperatureF));
  Serial.println("---");

  delay(1000); 
}

void renogy_read_data_registers() 
{
  uint8_t j, result;
  uint16_t data_registers[num_data_registers];
  char buffer1[40], buffer2[40];
  uint8_t raw_data;

  // prints data about each read to the console
  bool print_data = 0; 
  
  Serial.println("Attempting to read data registers...");
  result = node.readHoldingRegisters(0x100, num_data_registers);
  if (result == node.ku8MBSuccess)
  {
    Serial.println("Successfully read the data registers!");
    renogy_data.controller_connected = true;
    for (j = 0; j < num_data_registers; j++)
    {
      data_registers[j] = node.getResponseBuffer(j);
      if (print_data) Serial.println(data_registers[j]);
    }

    renogy_data.battery_soc = data_registers[0]; 
    renogy_data.battery_voltage = data_registers[1] * .1;
    renogy_data.battery_charging_amps = data_registers[2] * .1;

    renogy_data.battery_charging_watts = renogy_data.battery_voltage * renogy_data.battery_charging_amps;
    
    //0x103 returns two bytes, one for battery and one for controller temp in c
    uint16_t raw_data = data_registers[3];
    renogy_data.controller_temperature = raw_data/256;
    renogy_data.battery_temperature = raw_data%256; 
    // for convenience, fahrenheit versions of the temperatures
    renogy_data.controller_temperatureF = (renogy_data.controller_temperature * 1.8)+32;
    renogy_data.battery_temperatureF = (renogy_data.battery_temperature * 1.8)+32;

    renogy_data.load_voltage = data_registers[4] * .1;
    renogy_data.load_amps = data_registers[5] * .01;
    renogy_data.load_watts = data_registers[6];
    renogy_data.solar_panel_voltage = data_registers[7] * .1;
    renogy_data.solar_panel_amps = data_registers[8] * .01;
    renogy_data.solar_panel_watts = data_registers[9];
    renogy_data.min_voltage_today = data_registers[11] * .1;
    renogy_data.max_voltage_today = data_registers[12] * .1; 
    renogy_data.max_charging_amps_today = data_registers[13] * .01;
    renogy_data.max_discharging_amps_today = data_registers[14] * .1;
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

    if (print_data) Serial.println("---");
  }
  else 
  {
    if (result == 0xE2) 
    {
      Serial.println("Timed out reading the data registers!");
    }
    else 
    {
      Serial.print("Failed to read the data registers, error code: ");
      Serial.println(result, HEX);
    }
    // Reset some values if we don't get a reading
    renogy_data.controller_connected = false;
    renogy_data.battery_voltage = 0; 
    renogy_data.battery_charging_amps = 0;
    renogy_data.battery_soc = 0;
    renogy_data.battery_charging_amps = 0;
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

void renogy_read_info_registers() 
{
  uint8_t j, result;
  uint16_t info_registers[num_info_registers];
  char buffer1[40], buffer2[40];
  uint8_t raw_data;

  // prints data about the read to the console
  bool print_data = 0;
  
  Serial.println("Attempting to read info registers...");
  result = node.readHoldingRegisters(0x00A, num_info_registers);
  if (result == node.ku8MBSuccess)
  {
    Serial.println("Successfully read the info registers!");
    for (j = 0; j < num_info_registers; j++)
    {
      info_registers[j] = node.getResponseBuffer(j);
      if (print_data) Serial.println(info_registers[j]);
    }

    // read and process each value
    raw_data = info_registers[0]; 
    renogy_info.voltage_rating = raw_data/256; 
    renogy_info.amp_rating = raw_data%256;
    renogy_info.wattage_rating = renogy_info.voltage_rating * renogy_info.amp_rating;

    raw_data = info_registers[1]; 
    renogy_info.discharge_amp_rating = raw_data/256;
    renogy_info.type = raw_data%256;

    itoa(info_registers[10],buffer1,10); 
    itoa(info_registers[11],buffer2,10);
    strcat(buffer1, buffer2);
    strcpy(renogy_info.software_version, buffer1); 

    itoa(info_registers[12],buffer1,10); 
    itoa(info_registers[13],buffer2,10);
    strcat(buffer1, buffer2);
    strcpy(renogy_info.hardware_version, buffer1);

    itoa(info_registers[14],buffer1,10); 
    itoa(info_registers[15],buffer2,10);
    strcat(buffer1, buffer2);
    strcpy(renogy_info.serial_number, buffer1);

    renogy_info.modbus_address = info_registers[16];
    renogy_info.last_update_time = millis();
  
    if (print_data) Serial.println("---");
  }
  else
  {
    if (result == 0xE2) 
    {
      Serial.println("Timed out reading the info registers!");
    }
    else 
    {
      Serial.print("Failed to read the info registers, error code: ");
      Serial.println(result, HEX);
    }
  }
}

void renogy_control_load(bool state) {
  if (state==1) node.writeSingleRegister(0x010A, 1);
  else node.writeSingleRegister(0x010A, 0);
}