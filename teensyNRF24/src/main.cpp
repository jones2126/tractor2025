//#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// Define CE and CSN pins for Teensy
RF24 radio(9, 10);

// Must match the struct sent from the transmitter
struct RadioData {
  uint16_t throttle_val;     // Optional
  uint16_t transmission_val; // What we're interested in
  uint8_t control_mode;
};

RadioData radioData;

// Must match transmitter pipe
const byte address[6] = "1Node";

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for USB Serial

  if (!radio.begin()) {
    Serial.println("NRF24 initialization failed!");
    while (1);
  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println("NRF24 initialized and listening...");
}

void loop() {
  if (radio.available()) {
    radio.read(&radioData, sizeof(radioData));
    Serial.print("transmission_val = ");
    Serial.println(radioData.transmission_val);
  }
}
