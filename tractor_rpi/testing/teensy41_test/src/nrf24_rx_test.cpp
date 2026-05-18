// nrf24_rx_test.cpp
// Minimal NRF24 receiver test for Teensy 4.1
// Flash this, open serial monitor at 115200
// You should see packet counts and steering values if radio is working

#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);  // CE, CSN — same as main firmware

const uint8_t ADDR_TRACTOR[6]       = "1Node";
const uint8_t ADDR_RADIO_CONTROL[6] = "2Node";

struct __attribute__((packed)) RadioControlStruct {
    int16_t steering_val;
    int16_t throttle_val;
    int16_t transmission_val;
    uint16_t voltage_mv;
    int16_t pot4_val;
    byte estop;
    byte control_mode;
    byte button02;
    byte button03;
};

struct __attribute__((packed)) AckPayloadStruct {
    byte gps_status;
    byte button02_status;
    byte button03_status;
    byte padding[11];
};

RadioControlStruct radioData;
AckPayloadStruct ackPayload;

unsigned long packetCount = 0;
unsigned long lastPrint = 0;

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("NRF24 RX Test starting...");

    SPI.setSCK(13);
    SPI.setMOSI(11);
    SPI.setMISO(12);
    SPI.begin();
    delay(100);

    if (!radio.begin()) {
        Serial.println("ERROR: Radio hardware not responding!");
        while (1) { delay(500); Serial.println("Radio failed."); }
    }
    Serial.println("Radio initialized OK");

    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.setPayloadSize(14);
    radio.enableAckPayload();

    radio.openWritingPipe(ADDR_RADIO_CONTROL);
    radio.openReadingPipe(1, ADDR_TRACTOR);

    // radio.openWritingPipe(ADDR_TRACTOR);          // was ADDR_RADIO_CONTROL
    // radio.openReadingPipe(1, ADDR_RADIO_CONTROL); // was ADDR_TRACTOR
    
    radio.startListening();

    // Simple ACK payload
    memset(&ackPayload, 0, sizeof(ackPayload));
    ackPayload.gps_status = 2;  // dummy value so handheld LED goes yellow

    Serial.println("Listening... (should see packets from handheld)");
}

void loop() {
    if (radio.available()) {
        radio.read(&radioData, sizeof(RadioControlStruct));
        packetCount++;

        // Send ACK so handheld LED goes green
        radio.writeAckPayload(1, &ackPayload, sizeof(AckPayloadStruct));

        // Print every packet
        Serial.print("PKT #"); Serial.print(packetCount);
        Serial.print("  steer="); Serial.print(radioData.steering_val);
        Serial.print("  mode="); Serial.print(radioData.control_mode);
        Serial.print("  estop="); Serial.println(radioData.estop);
    }

    if (millis() - lastPrint >= 5000) {
        Serial.print("Packets received in last 5s: ");
        Serial.println(packetCount);
        packetCount = 0;
        lastPrint = millis();
    }
}