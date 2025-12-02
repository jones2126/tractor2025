#include <SPI.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

// THIS ORDER WORKED
const uint8_t address[][6] = {"TRACT", "RCTRL"};

struct RadioControlStruct {
    float steering_val;
    float throttle_val;
    float voltage;
    float transmission_val;
    float pot4_val;
    byte estop;
    byte control_mode;
    byte button01;
    byte button02;
};
RadioControlStruct radioData;

struct __attribute__((packed)) AckPayloadStruct {
    unsigned long counter;
    uint32_t dummy[4];
};
AckPayloadStruct ackPayload;

void setup() {
    Serial.begin(921600);
    while (!Serial) delay(10);

    Serial.println("=== ROBOT WORKING VERSION ===");

    SPI.begin();
    delay(100);

    if (!radio.begin()) {
        Serial.println("RADIO FAIL");
        while(1);
    }

    radio.setChannel(124);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_HIGH);
    radio.enableAckPayload();

    // THIS WAS THE WINNING CONFIG
    radio.openWritingPipe(address[1]);    // Send ACK to "RCTRL"
    radio.openReadingPipe(1, address[0]); // LISTEN ON "TRACT"
    radio.startListening();

    Serial.println("ROBOT: Listening on TRACT, ACK to RCTRL");
}

void loop() {
    if (radio.available()) {
        radio.read(&radioData, sizeof(radioData));
        ackPayload.counter++;
        radio.writeAckPayload(1, &ackPayload, sizeof(ackPayload));

        Serial.print("RX #");
        Serial.print(ackPayload.counter);
        Serial.print(": mode=");
        Serial.print(radioData.control_mode);
        Serial.print(" steer=");
        Serial.print(radioData.steering_val, 0);
        Serial.print(" trans=");
        Serial.print(radioData.transmission_val, 0);
        Serial.println();
    }
}