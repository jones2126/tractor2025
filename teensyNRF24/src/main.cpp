#include <SPI.h>
#include <RF24.h>

// NRF24 radio setup
RF24 radio(9, 10);  // CE, CSN

// Matching structure from your working code
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

const uint8_t address[][6] = {"RCTRL", "TRACT"};

void setup() {
    delay(1000);
    Serial.begin(115200);
    while (!Serial && millis() < 10000);

    Serial.println("Starting NRF24 test with original config...");

    // Same init logic from your working code
    bool initialized = false;
    for (int i = 0; i < 5; i++) {
        Serial.print("Radio init attempt ");
        Serial.println(i + 1);
        if (radio.begin()) {
            initialized = true;
            Serial.println("Radio initialized!");
            break;
        }
        delay(1000);
    }

    if (!initialized) {
        Serial.println("Radio hardware not responding.");
        return;
    }

    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(124);
    radio.openWritingPipe(address[0]);    // Send to RCTRL
    radio.openReadingPipe(1, address[1]); // Receive from TRACT
    radio.enableAckPayload();
    radio.startListening();
    Serial.println("Listening for packets...");
}

void loop() {
    if (radio.available()) {
        radio.read(&radioData, sizeof(radioData));
        Serial.print("transmission_val: ");
        Serial.println(radioData.transmission_val, 2);
    }
}
