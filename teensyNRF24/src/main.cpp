#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);  // CE, CSN pins

// Data structure for receiving (matches transmitter)
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

// Data structure for acknowledgment
struct AckPayloadStruct {
    unsigned long counter;
    uint32_t dummy[4];
};

RadioControlStruct radioData;
AckPayloadStruct ackPayload;

const uint8_t address[][6] = {"RCTRL", "TRACT"};
unsigned long lastDataReceived = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);  // Wait up to 5 seconds for serial
    
    Serial.println("Simple NRF24 Receiver Starting...");

    // Configure alternate SPI pins (if using Teensy 3.5)
    SPI.setSCK(27);   
    SPI.setMOSI(28);  
    SPI.setMISO(39);  

    // Initialize SPI
    SPI.begin();
    delay(100);

    // Initialize radio
    bool initialized = false;
    for (int i = 0; i < 5; i++) {
        Serial.print("Radio initialization attempt ");
        Serial.println(i + 1);
        if (radio.begin()) {
            initialized = true;
            Serial.println("Radio initialized successfully!");
            break;
        }
        Serial.println("Radio init failed, retrying...");
        delay(1000);
    }

    if (!initialized) {
        Serial.println("Radio hardware not responding!");
        while(1) {
            delay(1000);
        }
    }

    // Configure radio settings
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(124);
    radio.openWritingPipe(address[0]);     // "RCTRL" - send ACKs back to transmitter
    radio.openReadingPipe(1, address[1]);  // "TRACT" - listen for data
    radio.enableAckPayload();
    radio.startListening();
    
    // Initialize ACK payload
    ackPayload.counter = 0;
    for (int i = 0; i < 4; i++) {
        ackPayload.dummy[i] = 0xDEADBEEF;
    }

    Serial.println("Setup complete - listening for data...");
    Serial.println("Transmission values will be printed below:");
}

void loop() {
    // Check for incoming data
    if (radio.available()) {
        // Read the data
        radio.read(&radioData, sizeof(RadioControlStruct));
        
        // Print the transmission value (main goal)
        Serial.print("transmission_val: ");
        Serial.println(radioData.transmission_val, 3);  // 3 decimal places
        
        // Optional: Print all values for debugging
        Serial.print("  [steering: ");
        Serial.print(radioData.steering_val, 2);
        Serial.print(", throttle: ");
        Serial.print(radioData.throttle_val, 2);
        Serial.print(", voltage: ");
        Serial.print(radioData.voltage, 1);
        Serial.print(", estop: ");
        Serial.print(radioData.estop);
        Serial.print(", mode: ");
        Serial.print(radioData.control_mode);
        Serial.println("]");
        
        // Send acknowledgment
        ackPayload.counter++;
        radio.writeAckPayload(1, &ackPayload, sizeof(AckPayloadStruct));
        
        lastDataReceived = millis();
    }
    
    // Simple connection status check
    if (millis() - lastDataReceived > 2000 && lastDataReceived != 0) {
        Serial.println("Warning: No data received for 2+ seconds");
        delay(1000);  // Prevent spam
    }
}