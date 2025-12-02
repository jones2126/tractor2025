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

// Add feedback reading function for debugging
uint16_t readJrkFeedback() {
    Serial3.write(0xE5);  // Command: Get variables
    Serial3.write(0x04);  // Offset for Feedback
    Serial3.write(0x02);  // Length = 2 bytes

    unsigned long start = millis();
    while (Serial3.available() < 2) {
        if (millis() - start > 100) {
            return 0xFFFF;  // Error code - don't print timeout in normal operation
        }
    }

    uint8_t low = Serial3.read();
    uint8_t high = Serial3.read();
    return (high << 8) | low;
}

RadioControlStruct radioData;
AckPayloadStruct ackPayload;

const uint8_t address[][6] = {"RCTRL", "TRACT"};

// JRK G2 constants - Full range mapping
#define JRK_BAUD 9600
const uint16_t JRK_MIN = 0;      // JRK minimum position
const uint16_t JRK_MAX = 4095;   // JRK maximum position
const uint16_t JRK_NEUTRAL = 2048; // JRK neutral position

// Timing variables
unsigned long currentMillis = 0;
unsigned long lastDataReceived = 0;
unsigned long lastJrkUpdate = 0;
const unsigned long jrkUpdateInterval = 100;  // 10 Hz (same as radio rate)
unsigned long lastStatusPrint = 0;
const unsigned long statusPrintInterval = 2000;  // Print status every 2 seconds

// Control variables
uint16_t currentJrkTarget = JRK_NEUTRAL;
float smoothedTransmissionVal = 512.0;  // Start at middle
const float SMOOTHING_FACTOR = 0.1;     // Lower = more smoothing (0.05-0.2 range)
const uint16_t MIN_CHANGE_THRESHOLD = 10; // Only update if change is significant
bool radioSignalGood = false;
const unsigned long radioTimeoutMs = 500;  // Consider signal lost after 500ms

// Send a target position to the JRK controller (using working compact method)
void setJrkTarget(uint16_t target) {
    if (target > 4095) target = 4095;  // Safety limit
    Serial3.write(0xC0 + (target & 0x1F));
    Serial3.write((target >> 5) & 0x7F);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);  // Wait up to 5 seconds for serial
    
    Serial.println("Integrated NRF24 + JRK G2 Controller Starting...");

    // Initialize JRK serial communication
    Serial3.begin(JRK_BAUD);
    Serial.println("JRK G2 serial initialized");

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
        Serial.println("ERROR: Radio hardware not responding!");
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

    // Set initial JRK target to neutral
    setJrkTarget(currentJrkTarget);
    Serial.print("JRK initialized to neutral position: ");
    Serial.println(currentJrkTarget);

    Serial.println("Setup complete - listening for data...");
    Serial.println("Linear mapping with smoothing:");
    Serial.println("  transmission_val 1023 -> JRK 0    (CCW - Full position A)");
    Serial.println("  transmission_val ~512 -> JRK 2048 (Middle - Neutral)");
    Serial.println("  transmission_val 1    -> JRK 4095 (CW - Full position B)");
    Serial.print("  Smoothing factor: ");
    Serial.print(SMOOTHING_FACTOR);
    Serial.print(", Min change threshold: ");
    Serial.println(MIN_CHANGE_THRESHOLD);
    Serial.println("Format: raw -> smoothed -> JRK_target -> feedback");
}

void updateRadioStatus() {
    // Check if radio signal is good based on recent data
    radioSignalGood = (currentMillis - lastDataReceived) < radioTimeoutMs;
}

void processRadioData() {
    // Check for incoming data
    if (radio.available()) {
        // Read the data
        radio.read(&radioData, sizeof(RadioControlStruct));
        
        // Apply smoothing filter to reduce noise
        smoothedTransmissionVal = (SMOOTHING_FACTOR * radioData.transmission_val) + 
                                 ((1.0 - SMOOTHING_FACTOR) * smoothedTransmissionVal);
        
        // Direct linear mapping using smoothed value
        // transmission_val 1023 (CCW) -> JRK 0
        // transmission_val 1 (CW) -> JRK 4095
        uint16_t requestedTarget = map((int)smoothedTransmissionVal, 1, 1023, JRK_MAX, JRK_MIN);
        
        // Only update if the change is significant (reduces jitter)
        if (abs((int)requestedTarget - (int)currentJrkTarget) < MIN_CHANGE_THRESHOLD && 
            radioData.control_mode == 1) {
            requestedTarget = currentJrkTarget; // Keep current target
        }
        
        // Apply control mode logic
        switch (radioData.control_mode) {
            case 0:  // Emergency/Neutral mode
                requestedTarget = JRK_NEUTRAL;
                break;
            case 1:  // Normal linear mapping mode
                // requestedTarget already calculated above
                break;
            default:
                requestedTarget = JRK_NEUTRAL;
                break;
        }
        
        // Update current target
        currentJrkTarget = requestedTarget;
        
        // Print detailed status with smoothing info
        uint16_t feedback = readJrkFeedback();
        Serial.print("raw: ");
        Serial.print(radioData.transmission_val, 0);
        Serial.print(" -> smoothed: ");
        Serial.print(smoothedTransmissionVal, 1);
        Serial.print(" -> JRK_target: ");
        Serial.print(requestedTarget);
        Serial.print(" -> feedback: ");
        Serial.print(feedback);
        Serial.print(" -> mode: ");
        Serial.print(radioData.control_mode);
        Serial.print(" -> estop: ");
        Serial.print(radioData.estop);
        
        // Add direction indicator for clarity
        if (requestedTarget > JRK_NEUTRAL) {
            Serial.println(" (POSITION B)");
        } else if (requestedTarget < JRK_NEUTRAL) {
            Serial.println(" (POSITION A)");
        } else {
            Serial.println(" (NEUTRAL)");
        }
        
        // Send acknowledgment
        ackPayload.counter++;
        radio.writeAckPayload(1, &ackPayload, sizeof(AckPayloadStruct));
        
        lastDataReceived = currentMillis;
    }
}

void updateJrkController() {
    // Send JRK commands at 10 Hz
    if (currentMillis - lastJrkUpdate >= jrkUpdateInterval) {
        
        uint16_t targetToSend;
        
        if (!radioSignalGood) {
            // Radio signal lost - go to neutral for safety
            targetToSend = JRK_NEUTRAL;
            Serial.println("WARNING: Radio signal lost - sending neutral command");
        } else if (radioData.estop == 1) {
            // Emergency stop - go to neutral
            targetToSend = JRK_NEUTRAL;
        } else {
            // Normal operation
            targetToSend = currentJrkTarget;
        }
        
        // Send command to JRK controller
        setJrkTarget(targetToSend);
        
        lastJrkUpdate = currentMillis;
    }
}

void printStatus() {
    // Print periodic status
    if (currentMillis - lastStatusPrint >= statusPrintInterval) {
        Serial.print("STATUS - Radio: ");
        Serial.print(radioSignalGood ? "GOOD" : "LOST");
        Serial.print(", Current target: ");
        Serial.print(currentJrkTarget);
        Serial.print(", ACK count: ");
        Serial.println(ackPayload.counter);
        
        lastStatusPrint = currentMillis;
    }
}

void loop() {
    currentMillis = millis();
    
    updateRadioStatus();
    processRadioData();
    updateJrkController();
    printStatus();
    
    // Small delay to prevent overwhelming the system
    delay(1);
}