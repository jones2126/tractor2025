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

// JRK G2 constants - Updated based on actual feedback range
#define JRK_BAUD 9600
const uint16_t transmissionFullReversePos = 312;   // Actual minimum feedback position
const uint16_t transmissionFullForwardPos = 3696;  // Actual maximum feedback position  
const uint16_t transmissionNeutralPos = 2048;      // Keep neutral in middle

// 10 buckets for smoother control (reverse to forward)
// transmission_val: 1023 = full reverse, 1 = full forward
const uint16_t bucketTargets[10] = {
    3696,  // Bucket 0: transmission_val ~1023 -> full forward
    3358,  // Bucket 1: transmission_val ~920
    3020,  // Bucket 2: transmission_val ~818
    2682,  // Bucket 3: transmission_val ~716
    2344,  // Bucket 4: transmission_val ~614
    2048,  // Bucket 5: transmission_val ~512 -> neutral
    1710,  // Bucket 6: transmission_val ~410
    1372,  // Bucket 7: transmission_val ~307
    1034,  // Bucket 8: transmission_val ~205
    312    // Bucket 9: transmission_val ~102-1 -> full reverse
};

// Timing variables
unsigned long currentMillis = 0;
unsigned long lastDataReceived = 0;
unsigned long lastJrkUpdate = 0;
const unsigned long jrkUpdateInterval = 100;  // 10 Hz (same as radio rate)
unsigned long lastStatusPrint = 0;
const unsigned long statusPrintInterval = 2000;  // Print status every 2 seconds

// Control variables
int bucket = 5;  // Start at middle bucket (neutral)
uint16_t currentJrkTarget = transmissionNeutralPos;
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
    Serial.println("Control mapping (Updated for actual actuator range):");
    Serial.println("  transmission_val 1023 -> bucket 0 -> JRK 3696 (FULL FORWARD)");
    Serial.println("  transmission_val ~512 -> bucket 5 -> JRK 2048 (NEUTRAL)");
    Serial.println("  transmission_val 1    -> bucket 9 -> JRK 312  (FULL REVERSE)");
    Serial.println("Format: transmission_val -> bucket -> JRK_target -> direction");
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
        
        // Calculate bucket (0-9) from transmission_val (1-1023)
        // transmission_val 1023 = bucket 0 (full forward)
        // transmission_val 1 = bucket 9 (full reverse)
        // Invert the mapping: higher transmission_val = lower bucket number
        bucket = constrain(9 - ((radioData.transmission_val - 1) / 102), 0, 9);
        
        // Determine target based on control mode
        uint16_t requestedTarget;
        switch (radioData.control_mode) {
            case 0:  // Emergency/Neutral mode
                requestedTarget = transmissionNeutralPos;
                break;
            case 1:  // Normal bucket mode
                requestedTarget = bucketTargets[bucket];
                break;
            default:
                requestedTarget = transmissionNeutralPos;
                break;
        }
        
        // Update current target
        currentJrkTarget = requestedTarget;
        
        // Print detailed status with feedback
        uint16_t feedback = readJrkFeedback();
        Serial.print("transmission_val: ");
        Serial.print(radioData.transmission_val, 1);
        Serial.print(" -> bucket: ");
        Serial.print(bucket);
        Serial.print(" -> JRK_target: ");
        Serial.print(requestedTarget);
        Serial.print(" -> feedback: ");
        Serial.print(feedback);
        Serial.print(" -> mode: ");
        Serial.print(radioData.control_mode);
        Serial.print(" -> estop: ");
        Serial.print(radioData.estop);
        
        // Add direction indicator for clarity
        if (requestedTarget > transmissionNeutralPos) {
            Serial.println(" (FORWARD)");
        } else if (requestedTarget < transmissionNeutralPos) {
            Serial.println(" (REVERSE)");
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
            targetToSend = transmissionNeutralPos;
            Serial.println("WARNING: Radio signal lost - sending neutral command");
        } else if (radioData.estop == 1) {
            // Emergency stop - go to neutral
            targetToSend = transmissionNeutralPos;
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