#include <Arduino.h>

// --- Configuration ---
const uint16_t positions[]       = {2951, 4095, 0};  // neutral, full reverse, full forward
const int      numPositions      = 3;
const unsigned long targetInterval   = 10000;  // ms between target changes
const unsigned long feedbackInterval =  2000;  // ms between feedback reads

// --- State ---
uint16_t       jrkTarget         = 0;
int            posIndex          = 0;
unsigned long  lastTargetTime    = 0;
unsigned long  lastFeedbackTime  = 0;
bool           firstRun          = true;

// --- JRK compact protocol ---
void setTargetCompact(uint16_t target) {
    if (target > 4095) target = 4095;
    Serial3.write(0xC0 + (target & 0x1F));
    Serial3.write((target >> 5) & 0x7F);
}

uint16_t readFeedback() {
    Serial3.write(0xE5);  // Command: Get variables
    Serial3.write(0x04);  // Offset: Feedback
    Serial3.write(0x02);  // Length: 2 bytes

    unsigned long start = millis();
    while (Serial3.available() < 2) {
        if (millis() - start > 100) {
            Serial.println("Timeout waiting for feedback");
            return 0xFFFF;
        }
    }
    uint8_t low  = Serial3.read();
    uint8_t high = Serial3.read();
    return (high << 8) | low;
}

void sendNextTarget() {
    jrkTarget = positions[posIndex];
    setTargetCompact(jrkTarget);
    Serial.print("Sent Target: ");
    Serial.print(jrkTarget);
    Serial.print("  (position ");
    Serial.print(posIndex);
    Serial.println(")");
    posIndex = (posIndex + 1) % numPositions;
    lastTargetTime = millis();
}

void setup() {
    Serial.begin(115200);
    Serial3.begin(9600);
    delay(2000);  // one-time startup only - not in loop
    Serial.println("JRK non-blocking test starting...");
    Serial.print("Target interval: ");  Serial.print(targetInterval / 1000); Serial.println("s");
    Serial.print("Feedback interval: "); Serial.print(feedbackInterval / 1000); Serial.println("s");
}

void loop() {
    unsigned long now = millis();

    // --- Send first target immediately on first pass ---
    if (firstRun) {
        sendNextTarget();
        firstRun = false;
        return;
    }

    // --- Send next target every 10 seconds ---
    if (now - lastTargetTime >= targetInterval) {
        sendNextTarget();
    }

    // --- Read and print feedback every 2 seconds ---
    if (now - lastFeedbackTime >= feedbackInterval) {
        uint16_t feedback = readFeedback();
        if (feedback != 0xFFFF) {
            Serial.print("  Feedback: ");
            Serial.print(feedback);
            Serial.print("  Target: ");
            Serial.println(jrkTarget);
        }
        lastFeedbackTime = now;
    }
}
