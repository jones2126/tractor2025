// jrk_read_position.cpp
// Read-only: reads and prints the current JRK G2 actuator position every second.
// Does NOT send any target commands — safe to run with actuator connected.
// Use this to determine the true neutral position of the transmission.
//
// Expected neutral candidates: 2985 (firmware default) or ~2800 (field estimate)
// Serial3 = JRK G2 at 9600 baud

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
    Serial3.begin(9600);
    delay(500);
    Serial.println("=== JRK G2 Position Reader ===");
    Serial.println("No commands sent — read only.");
    Serial.println("Neutral candidates: firmware=2985, field estimate=~2800");
    Serial.println("==============================");
}

uint16_t readFeedback() {
    // Flush any stale bytes first
    while (Serial3.available()) Serial3.read();

    Serial3.write(0xE5);  // Get variables command
    Serial3.write(0x04);  // Offset: feedback
    Serial3.write(0x02);  // Length: 2 bytes

    unsigned long start = millis();
    while (Serial3.available() < 2) {
        if (millis() - start > 100) {
            return 0xFFFF;  // timeout
        }
    }
    uint8_t lo = Serial3.read();
    uint8_t hi = Serial3.read();
    return (hi << 8) | lo;
}

void loop() {
    uint16_t pos = readFeedback();

    if (pos == 0xFFFF) {
        Serial.println("ERROR: No response from JRK — check wiring and baud rate");
    } else {
        Serial.print("JRK position: ");
        Serial.print(pos);

        // Annotate relative to known candidates
        if (pos >= 2950 && pos <= 3020) {
            Serial.print("  <-- near firmware neutral (2985)");
        } else if (pos >= 2750 && pos <= 2850) {
            Serial.print("  <-- near field estimate neutral (~2800)");
        } else if (pos > 3020) {
            Serial.print("  (reverse side of neutral)");
        } else if (pos < 2750) {
            Serial.print("  (forward side of neutral)");
        }
        Serial.println();
    }

    delay(1000);
}
