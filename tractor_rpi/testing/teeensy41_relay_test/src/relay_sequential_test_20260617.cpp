/*
  relay_sequential_test_20260617.cpp
  ====================================
  Sequential 4-Port Relay Test for Teensy 4.1
  Pins tested: 29, 30, 31, 32

  PURPOSE:
    Fire each relay pin ONE AT A TIME so you can watch the relay board
    and determine which Teensy pin maps to which relay channel (IN1-IN4).
    Record which relay channel (IN1/IN2/IN3/IN4) lights up for each pin,
    then update ESTOP_RELAY_PIN in production firmware accordingly.

  BEHAVIOR:
    - All relay pins start HIGH (relays OFF — active LOW board)
    - Each pin is pulled LOW (relay ON) for 5 seconds, then HIGH (relay OFF)
    - 2 second pause between each pin
    - Serial monitor prints which pin is active so you can correlate
    - After all 4 pins tested, waits 5 seconds and repeats

  WIRING ASSUMPTION:
    Relay board is active LOW (standard 4-port relay boards):
      HIGH = relay OFF
      LOW  = relay ON (coil energized, LED illuminated)

  HOW TO USE:
    1. Flash this sketch to Teensy 4.1
    2. Open Serial Monitor at 115200 baud
    3. Watch the relay board LEDs as each pin fires sequentially
    4. Note which relay channel (IN1/IN2/IN3/IN4) lights for each pin
    5. Find the pin that lights IN2 — that is your ESTOP_RELAY_PIN
    6. Update #define ESTOP_RELAY_PIN in teensy_main_20260609.cpp
    7. Reflash production firmware

  EXPECTED OUTPUT (Serial Monitor):
    === Relay Sequential Test - Cycle 1 ===
    All relays OFF (pins HIGH)
    --- Testing Pin 29 (Relay position TBD) ---
    Pin 29 ON (LOW) - watch which relay channel lights up
    ON - Second 1/5 | Pin 29 ACTIVE
    ...
    Pin 29 OFF. Note which channel was lit.
    --- Testing Pin 30 ...
*/

#include <Arduino.h>

// All four relay pins wired to the 4-port relay board
const int relayPins[] = {29, 30, 31, 32};
const int numRelays = 4;
const int ON_DURATION_SEC  = 5;   // seconds each relay stays ON
const int PAUSE_BETWEEN_SEC = 2;  // seconds between relays (all OFF)

// -----------------------------------------------------------------------
void allRelaysOff() {
    for (int i = 0; i < numRelays; i++) {
        digitalWrite(relayPins[i], HIGH);  // HIGH = OFF on active-LOW board
    }
}

// -----------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}  // wait up to 3s for USB serial

    // Initialize all relay pins as outputs, start OFF
    for (int i = 0; i < numRelays; i++) {
        pinMode(relayPins[i], OUTPUT);
    }
    allRelaysOff();

    Serial.println();
    Serial.println("=============================================");
    Serial.println("  Teensy 4.1 - Sequential Relay Test");
    Serial.println("  relay_sequential_test_20260617.cpp");
    Serial.println("=============================================");
    Serial.println("  Pins under test: 29, 30, 31, 32");
    Serial.println("  Each pin fires LOW (ON) for 5 seconds.");
    Serial.println("  Watch relay board: note which channel");
    Serial.println("  (IN1/IN2/IN3/IN4) lights for each pin.");
    Serial.println("  Goal: find which pin controls IN2.");
    Serial.println("=============================================");
    Serial.println();

    delay(2000);  // brief pause before first cycle
}

// -----------------------------------------------------------------------
void loop() {
    static int cycle = 1;

    Serial.print("=== Relay Sequential Test - Cycle ");
    Serial.print(cycle);
    Serial.println(" ===");
    Serial.println("All relays OFF (pins HIGH)");
    allRelaysOff();
    delay(1000);

    for (int i = 0; i < numRelays; i++) {
        int pin = relayPins[i];

        Serial.println();
        Serial.print("--- Testing Pin ");
        Serial.print(pin);
        Serial.println(" ---");
        Serial.print("Pin ");
        Serial.print(pin);
        Serial.println(" ON (LOW) - watch which relay channel lights up");

        digitalWrite(pin, LOW);  // ON

        for (int s = 1; s <= ON_DURATION_SEC; s++) {
            Serial.print("  ON - Second ");
            Serial.print(s);
            Serial.print("/");
            Serial.print(ON_DURATION_SEC);
            Serial.print(" | Pin ");
            Serial.print(pin);
            Serial.println(" ACTIVE");
            delay(1000);
        }

        digitalWrite(pin, HIGH);  // OFF
        Serial.print("Pin ");
        Serial.print(pin);
        Serial.println(" OFF. Note which relay channel was lit.");

        if (i < numRelays - 1) {
            Serial.print("Pausing ");
            Serial.print(PAUSE_BETWEEN_SEC);
            Serial.println("s before next pin...");
            delay(PAUSE_BETWEEN_SEC * 1000);
        }
    }

    Serial.println();
    Serial.println("--- All 4 pins tested. ---");
    Serial.println("Record your observations, then reflash production firmware.");
    Serial.println("Repeating in 5 seconds...");
    Serial.println();
    delay(5000);

    cycle++;
}
