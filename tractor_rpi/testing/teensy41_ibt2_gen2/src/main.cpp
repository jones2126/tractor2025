// teensy41_ibt2_gen2
// IBT-2 (BTS7960B) Next Generation test — NOT integrated into tractor firmware yet.
//
// Demonstrates full 6-pin IBT-2 control:
//   - RPWM / LPWM : PWM drive (existing, pins 5 & 6)
//   - R_EN / L_EN : enable control via open-drain + 10kΩ pull-up to 5V (pins 7 & 8)
//   - R_IS / L_IS : current sense via 10kΩ/10kΩ voltage divider (pins A7=21 & A8=22)
//
// Wiring required before use:
//   R_EN (pin 7) and L_EN (pin 8): each with 10kΩ pull-up to 5V (open-drain)
//   R_IS (pin 21) and L_IS (pin 22): each with 10kΩ/10kΩ voltage divider to GND
//
// 3.3V safety: EN pins only driven LOW (never HIGH to 5V)
//              IS pins clamped to ~2.5V max via divider
//
// Test sequence: ramp right, hold, detect stall, auto-recover, ramp left, repeat.

#include <Arduino.h>

// -------------------------------------------------------------------
// Pin definitions

#define RPWM_PIN      5    // IBT-2 right PWM
#define LPWM_PIN      6    // IBT-2 left PWM
#define R_EN_PIN      7    // IBT-2 right enable (open-drain, 10kΩ pull-up to 5V)
#define L_EN_PIN      8    // IBT-2 left enable  (open-drain, 10kΩ pull-up to 5V)
#define R_IS_PIN      A7   // IBT-2 right current sense (10kΩ/10kΩ divider)
#define L_IS_PIN      A8   // IBT-2 left current sense  (10kΩ/10kΩ divider)
#define STEER_POT_PIN A9   // Steering potentiometer

// -------------------------------------------------------------------
// IBT-2 configuration

#define PWM_TEST       150  // Test PWM value (confirmed moves wheel when stationary)
#define STALL_POT_THRESHOLD   3    // pot counts — below this = considered stalled
#define STALL_TIME_MS      2000    // ms before declaring stall
#define IS_STALL_THRESHOLD  512    // ADC value (~1.65V after divider) — tune from field data
#define FAULT_RECOVERY_MS   150    // ms to hold EN LOW when clearing fault

// -------------------------------------------------------------------
// State

int    lastPot          = 0;
unsigned long stallStart = 0;
bool   stalledFlag       = false;
bool   ibt2Enabled       = false;

// -------------------------------------------------------------------
// IBT-2 enable control (open-drain)
// HIGH → release pin → pull-up takes EN to 5V → IBT-2 enabled
// LOW  → sink pin to GND → EN = 0V → IBT-2 disabled / fault cleared

void ibt2Enable() {
    // Release both EN pins — pull-ups take them to 5V
    pinMode(R_EN_PIN, INPUT);
    pinMode(L_EN_PIN, INPUT);
    ibt2Enabled = true;
    Serial.println("IBT-2: ENABLED (EN pins released to 5V pull-up)");
}

void ibt2Disable() {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
    pinMode(R_EN_PIN, OUTPUT); digitalWrite(R_EN_PIN, LOW);
    pinMode(L_EN_PIN, OUTPUT); digitalWrite(L_EN_PIN, LOW);
    ibt2Enabled = false;
    Serial.println("IBT-2: DISABLED (EN pins pulled LOW)");
}

void ibt2ClearFault() {
    // Cycle EN LOW → wait → release → clears BTS7960 overcurrent latch
    Serial.println("IBT-2: clearing fault latch...");
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
    pinMode(R_EN_PIN, OUTPUT); digitalWrite(R_EN_PIN, LOW);
    pinMode(L_EN_PIN, OUTPUT); digitalWrite(L_EN_PIN, LOW);
    delay(FAULT_RECOVERY_MS);
    ibt2Enable();
    Serial.println("IBT-2: fault cleared, re-enabled");
}

// -------------------------------------------------------------------
// Current sense reading
// Returns actual IS voltage estimate (before divider) in volts
// Divider halves the voltage, so multiply ADC reading back by 2

float readCurrentSense(int pin) {
    int raw = analogRead(pin);
    float vDivided = (raw / 1023.0f) * 3.3f;   // voltage at Teensy pin
    float vActual  = vDivided * 2.0f;            // actual IS voltage (before divider)
    return vActual;
}

// -------------------------------------------------------------------
// Stall detection
// Returns true if motor appears stalled (pot not moving while PWM applied)

bool checkStall(int currentPot, int pwm) {
    if (pwm == 0) {
        stallStart = 0;
        stalledFlag = false;
        return false;
    }

    if (abs(currentPot - lastPot) > STALL_POT_THRESHOLD) {
        // Pot is moving — reset stall timer
        stallStart  = millis();
        stalledFlag = false;
    } else if (stallStart == 0) {
        stallStart = millis();
    } else if (millis() - stallStart > STALL_TIME_MS) {
        stalledFlag = true;
    }

    lastPot = currentPot;
    return stalledFlag;
}

// -------------------------------------------------------------------
// Setup

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000);
    Serial.println("=== IBT-2 Gen2 Test ===");
    Serial.println("Pins: RPWM=5 LPWM=6 R_EN=7 L_EN=8 R_IS=A7 L_IS=A8 POT=A9");
    Serial.println("Wiring required: 10kΩ pull-ups on EN, 10kΩ/10kΩ dividers on IS");
    Serial.println("");

    pinMode(RPWM_PIN, OUTPUT);
    pinMode(LPWM_PIN, OUTPUT);
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);

    analogReadResolution(10);   // 0–1023

    // Start with IBT-2 enabled
    ibt2Enable();
    delay(500);

    lastPot    = analogRead(STEER_POT_PIN);
    stallStart = millis();

    Serial.println("Starting test in 2 seconds...");
    delay(2000);
}

// -------------------------------------------------------------------
// Main loop: ramp right → stall check → recover → ramp left → repeat

enum TestState { MOVING_RIGHT, MOVING_LEFT, RECOVERING };
TestState state    = MOVING_RIGHT;
unsigned long stateStart = 0;
const unsigned long MOVE_DURATION_MS = 4000;  // run each direction 4 seconds

void loop() {
    unsigned long now = millis();
    int pot    = analogRead(STEER_POT_PIN);
    float rIS  = readCurrentSense(R_IS_PIN);
    float lIS  = readCurrentSense(L_IS_PIN);
    int   pwm  = 0;

    // --- State machine ---
    switch (state) {
        case MOVING_RIGHT:
            pwm = PWM_TEST;
            analogWrite(LPWM_PIN, 0);
            analogWrite(RPWM_PIN, pwm);

            if (checkStall(pot, pwm)) {
                Serial.println("STALL detected (right) — recovering");
                ibt2ClearFault();
                stalledFlag = false;
                stallStart  = 0;
                state       = RECOVERING;
                stateStart  = now;
            } else if (now - stateStart > MOVE_DURATION_MS) {
                analogWrite(RPWM_PIN, 0);
                Serial.println("--- switching to LEFT ---");
                state      = MOVING_LEFT;
                stateStart = now;
                stallStart = 0;
            }
            break;

        case MOVING_LEFT:
            pwm = PWM_TEST;
            analogWrite(RPWM_PIN, 0);
            analogWrite(LPWM_PIN, pwm);

            if (checkStall(pot, pwm)) {
                Serial.println("STALL detected (left) — recovering");
                ibt2ClearFault();
                stalledFlag = false;
                stallStart  = 0;
                state       = RECOVERING;
                stateStart  = now;
            } else if (now - stateStart > MOVE_DURATION_MS) {
                analogWrite(LPWM_PIN, 0);
                Serial.println("--- switching to RIGHT ---");
                state      = MOVING_RIGHT;
                stateStart = now;
                stallStart = 0;
            }
            break;

        case RECOVERING:
            // Brief pause after fault recovery before restarting
            analogWrite(RPWM_PIN, 0);
            analogWrite(LPWM_PIN, 0);
            if (now - stateStart > 1000) {
                Serial.println("--- resuming RIGHT after recovery ---");
                state      = MOVING_RIGHT;
                stateStart = now;
                stallStart = 0;
            }
            break;
    }

    // --- Status print every second ---
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 1000) {
        Serial.print("pot="); Serial.print(pot);
        Serial.print("  R_IS="); Serial.print(rIS, 2); Serial.print("V");
        Serial.print("  L_IS="); Serial.print(lIS, 2); Serial.print("V");
        Serial.print("  pwm="); Serial.print(pwm);
        Serial.print("  state=");
        Serial.print(state == MOVING_RIGHT ? "RIGHT" : state == MOVING_LEFT ? "LEFT" : "RECOVER");
        Serial.print("  stalled="); Serial.println(stalledFlag ? "YES" : "no");
        lastPrint = now;
    }
}
