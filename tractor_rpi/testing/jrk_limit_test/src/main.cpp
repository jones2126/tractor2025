/*********************************************************************
  jrk_limit_test / main.cpp
  --------------------------------------------------------------
  Phase 1 (stationary, engine OFF) forward-limit investigation --
  step 1: confirm the JRK G2 21v3 "Current" variable (offset 0x19,
  milliamps) can be read reliably, while moving only through targets
  already field-validated on 2026-08-31 (2836 neutral down to 2288
  full forward). This script does NOT probe beyond 2288 -- it is a
  read-current sanity check on known-safe PWM/JRK targets before any
  attempt to find the mechanical limit below 2288.

  Run with: key ON, engine OFF, wheels chocked, blades irrelevant
  (nothing is turning). Two-person discipline still applies per the
  Phase 1 protocol -- watch the actuator/linkage the whole time.

  BEHAVIOR
    1. Commands NEUTRAL, exits JRK safe-start (matches production
       teensy_main_20260804.cpp startup sequence).
    2. Steps from START_TARGET to END_TARGET in NUM_STEPS equal
       increments (6 commanded positions total, including both
       ends). Holds at each step, printing target + measured
       current (mA) at SAMPLE_HZ.
    3. Returns to NEUTRAL and idles there, continuing to print
       current at SAMPLE_HZ so you can watch the neutral baseline
       for as long as you want. Power-cycle/reset the Teensy to
       run the sweep again.

  PARAMETERS (edit these three, then recompile -- this is the
  "starting / ending / increment-count" the sweep is built from):
*********************************************************************/

#include <Arduino.h>

// -------------------------------------------------------------------
// Sweep parameters -- edit and recompile to change the test.
const uint16_t START_TARGET = 2836;   // neutral -- confirmed 2026-06-09
const uint16_t END_TARGET   = 2288;   // full forward -- field-validated 2026-08-31
const uint8_t  NUM_STEPS    = 5;      // number of increments from START to END
                                       // (produces NUM_STEPS + 1 commanded positions)

// Sampling
const float SAMPLE_HZ         = 5.0f;
const unsigned long SAMPLE_INTERVAL_MS = (unsigned long)(1000.0f / SAMPLE_HZ);
const unsigned long SETTLE_MS          = 500;   // wait after commanding a new target before sampling
const unsigned long STEP_HOLD_MS       = 4000;  // total time spent at each step, including settle

// Safety clamp -- refuse to command outside this range regardless of
// the parameters above. Mirrors the bounds already used in
// jrk_actuator_test/jrk_range_explorer_20260609.cpp.
const uint16_t SAFETY_MIN_TARGET = 2200;   // forward limit
const uint16_t SAFETY_MAX_TARGET = 3600;   // reverse limit

#define JRK_BAUD 9600

// -------------------------------------------------------------------
// JRK G2 serial helpers -- mirrors readJrkVariables()/readU16LE()/
// setJrkTarget() in tractor_teensy/src/teensy_main_20260804.cpp so
// this current-read logic can be dropped straight into production
// once it's verified here.

bool readJrkVariables(uint8_t offset, uint8_t length, uint8_t *buffer) {
    if (length == 0 || length > 15) return false;

    while (Serial3.available() > 0) Serial3.read();  // discard stale bytes

    unsigned long start = millis();
    Serial3.write(0xE5);
    Serial3.write(offset);
    Serial3.write(length);
    Serial3.flush();

    while (Serial3.available() < length) {
        if (millis() - start > 30) return false;  // timeout
    }
    for (uint8_t i = 0; i < length; i++) buffer[i] = Serial3.read();
    return true;
}

uint16_t readU16LE(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

void setJrkTarget(uint16_t target) {
    if (target > 4095) target = 4095;
    Serial3.write(0xC0 + (target & 0x1F));
    Serial3.write((target >> 5) & 0x7F);
}

// Current variable: offset 0x19, 2 bytes, milliamps (Pololu Jrk G2
// User's Guide, Section 10, "Variable reference").
bool readCurrentMa(uint16_t *outMa) {
    uint8_t bytes[2];
    if (!readJrkVariables(0x19, sizeof(bytes), bytes)) return false;
    *outMa = readU16LE(bytes);
    return true;
}

// Feedback: offset 0x04, 2 bytes -- printed alongside current so each
// line is self-describing (which commanded step it belongs to).
bool readFeedback(uint16_t *outFb) {
    uint8_t bytes[2];
    if (!readJrkVariables(0x04, sizeof(bytes), bytes)) return false;
    *outFb = readU16LE(bytes);
    return true;
}

// -------------------------------------------------------------------
uint16_t stepTargets[NUM_STEPS + 1];

void computeStepTargets() {
    // Linear interpolation from START_TARGET to END_TARGET in
    // NUM_STEPS increments. Rounds each intermediate point but the
    // last point is always forced to exactly END_TARGET.
    for (uint8_t i = 0; i <= NUM_STEPS; i++) {
        if (i == NUM_STEPS) {
            stepTargets[i] = END_TARGET;
        } else {
            float f = (float)i / (float)NUM_STEPS;
            float t = (float)START_TARGET +
                      f * ((float)END_TARGET - (float)START_TARGET);
            stepTargets[i] = (uint16_t)(t + 0.5f);
        }
    }
}

void printLine(unsigned long tMs, int stepIndex, uint16_t target,
               bool haveCurrent, uint16_t currentMa,
               bool haveFeedback, uint16_t feedback) {
    Serial.print(tMs);
    Serial.print(",");
    Serial.print(stepIndex);
    Serial.print(",");
    Serial.print(target);
    Serial.print(",");
    if (haveCurrent) Serial.print(currentMa); else Serial.print("TIMEOUT");
    Serial.print(",");
    if (haveFeedback) Serial.print(feedback); else Serial.print("TIMEOUT");
    Serial.println();
}

void holdAndSample(int stepIndex, uint16_t target) {
    unsigned long stepStart = millis();
    unsigned long lastSample = 0;
    bool firstSample = true;

    while (millis() - stepStart < STEP_HOLD_MS) {
        // Wait out the settle period before the first sample.
        if (millis() - stepStart < SETTLE_MS) continue;

        if (firstSample || millis() - lastSample >= SAMPLE_INTERVAL_MS) {
            uint16_t currentMa = 0, feedback = 0;
            bool haveCurrent = readCurrentMa(&currentMa);
            bool haveFeedback = readFeedback(&feedback);
            printLine(millis(), stepIndex, target,
                      haveCurrent, currentMa, haveFeedback, feedback);
            lastSample = millis();
            firstSample = false;
        }
    }
}

// -------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);  // wait for platformio monitor, but don't hang forever

    Serial3.begin(JRK_BAUD);
    delay(100);
    Serial3.write(0x83);  // Exit safe start -- matches production teensy_main startup
    Serial3.flush();
    delay(100);

    computeStepTargets();

    Serial.println("==================================================");
    Serial.println("  jrk_limit_test -- Phase 1 current-read check");
    Serial.println("  Engine OFF, key ON, wheels chocked. Known-safe");
    Serial.println("  targets only (2836 -> 2288). No mechanical-limit");
    Serial.println("  probing in this script.");
    Serial.println("==================================================");
    Serial.print("  START_TARGET = "); Serial.println(START_TARGET);
    Serial.print("  END_TARGET   = "); Serial.println(END_TARGET);
    Serial.print("  NUM_STEPS    = "); Serial.println(NUM_STEPS);
    Serial.print("  SAMPLE_HZ    = "); Serial.println(SAMPLE_HZ);
    Serial.print("  STEP_HOLD_MS = "); Serial.println(STEP_HOLD_MS);
    Serial.print("  Computed step targets:");
    for (uint8_t i = 0; i <= NUM_STEPS; i++) {
        Serial.print("  "); Serial.print(stepTargets[i]);
    }
    Serial.println();
    Serial.println("==================================================");

    for (uint8_t i = 0; i <= NUM_STEPS; i++) {
        if (stepTargets[i] < SAFETY_MIN_TARGET || stepTargets[i] > SAFETY_MAX_TARGET) {
            Serial.println("  ABORT: a computed step target is outside the safety clamp");
            Serial.print("  ("); Serial.print(SAFETY_MIN_TARGET);
            Serial.print(" - "); Serial.print(SAFETY_MAX_TARGET);
            Serial.println("). Check START_TARGET/END_TARGET/NUM_STEPS.");
            while (true) delay(1000);  // halt
        }
    }

    Serial.println("  Commanding NEUTRAL and settling for 2 s before starting...");
    setJrkTarget(START_TARGET);
    delay(2000);

    Serial.println("t_ms,step,target,current_mA,feedback");

    for (uint8_t i = 0; i <= NUM_STEPS; i++) {
        Serial.print("=== STEP "); Serial.print(i); Serial.print("/"); Serial.print(NUM_STEPS);
        Serial.print("  target="); Serial.print(stepTargets[i]); Serial.println(" ===");
        setJrkTarget(stepTargets[i]);
        holdAndSample(i, stepTargets[i]);
    }

    Serial.println("=== SWEEP COMPLETE -- returning to NEUTRAL ===");
    setJrkTarget(START_TARGET);
    delay(2000);
    Serial.println("=== Holding NEUTRAL -- printing current at SAMPLE_HZ indefinitely.");
    Serial.println("=== Reset/power-cycle the Teensy to run the sweep again. ===");
}

void loop() {
    static unsigned long lastSample = 0;
    if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
        uint16_t currentMa = 0, feedback = 0;
        bool haveCurrent = readCurrentMa(&currentMa);
        bool haveFeedback = readFeedback(&feedback);
        printLine(millis(), -1, START_TARGET, haveCurrent, currentMa, haveFeedback, feedback);
        lastSample = millis();
    }
}
