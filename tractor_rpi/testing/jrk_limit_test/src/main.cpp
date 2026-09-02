/*********************************************************************
  jrk_limit_test / main.cpp
  --------------------------------------------------------------
  Phase 1 (stationary, engine OFF) forward-limit investigation --
  step 1: confirm the JRK G2 21v3 "Current" variable (offset 0x19,
  milliamps) can be read reliably, while moving only through targets
  already field-validated on 2026-08-31 (2836 neutral down to 2288
  full forward, by default). This script does NOT probe below 2288
  unless you explicitly type a lower END_TARGET at the prompt.

  Run with: key ON, engine OFF, wheels chocked, blades irrelevant
  (nothing is turning). Two-person discipline still applies -- watch
  the actuator/linkage the whole time.

  STARTUP SAFETY -- what happens before you see the ready prompt:
    - Serial (USB) and Serial3 (JRK) are opened.
    - The JRK's Feedback variable is READ (0xE5 Get Variable -- a
      read-only query, not a motion command) and printed so you can
      see where the actuator actually is before anything moves.
    - NOTHING ELSE happens. No "exit safe start" byte and no "Set
      Target" command are sent to the JRK until you type 'y' at the
      ready prompt below. The JRK powers up in its own "Awaiting
      command" state and will not drive the motor on its own.
    - You will be asked to confirm the sweep parameters, then asked
      "Ready to begin? [y/N]" before the FIRST command that can
      cause any motion is sent. Typing 'q' at any prompt aborts
      cleanly -- no JRK write ever happens in that case.

  BEHAVIOR ONCE CONFIRMED:
    1. Sends 0x83 (exit safe start -- matches production
       teensy_main_20260804.cpp startup sequence), then commands
       START_TARGET.
    2. Steps from START_TARGET to END_TARGET in NUM_STEPS equal
       increments (NUM_STEPS + 1 commanded positions total,
       including both ends). Holds/samples at each step for the
       pause-between-steps duration you entered, printing target +
       measured current (mA) + feedback at 5 Hz.
    3. If you opted in at the prompt, returns to NEUTRAL (2836)
       and pauses there after every step, before moving to the
       next one.
    4. Returns to NEUTRAL at the end and idles there, continuing to
       print current at 5 Hz so you can watch the baseline for as
       long as you want. Reset/power-cycle the Teensy to run again.
*********************************************************************/

#include <Arduino.h>

// -------------------------------------------------------------------
// Fixed physical reference -- NOT prompted for. This is the actual
// neutral position (confirmed 2026-06-09), used for the optional
// "return to neutral between steps" behavior regardless of what
// START_TARGET is set to at runtime.
const uint16_t NEUTRAL_POSITION = 2836;

// Safety clamp -- refuse to command outside this range no matter
// what is entered at the prompts. Mirrors the bounds already used in
// jrk_actuator_test/jrk_range_explorer_20260609.cpp.
const uint16_t SAFETY_MIN_TARGET = 2200;   // forward limit
const uint16_t SAFETY_MAX_TARGET = 3600;   // reverse limit

const int MAX_STEPS = 50;  // upper bound on how many steps can be entered

// Sampling rate while holding at each step / at neutral. Not prompted
// for -- the request was to make start/end/steps/pause queryable.
const float SAMPLE_HZ = 5.0f;
const unsigned long SAMPLE_INTERVAL_MS = (unsigned long)(1000.0f / SAMPLE_HZ);
const unsigned long SETTLE_MS = 500;  // wait after commanding a target before first sample

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

// Sends a "Set Target" command -- the only function in this file that
// can cause the actuator to move. Every call site is guarded so this
// is never reached before the ready confirmation.
void setJrkTarget(uint16_t target) {
    if (target > 4095) target = 4095;
    Serial3.write(0xC0 + (target & 0x1F));
    Serial3.write((target >> 5) & 0x7F);
}

// Current variable: offset 0x19, 2 bytes, milliamps (Pololu Jrk G2
// User's Guide, Section 10, "Variable reference"). Read-only -- safe
// to call at any time, including before the ready confirmation.
bool readCurrentMa(uint16_t *outMa) {
    uint8_t bytes[2];
    if (!readJrkVariables(0x19, sizeof(bytes), bytes)) return false;
    *outMa = readU16LE(bytes);
    return true;
}

// Feedback: offset 0x04, 2 bytes. Also read-only.
bool readFeedback(uint16_t *outFb) {
    uint8_t bytes[2];
    if (!readJrkVariables(0x04, sizeof(bytes), bytes)) return false;
    *outFb = readU16LE(bytes);
    return true;
}

// -------------------------------------------------------------------
// Serial line-input helpers for the interactive prompts. The
// PlatformIO monitor does not locally echo what you type, so this
// echoes each character back over Serial as it's received.

String readSerialLine() {
    String line = "";
    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\r') continue;
            if (c == '\n') {
                Serial.println();
                break;
            }
            if (c == 8 || c == 127) {  // backspace / delete
                if (line.length() > 0) {
                    line.remove(line.length() - 1);
                    Serial.print("\b \b");
                }
                continue;
            }
            line += c;
            Serial.print(c);
        }
    }
    line.trim();
    return line;
}

long promptInt(const char *label, long defaultVal) {
    Serial.print(label);
    Serial.print(" [");
    Serial.print(defaultVal);
    Serial.print("]: ");
    String line = readSerialLine();
    if (line.length() == 0) return defaultVal;
    return line.toInt();
}

bool promptYesNo(const char *label, bool defaultYes) {
    while (true) {
        Serial.print(label);
        Serial.print(defaultYes ? " [Y/n]: " : " [y/N]: ");
        String line = readSerialLine();
        if (line.length() == 0) return defaultYes;
        char c = line.charAt(0);
        if (c == 'y' || c == 'Y') return true;
        if (c == 'n' || c == 'N') return false;
        if (c == 'q' || c == 'Q') {
            Serial.println("Aborted by user. No commands were sent to the JRK.");
            while (true) delay(1000);
        }
        Serial.println("  Please answer y or n (or q to abort).");
    }
}

// -------------------------------------------------------------------
uint16_t stepTargets[MAX_STEPS + 1];
int numSteps;
unsigned long pauseBetweenStepsMs;
bool returnToNeutralBetweenSteps;
uint16_t startTarget;
uint16_t endTarget;

void computeStepTargets() {
    for (int i = 0; i <= numSteps; i++) {
        if (i == numSteps) {
            stepTargets[i] = endTarget;
        } else {
            float f = (float)i / (float)numSteps;
            float t = (float)startTarget + f * ((float)endTarget - (float)startTarget);
            stepTargets[i] = (uint16_t)(t + 0.5f);
        }
    }
}

void printLine(unsigned long tMs, int stepIndex, uint16_t target,
               bool haveCurrent, uint16_t currentMa,
               bool haveFeedback, uint16_t feedback) {
    Serial.print(tMs); Serial.print(",");
    Serial.print(stepIndex); Serial.print(",");
    Serial.print(target); Serial.print(",");
    if (haveCurrent) Serial.print(currentMa); else Serial.print("TIMEOUT");
    Serial.print(",");
    if (haveFeedback) Serial.print(feedback); else Serial.print("TIMEOUT");
    Serial.println();
}

void holdAndSample(int stepIndex, uint16_t target, unsigned long durationMs) {
    unsigned long stepStart = millis();
    unsigned long lastSample = 0;
    bool firstSample = true;

    while (millis() - stepStart < durationMs) {
        if (millis() - stepStart < SETTLE_MS) continue;

        if (firstSample || millis() - lastSample >= SAMPLE_INTERVAL_MS) {
            uint16_t currentMa = 0, feedback = 0;
            bool haveCurrent = readCurrentMa(&currentMa);
            bool haveFeedback = readFeedback(&feedback);
            printLine(millis(), stepIndex, target, haveCurrent, currentMa, haveFeedback, feedback);
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
    delay(200);  // let the JRK finish its own power-up before we even read it

    Serial.println("==================================================");
    Serial.println("  jrk_limit_test -- Phase 1 current-read check");
    Serial.println("  Engine OFF, key ON, wheels chocked.");
    Serial.println("==================================================");
    Serial.println("  No command has been sent to the JRK yet -- only");
    Serial.println("  the serial port has been opened. Reading current");
    Serial.println("  actuator feedback (read-only, cannot move it):");

    uint16_t startupFeedback = 0;
    bool haveStartupFeedback = readFeedback(&startupFeedback);
    Serial.print("    Feedback = ");
    if (haveStartupFeedback) {
        Serial.print(startupFeedback);
        Serial.print("  (expected near ");
        Serial.print(NEUTRAL_POSITION);
        Serial.println(" = neutral, since the actuator should be there after shutdown)");
        if (startupFeedback < NEUTRAL_POSITION - 30 || startupFeedback > NEUTRAL_POSITION + 30) {
            Serial.println("    *** WARNING: feedback is not close to the expected neutral");
            Serial.println("    position. Verify the actuator's actual position before");
            Serial.println("    continuing -- do not assume this script's parameters are safe.");
        }
    } else {
        Serial.println("TIMEOUT -- no response from JRK. Check wiring/power before proceeding.");
    }
    Serial.println("  Still no target or exit-safe-start command sent.");
    Serial.println("==================================================");

    // ---- Prompts (still no JRK write commands issued) ----
    startTarget = (uint16_t)promptInt("Start position", 2836);
    endTarget = (uint16_t)promptInt("End position", 2288);
    numSteps = (int)promptInt("Number of steps", 5);
    long pauseSec = promptInt("Pause between steps (seconds)", 5);
    returnToNeutralBetweenSteps =
        promptYesNo("Return to NEUTRAL (2836) between each step?", true);

    if (numSteps < 1) numSteps = 1;
    if (numSteps > MAX_STEPS) {
        Serial.print("  Clamping number of steps to "); Serial.println(MAX_STEPS);
        numSteps = MAX_STEPS;
    }
    if (pauseSec < 1) pauseSec = 1;
    pauseBetweenStepsMs = (unsigned long)pauseSec * 1000UL;

    computeStepTargets();

    Serial.println("==================================================");
    Serial.println("  PLAN:");
    Serial.print("    Start target        = "); Serial.println(startTarget);
    Serial.print("    End target           = "); Serial.println(endTarget);
    Serial.print("    Number of steps      = "); Serial.println(numSteps);
    Serial.print("    Pause between steps  = "); Serial.print(pauseSec); Serial.println(" s");
    Serial.print("    Return to neutral between steps = ");
    Serial.println(returnToNeutralBetweenSteps ? "YES" : "no");
    Serial.print("    Computed step targets:");
    for (int i = 0; i <= numSteps; i++) {
        Serial.print("  "); Serial.print(stepTargets[i]);
    }
    Serial.println();

    bool safe = true;
    for (int i = 0; i <= numSteps; i++) {
        if (stepTargets[i] < SAFETY_MIN_TARGET || stepTargets[i] > SAFETY_MAX_TARGET) safe = false;
    }
    if (!safe) {
        Serial.print("  ABORT: a computed step target is outside the safety clamp (");
        Serial.print(SAFETY_MIN_TARGET); Serial.print(" - "); Serial.print(SAFETY_MAX_TARGET);
        Serial.println("). No command was sent. Reset and try different values.");
        while (true) delay(1000);
    }
    Serial.println("==================================================");

    // ---- Ready gate -- this is the only place the sweep can start ----
    bool ready = promptYesNo("Ready to begin? The actuator WILL move once confirmed.", false);
    if (!ready) {
        Serial.println("Not confirmed. No commands were sent to the JRK. Halting.");
        Serial.println("Reset the Teensy to be prompted again.");
        while (true) delay(1000);
    }

    // ---- First JRK write commands of this run, now that the operator confirmed ----
    Serial3.write(0x83);  // Exit safe start -- matches production teensy_main startup
    Serial3.flush();
    delay(100);

    Serial.println("t_ms,step,target,current_mA,feedback");

    for (int i = 0; i <= numSteps; i++) {
        Serial.print("=== STEP "); Serial.print(i); Serial.print("/"); Serial.print(numSteps);
        Serial.print("  target="); Serial.print(stepTargets[i]); Serial.println(" ===");
        setJrkTarget(stepTargets[i]);
        holdAndSample(i, stepTargets[i], pauseBetweenStepsMs);

        if (returnToNeutralBetweenSteps && stepTargets[i] != NEUTRAL_POSITION) {
            Serial.print("=== step "); Serial.print(i); Serial.println(": returning to NEUTRAL ===");
            setJrkTarget(NEUTRAL_POSITION);
            holdAndSample(i, NEUTRAL_POSITION, pauseBetweenStepsMs);
        }
    }

    Serial.println("=== SWEEP COMPLETE -- returning to NEUTRAL ===");
    setJrkTarget(NEUTRAL_POSITION);
    delay(2000);
    Serial.println("=== Holding NEUTRAL -- printing current at 5 Hz indefinitely.");
    Serial.println("=== Reset/power-cycle the Teensy to run again. ===");
}

void loop() {
    static unsigned long lastSample = 0;
    if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
        uint16_t currentMa = 0, feedback = 0;
        bool haveCurrent = readCurrentMa(&currentMa);
        bool haveFeedback = readFeedback(&feedback);
        printLine(millis(), -1, NEUTRAL_POSITION, haveCurrent, currentMa, haveFeedback, feedback);
        lastSample = millis();
    }
}
