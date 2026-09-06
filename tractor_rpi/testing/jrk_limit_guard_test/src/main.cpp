/*********************************************************************
  JRK forward-limit data test with fail-safe return to neutral.

  This is a separate, temporary Teensy 4.1 test.  It does not modify
  JRK settings.  It commands one target at a time, records the JRK's
  current and feedback, and advances in small target increments without
  returning to neutral between successful probes.

  Probe cutoff behavior:
    - Immediately command neutral (2836).
    - Stop the entire sweep; never advance to another target.
    - Supervise the return to neutral with the same protections.
    - If the return itself cannot make progress, send Stop Motor (0xFF).

  Run only with engine OFF, tractor secured against movement, and an
  operator watching the actuator/linkage with a physical power cutoff
  immediately available.
*********************************************************************/

#include <Arduino.h>

constexpr uint32_t JRK_BAUD = 9600;
constexpr uint16_t NEUTRAL_TARGET = 2836;
constexpr uint16_t FIRST_PROBE_TARGET = NEUTRAL_TARGET - 20;
constexpr uint16_t LOWEST_ALLOWED_TARGET = 2200;
constexpr uint16_t TARGET_TOLERANCE = 10;

// These are thresholds in the JRK's reported milliamps/counts.  The
// JRK configuration is not changed by this test.
constexpr uint16_t HIGH_CURRENT_MA = 3000;
// Two guarded runs measured normal, moving startup transients above 3.8 A,
// so 3.5 A cannot be used as a current-only cutoff.  This ceiling remains
// below 5 A; the primary stall cutoff is current PLUS lack of progress.
constexpr uint16_t ABSOLUTE_CURRENT_MA = 4750;
constexpr uint16_t PROGRESS_COUNTS = 2;
constexpr uint32_t HIGH_CURRENT_NO_PROGRESS_MS = 150;
constexpr uint32_t NO_PROGRESS_MS = 250;
constexpr uint32_t MAX_PROBE_MOVE_MS = 750;
constexpr uint32_t MAX_RETURN_MOVE_MS = 2000;
constexpr uint32_t SAMPLE_INTERVAL_MS = 40;
constexpr uint32_t INITIAL_NEUTRAL_SETTLE_MS = 1000;
constexpr uint32_t READ_TIMEOUT_MS = 30;

enum class MoveResult {
    reached,
    operator_abort,
    absolute_current,
    high_current_no_progress,
    no_progress,
    move_timeout,
    read_timeout,
};

const char *resultName(MoveResult result) {
    switch (result) {
        case MoveResult::reached: return "TARGET_REACHED";
        case MoveResult::operator_abort: return "OPERATOR_ABORT";
        case MoveResult::absolute_current: return "ABSOLUTE_CURRENT";
        case MoveResult::high_current_no_progress: return "HIGH_CURRENT_NO_PROGRESS";
        case MoveResult::no_progress: return "NO_PROGRESS";
        case MoveResult::move_timeout: return "MOVE_TIMEOUT";
        case MoveResult::read_timeout: return "READ_TIMEOUT";
    }
    return "UNKNOWN";
}

uint16_t absDifference(uint16_t a, uint16_t b) {
    return a > b ? a - b : b - a;
}

bool readJrkVariables(uint8_t offset, uint8_t length, uint8_t *buffer) {
    if (length == 0 || length > 15) return false;

    while (Serial3.available() > 0) Serial3.read();
    Serial3.write(0xE5);  // Get variables.
    Serial3.write(offset);
    Serial3.write(length);
    Serial3.flush();

    const uint32_t started = millis();
    while (Serial3.available() < length) {
        if (millis() - started > READ_TIMEOUT_MS) return false;
    }
    for (uint8_t i = 0; i < length; ++i) buffer[i] = Serial3.read();
    return true;
}

uint16_t readU16LE(const uint8_t *bytes) {
    return static_cast<uint16_t>(bytes[0]) |
           (static_cast<uint16_t>(bytes[1]) << 8);
}

bool readCurrent(uint16_t &currentMa) {
    uint8_t bytes[2];
    if (!readJrkVariables(0x19, sizeof(bytes), bytes)) return false;
    currentMa = readU16LE(bytes);
    return true;
}

bool readFeedback(uint16_t &feedback) {
    uint8_t bytes[2];
    if (!readJrkVariables(0x04, sizeof(bytes), bytes)) return false;
    feedback = readU16LE(bytes);
    return true;
}

bool readSnapshot(uint16_t &currentMa, uint16_t &feedback) {
    return readCurrent(currentMa) && readFeedback(feedback);
}

void setTarget(uint16_t target) {
    if (target > 4095) target = 4095;
    Serial3.write(0xC0 + (target & 0x1F));
    Serial3.write((target >> 5) & 0x7F);
    Serial3.flush();
}

void stopMotor() {
    Serial3.write(0xFF);  // JRK G2 compact-protocol Stop Motor.
    Serial3.flush();
}

bool operatorAbortRequested() {
    while (Serial.available() > 0) {
        const char c = Serial.read();
        if (c == 'q' || c == 'Q' || c == 'x' || c == 'X') return true;
    }
    return false;
}

String readLine(const String &promptText) {
    String line;
    uint32_t lastPromptAt = millis();
    while (true) {
        if (!Serial.available()) {
            if (millis() - lastPromptAt >= 5000) {
                Serial.println();
                Serial.print(promptText);
                lastPromptAt = millis();
            }
            continue;
        }
        const char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            Serial.println();
            line.trim();
            return line;
        }
        if (c == 8 || c == 127) {
            if (line.length()) {
                line.remove(line.length() - 1);
                Serial.print("\b \b");
            }
            continue;
        }
        line += c;
        Serial.print(c);
    }
}

long promptNumber(const char *label, long defaultValue) {
    String promptText = String(label) + " [" + String(defaultValue) + "]: ";
    Serial.print(promptText);
    const String line = readLine(promptText);
    return line.length() ? line.toInt() : defaultValue;
}

bool confirmStart() {
    const String promptText = "Begin guarded limit-data test? [y/N]: ";
    Serial.print(promptText);
    const String line = readLine(promptText);
    return line.length() && (line[0] == 'y' || line[0] == 'Y');
}

void printSample(const char *phase, uint16_t target, uint16_t currentMa,
                 uint16_t feedback, uint16_t peakMa, uint32_t elapsedMs) {
    Serial.print(millis()); Serial.print(',');
    Serial.print(phase); Serial.print(',');
    Serial.print(target); Serial.print(',');
    Serial.print(currentMa); Serial.print(',');
    Serial.print(feedback); Serial.print(',');
    Serial.print(peakMa); Serial.print(',');
    Serial.println(elapsedMs);
}

// For a probe fault, neutralOnFault is true and neutral is commanded before
// this function returns.  While returning to neutral, it is false; a fault
// then sends Stop Motor instead.
MoveResult moveWithGuard(uint16_t target, const char *phase,
                         bool neutralOnFault, uint16_t &peakMaOut,
                         uint16_t &finalFeedbackOut,
                         uint32_t maxMoveMs) {
    uint16_t initialCurrent = 0;
    uint16_t initialFeedback = 0;
    if (!readSnapshot(initialCurrent, initialFeedback)) {
        if (neutralOnFault) setTarget(NEUTRAL_TARGET); else stopMotor();
        return MoveResult::read_timeout;
    }

    peakMaOut = initialCurrent;
    finalFeedbackOut = initialFeedback;
    const int direction = target >= initialFeedback ? 1 : -1;
    uint16_t progressAnchor = initialFeedback;
    uint32_t lastProgressAt = millis();
    const uint32_t moveStarted = millis();
    uint32_t lastSampleAt = 0;

    setTarget(target);

    while (true) {
        if (operatorAbortRequested()) {
            if (neutralOnFault) setTarget(NEUTRAL_TARGET); else stopMotor();
            return MoveResult::operator_abort;
        }

        const uint32_t now = millis();
        if (lastSampleAt != 0 && now - lastSampleAt < SAMPLE_INTERVAL_MS) continue;
        lastSampleAt = now;

        uint16_t currentMa = 0;
        uint16_t feedback = 0;
        if (!readSnapshot(currentMa, feedback)) {
            if (neutralOnFault) setTarget(NEUTRAL_TARGET); else stopMotor();
            return MoveResult::read_timeout;
        }
        finalFeedbackOut = feedback;
        if (currentMa > peakMaOut) peakMaOut = currentMa;

        const uint32_t elapsed = millis() - moveStarted;
        printSample(phase, target, currentMa, feedback, peakMaOut, elapsed);

        if (absDifference(target, feedback) <= TARGET_TOLERANCE) {
            return MoveResult::reached;
        }

        const int progress = direction *
            (static_cast<int>(feedback) - static_cast<int>(progressAnchor));
        if (progress >= PROGRESS_COUNTS) {
            progressAnchor = feedback;
            lastProgressAt = millis();
        } else if (progress < 0) {
            // A target reversal can briefly keep moving in the old direction.
            // Follow that farthest excursion so the first real movement back
            // toward the new target is recognized as progress.
            progressAnchor = feedback;
        }

        MoveResult fault = MoveResult::reached;
        bool faulted = false;
        if (currentMa >= ABSOLUTE_CURRENT_MA) {
            fault = MoveResult::absolute_current;
            faulted = true;
        } else if (currentMa >= HIGH_CURRENT_MA &&
                   millis() - lastProgressAt >= HIGH_CURRENT_NO_PROGRESS_MS) {
            fault = MoveResult::high_current_no_progress;
            faulted = true;
        } else if (millis() - lastProgressAt >= NO_PROGRESS_MS) {
            fault = MoveResult::no_progress;
            faulted = true;
        } else if (millis() - moveStarted >= maxMoveMs) {
            fault = MoveResult::move_timeout;
            faulted = true;
        }

        if (faulted) {
            if (neutralOnFault) setTarget(NEUTRAL_TARGET); else stopMotor();
            return fault;
        }
    }
}

void haltForever() {
    Serial.println("Test halted. Reset the Teensy to start over.");
    while (true) delay(1000);
}

bool returnToNeutral(const char *reason) {
    Serial.print("RETURN_TO_NEUTRAL,");
    Serial.println(reason);
    uint16_t peakMa = 0;
    uint16_t finalFeedback = 0;
    const MoveResult result = moveWithGuard(
        NEUTRAL_TARGET, "RETURN", false, peakMa, finalFeedback,
        MAX_RETURN_MOVE_MS);
    Serial.print("RETURN_RESULT,"); Serial.print(resultName(result));
    Serial.print(",peak_mA="); Serial.print(peakMa);
    Serial.print(",feedback="); Serial.println(finalFeedback);
    if (result != MoveResult::reached) {
        Serial.println("RETURN FAILED: JRK Stop Motor command sent.");
        return false;
    }
    Serial.println("NEUTRAL_REACHED");
    return true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}
    Serial3.begin(JRK_BAUD);
    delay(200);

    Serial.println("============================================================");
    Serial.println("JRK GUARDED FORWARD-LIMIT DATA TEST");
    Serial.println("No JRK settings are read or changed by this firmware.");
    Serial.println("Engine OFF; tractor secured; operator at power cutoff.");
    Serial.println("Press q or x during motion to command return to neutral.");
    Serial.println("============================================================");

    uint16_t currentMa = 0;
    uint16_t feedback = 0;
    if (!readSnapshot(currentMa, feedback)) {
        Serial.println("ABORT: JRK current/feedback read timed out. No target sent.");
        haltForever();
    }
    Serial.print("Startup current mA: "); Serial.println(currentMa);
    Serial.print("Startup feedback: "); Serial.println(feedback);
    if (absDifference(feedback, NEUTRAL_TARGET) > 30) {
        Serial.println("ABORT: actuator is not near neutral. No target sent.");
        haltForever();
    }

    long firstTarget = promptNumber("First target", FIRST_PROBE_TARGET);
    long lastTarget = promptNumber("Lowest target", LOWEST_ALLOWED_TARGET);
    long stepSize = promptNumber("Target decrement", 20);

    if (firstTarget > FIRST_PROBE_TARGET || firstTarget < LOWEST_ALLOWED_TARGET ||
        lastTarget < LOWEST_ALLOWED_TARGET || lastTarget > firstTarget ||
        stepSize < 1 || stepSize > 50) {
        Serial.println("ABORT: requested range is outside guarded test limits.");
        haltForever();
    }

    Serial.print("Targets: ");
    for (long target = firstTarget; target >= lastTarget; target -= stepSize) {
        Serial.print(target); Serial.print(' ');
        if (target - stepSize < lastTarget && target != lastTarget) {
            Serial.print(lastTarget); Serial.print(' ');
            break;
        }
    }
    Serial.println();
    Serial.println("CSV: t_ms,phase,target,current_mA,feedback,peak_mA,elapsed_ms");

    if (!confirmStart()) {
        Serial.println("Not confirmed. No target was sent.");
        haltForever();
    }

    // Establish exact neutral before beginning the small progressive steps.
    if (!returnToNeutral("INITIALIZE")) haltForever();
    delay(INITIAL_NEUTRAL_SETTLE_MS);

    long target = firstTarget;
    while (true) {
        Serial.print("PROBE_BEGIN,target="); Serial.println(target);
        uint16_t peakMa = 0;
        uint16_t finalFeedback = 0;
        const MoveResult result = moveWithGuard(
            static_cast<uint16_t>(target), "PROBE", true, peakMa,
            finalFeedback, MAX_PROBE_MOVE_MS);

        Serial.print("PROBE_RESULT,target="); Serial.print(target);
        Serial.print(",result="); Serial.print(resultName(result));
        Serial.print(",peak_mA="); Serial.print(peakMa);
        Serial.print(",feedback="); Serial.println(finalFeedback);

        if (result != MoveResult::reached) {
            if (!returnToNeutral(resultName(result))) {
                haltForever();
            }
            Serial.println("CUTOFF RECORDED. No further targets will be commanded.");
            haltForever();
        }

        if (target == lastTarget) {
            if (!returnToNeutral("RANGE_COMPLETE")) haltForever();
            Serial.println("RANGE COMPLETE: all requested targets were achieved.");
            haltForever();
        }

        long nextTarget = target - stepSize;
        if (nextTarget < lastTarget) nextTarget = lastTarget;
        target = nextTarget;
    }
}

void loop() {}
