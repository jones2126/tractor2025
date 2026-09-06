/*********************************************************************
  JRK forward-limit data test with fail-safe return to neutral.

  This is a separate, temporary Teensy 4.1 test.  It does not modify
  JRK settings.  It commands one target at a time, records the JRK's
  current and feedback, and stops the motor after each small target
  increment before advancing to the next one.

  Probe cutoff behavior:
    - Immediately send Stop Motor (0xFF).
    - Stop the entire sweep; never advance to another target.
    - Return to neutral through guarded 40-count steps.
    - If a return step faults, leave the motor stopped.

  Run only with engine OFF, tractor secured against movement, and an
  operator watching the actuator/linkage with a physical power cutoff
  immediately available.
*********************************************************************/

#include <Arduino.h>

constexpr uint32_t JRK_BAUD = 9600;
constexpr uint16_t NEUTRAL_TARGET = 2836;
constexpr uint16_t TARGET_STEP_COUNTS = 40;
constexpr uint16_t FIRST_PROBE_TARGET = NEUTRAL_TARGET - TARGET_STEP_COUNTS;
// Guarded sweeps reached 2200 and then 2040 without a high-current stall.
// Extend the next staged sweep by only four additional 40-count pulses.
constexpr uint16_t LOWEST_ALLOWED_TARGET = 1880;
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
constexpr uint32_t MAX_RETURN_STEP_MS = 750;
constexpr uint32_t SAMPLE_INTERVAL_MS = 40;
constexpr uint32_t INITIAL_NEUTRAL_SETTLE_MS = 1000;
constexpr uint32_t READ_TIMEOUT_MS = 30;

enum class MoveResult {
    reached,
    operator_abort,
    absolute_current,
    high_current_no_progress,
    low_current_no_progress,
    move_timeout,
    read_timeout,
};

const char *resultName(MoveResult result) {
    switch (result) {
        case MoveResult::reached: return "TARGET_REACHED";
        case MoveResult::operator_abort: return "OPERATOR_ABORT";
        case MoveResult::absolute_current: return "ABSOLUTE_CURRENT";
        case MoveResult::high_current_no_progress: return "HIGH_CURRENT_NO_PROGRESS";
        case MoveResult::low_current_no_progress: return "LOW_CURRENT_NO_PROGRESS";
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

MoveResult moveWithGuard(uint16_t target, const char *phase,
                         uint16_t &peakMaOut, uint16_t &finalFeedbackOut,
                         uint32_t maxMoveMs) {
    uint16_t initialCurrent = 0;
    uint16_t initialFeedback = 0;
    if (!readSnapshot(initialCurrent, initialFeedback)) {
        stopMotor();
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
            stopMotor();
            return MoveResult::operator_abort;
        }

        const uint32_t now = millis();
        if (lastSampleAt != 0 && now - lastSampleAt < SAMPLE_INTERVAL_MS) continue;
        lastSampleAt = now;

        uint16_t currentMa = 0;
        uint16_t feedback = 0;
        if (!readSnapshot(currentMa, feedback)) {
            stopMotor();
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
            // With little or no current, a small target step can sit inside
            // drivetrain friction/deadband.  The caller may safely stop and
            // advance one more step to build enough position error to move.
            fault = MoveResult::low_current_no_progress;
            faulted = true;
        } else if (millis() - moveStarted >= maxMoveMs) {
            fault = MoveResult::move_timeout;
            faulted = true;
        }

        if (faulted) {
            stopMotor();
            return fault;
        }
    }
}

void haltForever() {
    Serial.println("Test halted. Reset the Teensy to start over.");
    while (true) delay(1000);
}

bool returnToNeutralStaged(const char *reason) {
    Serial.print("RETURN_TO_NEUTRAL,");
    Serial.println(reason);

    while (true) {
        uint16_t currentMa = 0;
        uint16_t feedback = 0;
        if (!readSnapshot(currentMa, feedback)) {
            stopMotor();
            Serial.println("RETURN FAILED: status read timeout; motor stopped.");
            return false;
        }
        if (absDifference(feedback, NEUTRAL_TARGET) <= TARGET_TOLERANCE) {
            stopMotor();
            Serial.print("NEUTRAL_REACHED,feedback=");
            Serial.println(feedback);
            return true;
        }

        uint16_t nextTarget;
        if (feedback < NEUTRAL_TARGET) {
            const uint32_t candidate =
                static_cast<uint32_t>(feedback) + TARGET_STEP_COUNTS;
            nextTarget = candidate > NEUTRAL_TARGET
                ? NEUTRAL_TARGET : static_cast<uint16_t>(candidate);
        } else {
            nextTarget = feedback > NEUTRAL_TARGET + TARGET_STEP_COUNTS
                ? feedback - TARGET_STEP_COUNTS : NEUTRAL_TARGET;
        }

        Serial.print("RETURN_STEP_BEGIN,target="); Serial.println(nextTarget);
        uint16_t peakMa = 0;
        uint16_t finalFeedback = feedback;
        const MoveResult result = moveWithGuard(
            nextTarget, "RETURN", peakMa, finalFeedback, MAX_RETURN_STEP_MS);
        stopMotor();
        Serial.print("RETURN_STEP_RESULT,target="); Serial.print(nextTarget);
        Serial.print(",result="); Serial.print(resultName(result));
        Serial.print(",peak_mA="); Serial.print(peakMa);
        Serial.print(",feedback="); Serial.println(finalFeedback);
        if (result != MoveResult::reached) {
            Serial.println("RETURN FAILED: motor remains stopped.");
            return false;
        }
        delay(100);
    }
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
    long stepSize = promptNumber("Target decrement", TARGET_STEP_COUNTS);

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
    if (!returnToNeutralStaged("INITIALIZE")) haltForever();
    delay(INITIAL_NEUTRAL_SETTLE_MS);

    long target = firstTarget;
    while (true) {
        Serial.print("PROBE_BEGIN,target="); Serial.println(target);
        uint16_t peakMa = 0;
        uint16_t finalFeedback = 0;
        const MoveResult result = moveWithGuard(
            static_cast<uint16_t>(target), "PROBE", peakMa, finalFeedback,
            MAX_PROBE_MOVE_MS);
        stopMotor();

        Serial.print("PROBE_RESULT,target="); Serial.print(target);
        Serial.print(",result="); Serial.print(resultName(result));
        Serial.print(",peak_mA="); Serial.print(peakMa);
        Serial.print(",feedback="); Serial.println(finalFeedback);

        const bool lowCurrentSkip =
            result == MoveResult::low_current_no_progress;
        if (result != MoveResult::reached && !lowCurrentSkip) {
            if (!returnToNeutralStaged(resultName(result))) {
                haltForever();
            }
            Serial.println("CUTOFF RECORDED. No further targets will be commanded.");
            haltForever();
        }

        if (lowCurrentSkip) {
            Serial.println("LOW_CURRENT_NO_PROGRESS: advancing one guarded step.");
        }

        if (target == lastTarget) {
            if (!returnToNeutralStaged("RANGE_COMPLETE")) haltForever();
            Serial.println("RANGE COMPLETE: guarded floor reached without a high-current stall.");
            haltForever();
        }

        long nextTarget = target - stepSize;
        if (nextTarget < lastTarget) nextTarget = lastTarget;
        target = nextTarget;
    }
}

void loop() {}
