/*********************************************************************
  steering_left_limit_test / main.cpp
  ------------------------------------------------------------------
  Guarded Phase 2 test for locating either unloaded steering mechanical
  limit. The operator selects LEFT or RIGHT before confirmation.

  Based on the successful Phase 1 result from 2026-09-14:
    - PWM 65 moved reliably in both directions.
    - Each drive pulse is 100 ms.
    - Both PWM outputs are then zero for 100 ms before measurement.
    - The reported position is the median of nine pot readings.

  The probe stops after three consecutive pulses produce less than two
  counts of movement toward the selected side. It records the farthest
  settled pot position and DOES NOT increase PWM at the suspected stop.
  After a motor-off pause, it returns toward center with the same PWM 65,
  using shorter pulses near center for a more precise final position.

  IMPORTANT LIMITATION:
    Gen1 IBT-2 wiring does not expose current sense or enable/reset.
    This test limits PWM duty and stall duration, not actual peak current.
    A no-progress result is a candidate mechanical limit at PWM 65; visual
    inspection is required to distinguish a hard stop from rising load.

  During motion, q or x immediately stops both PWM outputs and halts.
*********************************************************************/

#include <Arduino.h>

constexpr uint8_t RPWM_PIN = 5;
constexpr uint8_t LPWM_PIN = 6;
constexpr uint8_t STEER_POT_PIN = A9;

constexpr int POT_CENTER = 447;
constexpr int START_CENTER_TOLERANCE = 40;
constexpr int CENTER_TOLERANCE = 2;

constexpr int TEST_PWM = 65;
constexpr uint32_t PWM_PULSE_MS = 100;
constexpr uint32_t RETURN_MEDIUM_PULSE_MS = 60;
constexpr uint32_t RETURN_FINE_PULSE_MS = 35;
constexpr uint32_t PWM_OFF_SETTLE_MS = 100;
constexpr uint32_t LIMIT_PAUSE_MS = 1000;
constexpr int FILTER_SAMPLES = 9;

// Phase 1 settled raw/median pairs differed by at most one count, so two
// counts is treated as real motion while zero/one count remains no progress.
constexpr int PROGRESS_COUNTS = 2;
constexpr int NO_PROGRESS_PULSES_AT_LIMIT = 3;
constexpr int WRONG_DIRECTION_COUNTS = 6;
constexpr int WRONG_DIRECTION_PULSES = 2;
constexpr int RETURN_NO_PROGRESS_PULSES = 3;

constexpr int POT_SANITY_MIN = 50;
constexpr int POT_SANITY_MAX = 970;
constexpr int MAX_CHANGE_PER_PULSE = 75;
constexpr int MAX_PROBE_PULSES = 80;
constexpr int MAX_RETURN_PULSES = 80;
constexpr uint32_t MAX_PROBE_MS = 30000;
constexpr uint32_t MAX_RETURN_MS = 30000;

enum class Direction : int8_t {
    right = -1,
    left = 1,
};

enum class MoveResult : uint8_t {
    candidate_limit,
    reached_center,
    operator_abort,
    wrong_direction,
    sensor_fault,
    no_progress,
    pulse_limit,
    timeout,
};

void stopMotor() {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
}

void drive(Direction direction) {
    if (direction == Direction::left) {
        analogWrite(RPWM_PIN, 0);
        analogWrite(LPWM_PIN, TEST_PWM);
    } else {
        analogWrite(LPWM_PIN, 0);
        analogWrite(RPWM_PIN, TEST_PWM);
    }
}

const char *directionName(Direction direction) {
    return direction == Direction::left ? "LEFT" : "RIGHT";
}

const char *resultName(MoveResult result) {
    switch (result) {
        case MoveResult::candidate_limit: return "CANDIDATE_LIMIT";
        case MoveResult::reached_center: return "CENTER_REACHED";
        case MoveResult::operator_abort: return "OPERATOR_ABORT";
        case MoveResult::wrong_direction: return "WRONG_DIRECTION";
        case MoveResult::sensor_fault: return "SENSOR_FAULT";
        case MoveResult::no_progress: return "NO_PROGRESS";
        case MoveResult::pulse_limit: return "PULSE_LIMIT";
        case MoveResult::timeout: return "TIMEOUT";
    }
    return "UNKNOWN";
}

bool abortRequested() {
    while (Serial.available() > 0) {
        const char c = Serial.read();
        if (c == 'q' || c == 'Q' || c == 'x' || c == 'X') return true;
    }
    return false;
}

String readLine(const char *prompt) {
    String line;
    Serial.print(prompt);
    uint32_t lastPromptAt = millis();

    while (true) {
        if (!Serial.available()) {
            if (millis() - lastPromptAt >= 5000) {
                Serial.println();
                Serial.print(prompt);
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

int readMedianPot() {
    int samples[FILTER_SAMPLES];
    for (int i = 0; i < FILTER_SAMPLES; ++i) {
        samples[i] = analogRead(STEER_POT_PIN);
        delayMicroseconds(500);
    }

    for (int i = 1; i < FILTER_SAMPLES; ++i) {
        const int value = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > value) {
            samples[j + 1] = samples[j];
            --j;
        }
        samples[j + 1] = value;
    }
    return samples[FILTER_SAMPLES / 2];
}

void haltForever(const char *message) {
    stopMotor();
    Serial.println(message);
    while (true) {
        stopMotor();
        Serial.print("HALTED: "); Serial.print(message);
        Serial.print(" Live pot="); Serial.println(readMedianPot());
        delay(5000);
    }
}

bool delayWithAbort(uint32_t durationMs) {
    const uint32_t started = millis();
    while (millis() - started < durationMs) {
        if (abortRequested()) {
            stopMotor();
            return false;
        }
        delay(2);
    }
    return true;
}

bool pulseAndMeasure(Direction direction, uint32_t pulseMs,
                     int &rawPot, int &filteredPot) {
    drive(direction);
    if (!delayWithAbort(pulseMs)) return false;

    stopMotor();
    if (!delayWithAbort(PWM_OFF_SETTLE_MS)) return false;

    rawPot = analogRead(STEER_POT_PIN);
    filteredPot = readMedianPot();
    return true;
}

bool potReadingSane(int pot, int previousPot) {
    return pot >= POT_SANITY_MIN && pot <= POT_SANITY_MAX &&
           abs(pot - previousPot) <= MAX_CHANGE_PER_PULSE;
}

void printPulse(const char *phase, Direction direction, int pulseNumber,
                int rawPot, int filteredPot, int previousPot,
                int directionalStep, int noProgressPulses,
                uint32_t pulseMs, uint32_t elapsedMs) {
    Serial.print(millis()); Serial.print(',');
    Serial.print(phase); Serial.print(',');
    Serial.print(directionName(direction)); Serial.print(',');
    Serial.print(TEST_PWM); Serial.print(',');
    Serial.print(pulseNumber); Serial.print(',');
    Serial.print(rawPot); Serial.print(',');
    Serial.print(filteredPot); Serial.print(',');
    Serial.print(previousPot); Serial.print(',');
    Serial.print(directionalStep); Serial.print(',');
    Serial.print(noProgressPulses); Serial.print(',');
    Serial.print(pulseMs); Serial.print(',');
    Serial.println(elapsedMs);
}

MoveResult probeLimit(Direction direction, int &candidateLimit, int &pulseCount) {
    delay(PWM_OFF_SETTLE_MS);
    int previousPot = readMedianPot();
    const int startingPot = previousPot;
    int farthestPot = previousPot;
    int noProgressPulses = 0;
    int wrongDirectionStreak = 0;
    const uint32_t started = millis();

    for (int pulse = 1; pulse <= MAX_PROBE_PULSES; ++pulse) {
        int rawPot = 0;
        int filteredPot = previousPot;
        if (!pulseAndMeasure(direction, PWM_PULSE_MS, rawPot, filteredPot)) {
            candidateLimit = farthestPot;
            pulseCount = pulse;
            return MoveResult::operator_abort;
        }

        pulseCount = pulse;
        if (!potReadingSane(filteredPot, previousPot)) {
            candidateLimit = farthestPot;
            return MoveResult::sensor_fault;
        }

        const int directionalStep =
            static_cast<int>(direction) * (filteredPot - previousPot);
        const int totalDirectionalMovement =
            static_cast<int>(direction) * (filteredPot - startingPot);
        if (static_cast<int>(direction) * (filteredPot - farthestPot) > 0) {
            farthestPot = filteredPot;
        }

        if (directionalStep >= PROGRESS_COUNTS) {
            noProgressPulses = 0;
            wrongDirectionStreak = 0;
        } else {
            ++noProgressPulses;
            if (directionalStep <= -PROGRESS_COUNTS) {
                ++wrongDirectionStreak;
            } else {
                wrongDirectionStreak = 0;
            }
        }

        printPulse("PROBE", direction, pulse, rawPot, filteredPot,
                   previousPot, directionalStep, noProgressPulses,
                   PWM_PULSE_MS, millis() - started);

        if (wrongDirectionStreak >= WRONG_DIRECTION_PULSES &&
            totalDirectionalMovement <= -WRONG_DIRECTION_COUNTS) {
            candidateLimit = farthestPot;
            return MoveResult::wrong_direction;
        }

        if (noProgressPulses >= NO_PROGRESS_PULSES_AT_LIMIT) {
            candidateLimit = farthestPot;
            return MoveResult::candidate_limit;
        }

        previousPot = filteredPot;
        if (millis() - started >= MAX_PROBE_MS) {
            candidateLimit = farthestPot;
            return MoveResult::timeout;
        }
    }

    candidateLimit = farthestPot;
    return MoveResult::pulse_limit;
}

MoveResult returnToCenter(Direction direction, int &finalPot, int &pulseCount) {
    delay(PWM_OFF_SETTLE_MS);
    int previousPot = readMedianPot();
    const int startingPot = previousPot;
    int noProgressPulses = 0;
    int wrongDirectionStreak = 0;
    const uint32_t started = millis();

    for (int pulse = 1; pulse <= MAX_RETURN_PULSES; ++pulse) {
        int rawPot = 0;
        int filteredPot = previousPot;
        const int distanceToCenter = abs(POT_CENTER - previousPot);
        uint32_t pulseMs = PWM_PULSE_MS;
        if (distanceToCenter <= 12) {
            pulseMs = RETURN_FINE_PULSE_MS;
        } else if (distanceToCenter <= 35) {
            pulseMs = RETURN_MEDIUM_PULSE_MS;
        }

        if (!pulseAndMeasure(direction, pulseMs, rawPot, filteredPot)) {
            finalPot = previousPot;
            pulseCount = pulse;
            return MoveResult::operator_abort;
        }

        pulseCount = pulse;
        finalPot = filteredPot;
        if (!potReadingSane(filteredPot, previousPot)) {
            return MoveResult::sensor_fault;
        }

        const int directionalStep =
            static_cast<int>(direction) * (filteredPot - previousPot);
        const int totalDirectionalMovement =
            static_cast<int>(direction) * (filteredPot - startingPot);

        if (directionalStep >= PROGRESS_COUNTS) {
            noProgressPulses = 0;
            wrongDirectionStreak = 0;
        } else {
            ++noProgressPulses;
            if (directionalStep <= -PROGRESS_COUNTS) {
                ++wrongDirectionStreak;
            } else {
                wrongDirectionStreak = 0;
            }
        }

        printPulse("RETURN", direction, pulse, rawPot, filteredPot,
                   previousPot, directionalStep, noProgressPulses,
                   pulseMs, millis() - started);

        if (abs(POT_CENTER - filteredPot) <= CENTER_TOLERANCE ||
            static_cast<int>(direction) * (POT_CENTER - filteredPot) <= 0) {
            return MoveResult::reached_center;
        }

        if (wrongDirectionStreak >= WRONG_DIRECTION_PULSES &&
            totalDirectionalMovement <= -WRONG_DIRECTION_COUNTS) {
            return MoveResult::wrong_direction;
        }

        if (noProgressPulses >= RETURN_NO_PROGRESS_PULSES) {
            return MoveResult::no_progress;
        }

        previousPot = filteredPot;
        if (millis() - started >= MAX_RETURN_MS) return MoveResult::timeout;
    }

    return MoveResult::pulse_limit;
}

void setup() {
    pinMode(RPWM_PIN, OUTPUT);
    pinMode(LPWM_PIN, OUTPUT);
    stopMotor();
    analogReadResolution(10);
    analogWriteResolution(8);

    Serial.begin(115200);
    while (!Serial) {
        stopMotor();
        delay(50);
    }
    delay(100);

    Serial.println("============================================================");
    Serial.println("GUARDED STEERING MECHANICAL-LIMIT TEST - PHASE 2");
    Serial.println("Engine OFF; front wheels raised; rear wheels chocked.");
    Serial.println("WATCH THE WHEELS/LINKAGE; keep the power cutoff in reach.");
    Serial.println("Press q or x during motion for an immediate PWM stop.");
    Serial.println("============================================================");

    delay(PWM_OFF_SETTLE_MS);
    const int startupPot = readMedianPot();
    Serial.print("Startup filtered pot (read-only): "); Serial.println(startupPot);
    if (abs(startupPot - POT_CENTER) > START_CENTER_TOLERANCE) {
        haltForever("ABORT: steering is not within 40 pot counts of center.");
    }

    const String directionLine = readLine("Test direction [l/r, q=quit]: ");
    if (directionLine.length() == 0 || directionLine[0] == 'q' ||
        directionLine[0] == 'Q' || directionLine[0] == 'x' ||
        directionLine[0] == 'X') {
        haltForever("No direction selected. No motion command was sent.");
    }

    Direction probeDirection;
    if (directionLine[0] == 'l' || directionLine[0] == 'L') {
        probeDirection = Direction::left;
    } else if (directionLine[0] == 'r' || directionLine[0] == 'R') {
        probeDirection = Direction::right;
    } else {
        haltForever("ABORT: direction must be l or r. No motion command was sent.");
    }
    const Direction returnDirection = probeDirection == Direction::left
        ? Direction::right : Direction::left;

    Serial.println("----------------------------- PLAN ---------------------------");
    Serial.print("Probe direction: "); Serial.print(directionName(probeDirection));
    Serial.print(" at fixed PWM "); Serial.println(TEST_PWM);
    Serial.print("Pulse timing: "); Serial.print(PWM_PULSE_MS);
    Serial.print(" ms ON, then "); Serial.print(PWM_OFF_SETTLE_MS);
    Serial.println(" ms OFF before median measurement");
    Serial.print("Candidate limit: ");
    Serial.print(NO_PROGRESS_PULSES_AT_LIMIT);
    Serial.println(" consecutive pulses with less than 2 counts movement");
    Serial.println("PWM will NOT be increased at the candidate limit.");
    Serial.print("After recording it, the test returns ");
    Serial.print(directionName(returnDirection));
    Serial.println(" to center at PWM 65 with shorter pulses near center.");
    Serial.println("--------------------------------------------------------------");

    const String confirmation =
        readLine("Begin selected mechanical-limit test? Steering WILL move [y/N]: ");
    if (confirmation.length() == 0 ||
        (confirmation[0] != 'y' && confirmation[0] != 'Y')) {
        haltForever("Not confirmed. No motion command was sent.");
    }

    Serial.println("Confirmed. Both PWM outputs remain zero during countdown.");
    Serial.println("Start video now and move into a safe viewing position.");
    for (int seconds = 15; seconds >= 1; --seconds) {
        stopMotor();
        Serial.print("STEERING STARTS IN "); Serial.print(seconds);
        Serial.println(" SECONDS");
        if (!delayWithAbort(1000)) {
            haltForever("Operator aborted during the startup countdown.");
        }
    }
    Serial.print("COUNTDOWN COMPLETE - STARTING ");
    Serial.print(directionName(probeDirection));
    Serial.println(" PROBE NOW");

    Serial.println(
        "CSV: t_ms,phase,direction,pwm,pulse,raw_pot,filtered_pot,"
        "previous_pot,directional_step,no_progress_pulses,pulse_ms,elapsed_ms"
    );

    int candidateLimit = startupPot;
    int probePulses = 0;
    const MoveResult probeResult =
        probeLimit(probeDirection, candidateLimit, probePulses);
    stopMotor();

    Serial.print("PROBE_RESULT,result="); Serial.print(resultName(probeResult));
    Serial.print(",candidate_"); Serial.print(directionName(probeDirection));
    Serial.print("_limit="); Serial.print(candidateLimit);
    Serial.print(",pulses="); Serial.println(probePulses);

    if (probeResult != MoveResult::candidate_limit) {
        haltForever("Probe ended on a safety fault; automatic return was not attempted.");
    }

    Serial.print("CANDIDATE_"); Serial.print(directionName(probeDirection));
    Serial.print("_LIMIT="); Serial.println(candidateLimit);
    Serial.println("Motor off; pausing before guarded return to center.");
    if (!delayWithAbort(LIMIT_PAUSE_MS)) {
        haltForever("Operator aborted during the limit pause.");
    }

    int finalPot = candidateLimit;
    int returnPulses = 0;
    const MoveResult returnResult =
        returnToCenter(returnDirection, finalPot, returnPulses);
    stopMotor();

    Serial.print("RETURN_RESULT,result="); Serial.print(resultName(returnResult));
    Serial.print(",pot="); Serial.print(finalPot);
    Serial.print(",pulses="); Serial.println(returnPulses);

    if (returnResult != MoveResult::reached_center) {
        haltForever("Return did not safely reach center; PWM remains stopped.");
    }

    Serial.println("TEST_COMPLETE");
    Serial.print("Recorded candidate "); Serial.print(directionName(probeDirection));
    Serial.print(" limit at PWM 65: ");
    Serial.println(candidateLimit);
    Serial.print("Final centered pot: "); Serial.println(finalPot);
    Serial.println("Both PWM outputs are zero. Reset the Teensy to run again.");
}

void loop() {
    stopMotor();
}
