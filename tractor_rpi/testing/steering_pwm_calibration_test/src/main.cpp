/*********************************************************************
  steering_pwm_calibration_test / main.cpp
  ------------------------------------------------------------------
  Phase 1 of the steering mechanical-limit investigation.

  Purpose:
    Find a low PWM value that reliably moves the unloaded steering from
    near center to the halfway point on either the right or left side,
    then use exactly that PWM to return to the production center value.

  This test DOES NOT seek a mechanical stop. It stays inside the steering
  positions already used by production firmware (197/447/815).

  IMPORTANT LIMITATION:
    The installed Gen1 IBT-2 wiring exposes RPWM and LPWM only. Therefore
    this test limits PWM duty and how long an unresponsive motor is driven;
    it does not measure or impose a true motor-current limit.

  Direction and feedback conventions copied from
  tractor_teensy/src/archive/teensy_main_20260804.cpp:
    RPWM pin 5 -> RIGHT -> pot value decreases
    LPWM pin 6 -> LEFT  -> pot value increases

  Startup behavior:
    - Both PWM outputs are set to zero before any prompt.
    - The steering-pot value is read and displayed.
    - No motion is possible until the operator chooses a direction and
      types an explicit 'y' at the final confirmation prompt.

  During motion, q or x immediately stops both PWM outputs and halts.
*********************************************************************/

#include <Arduino.h>

constexpr uint8_t RPWM_PIN = 5;
constexpr uint8_t LPWM_PIN = 6;
constexpr uint8_t STEER_POT_PIN = A9;

// Current production steering references (2026-09-08 firmware ancestry).
constexpr int POT_RIGHT = 197;
constexpr int POT_CENTER = 447;
constexpr int POT_LEFT = 815;
constexpr int RIGHT_HALF_TARGET = (POT_RIGHT + POT_CENTER) / 2;  // 322
constexpr int LEFT_HALF_TARGET = (POT_CENTER + POT_LEFT) / 2;    // 631

// The tractor is expected to start close to straight ahead.
constexpr int START_CENTER_TOLERANCE = 40;
constexpr int TARGET_TOLERANCE = 5;

// Conservative PWM search. Production firmware currently regards 150 as
// the empirical minimum needed to move under normal load.
constexpr int PWM_START = 60;
constexpr int PWM_INCREMENT = 5;
constexpr int PWM_MAX = 180;

// A PWM step is raised only after this long without directional progress.
constexpr int PROGRESS_COUNTS = 2;
constexpr uint32_t RAMP_NO_PROGRESS_MS = 400;
constexpr uint32_t RETURN_NO_PROGRESS_MS = 750;
constexpr uint32_t MAX_OUTBOUND_MS = 20000;
constexpr uint32_t MAX_RETURN_MS = 12000;
constexpr uint32_t SAMPLE_INTERVAL_MS = 20;
constexpr uint32_t SETTLE_AT_HALF_MS = 750;

enum class Direction : int8_t {
    right = -1,
    left = 1,
};

enum class MoveResult : uint8_t {
    reached,
    operator_abort,
    no_progress,
    pwm_ceiling,
    timeout,
    wrong_direction,
};

void stopMotor() {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
}

void drive(Direction direction, int pwm) {
    if (direction == Direction::right) {
        analogWrite(LPWM_PIN, 0);
        analogWrite(RPWM_PIN, pwm);
    } else {
        analogWrite(RPWM_PIN, 0);
        analogWrite(LPWM_PIN, pwm);
    }
}

const char *directionName(Direction direction) {
    return direction == Direction::right ? "RIGHT" : "LEFT";
}

const char *resultName(MoveResult result) {
    switch (result) {
        case MoveResult::reached: return "TARGET_REACHED";
        case MoveResult::operator_abort: return "OPERATOR_ABORT";
        case MoveResult::no_progress: return "NO_PROGRESS";
        case MoveResult::pwm_ceiling: return "PWM_CEILING_NO_PROGRESS";
        case MoveResult::timeout: return "MOVE_TIMEOUT";
        case MoveResult::wrong_direction: return "WRONG_DIRECTION";
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

void haltForever(const char *message) {
    stopMotor();
    Serial.println(message);
    Serial.println("Motor stopped. Reset the Teensy to run again.");
    while (true) delay(1000);
}

void printSample(const char *phase, Direction direction, int target, int pwm,
                 int pot, int anchor, uint32_t elapsedMs) {
    Serial.print(millis()); Serial.print(',');
    Serial.print(phase); Serial.print(',');
    Serial.print(directionName(direction)); Serial.print(',');
    Serial.print(target); Serial.print(',');
    Serial.print(pwm); Serial.print(',');
    Serial.print(pot); Serial.print(',');
    Serial.print(anchor); Serial.print(',');
    Serial.println(elapsedMs);
}

// Outbound move: begin at PWM_START and raise PWM_INCREMENT only when the
// pot has not made PROGRESS_COUNTS of directional progress in the allowed
// interval. Once motion continues, retain the current PWM.
MoveResult rampToHalf(Direction direction, int target, int &foundPwm,
                      int &finalPot) {
    int pwm = PWM_START;
    int progressAnchor = analogRead(STEER_POT_PIN);
    uint32_t lastProgressAt = millis();
    const uint32_t moveStarted = millis();
    uint32_t lastSampleAt = 0;

    drive(direction, pwm);

    while (true) {
        if (abortRequested()) {
            stopMotor();
            return MoveResult::operator_abort;
        }

        const uint32_t now = millis();
        if (now - lastSampleAt < SAMPLE_INTERVAL_MS) continue;
        lastSampleAt = now;

        const int pot = analogRead(STEER_POT_PIN);
        finalPot = pot;
        const int remaining = static_cast<int>(direction) * (target - pot);
        const int progress = static_cast<int>(direction) * (pot - progressAnchor);
        printSample("OUTBOUND", direction, target, pwm, pot, progressAnchor,
                    now - moveStarted);

        if (abs(target - pot) <= TARGET_TOLERANCE || remaining <= 0) {
            stopMotor();
            foundPwm = pwm;
            return MoveResult::reached;
        }

        if (progress >= PROGRESS_COUNTS) {
            progressAnchor = pot;
            lastProgressAt = now;
        } else if (progress <= -PROGRESS_COUNTS) {
            stopMotor();
            foundPwm = pwm;
            return MoveResult::wrong_direction;
        }

        if (now - lastProgressAt >= RAMP_NO_PROGRESS_MS) {
            stopMotor();
            if (pwm >= PWM_MAX) {
                foundPwm = pwm;
                return MoveResult::pwm_ceiling;
            }
            pwm += PWM_INCREMENT;
            if (pwm > PWM_MAX) pwm = PWM_MAX;
            Serial.print("PWM_STEP,"); Serial.println(pwm);
            progressAnchor = pot;
            lastProgressAt = now;
            drive(direction, pwm);
        }

        if (now - moveStarted >= MAX_OUTBOUND_MS) {
            stopMotor();
            foundPwm = pwm;
            return MoveResult::timeout;
        }
    }
}

// Return with exactly the outbound PWM; do not increase it automatically.
MoveResult returnToCenter(Direction direction, int pwm, int &finalPot) {
    int progressAnchor = analogRead(STEER_POT_PIN);
    uint32_t lastProgressAt = millis();
    const uint32_t moveStarted = millis();
    uint32_t lastSampleAt = 0;

    drive(direction, pwm);

    while (true) {
        if (abortRequested()) {
            stopMotor();
            return MoveResult::operator_abort;
        }

        const uint32_t now = millis();
        if (now - lastSampleAt < SAMPLE_INTERVAL_MS) continue;
        lastSampleAt = now;

        const int pot = analogRead(STEER_POT_PIN);
        finalPot = pot;
        const int remaining = static_cast<int>(direction) * (POT_CENTER - pot);
        const int progress = static_cast<int>(direction) * (pot - progressAnchor);
        printSample("RETURN", direction, POT_CENTER, pwm, pot, progressAnchor,
                    now - moveStarted);

        if (abs(POT_CENTER - pot) <= TARGET_TOLERANCE || remaining <= 0) {
            stopMotor();
            return MoveResult::reached;
        }

        if (progress >= PROGRESS_COUNTS) {
            progressAnchor = pot;
            lastProgressAt = now;
        } else if (progress <= -PROGRESS_COUNTS) {
            stopMotor();
            return MoveResult::wrong_direction;
        }

        if (now - lastProgressAt >= RETURN_NO_PROGRESS_MS) {
            stopMotor();
            return MoveResult::no_progress;
        }

        if (now - moveStarted >= MAX_RETURN_MS) {
            stopMotor();
            return MoveResult::timeout;
        }
    }
}

void setup() {
    pinMode(RPWM_PIN, OUTPUT);
    pinMode(LPWM_PIN, OUTPUT);
    stopMotor();

    analogReadResolution(10);
    analogWriteResolution(8);

    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}
    delay(100);

    Serial.println("============================================================");
    Serial.println("STEERING PWM CALIBRATION TEST - PHASE 1");
    Serial.println("Engine OFF; front wheels raised; rear wheels chocked.");
    Serial.println("Operator must remain at the physical power cutoff.");
    Serial.println("Press q or x during motion for an immediate PWM stop.");
    Serial.println("This test does NOT seek either mechanical stop.");
    Serial.println("============================================================");
    Serial.print("Production references: RIGHT="); Serial.print(POT_RIGHT);
    Serial.print(" CENTER="); Serial.print(POT_CENTER);
    Serial.print(" LEFT="); Serial.println(POT_LEFT);
    Serial.print("Halfway targets: RIGHT="); Serial.print(RIGHT_HALF_TARGET);
    Serial.print(" LEFT="); Serial.println(LEFT_HALF_TARGET);

    const int startupPot = analogRead(STEER_POT_PIN);
    Serial.print("Startup pot (read-only): "); Serial.println(startupPot);
    if (abs(startupPot - POT_CENTER) > START_CENTER_TOLERANCE) {
        haltForever("ABORT: steering is not within 40 pot counts of center.");
    }

    const String directionLine = readLine("Test direction [r/l, q=quit]: ");
    if (directionLine.length() == 0 || directionLine[0] == 'q' ||
        directionLine[0] == 'Q' || directionLine[0] == 'x' ||
        directionLine[0] == 'X') {
        haltForever("No direction selected. No motion command was sent.");
    }

    Direction outboundDirection;
    int halfwayTarget;
    if (directionLine[0] == 'r' || directionLine[0] == 'R') {
        outboundDirection = Direction::right;
        halfwayTarget = RIGHT_HALF_TARGET;
    } else if (directionLine[0] == 'l' || directionLine[0] == 'L') {
        outboundDirection = Direction::left;
        halfwayTarget = LEFT_HALF_TARGET;
    } else {
        haltForever("ABORT: direction must be r or l. No motion command was sent.");
    }

    Serial.println("----------------------------- PLAN ---------------------------");
    Serial.print("Outbound direction: "); Serial.println(directionName(outboundDirection));
    Serial.print("Outbound target: "); Serial.println(halfwayTarget);
    Serial.print("PWM search: "); Serial.print(PWM_START); Serial.print(" to ");
    Serial.print(PWM_MAX); Serial.print(" in steps of "); Serial.println(PWM_INCREMENT);
    Serial.print("Return target: "); Serial.print(POT_CENTER);
    Serial.println(" using exactly the PWM found outbound");
    Serial.println("--------------------------------------------------------------");

    const String confirmation = readLine("Begin test? Steering WILL move [y/N]: ");
    if (confirmation.length() == 0 ||
        (confirmation[0] != 'y' && confirmation[0] != 'Y')) {
        haltForever("Not confirmed. No motion command was sent.");
    }

    Serial.println("CSV: t_ms,phase,direction,target,pwm,pot,progress_anchor,elapsed_ms");
    int foundPwm = 0;
    int finalPot = startupPot;
    const MoveResult outboundResult =
        rampToHalf(outboundDirection, halfwayTarget, foundPwm, finalPot);

    Serial.print("OUTBOUND_RESULT,result="); Serial.print(resultName(outboundResult));
    Serial.print(",pwm="); Serial.print(foundPwm);
    Serial.print(",pot="); Serial.println(finalPot);
    if (outboundResult != MoveResult::reached) {
        haltForever("Outbound move did not safely reach halfway; return was not attempted.");
    }

    Serial.print("CALIBRATED_OUTBOUND_PWM="); Serial.println(foundPwm);
    delay(SETTLE_AT_HALF_MS);

    const Direction returnDirection = outboundDirection == Direction::right
        ? Direction::left : Direction::right;
    const MoveResult returnResult =
        returnToCenter(returnDirection, foundPwm, finalPot);

    Serial.print("RETURN_RESULT,result="); Serial.print(resultName(returnResult));
    Serial.print(",pwm="); Serial.print(foundPwm);
    Serial.print(",pot="); Serial.println(finalPot);
    if (returnResult != MoveResult::reached) {
        haltForever("Return did not reach center; PWM remains stopped.");
    }

    stopMotor();
    Serial.println("TEST_COMPLETE");
    Serial.print("Retain this candidate PWM for this outbound direction: ");
    Serial.println(foundPwm);
    Serial.println("Both PWM outputs are zero. Reset the Teensy to run again.");
}

void loop() {
    stopMotor();
}
