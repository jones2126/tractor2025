/*********************************************************************
  steering_pot_alignment_test / main.cpp
  ------------------------------------------------------------------
  Interactive, stationary steering alignment tool.

  Measured unloaded hard stops (2026-09-14):
    RIGHT = 46, LEFT = 752, midpoint = 399.

  Phase 1 -- position the steering mechanism:
    Enter r or l followed by Enter for one short PWM pulse. Both PWM
    outputs are then zero before raw and median pot readings are printed.
    Nudge until the settled reading is close to 399.

  Phase 2 -- physically adjust the 10-turn potentiometer:
    Enter a to permanently disable motor nudges for this run and begin
    continuous read-only monitoring. With steering left at its mechanical
    midpoint, physically adjust the pot toward ADC midpoint 512.

  q or x stops both PWM outputs and ends the session.

  IMPORTANT LIMITATION:
    Gen1 IBT-2 wiring does not expose current sense. Each nudge is limited
    by PWM and duration, not by a measured peak-current limit.
*********************************************************************/

#include <Arduino.h>

constexpr uint8_t RPWM_PIN = 5;
constexpr uint8_t LPWM_PIN = 6;
constexpr uint8_t STEER_POT_PIN = A9;

constexpr int MEASURED_RIGHT_STOP = 46;
constexpr int MEASURED_LEFT_STOP = 752;
constexpr int STEERING_MIDPOINT =
    (MEASURED_RIGHT_STOP + MEASURED_LEFT_STOP) / 2;  // 399
constexpr int POT_ELECTRICAL_MIDPOINT = 512;

constexpr int NUDGE_PWM = 65;
constexpr uint32_t NUDGE_MS = 25;
constexpr uint32_t PWM_OFF_SETTLE_MS = 150;
constexpr uint32_t LIVE_PRINT_INTERVAL_MS = 500;
constexpr int FILTER_SAMPLES = 9;
constexpr int POT_SANITY_MIN = 20;
constexpr int POT_SANITY_MAX = 1000;
constexpr int MAX_CHANGE_PER_NUDGE = 40;

enum class Direction : int8_t {
    right = -1,
    left = 1,
};

void stopMotor() {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
}

void drive(Direction direction) {
    if (direction == Direction::right) {
        analogWrite(LPWM_PIN, 0);
        analogWrite(RPWM_PIN, NUDGE_PWM);
    } else {
        analogWrite(RPWM_PIN, 0);
        analogWrite(LPWM_PIN, NUDGE_PWM);
    }
}

const char *directionName(Direction direction) {
    return direction == Direction::right ? "RIGHT" : "LEFT";
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

String readLine(const char *prompt) {
    String line;
    Serial.print(prompt);
    uint32_t lastPromptAt = millis();

    while (true) {
        stopMotor();
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

void printPosition(const char *label) {
    stopMotor();
    delay(PWM_OFF_SETTLE_MS);
    const int rawPot = analogRead(STEER_POT_PIN);
    const int filteredPot = readMedianPot();
    Serial.print(label);
    Serial.print(",raw_pot="); Serial.print(rawPot);
    Serial.print(",filtered_pot="); Serial.print(filteredPot);
    Serial.print(",steering_midpoint_error=");
    Serial.print(filteredPot - STEERING_MIDPOINT);
    Serial.print(",adjustment_target_error=");
    Serial.println(filteredPot - POT_ELECTRICAL_MIDPOINT);
}

bool nudge(Direction direction, int &previousPot) {
    drive(direction);
    delay(NUDGE_MS);
    stopMotor();
    delay(PWM_OFF_SETTLE_MS);

    const int rawPot = analogRead(STEER_POT_PIN);
    const int filteredPot = readMedianPot();
    const int directionalMovement =
        static_cast<int>(direction) * (filteredPot - previousPot);

    Serial.print("NUDGE_RESULT,direction="); Serial.print(directionName(direction));
    Serial.print(",pwm="); Serial.print(NUDGE_PWM);
    Serial.print(",pulse_ms="); Serial.print(NUDGE_MS);
    Serial.print(",raw_pot="); Serial.print(rawPot);
    Serial.print(",filtered_pot="); Serial.print(filteredPot);
    Serial.print(",directional_movement="); Serial.print(directionalMovement);
    Serial.print(",midpoint_error=");
    Serial.println(filteredPot - STEERING_MIDPOINT);

    if (filteredPot < POT_SANITY_MIN || filteredPot > POT_SANITY_MAX ||
        abs(filteredPot - previousPot) > MAX_CHANGE_PER_NUDGE) {
        Serial.println("FAULT: pot reading failed the nudge sanity check.");
        return false;
    }

    if (directionalMovement < -2) {
        Serial.println("FAULT: settled pot movement was opposite the command.");
        return false;
    }

    previousPot = filteredPot;
    if (abs(filteredPot - STEERING_MIDPOINT) <= 2) {
        Serial.println("STEERING_MIDPOINT_REACHED: enter a before adjusting the pot.");
    }
    return true;
}

void adjustmentMonitor() {
    stopMotor();
    Serial.println("============================================================");
    Serial.println("POTENTIOMETER ADJUSTMENT MODE - MOTOR NUDGES LOCKED OUT");
    Serial.println("Leave the steering mechanism stationary.");
    Serial.println("Physically adjust the potentiometer toward filtered_pot=512.");
    Serial.println("Readings print every 500 ms. Enter q or x when finished.");
    Serial.println("============================================================");

    uint32_t lastPrintAt = 0;
    while (true) {
        stopMotor();

        while (Serial.available() > 0) {
            const char c = Serial.read();
            if (c == 'q' || c == 'Q' || c == 'x' || c == 'X') {
                stopMotor();
                Serial.println("ALIGNMENT_SESSION_COMPLETE");
                Serial.print("Final filtered pot: ");
                Serial.println(readMedianPot());
                Serial.println("Both PWM outputs are zero. Reset to run again.");
                return;
            }
        }

        if (millis() - lastPrintAt >= LIVE_PRINT_INTERVAL_MS) {
            const int rawPot = analogRead(STEER_POT_PIN);
            const int filteredPot = readMedianPot();
            Serial.print("ADJUSTMENT,raw_pot="); Serial.print(rawPot);
            Serial.print(",filtered_pot="); Serial.print(filteredPot);
            Serial.print(",target=512,error=");
            Serial.println(filteredPot - POT_ELECTRICAL_MIDPOINT);
            lastPrintAt = millis();
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
    while (!Serial) {
        stopMotor();
        delay(50);
    }
    delay(100);

    Serial.println("============================================================");
    Serial.println("INTERACTIVE STEERING / POTENTIOMETER ALIGNMENT TEST");
    Serial.println("Engine OFF; front wheels raised; rear wheels chocked.");
    Serial.println("Each motor command requires Enter and lasts only 25 ms.");
    Serial.println("============================================================");
    Serial.print("Measured stops: RIGHT="); Serial.print(MEASURED_RIGHT_STOP);
    Serial.print(" LEFT="); Serial.println(MEASURED_LEFT_STOP);
    Serial.print("Steering midpoint target: "); Serial.println(STEERING_MIDPOINT);
    Serial.print("Pot adjustment target after entering a: ");
    Serial.println(POT_ELECTRICAL_MIDPOINT);

    delay(PWM_OFF_SETTLE_MS);
    int currentPot = readMedianPot();
    if (currentPot < POT_SANITY_MIN || currentPot > POT_SANITY_MAX) {
        Serial.println("ABORT: startup pot reading is outside sanity bounds.");
        while (true) {
            stopMotor();
            delay(1000);
        }
    }
    printPosition("STARTUP");

    while (true) {
        const String command = readLine(
            "Command [r=nudge right, l=nudge left, p=print, a=adjust pot, q=quit]: "
        );
        if (command.length() == 0) continue;

        const char c = command[0];
        if (c == 'r' || c == 'R') {
            if (!nudge(Direction::right, currentPot)) break;
        } else if (c == 'l' || c == 'L') {
            if (!nudge(Direction::left, currentPot)) break;
        } else if (c == 'p' || c == 'P') {
            printPosition("POSITION");
            currentPot = readMedianPot();
        } else if (c == 'a' || c == 'A') {
            if (abs(currentPot - STEERING_MIDPOINT) > 5) {
                Serial.println(
                    "REFUSED: steering is not within 5 counts of midpoint 399."
                );
                continue;
            }
            adjustmentMonitor();
            return;
        } else if (c == 'q' || c == 'Q' || c == 'x' || c == 'X') {
            stopMotor();
            Serial.println("ALIGNMENT_SESSION_ENDED; both PWM outputs are zero.");
            return;
        } else {
            Serial.println("Unknown command; no movement was commanded.");
        }
    }

    stopMotor();
    Serial.println("SESSION_HALTED_ON_FAULT; both PWM outputs are zero.");
}

void loop() {
    stopMotor();
}
