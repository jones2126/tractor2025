#include <Arduino.h>

/*
 * IBT_test_PID_direct.cpp
 * PID with direct PWM output (Cytron approach)
 *
 * Key difference from IBT_test_PID_variablePWM.cpp:
 *   - REMOVED: Non-linear PWM mapper that bucketed PID output into
 *              discrete PWM bands (65/110/235 etc.)
 *   - ADDED:   Raw PID output is constrained directly to PWM_MIN..PWM_MAX
 *              so PWM is proportional to error at all times.
 *
 * PWM_MIN is the stall floor - set to the lowest PWM that actually
 * moves your actuator.  Start with 0 and raise if it stalls near target.
 */

#define LED_PIN 13
const int RPWM = 5;
const int LPWM = 6;
const int POT_PIN = A9;

const int POT_RIGHT  = 197;
const int POT_CENTER = 446;
const int POT_LEFT   = 815;

const int   DEADBAND = 10;
int         PWM_MIN  = 150; 
const int   PWM_MAX  = 235;

const unsigned long HOLD_TIME    = 4000;
const unsigned long MOVE_TIMEOUT = 12000;

const int MAX_CYCLES = 2;

const int NUM_TARGETS = 4;
const int targets[NUM_TARGETS]     = { POT_RIGHT, POT_LEFT, POT_RIGHT, POT_CENTER };
const char* targetNames[NUM_TARGETS] = { "hard right", "hard left", "hard right", "center" };

// PID gains
float kp = 1.0;
float ki = 0.02;
float kd = 0.08;

float pid_error_sum  = 0;
float pid_last_error = 0;
unsigned long pid_last_time = 0;

int currentTarget    = 0;
unsigned long holdStart      = 0;
unsigned long moveStartTime  = 0;
bool holdingPosition = false;
int cycleCount       = 0;
bool testDone        = false;

unsigned long moveTimes[20];
int moveCount = 0;

unsigned long lastTrajLog = 0;
const unsigned long TRAJ_LOG_INTERVAL = 100;
int last_pwm = 0;

bool ledState = false;
unsigned long lastBlink = 0;

void stopMotor() { analogWrite(RPWM, 0); analogWrite(LPWM, 0); }

void resetPID() {
    pid_error_sum    = 0;
    pid_last_error   = 0;
    pid_last_time    = millis();
}

void resetTest() {
    currentTarget    = 0;
    holdingPosition  = false;
    moveStartTime    = 0;
    lastTrajLog      = 0;
    resetPID();
}

void recordMoveTime(unsigned long d) {
    if (moveCount < 20) moveTimes[moveCount++] = d;
    else { for (int i = 0; i < 19; i++) moveTimes[i] = moveTimes[i+1]; moveTimes[19] = d; }
}

void printMoveStats() {
    if (moveCount == 0) return;
    unsigned long total = 0, minT = moveTimes[0], maxT = moveTimes[0];
    for (int i = 0; i < moveCount; i++) {
        total += moveTimes[i];
        if (moveTimes[i] < minT) minT = moveTimes[i];
        if (moveTimes[i] > maxT) maxT = moveTimes[i];
    }
    Serial.print("=== Move Stats ("); Serial.print(moveCount); Serial.println(" moves) ===");
    Serial.print("  Avg: ");  Serial.print(total / moveCount / 1000.0f, 2); Serial.println(" s");
    Serial.print("  Fast: "); Serial.print(minT / 1000.0f, 2); Serial.println(" s");
    Serial.print("  Slow: "); Serial.print(maxT / 1000.0f, 2); Serial.println(" s");
}

bool driveToTarget(int target) {
    int pot      = analogRead(POT_PIN);
    float error  = target - pot;

    unsigned long now = millis();
    if (now - lastTrajLog >= TRAJ_LOG_INTERVAL) {
        Serial.print("  moving  pot="); Serial.print(pot);
        Serial.print("  target="); Serial.print(target);
        Serial.print("  error="); Serial.print((int)error);
        Serial.print("  pwm="); Serial.println(last_pwm);
        lastTrajLog = now;
    }

    if (abs(error) <= DEADBAND) {
        stopMotor();
        return true;
    }

    // PID calculation (unchanged)
    float dt = (now - pid_last_time) / 1000.0f;
    if (dt <= 0) dt = 0.01f;
    pid_last_time = now;

    pid_error_sum += error * dt;
    pid_error_sum  = constrain(pid_error_sum, -150, 150);  // windup contrain for PID I

    float d_error    = (error - pid_last_error) / dt;
    pid_last_error   = error;

    float output = kp * error + ki * pid_error_sum + kd * d_error;

    // CHANGED (lines 127-134): Replaced non-linear PWM mapper with direct constrain.
    // Raw PID output magnitude becomes PWM directly.
    // At error=606 this gives PWM=255; at error=13 this gives PWM=13.
    int pwm = constrain(abs((int)output), PWM_MIN, PWM_MAX);
    last_pwm = pwm;

    if (output > 0) {
        analogWrite(RPWM, 0); analogWrite(LPWM, pwm);
    } else {
        analogWrite(LPWM, 0); analogWrite(RPWM, pwm);
    }
    return false;
}

void setup() {
    pinMode(LED_PIN, OUTPUT); pinMode(RPWM, OUTPUT); pinMode(LPWM, OUTPUT);
    stopMotor();
    for (int i = 0; i < 3; i++) { digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW); delay(100); }

    delay(30000);
    Serial.begin(115200);
    Serial.println("\n=== PID Direct PWM Test (Cytron approach) ===");  // CHANGED: updated label
    Serial.print("kp="); Serial.print(kp);
    Serial.print(" ki="); Serial.print(ki);
    Serial.print(" kd="); Serial.print(kd);
    Serial.print(" PWM_MIN="); Serial.println(PWM_MIN);  // ADDED: show PWM_MIN at startup
    resetTest();
}

void loop() {
    unsigned long now = millis();
    if (now - lastBlink >= 500) { ledState = !ledState; digitalWrite(LED_PIN, ledState); lastBlink = now; }
    if (testDone) return;

    if (!holdingPosition) {
        if (moveStartTime == 0) moveStartTime = now;
        if (now - moveStartTime >= MOVE_TIMEOUT) {
            Serial.print("TIMEOUT moving to "); Serial.println(targetNames[currentTarget]);
            stopMotor();
            moveStartTime = 0; holdingPosition = true; holdStart = now;
            resetPID();
        }
        if (driveToTarget(targets[currentTarget])) {
            unsigned long dur = now - moveStartTime;
            Serial.print(">>> REACHED "); Serial.print(targetNames[currentTarget]);
            Serial.print(" in "); Serial.print(dur / 1000.0f, 2); Serial.println(" s");
            recordMoveTime(dur);
            moveStartTime = 0; holdingPosition = true; holdStart = now;
            resetPID();
        }
    } else {
        static unsigned long lastPrint = 0;
        if (now - lastPrint >= 2000) {
            Serial.print("    Holding "); Serial.print(targetNames[currentTarget]);
            Serial.print(" pot="); Serial.print(analogRead(POT_PIN));
            Serial.print(" PWM_MIN="); Serial.println(PWM_MIN);
            lastPrint = now;
        }
        if (now - holdStart >= HOLD_TIME) {
            if (currentTarget == NUM_TARGETS - 1) {
                cycleCount++;
                Serial.print("--- Cycle "); Serial.print(cycleCount); Serial.println(" complete ---");
                printMoveStats();
            }
            if (cycleCount >= MAX_CYCLES) {
                stopMotor(); testDone = true;
                Serial.println("=== TEST COMPLETE ===");
                printMoveStats();
                return;
            }
            currentTarget    = (currentTarget + 1) % NUM_TARGETS;
            holdingPosition  = false; moveStartTime = 0; lastTrajLog = 0;
            Serial.print("Moving to: "); Serial.println(targetNames[currentTarget]);
        }
    }
}
