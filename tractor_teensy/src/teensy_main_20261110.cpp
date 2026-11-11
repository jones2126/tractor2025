/*********************************************************************
  teensy_main_20261110.cpp

  Consolidated Tractor01 Teensy 4.1 firmware based on the commissioned
  2026-11-04 RPi command-lease v3 build.

  This source intentionally contains one implementation of setup(), loop(),
  serial parsing, transmission control, steering control, and calibration.
  The superseded include/macro layering remains available only in src/archive.

  NRF24 and SPI support are retired. The RPi RPICMD lease is the sole motion
  command authority. Before the first valid session claim, after lease expiry,
  and during software E-stop, propulsion is neutral and steering drive is off
  or held according to the existing Pause behavior. Local steering/JRK faults
  retain authority.
*********************************************************************/

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TRACTOR_FIRMWARE_ID "teensy_main_20261110_rpi_command_lease_v5"

// JRK controller
#define JRK_BAUD 9600
const uint16_t transmissionNeutralPos = 2836;
const uint16_t bucketTargets[10] = {
    3138, 3063, 2987, 2912, 2836, 2616, 2534, 2452, 2370, 2288
};

struct SpeedCalPoint { float mps; uint16_t jrkTarget; };

const SpeedCalPoint SPEED_CAL[] = {
    {0.00f, 2836},
    {0.14f, 2428},
    {1.00f, 2300},
    {1.20f, 2233},
    {1.50f, 2178},
    {1.80f, 2135},
};
const int SPEED_CAL_POINTS = sizeof(SPEED_CAL) / sizeof(SPEED_CAL[0]);

// IBT-2 steering pins
int RPWM_Output = 5;
int LPWM_Output = 6;

// Steering pot & PID
#define STEER_POT_PIN A9
#define STEER_DEADBAND 10
#define STEER_MIN_PWM  150   // minimum PWM to overcome motor stall (~150 empirically)

// Steering-response watchdog. A drive attempt must move the steering pot at
// least STEER_RESPONSE_MIN_MOVEMENT counts toward the command before this
// timeout expires. The 20 Hz telemetry exposes every attempt so the thresholds
// can be refined from field data.
const int STEER_RESPONSE_MIN_ERROR = 25;
const int STEER_RESPONSE_MIN_PWM = 150;
const int STEER_RESPONSE_MIN_MOVEMENT = 5;
const unsigned long STEER_RESPONSE_TIMEOUT_MS = 750;

enum SteeringResponseState : uint8_t {
    STEER_RESPONSE_IDLE = 0,
    STEER_RESPONSE_PENDING = 1,
    STEER_RESPONSE_RESPONDED = 2,
    STEER_RESPONSE_FAULT = 3
};

float steer_kp = 1.0;
float steer_ki = 0.0;
float steer_kd = 0.0;

// Tractor steering calibration. These are permanent source values; change
// them here and rebuild if the physical steering calibration changes.
const int STEER_POT_RIGHT  = 191;
const int STEER_POT_CENTER = 525;
const int STEER_POT_LEFT   = 885;

// Historical raw range retained only for stationary steering-fault recovery.
const int RECOVERY_STEER_RIGHT  = 1;    // RC joystick pushed hard right
const int RECOVERY_STEER_CENTER = 503;  // RC joystick centered
const int RECOVERY_STEER_LEFT   = 1024; // RC joystick pushed hard left

float steer_setpoint = STEER_POT_CENTER;
float steer_current   = STEER_POT_CENTER;
float steer_error     = 0;
float steer_error_sum = 0;
float steer_last_error = 0;
unsigned long steer_last_time = 0;

// Most recent low-level PID details for telemetry only.
float steer_pid_dt_s = 0.0f;
float steer_pid_derivative = 0.0f;
float steer_pid_p_term = 0.0f;
float steer_pid_i_term = 0.0f;
float steer_pid_d_term = 0.0f;
float steer_pid_output = 0.0f;

SteeringResponseState steerResponseState = STEER_RESPONSE_IDLE;
unsigned long steerResponseAttempt = 0;
unsigned long steerResponseStartMs = 0;
unsigned long steerResponseElapsedMs = 0;
long steerResponseTimeMs = -1;
float steerResponseStartPot = STEER_POT_CENTER;
int steerResponseMovement = 0;
int steerResponseDirection = 0;  // +1 left/increasing pot, -1 right/decreasing pot
bool steerDriveBlocked = false;
bool steeringFaultLatched = false;
unsigned long steeringFaultCount = 0;
bool steeringRecoveryPauseSeen = false;
bool steeringManualResponseOK = false;

// Transmission vars
uint16_t currentTransmissionOutput = transmissionNeutralPos;
int bucket = 5;

struct JrkDiagnostics {
    uint16_t requestedTarget = transmissionNeutralPos;
    uint16_t actualTarget = transmissionNeutralPos;
    uint16_t feedback = transmissionNeutralPos;
    uint16_t scaledFeedback = transmissionNeutralPos;
    int16_t integral = 0;
    int16_t dutyCycleTarget = 0;
    int16_t dutyCycle = 0;
    uint16_t errorsHalting = 0;
    uint16_t errorsOccurred = 0;
    uint32_t sequence = 0;
    uint32_t timeouts = 0;
    uint32_t discardedBytes = 0;
    uint16_t readLatencyMs = 0;
    bool valid = false;
} jrkDiagnostics;

// NeoPixel (not used)
#define NUM_LEDS 1
#define DATA_PIN 2


enum ControlMode : byte {
    CONTROL_AUTO = 0,
    CONTROL_RECOVERY_MANUAL = 1,
    CONTROL_PAUSE = 2,
    CONTROL_SAFETY = 9,
};

struct ControlCommandState {
    int16_t steering_val = RECOVERY_STEER_CENTER;
    int16_t transmission_val = 600;
    byte control_mode = CONTROL_PAUSE;
};

ControlCommandState commandState;
bool commandAuthorityReady = false;
unsigned long currentMillis = 0;
bool percentDriveCommand = false;
float percentDriveValue = 0.0f;
uint16_t jrkMotorCurrentMa = 0;
uint16_t jrkPeakCurrentMaSincePrint = 0;
bool jrkMotorCurrentValid = false;
uint16_t jrkVinVoltageMv = 0;
bool jrkVinVoltageValid = false;

#define TEXT_LOG_INTERVAL 2000UL
static unsigned long lastTextLog = 0;

// E-stop
unsigned long lastEstopCheckRun = 0;
const unsigned long estopCheckInterval = 50;
#define ESTOP_RELAY_PIN 30

// Control printing intervals
unsigned long lastTargetPrint = 0;
const unsigned long targetPrintInterval = 5000;

unsigned long lastTransmissionControlRun = 0;
const unsigned long controlTransmissionInterval = 100;   // 10 Hz

unsigned long lastSteeringControlRun = 0;
const unsigned long controlSteeringInterval = 50;        // 20 Hz; PID and telemetry share this cycle

// -------------------------------------------------------------------
// cmd_vel
struct CmdVelCommand {
    float linear_x = 0.0;
    float angular_z = 0.0;
    unsigned long timestamp = 0;
    bool received = false;
    unsigned long message_count = 0;
    float current_hz = 0.0;
} cmdVel;

unsigned long cmd_vel_count = 0;
unsigned long last_cmd_vel_time = 0;
const unsigned long CMD_VEL_TIMEOUT = 500;

// -------------------------------------------------------------------
// Serial processing rate limiting
unsigned long lastSerialProcessTime = 0;
const unsigned long serialProcessInterval = 10;   // 100 Hz max
const unsigned long maxSerialProcessTime = 5;     // 5 ms per call

struct SerialStats {
    unsigned long maxBufferSeen = 0;
    unsigned long overrunCount = 0;
    unsigned long lastWarning = 0;
} serialStats;

// TRANS_LOG (only in auto mode)
unsigned long lastTransLogPrint = 0;
const unsigned long transLogInterval = 5000;   // 0.2 Hz

// -------------------------------------------------------------------

void safeTextLog(const char* msg) {
    // Low-priority human-readable diagnostics share one two-second throttle.
    // If any caller printed recently, this call is intentionally discarded so
    // optional text cannot crowd out STEER, TRANS, and RPI_CTL telemetry.
    // Serial.flush() waits for bytes to leave the Teensy; it does not prove
    // that the RPi received, parsed, or saved the message.
    if (currentMillis - lastTextLog < TEXT_LOG_INTERVAL) return;
    Serial.println(msg);
    Serial.flush();
    lastTextLog = currentMillis;
}

bool readJrkVariables(uint8_t offset, uint8_t length, uint8_t *buffer) {
    if (length == 0 || length > 15) return false;

    uint16_t discarded = 0;
    while (Serial3.available() > 0) {
        Serial3.read();
        discarded++;
    }
    jrkDiagnostics.discardedBytes += discarded;

    unsigned long start = millis();
    Serial3.write(0xE5);
    Serial3.write(offset);
    Serial3.write(length);
    Serial3.flush();

    while (Serial3.available() < length) {
        if (millis() - start > 30) {
            jrkDiagnostics.timeouts++;
            jrkDiagnostics.readLatencyMs = millis() - start;
            return false;
        }
    }
    for (uint8_t i = 0; i < length; i++) buffer[i] = Serial3.read();
    jrkDiagnostics.readLatencyMs = millis() - start;
    return true;
}

uint16_t readU16LE(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

int16_t readI16LE(const uint8_t *p) {
    return (int16_t)readU16LE(p);
}

void updateJrkDiagnostics() {
    // Offset 0x02 through 0x0D: Target, Feedback, Scaled feedback,
    // Integral, Duty cycle target, and Duty cycle.
    uint8_t values[12];
    if (!readJrkVariables(0x02, sizeof(values), values)) {
        jrkDiagnostics.valid = false;
        return;
    }

    jrkDiagnostics.actualTarget = readU16LE(values + 0);
    jrkDiagnostics.feedback = readU16LE(values + 2);
    jrkDiagnostics.scaledFeedback = readU16LE(values + 4);
    jrkDiagnostics.integral = readI16LE(values + 6);
    jrkDiagnostics.dutyCycleTarget = readI16LE(values + 8);
    jrkDiagnostics.dutyCycle = readI16LE(values + 10);
    jrkDiagnostics.sequence++;
    jrkDiagnostics.valid = true;

    // Error flags change slowly; read them once per second to limit blocking
    // serial work on the 9600-baud JRK link.
    static unsigned long lastErrorRead = 0;
    if (currentMillis - lastErrorRead >= 1000) {
        uint8_t errors[4];
        if (readJrkVariables(0x12, sizeof(errors), errors)) {
            jrkDiagnostics.errorsHalting = readU16LE(errors + 0);
            jrkDiagnostics.errorsOccurred = readU16LE(errors + 2);
        }
        lastErrorRead = currentMillis;
    }
}

void setJrkTarget(uint16_t target) {
    if (target > 4095) target = 4095;
    Serial3.write(0xC0 + (target & 0x1F));
    Serial3.write((target >> 5) & 0x7F);
    jrkDiagnostics.requestedTarget = target;

    static unsigned long lastJrkPrint = 0;
    if (currentMillis - lastJrkPrint >= 5000) {
        char buf[48];
        snprintf(buf, sizeof(buf), "1,%lu,JRK,tgt=%u", currentMillis, target);
        safeTextLog(buf);
        lastJrkPrint = currentMillis;
    }
}

void monitorSerialBuffer() {
    static unsigned long last = 0;
    if (currentMillis - last < 1000) return;
    int avail = Serial.available();
    if ((unsigned long)avail > serialStats.maxBufferSeen) {
        serialStats.maxBufferSeen = (unsigned long)avail;
    }
    if (avail > 40) {                 // warning threshold
        char msg[64];
        snprintf(msg, sizeof(msg), "1,%lu,SERIAL,buf=%d,WARN", currentMillis, avail);
        safeTextLog(msg);
    }
    last = currentMillis;
}

void beginSteeringResponseAttempt(int direction) {
    steerResponseAttempt++;
    steerResponseState = STEER_RESPONSE_PENDING;
    steerResponseStartMs = currentMillis;
    steerResponseElapsedMs = 0;
    steerResponseTimeMs = -1;
    steerResponseStartPot = steer_current;
    steerResponseMovement = 0;
    steerResponseDirection = direction;
    steerDriveBlocked = false;
}

void latchSteeringResponseFault() {
    steerResponseState = STEER_RESPONSE_FAULT;
    steerResponseElapsedMs = currentMillis - steerResponseStartMs;
    steerDriveBlocked = true;

    if (!steeringFaultLatched) {
        steeringFaultLatched = true;
        steeringFaultCount++;
        steeringRecoveryPauseSeen = false;
        steeringManualResponseOK = false;
    }

    // Do not wait for the next 10 Hz transmission cycle to remove propulsion.
    setJrkTarget(transmissionNeutralPos);
}

void clearSteeringFaultAfterManualRecovery() {
    steeringFaultLatched = false;
    steeringRecoveryPauseSeen = false;
    steeringManualResponseOK = false;
    steerDriveBlocked = false;
}

bool updateSteeringResponseWatchdog(int pwmValue, int direction, bool pidActive) {
    const bool qualifies = pidActive &&
        pwmValue >= STEER_RESPONSE_MIN_PWM &&
        abs(steer_error) >= STEER_RESPONSE_MIN_ERROR &&
        direction != 0;

    if (!qualifies) {
        steerResponseState = STEER_RESPONSE_IDLE;
        steerResponseStartMs = 0;
        steerResponseElapsedMs = 0;
        steerResponseTimeMs = -1;
        steerResponseMovement = 0;
        steerResponseDirection = 0;
        steerDriveBlocked = false;
        return true;
    }

    if (steerDriveBlocked && direction == steerResponseDirection) {
        steerResponseState = STEER_RESPONSE_FAULT;
        return false;
    }

    if (steerResponseStartMs == 0 || direction != steerResponseDirection ||
        steerResponseState == STEER_RESPONSE_IDLE) {
        beginSteeringResponseAttempt(direction);
    }

    steerResponseElapsedMs = currentMillis - steerResponseStartMs;
    steerResponseMovement = direction * (int)(steer_current - steerResponseStartPot);

    if (steerResponseMovement >= STEER_RESPONSE_MIN_MOVEMENT &&
        steerResponseState == STEER_RESPONSE_PENDING) {
        steerResponseState = STEER_RESPONSE_RESPONDED;
        steerResponseTimeMs = (long)steerResponseElapsedMs;
        if (commandState.control_mode == 1 && steeringFaultLatched &&
            steeringRecoveryPauseSeen) {
            steeringManualResponseOK = true;
        }
    }

    if (steerResponseState == STEER_RESPONSE_PENDING &&
        steerResponseElapsedMs >= STEER_RESPONSE_TIMEOUT_MS) {
        latchSteeringResponseFault();
        return false;
    }

    return true;
}

float calculateSteerPID() {
    unsigned long now = millis();
    float dt = (now - steer_last_time) / 1000.0f;
    steer_last_time = now;

    steer_error = steer_setpoint - steer_current;

    steer_error_sum += steer_error * dt;
    steer_error_sum = constrain(steer_error_sum, -100, 100);

    float d_error = (steer_error - steer_last_error) / dt;
    steer_last_error = steer_error;

    // Retain each component for telemetry. The returned output is
    // mathematically identical to the previous implementation.
    steer_pid_dt_s = dt;
    steer_pid_derivative = d_error;
    steer_pid_p_term = steer_kp * steer_error;
    steer_pid_i_term = steer_ki * steer_error_sum;
    steer_pid_d_term = steer_kd * d_error;
    steer_pid_output = steer_pid_p_term +
                       steer_pid_i_term +
                       steer_pid_d_term;

    return steer_pid_output;
}

float mapSteerSetpoint(int radioVal) {
    radioVal = constrain(radioVal, RECOVERY_STEER_RIGHT, RECOVERY_STEER_LEFT);
    if (radioVal <= RECOVERY_STEER_CENTER) {
        // Right half
        return STEER_POT_RIGHT +
               (float)(radioVal - RECOVERY_STEER_RIGHT) *
               (float)(STEER_POT_CENTER - STEER_POT_RIGHT) /
               (float)(RECOVERY_STEER_CENTER - RECOVERY_STEER_RIGHT);
    } else {
        // Left half
        return STEER_POT_CENTER +
               (float)(radioVal - RECOVERY_STEER_CENTER) *
               (float)(STEER_POT_LEFT - STEER_POT_CENTER) /
               (float)(RECOVERY_STEER_LEFT - RECOVERY_STEER_CENTER);
    }
}

float mapNormalizedSteer(float n) {
    n = constrain(n, -1.0f, 1.0f);
    if (n >= 0.0f) {
        // Left half: 0..+1 -> 447..815
        return STEER_POT_CENTER + n * (float)(STEER_POT_LEFT - STEER_POT_CENTER);
    } else {
        // Right half: -1..0 -> 197..447  (n is negative, so this subtracts)
        return STEER_POT_CENTER + n * (float)(STEER_POT_CENTER - STEER_POT_RIGHT);
    }
}

void publishSteeringTelemetry(
    const char* state,
    const char* dir,
    int pwmValue,
    int leftPwm,
    int rightPwm,
    bool pidActive,
    bool deadbandActive,
    bool minClampApplied,
    bool pwmSaturated
) {
    static unsigned long sequence = 0;
    sequence++;

    const float reportedError = steer_setpoint - steer_current;
    const long cmdAgeMs = cmdVel.timestamp > 0
        ? (long)(currentMillis - cmdVel.timestamp)
        : -1L;

    char buf[320];
    snprintf(
        buf,
        sizeof(buf),
        "1,%lu,STEER,q=%lu,m=%d,st=%s,sp=%.0f,c=%.0f,e=%.0f,"
        "d=%s,p=%d,lp=%d,rp=%d,z=%.3f,pa=%d,db=%d,mc=%d,sat=%d,"
        "pdt=%.3f,es=%.1f,de=%.1f,pt=%.1f,it=%.1f,dt=%.1f,"
        "out=%.1f,ca=%ld,ra=%lu,rs=%u,re=%lu,rt=%ld,rm=%d,"
        "sf=%d,fc=%lu,ps=%d,rb=%d",
        currentMillis,
        sequence,
        commandAuthorityReady ? commandState.control_mode : 9,
        state,
        steer_setpoint,
        steer_current,
        reportedError,
        dir,
        pwmValue,
        leftPwm,
        rightPwm,
        cmdVel.angular_z,
        pidActive ? 1 : 0,
        deadbandActive ? 1 : 0,
        minClampApplied ? 1 : 0,
        pwmSaturated ? 1 : 0,
        pidActive ? steer_pid_dt_s : 0.0f,
        steer_error_sum,
        pidActive ? steer_pid_derivative : 0.0f,
        pidActive ? steer_pid_p_term : 0.0f,
        pidActive ? steer_pid_i_term : 0.0f,
        pidActive ? steer_pid_d_term : 0.0f,
        pidActive ? steer_pid_output : 0.0f,
        cmdAgeMs,
        steerResponseAttempt,
        (unsigned int)steerResponseState,
        steerResponseElapsedMs,
        steerResponseTimeMs,
        steerResponseMovement,
        steeringFaultLatched ? 1 : 0,
        steeringFaultCount,
        steeringRecoveryPauseSeen ? 1 : 0,
        steerDriveBlocked ? 1 : 0
    );

    Serial.println(buf);
}

void controlSteering() {
    if (currentMillis - lastSteeringControlRun < controlSteeringInterval) return;

    steer_current = analogRead(STEER_POT_PIN);

    // ---- NO SIGNAL â†’ stop motors ----
    if (!commandAuthorityReady) {
        analogWrite(RPWM_Output, 0);
        analogWrite(LPWM_Output, 0);
        updateSteeringResponseWatchdog(0, 0, false);
        publishSteeringTelemetry(
            "NO_SIG", "NS", 0, 0, 0,
            false, false, false, false
        );
        lastSteeringControlRun = currentMillis;
        //sendSteeringBinary(0);
        return;
    }

    int pwmValue = 0;
    int leftPwm = 0;
    int rightPwm = 0;
    const char* dir = "N";
    const char* state = "OK";
    bool pidActive = false;
    bool deadbandActive = false;
    bool minClampApplied = false;
    bool pwmSaturated = false;

    switch (commandState.control_mode) {
        case 0:  // Auto (cmd_vel) -- DOWN position on RC switch
            if (cmdVel.received) {
                // CHANGED 20260714: angular_z is now NORMALIZED -1..+1
                // (+1 = full left) from pure_pursuit_controller_20260714.py.
                // (Was: raw pot/PWM value passed straight into the setpoint.)
                steer_setpoint = mapNormalizedSteer(cmdVel.angular_z);
                float out = calculateSteerPID();
                pidActive = true;
                if (abs(steer_error) <= STEER_DEADBAND) {
                    pwmValue = 0;
                    deadbandActive = true;
                    dir = "AN";
                } else {
                    pwmSaturated = out > 255.0f || out < -255.0f;
                    pwmValue = constrain(abs((int)out), 0, 255);
                    if (pwmValue > 0 && pwmValue < STEER_MIN_PWM) {
                        pwmValue = STEER_MIN_PWM;
                        minClampApplied = true;
                    }
                    if (out > 0) {
                        // Positive error → steer LEFT → drive LPWM
                        leftPwm = pwmValue;
                        dir = "AL";
                    } else {
                        // Negative error → steer RIGHT → drive RPWM
                        rightPwm = pwmValue;
                        dir = "AR";
                    }
                }
            } else {
                steer_setpoint = steer_current;
                dir = "AH";
                state = "NO_CMD";
            }
            break;

        case 1:  // Manual PID — MIDDLE position on RC switch
            steer_setpoint = mapSteerSetpoint((int)commandState.steering_val);

            {
                float out = calculateSteerPID();
                pidActive = true;
                if (abs(steer_error) <= STEER_DEADBAND) {
                    pwmValue = 0;
                    deadbandActive = true;
                    dir = "N";
                } else {
                    pwmSaturated = out > 255.0f || out < -255.0f;
                    pwmValue = constrain(abs((int)out), 0, 255);
                    if (pwmValue > 0 && pwmValue < STEER_MIN_PWM) {
                        pwmValue = STEER_MIN_PWM;
                        minClampApplied = true;
                    }
                    if (out > 0) {
                        // Positive error → pot needs to increase → steer LEFT → drive LPWM
                        leftPwm = pwmValue;
                        dir = "L";
                    } else {
                        // Negative error → pot needs to decrease → steer RIGHT → drive RPWM
                        rightPwm = pwmValue;
                        dir = "R";
                    }
                }
            }
            break;

        case 2:  // Pause -- UP position on RC switch
            steer_setpoint = steer_current;
            dir = "P";
            state = "PAUSE";
            if (steeringFaultLatched) steeringRecoveryPauseSeen = true;
            break;

        default:
            dir = "E";
            state = "MODE_ERR";
            break;
    }

    const int responseDirection = leftPwm > 0 ? 1 : (rightPwm > 0 ? -1 : 0);
    bool steeringDriveAllowed = true;

    if (steeringFaultLatched && commandState.control_mode == 0) {
        // Auto cannot resume until Pause acknowledgement plus a successful
        // stationary Manual steering response clears the latched fault.
        steeringDriveAllowed = false;
        state = "STEER_FAULT";
    } else {
        steeringDriveAllowed = updateSteeringResponseWatchdog(
            pwmValue, responseDirection, pidActive
        );
        if (!steeringDriveAllowed) state = "STEER_FAULT";
    }

    if (!steeringDriveAllowed) {
        pwmValue = 0;
        leftPwm = 0;
        rightPwm = 0;
        dir = "F";
    }

    analogWrite(LPWM_Output, leftPwm);
    analogWrite(RPWM_Output, rightPwm);

    publishSteeringTelemetry(
        state, dir, pwmValue, leftPwm, rightPwm,
        pidActive, deadbandActive, minClampApplied, pwmSaturated
    );

    lastSteeringControlRun = currentMillis;
    //sendSteeringBinary(pwmValue);          // <<< BINARY PACKET
}

void debugSteerPot() {
    static unsigned long lastPotDebug = 0;
    if (currentMillis - lastPotDebug >= 1000) {
        int rawPot = analogRead(STEER_POT_PIN);
        char buf[64];
        snprintf(buf, sizeof(buf), "debug,Raw pot: %d, V: %.2f", rawPot, (rawPot * 3.3)/1023.0);
        safeTextLog(buf);  // Use newer's rate-limiter
        lastPotDebug = currentMillis;
    }
}

uint16_t mpsToJrkTarget(float mps) {
    if (mps <= SPEED_CAL[0].mps)
        return SPEED_CAL[0].jrkTarget;

    if (mps >= SPEED_CAL[SPEED_CAL_POINTS - 1].mps)
        return SPEED_CAL[SPEED_CAL_POINTS - 1].jrkTarget;

    for (int i = 1; i < SPEED_CAL_POINTS; i++) {
        if (mps <= SPEED_CAL[i].mps) {
            float f =
                (mps - SPEED_CAL[i - 1].mps) /
                (SPEED_CAL[i].mps -
                 SPEED_CAL[i - 1].mps);

            float t =
                (float)SPEED_CAL[i - 1].jrkTarget +
                f * ((float)SPEED_CAL[i].jrkTarget -
                     (float)SPEED_CAL[i - 1].jrkTarget);

            return (uint16_t)(t + 0.5f);
        }
    }

    return SPEED_CAL[0].jrkTarget;  // defensive/unreachable
}

uint16_t drivePercentToJrkTarget(float percent) {
    if (percent <= -100.0f) return 3138;
    if (percent >= 100.0f) return 2288;

    if (percent < 0.0f) {
        float target = 2836.0f + (-percent / 100.0f) * (3138.0f - 2836.0f);
        return (uint16_t)(target + 0.5f);
    }

    float target = 2836.0f - (percent / 100.0f) * (2836.0f - 2288.0f);
    return (uint16_t)(target + 0.5f);
}

void controlTransmission() {
    if (currentMillis - lastTransmissionControlRun <
        controlTransmissionInterval) return;

    if (!commandAuthorityReady) {
        commandState.control_mode = 9;   // safety
    }

    uint16_t requestedTarget = transmissionNeutralPos;
    int bucketTmp = bucket;

    switch (commandState.control_mode) {
        case 0:
            // RPi motion authority selects either m/s or signed-percent drive.
            if (cmdVel.received) {
                requestedTarget = percentDriveCommand
                    ? drivePercentToJrkTarget(percentDriveValue)
                    : mpsToJrkTarget(cmdVel.linear_x);
                bucketTmp = 4;
            } else {
                requestedTarget = transmissionNeutralPos;
                bucketTmp = 4;
            }
            break;

        case 1:
            // Stationary fault-recovery mode; propulsion remains neutral.
            {
                int tv = (int)commandState.transmission_val;

                if      (tv >= 931) bucketTmp = 0;
                else if (tv >= 838) bucketTmp = 1;
                else if (tv >= 746) bucketTmp = 2;
                else if (tv >= 654) bucketTmp = 3;
                else if (tv >= 562) bucketTmp = 4;
                else if (tv >= 469) bucketTmp = 5;
                else if (tv >= 377) bucketTmp = 6;
                else if (tv >= 285) bucketTmp = 7;
                else if (tv >= 192) bucketTmp = 8;
                else                bucketTmp = 9;

                requestedTarget = bucketTargets[bucketTmp];
            }
            break;

        case 2:
            // Pause mode.
            requestedTarget = transmissionNeutralPos;
            break;

        default:
            // Includes mode 9: missing-command-authority safety.
            requestedTarget = transmissionNeutralPos;
            break;
    }

    if (steeringFaultLatched) {
        if (commandState.control_mode == 2) {
            steeringRecoveryPauseSeen = true;
        }

        // A successful Manual steering response after Pause proves that the
        // actuator is available again. Require the transmission command to be
        // neutral before clearing so recovery cannot cause a surprise launch.
        const bool recoveryComplete =
            commandState.control_mode == 1 &&
            steeringRecoveryPauseSeen &&
            steeringManualResponseOK &&
            requestedTarget == transmissionNeutralPos;

        if (recoveryComplete) {
            clearSteeringFaultAfterManualRecovery();
        } else {
            requestedTarget = transmissionNeutralPos;
            bucketTmp = 4;
        }
    }

    bucket = bucketTmp;

    // Continue commanding the JRK at the normal transmission-control rate.
    setJrkTarget(requestedTarget);

    // Poll a coherent JRK diagnostic snapshot at 5 Hz.
    static unsigned long lastJrkFeedbackRead = 0;
    static const unsigned long jrkFeedbackInterval = 200;

    if (currentMillis - lastJrkFeedbackRead >= jrkFeedbackInterval) {
        updateJrkDiagnostics();
        currentTransmissionOutput = jrkDiagnostics.feedback;
        lastJrkFeedbackRead = currentMillis;
    }

    // Read JRK VIN (0x17, mV) and motor current (0x19, mA) in one consistent
    // snapshot at each 10 Hz transmission cycle. Preserve the current peak
    // between 5 Hz TRANS messages so short movement is less likely to be lost.
    uint8_t powerBytes[4];
    const bool powerReadValid = readJrkVariables(
        0x17, sizeof(powerBytes), powerBytes
    );
    jrkVinVoltageValid = powerReadValid;
    jrkMotorCurrentValid = powerReadValid;
    if (powerReadValid) {
        jrkVinVoltageMv = readU16LE(powerBytes);
        jrkMotorCurrentMa = readU16LE(powerBytes + 2);
    }
    if (jrkMotorCurrentValid) {
        if (jrkMotorCurrentMa > jrkPeakCurrentMaSincePrint) {
            jrkPeakCurrentMaSincePrint = jrkMotorCurrentMa;
        }
    }

    // Publish machine-readable telemetry at 5 Hz.
    static unsigned long lastTransStatusPrint = 0;
    static const unsigned long transStatusInterval = 200;

    if (currentMillis - lastTransStatusPrint >= transStatusInterval) {
        char buf[320];

        snprintf(
            buf,
            sizeof(buf),
            "1,%lu,TRANS,m=%d,b=%d,tgt=%u,cur=%u,at=%u,sfb=%u,"
            "it=%d,dtt=%d,dc=%d,jvin=%u,jvv=%d,jma=%u,jmp=%u,jmv=%d,"
            "eh=%u,eo=%u,jq=%lu,jv=%d,jl=%u,"
            "jto=%lu,jdb=%lu,x=%.3f,pct=%d,pc=%.1f,ca=%lu",
            currentMillis,
            commandState.control_mode,
            bucket,
            requestedTarget,
            currentTransmissionOutput,
            jrkDiagnostics.actualTarget,
            jrkDiagnostics.scaledFeedback,
            jrkDiagnostics.integral,
            jrkDiagnostics.dutyCycleTarget,
            jrkDiagnostics.dutyCycle,
            jrkVinVoltageMv,
            jrkVinVoltageValid ? 1 : 0,
            jrkMotorCurrentMa,
            jrkPeakCurrentMaSincePrint,
            jrkMotorCurrentValid ? 1 : 0,
            jrkDiagnostics.errorsHalting,
            jrkDiagnostics.errorsOccurred,
            (unsigned long)jrkDiagnostics.sequence,
            jrkDiagnostics.valid ? 1 : 0,
            jrkDiagnostics.readLatencyMs,
            (unsigned long)jrkDiagnostics.timeouts,
            (unsigned long)jrkDiagnostics.discardedBytes,
            cmdVel.linear_x,
            percentDriveCommand ? 1 : 0,
            percentDriveValue,
            cmdVel.received
                ? currentMillis - cmdVel.timestamp
                : 999999UL
        );

        Serial.println(buf);
        jrkPeakCurrentMaSincePrint =
            jrkMotorCurrentValid ? jrkMotorCurrentMa : 0;
        lastTransStatusPrint = currentMillis;
    }

    // Optional low-rate Auto-mode diagnostic; reuse cached JRK feedback.
    if (currentMillis - lastTransLogPrint >= transLogInterval &&
        commandState.control_mode == 0 &&
        cmdVel.received) {

        char buf[96];

        snprintf(
            buf,
            sizeof(buf),
            "1,%lu,TL,o=%u,t=%u,b=%d,fb=%u,hz=%.1f,x=%.2f",
            currentMillis,
            currentTransmissionOutput,
            requestedTarget,
            bucket,
            currentTransmissionOutput,
            cmdVel.current_hz,
            cmdVel.linear_x
        );

        safeTextLog(buf);
        lastTransLogPrint = currentMillis;
    }

    lastTransmissionControlRun = currentMillis;
}

void publishSystemIdentity() {
    Serial.println(
        "1,0,SYS,start,fw=" TRACTOR_FIRMWARE_ID ",steer_hz=20"
    );
}

void publishPeriodicSystemIdentity() {
    static unsigned long lastIdentityPrint = 0;
    const unsigned long identityPrintInterval = 5000;

    if (currentMillis - lastIdentityPrint >= identityPrintInterval) {
        publishSystemIdentity();
        lastIdentityPrint = currentMillis;
    }
}

enum RpiCommandMode : byte {
    RPI_MODE_PAUSE = 0,
    RPI_MODE_MANUAL_PERCENT = 1,
    RPI_MODE_AUTO_MPS = 2,
    RPI_MODE_ESTOP = 3,
    RPI_MODE_RESET_PAUSE = 4,
};

enum RpiRejectReason : byte {
    RPI_REJECT_NONE = 0,
    RPI_REJECT_CHECKSUM = 1,
    RPI_REJECT_FORMAT = 2,
    RPI_REJECT_RANGE = 3,
    RPI_REJECT_SEQUENCE = 4,
    RPI_REJECT_UNSUPPORTED = 5,
    RPI_REJECT_SESSION = 6,
};

static const unsigned long RPI_COMMAND_LEASE_MS = CMD_VEL_TIMEOUT;
static_assert(RPI_COMMAND_LEASE_MS <= 500UL, "RPi command lease may not exceed 500 ms");

static bool rpiCommandSelected = false;
static bool rpiSessionSeen = false;
static uint32_t rpiSession = 0;
static bool rpiSequenceSeen = false;
static uint32_t rpiSequence = 0;
static byte rpiCommandMode = RPI_MODE_PAUSE;
static float rpiDriveCommand = 0.0f;
static float rpiSteeringCommand = 0.0f;
static unsigned long rpiCommandTimestamp = 0;
static bool rpiLeaseExpiredLatched = false;
static unsigned long rpiAcceptedCount = 0;
static unsigned long rpiRejectedCount = 0;
static byte rpiLastRejectReason = RPI_REJECT_NONE;
static bool rpiSoftwareEstopLatched = false;


static uint16_t rpiCrc16Ccitt(const char* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(uint8_t)data[i] << 8;
        for (byte bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000)
                ? (uint16_t)((crc << 1) ^ 0x1021)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}


static bool parseFourHexDigits(const char* text, uint16_t* value) {
    if (!text || !value || strlen(text) != 4) return false;
    uint16_t parsed = 0;
    for (byte i = 0; i < 4; i++) {
        const char c = text[i];
        byte nibble;
        if (c >= '0' && c <= '9') nibble = (byte)(c - '0');
        else if (c >= 'A' && c <= 'F') nibble = (byte)(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') nibble = (byte)(c - 'a' + 10);
        else return false;
        parsed = (uint16_t)((parsed << 4) | nibble);
    }
    *value = parsed;
    return true;
}


static bool rpiSequenceIsNewer(uint32_t candidate, uint32_t previous) {
    const uint32_t difference = candidate - previous;
    return difference != 0 && difference < 0x80000000UL;
}


static bool parseUint32Token(const char* text, uint32_t* value) {
    if (!text || !*text || !value) return false;
    uint32_t parsed = 0;
    for (const char* cursor = text; *cursor; cursor++) {
        if (*cursor < '0' || *cursor > '9') return false;
        const uint32_t digit = (uint32_t)(*cursor - '0');
        if (parsed > (0xFFFFFFFFUL - digit) / 10UL) return false;
        parsed = parsed * 10UL + digit;
    }
    *value = parsed;
    return true;
}


static bool parseFiniteFloatToken(const char* text, float* value) {
    if (!text || !*text || !value) return false;
    char* end = nullptr;
    const float parsed = strtof(text, &end);
    if (end == text || *end != '\0' || !isfinite(parsed)) return false;
    *value = parsed;
    return true;
}


static int normalizedSteerToRecoveryRaw(float normalized) {
    normalized = constrain(normalized, -1.0f, 1.0f);
    if (normalized >= 0.0f) {
        return RECOVERY_STEER_CENTER + (int)(
            normalized * (RECOVERY_STEER_LEFT - RECOVERY_STEER_CENTER) + 0.5f
        );
    }
    return RECOVERY_STEER_CENTER + (int)(
        normalized * (RECOVERY_STEER_CENTER - RECOVERY_STEER_RIGHT) - 0.5f
    );
}


static void rejectRpiCommand(byte reason) {
    rpiRejectedCount++;
    rpiLastRejectReason = reason;
}


static bool acceptRpiCommandLine(char* line) {
    if (strncmp(line, "RPICMD,", 7) != 0) return false;

    char* separator = strrchr(line, '*');
    if (!separator || separator == line || strchr(separator + 1, '*')) {
        rejectRpiCommand(RPI_REJECT_FORMAT);
        return true;
    }

    uint16_t suppliedCrc = 0;
    if (!parseFourHexDigits(separator + 1, &suppliedCrc)) {
        rejectRpiCommand(RPI_REJECT_CHECKSUM);
        return true;
    }
    const size_t bodyLength = (size_t)(separator - line);
    if (rpiCrc16Ccitt(line, bodyLength) != suppliedCrc) {
        rejectRpiCommand(RPI_REJECT_CHECKSUM);
        return true;
    }
    *separator = '\0';

    char* fields[5];
    char* cursor = line + 7;
    for (byte i = 0; i < 4; i++) {
        fields[i] = cursor;
        char* comma = strchr(cursor, ',');
        if (!comma) {
            rejectRpiCommand(RPI_REJECT_FORMAT);
            return true;
        }
        *comma = '\0';
        cursor = comma + 1;
    }
    fields[4] = cursor;
    if (strchr(fields[4], ',')) {
        rejectRpiCommand(RPI_REJECT_FORMAT);
        return true;
    }

    uint32_t parsedSession = 0;
    uint32_t parsedSequence = 0;
    float parsedDrive = 0.0f;
    float parsedSteering = 0.0f;
    if (!parseUint32Token(fields[0], &parsedSession) ||
        !parseUint32Token(fields[1], &parsedSequence) ||
        strlen(fields[2]) != 1 || fields[2][0] < '0' ||
        fields[2][0] > '4' ||
        !parseFiniteFloatToken(fields[3], &parsedDrive) ||
        !parseFiniteFloatToken(fields[4], &parsedSteering)) {
        rejectRpiCommand(RPI_REJECT_FORMAT);
        return true;
    }
    const byte parsedMode = (byte)(fields[2][0] - '0');
    if (parsedSteering < -1.0f || parsedSteering > 1.0f) {
        rejectRpiCommand(RPI_REJECT_RANGE);
        return true;
    }
    if ((parsedMode == RPI_MODE_PAUSE &&
         (parsedDrive != 0.0f || parsedSteering != 0.0f)) ||
        ((parsedMode == RPI_MODE_ESTOP || parsedMode == RPI_MODE_RESET_PAUSE) &&
         (parsedDrive != 0.0f || parsedSteering != 0.0f)) ||
        (parsedMode == RPI_MODE_MANUAL_PERCENT &&
         (parsedDrive < -100.0f || parsedDrive > 100.0f)) ||
        (parsedMode == RPI_MODE_AUTO_MPS &&
         (parsedDrive < 0.0f || parsedDrive > 1.8f))) {
        rejectRpiCommand(RPI_REJECT_RANGE);
        return true;
    }

    const bool sessionChanged = !rpiSessionSeen || parsedSession != rpiSession;
    if (sessionChanged && parsedMode != RPI_MODE_PAUSE &&
        parsedMode != RPI_MODE_ESTOP) {
        rejectRpiCommand(RPI_REJECT_SESSION);
        return true;
    }
    if (!sessionChanged && rpiSequenceSeen &&
        !rpiSequenceIsNewer(parsedSequence, rpiSequence)) {
        rejectRpiCommand(RPI_REJECT_SEQUENCE);
        return true;
    }

    rpiSessionSeen = true;
    rpiSession = parsedSession;
    rpiSequenceSeen = true;
    rpiSequence = parsedSequence;
    rpiCommandMode = parsedMode;
    rpiDriveCommand = parsedDrive;
    rpiSteeringCommand = parsedSteering;
    rpiCommandTimestamp = millis();
    rpiCommandSelected = true;
    if (rpiCommandMode == RPI_MODE_PAUSE ||
        rpiCommandMode == RPI_MODE_RESET_PAUSE) {
        rpiLeaseExpiredLatched = false;
    }
    if (rpiCommandMode == RPI_MODE_ESTOP) rpiSoftwareEstopLatched = true;
    if (rpiCommandMode == RPI_MODE_RESET_PAUSE) rpiSoftwareEstopLatched = false;
    rpiAcceptedCount++;
    rpiLastRejectReason = RPI_REJECT_NONE;

    if (rpiCommandMode == RPI_MODE_PAUSE ||
        rpiCommandMode == RPI_MODE_ESTOP ||
        rpiCommandMode == RPI_MODE_RESET_PAUSE) {
        cmdVel.linear_x = 0.0f;
        cmdVel.angular_z = 0.0f;
        cmdVel.received = false;
    } else if (rpiCommandMode == RPI_MODE_MANUAL_PERCENT) {
        cmdVel.linear_x = 0.0f;
        cmdVel.angular_z = rpiSteeringCommand;
        cmdVel.timestamp = rpiCommandTimestamp;
        cmdVel.received = true;
    } else {
        cmdVel.linear_x = rpiDriveCommand;
        cmdVel.angular_z = rpiSteeringCommand;
        cmdVel.timestamp = rpiCommandTimestamp;
        cmdVel.received = true;
    }
    cmdVel.message_count++;
    last_cmd_vel_time = rpiCommandTimestamp;
    return true;
}


static void parseRpiCommandLeaseSerial() {
    if (currentMillis - lastSerialProcessTime < serialProcessInterval) return;

    const unsigned long startTime = millis();
    static char buffer[96];
    static uint8_t index = 0;
    static unsigned long lastRateCalc = 0;
    static unsigned long messagesSinceRate = 0;
    int processed = 0;

    while (Serial.available() > 0 && processed < 128 &&
           (millis() - startTime) < maxSerialProcessTime) {
        const char c = Serial.read();
        processed++;

        if (c == '\n') {
            buffer[index] = '\0';
            if (index > 0) {
                if (acceptRpiCommandLine(buffer)) {
                    if (rpiLastRejectReason == RPI_REJECT_NONE) messagesSinceRate++;
                } else {
                    // Only RPICMD frames are accepted. GPS remains an RPi
                    // responsibility; legacy CMD/WIFI/GPS frames are rejected.
                    rejectRpiCommand(RPI_REJECT_UNSUPPORTED);
                }
            }
            index = 0;
        } else if (c != '\r' && index < sizeof(buffer) - 1) {
            buffer[index++] = c;
        } else if (c != '\r') {
            index = 0;
            serialStats.overrunCount++;
            rejectRpiCommand(RPI_REJECT_FORMAT);
        }
    }

    if (currentMillis - lastRateCalc >= 1000) {
        const float elapsed = (currentMillis - lastRateCalc) / 1000.0f;
        cmdVel.current_hz = messagesSinceRate / elapsed;
        messagesSinceRate = 0;
        lastRateCalc = currentMillis;
    }
    lastSerialProcessTime = currentMillis;
}


static void publishRpiCommandLeaseStatus(bool arrivalFresh, bool leaseUsable) {
    static unsigned long lastStatusPrint = 0;
    if (currentMillis - lastStatusPrint < 200) return;
    const unsigned long age = rpiCommandSelected
        ? currentMillis - rpiCommandTimestamp
        : 999999UL;
    char buffer[176];
    snprintf(
        buffer,
        sizeof(buffer),
        "1,%lu,RPI_CTL,m=%u,f=%d,u=%d,l=%d,a=%lu,ss=%lu,s=%lu,ok=%lu,rj=%lu,rr=%u,e=%d",
        currentMillis,
        (unsigned int)rpiCommandMode,
        arrivalFresh ? 1 : 0,
        leaseUsable ? 1 : 0,
        rpiLeaseExpiredLatched ? 1 : 0,
        age,
        (unsigned long)rpiSession,
        (unsigned long)rpiSequence,
        rpiAcceptedCount,
        rpiRejectedCount,
        (unsigned int)rpiLastRejectReason,
        rpiSoftwareEstopLatched ? 1 : 0
    );
    Serial.println(buffer);
    lastStatusPrint = currentMillis;
}


extern "C" void setup() {
    delay(45000);

    Serial.begin(460800);
    while (!Serial && millis() < 10000) {}
    Serial3.begin(JRK_BAUD);

    // Exit JRK safe start and immediately establish the neutral target.
    delay(100);
    Serial3.write(0x83);
    Serial3.flush();
    setJrkTarget(transmissionNeutralPos);

    pinMode(RPWM_Output, OUTPUT);
    pinMode(LPWM_Output, OUTPUT);
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, 0);

    pinMode(ESTOP_RELAY_PIN, OUTPUT);
    digitalWrite(ESTOP_RELAY_PIN, HIGH);

    publishSystemIdentity();
    Serial.printf(
        "1,0,SYS,steer_cal,right=%d,center=%d,left=%d,"
        "hard_right=171,hard_left=905\n",
        STEER_POT_RIGHT,
        STEER_POT_CENTER,
        STEER_POT_LEFT
    );
    Serial.println("1,0,SYS,command_authority=rpi_lease,nrf24=retired");
    Serial.flush();
}


extern "C" void loop() {
    currentMillis = millis();

    parseRpiCommandLeaseSerial();
    currentMillis = millis();
    monitorSerialBuffer();
    publishPeriodicSystemIdentity();

    const bool arrivalFresh = rpiCommandSelected &&
        currentMillis - rpiCommandTimestamp <= RPI_COMMAND_LEASE_MS;
    if (rpiCommandSelected && !arrivalFresh) rpiLeaseExpiredLatched = true;
    const bool leaseUsable = arrivalFresh && !rpiLeaseExpiredLatched &&
        !rpiSoftwareEstopLatched;

    commandAuthorityReady = rpiCommandSelected;
    commandState.control_mode = CONTROL_PAUSE;
    commandState.transmission_val = 600;
    commandState.steering_val = RECOVERY_STEER_CENTER;

    if (!leaseUsable || rpiCommandMode == RPI_MODE_PAUSE ||
        rpiCommandMode == RPI_MODE_ESTOP ||
        rpiCommandMode == RPI_MODE_RESET_PAUSE) {
        percentDriveCommand = true;
        percentDriveValue = 0.0f;
        cmdVel.received = false;
    } else if (rpiCommandMode == RPI_MODE_MANUAL_PERCENT) {
        commandState.control_mode = CONTROL_AUTO;
        percentDriveCommand = true;
        percentDriveValue = rpiDriveCommand;
        cmdVel.linear_x = 0.0f;
        cmdVel.angular_z = rpiSteeringCommand;
        cmdVel.timestamp = rpiCommandTimestamp;
        cmdVel.received = true;
    } else if (rpiCommandMode == RPI_MODE_AUTO_MPS) {
        commandState.control_mode = CONTROL_AUTO;
        percentDriveCommand = false;
        percentDriveValue = 0.0f;
        cmdVel.linear_x = rpiDriveCommand;
        cmdVel.angular_z = rpiSteeringCommand;
        cmdVel.timestamp = rpiCommandTimestamp;
        cmdVel.received = true;
    }

    // Preserve the commissioned Pause -> neutral stationary recovery sequence.
    if (rpiCommandMode == RPI_MODE_MANUAL_PERCENT && leaseUsable &&
        steeringFaultLatched && steeringRecoveryPauseSeen &&
        fabsf(rpiDriveCommand) < 0.5f) {
        commandState.control_mode = CONTROL_RECOVERY_MANUAL;
        commandState.transmission_val = 600;
        commandState.steering_val = normalizedSteerToRecoveryRaw(
            rpiSteeringCommand
        );
    }

    if (currentMillis - lastEstopCheckRun >= estopCheckInterval) {
        digitalWrite(
            ESTOP_RELAY_PIN,
            rpiSoftwareEstopLatched ? LOW : HIGH
        );
        lastEstopCheckRun = currentMillis;
    }

    controlTransmission();
    controlSteering();
    publishRpiCommandLeaseStatus(arrivalFresh, leaseUsable);
    debugSteerPot();
}
