/*********************************************************************
  teensy_main_20260501.cpp
  --------------------------------------------------------------
  * Date: November 28, 2025
  * FIXED: Moved GPS functions after dependencies (GpsStatus struct and safeTextLog)
  * Date: April 13, 2026 (rev 1)
  * CHANGED: Updated JRK targets for new 4" linear actuator
  *          Full reverse=4095, Neutral=2951, Full forward=0
  * FIXED: setJrkTarget() log uses Serial.println directly (was safeTextLog)
  * Date: April 13, 2026 (rev 2)
  * CHANGED: Neutral updated to 2800 after full JRK calibration
  *          Bucket spacing: reverse 259/step (4095->2800), forward 700/step (2800->0)
  * Date: May 1, 2026
  * CHANGED: Steering pot constants updated to measured values
  *          STEER_POT_RIGHT=197, STEER_POT_CENTER=447, STEER_POT_LEFT=815
  * CHANGED: Removed old generic STEER_MIN_POT/STEER_MAX_POT/STEER_CENTER_POT
  * ADDED:   STEER_PWM_MIN=150, STEER_PWM_MAX=235 from bench test (20260501)
  * ADDED:   mapSteerSetpoint() — two-segment linear map handles asymmetric pot
  *          Right range: 447->197 (250 counts), Left range: 447->815 (368 counts)
  * CHANGED: controlSteering() case 1 uses mapSteerSetpoint() for manual setpoint
  * CHANGED: constrain(abs(out), 0, 255) -> constrain(abs(out), STEER_PWM_MIN, STEER_PWM_MAX)
  *          in both case 1 (manual) and case 2 (auto) of controlSteering()
  * CHANGED: steer_kp/ki/kd updated to match bench-tested values (1.0/0.02/0.08)
  * CHANGED: steer_setpoint default initialised to STEER_POT_CENTER (447)
*********************************************************************/

#include <SPI.h>
#include <RF24.h>
#include <string.h>  // For memset

// -------------------------------------------------------------------
// JRK controller
#define JRK_BAUD 9600
const uint16_t transmissionNeutralPos = 2800;  // calibrated neutral (was 2951)

// neutral=2800 after JRK calibration
// Reverse side: 4095->2800, range=1295, 5 steps of 259
// Forward side: 2800->0,    range=2800, 4 steps of 700
const uint16_t bucketTargets[10] = {
    4095, 3836, 3577, 3318, 3059,  // buckets 0-4: full reverse -> approaching neutral
    2800,                           // bucket  5:   neutral
    2100, 1400,  700,    0          // buckets 6-9: leaving neutral -> full forward
};

// IBT-2 steering pins
int RPWM_Output = 5;
int LPWM_Output = 6;

// Steering pot & PID
#define STEER_POT_PIN A9
#define STEER_DEADBAND 10

// CHANGED (was 1.0/0.0/0.0): updated to bench-tested values from 20260501
float steer_kp = 1.0;
float steer_ki = 0.02;
float steer_kd = 0.08;

// CHANGED: steer_setpoint default updated from 512 to measured center
float steer_setpoint = 447;   // CHANGED: was 512; now initialised to STEER_POT_CENTER
float steer_current   = 447;  // CHANGED: was 512
float steer_error     = 0;
float steer_error_sum = 0;
float steer_last_error = 0;
unsigned long steer_last_time = 0;

// CHANGED: Replaced generic STEER_MIN_POT/STEER_MAX_POT/STEER_CENTER_POT with
//          measured calibration values from bench test 20260501.
//          REMOVED: const int STEER_MIN_POT = 0;
//          REMOVED: const int STEER_MAX_POT = 1023;
//          REMOVED: const int STEER_CENTER_POT = 512;
const int STEER_POT_RIGHT  = 197;   // NEW: measured full-right pot value
const int STEER_POT_CENTER = 447;   // NEW: measured straight-ahead pot value (was 512)
const int STEER_POT_LEFT   = 815;   // NEW: measured full-left pot value

// NEW: PWM limits from bench test 20260501 (IBT_test_PID_direct.cpp, PWM_MIN=150)
const int STEER_PWM_MIN = 150;      // NEW: stall floor — lowest PWM that moves actuator
const int STEER_PWM_MAX = 235;      // NEW: peak PWM cap (matches test PWM_MAX)

// NEW: Radio joystick reference points for setpoint mapping
const int RADIO_STEER_CENTER = 512; // NEW: radio sends ~512 when joystick centred
const int RADIO_STEER_MAX    = 1024;// NEW: radio full-scale

// Transmission vars
uint16_t currentTransmissionOutput = transmissionNeutralPos;
int bucket = 5;

// NeoPixel (not used)
#define NUM_LEDS 1
#define DATA_PIN 2

// Radio pins
RF24 radio(9, 10);

// -------------------------------------------------------------------
struct __attribute__((packed)) RadioControlStruct {
    int16_t steering_val;      // 2B: Pin 15
    int16_t throttle_val;      // 2B: Pin 14
    int16_t transmission_val;  // 2B: Pin 16
    uint16_t voltage_mv;       // 2B: Voltage in millivolts
    int16_t pot4_val;          // 2B: Pin 17
    byte estop;                // 1B: Pin 10
    byte control_mode;         // 1B: Pin 3 & 4  From mode switch
    byte button02;             // 1B: Pin 9
    byte button03;             // 1B: Pin 6
    // Total: 14 bytes 
};

struct __attribute__((packed)) AckPayloadStruct {
    byte gps_status;         // 1=no NMEA, 2=GPS no RTK, 3=RTK Fix (0=unset)
    byte button02_status;    // Echo of received button02
    byte button03_status;    // Echo of received button03
    byte padding[11];        // 11B: Pad to 14 bytes total
};

RadioControlStruct radioData;
AckPayloadStruct ackPayload;

const uint8_t NRF24_ADDRESS_TRACTOR[6]       = "1Node";
const uint8_t NRF24_ADDRESS_RADIO_CONTROL[6] = "2Node";

unsigned long currentMillis = 0;

// -------------------------------------------------------------------
// Radio stats
struct RadioStats {
    unsigned long lastAckTime = 0;
    unsigned long ackCount = 0;
    unsigned long shortTermAckCount = 0;
    unsigned long lastRateReport = 0;
    unsigned long lastDataPrint = 0;
    bool signalGood = false;
    unsigned long packets_received_total = 0;
    unsigned long acks_sent_total = 0;
    unsigned long packets_received_20s = 0;
    unsigned long acks_sent_20s = 0;
    unsigned long lastStatsTime = 0;

    const unsigned long signalTimeout = 2000;
    const unsigned long rateReportInterval = 10000;
    const unsigned long dataPrintInterval = 5000;
    const unsigned long statsInterval = 20000;
} radioStats;

float smoothedRadioVal = 0.0f;
const float radioAlpha = 0.1f;

void updateRadioSmoothing() {
    smoothedRadioVal = radioAlpha * (float)radioData.transmission_val / 1024.0f +
                      (1.0f - radioAlpha) * smoothedRadioVal;
}

// E-stop
unsigned long lastEstopCheckRun = 0;
const unsigned long estopCheckInterval = 50;
#define ESTOP_RELAY_PIN 32

// Control printing intervals
unsigned long lastTargetPrint = 0;
const unsigned long targetPrintInterval = 2000;

unsigned long lastTransmissionControlRun = 0;
const unsigned long controlTransmissionInterval = 100;   // 10 Hz

unsigned long lastSteeringControlRun = 0;
const unsigned long controlSteeringInterval = 100;       // 10 Hz
unsigned long lastSteeringPrint = 0;
//const unsigned long steeringPrintInterval = 2000;
const unsigned long steeringPrintInterval = 200;

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

// GPS status tracking (received from bridge via serial)
struct GpsStatus {
    byte status = 1;                    // 0=unset, 1=noNMEA, 2=noRTK, 3=RTK Fix
    unsigned long last_update = 0;
    unsigned long messages_received = 0;
    unsigned long last_log = 0;
} gpsStatus;

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

#define CSV_LOG_ENABLED 0

// -------------------------------------------------------------------
// TEXT LOG RATE LIMITER (human readable)
#define TEXT_LOG_INTERVAL 2000UL
static unsigned long lastTextLog = 0;

void safeTextLog(const char* msg) {
    if (currentMillis - lastTextLog < TEXT_LOG_INTERVAL) return;
    Serial.println(msg);
    Serial.flush();
    lastTextLog = currentMillis;
}

// -------------------------------------------------------------------
// GPS FUNCTIONS
byte getGpsStatus() {
    unsigned long age = currentMillis - gpsStatus.last_update;
    if (age > 5000) {
        static unsigned long lastTimeoutLog = 0;
        if (currentMillis - lastTimeoutLog > 5000) {
            char buf[64];
            snprintf(buf, sizeof(buf), "1,%lu,GPS_TIMEOUT,age=%lu,st=%d",
                     currentMillis, age, gpsStatus.status);
            Serial.println(buf);
            lastTimeoutLog = currentMillis;
        }
        return 1;
    }
    return gpsStatus.status;
}

void updateGpsStatus(byte newStatus) {
    gpsStatus.status = newStatus;
    gpsStatus.last_update = currentMillis;
    gpsStatus.messages_received++;
    
    static byte lastLoggedStatus = 0;
    if (newStatus != lastLoggedStatus) {
        char buf[64];
        snprintf(buf, sizeof(buf), "1,%lu,GPS_UPDATE,st=%d->%d,cnt=%lu",
                 currentMillis, lastLoggedStatus, newStatus,
                 gpsStatus.messages_received);
        Serial.println(buf);
        lastLoggedStatus = newStatus;
    }
    
    if (currentMillis - gpsStatus.last_log >= 10000) {
        char buf[64];
        snprintf(buf, sizeof(buf), "1,%lu,GPS,st=%d,age=%lu,cnt=%lu",
                 currentMillis, gpsStatus.status,
                 currentMillis - gpsStatus.last_update,
                 gpsStatus.messages_received);
        Serial.println(buf);
        gpsStatus.last_log = currentMillis;
    }
}

// -------------------------------------------------------------------
// JRK feedback (return neutral on timeout)
uint16_t readFeedback() {
    Serial3.write(0xE5);
    Serial3.write(0x04);
    Serial3.write(0x02);

    unsigned long start = millis();
    while (Serial3.available() < 2) {
        if (millis() - start > 15) return transmissionNeutralPos;
    }
    uint8_t lo = Serial3.read();
    uint8_t hi = Serial3.read();
    return (hi << 8) | lo;
}

// -------------------------------------------------------------------
// JRK target
void setJrkTarget(uint16_t target) {
    if (target > 4095) target = 4095;
    Serial3.write(0xC0 + (target & 0x1F));
    Serial3.write((target >> 5) & 0x7F);

    static unsigned long lastJrkPrint = 0;
    if (currentMillis - lastJrkPrint >= 5000) {
        char buf[48];
        snprintf(buf, sizeof(buf), "1,%lu,JRK,tgt=%u", currentMillis, target);
        Serial.println(buf);
        lastJrkPrint = currentMillis;
    }
}

// -------------------------------------------------------------------
// Serial command parsing (CMD,<x>,<z>\n and GPS,<status>\n)
void parseSerialCommand() {
    if (currentMillis - lastSerialProcessTime < serialProcessInterval) return;

    unsigned long startTime = millis();
    static char buffer[64];
    static uint8_t idx = 0;
    static unsigned long lastRateCalc = 0;
    static unsigned long msgSinceCalc = 0;

    int processed = 0;
    const int maxPerCall = 128;

    while (Serial.available() > 0 &&
           processed < maxPerCall &&
           (millis() - startTime) < maxSerialProcessTime) {

        char c = Serial.read();
        processed++;

        if (c == '\n') {
            buffer[idx] = '\0';
            
            if (idx >= 4 && memcmp(buffer, "CMD,", 4) == 0) {
                float lx, az;
                if (sscanf(buffer + 4, "%f,%f", &lx, &az) == 2) {
                    cmdVel.linear_x = lx;
                    cmdVel.angular_z = az;
                    cmdVel.timestamp = millis();
                    cmdVel.received = true;
                    cmdVel.message_count++;
                    msgSinceCalc++;
                    last_cmd_vel_time = millis();

                    if (cmdVel.message_count % 50 == 0) {
                        char echo[64];
                        snprintf(echo, sizeof(echo),
                                 "3,%lu,CE,x=%.2f,z=%.2f,hz=%.1f",
                                 millis(), lx, az, cmdVel.current_hz);
                        Serial.println(echo);
                    }
                }
            }
            else if (idx >= 4 && memcmp(buffer, "GPS,", 4) == 0) {
                int status;
                if (sscanf(buffer + 4, "%d", &status) == 1) {
                    if (status >= 0 && status <= 3) {
                        updateGpsStatus((byte)status);
                        
                        if (gpsStatus.messages_received % 20 == 0) {
                            char echo[48];
                            snprintf(echo, sizeof(echo), "3,%lu,GPS_ECHO,s=%d,cnt=%lu",
                                     millis(), status, gpsStatus.messages_received);
                            Serial.println(echo);
                        }
                    } else {
                        char warn[48];
                        snprintf(warn, sizeof(warn), "2,%lu,GPS,invalid_status=%d",
                                 millis(), status);
                        safeTextLog(warn);
                    }
                }
            }
            idx = 0;
        } else if (idx < 63) {
            buffer[idx++] = c;
        } else {
            idx = 0;
            serialStats.overrunCount++;
            if (currentMillis - serialStats.lastWarning > 5000) {
                safeTextLog("2,0,SERIAL,buffer_overflow");
                serialStats.lastWarning = currentMillis;
            }
        }
    }

    if (currentMillis - lastRateCalc >= 1000) {
        float elapsed = (currentMillis - lastRateCalc) / 1000.0f;
        cmdVel.current_hz = msgSinceCalc / elapsed;
        msgSinceCalc = 0;
        lastRateCalc = currentMillis;
    }

    lastSerialProcessTime = currentMillis;
}

// -------------------------------------------------------------------
// cmd_vel timeout check
void checkCmdVelTimeout() {
    if (cmdVel.received && (currentMillis - cmdVel.timestamp) > CMD_VEL_TIMEOUT) {
        cmdVel.received = false;
    }
}

// -------------------------------------------------------------------
// Serial buffer health monitor
void monitorSerialBuffer() {
    static unsigned long last = 0;
    if (currentMillis - last < 1000) return;
    int avail = Serial.available();
    if ((unsigned long)avail > serialStats.maxBufferSeen) {
        serialStats.maxBufferSeen = (unsigned long)avail;
    }
    if (avail > 40) {
        char msg[64];
        snprintf(msg, sizeof(msg), "1,%lu,SERIAL,buf=%d,WARN", currentMillis, avail);
        safeTextLog(msg);
    }
    last = currentMillis;
}

// -------------------------------------------------------------------
// Transmission control (10 Hz)
void controlTransmission() {
    if (currentMillis - lastTransmissionControlRun < controlTransmissionInterval) return;

    if (!radioStats.signalGood) radioData.control_mode = 9;   // safety

    uint16_t requestedTarget = transmissionNeutralPos;
    int bucketTmp = bucket;

    switch (radioData.control_mode) {
        case 0:  // Pause
            requestedTarget = transmissionNeutralPos;
            break;

        case 1:  // Manual bucket
            {
                int tv = (int)radioData.transmission_val;
                if (tv >= 931) bucketTmp = 0;
                else if (tv >= 838) bucketTmp = 1;
                else if (tv >= 746) bucketTmp = 2;
                else if (tv >= 654) bucketTmp = 3;
                else if (tv >= 562) bucketTmp = 4;
                else if (tv >= 469) bucketTmp = 5;
                else if (tv >= 377) bucketTmp = 6;
                else if (tv >= 285) bucketTmp = 7;
                else if (tv >= 192) bucketTmp = 8;
                else bucketTmp = 9;
                requestedTarget = bucketTargets[bucketTmp];
            }
            break;

        case 2:  // Auto (cmd_vel)
            if (cmdVel.received) {
                float scaled = (cmdVel.linear_x + 1.0f) * 511.5f;
                int bucketScaled = (int)scaled;
                if (bucketScaled >= 931) bucketTmp = 0;
                else if (bucketScaled >= 838) bucketTmp = 1;
                else if (bucketScaled >= 746) bucketTmp = 2;
                else if (bucketScaled >= 654) bucketTmp = 3;
                else if (bucketScaled >= 562) bucketTmp = 4;
                else if (bucketScaled >= 469) bucketTmp = 5;
                else if (bucketScaled >= 377) bucketTmp = 6;
                else if (bucketScaled >= 285) bucketTmp = 7;
                else if (bucketScaled >= 192) bucketTmp = 8;
                else bucketTmp = 9;
                requestedTarget = bucketTargets[bucketTmp];
            } else {
                requestedTarget = transmissionNeutralPos;
            }
            break;

        default:
            requestedTarget = transmissionNeutralPos;
            break;
    }

    bucket = bucketTmp;
    setJrkTarget(requestedTarget);

    if (currentMillis - lastTargetPrint >= targetPrintInterval) {
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "1,%lu,TRANS,m=%d,b=%d,tgt=%u,cur=%u",
                 currentMillis, radioData.control_mode, bucket,
                 requestedTarget, currentTransmissionOutput);
        Serial.println(buf);
        lastTargetPrint = currentMillis;
    }

    if (currentMillis - lastTransLogPrint >= transLogInterval &&
        radioData.control_mode == 2 && cmdVel.received) {

        uint16_t fb = readFeedback();
        char buf[96];
        snprintf(buf, sizeof(buf),
                 "1,%lu,TL,o=%u,tv=%d,t=%u,b=%d,fb=%u,hz=%.1f,x=%.2f",
                 currentMillis, currentTransmissionOutput,
                 (int)radioData.transmission_val,
                 requestedTarget,
                 bucket, fb, cmdVel.current_hz, cmdVel.linear_x);
        safeTextLog(buf);
        lastTransLogPrint = currentMillis;
    }

    #if CSV_LOG_ENABLED
        uint16_t feedback = readFeedback();
        char csvBuf[128];
        snprintf(csvBuf, sizeof(csvBuf), "log,%lu,%u,%d,%u,%d,%u",
                 currentMillis, currentTransmissionOutput, 
                 (int)radioData.transmission_val,
                 requestedTarget, bucket, feedback);
        Serial.println(csvBuf);
    #endif

    lastTransmissionControlRun = currentMillis;
}

// -------------------------------------------------------------------
// NEW: Two-segment linear map from radio joystick (0-1024) to pot space.
//
// The steering pot is not centred symmetrically:
//   Right range: RADIO 0..512   -> POT 197..447  (250 pot counts)
//   Left  range: RADIO 512..1024 -> POT 447..815  (368 pot counts)
//
// A single map() call would compress one side and stretch the other.
// Using two segments ensures that joystick centre always maps to pot
// centre, and full travel on each side maps to the actual mechanical stop.
//
// Used only for manual (case 1) mode.  Auto (case 2) receives a setpoint
// already expressed in pot units from Pure Pursuit.
float mapSteerSetpoint(int radioVal) {             // NEW function
    radioVal = constrain(radioVal, 0, RADIO_STEER_MAX);
    if (radioVal <= RADIO_STEER_CENTER) {
        // Right half: radio 0..512 -> pot RIGHT..CENTER
        return STEER_POT_RIGHT +
               (float)radioVal *
               (float)(STEER_POT_CENTER - STEER_POT_RIGHT) /
               (float)RADIO_STEER_CENTER;
    } else {
        // Left half: radio 512..1024 -> pot CENTER..LEFT
        return STEER_POT_CENTER +
               (float)(radioVal - RADIO_STEER_CENTER) *
               (float)(STEER_POT_LEFT - STEER_POT_CENTER) /
               (float)(RADIO_STEER_MAX - RADIO_STEER_CENTER);
    }
}

// -------------------------------------------------------------------
// Steering PID calculator
float calculateSteerPID() {
    unsigned long now = millis();
    float dt = (now - steer_last_time) / 1000.0f;
    steer_last_time = now;

    steer_error = steer_setpoint - steer_current;

    steer_error_sum += steer_error * dt;
    steer_error_sum = constrain(steer_error_sum, -100, 100);

    float d_error = (steer_error - steer_last_error) / dt;
    steer_last_error = steer_error;

    return steer_kp * steer_error +
           steer_ki * steer_error_sum +
           steer_kd * d_error;
}

// -------------------------------------------------------------------
// Steering control (10 Hz)
void controlSteering() {
    if (currentMillis - lastSteeringControlRun < controlSteeringInterval) return;

    steer_current = analogRead(STEER_POT_PIN);

    // ---- NO SIGNAL -> stop motors ----
    if (!radioStats.signalGood) {
        analogWrite(RPWM_Output, 0);
        analogWrite(LPWM_Output, 0);
        if (currentMillis - lastSteeringPrint >= steeringPrintInterval) {
            char buf[48];
            snprintf(buf, sizeof(buf), "1,%lu,STEER,st=NO_SIG,pot=%.0f",
                     currentMillis, steer_current);
            Serial.println(buf);
            lastSteeringPrint = currentMillis;
        }
        lastSteeringControlRun = currentMillis;
        return;
    }

    int pwmValue = 0;
    const char* dir = "N";

    switch (radioData.control_mode) {
        case 0:  // Pause
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            steer_setpoint = steer_current;
            dir = "P";
            break;

        case 1:  // Manual PID
            // CHANGED: was (float)radioData.steering_val directly.
            // Now maps radio 0-1024 to actual asymmetric pot range via mapSteerSetpoint().
            steer_setpoint = mapSteerSetpoint((int)radioData.steering_val);

            {
                float out = calculateSteerPID();
                if (abs(steer_error) <= STEER_DEADBAND) {
                    pwmValue = 0;
                    dir = "N";
                    analogWrite(RPWM_Output, 0);
                    analogWrite(LPWM_Output, 0);
                } else {
                    // CHANGED: was constrain(abs((int)out), 0, 255)
                    pwmValue = constrain(abs((int)out), STEER_PWM_MIN, STEER_PWM_MAX);
                    if (out > 0) {
                        // analogWrite(LPWM_Output, 0);
                        // analogWrite(RPWM_Output, pwmValue);
                        // dir = "R";
                        analogWrite(RPWM_Output, 0);
                        analogWrite(LPWM_Output, pwmValue);  // was RPWM_Output
                        dir = "L";                      
                    } else {
                        // analogWrite(RPWM_Output, 0);
                        // analogWrite(LPWM_Output, pwmValue);
                        // dir = "L";
                        analogWrite(LPWM_Output, 0);
                        analogWrite(RPWM_Output, pwmValue);  // was LPWM_Output
                        dir = "R";  
                    }
                }
            }
            break;

        case 2:  // Auto (cmd_vel)
            // angular_z contains pot-space setpoint (0-1023) from Pure Pursuit — no remapping needed
            if (cmdVel.received) {
                steer_setpoint = cmdVel.angular_z;
                float out = calculateSteerPID();
                if (abs(steer_error) <= STEER_DEADBAND) {
                    pwmValue = 0;
                    dir = "AN";
                    analogWrite(RPWM_Output, 0);
                    analogWrite(LPWM_Output, 0);
                } else {
                    // CHANGED: was constrain(abs((int)out), 0, 255)
                    pwmValue = constrain(abs((int)out), STEER_PWM_MIN, STEER_PWM_MAX);
                    if (out > 0) {
                        analogWrite(LPWM_Output, 0);
                        analogWrite(RPWM_Output, pwmValue);
                        dir = "AR";
                    } else {
                        analogWrite(RPWM_Output, 0);
                        analogWrite(LPWM_Output, pwmValue);
                        dir = "AL";
                    }
                }
            } else {
                steer_setpoint = steer_current;
                analogWrite(RPWM_Output, 0);
                analogWrite(LPWM_Output, 0);
                dir = "AH";
            }
            break;

        default:
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            dir = "E";
            break;
    }

    // ---- Human readable status (rate limited) ----
    if (currentMillis - lastSteeringPrint >= steeringPrintInterval) {
        char buf[96];
        if (radioData.control_mode == 2) {
            snprintf(buf, sizeof(buf),
                     "1,%lu,STEER,m=%d,sp=%.0f,c=%.0f,e=%.0f,d=%s,p=%d,z=%.2f",
                     currentMillis, radioData.control_mode,
                     steer_setpoint, steer_current, steer_error,
                     dir, pwmValue, cmdVel.angular_z);
        } else {
            snprintf(buf, sizeof(buf),
                     "1,%lu,STEER,m=%d,sp=%.0f,c=%.0f,e=%.0f,d=%s,p=%d",
                     currentMillis, radioData.control_mode,
                     steer_setpoint, steer_current, steer_error,
                     dir, pwmValue);
        }
        Serial.println(buf);
        lastSteeringPrint = currentMillis;
    }

    lastSteeringControlRun = currentMillis;
}

// -------------------------------------------------------------------
// E-stop check
void estopCheck() {
    if (currentMillis - lastEstopCheckRun < estopCheckInterval) return;
    digitalWrite(ESTOP_RELAY_PIN, radioData.estop ? LOW : HIGH);
    lastEstopCheckRun = currentMillis;
}

// -------------------------------------------------------------------
// Radio handling
void handleRadio() {
    if (radio.available()) {
        radioStats.packets_received_total++;
        radioStats.packets_received_20s++;

        radio.read(&radioData, sizeof(RadioControlStruct));
        
        if (currentMillis - radioStats.lastDataPrint >= radioStats.dataPrintInterval) {
            char buf[80];
            snprintf(buf, sizeof(buf), "1,%lu,RADIO,s=%d,t=%d,x=%d,v=%.2f,e=%d,m=%d,b2=%d,b3=%d",
                     currentMillis,
                     (int)radioData.steering_val,      
                     (int)radioData.throttle_val,      
                     (int)radioData.transmission_val,  
                     radioData.voltage_mv / 1000.0f,   
                     radioData.estop,
                     radioData.control_mode,
                     radioData.button02,
                     radioData.button03);
            Serial.println(buf);
            radioStats.lastDataPrint = currentMillis;
        }
        
        ackPayload.button02_status = radioData.button02;
        ackPayload.button03_status = radioData.button03;
        ackPayload.gps_status = getGpsStatus();
        memset(ackPayload.padding, 0, sizeof(ackPayload.padding));

        static unsigned long lastAckWriteDebug = 0;
        if (currentMillis - lastAckWriteDebug >= 5000) {
            char buf[80];
            snprintf(buf, sizeof(buf), 
                     "1,%lu,ACK_PREP,gps=%d,b2=%d,b3=%d,size=%d",
                     currentMillis, 
                     ackPayload.gps_status,
                     ackPayload.button02_status, 
                     ackPayload.button03_status,
                     sizeof(AckPayloadStruct));
            Serial.println(buf);
            lastAckWriteDebug = currentMillis;
        }
        
        bool ackWriteOk = radio.writeAckPayload(1, &ackPayload, sizeof(AckPayloadStruct));
        if (ackWriteOk) {
            radioStats.ackCount++;
            radioStats.shortTermAckCount++;
            radioStats.acks_sent_total++;
            radioStats.acks_sent_20s++;
        } else {
            safeTextLog("1,0,RADIO,ack_send_fail");
        }
        
        radioStats.lastAckTime = currentMillis;
        
        updateRadioSmoothing();
    }
    
    radioStats.signalGood = (currentMillis - radioStats.lastAckTime < radioStats.signalTimeout);
    
    if (currentMillis - radioStats.lastRateReport >= radioStats.rateReportInterval) {
        float timeElapsed = (currentMillis - radioStats.lastRateReport) / 1000.0;
        float ackRate = radioStats.ackCount / timeElapsed;
        float currentRate = radioStats.shortTermAckCount / timeElapsed;
        
        char buf[64];
        snprintf(buf, sizeof(buf), "1,%lu,RADIO,ar=%.1f,cr=%.1f,ta=%lu,sg=%d",
                 currentMillis, ackRate, currentRate, 
                 radioStats.ackCount, radioStats.signalGood ? 1 : 0);
        Serial.println(buf);
        
        radioStats.ackCount = 0;
        radioStats.shortTermAckCount = 0;
        radioStats.lastRateReport = currentMillis;
    }
    
    if (currentMillis - radioStats.lastStatsTime >= radioStats.statsInterval) {
        unsigned long received_delta = radioStats.packets_received_20s;
        unsigned long acks_delta = radioStats.acks_sent_20s;

        char buf[128];
        snprintf(buf, sizeof(buf), "1,%lu,RADIO,20s_stats: rx=%lu/%lu, ack=%lu/%lu",
                 currentMillis, radioStats.packets_received_total, received_delta,
                 radioStats.acks_sent_total, acks_delta);
        Serial.println(buf);

        radioStats.packets_received_20s = 0;
        radioStats.acks_sent_20s = 0;
        radioStats.lastStatsTime = currentMillis;
    }
    
    static unsigned long lastHeartbeat = 0;
    if (currentMillis - lastHeartbeat >= 5000) {
        Serial.println("1,0,SYS,hb");
        lastHeartbeat = currentMillis;
    }
}

// -------------------------------------------------------------------
void setup() {
    delay(45000);  // waiting for the RPi to boot so the serial connection is made
    
    Serial.begin(460800);
    while (!Serial && millis() < 10000);
    Serial.println("Teensy Receiver Starting v20260501...");  // CHANGED: version string
    Serial.flush();

    for(int i = 0; i < 4; i++) {
        Serial.print("Debug message #");
        Serial.println(i);
        Serial.flush();
        delay(100);
    }
    
    Serial.println("If you see this, serial is working!");
    Serial.flush();
    
    Serial.println("*** NEW BUILD: CSV_LOG_ENABLED=0 Timestamp: " __TIMESTAMP__);
    Serial.flush();

    // Print steering calibration so it appears in the monitor on every boot
    // NEW block: confirms pot constants are loaded correctly
    Serial.print("Steer cal: RIGHT="); Serial.print(STEER_POT_RIGHT);
    Serial.print(" CTR=");   Serial.print(STEER_POT_CENTER);
    Serial.print(" LEFT=");  Serial.print(STEER_POT_LEFT);
    Serial.print(" PWM=");   Serial.print(STEER_PWM_MIN);
    Serial.print("..");      Serial.println(STEER_PWM_MAX);
    Serial.flush();
    
    Serial3.begin(JRK_BAUD);

    // === IBT-2 Steering controller SETUP ===
    pinMode(RPWM_Output, OUTPUT);
    pinMode(LPWM_Output, OUTPUT);
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, 0);

    pinMode(ESTOP_RELAY_PIN, OUTPUT);
    digitalWrite(ESTOP_RELAY_PIN, HIGH);

    SPI.setSCK(13);  
    SPI.setMOSI(11);   
    SPI.setMISO(12);

    SPI.begin();
    delay(100);
    
    bool initialized = false;
    for (int i = 0; i < 5; i++) {
        Serial.print("Radio init attempt ");
        Serial.println(i + 1);
        Serial.flush();
        if (radio.begin()) {
            initialized = true;
            Serial.println("Radio initialized successfully!");
            Serial.flush();
            break;
        }
        Serial.println("Radio init failed, retrying...");
        Serial.flush();
        delay(1000);
    }

    if (!initialized) {
        Serial.println("Radio hardware not responding!");
        Serial.flush();
    }
    
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);  
    radio.enableAckPayload();
    radio.setPayloadSize(14);  

    radio.openWritingPipe(NRF24_ADDRESS_RADIO_CONTROL);
    radio.openReadingPipe(1, NRF24_ADDRESS_TRACTOR);
    radio.startListening();

    Serial.println("=== RADIO CONFIGURATION ===");
    Serial.print("Channel: "); Serial.println(radio.getChannel());
    Serial.print("Payload Size: "); Serial.println(radio.getPayloadSize());
    Serial.print("Data Rate: ");
    uint8_t dr = radio.getDataRate();
    if (dr == RF24_250KBPS) Serial.println("250KBPS");
    else if (dr == RF24_1MBPS) Serial.println("1MBPS");
    else Serial.println("2MBPS");
    Serial.print("PA Level: ");
    uint8_t pa = radio.getPALevel();
    if (pa == RF24_PA_MIN) Serial.println("MIN");
    else if (pa == RF24_PA_LOW) Serial.println("LOW");
    else if (pa == RF24_PA_HIGH) Serial.println("HIGH");
    else Serial.println("MAX");

    radio.openReadingPipe(1, NRF24_ADDRESS_TRACTOR);
    Serial.print("Reading Pipe 1 Address: ");
    for(int i = 0; i < 5; i++) {
        Serial.print((char)NRF24_ADDRESS_TRACTOR[i]);
    }
    Serial.println();
    Serial.print("Writing Pipe Address: ");
    for(int i = 0; i < 5; i++) {
        Serial.print((char)NRF24_ADDRESS_RADIO_CONTROL[i]);
    }
    Serial.println();
    Serial.println("=========================");

    memset(&ackPayload, 0, sizeof(AckPayloadStruct));

    Serial.println("10-Bucket Control System:");
    Serial.println("  transmission_val 1023 -> bucket 0 -> JRK 4095 (FULL REVERSE)");
    Serial.println("  transmission_val ~512 -> bucket 5 -> JRK 2800 (NEUTRAL)");
    Serial.println("  transmission_val    1 -> bucket 9 -> JRK    0 (FULL FORWARD)");

    Serial.println("1,0,SYS,start");
    Serial.flush();
}

// -------------------------------------------------------------------
// MAIN LOOP
void loop() {
    currentMillis = millis();

    // 1. Safety
    estopCheck();

    // 2. Serial input
    parseSerialCommand();
    checkCmdVelTimeout();
    monitorSerialBuffer();

    // 3. Radio
    handleRadio();

    // 4. Control (rate limited)
    controlTransmission();
    controlSteering();
}
