/*********************************************************************
  teensy_main_20251129.cpp
  --------------------------------------------------------------
  * Date: November 28, 2025
  * FIXED: Moved GPS functions after dependencies (GpsStatus struct and safeTextLog)
*********************************************************************/

#include <SPI.h>
#include <RF24.h>
#include <string.h>  // For memset

// -------------------------------------------------------------------
// JRK controller
#define JRK_BAUD 9600
const uint16_t transmissionNeutralPos = 2985;

// 10-bucket transmission targets
const uint16_t bucketTargets[10] = {
    3696, 3554, 3412, 3270, 3128,
    2985, 2751, 2517, 2283, 2048
};

// IBT-2 steering pins
int RPWM_Output = 5;
int LPWM_Output = 6;

// Steering pot & PID
#define STEER_POT_PIN A9
#define STEER_DEADBAND 10

float steer_kp = 1.0;
float steer_ki = 0.0;
float steer_kd = 0.0;

float steer_setpoint = 512;
float steer_current   = 512;
float steer_error     = 0;
float steer_error_sum = 0;
float steer_last_error = 0;
unsigned long steer_last_time = 0;

const int STEER_MIN_POT = 0;
const int STEER_MAX_POT = 1023;
const int STEER_CENTER_POT = 512;

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
    int16_t steering_val;      // 2B: Pin 15 (was 4B int)
    int16_t throttle_val;      // 2B: Pin 14 (was 4B int)
    int16_t transmission_val;  // 2B: Pin 16 (was 4B int)
    uint16_t voltage_mv;       // 2B: Voltage in millivolts (was 4B float)
    int16_t pot4_val;          // 2B: Pin 17 (was 4B int)
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

// NRF24 addresses to explicit const arrays
// const uint8_t NRF24_ADDRESS_TRACTOR[6]       = "TRACT";
// const uint8_t NRF24_ADDRESS_RADIO_CONTROL[6] = "RCTRL";

const uint8_t NRF24_ADDRESS_TRACTOR[6]       = "1Node";
const uint8_t NRF24_ADDRESS_RADIO_CONTROL[6] = "2Node";

unsigned long currentMillis = 0;

// -------------------------------------------------------------------
// Radio stats (MERGED: Old signalGood + new received/ACK counts)
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
    const unsigned long statsInterval = 20000;  // New: 20s stats
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
const unsigned long targetPrintInterval = 5000;

unsigned long lastTransmissionControlRun = 0;
const unsigned long controlTransmissionInterval = 100;   // 10 Hz

unsigned long lastSteeringControlRun = 0;
const unsigned long controlSteeringInterval = 100;       // 10 Hz (change to 50 for 20 Hz)
unsigned long lastSteeringPrint = 0;
const unsigned long steeringPrintInterval = 2000;

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

// CSV Logging Toggle (if you enable this now you will have to update the teensy serial bridge 
// to know what to do with 'log' statements)  (set to 0 at 3:42pm on 11/28/25)
#define CSV_LOG_ENABLED 0

// -------------------------------------------------------------------
// TEXT LOG RATE LIMITER (human readable)
#define TEXT_LOG_INTERVAL 2000UL
static unsigned long lastTextLog = 0;

void safeTextLog(const char* msg) {
    if (currentMillis - lastTextLog < TEXT_LOG_INTERVAL) return;
    Serial.println(msg);
    Serial.flush();                 // guarantee delivery
    lastTextLog = currentMillis;
}

// -------------------------------------------------------------------
// GPS FUNCTIONS (MOVED HERE - after GpsStatus struct and safeTextLog)
byte getGpsStatus() {
    // Return cached GPS status from bridge
    // Timeout to safe default after 5 seconds of no updates
    unsigned long age = currentMillis - gpsStatus.last_update;
    if (age > 5000) {
        // Add debug for timeout
        static unsigned long lastTimeoutLog = 0;
        if (currentMillis - lastTimeoutLog > 5000) {
            char buf[64];
            snprintf(buf, sizeof(buf), "1,%lu,GPS_TIMEOUT,age=%lu,st=%d",
                     currentMillis, age, gpsStatus.status);
            Serial.println(buf);
            lastTimeoutLog = currentMillis;
        }
        return 1;  // Default to "no fix" on timeout
    }
    return gpsStatus.status;
}

void updateGpsStatus(byte newStatus) {
    gpsStatus.status = newStatus;
    gpsStatus.last_update = currentMillis;  // Use currentMillis for consistency
    gpsStatus.messages_received++;
    
    // Log status changes immediately (not just every 10s)
    static byte lastLoggedStatus = 0;
    if (newStatus != lastLoggedStatus) {
        char buf[64];
        snprintf(buf, sizeof(buf), "1,%lu,GPS_UPDATE,st=%d->%d,cnt=%lu",
                 currentMillis, lastLoggedStatus, newStatus,
                 gpsStatus.messages_received);
        Serial.println(buf);
        lastLoggedStatus = newStatus;
    }
    
    // Periodic log (every 10s)
    if (currentMillis - gpsStatus.last_log >= 10000) {
        char buf[64];
        snprintf(buf, sizeof(buf), "1,%lu,GPS,st=%d,age=%lu,cnt=%lu",
                 currentMillis, gpsStatus.status,
                 currentMillis - gpsStatus.last_update,
                 gpsStatus.messages_received);
        Serial.println(buf);  // Changed from safeTextLog to Serial.println
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
        safeTextLog(buf);
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
            
            // CMD parsing
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

                    // Echo every 50th message (very light)
                    if (cmdVel.message_count % 50 == 0) {
                        char echo[64];
                        snprintf(echo, sizeof(echo),
                                 "3,%lu,CE,x=%.2f,z=%.2f,hz=%.1f",
                                 millis(), lx, az, cmdVel.current_hz);
                        Serial.println(echo);
                    }
                }
            }
            // GPS parsing (ADDED)
            else if (idx >= 4 && memcmp(buffer, "GPS,", 4) == 0) {
                int status;
                if (sscanf(buffer + 4, "%d", &status) == 1) {
                    if (status >= 0 && status <= 3) {  // Validate range
                        updateGpsStatus((byte)status);
                        
                        // Echo every 20th GPS message for monitoring
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

    // Hz calculation
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
    if (avail > 40) {                 // warning threshold
        char msg[64];
        snprintf(msg, sizeof(msg), "1,%lu,SERIAL,buf=%d,WARN", currentMillis, avail);
        safeTextLog(msg);
    }
    last = currentMillis;
}

// -------------------------------------------------------------------
// Transmission control (10 Hz) - UPDATED: transmission_val now int
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
                int tv = (int)radioData.transmission_val;  // CHANGED: Cast int16_t to int
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
                int bucketScaled = (int)scaled;  // Cast to int for thresholds
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

    // ---- Human status (rate limited) ----
    if (currentMillis - lastTargetPrint >= targetPrintInterval) {
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "1,%lu,TRANS,m=%d,b=%d,tgt=%u,cur=%u",
                 currentMillis, radioData.control_mode, bucket,
                 requestedTarget, currentTransmissionOutput);
        safeTextLog(buf);
        lastTargetPrint = currentMillis;
    }

    // ---- TRANS_LOG (only in auto + cmd_vel) ----
    if (currentMillis - lastTransLogPrint >= transLogInterval &&
        radioData.control_mode == 2 && cmdVel.received) {

        uint16_t fb = readFeedback();
        char buf[96];
        snprintf(buf, sizeof(buf),
                 "1,%lu,TL,o=%u,tv=%d,t=%u,b=%d,fb=%u,hz=%.1f,x=%.2f",
                 currentMillis, currentTransmissionOutput,
                 (int)radioData.transmission_val,  // CHANGED: Explicit cast for clarity
                 requestedTarget,
                 bucket, fb, cmdVel.current_hz, cmdVel.linear_x);
        safeTextLog(buf);
        lastTransLogPrint = currentMillis;
    }

    // MERGED FROM OLDER: CSV Logging for data analysis (toggle with CSV_LOG_ENABLED)
    #if CSV_LOG_ENABLED
        uint16_t feedback = readFeedback();
        char csvBuf[128];
        snprintf(csvBuf, sizeof(csvBuf), "log,%lu,%u,%d,%u,%d,%u",
                 currentMillis, currentTransmissionOutput, 
                 (int)radioData.transmission_val,  // CHANGED: Cast int16_t
                 requestedTarget, bucket, feedback);
        Serial.println(csvBuf);
    #endif

    lastTransmissionControlRun = currentMillis;
}

// -------------------------------------------------------------------
// Steering control (10 Hz → 20 Hz safe) - UPDATED: steering_val now int
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

void controlSteering() {
    if (currentMillis - lastSteeringControlRun < controlSteeringInterval) return;

    steer_current = analogRead(STEER_POT_PIN);

    // ---- NO SIGNAL → stop motors ----
    if (!radioStats.signalGood) {
        analogWrite(RPWM_Output, 0);
        analogWrite(LPWM_Output, 0);
        if (currentMillis - lastSteeringPrint >= steeringPrintInterval) {
            char buf[48];
            snprintf(buf, sizeof(buf), "1,%lu,STEER,st=NO_SIG,pot=%.0f",
                     currentMillis, steer_current);
            safeTextLog(buf);
            lastSteeringPrint = currentMillis;
        }
        lastSteeringControlRun = currentMillis;
        //sendSteeringBinary(0);
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
            steer_setpoint = (float)radioData.steering_val;  // Cast int to float

            {
                float out = calculateSteerPID();
                if (abs(steer_error) <= STEER_DEADBAND) {
                    pwmValue = 0;
                    dir = "N";
                    analogWrite(RPWM_Output, 0);
                    analogWrite(LPWM_Output, 0);
                } else {
                    pwmValue = constrain(abs((int)out), 0, 255);
                    if (out > 0) {
                        analogWrite(LPWM_Output, 0);
                        analogWrite(RPWM_Output, pwmValue);
                        dir = "R";
                    } else {
                        analogWrite(RPWM_Output, 0);
                        analogWrite(LPWM_Output, pwmValue);
                        dir = "L";
                    }
                }
            }
            break;

        case 2:  // Auto (cmd_vel)
            if (cmdVel.received) {
                steer_setpoint = cmdVel.angular_z;  // angular_z contains PWM value (0-1023) from Pure Pursuit
                float out = calculateSteerPID();
                if (abs(steer_error) <= STEER_DEADBAND) {
                    pwmValue = 0;
                    dir = "AN";
                    analogWrite(RPWM_Output, 0);
                    analogWrite(LPWM_Output, 0);
                } else {
                    pwmValue = constrain(abs((int)out), 0, 255);
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
        safeTextLog(buf);
        lastSteeringPrint = currentMillis;
    }

    lastSteeringControlRun = currentMillis;
    //sendSteeringBinary(pwmValue);          // <<< BINARY PACKET
}

// -------------------------------------------------------------------
// E-stop check
void estopCheck() {
    if (currentMillis - lastEstopCheckRun < estopCheckInterval) return;
    digitalWrite(ESTOP_RELAY_PIN, radioData.estop ? LOW : HIGH);
    lastEstopCheckRun = currentMillis;
}

// -------------------------------------------------------------------
// MERGED FROM OLDER: Raw pot monitoring (1s interval)
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

// -------------------------------------------------------------------
// Radio handling (MERGED: Old smoothing/prints/signal + new ACK echoes/stats)
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
        
        // NEW: Echo buttons and set GPS in ACK
        ackPayload.button02_status = radioData.button02;
        ackPayload.button03_status = radioData.button03;
        ackPayload.gps_status = getGpsStatus();
        memset(ackPayload.padding, 0, sizeof(ackPayload.padding));  // Ensure padding zeroed

        // new debug
        // ========== ADD ACK WRITE DEBUG ==========
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
        // ========== END DEBUG ==========
        // end of debug        
        
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
    
    // OLD: Compact statistics format (10s rate)
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
    
    // NEW: 20s stats print (from testing)
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
    
    // Heartbeat every 5 seconds
    static unsigned long lastHeartbeat = 0;
    if (currentMillis - lastHeartbeat >= 5000) {
        Serial.println("1,0,SYS,hb");
        lastHeartbeat = currentMillis;
    }
}

// -------------------------------------------------------------------
void setup() {
    delay(45000);  // waiting for the RPi to boot so the serial connection is made
    
    // MERGED FROM OLDER: Verbose serial init for reliability
    Serial.begin(921600);
    while (!Serial && millis() < 10000);  // wait up to 10 seconds for USB serial to initialize 
    Serial.println("Teensy Receiver Starting...");
    Serial.flush();

    // Print multiple times to ensure we see something
    for(int i = 0; i < 4; i++) {
        Serial.print("Debug message #");
        Serial.println(i);
        Serial.flush();
        delay(100);
    }
    
    Serial.println("If you see this, serial is working!");
    Serial.flush();
    
    Serial.println("*** NEW BUILD: CSV_LOG_ENABLED=0 - No more logs! Timestamp: " __TIMESTAMP__);
    Serial.flush();
    
    Serial3.begin(JRK_BAUD);

    // === IBT-2 Steering controller SETUP ===
    pinMode(RPWM_Output, OUTPUT);
    pinMode(LPWM_Output, OUTPUT);
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, 0);

    pinMode(ESTOP_RELAY_PIN, OUTPUT);
    digitalWrite(ESTOP_RELAY_PIN, HIGH);

    // MERGED FROM OLDER: Explicit SPI pins (redundancy)
    SPI.setSCK(13);  
    SPI.setMOSI(11);   
    SPI.setMISO(12);

    // === SPI & RADIO SETUP (UPDATED: Channel 76, payload size) ===
    SPI.begin();  // REQUIRED: initializes SPI0
    delay(100);
    
    // MERGED FROM OLDER: Radio init retries for robustness
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
        // Note: Continue anyway, but monitor logs
    }
    
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);  
    radio.enableAckPayload();
    radio.setPayloadSize(14);  

    radio.openWritingPipe(NRF24_ADDRESS_RADIO_CONTROL);    // Send ACKs to control unit
    radio.openReadingPipe(1, NRF24_ADDRESS_TRACTOR);       // Listen for commands
    radio.startListening();

    // ========== ADD THESE DEBUG LINES ==========
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

    // Print addresses
    //uint8_t addr[6];
    radio.openReadingPipe(1, NRF24_ADDRESS_TRACTOR);  // Re-open to ensure it's set
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
    // ========== END DEBUG LINES ==========


    // Init ACK (zero padding)
    memset(&ackPayload, 0, sizeof(AckPayloadStruct));

    // MERGED FROM OLDER: Print bucket system info
    Serial.println("10-Bucket Control System:");
    Serial.println("  transmission_val 1023 -> bucket 0 -> JRK 3696 (FULL REVERSE)");
    Serial.println("  transmission_val ~512 -> bucket 5 -> JRK 2985 (NEUTRAL)");
    Serial.println("  transmission_val 1    -> bucket 9 -> JRK 2048 (FULL FORWARD)");

    Serial.println("1,0,SYS,start");
    Serial.flush();
}

// -------------------------------------------------------------------
// MAIN LOOP (priority order unchanged)
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

    // MERGED FROM OLDER: Pot monitoring
    debugSteerPot();
}
