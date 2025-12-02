/*********************************************************************
  teensy_main_20251028.cpp
  --------------------------------------------------------------
  * Binary steering packet (5 B) every control cycle
  * Human logs ≤ 1 line / 2 s
  * Serial @ 921600 baud
*********************************************************************/

#include <SPI.h>
#include <RF24.h>

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
const uint8_t transmissionRampStep = 10;
int bucket = 5;

// NeoPixel (not used)
#define NUM_LEDS 1
#define DATA_PIN 2

RF24 radio(9, 10);

// -------------------------------------------------------------------
// Radio structures
struct RadioControlStruct {
    float steering_val;
    float throttle_val;
    float voltage;
    float transmission_val;
    float pot4_val;
    byte estop;
    byte control_mode;
    byte button01;
    byte button02;
};

struct __attribute__((packed)) AckPayloadStruct {
    unsigned long counter;
    uint32_t dummy[4];
};

RadioControlStruct radioData;
AckPayloadStruct ackPayload;

const uint8_t address[][6] = {"TRACT", "RCTRL"};
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

    const unsigned long signalTimeout = 2000;
    const unsigned long rateReportInterval = 10000;
    const unsigned long dataPrintInterval = 5000;
} radioStats;


float smoothedRadioVal = 0.0f;
const float radioAlpha = 0.1f;

void updateRadioSmoothing() {
    smoothedRadioVal = radioAlpha * radioData.transmission_val +
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
// BINARY STEERING PACKET (5 bytes, every steering cycle)
void sendSteeringBinary(int pwmValue) {
    int16_t pot = (int16_t)steer_current;
    int16_t err = (int16_t)constrain((int)steer_error, -512, 511);
    uint8_t pwm = (uint8_t)pwmValue;

    uint8_t pkt[5] = {
        0xAA,                     // start byte
        (uint8_t)(pot >> 8),
        (uint8_t)(pot & 0xFF),
        (uint8_t)err,
        pwm
    };
    Serial.write(pkt, sizeof(pkt));   // no flush – keep streaming
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
// Serial command parsing (CMD,<x>,<z>\n)
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
                float tv = radioData.transmission_val;
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
                if (scaled >= 931) bucketTmp = 0;
                else if (scaled >= 838) bucketTmp = 1;
                else if (scaled >= 746) bucketTmp = 2;
                else if (scaled >= 654) bucketTmp = 3;
                else if (scaled >= 562) bucketTmp = 4;
                else if (scaled >= 469) bucketTmp = 5;
                else if (scaled >= 377) bucketTmp = 6;
                else if (scaled >= 285) bucketTmp = 7;
                else if (scaled >= 192) bucketTmp = 8;
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
                 "1,%lu,TL,o=%u,tv=%.0f,t=%u,b=%d,fb=%u,hz=%.1f,x=%.2f",
                 currentMillis, currentTransmissionOutput,
                 radioData.transmission_val, requestedTarget,
                 bucket, fb, cmdVel.current_hz, cmdVel.linear_x);
        safeTextLog(buf);
        lastTransLogPrint = currentMillis;
    }

    lastTransmissionControlRun = currentMillis;
}

// -------------------------------------------------------------------
// Steering control (10 Hz → 20 Hz safe)
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
        sendSteeringBinary(0);
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
            steer_setpoint = radioData.steering_val;
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
                steer_setpoint = cmdVel.angular_z;
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
    sendSteeringBinary(pwmValue);          // <<< BINARY PACKET
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
        radio.read(&radioData, sizeof(RadioControlStruct));
        
        // CHANGED: Compact format with single println (was 7 separate prints)
        if (currentMillis - radioStats.lastDataPrint >= radioStats.dataPrintInterval) {
            char buf[80];
            snprintf(buf, sizeof(buf), "1,%lu,RADIO,s=%.2f,t=%.2f,x=%.0f,v=%.1f,e=%d,m=%d",
                     currentMillis,
                     radioData.steering_val,
                     radioData.throttle_val,
                     radioData.transmission_val,
                     radioData.voltage,
                     radioData.estop,
                     radioData.control_mode);
            Serial.println(buf);
            radioStats.lastDataPrint = currentMillis;
        }
        
        ackPayload.counter++;
        radio.writeAckPayload(1, &ackPayload, sizeof(AckPayloadStruct));
        
        radioStats.lastAckTime = currentMillis;
        radioStats.ackCount++;
        radioStats.shortTermAckCount++;
        
        updateRadioSmoothing();
    }
    
    radioStats.signalGood = (currentMillis - radioStats.lastAckTime < radioStats.signalTimeout);
    
    // CHANGED: Compact statistics format
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
    Serial.begin(921600);
    Serial.println("1,0,SYS,starting");    
    Serial3.begin(JRK_BAUD);

    // === IBT-2 Steering controller SETUP ===
    pinMode(RPWM_Output, OUTPUT);
    pinMode(LPWM_Output, OUTPUT);
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, 0);


    pinMode(ESTOP_RELAY_PIN, OUTPUT);
    digitalWrite(ESTOP_RELAY_PIN, HIGH);

    // === SPI & RADIO SETUP ===
    SPI.begin();  // REQUIRED: initializes SPI0
    delay(100);
    if (!radio.begin()) {
        safeTextLog("1,0,RADIO,init_fail");
        while (1) delay(1000);
    }    

    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(124);
    radio.enableAckPayload();

    radio.openWritingPipe(address[1]);    // "RCTRL"
    radio.openReadingPipe(1, address[0]); //  "TRACT" 
    radio.startListening();


    safeTextLog("1,0,SYS,start");
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
}