#include <SPI.h>
#include <RF24.h>

// JRK controller and transmission values - Updated for 10-bucket system
#define JRK_BAUD 9600
const uint16_t transmissionNeutralPos = 2985;

const uint16_t bucketTargets[10] = {
    3696,   // Bucket 0: full reverse (CCW)
    3554,   // Bucket 1
    3412,   // Bucket 2
    3270,   // Bucket 3
    3128,   // Bucket 4
    2985,   // Bucket 5: neutral
    2751,   // Bucket 6
    2517,   // Bucket 7
    2283,   // Bucket 8
    2048    // Bucket 9: full forward (CW)
};

// IBT-2 pin definitions for steering
int RPWM_Output = 5;
int LPWM_Output = 6;

// Steering potentiometer and PID definitions
#define STEER_POT_PIN A9
#define STEER_DEADBAND 10

// PID variables for steering
float steer_kp = 1.0;
float steer_ki = 0.0;
float steer_kd = 0.0;

float steer_setpoint = 512;
float steer_current = 512;
float steer_error = 0;
float steer_error_sum = 0;
float steer_last_error = 0;
unsigned long steer_last_time = 0;

const int STEER_MIN_POT = 0;
const int STEER_MAX_POT = 1023;
const int STEER_CENTER_POT = 512;

// Transmission variables
uint16_t currentTransmissionOutput = transmissionNeutralPos;
const uint8_t transmissionRampStep = 10;
int bucket = 5;

// NeoPixel definitions
#define NUM_LEDS 1
#define DATA_PIN 2

RF24 radio(9, 10);

// Data structure for receiving
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

// Data structure for acknowledgment
struct __attribute__((packed)) AckPayloadStruct {
    unsigned long counter;
    uint32_t dummy[4];
};

RadioControlStruct radioData;
AckPayloadStruct ackPayload;

const uint8_t address[][6] = {"RCTRL", "TRACT"};
unsigned long currentMillis = 0;

// Radio timing variables
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

// E-stop timing and relay pin
unsigned long lastEstopCheckRun = 0;
const unsigned long estopCheckInterval = 50; // 20 Hz
#define ESTOP_RELAY_PIN 32

// Control timing
unsigned long lastTargetPrint = 0;
const unsigned long targetPrintInterval = 5000;

unsigned long lastTransmissionControlRun = 0;
const unsigned long controlTransmissionInterval = 100; // 10 Hz

unsigned long lastSteeringControlRun = 0;
const unsigned long controlSteeringInterval = 100; // 10 Hz
unsigned long lastSteeringPrint = 0;
const unsigned long steeringPrintInterval = 2000;

// ============================================================================
// CRITICAL: cmd_vel structure with rate tracking
// ============================================================================
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

// ============================================================================
// CRITICAL: Serial processing rate limiting (from previous testing)
// ============================================================================
unsigned long lastSerialProcessTime = 0;
const unsigned long serialProcessInterval = 10;  // Process every 10ms (100Hz max)
const unsigned long maxSerialProcessTime = 5;    // Max 5ms per call

// Serial buffer monitoring
struct SerialStats {
    unsigned long maxBufferSeen = 0;
    unsigned long overrunCount = 0;
    unsigned long lastWarning = 0;
} serialStats;

// TRANS_LOG frequency - reduced from 10Hz to 1Hz
unsigned long lastTransLogPrint = 0;
const unsigned long transLogInterval = 1000;

// Function declarations
void setJrkTarget(uint16_t target);
void controlTransmission();
void controlSteering();
void handleRadio();
void monitorSerialBuffer();

float smoothedRadioVal = 0.0f;
const float radioAlpha = 0.1f;

void updateRadioSmoothing() {
    smoothedRadioVal = radioAlpha * radioData.transmission_val +
                      (1.0f - radioAlpha) * smoothedRadioVal;
}

uint16_t readFeedback() {
    Serial3.write(0xE5);
    Serial3.write(0x04);
    Serial3.write(0x02);

    unsigned long start = millis();
    while (Serial3.available() < 2) {
        if (millis() - start > 100) {
            return 0xFFFF;
        }
    }

    uint8_t low = Serial3.read();
    uint8_t high = Serial3.read();
    return (high << 8) | low;
}

// ============================================================================
// CRITICAL: Optimized parseSerialCommand with time limits
// Based on testing: process 1 byte per loop, time-boxed
// ============================================================================
void parseSerialCommand() {
    // Rate limit serial processing
    if (currentMillis - lastSerialProcessTime < serialProcessInterval) {
        return;
    }
    
    unsigned long startTime = millis();
    static char buffer[64];
    static uint8_t bufferIndex = 0;
    static unsigned long lastRateCalc = 0;
    static unsigned long msgCountSinceCalc = 0;
    
    // CRITICAL: Process max 128 bytes per call, time-limited to 5ms
    int bytesProcessed = 0;
    const int maxBytesPerCall = 128;
    
    while (Serial.available() > 0 && 
           bytesProcessed < maxBytesPerCall && 
           (millis() - startTime) < maxSerialProcessTime) {
        
        char c = Serial.read();
        bytesProcessed++;
        
        if (c == '\n') {
            buffer[bufferIndex] = '\0';
            
            // Only process CMD messages
            if (bufferIndex >= 4 && 
                buffer[0] == 'C' && 
                buffer[1] == 'M' && 
                buffer[2] == 'D' && 
                buffer[3] == ',') {
                
                float linear_x, angular_z;
                if (sscanf(buffer, "CMD,%f,%f", &linear_x, &angular_z) == 2) {
                    cmdVel.linear_x = linear_x;
                    cmdVel.angular_z = angular_z;
                    cmdVel.timestamp = millis();
                    cmdVel.received = true;
                    cmdVel.message_count++;
                    msgCountSinceCalc++;
                    last_cmd_vel_time = millis();
                    
                    // Echo only every 50th message (98% reduction in echo traffic)
                    if (cmdVel.message_count % 50 == 0) {
                        char buf[48];
                        snprintf(buf, sizeof(buf), "3,%lu,CE,x=%.2f,z=%.2f,hz=%.1f",
                                 millis(), linear_x, angular_z, cmdVel.current_hz);
                        Serial.println(buf);
                    }
                }
            }
            bufferIndex = 0;
        } else if (bufferIndex < 63) {
            buffer[bufferIndex++] = c;
        } else {
            // Buffer overflow - reset
            bufferIndex = 0;
            serialStats.overrunCount++;
            if (currentMillis - serialStats.lastWarning > 5000) {
                Serial.println("2,0,SERIAL,buffer_overflow");
                serialStats.lastWarning = currentMillis;
            }
        }
    }
    
    // Calculate receive rate every second
    if (currentMillis - lastRateCalc >= 1000) {
        float elapsed = (currentMillis - lastRateCalc) / 1000.0;
        cmdVel.current_hz = msgCountSinceCalc / elapsed;
        msgCountSinceCalc = 0;
        lastRateCalc = currentMillis;
    }
    
    lastSerialProcessTime = currentMillis;
}

// ============================================================================
// NEW: Monitor serial buffer health
// ============================================================================
void monitorSerialBuffer() {
    static unsigned long lastMonitor = 0;
    if (currentMillis - lastMonitor >= 1000) {
        int available = Serial.available();
        
        if ((unsigned long)available > serialStats.maxBufferSeen) {
            serialStats.maxBufferSeen = (unsigned long)available;
        }
        
        // Warn if buffer >75% full
        if (available > 48) {
            char buf[48];
            snprintf(buf, sizeof(buf), "1,%lu,SERIAL,buf=%d,max=%lu",
                     currentMillis, available, serialStats.maxBufferSeen);
            Serial.println(buf);
        }
        
        lastMonitor = currentMillis;
    }
}

void checkCmdVelTimeout() {
    if (cmdVel.received && (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT)) {
        cmdVel.linear_x = 0.0;
        cmdVel.angular_z = 0.0;
        cmdVel.received = false;
        cmdVel.current_hz = 0.0;
        Serial.println("1,0,CMD,timeout");
    }
}

void setup() {
    delay(45000);
    Serial.begin(115200);
    while (!Serial && millis() < 10000);

    Serial.println("1,0,SYS,starting");

    for(int i = 0; i < 5; i++) {
        char buf[32];
        snprintf(buf, sizeof(buf), "2,%lu,SYS,dbg_%d", millis(), i);
        Serial.println(buf);
        Serial.flush();
        delay(100);
    }
    
    Serial.println("1,0,SYS,serial_ok");
    
    pinMode(RPWM_Output, OUTPUT);
    pinMode(LPWM_Output, OUTPUT);
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, 0);

    Serial3.begin(JRK_BAUD);

    SPI.setSCK(13);  
    SPI.setMOSI(11);   
    SPI.setMISO(12);

    SPI.begin();
    delay(100);

    bool initialized = false;
    for (int i = 0; i < 5; i++) {
        char buf[32];
        snprintf(buf, sizeof(buf), "1,%lu,RADIO,init_%d", millis(), i + 1);
        Serial.println(buf);
        
        if (radio.begin()) {
            initialized = true;
            Serial.println("1,0,RADIO,ok");
            break;
        }
        delay(1000);
    }

    if (!initialized) {
        Serial.println("1,0,RADIO,failed");
    }
    
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(124);
    
    radio.openWritingPipe(address[0]);
    radio.openReadingPipe(1, address[1]);

    radio.enableAckPayload(); 
    radio.startListening();
    radio.printDetails();

    ackPayload.counter = 0;
    for (int i = 0; i < 4; i++) {
        ackPayload.dummy[i] = 0xDEADBEEF;
    }

    pinMode(ESTOP_RELAY_PIN, OUTPUT);
    digitalWrite(ESTOP_RELAY_PIN, HIGH);

    Serial.println("1,0,SYS,ready");
    Serial.println("1,0,SYS,20hz_opt");
}

float calculateSteerPID() {
    unsigned long current_time = millis();
    float dt = (current_time - steer_last_time) / 1000.0;
    
    if (dt <= 0) dt = 0.001;
    
    steer_error = steer_setpoint - steer_current;
    steer_error_sum += steer_error * dt;
    float error_rate = (steer_error - steer_last_error) / dt;
    
    float output = (steer_kp * steer_error) + 
                   (steer_ki * steer_error_sum) + 
                   (steer_kd * error_rate);
    
    steer_last_error = steer_error;
    steer_last_time = current_time;
    
    return output;
}

// ============================================================================
// CRITICAL CHANGE: Compact message format using snprintf()
// Single Serial.println() instead of multiple Serial.print() calls
// ============================================================================
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

void setJrkTarget(uint16_t target) {
    if (target > 4095) target = 4095;
    Serial3.write(0xC0 + (target & 0x1F));
    Serial3.write((target >> 5) & 0x7F);
    
    static unsigned long lastJrkPrint = 0;
    if (currentMillis - lastJrkPrint >= 5000) {
        char buf[32];
        snprintf(buf, sizeof(buf), "1,%lu,JRK,tgt=%u", currentMillis, target);
        Serial.println(buf);
        lastJrkPrint = currentMillis;
    }
}

void controlTransmission() {
    if (currentMillis - lastTransmissionControlRun < controlTransmissionInterval) {
        return;
    }

    if (!radioStats.signalGood) {
        radioData.control_mode = 9;
    }

    uint16_t requestedTarget;
    switch (radioData.control_mode) {
        case 0:  // Pause
            requestedTarget = transmissionNeutralPos;
            break;

        case 1:  // Manual
            if (radioData.transmission_val >= 931) bucket = 0;
            else if (radioData.transmission_val >= 838) bucket = 1;
            else if (radioData.transmission_val >= 746) bucket = 2;
            else if (radioData.transmission_val >= 654) bucket = 3;
            else if (radioData.transmission_val >= 562) bucket = 4;
            else if (radioData.transmission_val >= 469) bucket = 5;
            else if (radioData.transmission_val >= 377) bucket = 6;
            else if (radioData.transmission_val >= 285) bucket = 7;
            else if (radioData.transmission_val >= 192) bucket = 8;
            else bucket = 9;
            requestedTarget = bucketTargets[bucket];
            break;
            
        case 2:  // Auto mode with cmd_vel
            if (cmdVel.received) {
                float scaled_val = (cmdVel.linear_x + 1.0) * 511.5;
                if (scaled_val >= 931) bucket = 0;
                else if (scaled_val >= 838) bucket = 1;
                else if (scaled_val >= 746) bucket = 2;
                else if (scaled_val >= 654) bucket = 3;
                else if (scaled_val >= 562) bucket = 4;
                else if (scaled_val >= 469) bucket = 5;
                else if (scaled_val >= 377) bucket = 6;
                else if (scaled_val >= 285) bucket = 7;
                else if (scaled_val >= 192) bucket = 8;
                else bucket = 9;
                requestedTarget = bucketTargets[bucket];
            } else {
                requestedTarget = transmissionNeutralPos;
            }
            break;

        default:  // Error
            requestedTarget = transmissionNeutralPos;
            break;
    }

    setJrkTarget(requestedTarget);

    // CHANGED: Compact status format
    if (currentMillis - lastTargetPrint >= targetPrintInterval) {
        char buf[56];
        snprintf(buf, sizeof(buf), "1,%lu,TRANS,m=%d,b=%d,tgt=%u,cur=%u",
                 currentMillis, radioData.control_mode, bucket,
                 requestedTarget, currentTransmissionOutput);
        Serial.println(buf);
        lastTargetPrint = currentMillis;
    }

    // CRITICAL: Reduced from 10Hz to 1Hz (90% reduction in traffic)
    if (currentMillis - lastTransLogPrint >= transLogInterval) {
        uint16_t feedback = readFeedback();
        char buf[80];
        
        if (radioData.control_mode == 2) {
            // Include cmd_vel data in auto mode
            snprintf(buf, sizeof(buf), "1,%lu,TL,o=%u,tv=%.0f,t=%u,b=%d,fb=%u,hz=%.1f,x=%.2f",
                     currentMillis, currentTransmissionOutput, radioData.transmission_val,
                     requestedTarget, bucket, feedback, cmdVel.current_hz, cmdVel.linear_x);
        } else {
            snprintf(buf, sizeof(buf), "1,%lu,TL,o=%u,tv=%.0f,t=%u,b=%d,fb=%u",
                     currentMillis, currentTransmissionOutput, radioData.transmission_val,
                     requestedTarget, bucket, feedback);
        }
        Serial.println(buf);
        
        lastTransLogPrint = currentMillis;
    }

    lastTransmissionControlRun = currentMillis;
}

void controlSteering() {
    if (currentMillis - lastSteeringControlRun < controlSteeringInterval) {
        return;
    }

    steer_current = analogRead(STEER_POT_PIN);

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
    const char* direction = "N";  // N=neutral, L=left, R=right, P=pause

    switch (radioData.control_mode) {
        case 0:  // Pause
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            steer_setpoint = steer_current;
            direction = "P";
            break;

        case 1:  // Manual with PID
            {
                steer_setpoint = radioData.steering_val;
                float steer_pid_output = calculateSteerPID();
                
                if (abs(steer_error) <= STEER_DEADBAND) {
                    pwmValue = 0;
                    direction = "N";
                    analogWrite(RPWM_Output, 0);
                    analogWrite(LPWM_Output, 0);
                } else {
                    pwmValue = constrain(abs(steer_pid_output), 0, 255);
                    
                    if (steer_pid_output > 0) {
                        analogWrite(LPWM_Output, 0);
                        analogWrite(RPWM_Output, pwmValue);
                        direction = "R";
                    } else {
                        analogWrite(RPWM_Output, 0);
                        analogWrite(LPWM_Output, pwmValue);
                        direction = "L";
                    }
                }
            }
            break;

        case 2:  // Auto mode with cmd_vel
            if (cmdVel.received) {
                steer_setpoint = cmdVel.angular_z;
                float steer_pid_output = calculateSteerPID();
                
                if (abs(steer_error) <= STEER_DEADBAND) {
                    pwmValue = 0;
                    direction = "AN";  // Auto-neutral
                    analogWrite(RPWM_Output, 0);
                    analogWrite(LPWM_Output, 0);
                } else {
                    pwmValue = constrain(abs(steer_pid_output), 0, 255);
                    
                    if (steer_pid_output > 0) {
                        analogWrite(LPWM_Output, 0);
                        analogWrite(RPWM_Output, pwmValue);
                        direction = "AR";  // Auto-right
                    } else {
                        analogWrite(RPWM_Output, 0);
                        analogWrite(LPWM_Output, pwmValue);
                        direction = "AL";  // Auto-left
                    }
                }
            } else {
                steer_setpoint = steer_current;
                analogWrite(RPWM_Output, 0);
                analogWrite(LPWM_Output, 0);
                direction = "AH";  // Auto-hold
            }
            break;

        default:
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            direction = "E";  // Error
            break;
    }

    // CHANGED: Compact steering status
    if (currentMillis - lastSteeringPrint >= steeringPrintInterval) {
        char buf[80];
        
        if (radioData.control_mode == 2) {
            snprintf(buf, sizeof(buf), "1,%lu,STEER,m=%d,sp=%.0f,c=%.0f,e=%.0f,d=%s,p=%d,z=%.2f",
                     currentMillis, radioData.control_mode, steer_setpoint,
                     steer_current, steer_error, direction, pwmValue, cmdVel.angular_z);
        } else {
            snprintf(buf, sizeof(buf), "1,%lu,STEER,m=%d,sp=%.0f,c=%.0f,e=%.0f,d=%s,p=%d",
                     currentMillis, radioData.control_mode, steer_setpoint,
                     steer_current, steer_error, direction, pwmValue);
        }
        Serial.println(buf);
        
        lastSteeringPrint = currentMillis;
    }

    lastSteeringControlRun = currentMillis;
}

void estopCheck() {
    if (currentMillis - lastEstopCheckRun < estopCheckInterval) {
        return;
    }
    
    if (radioData.estop) {
        digitalWrite(ESTOP_RELAY_PIN, LOW);
    } else {
        digitalWrite(ESTOP_RELAY_PIN, HIGH);
    }
    
    lastEstopCheckRun = currentMillis;
}

// ============================================================================
// CRITICAL: Reordered loop() based on testing results
// PRIORITY 1: Safety checks FIRST (every loop)
// PRIORITY 2: Serial communication
// PRIORITY 3: Control loops
// ============================================================================
void loop() {
    currentMillis = millis();
    
    // PRIORITY 1: Safety first - check every loop iteration
    estopCheck();
    
    // PRIORITY 2: Serial processing (rate-limited internally)
    parseSerialCommand();
    checkCmdVelTimeout();
    monitorSerialBuffer();
    
    // PRIORITY 3: Radio communication
    handleRadio();
    
    // PRIORITY 4: Control loops (10Hz rate-limited)
    controlTransmission();
    controlSteering();
}
