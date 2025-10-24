#include <SPI.h>
#include <RF24.h>

// JRK controller and transmission values - Updated for 10-bucket system
#define JRK_BAUD 9600
const uint16_t transmissionNeutralPos = 2985;      // Keep neutral in middle

// 10 buckets for smoother control (reverse to forward)
// transmission_val: 1023 = full reverse, 1 = full forward
const uint16_t bucketTargets[10] = {
    3696,   // Bucket 0: transmission_val ~1023 - 931 -> full reverse (CCW)
    3554,   // Bucket 1: transmission_val ~838
    3412,   // Bucket 2: transmission_val ~746
    3270,   // Bucket 3: transmission_val ~654
    3128,   // Bucket 4: transmission_val ~562
    2985,   // Bucket 5: transmission_val ~469 -> neutral
    2751,   // Bucket 6: transmission_val ~377
    2517,   // Bucket 7: transmission_val ~285
    2283,   // Bucket 8: transmission_val ~192
    2048    // Bucket 9: transmission_val ~102-1 -> full forward (CW)
};

// IBT-2 pin definitions for steering
int RPWM_Output = 5; // Connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 6; // Connect to IBT-2 pin 2 (LPWM)

// Steering potentiometer and PID definitions
#define STEER_POT_PIN A9  // pin 23 for steering potentiometer
#define STEER_DEADBAND 10 // Potentiometer deadband (adjust as needed)

// PID variables for steering
float steer_kp = 1.0;     // Proportional gain (tune these values)
float steer_ki = 0.0;     // Integral gain
float steer_kd = 0.0;     // Derivative gain

float steer_setpoint = 512;    // Target steering position (0-1023)
float steer_current = 512;     // Current steering position from pot
float steer_error = 0;
float steer_error_sum = 0;
float steer_last_error = 0;
unsigned long steer_last_time = 0;

// Steering limits (adjust these based on your physical setup)
const int STEER_MIN_POT = 0;     // Minimum pot value (full left)
const int STEER_MAX_POT = 1023;  // Maximum pot value (full right)
const int STEER_CENTER_POT = 512; // Center position

// Transmission variables
uint16_t currentTransmissionOutput = transmissionNeutralPos;  // Start at neutral
const uint8_t transmissionRampStep = 10;  // Max change per update (in JRK units)
int bucket = 5;  // Start at middle bucket (neutral)

// NeoPixel definitions
#define NUM_LEDS 1
#define DATA_PIN 2

RF24 radio(9, 10);  // CE, CSN pins for Teensy 4.1

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

// ============================================================================
// CHANGED: Consolidated radio timing variables
// ============================================================================
struct RadioStats {
    unsigned long lastAckTime = 0;           // Last time we got ACK
    unsigned long ackCount = 0;              // Total ACKs
    unsigned long shortTermAckCount = 0;     // ACKs since last rate calc
    unsigned long lastRateReport = 0;        // Last comprehensive rate report
    unsigned long lastDataPrint = 0;         // Last radio data print
    bool signalGood = false;                 // Current signal status
    
    // Timing intervals
    const unsigned long signalTimeout = 2000;      // 2 seconds
    const unsigned long rateReportInterval = 10000; // 10 seconds
    const unsigned long dataPrintInterval = 5000;   // 5 seconds
} radioStats;

// Control e-stop timing and E-stop relay pin
unsigned long lastEstopCheckRun = 0;
const unsigned long estopCheckInterval = 50; // 20 Hz (50ms)
#define ESTOP_RELAY_PIN 32

// Control how often we print JRK target
unsigned long lastTargetPrint = 0;
const unsigned long targetPrintInterval = 5000; // every 5 seconds

// Control transmission timing
unsigned long lastTransmissionControlRun = 0;
const unsigned long controlTransmissionInterval = 100; // 10 Hz

// Control steering timing
unsigned long lastSteeringControlRun = 0;
const unsigned long controlSteeringInterval = 100; // 10 Hz
unsigned long lastSteeringPrint = 0;
const unsigned long steeringPrintInterval = 2000; // every 2 seconds 

// ============================================================================
// CHANGED: Standardized serial print functions
// ============================================================================
// Message Type 1: Status/Logging - for RPi consumption
void printStatus(const char* subsystem, const char* message) {
    Serial.print("1,");           // Message type
    Serial.print(currentMillis);  // Timestamp
    Serial.print(",");
    Serial.print(subsystem);
    Serial.print(",");
    Serial.println(message);
}

void printStatusKV(const char* subsystem, const char* key1, float val1) {
    Serial.print("1,");
    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(subsystem);
    Serial.print(",");
    Serial.print(key1);
    Serial.print("=");
    Serial.println(val1, 2);
}

void printStatusMultiKV(const char* subsystem) {
    Serial.print("1,");
    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(subsystem);
    Serial.print(",");
}

// Message Type 2: Debug - for development/troubleshooting
void printDebug(const char* subsystem, const char* message) {
    Serial.print("2,");
    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(subsystem);
    Serial.print(",");
    Serial.println(message);
}

void printDebugKV(const char* subsystem, const char* key, float value) {
    Serial.print("2,");
    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(subsystem);
    Serial.print(",");
    Serial.print(key);
    Serial.print("=");
    Serial.println(value, 2);
}

// Function declarations
void setJrkTarget(uint16_t target);
void controlTransmission();
void controlSteering();
void handleRadio();  // CHANGED: New consolidated function

float smoothedRadioVal = 0.0f;
const float radioAlpha = 0.1f;  // Smaller = smoother

void updateRadioSmoothing() {
  smoothedRadioVal = radioAlpha * radioData.transmission_val +
                     (1.0f - radioAlpha) * smoothedRadioVal;
}

uint16_t readFeedback() {
  Serial3.write(0xE5);  // Command: Get variables
  Serial3.write(0x04);  // Offset for Feedback
  Serial3.write(0x02);  // Length = 2 bytes

  unsigned long start = millis();
  while (Serial3.available() < 2) {
    if (millis() - start > 100) {
      printDebug("JRK", "timeout_waiting_feedback");
      return 0xFFFF;  // Error code
    }
  }

  uint8_t low = Serial3.read();
  uint8_t high = Serial3.read();
  return (high << 8) | low;
}

void setup() {
    delay(45000);  // Wait for RPi to boot
    Serial.begin(115200);
    while (!Serial && millis() < 10000);  // wait up to 10 seconds
    
    printStatus("SYSTEM", "teensy_starting");

    // Print multiple times to ensure we see something
    for(int i = 0; i < 10; i++) {
        char msg[32];
        snprintf(msg, sizeof(msg), "debug_msg_%d", i);
        printDebug("SYSTEM", msg);
        Serial.flush();
        delay(100);
    }
    
    printStatus("SYSTEM", "serial_confirmed");
    
    // Initialize IBT-2 pins for steering
    pinMode(RPWM_Output, OUTPUT);
    pinMode(LPWM_Output, OUTPUT);
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, 0);

    Serial3.begin(JRK_BAUD);

    // Configure SPI pins
    SPI.setSCK(13);  
    SPI.setMOSI(11);   
    SPI.setMISO(12);

    // Initialize SPI
    SPI.begin();
    delay(100);

    // Try to initialize the radio
    bool initialized = false;
    for (int i = 0; i < 5; i++) {
        char msg[32];
        snprintf(msg, sizeof(msg), "radio_init_attempt_%d", i + 1);
        printStatus("RADIO", msg);
        
        if (radio.begin()) {
            initialized = true;
            printStatus("RADIO", "initialized_success");
            break;
        }
        printStatus("RADIO", "init_failed_retry");
        delay(1000);
    }

    if (!initialized) {
        printStatus("RADIO", "hardware_not_responding");
    }
    
    // Configure the radio
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(124);
    radio.openWritingPipe(address[0]);    // "RCTRL" = send ACKs back to control unit
    radio.openReadingPipe(1, address[1]); // "TRACT" = listen for data sent TO tractor
    radio.enableAckPayload(); 
    radio.startListening();
    radio.printDetails();

    // Initialize counters
    ackPayload.counter = 0;
    for (int i = 0; i < 4; i++) {
        ackPayload.dummy[i] = 0xDEADBEEF;
    }

    // Initialize e-stop relay pin
    pinMode(ESTOP_RELAY_PIN, OUTPUT);
    digitalWrite(ESTOP_RELAY_PIN, HIGH); // Start with e-stop relay off

    printStatus("SYSTEM", "setup_complete");
    printStatus("SYSTEM", "bucket_system_ready");
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
// CHANGED: New consolidated radio handler
// ============================================================================
void handleRadio() {
    // Check for incoming data
    if (radio.available()) {
        radio.read(&radioData, sizeof(RadioControlStruct));
        
        // Print received data periodically
        if (currentMillis - radioStats.lastDataPrint >= radioStats.dataPrintInterval) {
            printStatusMultiKV("RADIO");
            Serial.print("steer=");
            Serial.print(radioData.steering_val, 2);
            Serial.print(",throttle=");
            Serial.print(radioData.throttle_val, 2);
            Serial.print(",trans=");
            Serial.print(radioData.transmission_val, 2);
            Serial.print(",volt=");
            Serial.print(radioData.voltage, 1);
            Serial.print(",estop=");
            Serial.print(radioData.estop);
            Serial.print(",mode=");
            Serial.println(radioData.control_mode);
            radioStats.lastDataPrint = currentMillis;
        }
        
        // Update ACK payload and send
        ackPayload.counter++;
        radio.writeAckPayload(1, &ackPayload, sizeof(AckPayloadStruct));
        
        // Update timing and counters
        radioStats.lastAckTime = currentMillis;
        radioStats.ackCount++;
        radioStats.shortTermAckCount++;
        
        updateRadioSmoothing();
    }
    
    // Check signal quality based on last ACK time
    radioStats.signalGood = (currentMillis - radioStats.lastAckTime < radioStats.signalTimeout);
    
    // Periodic rate reporting
    if (currentMillis - radioStats.lastRateReport >= radioStats.rateReportInterval) {
        float timeElapsed = (currentMillis - radioStats.lastRateReport) / 1000.0;
        float ackRate = radioStats.ackCount / timeElapsed;
        float currentRate = radioStats.shortTermAckCount / timeElapsed;
        
        // Print comprehensive radio statistics
        printStatusMultiKV("RADIO");
        Serial.print("ack_rate=");
        Serial.print(ackRate, 1);
        Serial.print(",current_rate=");
        Serial.print(currentRate, 1);
        Serial.print(",total_acks=");
        Serial.print(radioStats.ackCount);
        Serial.print(",signal=");
        Serial.print(radioStats.signalGood ? "GOOD" : "LOST");
        Serial.print(",struct_size=");
        Serial.println(sizeof(RadioControlStruct));
        
        // Reset counters
        radioStats.ackCount = 0;
        radioStats.shortTermAckCount = 0;
        radioStats.lastRateReport = currentMillis;
    }
    
    // Heartbeat every 5 seconds when no other messages
    static unsigned long lastHeartbeat = 0;
    if (currentMillis - lastHeartbeat >= 5000) {
        printStatus("SYSTEM", "heartbeat");
        lastHeartbeat = currentMillis;
    }
}

// Send a target position to the JRK controller
void setJrkTarget(uint16_t target) {
    if (target > 4095) target = 4095;
    Serial3.write(0xC0 + (target & 0x1F));
    Serial3.write((target >> 5) & 0x7F);
    
    static unsigned long lastJrkPrint = 0;
    if (currentMillis - lastJrkPrint >= 5000) {
        printStatusKV("JRK", "target", target);
        lastJrkPrint = currentMillis;
    }
}

void controlTransmission() {
    if (currentMillis - lastTransmissionControlRun < controlTransmissionInterval) {
        return;
    }

    if (!radioStats.signalGood) {
        radioData.control_mode = 9;  // Safety override
    }

    uint16_t requestedTarget;
    switch (radioData.control_mode) {
        case 0:  // Pause mode
            requestedTarget = transmissionNeutralPos;
            break;

        case 1:  // Manual mode
            // Use 10-bucket system
            if (radioData.transmission_val >= 931) {
                bucket = 0;
            } else if (radioData.transmission_val >= 838) {
                bucket = 1;
            } else if (radioData.transmission_val >= 746) {
                bucket = 2;
            } else if (radioData.transmission_val >= 654) {
                bucket = 3;
            } else if (radioData.transmission_val >= 562) {
                bucket = 4;
            } else if (radioData.transmission_val >= 469) {
                bucket = 5;
            } else if (radioData.transmission_val >= 377) {
                bucket = 6;
            } else if (radioData.transmission_val >= 285) {
                bucket = 7;
            } else if (radioData.transmission_val >= 192) {
                bucket = 8;
            } else {
                bucket = 9;
            }
            requestedTarget = bucketTargets[bucket];
            break;
            
        case 2:  // Auto mode
            requestedTarget = transmissionNeutralPos;  // Placeholder for cmd_vel
            break;

        default:  // Error state
            requestedTarget = transmissionNeutralPos;
            break;
    }

    setJrkTarget(requestedTarget);

    // Print periodic status
    if (currentMillis - lastTargetPrint >= targetPrintInterval) {
        printStatusMultiKV("TRANS");
        Serial.print("mode=");
        Serial.print(radioData.control_mode);
        Serial.print(",bucket=");
        Serial.print(bucket);
        Serial.print(",target=");
        Serial.print(requestedTarget);
        Serial.print(",current=");
        Serial.println(currentTransmissionOutput);
        lastTargetPrint = currentMillis;
    }

    // CSV log for analysis
    uint16_t feedback = readFeedback();
    printStatusMultiKV("TRANS_LOG");
    Serial.print("output=");
    Serial.print(currentTransmissionOutput);
    Serial.print(",trans_val=");
    Serial.print(radioData.transmission_val);
    Serial.print(",target=");
    Serial.print(requestedTarget);
    Serial.print(",bucket=");
    Serial.print(bucket);
    Serial.print(",feedback=");
    Serial.println(feedback);

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
            printStatusMultiKV("STEER");
            Serial.print("status=NO_SIGNAL,pot=");
            Serial.print(steer_current);
            Serial.println(",pwm=0");
            lastSteeringPrint = currentMillis;
        }
        lastSteeringControlRun = currentMillis;
        return;
    }

    int pwmValue = 0;
    String direction = "NEUTRAL";

    switch (radioData.control_mode) {
        case 0: // Pause
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            steer_setpoint = steer_current;
            direction = "PAUSE";
            break;

        case 1: // Manual with PID
            {
                steer_setpoint = radioData.steering_val;
                float steer_pid_output = calculateSteerPID();
                
                if (abs(steer_error) <= STEER_DEADBAND) {
                    pwmValue = 0;
                    direction = "NEUTRAL";
                    analogWrite(RPWM_Output, 0);
                    analogWrite(LPWM_Output, 0);
                } else {
                    pwmValue = constrain(abs(steer_pid_output), 0, 255);
                    
                    if (steer_pid_output > 0) {
                        analogWrite(LPWM_Output, 0);
                        analogWrite(RPWM_Output, pwmValue);
                        direction = "RIGHT";
                    } else {
                        analogWrite(RPWM_Output, 0);
                        analogWrite(LPWM_Output, pwmValue);
                        direction = "LEFT";
                    }
                }
            }
            break;

        case 2: // Auto mode
            steer_setpoint = steer_current;
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            direction = "AUTO";
            break;

        default:
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            direction = "ERROR";
            break;
    }

    if (currentMillis - lastSteeringPrint >= steeringPrintInterval) {
        printStatusMultiKV("STEER");
        Serial.print("mode=");
        Serial.print(radioData.control_mode);
        Serial.print(",setpt=");
        Serial.print(steer_setpoint);
        Serial.print(",current=");
        Serial.print(steer_current);
        Serial.print(",error=");
        Serial.print(steer_error);
        Serial.print(",dir=");
        Serial.print(direction);
        Serial.print(",pwm=");
        Serial.println(pwmValue);
        lastSteeringPrint = currentMillis;
    }

    lastSteeringControlRun = currentMillis;
}

void estopCheck() {
    if (currentMillis - lastEstopCheckRun < estopCheckInterval) {
        return;
    }
    
    if (radioData.estop) {
        digitalWrite(ESTOP_RELAY_PIN, LOW);  // Activate relay
    } else {
        digitalWrite(ESTOP_RELAY_PIN, HIGH);
    }
    
    lastEstopCheckRun = currentMillis;
}

// ============================================================================
// CHANGED: Optional debug function for pot reading
// ============================================================================
void debugSteerPot() {
    static unsigned long lastPotDebug = 0;
    if (currentMillis - lastPotDebug >= 1000) {  
        int rawPot = analogRead(STEER_POT_PIN);
        printDebugKV("STEER_POT", "raw", rawPot);
        printDebugKV("STEER_POT", "voltage", (rawPot * 3.3) / 1023.0);
        lastPotDebug = currentMillis;
    }
}

// ============================================================================
// CHANGED: Simplified main loop
// ============================================================================
void loop() {
    currentMillis = millis();
    
    handleRadio();           // CHANGED: Single consolidated function
    controlTransmission();
    controlSteering();
    estopCheck();
    
    // Uncomment for pot debugging:
    // debugSteerPot();
}
