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
unsigned long steer_last_time = 0;  // CHANGED: More specific naming

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

RF24 radio(9, 10);  // CE, CSN pins for Teensy 3.5 since JRK G2 motor controller is using 7 and 8

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

// Timing variables
unsigned long lastRateCalc = 0;
const unsigned long rateCalcInterval = 10000; // Print rate every 10 seconds
const long minlastNRF24ack = 2000;
unsigned long lastNRF24ackTime = 0;
const unsigned long ledUpdateInterval = 500; // Update LEDs every 500ms (2 Hz)
unsigned long ackCount = 0;
unsigned long shortTermAckCount = 0;
unsigned long lastCommRatePrint = 0;
const unsigned long commRatePrintInterval = 10000; // Throttle rate prints to 10 seconds 0.1 Hz

bool NRF24radioSignalGood = false;

// Control e-stop timing and E-stop relay pin
unsigned long lastEstopCheckRun = 0;
const unsigned long estopCheckInterval = 50; // 20 Hz (50ms)
#define ESTOP_RELAY_PIN 32

// Control how often we print received data
unsigned long lastDataPrint = 0;
const unsigned long dataPrintInterval = 5000; // every 5 seconds

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

// Function declarations
void setJrkTarget(uint16_t target);
void controlTransmission();
void controlSteering();
void debugSerial();

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
      Serial.println("Timeout waiting for feedback");
      return 0xFFFF;  // Error code
    }
  }

  uint8_t low = Serial3.read();
  uint8_t high = Serial3.read();
  return (high << 8) | low;
}

void setup() {
    delay(45000);  // Added this delay to wait for the Rpi to boot up to help ensure serial comms is ready
    Serial.begin(115200);
    while (!Serial && millis() < 10000);  // wait up to 10 seconds for USB serial to initialize 
    Serial.println("Teensy 3.5 Receiver Starting...");
    Serial.flush();

    // Print multiple times to ensure we see something
    for(int i = 0; i < 10; i++) {
        Serial.print("Debug message #");
        Serial.println(i);
        Serial.flush();
        delay(100);
    }
    
    Serial.println("If you see this, serial is working!");
    Serial.flush();
    
    // Continue with your normal setup...
    Serial.println("Teensy 3.5 Receiver continuing...");

    // Initialize IBT-2 pins for steering
    pinMode(RPWM_Output, OUTPUT);
    pinMode(LPWM_Output, OUTPUT);
    analogWrite(RPWM_Output, 0); // Ensure motor is stopped
    analogWrite(LPWM_Output, 0);

    Serial3.begin(JRK_BAUD);

    // Configure alternate SPI pins BEFORE SPI.begin() to avoid on-board LED pin 13
    SPI.setSCK(27);   // Set SCK to pin 27
    SPI.setMOSI(28);  // Set MOSI to pin 28  
    SPI.setMISO(39);  // Set MISO to pin 39

    // Initialize SPI
    SPI.begin();
    delay(100);

    // Try to initialize the radio
    bool initialized = false;
    for (int i = 0; i < 5; i++) {
        Serial.print("Radio initialization attempt ");
        Serial.println(i + 1);
        if (radio.begin()) {
            initialized = true;
            Serial.println("Radio initialized successfully!");
            break;
        }
        Serial.println("Radio init failed, retrying...");
        delay(1000);
    }

    if (!initialized) {
        Serial.println("Radio hardware not responding!");
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
    digitalWrite(ESTOP_RELAY_PIN, HIGH); // Start with e-stop relay off (ignition not grounded)

    lastRateCalc = millis();

    Serial.println("Setup complete - listening for transmissions...");
    Serial.println("10-Bucket Control System:");
    Serial.println("  transmission_val 1023 -> bucket 0 -> JRK 3696 (FULL FORWARD)");
    Serial.println("  transmission_val ~512 -> bucket 5 -> JRK 2048 (NEUTRAL)");
    Serial.println("  transmission_val 1    -> bucket 9 -> JRK 312  (FULL REVERSE)");
}

float calculateSteerPID() {  // CHANGED: Function name from calculateSteeringPID to calculateSteerPID
    unsigned long current_time = millis();
    float dt = (current_time - steer_last_time) / 1000.0; // Convert to seconds
    
    if (dt <= 0) dt = 0.001; // Prevent division by zero
    
    steer_error = steer_setpoint - steer_current;
    steer_error_sum += steer_error * dt;
    float error_rate = (steer_error - steer_last_error) / dt;
    
    // Calculate PID output
    float output = (steer_kp * steer_error) + 
                   (steer_ki * steer_error_sum) + 
                   (steer_kd * error_rate);
    
    steer_last_error = steer_error;
    steer_last_time = current_time;
    
    return output;
}

void debugSerial() {
    static unsigned long lastDebug = 0;
    if (currentMillis - lastDebug > 5000) {  // Every 5 seconds
        Serial.print("info,");
        Serial.println("HEARTBEAT - Serial is working");
        Serial.flush();
        lastDebug = currentMillis;
    }
}

void calcRadioCommRate() {
    if (currentMillis - lastCommRatePrint >= commRatePrintInterval) {
        float timeElapsed = (currentMillis - lastCommRatePrint) / 1000.0;
        float currentRate = shortTermAckCount / timeElapsed;
        Serial.print("info,");
        Serial.print("Current communication rate: ");
        Serial.print(currentRate);
        Serial.println(" Hz");
        shortTermAckCount = 0; // Reset counter
        lastCommRatePrint = currentMillis;
    }
}

void getData() {
    if (radio.available()) {
        radio.read(&radioData, sizeof(RadioControlStruct));

        // Print received data for debugging at 1 Hz
        if (currentMillis - lastDataPrint >= dataPrintInterval) {
            Serial.print("info,");
            Serial.print("Data: steering=");
            Serial.print(radioData.steering_val, 2);
            Serial.print(", throttle=");
            Serial.print(radioData.throttle_val, 2);
            Serial.print(", transmission=");
            Serial.print(radioData.transmission_val, 2);
            Serial.print(", pot4=");
            Serial.print(radioData.pot4_val, 2);
            Serial.print(", voltage=");
            Serial.print(radioData.voltage, 1);
            Serial.print("V, estop=");
            Serial.print(radioData.estop);
            Serial.print(", mode=");
            Serial.print(radioData.control_mode);
            Serial.print(", btn01=");
            Serial.print(radioData.button01);
            Serial.print(", btn02=");
            Serial.println(radioData.button02);
            lastDataPrint = currentMillis;
        }

        ackPayload.counter++;
        radio.writeAckPayload(1, &ackPayload, sizeof(AckPayloadStruct));
        lastNRF24ackTime = currentMillis;
        ackCount++;
        shortTermAckCount++;
        updateRadioSmoothing();
    }
}

void checkNRF24ack(){
  if (currentMillis - lastNRF24ackTime < minlastNRF24ack){
    NRF24radioSignalGood = true;
  } else {
    NRF24radioSignalGood = false;
  }
}

void printACKRate() {
    if (currentMillis - lastRateCalc >= rateCalcInterval) {
        float timeElapsed = (currentMillis - lastRateCalc) / 1000.0;
        float rate = ackCount / timeElapsed;
        Serial.print("info,");
        Serial.print("ACK Rate (Receiver): ");
        Serial.print(rate);
        Serial.print(" Hz (");
        Serial.print(ackCount);
        Serial.println(" ACKs sent in 10 seconds)");

        ackCount = 0;
        lastRateCalc = currentMillis;
        Serial.print("info,");
        Serial.print("RadioControlStruct size: ");
        Serial.println(sizeof(RadioControlStruct));
    }
}

// Send a target position to the JRK controller (using working compact method)
void setJrkTarget(uint16_t target) {
    if (target > 4095) target = 4095;  // Safety limit
    Serial3.write(0xC0 + (target & 0x1F));
    Serial3.write((target >> 5) & 0x7F);
    Serial.print("info,");
    Serial.print("setJrkTarget: ");
    Serial.println(target);    
}

void controlTransmission() {
    if (currentMillis - lastTransmissionControlRun < controlTransmissionInterval) {
        return;
    }

    if (!NRF24radioSignalGood) {radioData.control_mode = 9; } // Safety override if radio signal is lost

    // Case for mode switch
    uint16_t requestedTarget;
    switch (radioData.control_mode) {   // tractor in 'pause' mode
        case 0:
            requestedTarget = transmissionNeutralPos;
            break;

        case 1:                         // tractor in 'manual' mode
            // Use 10-bucket system to calculate bucket (0-9) from transmission_val (1-1023)
            // I should also put a 0.1µF ceramic capacitor between the wiper and ground on the pot to smooth the signal
            if (radioData.transmission_val >= 931) {
                bucket = 0; // Full reverse (CCW) 1023-931
            } else if (radioData.transmission_val >= 838) {
                bucket = 1;
            } else if (radioData.transmission_val >= 746) {
                bucket = 2;
            } else if (radioData.transmission_val >= 654) {
                bucket = 3;
            } else if (radioData.transmission_val >= 562) {
                bucket = 4;
            } else if (radioData.transmission_val >= 469) {
                bucket = 5; // Neutral area
            } else if (radioData.transmission_val >= 377) {
                bucket = 6;
            } else if (radioData.transmission_val >= 285) {
                bucket = 7;
            } else if (radioData.transmission_val >= 192) {
                bucket = 8;
            } else {
                bucket = 9; // Full forward (CW) (transmission_val 1-192)
            }

            requestedTarget = bucketTargets[bucket];
            break;
            
        case 2:                         // tractor in 'auto' mode
            requestedTarget = transmissionNeutralPos;  // Placeholder for cmd_vel
            break;

        default:                         // mode switch in error state
            requestedTarget = transmissionNeutralPos;
            break;
    }

    setJrkTarget(requestedTarget);

    // Print debug output at 2 Hz
    if (currentMillis - lastTargetPrint >= targetPrintInterval) {
        Serial.print("info,");
        Serial.print("control_mode=");
        Serial.print(radioData.control_mode);
        Serial.print(", transmission_val=");
        Serial.print(radioData.transmission_val);
        Serial.print(", bucket=");
        Serial.print(bucket);
        Serial.print(", requestedTarget=");
        Serial.print(requestedTarget);
        Serial.print(", currentTransmissionOutput=");
        Serial.println(currentTransmissionOutput);
        lastTargetPrint = currentMillis;
    }

    // Log for CSV: timestamp, output, feedback, transmission_val, requestedTarget, bucket
    uint16_t feedback = readFeedback();
    Serial.print("log,");
    Serial.print(currentMillis);
    Serial.print(",");
    Serial.print(currentTransmissionOutput);
    Serial.print(",");
    Serial.print(radioData.transmission_val);
    Serial.print(",");
    Serial.print(requestedTarget);
    Serial.print(",");
    Serial.print(bucket);
    Serial.print(",");
    Serial.println(feedback);

    lastTransmissionControlRun = currentMillis;
}

void controlSteering() {
    if (currentMillis - lastSteeringControlRun < controlSteeringInterval) {
        return;
    }

    // Read current steering position from potentiometer
    steer_current = analogRead(STEER_POT_PIN);

    if (!NRF24radioSignalGood) {
        // Stop motor if radio signal is lost
        analogWrite(RPWM_Output, 0);
        analogWrite(LPWM_Output, 0);
        if (currentMillis - lastSteeringPrint >= steeringPrintInterval) {
            Serial.print("info,");
            Serial.print("steering: NO_SIGNAL, pot=");
            Serial.print(steer_current);
            Serial.println(", pwm=0");
            lastSteeringPrint = currentMillis;
        }
        lastSteeringControlRun = currentMillis;
        return;
    }

    int pwmValue = 0;
    String direction = "NEUTRAL";

    switch (radioData.control_mode) {
        case 0: // Pause mode
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            steer_setpoint = steer_current; // Hold current position
            pwmValue = 0;
            direction = "PAUSE";
            break;

case 1: // Manual mode with PID control
            {  // Add curly braces to create scope for case 1
                // Map radio steering value (0-1023) to pot range (0-1023)
                steer_setpoint = radioData.steering_val;
                
                // Calculate PID output
                float steer_pid_output = calculateSteerPID();  // CHANGED: Function name and variable name
                
                // Apply deadband
                if (abs(steer_error) <= STEER_DEADBAND) {
                    pwmValue = 0;
                    direction = "NEUTRAL";
                    analogWrite(RPWM_Output, 0);
                    analogWrite(LPWM_Output, 0);
                } else {
                    // Convert PID output to PWM value (limit to 0-255)
                    pwmValue = constrain(abs(steer_pid_output), 0, 255);  // CHANGED: Variable name
                    
                    if (steer_pid_output > 0) {  // CHANGED: Variable name
                        // Need to turn right (increase pot value)
                        analogWrite(LPWM_Output, 0);       // Turn off reverse
                        analogWrite(RPWM_Output, pwmValue); // Set forward speed
                        direction = "RIGHT";
                    } else {
                        // Need to turn left (decrease pot value)
                        analogWrite(RPWM_Output, 0);       // Turn off forward
                        analogWrite(LPWM_Output, pwmValue); // Set reverse speed
                        direction = "LEFT";
                    }
                }
            }  // Close the scope for case 1
            break;

        case 2: // Auto mode (placeholder for ROS cmd_vel)
            // For now, hold current position
            steer_setpoint = steer_current;
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            pwmValue = 0;
            direction = "AUTO";
            break;

        default: // Error state
            analogWrite(RPWM_Output, 0);
            analogWrite(LPWM_Output, 0);
            pwmValue = 0;
            direction = "ERROR";
            break;
    }

    // Print debug output at 2 Hz
    if (currentMillis - lastSteeringPrint >= steeringPrintInterval) {
        Serial.print("info,");
        Serial.print("mode=");
        Serial.print(radioData.control_mode);
        Serial.print(", setpoint=");
        Serial.print(steer_setpoint);
        Serial.print(", current=");
        Serial.print(steer_current);
        Serial.print(", error=");
        Serial.print(steer_error);
        Serial.print(", direction=");
        Serial.print(direction);
        Serial.print(", pwm=");
        Serial.println(pwmValue);
        lastSteeringPrint = currentMillis;
    }

    lastSteeringControlRun = currentMillis;
}

void estopCheck() {
    if (currentMillis - lastEstopCheckRun < estopCheckInterval) {
        return;
    }
    
    // If estop is high, activate relay (ground the ignition wire)
    if (radioData.estop) {
        digitalWrite(ESTOP_RELAY_PIN, LOW); // Activate relay - grounds ignition wire
    } else {
        digitalWrite(ESTOP_RELAY_PIN, HIGH);  // Deactivate relay - normal ignition operation
    }
    
    lastEstopCheckRun = currentMillis;
}

void debugSteerPot() {
    static unsigned long lastPotDebug = 0;
    if (currentMillis - lastPotDebug >= 1000) {  
        int rawPot = analogRead(STEER_POT_PIN);  
        Serial.print("debug,");
        Serial.print("Raw pot reading: ");
        Serial.print(rawPot);
        Serial.print(", Voltage: ");
        Serial.print((rawPot * 3.3) / 1023.0, 2);
        Serial.println("V");
        lastPotDebug = currentMillis;
    }
}

void loop() {
    currentMillis = millis();
    checkNRF24ack();
    getData();
    controlTransmission();
    controlSteering();
    estopCheck();
    calcRadioCommRate();
    printACKRate();
    debugSerial();
    debugSteerPot();    
}