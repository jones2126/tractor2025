#include <SPI.h>
#include <RF24.h>

// JRK controller and transmission values - Updated for 10-bucket system
#define JRK_BAUD 9600
const uint16_t transmissionNeutralPos = 2048;      // Keep neutral in middle

// 10 buckets for smoother control (reverse to forward)
// transmission_val: 1023 = full reverse, 1 = full forward
const uint16_t bucketTargets[10] = {
    3696,  // Bucket 0: transmission_val ~1023 -> full forward
    3358,  // Bucket 1: transmission_val ~920
    3020,  // Bucket 2: transmission_val ~818
    2682,  // Bucket 3: transmission_val ~716
    2344,  // Bucket 4: transmission_val ~614
    2048,  // Bucket 5: transmission_val ~512 -> neutral
    1710,  // Bucket 6: transmission_val ~410
    1372,  // Bucket 7: transmission_val ~307
    1034,  // Bucket 8: transmission_val ~205
    312    // Bucket 9: transmission_val ~102-1 -> full reverse
};

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

// Control how often we print received data
unsigned long lastDataPrint = 0;
const unsigned long dataPrintInterval = 2000; // 0.5 Hz

// Control how often we print JRK target
unsigned long lastTargetPrint = 0;
const unsigned long targetPrintInterval = 2000; // 0.5 Hz

// Control transmission timing
unsigned long lastTransmissionControlRun = 0;
const unsigned long controlTransmissionInterval = 100; // 10 Hz

// function declarations
void setJrkTarget(uint16_t target);
void controlTransmission();
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

    lastRateCalc = millis();

    Serial.println("Setup complete - listening for transmissions...");
    Serial.println("10-Bucket Control System:");
    Serial.println("  transmission_val 1023 -> bucket 0 -> JRK 3696 (FULL FORWARD)");
    Serial.println("  transmission_val ~512 -> bucket 5 -> JRK 2048 (NEUTRAL)");
    Serial.println("  transmission_val 1    -> bucket 9 -> JRK 312  (FULL REVERSE)");
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

    // Case for mode swith
    uint16_t requestedTarget;
    switch (radioData.control_mode) {   // tractor in 'pause' mode
        case 0:
            requestedTarget = transmissionNeutralPos;
            break;

        case 1:                         // tractor in 'manual' mode
            // Use 10-bucket system to calculate bucket (0-9) from transmission_val (1-1023)
            // I should also put a 0.1µF ceramic capacitor between the wiper and ground on the pot to smooth the signal
            if (radioData.transmission_val >= 920) {
                bucket = 0; // Full forward
            } else if (radioData.transmission_val >= 818) {
                bucket = 1;
            } else if (radioData.transmission_val >= 716) {
                bucket = 2;
            } else if (radioData.transmission_val >= 614) {
                bucket = 3;
            } else if (radioData.transmission_val >= 512) {
                bucket = 4;
            } else if (radioData.transmission_val >= 410) {
                bucket = 5; // Neutral area
            } else if (radioData.transmission_val >= 307) {
                bucket = 6;
            } else if (radioData.transmission_val >= 205) {
                bucket = 7;
            } else if (radioData.transmission_val >= 102) {
                bucket = 8;
            } else {
                bucket = 9; // Full reverse (transmission_val 1-101)
            }

            requestedTarget = bucketTargets[bucket];
            break;
            
        case 2:                         // tractor in 'auto' mode
        // Placeholder for until I get cmd_vel working
            requestedTarget = transmissionNeutralPos;  // Placeholder for cmd_vel
            break;

        default:                         // mode switch in error state
            requestedTarget = transmissionNeutralPos;
            break;
    }

    // // Smooth ramping toward requested target
    // if (requestedTarget > currentTransmissionOutput + transmissionRampStep) {
    //     currentTransmissionOutput += transmissionRampStep;
    // } else if (requestedTarget < currentTransmissionOutput - transmissionRampStep) {
    //     currentTransmissionOutput -= transmissionRampStep;
    // } else {
    //     currentTransmissionOutput = requestedTarget;
    // }

    // Send smoothed value to JRK
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

void loop() {
    currentMillis = millis();
    checkNRF24ack();
    getData();
    controlTransmission();
    calcRadioCommRate();
    printACKRate();
    debugSerial();
}