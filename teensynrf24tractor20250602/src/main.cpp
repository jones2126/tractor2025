#include <SPI.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>

// JRK controller and transmission values
#define JRK_BAUD 9600
const uint16_t transmissionFullReversePos = 500;
const uint16_t transmissionFullForwardPos = 3000;
const uint16_t transmissionNeutralPos = 1200;

// NeoPixel definitions
#define NUM_LEDS 1
#define DATA_PIN 2
Adafruit_NeoPixel strip(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

#define BLINK_INTERVAL 500 // LED blink interval in milliseconds

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
const long lastNRF24ackTime = 0;
unsigned long lastLedUpdate = 0;
const unsigned long ledUpdateInterval = 500; // Update LEDs every 500ms (2 Hz)
unsigned long lastBlinkUpdate = 0;
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

// Function to set NeoPixel color
void setNeoPixelColor(uint8_t red, uint8_t green, uint8_t blue) {
    strip.setPixelColor(0, strip.Color(red, green, blue));
    strip.show();
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

    // Initialize NeoPixel
    strip.begin();
    strip.show(); // Turn off all LEDs initially

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
        while (1) {
            setNeoPixelColor(255, 0, 0); // Indicate failure with red color
            delay(500);
            setNeoPixelColor(0, 0, 0); // Turn off
            delay(500);
        }
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
    lastLedUpdate = millis();
    lastBlinkUpdate = millis();

    Serial.println("Setup complete - listening for transmissions...");
}

void debugSerial() {
    static unsigned long lastDebug = 0;
    if (currentMillis - lastDebug > 5000) {  // Every 5 seconds
        Serial.println("HEARTBEAT - Serial is working");
        Serial.flush();
        lastDebug = currentMillis;
    }
}

void calcRadioCommRate() {
    if (currentMillis - lastCommRatePrint >= commRatePrintInterval) {
        float timeElapsed = (currentMillis - lastCommRatePrint) / 1000.0;
        float currentRate = shortTermAckCount / timeElapsed;
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

        Serial.print("ACK Rate (Receiver): ");
        Serial.print(rate);
        Serial.print(" Hz (");
        Serial.print(ackCount);
        Serial.println(" ACKs sent in 10 seconds)");

        ackCount = 0;
        lastRateCalc = currentMillis;
        Serial.print("RadioControlStruct size: ");
        Serial.println(sizeof(RadioControlStruct));
    }
}

// Send a target position to the JRK controller
void setJrkTarget(uint16_t target) {
    Serial3.write(0xC0);
    Serial3.write(target & 0x1F);
    Serial3.write((target >> 5) & 0x7F);
}

void controlTransmission() {
    if (currentMillis - lastTransmissionControlRun < controlTransmissionInterval) {
        return;
    }

    if (NRF24radioSignalGood == false){
        radioData.control_mode = 9;
    }
    uint16_t targetValue;

    switch (radioData.control_mode) {
        case 0:
            // Pause mode
            targetValue = transmissionNeutralPos;
            break;

        case 1:
            // Radio control mode
            targetValue = map(
                radioData.throttle_val,
                0, 4095,
                transmissionFullReversePos,
                transmissionFullForwardPos
            );
            break;

        case 2:
            // Future autonomous/cmd_vel mode — for now, neutral
            targetValue = transmissionNeutralPos;
            break;

        default:
            // Error condition: fallback to neutral
            targetValue = transmissionNeutralPos;
            break;
    }

    // Smooth acceleration ramping: move toward target gradually
    if (targetValue > currentTransmissionValue + rampStep) {
        currentTransmissionValue += rampStep;
    } else if (targetValue < currentTransmissionValue - rampStep) {
        currentTransmissionValue -= rampStep;
    } else {
        currentTransmissionValue = targetValue;  // Close enough
    }

    // Send updated value to JRK
    setJrkTarget(currentTransmissionValue);

    // Print debug output at 2 Hz
    if (currentMillis - lastTargetPrint >= targetPrintInterval) {
        Serial.print("control_mode=");
        Serial.print(radioData.control_mode);
        Serial.print(", throttle_val=");
        Serial.print(radioData.throttle_val);
        Serial.print(", targetValue=");
        Serial.print(targetValue);
        Serial.print(", currentTransmissionValue=");
        Serial.println(currentTransmissionValue);
        lastTargetPrint = currentMillis;
    }

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
