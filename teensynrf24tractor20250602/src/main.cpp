#include <SPI.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>

// JRK controller serial port
#define JRK_BAUD 9600

// NeoPixel definitions
#define NUM_LEDS 1
#define DATA_PIN 2
Adafruit_NeoPixel strip(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

#define BLINK_INTERVAL 500 // LED blink interval in milliseconds

RF24 radio(9, 10);  // CE, CSN pins for Teensy 3.5 since JRK G2 motor controller is using 7 and 8

// Data structure for receiving
// struct RadioControlStruct {
//     float steering_val;     // 4 bytes - Pin 15
//     float throttle_val;     // 4 bytes - Pin 14
//     float voltage;          // 4 bytes - TBD
//     float pot3_val;         // 4 bytes - Pin 16
//     float pot4_val;         // 4 bytes - Pin 17
//     byte estop;             // 1 byte - Pin 10
//     byte control_mode;      // 1 byte - Pins 3 and 4
//     byte button01;          // 1 byte - Pin 9
//     byte button02;          // 1 byte - Pin 6
// }; // Total: 24 bytes

// struct __attribute__((packed)) RadioControlStruct {
//     float steering_val;
//     float throttle_val;
//     float voltage;
//     float pot3_val;
//     float pot4_val;
//     byte estop;
//     byte control_mode;
//     byte button01;
//     byte button02;
// };

struct RadioControlStruct {
    float steering_val;
    float throttle_val;
    float voltage;
    float pot3_val;
    float pot4_val;
    byte estop;
    byte control_mode;
    byte button01;
    byte button02;
};

// Data structure for acknowledgment
// struct AckPayloadStruct {
//     unsigned long counter;
//     uint32_t dummy[4];
// };
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
unsigned long lastLedUpdate = 0;
const unsigned long ledUpdateInterval = 500; // Update LEDs every 500ms (2 Hz)
unsigned long lastBlinkUpdate = 0;
unsigned long ackCount = 0;
unsigned long shortTermAckCount = 0;
bool ledState = false;

// Control transmission timing
unsigned long lastControlRun = 0;
const unsigned long controlInterval = 100; // 10 Hz

// JRK helper function declarations
void setJrkTarget(uint16_t target);
void controlTransmission();

// Function to set NeoPixel color
void setNeoPixelColor(uint8_t red, uint8_t green, uint8_t blue) {
    strip.setPixelColor(0, strip.Color(red, green, blue));
    strip.show();
}

void setup() {
    Serial.begin(115200);
    Serial.println("Teensy 3.5 Receiver Starting...");

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
    // radio.enableDynamicPayloads();
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(124);
    // radio.openWritingPipe(address[1]);    // "TRACT" = robot tractor
    // radio.openReadingPipe(1, address[0]); // "RCTRL" = radio control unit

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

void updateLEDs() {
    if (currentMillis - lastLedUpdate >= ledUpdateInterval) {
        float timeElapsed = (currentMillis - lastLedUpdate) / 1000.0;
        float currentRate = shortTermAckCount / timeElapsed;

        Serial.print("Current communication rate: ");
        Serial.print(currentRate);
        Serial.println(" Hz");

        // Only update LED if not in special blink mode
        if (radioData.steering_val != 9999.0) {
            // Set NeoPixel color based on rate
            if (currentRate < 2.0) {
                setNeoPixelColor(255, 0, 0); // Red - poor signal
            } else if (currentRate >= 2.0 && currentRate <= 5.0) {
                setNeoPixelColor(255, 255, 0); // Yellow - moderate signal
            } else {
                setNeoPixelColor(0, 255, 0); // Green - good signal
            }
        }

        shortTermAckCount = 0; // Reset counter
        lastLedUpdate = currentMillis;
    }
}


// void getData() {
//     if (radio.available()) {
//         uint8_t bytes = radio.getDynamicPayloadSize();
//         Serial.print("Received packet, size: ");
//         Serial.println(bytes);
        
//         if (bytes == sizeof(RadioControlStruct)) {
//             radio.read(&radioData, sizeof(RadioControlStruct));

//             // Print received data for debugging
//             Serial.print("Data: steering=");
//             Serial.print(radioData.steering_val, 2);
//             Serial.print(", throttle=");
//             Serial.print(radioData.throttle_val, 2);
//             Serial.print(", pot3=");
//             Serial.print(radioData.pot3_val, 2);
//             Serial.print(", pot4=");
//             Serial.print(radioData.pot4_val, 2);
//             Serial.print(", voltage=");
//             Serial.print(radioData.voltage, 1);
//             Serial.print("V, estop=");
//             Serial.print(radioData.estop);
//             Serial.print(", mode=");
//             Serial.print(radioData.control_mode);
//             Serial.print(", btn01=");
//             Serial.print(radioData.button01);
//             Serial.print(", btn02=");
//             Serial.println(radioData.button02);

//             ackPayload.counter++;
//             radio.writeAckPayload(1, &ackPayload, sizeof(AckPayloadStruct));

//             ackCount++;
//             shortTermAckCount++;

//             // Handle special blink mode
//             if (radioData.steering_val == 9999.0) {
//                 if (currentMillis - lastBlinkUpdate >= BLINK_INTERVAL) {
//                     ledState = !ledState;
//                     if (ledState) {
//                         setNeoPixelColor(0, 0, 255); // Blink blue
//                     } else {
//                         setNeoPixelColor(0, 0, 0); // Turn off
//                     }
//                     lastBlinkUpdate = currentMillis;
//                 }
//             }
//         } else {
//             Serial.print("Wrong payload size, flushing buffer. Expected: ");
//             Serial.print(sizeof(RadioControlStruct));
//             Serial.print(", got: ");
//             Serial.println(bytes);
//             radio.flush_rx();
//         }
//     }
// }

void getData() {
    if (radio.available()) {
        radio.read(&radioData, sizeof(RadioControlStruct));

        // Print received data for debugging
        Serial.print("Data: steering=");
        Serial.print(radioData.steering_val, 2);
        Serial.print(", throttle=");
        Serial.print(radioData.throttle_val, 2);
        Serial.print(", pot3=");
        Serial.print(radioData.pot3_val, 2);
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

        ackPayload.counter++;
        radio.writeAckPayload(1, &ackPayload, sizeof(AckPayloadStruct));

        ackCount++;
        shortTermAckCount++;

        // // Handle special blink mode
        // if (radioData.steering_val == 9999.0) {
        //     if (currentMillis - lastBlinkUpdate >= BLINK_INTERVAL) {
        //         ledState = !ledState;
        //         if (ledState) {
        //             setNeoPixelColor(0, 0, 255); // Blink blue
        //         } else {
        //             setNeoPixelColor(0, 0, 0); // Turn off
        //         }
        //         lastBlinkUpdate = currentMillis;
        //     }
        // }
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

// Apply the throttle value from the radio to the JRK at 10 Hz
void controlTransmission() {
    if (currentMillis - lastControlRun < controlInterval) {
        return;
    }

    // Map throttle_val (-1 to 1) to JRK target range 0-2500
'''
MAX_JRK_TARGET represents the highest JRK controller value you want to send. The JRK accepts 
values up to 4095, but the code may limits it for safer testing.
normalized: radioData.throttle_val comes from the radio and ranges from -1 (full reverse) 
to 1 (full forward). Adding 1 shifts that range to 0-2. Dividing by 2 scales it down to 0-1. 
Now normalized is a percentage (0.0 to 1.0) of the throttle position.
target: The code multiplies normalized by MAX_JRK_TARGET to convert that 0-1 range into 
a 0 - MAX_JRK_TARGET range.  The result is cast to uint16_t (an unsigned 16-bit integer) 
because JRK expects an integer target.
'''
    const float MAX_JRK_TARGET = 2500.0f;
    float normalized = (radioData.throttle_val + 1.0f) / 2.0f;
    uint16_t target = (uint16_t)(normalized * MAX_JRK_TARGET);
    setJrkTarget(target);

    lastControlRun = currentMillis;
}

void loop() {
    currentMillis = millis();
    getData();
    controlTransmission();
    updateLEDs();
    printACKRate();
}
