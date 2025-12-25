//
// NRF24L01 Radio Control Unit / Transmitter Code with PL9823 LEDs
// - See RadioControlStruct for hardware pin assignments.
// - LEDs: #1=Signal, #2=E-stop, #3=Mode, #4=GPS Status.
//

#include <SPI.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>

RF24 radio(7, 8);  // CE, CSN pins

// PL9823 LED definitions
#define LED_PIN 2
#define NUM_LEDS 4  // Signal, E-stop, Mode, GPS
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_RGB + NEO_KHZ800);

// Color array for different patterns
struct Color {
  uint8_t r, g, b;
  const char* name;
};

// Define color palette
Color colors[] = {
  {255, 0, 0, "Red"},
  {0, 255, 0, "Green"},
  {0, 0, 255, "Blue"},
  {255, 255, 0, "Yellow"},
  {255, 0, 255, "Purple"},
  {0, 255, 255, "Cyan"},
  {255, 128, 0, "Orange"},
  {128, 0, 128, "Violet"}
};
int numColors = sizeof(colors) / sizeof(Color);

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

// Data structure for receiving acknowledgment 
struct __attribute__((packed)) AckPayloadStruct {
    byte gps_status;         // 1B: 0=unset,1=noNMEA,2=noRTK,3=RTK Fix
    byte button02_status;    // 1B: Echo of received button02
    byte button03_status;    // 1B: Echo of received button03
    byte padding[11];        // 11B: Pad to 14 bytes total
};

RadioControlStruct radioData;
AckPayloadStruct ackPayload;

// NRF24 addresses explicit const arrays 
// const uint8_t NRF24_ADDRESS_TRACTOR[6]       = "TRACT";
// const uint8_t NRF24_ADDRESS_RADIO_CONTROL[6] = "RCTRL";

const uint8_t NRF24_ADDRESS_TRACTOR[6]       = "1Node";
const uint8_t NRF24_ADDRESS_RADIO_CONTROL[6] = "2Node";

// Hardware Pin definitions
const int steeringPin = 16;      // Analog for steering
const int throttlePin = 14;      // Analog for throttle  
const int voltagePin = 18;       // Analog for voltage (TBD - placeholder; adjust scaling as needed)
const int transmissionPin = 15;  // Analog for transmission
const int pot4Pin = 17;          // Analog for pot4
const int estopPin = 10;         // Digital for e-stop (active LOW with pullup)
const int button02Pin = 9;       // Digital for button02 (active LOW with pullup)
const int button03Pin = 6;       // Digital for button03 (active LOW with pullup)
const int leftPin = 3;           // Mode switch left
const int rightPin = 4;          // Mode switch right

unsigned long lastTransmit = 0;
const unsigned long transmitInterval = 100;  // Send every 100ms = 10 Hz
//const unsigned long transmitInterval = 500;  // Send every 500ms = 2 Hz

// Variables for ACK rate calculation
unsigned long currentMillis = 0;
unsigned long lastRateCalc = 0;
const unsigned long rateCalcInterval = 10000;  // Print rate every 10 seconds
unsigned long lastLedUpdate = 0;
const unsigned long ledUpdateInterval = 500;   // Update LEDs every 500ms (2 Hz)
unsigned long lastModeCheck = 0;
const unsigned long modeCheckInterval = 100;   // Check mode switch every 100ms (10 Hz)
unsigned long ackCount = 0;
unsigned long shortTermAckCount = 0;
float currentRate = 0.0;

// ========== NEW: Voltage scaling constants (add after line ~95) ==========
const float VOLTAGE_DIVIDER_RATIO = 5.0;  // Adjust based on your resistor divider (e.g., 40k/10k = 5.0)
const float ADC_REF_VOLTAGE = 3.3;        // Teensy ADC reference voltage

void blinkStartup() {
  // Blink all LEDs once different colors
  for (int i = 0; i < numColors && i < NUM_LEDS; i++) {
    strip.clear();
    strip.setPixelColor(i, colors[i].r, colors[i].g, colors[i].b);
    strip.show();
    delay(300);
  }
  strip.clear();
  strip.show();
}

void setup() {
    Serial.begin(115200);
    Serial.println("Production Radio Controller Starting...");
    
    // Initialize PL9823 LEDs
    strip.begin();
    strip.setBrightness(64); // 25% brightness to reduce power demand
    strip.clear();
    strip.show();
    Serial.println("PL9823 LEDs initialized");
    blinkStartup();

    
    // Initialize input pins (pull-ups for buttons/switches)
    pinMode(leftPin, INPUT_PULLUP);
    pinMode(rightPin, INPUT_PULLUP);
    pinMode(estopPin, INPUT_PULLUP);
    pinMode(button02Pin, INPUT_PULLUP);
    pinMode(button03Pin, INPUT_PULLUP);
    Serial.println("Input pins initialized");
    
    // Initialize SPI manually first
    SPI.begin();
    delay(100);
    
    // Try to initialize radio multiple times
    bool initialized = false;
    for(int i = 0; i < 5; i++) {
        if (radio.begin()) {
            initialized = true;
            Serial.println("Radio initialized!");
            break;
        }
        Serial.println("Radio init failed. Retrying...");
        delay(1000);
    }
    
    if (!initialized) {
        Serial.println("Radio hardware not responding!");
        while (1) {
            // Flash LED #1 red to indicate failure
            strip.setPixelColor(0, colors[0].r, colors[0].g, colors[0].b); // Red
            strip.show();
            delay(500);
            strip.setPixelColor(0, 0, 0, 0); // Off
            strip.show();
            delay(500);
        }
    }
    
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);  // Production: Match Teensy channel
    radio.setPayloadSize(14); 

    radio.openWritingPipe(NRF24_ADDRESS_TRACTOR);           // Send data to the tractor
    radio.openReadingPipe(1, NRF24_ADDRESS_RADIO_CONTROL);  // Listen for ACKs

    radio.enableAckPayload();
    radio.stopListening();  // Transmitter mode

    // ========== ADD THESE DEBUG LINES ==========
    Serial.println("=== RADIO CONFIGURATION ===");
    Serial.print("Channel: "); Serial.println(radio.getChannel());
    Serial.print("Payload Size: "); Serial.println(radio.getPayloadSize());
    Serial.print("Data Rate: ");
    uint8_t dr = radio.getDataRate();
    if (dr == RF24_250KBPS) Serial.println("250KBPS");
    else if (dr == RF24_1MBPS) Serial.println("1MBPS");
    else Serial.println("2MBPS");

    Serial.print("Writing to Address: ");
    for(int i = 0; i < 5; i++) {
        Serial.print((char)NRF24_ADDRESS_TRACTOR[i]);
    }
    Serial.println();
    Serial.print("Listening on Pipe 1: ");
    for(int i = 0; i < 5; i++) {
        Serial.print((char)NRF24_ADDRESS_RADIO_CONTROL[i]);
    }
    Serial.println();
    Serial.println("=========================");
    // ========== END DEBUG LINES ==========


    radio.printDetails();  // Optional but helpful for debugging   
    
    // ========== Lines 177-185: CHANGED initialization values to match new types ==========
    radioData.steering_val = 512;       // int16_t: OK as-is
    radioData.throttle_val = 512;       // int16_t: OK as-is
    radioData.voltage_mv = 12000;       // CHANGED: 12.0V → 12000 millivolts
    radioData.transmission_val = 512;   // int16_t: OK as-is
    radioData.pot4_val = 0;             // int16_t: OK as-is
    radioData.estop = 0;
    radioData.control_mode = 1;
    radioData.button02 = 0;
    radioData.button03 = 0;

    Serial.println("Setup complete - Production mode active");
    
    // Initialize timing variables
    lastRateCalc = millis();
    lastLedUpdate = millis();
    lastModeCheck = millis();
    
    // Show initialization complete with green LED #1
    strip.setPixelColor(0, colors[1].r, colors[1].g, colors[1].b); // Green
    strip.show();
    delay(1000);
    strip.clear();
    strip.show();
}

void sendData(){
    // Send data every transmitInterval milliseconds (10 Hz)
    if (currentMillis - lastTransmit >= transmitInterval) {
        
        // ========== Lines 207-210: CHANGED to cast to int16_t (values already 0-1024 range) ==========
        radioData.steering_val = (int16_t)map(analogRead(steeringPin), 0, 1023, 0, 1024);
        radioData.throttle_val = (int16_t)map(analogRead(throttlePin), 0, 1023, 0, 1024);
        radioData.transmission_val = (int16_t)map(analogRead(transmissionPin), 0, 1023, 0, 1024);
        radioData.pot4_val = (int16_t)map(analogRead(pot4Pin), 0, 1023, 0, 1024);

        // ========== Lines 215-216: CHANGED voltage to millivolts (uint16_t) ==========
        // Voltage: Convert ADC reading to millivolts
        // Example: 12.6V battery → (12.6V / 5.0) = 2.52V on ADC pin
        //          2.52V / 3.3V * 1023 = 781 ADC counts
        //          (781 * 3.3 / 1023) * 5.0 * 1000 = 12600 millivolts
        float voltage_float = (analogRead(voltagePin) * ADC_REF_VOLTAGE / 1023.0) * VOLTAGE_DIVIDER_RATIO;
        radioData.voltage_mv = (uint16_t)(voltage_float * 1000.0);  // Convert to millivolts
        // REMOVE THIS LINE when voltage divider is connected:
        // radioData.voltage_mv = 12600;  // TBD: Placeholder 12.6V until voltage divider connected
        
        // Digital inputs (active LOW with pull-ups)
        radioData.estop = (digitalRead(estopPin) == LOW) ? 1 : 0;
        radioData.button02 = (digitalRead(button02Pin) == LOW) ? 1 : 0;
        radioData.button03 = (digitalRead(button03Pin) == LOW) ? 1 : 0;

        // Control mode will be updated in checkModeSW()
        
        // Send the data
        radio.stopListening();  // Ensure PTX mode
        bool report = radio.write(&radioData, sizeof(RadioControlStruct));
// start new code
        // ========== ADD COMPREHENSIVE ACK DEBUG ==========
        static unsigned long lastAckDebug = 0;
        
        if (!report) {
            Serial.print("TX FAILED at ");
            Serial.println(currentMillis);
        } else {
            // Transmission succeeded
            if (radio.isAckPayloadAvailable()) {
                // ACK payload is available
                uint8_t pipeNum;
                uint8_t payloadSize = radio.getPayloadSize();
                
                // Read the ACK
                radio.read(&ackPayload, sizeof(AckPayloadStruct));
                
                ackCount++;
                shortTermAckCount++;
                
                // Debug every 20th ACK (reduce spam)
                if (ackCount % 20 == 0 || currentMillis - lastAckDebug > 5000) {
                    Serial.println("===== ACK PAYLOAD DEBUG =====");
                    Serial.print("ACK Size: "); Serial.println(payloadSize);
                    Serial.print("GPS Status: "); Serial.println(ackPayload.gps_status);
                    Serial.print("Button02 Echo: "); Serial.println(ackPayload.button02_status);
                    Serial.print("Button03 Echo: "); Serial.println(ackPayload.button03_status);
                    Serial.print("Padding[0]: "); Serial.println(ackPayload.padding[0]);
                    Serial.print("Padding[1]: "); Serial.println(ackPayload.padding[1]);
                    Serial.println("============================");
                    lastAckDebug = currentMillis;
                }
            } else {
                // TX succeeded but no ACK payload
                if (currentMillis - lastAckDebug > 5000) {
                    Serial.println("TX OK but NO ACK PAYLOAD");
                    lastAckDebug = currentMillis;
                }
            }
        }
        // ========== END ACK DEBUG ==========        

// end new code        
        lastTransmit = currentMillis;
    }    
}

void printACKRate(){
    // Calculate and display rate every 10 seconds
    if (currentMillis - lastRateCalc >= rateCalcInterval) {
        float timeElapsed = (currentMillis - lastRateCalc) / 1000.0;  // Convert to seconds
        float rate = ackCount / timeElapsed;  // Calculate Hz
        
        Serial.print("ACK Rate: ");
        Serial.print(rate);
        Serial.print(" Hz (");
        Serial.print(ackCount);
        Serial.println(" ACKs in 10 seconds)");
        
        // Reset long-term counter
        ackCount = 0;
        lastRateCalc = currentMillis;
    }
}

void checkModeSW() {
    // Check mode switch every 100ms (10 Hz)
    if (currentMillis - lastModeCheck >= modeCheckInterval) {
        int leftState = digitalRead(leftPin);
        int rightState = digitalRead(rightPin);
        
        if (leftState == LOW) {
            radioData.control_mode = 0;  // Pause/Left position
        } else if (rightState == LOW) {
            radioData.control_mode = 1;  // Manual/Right position
        } else {
            radioData.control_mode = 2;  // Auto/Center position
        }
        
        lastModeCheck = currentMillis;
    }
}

void updateLEDs() {
    // Update LEDs every 500ms (2 Hz)
    if (currentMillis - lastLedUpdate >= ledUpdateInterval) {
        float timeElapsed = (currentMillis - lastLedUpdate) / 1000.0;  // Convert to seconds
        currentRate = shortTermAckCount / timeElapsed;  // Calculate Hz

        // Clear all LEDs first
        strip.clear();
        
        // LED #1: (top left)  Signal quality based on ACK rate; Green when tractor radio is working (ACK rate good)       
        if (currentRate < 2.0) {
            // Poor signal - Red
            strip.setPixelColor(0, colors[0].r, colors[0].g, colors[0].b);
            Serial.print("Signal: POOR (");
        } else if (currentRate >= 2.0 && currentRate <= 5.0) {
            // Moderate signal - Orange
            strip.setPixelColor(0, colors[6].r, colors[6].g, colors[6].b);
            Serial.print("Signal: MODERATE (");
        } else {
            // Good signal - Green
            strip.setPixelColor(0, colors[1].r, colors[1].g, colors[1].b);
            Serial.print("Signal: GOOD (");
        }
        
        // LED #2: E-Stop status
        if (radioData.estop) {
            strip.setPixelColor(1, colors[0].r, colors[0].g, colors[0].b); // Red for E-Stop active
        } else {
            strip.setPixelColor(1, 0, 0, 0);   // Off for E-Stop inactive
        }
        
        // LED #3: Control mode - (top right)
        if (radioData.control_mode == 0) {
            strip.setPixelColor(2, colors[4].r, colors[4].g, colors[4].b); // Purple for Pause
        } else if (radioData.control_mode == 1) {
            strip.setPixelColor(2, colors[2].r, colors[2].g, colors[2].b); // Blue for Manual
        } else {
            strip.setPixelColor(2, colors[3].r, colors[3].g, colors[3].b); // Yellow for Auto
        }
        
        // ========== Lines 325-333: CHANGED voltage display to convert from millivolts ==========
        // LED #4: Voltage status (assuming 12V nominal)
        // float voltage_display = radioData.voltage_mv / 1000.0;  // Convert millivolts to volts
        // if (voltage_display > 11.5) {
        //     strip.setPixelColor(3, colors[1].r, colors[1].g, colors[1].b); // Green for good voltage
        // } else if (voltage_display > 10.5) {
        //     strip.setPixelColor(3, colors[6].r, colors[6].g, colors[6].b); // Orange for low voltage
        // } else {
        //     strip.setPixelColor(3, colors[0].r, colors[0].g, colors[0].b); // Red for critical voltage
        // }
        
        // LED #4: GPS Status from latest ACK
        byte gps = ackPayload.gps_status;
        if (gps == 3) {
            strip.setPixelColor(3, colors[1].r, colors[1].g, colors[1].b);  // Green: RTK Fix
        } else if (gps == 2) {
            strip.setPixelColor(3, colors[3].r, colors[3].g, colors[3].b);  // Yellow: GPS no RTK
        } else {
            strip.setPixelColor(3, colors[0].r, colors[0].g, colors[0].b);  // Red: Bad/No GPS
        }
        
        strip.show(); // Update all LEDs
        
        // ========== Line 348: CHANGED voltage display to show millivolts/volts ==========
        Serial.print(currentRate);
        Serial.print(" Hz) | GPS=");
        Serial.print(gps);
        Serial.print(" | Volt=");
        Serial.print(radioData.voltage_mv / 1000.0, 2);  // CHANGED: Convert millivolts to volts for display
        Serial.print("V | Mode=");
        Serial.println(radioData.control_mode);
        
        shortTermAckCount = 0;  // Reset short-term counter
        lastLedUpdate = currentMillis;        
    }
}

void loop() {
    currentMillis = millis();
    checkModeSW();  // Check mode switch at 10 Hz
    sendData();     // Send data at 10 Hz
    updateLEDs();   // Update LEDs at 2 Hz
    printACKRate(); // Print ACK rate every 10s
}