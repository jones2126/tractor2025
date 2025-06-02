//
// NRF24L01 Radio Control Unit / Transmitter Code with PL9823 LEDs
//

#include <SPI.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>

RF24 radio(7, 8);  // CE, CSN pins

// PL9823 LED definitions
#define LED_PIN 2
#define NUM_LEDS 4
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

// Data structure for sending
struct RadioControlStruct {
    float steering_val;     // 4 bytes
    float throttle_val;     // 4 bytes
    float voltage;          // 4 bytes
    byte estop;            // 1 byte
    byte control_mode;     // 1 byte
    unsigned long counter; // 4 bytes
    uint32_t dummy;       // 4 bytes added to make total 22 bytes
};

// Data structure for receiving acknowledgment
struct AckPayloadStruct {
    unsigned long counter;  // 4 bytes
    uint32_t dummy[4];     // 18 bytes of dummy data to make total 22 bytes
};

RadioControlStruct radioData;
AckPayloadStruct ackPayload;
const uint8_t address[][6] = {"RCTRL", "TRACT"}; // "RCTRL" = radio control unit, "TRACT" = robot tractor
bool sendTestValue = true;
unsigned long lastTransmit = 0;
const unsigned long transmitInterval = 100;  // Send every 100ms = 10 Hz

// Mode switch pins
const int leftPin = 3;   // Left position of switch
const int rightPin = 4;  // Right position of switch

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

void setup() {
    Serial.begin(115200);
    Serial.println("Transmitter Starting...");
    
    // Initialize PL9823 LEDs
    strip.begin();
    strip.setBrightness(64); // 25% brightness to reduce power demand
    strip.clear();
    strip.show();
    Serial.println("PL9823 LEDs initialized");
    
    // Initialize mode switch pins
    pinMode(leftPin, INPUT_PULLUP);
    pinMode(rightPin, INPUT_PULLUP);
    Serial.println("Mode switch initialized");
    
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
    radio.setChannel(124);     
    radio.openWritingPipe(address[1]);    // "TRACT" = send TO the tractor
    radio.openReadingPipe(1, address[0]); // "RCTRL" = listen for ACKs from control unit

    radio.enableAckPayload();
    radio.stopListening();  // For initializing assume the transmitter will be first to send msgs    
    radio.printDetails();  // Optional but helpful for debugging   
    
    // Initialize dummy data
    radioData.steering_val = 0.0;
    radioData.throttle_val = 0.0;
    radioData.voltage = 12.0;
    radioData.estop = 0;
    radioData.control_mode = 1;
    radioData.counter = 0;
    radioData.dummy = 0xDEADBEEF;

    Serial.println("Setup complete");
    
    // Initialize timing variables
    lastRateCalc = millis();
    lastLedUpdate = millis();
    lastModeCheck = millis();
    
    // Show initialization complete with green LED #1
    strip.setPixelColor(0, colors[1].r, colors[1].g, colors[1].b); // Green
    strip.show();
    delay(1000);
}

void sendData(){
    // Send data every transmitInterval milliseconds
    if (currentMillis - lastTransmit >= transmitInterval) {
        radioData.counter++;
        
        // Set steering value based on test mode
        radioData.steering_val = sendTestValue ? 9999.0 : 0.0;
        
        // Send the data
        bool report = radio.write(&radioData, sizeof(RadioControlStruct));
        
        // Check if we got an acknowledgment payload
        if (report && radio.isAckPayloadAvailable()) {
            radio.read(&ackPayload, sizeof(AckPayloadStruct));
            ackCount++;        // For communication rate calculation
            shortTermAckCount++; // For LED update rate calculation
        }
        
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
            radioData.control_mode = 0;  // Left position
        } else if (rightState == LOW) {
            radioData.control_mode = 1;  // Right position
        } else {
            radioData.control_mode = 2;  // Center position
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
        
        // Set LED #1 (index 0) color based on currentRate
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
        
        // You can use LEDs #2, #3, #4 for other status indicators
        // Example: LED #2 for E-Stop status
        if (radioData.estop) {
            strip.setPixelColor(1, colors[0].r, colors[0].g, colors[0].b); // Red for E-Stop active
        } else {
            strip.setPixelColor(1, 0, 0, 0);   // Off for E-Stop inactive
        }
        
        // Example: LED #3 for control mode
        if (radioData.control_mode == 0) {
            strip.setPixelColor(2, colors[4].r, colors[4].g, colors[4].b); // Purple for LEFT position
        } else if (radioData.control_mode == 1) {
            strip.setPixelColor(2, colors[2].r, colors[2].g, colors[2].b); // Blue for RIGHT position
        } else {
            strip.setPixelColor(2, colors[3].r, colors[3].g, colors[3].b); // Yellow for CENTER position
        }
        
        // Example: LED #4 for voltage status (assuming 12V nominal)
        if (radioData.voltage > 11.5) {
            strip.setPixelColor(3, colors[1].r, colors[1].g, colors[1].b); // Green for good voltage
        } else if (radioData.voltage > 10.5) {
            strip.setPixelColor(3, colors[6].r, colors[6].g, colors[6].b); // Orange for low voltage
        } else {
            strip.setPixelColor(3, colors[0].r, colors[0].g, colors[0].b); // Red for critical voltage
        }
        
        strip.show(); // Update all LEDs
        
        Serial.print(currentRate);
        Serial.println(" Hz)");
        
        shortTermAckCount = 0;  // Reset short-term counter
        lastLedUpdate = currentMillis;        
    }
}

void loop() {
    currentMillis = millis();
    checkModeSW();  // Check mode switch at 10 Hz
    sendData();  
    updateLEDs();
    printACKRate();
}