#include <Arduino.h>

// Pin definitions
const int POT_PIN = 27;
const int LED_PIN = LED_BUILTIN;

// Relay pins
const int RELAY4_PIN = 1;   // IN4
const int RELAY3_PIN = 26;  // IN3

// Timing variables
unsigned long lastPotRead = 0;
unsigned long lastPrintTime = 0;
unsigned long lastLedToggle = 0;
unsigned long lastRelay4Toggle = 0;
unsigned long lastRelay3Toggle = 0;

// Intervals (in milliseconds)
const unsigned long POT_READ_INTERVAL = 100;    // 10 Hz = 100ms
const unsigned long PRINT_INTERVAL = 500;       // 2 Hz = 500ms
const unsigned long LED_BLINK_INTERVAL = 1000;  // 1 Hz = 1000ms
const unsigned long RELAY4_INTERVAL = 4000;     // Relay 4 = 4 sec
const unsigned long RELAY3_INTERVAL = 6000;     // Relay 3 = 6 sec

// Variables
int potValue = 0;
bool ledState = false;
bool relay4State = false;
bool relay3State = false;

void setup() {
    Serial.begin(9600);
    // while (!Serial) {
    //   ; // wait for serial port to connect. Needed for native USB
    // }

    pinMode(LED_PIN, OUTPUT);
    pinMode(POT_PIN, INPUT);

    pinMode(RELAY4_PIN, OUTPUT);
    pinMode(RELAY3_PIN, OUTPUT);

    digitalWrite(RELAY4_PIN, HIGH); // Relays off (active LOW)
    digitalWrite(RELAY3_PIN, HIGH);

    Serial.println("Teensy Potentiometer Reader Started");
    Serial.println("Reading pot at 10Hz, printing at 2Hz, LED blinking at 1Hz");
}

void loop() {
    unsigned long currentTime = millis();

    // Read potentiometer at 10 Hz
    if (currentTime - lastPotRead >= POT_READ_INTERVAL) {
        potValue = analogRead(POT_PIN);
        lastPotRead = currentTime;
    }

    // Print potentiometer value at 2 Hz
    if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
        Serial.print("Potentiometer value: ");
        Serial.println(potValue);
        lastPrintTime = currentTime;
    }

    // Blink LED at 1 Hz
    if (currentTime - lastLedToggle >= LED_BLINK_INTERVAL) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastLedToggle = currentTime;
    }

    // Toggle Relay 4 every 4 seconds
    if (currentTime - lastRelay4Toggle >= RELAY4_INTERVAL) {
        relay4State = !relay4State;
        digitalWrite(RELAY4_PIN, relay4State ? LOW : HIGH); // Active LOW
        Serial.print("Relay 4 toggled to: ");
        Serial.println(relay4State ? "ON" : "OFF");
        lastRelay4Toggle = currentTime;
    }

    // Toggle Relay 3 every 6 seconds
    if (currentTime - lastRelay3Toggle >= RELAY3_INTERVAL) {
        relay3State = !relay3State;
        digitalWrite(RELAY3_PIN, relay3State ? LOW : HIGH); // Active LOW
        Serial.print("Relay 3 toggled to: ");
        Serial.println(relay3State ? "ON" : "OFF");
        lastRelay3Toggle = currentTime;
    }
}
