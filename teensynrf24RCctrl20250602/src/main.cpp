#include <SPI.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>

RF24 radio(7, 8);  // CE, CSN pins

#define LED_PIN 2
#define NUM_LEDS 4
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_RGB + NEO_KHZ800);

struct Color {
  uint8_t r, g, b;
  const char* name;
};

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
const uint8_t address[][6] = {"RCTRL", "TRACT"};
unsigned long lastTransmit = 0;
const unsigned long transmitInterval = 100;

const int leftPin = 3;
const int rightPin = 4;
const int STEERING_PIN = 15;    // STEERING_PIN is the top left pot
const int THROTTLE_PIN = 14;    // THROTTLE_PIN is top right pot
const int TRANSMISSION_PIN = 16;        // TRANSMISSION_PIN is the lower right pot
const int POT4_PIN = 17;        // POT4_PIN is the lower left pot
const int BUTTON01_PIN = 9;
const int ESTOP_PIN = 10;
const int BUTTON02_PIN = 6;

unsigned long currentMillis = 0;
unsigned long lastRateCalc = 0;
const unsigned long rateCalcInterval = 10000;
unsigned long lastLedUpdate = 0;
const unsigned long ledUpdateInterval = 500;
unsigned long lastModeCheck = 0;
const unsigned long modeCheckInterval = 100;
unsigned long ackCount = 0;
unsigned long shortTermAckCount = 0;
float currentRate = 0.0;

void readControlInputs();
void checkModeSW();
void sendData();
void updateLEDs();
void printACKRate();

void setup() {
    Serial.begin(115200);
    Serial.println("Transmitter Starting...");

    strip.begin();
    strip.setBrightness(64);
    strip.clear();
    strip.show();

    pinMode(leftPin, INPUT_PULLUP);
    pinMode(rightPin, INPUT_PULLUP);
    pinMode(STEERING_PIN, INPUT);
    pinMode(THROTTLE_PIN, INPUT);
    pinMode(TRANSMISSION_PIN, INPUT);
    pinMode(POT4_PIN, INPUT);
    pinMode(BUTTON01_PIN, INPUT_PULLUP);
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    pinMode(BUTTON02_PIN, INPUT_PULLUP);

    SPI.begin();
    delay(100);

    bool initialized = false;
    for(int i = 0; i < 5; i++) {
        if (radio.begin()) {
            initialized = true;
            break;
        }
        delay(1000);
    }

    if (!initialized) {
        while (1) {
            strip.setPixelColor(0, colors[0].r, colors[0].g, colors[0].b);
            strip.show();
            delay(500);
            strip.setPixelColor(0, 0, 0, 0);
            strip.show();
            delay(500);
        }
    }

    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(124);
    radio.enableAckPayload();
    // radio.enableDynamicPayloads(); 
    radio.openWritingPipe(address[1]);
    radio.openReadingPipe(1, address[0]);
    radio.stopListening();
    radio.printDetails();

    lastRateCalc = millis();
    lastLedUpdate = millis();
    lastModeCheck = millis();

    strip.setPixelColor(0, colors[1].r, colors[1].g, colors[1].b);
    strip.show();
    delay(1000);
}

void sendData() {
    if (currentMillis - lastTransmit >= transmitInterval) {
        readControlInputs();
        bool report = radio.write(&radioData, sizeof(RadioControlStruct));
        if (report && radio.isAckPayloadAvailable()) {
            radio.read(&ackPayload, sizeof(AckPayloadStruct));
            ackCount++;
            shortTermAckCount++;
        }
        lastTransmit = currentMillis;
    }
}

void printACKRate() {
    if (currentMillis - lastRateCalc >= rateCalcInterval) {
        float timeElapsed = (currentMillis - lastRateCalc) / 1000.0;
        float rate = ackCount / timeElapsed;
        Serial.print("ACK Rate: ");
        Serial.print(rate);
        Serial.println(" Hz");
        ackCount = 0;
        lastRateCalc = currentMillis;
        Serial.print("RadioControlStruct size: ");
        Serial.println(sizeof(RadioControlStruct));
    }
}

void readControlInputs() {
    radioData.steering_val = analogRead(STEERING_PIN); // STEERING_PIN is the top left pot
    radioData.throttle_val = analogRead(THROTTLE_PIN);  // THROTTLE_PIN is top right pot
    radioData.transmission_val = analogRead(TRANSMISSION_PIN); // TRANSMISSION_PIN is the lower right pot - change this to TRANSMISSION_PIN
    radioData.pot4_val = analogRead(POT4_PIN); // POT4_PIN is the lower left pot
    radioData.voltage = analogRead(A2) * (5.0 / 1023.0) * 3.0;
    radioData.estop = !digitalRead(ESTOP_PIN);
    radioData.button01 = !digitalRead(BUTTON01_PIN);
    radioData.button02 = !digitalRead(BUTTON02_PIN);
}

void checkModeSW() {
    if (currentMillis - lastModeCheck >= modeCheckInterval) {
        int leftState = digitalRead(leftPin);
        int rightState = digitalRead(rightPin);
        if (leftState == LOW) {
            radioData.control_mode = 0;
        } else if (rightState == LOW) {
            radioData.control_mode = 1;
        } else {
            radioData.control_mode = 2;
        }
        lastModeCheck = currentMillis;
    }
}

void updateLEDs() {
    if (currentMillis - lastLedUpdate >= ledUpdateInterval) {
        float timeElapsed = (currentMillis - lastLedUpdate) / 1000.0;
        currentRate = shortTermAckCount / timeElapsed;
        strip.clear();
        if (currentRate < 2.0) {
            strip.setPixelColor(0, colors[0].r, colors[0].g, colors[0].b);
        } else if (currentRate <= 5.0) {
            strip.setPixelColor(0, colors[6].r, colors[6].g, colors[6].b);
        } else {
            strip.setPixelColor(0, colors[1].r, colors[1].g, colors[1].b);
        }

        if (radioData.estop)
            strip.setPixelColor(1, colors[0].r, colors[0].g, colors[0].b);
        else
            strip.setPixelColor(1, 0, 0, 0);

        if (radioData.control_mode == 0)
            strip.setPixelColor(2, colors[4].r, colors[4].g, colors[4].b);
        else if (radioData.control_mode == 1)
            strip.setPixelColor(2, colors[2].r, colors[2].g, colors[2].b);
        else
            strip.setPixelColor(2, colors[3].r, colors[3].g, colors[3].b);

        if (radioData.voltage > 11.5)
            strip.setPixelColor(3, colors[1].r, colors[1].g, colors[1].b);
        else if (radioData.voltage > 10.5)
            strip.setPixelColor(3, colors[6].r, colors[6].g, colors[6].b);
        else
            strip.setPixelColor(3, colors[0].r, colors[0].g, colors[0].b);

        strip.show();
        shortTermAckCount = 0;
        lastLedUpdate = currentMillis;
    }
}

void loop() {
    currentMillis = millis();
    checkModeSW();
    sendData();
    updateLEDs();
    printACKRate();
}
