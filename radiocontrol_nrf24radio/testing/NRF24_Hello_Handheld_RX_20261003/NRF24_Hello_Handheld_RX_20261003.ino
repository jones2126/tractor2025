// Reverse-direction minimal NRF24 test for the handheld Teensy 3.2.
// Receives "HELLO" and an incrementing counter from the tractor.

#include <SPI.h>
#include <RF24.h>
#include <string.h>

RF24 radio(7, 8);  // CE, CSN -- existing handheld wiring

const uint8_t TEST_ADDRESS[6] = "HELLO";

struct __attribute__((packed)) HelloPacket {
  uint32_t counter;
  char message[12];
};

static_assert(sizeof(HelloPacket) == 16, "HelloPacket must be 16 bytes");

HelloPacket packet = {};
unsigned long lastReceiveMillis = 0;
unsigned long lastWaitingPrintMillis = 0;
bool havePreviousCounter = false;
uint32_t previousCounter = 0;

void setup() {
  Serial.begin(115200);
  unsigned long waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) {
  }

  Serial.println("HANDHELD NRF24 REVERSE HELLO RECEIVER");

  SPI.begin();
  if (!radio.begin()) {
    Serial.println("FATAL: radio.begin() failed; NRF24 did not answer over SPI");
    while (true) {
      delay(1000);
    }
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);
  radio.setAutoAck(true);
  radio.setPayloadSize(sizeof(HelloPacket));
  radio.openReadingPipe(1, TEST_ADDRESS);
  radio.startListening();

  Serial.println("READY: address=HELLO channel=76 rate=250KBPS payload=16 PA=LOW");
  radio.printDetails();
  Serial.println("WAITING for tractor packets...");
}

void loop() {
  if (radio.available()) {
    radio.read(&packet, sizeof(packet));
    lastReceiveMillis = millis();

    Serial.print("RX counter=");
    Serial.print(packet.counter);
    Serial.print(" message=");
    Serial.print(packet.message);

    if (memcmp(packet.message, "HELLO", 5) != 0) {
      Serial.println(" status=BAD_MESSAGE");
    } else if (havePreviousCounter && packet.counter != previousCounter + 1) {
      Serial.print(" status=COUNTER_JUMP expected=");
      Serial.println(previousCounter + 1);
    } else {
      Serial.println(" status=OK");
    }

    previousCounter = packet.counter;
    havePreviousCounter = true;
  }

  unsigned long now = millis();
  if (now - lastWaitingPrintMillis >= 5000) {
    Serial.print("STATUS: last packet age ms=");
    if (lastReceiveMillis == 0) {
      Serial.println("never");
    } else {
      Serial.println(now - lastReceiveMillis);
    }
    lastWaitingPrintMillis = now;
  }
}
