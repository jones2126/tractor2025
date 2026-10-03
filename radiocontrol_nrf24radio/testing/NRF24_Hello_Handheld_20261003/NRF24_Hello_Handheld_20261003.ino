// Minimal NRF24 transmitter test for the handheld Teensy 3.2.
// Sends "HELLO" and an incrementing counter once per second.

#include <SPI.h>
#include <RF24.h>

RF24 radio(7, 8);  // CE, CSN -- existing handheld wiring

const uint8_t TEST_ADDRESS[6] = "HELLO";

struct __attribute__((packed)) HelloPacket {
  uint32_t counter;
  char message[12];
};

static_assert(sizeof(HelloPacket) == 16, "HelloPacket must be 16 bytes");

HelloPacket packet = {0, "HELLO"};

void setup() {
  Serial.begin(115200);
  unsigned long waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) {
    // Allow the USB serial monitor a short time to connect.
  }

  Serial.println("HANDHELD NRF24 HELLO TEST");

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
  radio.setRetries(5, 15);
  radio.setPayloadSize(sizeof(HelloPacket));
  radio.openWritingPipe(TEST_ADDRESS);
  radio.stopListening();

  Serial.println("READY: address=HELLO channel=76 rate=250KBPS payload=16 PA=LOW");
}

void loop() {
  bool acknowledged = radio.write(&packet, sizeof(packet));

  Serial.print("TX counter=");
  Serial.print(packet.counter);
  Serial.print(" message=");
  Serial.print(packet.message);
  Serial.print(" result=");
  Serial.println(acknowledged ? "ACK" : "NO_ACK");

  packet.counter++;
  delay(1000);
}
