#include <Arduino.h>

// Left encoder board diagnostic only. No I2C, steering or transmission control.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  if (Serial) Serial.println("# LEFT_ENCODER_BLINK_20261118 LED=ON");
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
